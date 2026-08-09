import math
import torch
import numpy as np
from scipy.spatial.transform import Rotation as R
from config.config import cfg


class LatticeParam:
    def __init__(self):
        ratio = 1.0 if cfg["train"] else cfg["velocity"] / cfg["vel_max_train"]
        self.vel_max = ratio * cfg["vel_max_train"]
        self.acc_max = ratio * ratio * cfg["acc_max_train"]
        self.segment_time = cfg["sgm_time"] / ratio
        self.horizon_num = cfg["horizon_num"]
        self.vertical_num = cfg["vertical_num"]
        self.radio_num = cfg["radio_num"]
        self.traj_num = cfg["traj_num"]
        self.horizon_fov = cfg["horizon_camera_fov"]
        self.vertical_fov = cfg["vertical_camera_fov"]
        self.horizon_anchor_fov = cfg["horizon_anchor_fov"]
        self.vertical_anchor_fov = cfg["vertical_anchor_fov"]
        self.radio_range = cfg["radio_range"]

        print("---------- Param --------")
        print(f"| {'max speed':<12} = {round(self.vel_max, 1):>6} |")
        print(f"| {'max accel':<12} = {round(self.acc_max, 1):>6} |")
        print(f"| {'traj time':<12} = {round(self.segment_time, 1):>6} |")
        print(f"| {'max radio':<12} = {round(2 * self.radio_range, 1):>6} |")
        print("-------------------------")


def _wrap(a):
    """wrap angle to (-pi, pi]"""
    return math.atan2(math.sin(a), math.cos(a))


def _pixel_dir_multicam_pinhole(u, v, sub_width, cam_yaw_deg, fx, fy, cx, cy):
    """Body-frame ray direction (dx,dy,dz) for image pixel (u,v) of the 4-camera pinhole rig.
    Must mirror the simulator kernel (sensor_simulator.cu, multicam branch)."""
    cam = int(u // sub_width)
    if cam >= len(cam_yaw_deg):
        cam = len(cam_yaw_deg) - 1
    lu = u - cam * sub_width
    y = -(lu - cx) / fx
    z = -(v - cy) / fy
    x = 1.0
    n = math.sqrt(x * x + y * y + z * z)
    x, y, z = x / n, y / n, z / n
    yaw = math.radians(cam_yaw_deg[cam])
    cy_, sy_ = math.cos(yaw), math.sin(yaw)
    dx = cy_ * x - sy_ * y
    dy = sy_ * x + cy_ * y
    dz = z
    return dx, dy, dz


class LatticePrimitive(LatticeParam):
    """
    Grid index layout in image (row-major, bottom-left origin):
                       +---+---+---+
                       | 8 | 7 | 6 |
                       +---+---+---+
                       | 5 | 4 | 3 |
                       +---+---+---+
                       | 2 | 1 | 0 |
                       +---+---+---+

    Anchors are placed at the center-ray direction of each backbone feature cell. The feature
    map is image / downsample(=32). The CNN output cell (i, j) corresponds (after the flip in
    state_transform) to lattice build cell (V-1-i, H-1-j), i.e. lattice build cell (i_b, j_b) is
    read by CNN cell (V-1-i_b, H-1-j_b), which views image pixel
        u_c = ((H-1-j_b) + 0.5) * DS,  v_c = ((V-1-i_b) + 0.5) * DS.
    For camera_model == "multicam_pinhole" the direction is the pinhole ray at (u_c, v_c);
    for "erp" the legacy uniform-angle spacing is used (reproduces the original behaviour).
    """
    _instance = None

    def __init__(self):
        super().__init__()
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        H, V = self.horizon_num, self.vertical_num
        radio_diff = self.radio_range / self.radio_num
        camera_model = cfg["camera_model"] if "camera_model" in cfg._data else "erp"

        # ---- per-cell anchor (yaw=alpha, pitch=beta) on the build grid [V, H] ----
        alpha_grid = np.zeros((V, H), dtype=np.float64)
        beta_grid = np.zeros((V, H), dtype=np.float64)

        if camera_model == "multicam_pinhole":
            W = int(cfg["image_width"])
            Hpx = int(cfg["image_height"])
            DS_w = W / H
            DS_h = Hpx / V
            sub_width = cfg["cam_sub_width"]
            cam_yaw_deg = list(cfg["cam_yaw_deg"])
            fx, fy = cfg["cam_fx"], cfg["cam_fy"]
            cx, cy = cfg["cam_cx"], cfg["cam_cy"]
            for i_b in range(V):
                for j_b in range(H):
                    u_c = ((H - 1 - j_b) + 0.5) * DS_w
                    v_c = ((V - 1 - i_b) + 0.5) * DS_h
                    dx, dy, dz = _pixel_dir_multicam_pinhole(u_c, v_c, sub_width, cam_yaw_deg, fx, fy, cx, cy)
                    alpha_grid[i_b, j_b] = math.atan2(dy, dx)
                    beta_grid[i_b, j_b] = math.atan2(dz, math.hypot(dx, dy))
        else:
            # legacy ERP: uniform angular spacing
            direction_diff = 0.0 if H == 1 else (self.horizon_fov / 180.0 * math.pi) / H
            altitude_diff = 0.0 if V == 1 else (self.vertical_fov / 180.0 * math.pi) / V
            for i_b in range(V):
                for j_b in range(H):
                    alpha_grid[i_b, j_b] = -direction_diff * (H - 1) / 2 + j_b * direction_diff
                    beta_grid[i_b, j_b] = -altitude_diff * (V - 1) / 2 + i_b * altitude_diff

        # ---- per-anchor half prediction range from neighbour spacing ----
        # horizontal: circular (panorama wraps 360°); vertical: one-sided at edges.
        yaw_diff_grid = np.zeros((V, H), dtype=np.float64)
        pitch_diff_grid = np.zeros((V, H), dtype=np.float64)
        for i_b in range(V):
            for j_b in range(H):
                # horizontal neighbours (wrap-around)
                jl, jr = (j_b - 1) % H, (j_b + 1) % H
                dyl = abs(_wrap(alpha_grid[i_b, j_b] - alpha_grid[i_b, jl]))
                dyr = abs(_wrap(alpha_grid[i_b, j_b] - alpha_grid[i_b, jr]))
                yaw_diff_grid[i_b, j_b] = 0.5 * max(dyl, dyr)
                # vertical neighbours (no wrap)
                vd = []
                if i_b > 0:
                    vd.append(abs(beta_grid[i_b, j_b] - beta_grid[i_b - 1, j_b]))
                if i_b < V - 1:
                    vd.append(abs(beta_grid[i_b, j_b] - beta_grid[i_b + 1, j_b]))
                pitch_diff_grid[i_b, j_b] = 0.5 * (max(vd) if vd else 0.5 * math.pi)

        # ---- assemble lattice in build order (radio, i, j): Bottom→Top, Right→Left ----
        lattice_pos_list, lattice_angle_list, lattice_Rbp_list = [], [], []
        yaw_diff_list, pitch_diff_list = [], []
        for h in range(0, self.radio_num):
            for i in range(0, V):
                for j in range(0, H):
                    search_radio = (h + 1) * radio_diff
                    alpha = float(alpha_grid[i, j])
                    beta = float(beta_grid[i, j])
                    pos_node = torch.tensor([math.cos(beta) * math.cos(alpha) * search_radio,
                                             math.cos(beta) * math.sin(alpha) * search_radio,
                                             math.sin(beta) * search_radio])
                    lattice_pos_list.append(pos_node)
                    lattice_angle_list.append(torch.tensor([alpha, beta]))
                    Rotation = R.from_euler('ZYX', [alpha, -beta, 0.0], degrees=False)  # yaw-pitch-roll
                    lattice_Rbp_list.append(torch.tensor(Rotation.as_matrix()))
                    yaw_diff_list.append(float(yaw_diff_grid[i, j]))
                    pitch_diff_list.append(float(pitch_diff_grid[i, j]))

        self.lattice_pos_node = torch.stack(lattice_pos_list).to(dtype=torch.float32, device=device)  # [N, 3]
        self.lattice_angle_node = torch.stack(lattice_angle_list).to(dtype=torch.float32, device=device)  # [N, 2]
        self.lattice_Rbp_node = torch.stack(lattice_Rbp_list).to(dtype=torch.float32, device=device)  # [N, 3, 3]

        # per-anchor prediction half-range [N] (was scalar in the ERP-uniform version)
        self.yaw_diff = torch.tensor(yaw_diff_list, dtype=torch.float32, device=device)
        self.pitch_diff = torch.tensor(pitch_diff_list, dtype=torch.float32, device=device)

    def getStateLattice(self, id=None):
        if id is not None:
            return self.lattice_pos_node[id, :]
        else:
            return self.lattice_pos_node

    def getAngleLattice(self, id=None):
        if id is not None:
            return self.lattice_angle_node[id, 0], self.lattice_angle_node[id, 1]  # yaw, pitch
        else:
            return self.lattice_angle_node[:, 0], self.lattice_angle_node[:, 1]  # yaw, pitch

    def getRotation(self, id=None):
        if id is not None:
            return self.lattice_Rbp_node[id]
        else:
            return self.lattice_Rbp_node

    def convert_ImageGrid_LatticeID(self, id):
        return self.traj_num - id - 1

    @classmethod
    def get_instance(self):
        if self._instance is None: self._instance = self()
        return self._instance
