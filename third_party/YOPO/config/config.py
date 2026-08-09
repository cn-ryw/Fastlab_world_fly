import os
from ruamel.yaml import YAML


# Global Configuration Management
class Config:
    def __init__(self):
        base_dir = os.path.dirname(os.path.abspath(__file__))
        yaml = YAML()
        base_path = os.path.join(base_dir, "traj_opt.yaml")
        with open(base_path, 'r') as stream:
            self._data = yaml.load(stream)

        overlay_name = os.environ.get("YOPO_CONFIG", "").strip()
        self.config_path = base_path
        if overlay_name:
            overlay_path = overlay_name
            if not os.path.isabs(overlay_path):
                overlay_path = os.path.join(base_dir, overlay_path)
            if not os.path.isfile(overlay_path):
                raise FileNotFoundError(f"YOPO_CONFIG does not exist: {overlay_path}")
            with open(overlay_path, 'r') as stream:
                overlay = yaml.load(stream) or {}
            self._data.update(overlay)
            self.config_path = overlay_path

        self._data["train"] = True
        self._data["goal_length"] = 2.0 * self._data['radio_range']
        self._data["sgm_time"] = 2 * self._data["radio_range"] / self._data["vel_max_train"]
        self._data["traj_num"] = self._data['horizon_num'] * self._data['vertical_num'] * self._data["radio_num"]
        self._validate()
        print(f"YOPO config: {self.config_path}")

    def _validate(self):
        if self._data["image_width"] != self._data["horizon_num"] * 32:
            raise ValueError("image_width must equal horizon_num * 32")
        if self._data["image_height"] != self._data["vertical_num"] * 32:
            raise ValueError("image_height must equal vertical_num * 32")
        for key in (
            "velocity",
            "vel_max_train",
            "acc_max_train",
            "radio_range",
            "arrival_distance_m",
        ):
            if self._data[key] <= 0:
                raise ValueError(f"{key} must be positive")
        if "depth_max_m" in self._data and self._data["depth_max_m"] <= 0:
            raise ValueError("depth_max_m must be positive")
        if "depth_min_m" in self._data and not (
            0 <= self._data["depth_min_m"] < self._data["depth_max_m"]
        ):
            raise ValueError("depth_min_m must be in [0, depth_max_m)")
        if "depth_mask_threshold" in self._data:
            threshold = self._data["depth_mask_threshold"]
            if not isinstance(threshold, int) or not 0 <= threshold <= 254:
                raise ValueError(
                    "depth_mask_threshold must be an integer in [0, 254]")
        safety_eval_points = self._data.get("safety_loss_eval_points", 30)
        if (
            not isinstance(safety_eval_points, int)
            or safety_eval_points < 2
        ):
            raise ValueError(
                "safety_loss_eval_points must be an integer of at least 2"
            )
        safety_far_weight = self._data.get("safety_far_weight", 0.0)
        safety_far_r = self._data.get("safety_far_r", self._data["r"])
        if safety_far_weight < 0:
            raise ValueError("safety_far_weight must be non-negative")
        if safety_far_r <= 0:
            raise ValueError("safety_far_r must be positive")
        if "validation_map_index" in self._data and self._data["validation_map_index"] < 0:
            raise ValueError("validation_map_index must be non-negative")
        radius = self._data.get("vehicle_radius_m")
        margin = self._data.get("safety_margin_m")
        if radius is not None or margin is not None:
            if radius is None or margin is None:
                raise ValueError(
                    "vehicle_radius_m and safety_margin_m must be set together"
                )
            if radius <= 0 or margin < 0:
                raise ValueError(
                    "vehicle_radius_m must be positive and safety_margin_m non-negative"
                )
            if abs(self._data["d0"] - (radius + margin)) > 1e-6:
                raise ValueError(
                    "d0 must equal vehicle_radius_m + safety_margin_m"
                )
        if "controller_accel_hard_cap_mps2" in self._data:
            hard_cap = self._data["controller_accel_hard_cap_mps2"]
            if hard_cap < self._data["acc_max_train"]:
                raise ValueError(
                    "controller_accel_hard_cap_mps2 must be at least acc_max_train"
                )
        if "mask_augment_enabled" in self._data:
            self._validate_mask_augmentation()
        if "acceleration_mixture" in self._data:
            self._validate_acceleration_mixture()

    def _validate_acceleration_mixture(self):
        mixture = self._data["acceleration_mixture"]
        if not isinstance(mixture, list) or not mixture:
            raise ValueError("acceleration_mixture must be a non-empty list")
        total_probability = 0.0
        previous_maximum = 0.0
        for index, item in enumerate(mixture):
            if not isinstance(item, list) or len(item) != 3:
                raise ValueError(
                    "each acceleration_mixture item must be [min, max, probability]"
                )
            minimum, maximum, probability = map(float, item)
            if minimum < 0.0 or maximum <= minimum or probability <= 0.0:
                raise ValueError(
                    "acceleration_mixture bounds must increase and probability "
                    "must be positive"
                )
            if index and abs(minimum - previous_maximum) > 1e-6:
                raise ValueError("acceleration_mixture intervals must be contiguous")
            previous_maximum = maximum
            total_probability += probability
        if abs(total_probability - 1.0) > 1e-6:
            raise ValueError("acceleration_mixture probabilities must sum to 1")
        if previous_maximum > 1.2 * self._data["acc_max_train"] + 1e-6:
            raise ValueError(
                "acceleration_mixture maximum must not exceed 1.2 * acc_max_train"
            )

    def _validate_mask_augmentation(self):
        def probability(key):
            value = self._data[key]
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"{key} must be in [0, 1]")

        def bounds(key, lower=0.0, upper=1.0):
            values = self._data[key]
            if len(values) != 2 or not lower <= values[0] <= values[1] <= upper:
                raise ValueError(
                    f"{key} must contain ordered bounds in [{lower}, {upper}]"
                )

        probability("mask_augment_probability")
        probability("mask_band_probability")
        bounds("mask_hole_width_fraction")
        bounds("mask_hole_height_fraction")
        bounds("mask_dropout_ratio")
        bounds("mask_hole_count", lower=1, upper=100)
        if not all(isinstance(value, int) for value in self._data["mask_hole_count"]):
            raise ValueError("mask_hole_count bounds must be integers")
        probability("mask_min_valid_ratio")
        if self._data["mask_fill_mode"] != "valid_mean":
            raise ValueError("mask_fill_mode must be 'valid_mean'")

    def __getitem__(self, key):
        return self._data[key]

    def __setitem__(self, key, value):
        self._data[key] = value


cfg = Config()
