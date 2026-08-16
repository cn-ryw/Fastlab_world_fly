#!/usr/bin/env python3
"""Render three DA360/Cesium figures from the recorded 12-capture evidence.

The script deliberately accepts only the capture IDs frozen in the historical
fit report.  It reads either the packaged evidence archive or the expanded
four-file capture bundles, validates their structural/semantic contracts, and
recomputes the leave-one-location-out (LOLO) scale-only results before drawing.

No synthetic observations, interpolation of Cesium truth, confidence
intervals, or dense ground-truth depth maps are created.

Examples
--------
Use the packaged evidence when it is available (the default)::

    python docs/figures/da360-real-data/plot_real_data_figures.py

Use the expanded capture bundles explicitly::

    python docs/figures/da360-real-data/plot_real_data_figures.py \
        --input-mode directory \
        --bundle-dir <capture-bundle-directory> \
        --fit-report ../experiment_data/metric_fit-lolo-20260810-12capture/fit_report.json
"""

from __future__ import annotations

import argparse
import csv
import io
import json
import math
import tarfile
from contextlib import AbstractContextManager
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import matplotlib as mpl

mpl.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm, Normalize
from matplotlib.lines import Line2D
from matplotlib.ticker import FuncFormatter, LogLocator, NullFormatter
import numpy as np
from PIL import Image, ImageOps
from scipy.optimize import least_squares


# Editable type in SVG/PDF is part of the export contract.
plt.rcParams["font.family"] = "sans-serif"
plt.rcParams["font.sans-serif"] = ["Arial", "DejaVu Sans", "Liberation Sans"]
plt.rcParams["svg.fonttype"] = "none"
plt.rcParams["pdf.fonttype"] = 42


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
PROJECT_ROOT = REPO.parent
DEFAULT_ARCHIVE = PROJECT_ROOT / "backups" / "da360-calibration-evidence-20260811.tar.gz"
DEFAULT_BUNDLE_DIR: Path | None = None
DEFAULT_FIT_REPORT = (
    PROJECT_ROOT
    / "experiment_data"
    / "metric_fit-lolo-20260810-12capture"
    / "fit_report.json"
)

CAPTURE_IDS = (
    "site-a-try-03",
    "site-a-try-04",
    "site-a-try-05",
    "site-b-try-01",
    "site-b-try-02",
    "site-b-try-03",
    "site-c-try-02",
    "site-c-try-03",
    "site-c-try-04",
    "site-d-try-02",
    "site-d-try-04",
    "site-d-try-05",
)
SITES = ("site-a", "site-b", "site-c", "site-d")
SITE_COLORS = {
    "site-a": "#4C78A8",
    "site-b": "#F28E2B",
    "site-c": "#8E6CBB",
    "site-d": "#2A9D8F",
}
STATUS_COLORS = {
    "hit": "#2A9D8F",
    "no_hit": "#E9A23B",
    "pole_excluded": "#A7ADB4",
}

REPRESENTATIVE_CAPTURE = "site-a-try-03"
METRIC_MIN_DEPTH_M = 0.5
METRIC_MAX_DEPTH_M = 20.0
MEDIAN_ABSREL_GATE = 0.15
P90_ABSREL_GATE = 0.30
HUBER_K = 1.345
MIN_HUBER_SCALE = 1e-6
DEFAULT_PNG_DPI = 300
DEFAULT_TIFF_DPI = 600


@dataclass
class Capture:
    capture_id: str
    site: str
    rgb: np.ndarray
    pred_disp: np.ndarray
    valid_mask: np.ndarray
    manifest: dict[str, Any]
    raw_metadata: dict[str, Any]
    anchor_metadata: dict[str, Any]
    records: list[dict[str, Any]]
    grid_rows: int
    grid_cols: int

    @property
    def n_candidates(self) -> int:
        return self.grid_rows * self.grid_cols

    @property
    def n_hits(self) -> int:
        return sum(record["status"] == "hit" for record in self.records)

    @property
    def n_valid_hits(self) -> int:
        return sum(
            record["status"] == "hit" and record["pred_disp"] is not None
            for record in self.records
        )


@dataclass
class Analysis:
    captures: list[Capture]
    range_records: list[dict[str, Any]]
    capture_rows: list[dict[str, Any]]
    fold_scales: dict[str, float]
    pooled_scale: float
    pooled_metrics: dict[str, float]
    fit_report: dict[str, Any]
    validation_deltas: dict[str, float]


class EvidenceReader(AbstractContextManager):
    """Read named evidence members without extracting an archive to disk."""

    def __init__(
        self,
        *,
        archive: Path | None,
        bundle_dir: Path | None,
        fit_report: Path | None,
    ) -> None:
        self.archive_path = archive
        self.bundle_dir = bundle_dir
        self.fit_report_path = fit_report
        self._archive: tarfile.TarFile | None = None
        self._members_by_basename: dict[str, list[tarfile.TarInfo]] = {}

        if archive is not None:
            if not archive.is_file():
                raise FileNotFoundError(f"evidence archive not found: {archive}")
            self._archive = tarfile.open(archive, mode="r:gz")
            for member in self._archive.getmembers():
                if member.isfile():
                    self._members_by_basename.setdefault(Path(member.name).name, []).append(member)
        else:
            if bundle_dir is None or not bundle_dir.is_dir():
                raise FileNotFoundError(f"bundle directory not found: {bundle_dir}")
            if fit_report is None or not fit_report.is_file():
                raise FileNotFoundError(f"fit report not found: {fit_report}")

    @property
    def mode(self) -> str:
        return "archive" if self._archive is not None else "directory"

    @property
    def source_label(self) -> str:
        if self.archive_path is not None:
            return "packaged evidence archive"
        return "expanded capture bundles"

    def _archive_bytes(self, basename: str) -> bytes:
        assert self._archive is not None
        matches = self._members_by_basename.get(basename, [])
        if len(matches) != 1:
            raise ValueError(
                f"archive must contain exactly one {basename!r}; found {len(matches)}"
            )
        handle = self._archive.extractfile(matches[0])
        if handle is None:
            raise ValueError(f"unable to read archive member {matches[0].name!r}")
        payload = handle.read()
        if not payload:
            raise ValueError(f"archive member is empty: {matches[0].name!r}")
        return payload

    def read_bundle_file(self, basename: str) -> bytes:
        if self._archive is not None:
            return self._archive_bytes(basename)
        assert self.bundle_dir is not None
        path = self.bundle_dir / basename
        if not path.is_file() or path.stat().st_size == 0:
            raise FileNotFoundError(f"capture artifact missing or empty: {path}")
        return path.read_bytes()

    def read_fit_report(self) -> bytes:
        if self._archive is not None:
            return self._archive_bytes("fit_report.json")
        assert self.fit_report_path is not None
        return self.fit_report_path.read_bytes()

    def __exit__(self, exc_type, exc_value, traceback):
        if self._archive is not None:
            self._archive.close()
        return False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Render three publication figures from the recorded DA360/Cesium bundles."
    )
    parser.add_argument(
        "--input-mode",
        choices=("auto", "archive", "directory"),
        default="auto",
        help="auto uses the packaged archive when present, otherwise the expanded directory",
    )
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--bundle-dir", type=Path, default=DEFAULT_BUNDLE_DIR)
    parser.add_argument("--fit-report", type=Path, default=DEFAULT_FIT_REPORT)
    parser.add_argument("--output-dir", type=Path, default=HERE)
    parser.add_argument("--png-dpi", type=int, default=DEFAULT_PNG_DPI)
    parser.add_argument("--tiff-dpi", type=int, default=DEFAULT_TIFF_DPI)
    return parser.parse_args()


def choose_reader(args: argparse.Namespace) -> EvidenceReader:
    use_archive = args.input_mode == "archive" or (
        args.input_mode == "auto" and args.archive.is_file()
    )
    if args.input_mode == "archive" and not args.archive.is_file():
        raise FileNotFoundError(f"requested evidence archive does not exist: {args.archive}")
    if use_archive:
        return EvidenceReader(archive=args.archive, bundle_dir=None, fit_report=None)
    if args.bundle_dir is None:
        raise ValueError("--bundle-dir is required when --input-mode=directory")
    return EvidenceReader(
        archive=None,
        bundle_dir=args.bundle_dir,
        fit_report=args.fit_report,
    )


def require_mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be an object")
    return value


def require_list(value: Any, label: str) -> list[Any]:
    if not isinstance(value, list):
        raise ValueError(f"{label} must be an array")
    return value


def require_string(mapping: dict[str, Any], key: str, label: str) -> str:
    value = mapping.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{label}.{key} must be a non-empty string")
    return value


def require_number(
    mapping: dict[str, Any], key: str, label: str, *, positive: bool = False
) -> float:
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label}.{key} must be numeric")
    result = float(value)
    if not math.isfinite(result) or (positive and result <= 0):
        qualifier = "positive and finite" if positive else "finite"
        raise ValueError(f"{label}.{key} must be {qualifier}")
    return result


def require_int(
    mapping: dict[str, Any], key: str, label: str, *, positive: bool = False
) -> int:
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{label}.{key} must be an integer")
    if positive and value <= 0:
        raise ValueError(f"{label}.{key} must be positive")
    return value


def json_object(payload: bytes, label: str) -> dict[str, Any]:
    try:
        parsed = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError(f"invalid {label}: {error}") from error
    return require_mapping(parsed, label)


def map_pixel_center(coordinate: float, source_size: int, target_size: int) -> float:
    return (float(coordinate) + 0.5) * target_size / source_size - 0.5


def sample_wrapped_bilinear(
    array: np.ndarray, valid_mask: np.ndarray, u: float, v: float
) -> float | None:
    """Sample an ERP array with horizontal wrap and vertical edge clamp."""

    height, width = array.shape
    if not np.isfinite(u) or not np.isfinite(v) or v < -0.5 or v > height - 0.5:
        return None
    wrapped_u = float(u) % width
    clamped_v = min(max(float(v), 0.0), height - 1.0)
    x0_raw = int(np.floor(wrapped_u))
    y0 = int(np.floor(clamped_v))
    x1_raw = x0_raw + 1
    y1 = min(y0 + 1, height - 1)
    fx = wrapped_u - x0_raw
    fy = clamped_v - y0
    weighted = (
        (x0_raw % width, y0, (1.0 - fx) * (1.0 - fy)),
        (x1_raw % width, y0, fx * (1.0 - fy)),
        (x0_raw % width, y1, (1.0 - fx) * fy),
        (x1_raw % width, y1, fx * fy),
    )
    value = 0.0
    for x_index, y_index, weight in weighted:
        if weight <= 1e-15:
            continue
        if not valid_mask[y_index, x_index] or not np.isfinite(array[y_index, x_index]):
            return None
        value += float(array[y_index, x_index]) * weight
    return value


def training_huber_scale(residuals: np.ndarray) -> float:
    residuals = np.asarray(residuals, dtype=np.float64)
    residuals = residuals[np.isfinite(residuals)]
    if residuals.size == 0:
        raise ValueError("cannot derive a Huber scale from empty residuals")
    median = float(np.median(residuals))
    mad = float(np.median(np.abs(residuals - median)))
    return max(MIN_HUBER_SCALE, HUBER_K * 1.4826 * mad)


def fit_scale_only(x: np.ndarray, inverse_depth: np.ndarray) -> float:
    """Reproduce the historical Huber scale-only fit y = a*x."""

    x = np.asarray(x, dtype=np.float64).reshape(-1)
    inverse_depth = np.asarray(inverse_depth, dtype=np.float64).reshape(-1)
    if x.size != inverse_depth.size or x.size == 0:
        raise ValueError("scale fitting requires matching non-empty arrays")
    if not np.all(np.isfinite(x)) or not np.all(np.isfinite(inverse_depth)):
        raise ValueError("scale fitting arrays must be finite")
    nonzero = np.abs(x) > 1e-12
    if not np.any(nonzero):
        raise ValueError("scale fitting requires non-zero disparities")
    initial = max(1e-9, float(np.median(inverse_depth[nonzero] / x[nonzero])))
    huber_scale = training_huber_scale(inverse_depth - initial * x)
    result = least_squares(
        lambda parameter: inverse_depth - parameter[0] * x,
        [initial],
        bounds=([1e-12], [np.inf]),
        max_nfev=2000,
        loss="huber",
        f_scale=huber_scale,
    )
    if not result.success or not np.isfinite(result.x[0]) or result.x[0] <= 0:
        raise ValueError("Huber scale-only fit did not converge to a positive scale")
    return float(result.x[0])


def compute_depth_metrics(true_depth: np.ndarray, predicted_depth: np.ndarray) -> dict[str, float]:
    true_depth = np.asarray(true_depth, dtype=np.float64)
    predicted_depth = np.asarray(predicted_depth, dtype=np.float64)
    valid = (
        np.isfinite(true_depth)
        & np.isfinite(predicted_depth)
        & (true_depth > 0)
        & (predicted_depth > 0)
    )
    if np.count_nonzero(valid) < 2:
        raise ValueError("at least two finite positive depths are required")
    truth = true_depth[valid]
    predicted = predicted_depth[valid]
    abs_error = np.abs(predicted - truth)
    abs_rel = abs_error / truth
    return {
        "median_abs_rel": float(np.median(abs_rel)),
        "p90_abs_rel": float(np.percentile(abs_rel, 90)),
        "rmse_m": float(np.sqrt(np.mean((predicted - truth) ** 2))),
        "median_error_m": float(np.median(abs_error)),
        "p90_error_m": float(np.percentile(abs_error, 90)),
        "n_valid": int(np.count_nonzero(valid)),
    }


def expected_location(capture_id: str) -> str:
    return "-".join(capture_id.split("-")[:2])


def short_capture_label(capture_id: str) -> str:
    pieces = capture_id.split("-")
    return f"{pieces[1].upper()}{pieces[-1]}"


def decode_rgb(payload: bytes, capture_id: str) -> np.ndarray:
    try:
        with Image.open(io.BytesIO(payload)) as encoded:
            rgb = ImageOps.exif_transpose(encoded).convert("RGB")
            rgb.load()
    except (OSError, ValueError) as error:
        raise ValueError(f"invalid RGB for {capture_id}: {error}") from error
    return np.asarray(rgb, dtype=np.uint8)


def load_capture(reader: EvidenceReader, capture_id: str) -> Capture:
    rgb_name = f"{capture_id}-rgb.jpg"
    raw_name = f"{capture_id}-raw.npz"
    anchors_name = f"{capture_id}-anchors.json"
    manifest_name = f"{capture_id}-manifest.json"

    rgb_bytes = reader.read_bundle_file(rgb_name)
    raw_bytes = reader.read_bundle_file(raw_name)
    anchor_bytes = reader.read_bundle_file(anchors_name)
    manifest_bytes = reader.read_bundle_file(manifest_name)
    manifest = json_object(manifest_bytes, f"{capture_id} manifest")
    if manifest.get("schemaVersion") != 2:
        raise ValueError(f"{capture_id}: manifest schemaVersion must be 2")

    identity = {
        key: require_string(manifest, key, f"{capture_id} manifest")
        for key in ("sessionId", "captureId", "locationId", "frameId")
    }
    if identity["captureId"] != capture_id:
        raise ValueError(f"{capture_id}: manifest captureId mismatch")
    if identity["locationId"] != expected_location(capture_id):
        raise ValueError(f"{capture_id}: manifest locationId mismatch")

    manifest_files = require_mapping(manifest.get("files"), f"{capture_id} manifest.files")
    for key, expected_name in (
        ("rgb", rgb_name),
        ("raw", raw_name),
        ("anchors", anchors_name),
    ):
        entry = require_mapping(manifest_files.get(key), f"{capture_id} manifest.files.{key}")
        if require_string(entry, "name", f"{capture_id} manifest.files.{key}") != expected_name:
            raise ValueError(f"{capture_id}: manifest {key} filename mismatch")

    rgb = decode_rgb(rgb_bytes, capture_id)
    rgb_height, rgb_width = rgb.shape[:2]
    if require_int(manifest, "rgbWidth", f"{capture_id} manifest", positive=True) != rgb_width:
        raise ValueError(f"{capture_id}: decoded RGB width mismatch")
    if require_int(manifest, "rgbHeight", f"{capture_id} manifest", positive=True) != rgb_height:
        raise ValueError(f"{capture_id}: decoded RGB height mismatch")

    projection = require_mapping(
        manifest.get("projectionConfig"), f"{capture_id} manifest.projectionConfig"
    )
    if require_int(projection, "rgbWidth", f"{capture_id} projection", positive=True) != rgb_width:
        raise ValueError(f"{capture_id}: projection RGB width mismatch")
    if require_int(projection, "rgbHeight", f"{capture_id} projection", positive=True) != rgb_height:
        raise ValueError(f"{capture_id}: projection RGB height mismatch")
    panorama_width = require_int(projection, "width", f"{capture_id} projection", positive=True)
    panorama_height = require_int(projection, "height", f"{capture_id} projection", positive=True)
    vertical_fov = require_number(projection, "verticalFovDeg", f"{capture_id} projection")
    upload_scale = require_number(projection, "uploadScale", f"{capture_id} projection", positive=True)
    if not math.isclose(upload_scale, rgb_width / panorama_width, abs_tol=1e-12):
        raise ValueError(f"{capture_id}: projection uploadScale is inconsistent")

    try:
        with np.load(io.BytesIO(raw_bytes), allow_pickle=False) as raw:
            required_arrays = {"pred_disp", "relative_depth", "valid_mask", "metadata_json"}
            missing = required_arrays - set(raw.files)
            if missing:
                raise ValueError(f"raw NPZ missing: {', '.join(sorted(missing))}")
            pred_disp = np.asarray(raw["pred_disp"], dtype=np.float32).copy()
            relative_depth = np.asarray(raw["relative_depth"], dtype=np.float32).copy()
            valid_mask = np.asarray(raw["valid_mask"], dtype=bool).copy()
            raw_metadata = json.loads(str(raw["metadata_json"]))
    except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
        raise ValueError(f"{capture_id}: invalid raw NPZ: {error}") from error
    raw_metadata = require_mapping(raw_metadata, f"{capture_id} raw.metadata")
    if pred_disp.ndim != 2 or relative_depth.shape != pred_disp.shape or valid_mask.shape != pred_disp.shape:
        raise ValueError(f"{capture_id}: raw arrays must be matching 2-D arrays")
    raw_height, raw_width = pred_disp.shape
    epsilon = require_number(raw_metadata, "epsilon", f"{capture_id} raw.metadata", positive=True)
    expected_valid = np.isfinite(pred_disp) & (pred_disp > epsilon)
    if not np.array_equal(valid_mask, expected_valid):
        raise ValueError(f"{capture_id}: raw valid_mask is inconsistent with pred_disp")
    if not np.allclose(
        relative_depth[expected_valid],
        1.0 / pred_disp[expected_valid],
        rtol=1e-5,
        atol=1e-6,
    ) or np.any(relative_depth[~expected_valid] != 0):
        raise ValueError(f"{capture_id}: relative_depth is inconsistent with pred_disp")
    for metadata_key, identity_key in (
        ("session_id", "sessionId"),
        ("capture_id", "captureId"),
        ("location_id", "locationId"),
        ("frame_id", "frameId"),
    ):
        if require_string(raw_metadata, metadata_key, f"{capture_id} raw.metadata") != identity[identity_key]:
            raise ValueError(f"{capture_id}: raw/manifest {identity_key} mismatch")
    semantic_requirements = {
        "api_version": 2,
        "depth_mode": "da360-relative",
        "unit_pred_disp": "raw disparity (inverse depth), NOT per-frame normalized",
    }
    for key, expected in semantic_requirements.items():
        if raw_metadata.get(key) != expected:
            raise ValueError(f"{capture_id}: unsupported raw semantic field {key}")
    if require_int(raw_metadata, "width", f"{capture_id} raw.metadata", positive=True) != raw_width:
        raise ValueError(f"{capture_id}: raw metadata width mismatch")
    if require_int(raw_metadata, "height", f"{capture_id} raw.metadata", positive=True) != raw_height:
        raise ValueError(f"{capture_id}: raw metadata height mismatch")
    if require_int(raw_metadata, "request_width", f"{capture_id} raw.metadata", positive=True) != rgb_width:
        raise ValueError(f"{capture_id}: raw request width mismatch")
    if require_int(raw_metadata, "request_height", f"{capture_id} raw.metadata", positive=True) != rgb_height:
        raise ValueError(f"{capture_id}: raw request height mismatch")
    if manifest.get("rawModel") != raw_metadata.get("model"):
        raise ValueError(f"{capture_id}: raw model mismatch")
    if manifest.get("rawWidth") != raw_width or manifest.get("rawHeight") != raw_height:
        raise ValueError(f"{capture_id}: raw dimensions disagree with manifest")
    if raw_metadata.get("projection_config") != projection:
        raise ValueError(f"{capture_id}: raw projection contract mismatch")

    anchor_data = json_object(anchor_bytes, f"{capture_id} anchors")
    anchors = require_list(anchor_data.get("anchors"), f"{capture_id} anchors.anchors")
    failures = require_list(anchor_data.get("failures"), f"{capture_id} anchors.failures")
    anchor_metadata = require_mapping(
        anchor_data.get("metadata"), f"{capture_id} anchors.metadata"
    )
    if anchor_metadata.get("schemaVersion") != 1:
        raise ValueError(f"{capture_id}: anchor schemaVersion must be 1")
    if anchor_metadata.get("tileState") != "ready":
        raise ValueError(f"{capture_id}: Cesium tiles were not ready")
    if anchor_metadata.get("raycastSource") != "panorama-capture-viewer":
        raise ValueError(f"{capture_id}: unsupported Cesium ray-cast source")
    if anchor_metadata.get("tilesetSharedWithRgb") is not True:
        raise ValueError(f"{capture_id}: RGB and anchors do not share the tileset")
    anchor_identity = require_mapping(
        anchor_metadata.get("identity"), f"{capture_id} anchors.metadata.identity"
    )
    if anchor_identity != identity:
        raise ValueError(f"{capture_id}: anchor/manifest identity mismatch")
    if anchor_metadata.get("transform") != manifest.get("transform"):
        raise ValueError(f"{capture_id}: anchor/manifest transform mismatch")
    image_metadata = require_mapping(
        anchor_metadata.get("image"), f"{capture_id} anchors.metadata.image"
    )
    if image_metadata.get("width") != rgb_width or image_metadata.get("height") != rgb_height:
        raise ValueError(f"{capture_id}: anchor/RGB dimensions mismatch")
    if image_metadata.get("pixelCoordinateConvention") != "integer-pixel-centres":
        raise ValueError(f"{capture_id}: unsupported anchor pixel convention")
    erp = require_mapping(anchor_metadata.get("erp"), f"{capture_id} anchors.metadata.erp")
    if erp.get("verticalFovDeg") != vertical_fov:
        raise ValueError(f"{capture_id}: anchor/projection vertical FOV mismatch")
    if erp.get("sensorFrame") != "NWU(+x forward,+y left,+z up)":
        raise ValueError(f"{capture_id}: unsupported ERP sensor frame")
    if erp.get("componentFrame") != "(+x right,+y up,+z back)":
        raise ValueError(f"{capture_id}: unsupported ERP component frame")
    source_image = require_mapping(
        anchor_metadata.get("panoramaSourceImage"),
        f"{capture_id} anchors.metadata.panoramaSourceImage",
    )
    if source_image.get("width") != panorama_width or source_image.get("height") != panorama_height:
        raise ValueError(f"{capture_id}: anchor panorama source dimensions mismatch")
    if source_image.get("verticalFovDeg") != vertical_fov:
        raise ValueError(f"{capture_id}: anchor panorama source FOV mismatch")
    readiness = require_list(
        anchor_metadata.get("panoramaFaceTileReadiness"),
        f"{capture_id} anchors.metadata.panoramaFaceTileReadiness",
    )
    ready_faces = {
        require_string(require_mapping(entry, "face readiness"), "face", "face readiness")
        for entry in readiness
        if require_mapping(entry, "face readiness").get("readyWhenCopied") is True
    }
    if ready_faces != {"front", "right", "back", "left", "up", "down"}:
        raise ValueError(f"{capture_id}: not all six panorama faces were ready")

    sampling = require_mapping(
        anchor_metadata.get("sampling"), f"{capture_id} anchors.metadata.sampling"
    )
    grid_cols = require_int(sampling, "gridCols", f"{capture_id} sampling", positive=True)
    grid_rows = require_int(sampling, "gridRows", f"{capture_id} sampling", positive=True)
    max_range = require_number(sampling, "maxRangeM", f"{capture_id} sampling", positive=True)
    if (grid_rows, grid_cols) != (8, 16):
        raise ValueError(f"{capture_id}: expected the recorded 8x16 anchor grid")
    total_cells = require_int(anchor_metadata, "totalCells", f"{capture_id} anchors.metadata")
    if total_cells != grid_rows * grid_cols or len(anchors) + len(failures) != total_cells:
        raise ValueError(f"{capture_id}: incomplete anchor grid accounting")
    if anchor_metadata.get("validAnchors") != len(anchors) or anchor_metadata.get("failureCount") != len(failures):
        raise ValueError(f"{capture_id}: anchor metadata counts mismatch")
    if manifest.get("validAnchors") != len(anchors) or manifest.get("failedAnchors") != len(failures):
        raise ValueError(f"{capture_id}: manifest/anchor counts mismatch")

    records: list[dict[str, Any]] = []
    seen_cells: set[tuple[int, int]] = set()
    for index, item in enumerate(anchors):
        anchor = require_mapping(item, f"{capture_id} anchor[{index}]")
        col = require_int(anchor, "col", f"{capture_id} anchor[{index}]")
        row = require_int(anchor, "row", f"{capture_id} anchor[{index}]")
        if not (0 <= col < grid_cols and 0 <= row < grid_rows) or (col, row) in seen_cells:
            raise ValueError(f"{capture_id}: invalid or duplicate anchor grid cell {(col, row)}")
        seen_cells.add((col, row))
        u = require_number(anchor, "u", f"{capture_id} anchor[{index}]")
        v = require_number(anchor, "v", f"{capture_id} anchor[{index}]")
        distance = require_number(anchor, "distance", f"{capture_id} anchor[{index}]", positive=True)
        if distance > max_range or not (-0.5 <= u <= rgb_width - 0.5) or not (-0.5 <= v <= rgb_height - 0.5):
            raise ValueError(f"{capture_id}: anchor outside its declared range/image")
        raw_u = map_pixel_center(u, rgb_width, raw_width)
        raw_v = map_pixel_center(v, rgb_height, raw_height)
        disparity = sample_wrapped_bilinear(pred_disp, valid_mask, raw_u, raw_v)
        records.append(
            {
                "capture_id": capture_id,
                "site": identity["locationId"],
                "col": col,
                "row": row,
                "u": u,
                "v": v,
                "status": "hit",
                "distance_m": distance,
                "raw_u": raw_u,
                "raw_v": raw_v,
                "pred_disp": disparity,
            }
        )
    for index, item in enumerate(failures):
        failure = require_mapping(item, f"{capture_id} failure[{index}]")
        col = require_int(failure, "col", f"{capture_id} failure[{index}]")
        row = require_int(failure, "row", f"{capture_id} failure[{index}]")
        if not (0 <= col < grid_cols and 0 <= row < grid_rows) or (col, row) in seen_cells:
            raise ValueError(f"{capture_id}: invalid or duplicate failure grid cell {(col, row)}")
        seen_cells.add((col, row))
        reason = require_string(failure, "reason", f"{capture_id} failure[{index}]")
        if reason not in {"no_hit", "pole_excluded"}:
            raise ValueError(f"{capture_id}: unexpected anchor failure reason {reason!r}")
        records.append(
            {
                "capture_id": capture_id,
                "site": identity["locationId"],
                "col": col,
                "row": row,
                "u": require_number(failure, "u", f"{capture_id} failure[{index}]"),
                "v": require_number(failure, "v", f"{capture_id} failure[{index}]"),
                "status": reason,
                "distance_m": None,
                "raw_u": None,
                "raw_v": None,
                "pred_disp": None,
            }
        )
    if len(seen_cells) != total_cells:
        raise ValueError(f"{capture_id}: anchor grid does not cover all cells")
    records.sort(key=lambda record: (record["row"], record["col"]))

    return Capture(
        capture_id=capture_id,
        site=identity["locationId"],
        rgb=rgb,
        pred_disp=pred_disp,
        valid_mask=valid_mask,
        manifest=manifest,
        raw_metadata=raw_metadata,
        anchor_metadata=anchor_metadata,
        records=records,
        grid_rows=grid_rows,
        grid_cols=grid_cols,
    )


def close_enough(actual: float, expected: float, label: str) -> float:
    actual_value = float(actual)
    expected_value = float(expected)
    delta = abs(actual_value - expected_value)
    if not np.isclose(actual_value, expected_value, rtol=2e-7, atol=2e-10):
        raise ValueError(
            f"recomputed {label} does not reproduce fit_report: "
            f"{actual_value:.12g} != {expected_value:.12g}"
        )
    return delta


def analyze(captures: list[Capture], fit_report: dict[str, Any]) -> Analysis:
    if tuple(capture.capture_id for capture in captures) != CAPTURE_IDS:
        raise ValueError("capture set/order differs from the frozen 12-capture report")
    if tuple(fit_report.get("sample_ids", [])) != CAPTURE_IDS:
        raise ValueError("fit_report sample_ids differ from the frozen 12-capture set")
    if fit_report.get("n_samples") != len(CAPTURE_IDS):
        raise ValueError("fit_report n_samples is not 12")
    if fit_report.get("validation", {}).get("strategy") != "leave-one-location-out":
        raise ValueError("fit_report does not use leave-one-location-out validation")
    if fit_report.get("selected_model") != "scale_only":
        raise ValueError("fit_report selected model is not scale_only")

    all_records = [record for capture in captures for record in capture.records]
    range_records = [
        record
        for record in all_records
        if record["status"] == "hit"
        and record["pred_disp"] is not None
        and METRIC_MIN_DEPTH_M <= record["distance_m"] <= METRIC_MAX_DEPTH_M
    ]
    if not range_records:
        raise ValueError("no valid anchors in the metric evaluation range")
    x = np.asarray([record["pred_disp"] for record in range_records], dtype=np.float64)
    true_depth = np.asarray([record["distance_m"] for record in range_records], dtype=np.float64)
    inverse_depth = 1.0 / true_depth
    sites = np.asarray([record["site"] for record in range_records], dtype=str)

    fold_scales: dict[str, float] = {}
    predicted_depth = np.full_like(true_depth, np.nan, dtype=np.float64)
    for site in SITES:
        train = sites != site
        test = sites == site
        if np.count_nonzero(train) < 4 or np.count_nonzero(test) < 2:
            raise ValueError(f"insufficient anchors for held-out fold {site}")
        fold_scale = fit_scale_only(x[train], inverse_depth[train])
        fold_scales[site] = fold_scale
        predicted_inverse = fold_scale * x[test]
        predicted_depth[test] = np.divide(
            1.0,
            predicted_inverse,
            out=np.full(np.count_nonzero(test), np.nan, dtype=np.float64),
            where=predicted_inverse > 0,
        )

    abs_error = np.abs(predicted_depth - true_depth)
    abs_rel = abs_error / true_depth
    for index, record in enumerate(range_records):
        record["lolo_scale"] = fold_scales[record["site"]]
        record["lolo_predicted_depth_m"] = float(predicted_depth[index])
        record["abs_error_m"] = float(abs_error[index])
        record["lolo_abs_rel"] = float(abs_rel[index])

    pooled_scale = fit_scale_only(x, inverse_depth)
    pooled_metrics = compute_depth_metrics(true_depth, predicted_depth)
    status_counts = {
        status: sum(record["status"] == status for record in all_records)
        for status in ("hit", "no_hit", "pole_excluded")
    }
    valid_hits = sum(capture.n_valid_hits for capture in captures)

    validation_deltas = {
        "pooled_scale": close_enough(
            pooled_scale, fit_report["scale_only"]["a"], "pooled scale"
        ),
        "pooled_median_abs_rel": close_enough(
            pooled_metrics["median_abs_rel"],
            fit_report["scale_only"]["median_abs_rel"],
            "pooled LOLO median AbsRel",
        ),
        "pooled_p90_abs_rel": close_enough(
            pooled_metrics["p90_abs_rel"],
            fit_report["scale_only"]["p90_abs_rel"],
            "pooled LOLO p90 AbsRel",
        ),
    }
    expected_counts = {
        "n_total_anchors": len(all_records),
        "n_valid_anchors": valid_hits,
        "n_range_anchors": len(range_records),
    }
    for key, actual in expected_counts.items():
        if fit_report.get(key) != actual:
            raise ValueError(f"recomputed {key}={actual} differs from fit_report")
    expected_fraction = valid_hits / len(all_records)
    validation_deltas["valid_anchor_fraction"] = close_enough(
        expected_fraction,
        fit_report["valid_anchor_fraction"],
        "valid anchor fraction",
    )

    report_folds = {
        fold["held_out"]: fold for fold in fit_report["validation"]["folds"]
    }
    if set(report_folds) != set(SITES):
        raise ValueError("fit_report held-out locations differ from the four recorded sites")
    for site in SITES:
        test = sites == site
        metrics = compute_depth_metrics(true_depth[test], predicted_depth[test])
        report_fold = report_folds[site]
        if report_fold.get("n_val") != int(np.count_nonzero(test)):
            raise ValueError(f"recomputed held-out count differs for {site}")
        if report_fold.get("n_train") != int(np.count_nonzero(~test)):
            raise ValueError(f"recomputed training count differs for {site}")
        for metric_name in (
            "median_abs_rel",
            "p90_abs_rel",
            "rmse_m",
            "median_error_m",
            "p90_error_m",
        ):
            validation_deltas[f"{site}_{metric_name}"] = close_enough(
                metrics[metric_name],
                report_fold["scale_only"][metric_name],
                f"{site} {metric_name}",
            )

    capture_rows: list[dict[str, Any]] = []
    for capture in captures:
        capture_range = [
            record for record in range_records if record["capture_id"] == capture.capture_id
        ]
        capture_x = np.asarray([record["pred_disp"] for record in capture_range])
        capture_inverse = 1.0 / np.asarray(
            [record["distance_m"] for record in capture_range], dtype=np.float64
        )
        capture_absrel = np.asarray(
            [record["lolo_abs_rel"] for record in capture_range], dtype=np.float64
        )
        status = {
            name: sum(record["status"] == name for record in capture.records)
            for name in ("hit", "no_hit", "pole_excluded")
        }
        capture_rows.append(
            {
                "capture_id": capture.capture_id,
                "site": capture.site,
                "n_candidates": capture.n_candidates,
                "n_hit": status["hit"],
                "n_no_hit": status["no_hit"],
                "n_pole_excluded": status["pole_excluded"],
                "n_valid_depth_samples": capture.n_valid_hits,
                "n_range_anchors": len(capture_range),
                "valid_anchor_fraction": capture.n_valid_hits / capture.n_candidates,
                "capture_fit_scale": fit_scale_only(capture_x, capture_inverse),
                "held_out_site_scale": fold_scales[capture.site],
                "lolo_median_abs_rel": float(np.median(capture_absrel)),
                "lolo_p90_abs_rel": float(np.percentile(capture_absrel, 90)),
            }
        )

    if sum(status_counts.values()) != 12 * 8 * 16:
        raise ValueError("pooled anchor status counts do not cover all 1,536 grid cells")
    if fit_report.get("acceptance", {}).get("passed") is not False:
        raise ValueError("expected the recorded acceptance gate to be failed")
    return Analysis(
        captures=captures,
        range_records=range_records,
        capture_rows=capture_rows,
        fold_scales=fold_scales,
        pooled_scale=pooled_scale,
        pooled_metrics=pooled_metrics,
        fit_report=fit_report,
        validation_deltas=validation_deltas,
    )


def apply_style() -> None:
    mpl.rcParams.update(
        {
            "font.family": "sans-serif",
            "font.sans-serif": ["Arial", "DejaVu Sans", "Liberation Sans"],
            "svg.fonttype": "none",
            "pdf.fonttype": 42,
            "font.size": 7.0,
            "axes.labelsize": 7.0,
            "axes.titlesize": 7.6,
            "xtick.labelsize": 6.2,
            "ytick.labelsize": 6.2,
            "axes.spines.right": False,
            "axes.spines.top": False,
            "axes.linewidth": 0.7,
            "legend.frameon": False,
            "legend.fontsize": 6.2,
            "savefig.facecolor": "white",
            "figure.facecolor": "white",
        }
    )


def panel_label(
    ax: plt.Axes,
    label: str,
    *,
    color: str = "#111827",
    inside: bool = False,
) -> None:
    ax.text(
        0.012 if inside else -0.08,
        0.975 if inside else 1.04,
        label,
        transform=ax.transAxes,
        ha="left",
        va="top" if inside else "bottom",
        color=color,
        fontweight="bold",
        fontsize=8.5,
        zorder=20,
    )


def style_image_axis(ax: plt.Axes) -> None:
    ax.set_facecolor("#080B0F")
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(False)


def save_figure(
    fig: plt.Figure,
    output_stem: Path,
    *,
    png_dpi: int,
    tiff_dpi: int,
) -> list[Path]:
    output_stem.parent.mkdir(parents=True, exist_ok=True)
    outputs = [output_stem.with_suffix(suffix) for suffix in (".svg", ".pdf", ".png", ".tiff")]
    common = {"bbox_inches": "tight", "facecolor": "white"}
    fig.savefig(outputs[0], **common)
    fig.savefig(outputs[1], **common, metadata={"Creator": "matplotlib", "CreationDate": None})
    fig.savefig(outputs[2], **common, dpi=png_dpi)
    fig.savefig(
        outputs[3],
        **common,
        dpi=tiff_dpi,
        pil_kwargs={"compression": "tiff_lzw"},
    )
    plt.close(fig)
    return outputs


def add_rgb(ax: plt.Axes, rgb: np.ndarray, *, dim: float = 1.0) -> None:
    shown = np.clip(rgb.astype(np.float32) * dim, 0, 255).astype(np.uint8)
    ax.imshow(shown, interpolation="nearest", origin="upper")
    ax.set_xlim(-0.5, rgb.shape[1] - 0.5)
    ax.set_ylim(rgb.shape[0] - 0.5, -0.5)


def figure1_anchor_sampling(
    analysis: Analysis,
    output_dir: Path,
    *,
    png_dpi: int,
    tiff_dpi: int,
) -> list[Path]:
    representative = next(
        capture for capture in analysis.captures if capture.capture_id == REPRESENTATIVE_CAPTURE
    )
    fig = plt.figure(figsize=(7.2047, 6.15), constrained_layout=True)
    grid = fig.add_gridspec(2, 2, height_ratios=(1.28, 1.0), width_ratios=(1.02, 1.25))
    ax_image = fig.add_subplot(grid[0, :])
    ax_heat = fig.add_subplot(grid[1, 0])
    ax_status = fig.add_subplot(grid[1, 1])

    style_image_axis(ax_image)
    add_rgb(ax_image, representative.rgb, dim=0.78)
    hits = [record for record in representative.records if record["status"] == "hit"]
    no_hits = [record for record in representative.records if record["status"] == "no_hit"]
    excluded = [
        record for record in representative.records if record["status"] == "pole_excluded"
    ]
    hit_scatter = ax_image.scatter(
        [record["u"] for record in hits],
        [record["v"] for record in hits],
        c=[record["distance_m"] for record in hits],
        cmap="viridis",
        norm=LogNorm(vmin=METRIC_MIN_DEPTH_M, vmax=100.0),
        s=23,
        edgecolor="white",
        linewidth=0.35,
        zorder=4,
    )
    ax_image.scatter(
        [record["u"] for record in no_hits],
        [record["v"] for record in no_hits],
        marker="x",
        s=25,
        color=STATUS_COLORS["no_hit"],
        linewidth=1.0,
        zorder=5,
    )
    ax_image.scatter(
        [record["u"] for record in excluded],
        [record["v"] for record in excluded],
        marker="s",
        s=22,
        facecolors="none",
        edgecolors="#E5E7EB",
        linewidth=0.8,
        zorder=5,
    )
    ax_image.set_title(
        f"a  Recorded ERP + Cesium anchors  |  {REPRESENTATIVE_CAPTURE}",
        loc="left",
        color="#111827",
        fontweight="bold",
        pad=4,
    )
    ax_image.text(
        0.99,
        0.04,
        f"{len(hits)} hits  ·  {len(no_hits)} no hit  ·  {len(excluded)} pole excluded",
        transform=ax_image.transAxes,
        ha="right",
        va="bottom",
        color="white",
        fontsize=6.2,
        bbox={"facecolor": "black", "alpha": 0.58, "edgecolor": "none", "pad": 2.0},
    )
    marker_handles = [
        Line2D([], [], marker="x", linestyle="none", color=STATUS_COLORS["no_hit"], label="No hit"),
        Line2D(
            [],
            [],
            marker="s",
            linestyle="none",
            markerfacecolor="none",
            markeredgecolor="#E5E7EB",
            label="Pole excluded",
        ),
    ]
    ax_image.legend(
        handles=marker_handles,
        loc="lower left",
        bbox_to_anchor=(0.006, 0.01),
        labelcolor="white",
        ncol=2,
        handletextpad=0.35,
        columnspacing=0.8,
        frameon=True,
        facecolor="black",
        edgecolor="none",
        framealpha=0.58,
    )
    colorbar = fig.colorbar(hit_scatter, ax=ax_image, pad=0.008, fraction=0.022, aspect=38)
    colorbar.set_label("Cesium ray-cast distance (m)")
    colorbar.set_ticks([0.5, 1, 2, 5, 10, 20, 50, 100])
    colorbar.ax.yaxis.set_major_formatter(FuncFormatter(lambda value, _: f"{value:g}"))
    colorbar.ax.tick_params(labelsize=5.8)

    hit_count = np.zeros((representative.grid_rows, representative.grid_cols), dtype=float)
    attempted_count = np.zeros_like(hit_count)
    for capture in analysis.captures:
        for record in capture.records:
            row, col = record["row"], record["col"]
            if record["status"] != "pole_excluded":
                attempted_count[row, col] += 1
            if record["status"] == "hit":
                hit_count[row, col] += 1
    hit_rate = np.divide(
        hit_count,
        attempted_count,
        out=np.full_like(hit_count, np.nan),
        where=attempted_count > 0,
    )
    heat_cmap = plt.get_cmap("Blues").copy()
    heat_cmap.set_bad("#D7DBDF")
    heat = ax_heat.imshow(
        np.ma.masked_invalid(hit_rate),
        origin="upper",
        cmap=heat_cmap,
        norm=Normalize(0, 1),
        interpolation="nearest",
        aspect="auto",
    )
    ax_heat.set_xticks(np.arange(representative.grid_cols))
    ax_heat.set_xticklabels(np.arange(1, representative.grid_cols + 1), fontsize=5.2)
    ax_heat.set_yticks(np.arange(representative.grid_rows))
    ax_heat.set_yticklabels(np.arange(1, representative.grid_rows + 1))
    ax_heat.set_xticks(np.arange(-0.5, representative.grid_cols, 1), minor=True)
    ax_heat.set_yticks(np.arange(-0.5, representative.grid_rows, 1), minor=True)
    ax_heat.grid(which="minor", color="white", linewidth=0.35, alpha=0.75)
    ax_heat.tick_params(which="minor", bottom=False, left=False)
    ax_heat.set_xlabel("Anchor-grid column")
    ax_heat.set_ylabel("Anchor-grid row")
    ax_heat.text(
        (representative.grid_cols - 1) / 2,
        0,
        "N/A: pole-excluded",
        ha="center",
        va="center",
        fontsize=5.7,
        color="#30343B",
        fontweight="bold",
    )
    ax_heat.set_title(
        "Attempted-ray hit rate across 12 captures",
        loc="left",
        fontweight="bold",
    )
    panel_label(ax_heat, "b")
    heat_bar = fig.colorbar(heat, ax=ax_heat, pad=0.02, fraction=0.05)
    heat_bar.set_label("Hit rate")
    heat_bar.set_ticks([0.25, 0.5, 0.75, 1])

    y_positions = np.arange(len(analysis.capture_rows))
    left = np.zeros(len(y_positions), dtype=float)
    stack_specs = (
        ("n_hit", "Hit", "hit"),
        ("n_no_hit", "No hit", "no_hit"),
        ("n_pole_excluded", "Pole excluded", "pole_excluded"),
    )
    for field, label, status in stack_specs:
        values = np.asarray([row[field] for row in analysis.capture_rows], dtype=float)
        bars = ax_status.barh(
            y_positions,
            values,
            left=left,
            height=0.66,
            color=STATUS_COLORS[status],
            edgecolor="white",
            linewidth=0.4,
            label=label,
        )
        for bar, value in zip(bars, values):
            ax_status.text(
                bar.get_x() + bar.get_width() / 2,
                bar.get_y() + bar.get_height() / 2,
                f"{int(value)}",
                ha="center",
                va="center",
                color="white" if status != "pole_excluded" else "#30343B",
                fontsize=5.2,
                fontweight="bold",
            )
        left += values
    ax_status.set_yticks(
        y_positions,
        [short_capture_label(row["capture_id"]) for row in analysis.capture_rows],
    )
    ax_status.invert_yaxis()
    # Reserve one empty row inside the axes for the status legend.
    ax_status.set_ylim(len(y_positions) - 0.45, -1.15)
    ax_status.set_xlim(0, 128)
    ax_status.set_xticks([0, 32, 64, 96, 128])
    ax_status.set_xlabel("Grid cells per capture (n = 128)")
    ax_status.set_title("Observed status counts by capture", loc="left", fontweight="bold")
    ax_status.grid(axis="x", color="#E3E7EB", linewidth=0.55, zorder=0)
    ax_status.legend(loc="upper center", bbox_to_anchor=(0.5, 0.99), ncol=3, fontsize=5.5)
    panel_label(ax_status, "c")

    total_hits = sum(row["n_hit"] for row in analysis.capture_rows)
    total_no_hit = sum(row["n_no_hit"] for row in analysis.capture_rows)
    total_pole = sum(row["n_pole_excluded"] for row in analysis.capture_rows)
    attempted_hit_rate = total_hits / (total_hits + total_no_hit)
    fig.suptitle(
        "Cesium geometry anchors: recorded sampling coverage and failure modes\n"
        f"12 sim-to-sim captures · {total_hits:,} hits · {total_no_hit} no hit · "
        f"{total_pole} pole excluded · attempted-ray hit rate {attempted_hit_rate:.2%}",
        x=0.01,
        ha="left",
        fontsize=9,
        fontweight="bold",
    )
    return save_figure(
        fig,
        output_dir / "figure1_cesium_anchor_sampling",
        png_dpi=png_dpi,
        tiff_dpi=tiff_dpi,
    )


def figure2_sparse_truth_comparison(
    analysis: Analysis,
    output_dir: Path,
    *,
    png_dpi: int,
    tiff_dpi: int,
) -> list[Path]:
    representative = next(
        capture for capture in analysis.captures if capture.capture_id == REPRESENTATIVE_CAPTURE
    )
    representative_records = [
        record
        for record in analysis.range_records
        if record["capture_id"] == REPRESENTATIVE_CAPTURE
    ]
    scale = analysis.fold_scales[representative.site]
    predicted_inverse = scale * representative.pred_disp
    metric_depth = np.divide(
        1.0,
        predicted_inverse,
        out=np.full_like(predicted_inverse, np.nan, dtype=np.float32),
        where=representative.valid_mask & np.isfinite(predicted_inverse) & (predicted_inverse > 0),
    )

    fig = plt.figure(figsize=(7.2047, 5.25), constrained_layout=True)
    grid = fig.add_gridspec(2, 3, height_ratios=(0.92, 1.58))
    ax_pred = fig.add_subplot(grid[0, 0])
    ax_truth = fig.add_subplot(grid[0, 1])
    ax_error = fig.add_subplot(grid[0, 2])
    ax_scatter = fig.add_subplot(grid[1, :])
    depth_norm = LogNorm(vmin=METRIC_MIN_DEPTH_M, vmax=METRIC_MAX_DEPTH_M)

    style_image_axis(ax_pred)
    pred_image = ax_pred.imshow(
        metric_depth,
        origin="upper",
        cmap="viridis",
        norm=depth_norm,
        interpolation="nearest",
    )
    ax_pred.set_title("a  DA360 LOLO depth (dense)", loc="left", fontweight="bold")

    style_image_axis(ax_truth)
    add_rgb(ax_truth, representative.rgb, dim=0.50)
    truth_points = ax_truth.scatter(
        [record["u"] for record in representative_records],
        [record["v"] for record in representative_records],
        c=[record["distance_m"] for record in representative_records],
        cmap="viridis",
        norm=depth_norm,
        s=24,
        edgecolors="white",
        linewidth=0.35,
    )
    ax_truth.set_title(
        f"b  Cesium sparse truth (n = {len(representative_records)})",
        loc="left",
        fontweight="bold",
    )
    ax_truth.text(
        0.985,
        0.035,
        "Markers only; no dense Cesium map",
        transform=ax_truth.transAxes,
        ha="right",
        va="bottom",
        color="white",
        fontsize=5.4,
        bbox={"facecolor": "black", "alpha": 0.58, "edgecolor": "none", "pad": 1.7},
    )

    style_image_axis(ax_error)
    add_rgb(ax_error, representative.rgb, dim=0.50)
    error_points = ax_error.scatter(
        [record["u"] for record in representative_records],
        [record["v"] for record in representative_records],
        c=[record["lolo_abs_rel"] for record in representative_records],
        cmap="magma",
        norm=Normalize(vmin=0, vmax=1.0),
        s=24,
        edgecolors="white",
        linewidth=0.35,
    )
    ax_error.set_title("c  Sparse-anchor LOLO AbsRel", loc="left", fontweight="bold")

    depth_bar = fig.colorbar(
        pred_image,
        ax=[ax_pred, ax_truth],
        orientation="horizontal",
        pad=0.035,
        fraction=0.08,
        aspect=34,
        extend="both",
    )
    depth_bar.set_label("Depth / distance (m; display clipped)")
    depth_bar.set_ticks([0.5, 1, 2, 5, 10, 20])
    depth_bar.ax.xaxis.set_major_formatter(FuncFormatter(lambda value, _: f"{value:g}"))
    error_bar = fig.colorbar(
        error_points,
        ax=ax_error,
        orientation="horizontal",
        pad=0.035,
        fraction=0.08,
        aspect=17,
    )
    error_bar.set_label("Absolute relative error")
    error_bar.set_ticks([0, 0.25, 0.5, 0.75, 1])

    for site in SITES:
        records = [record for record in analysis.range_records if record["site"] == site]
        ax_scatter.scatter(
            [record["distance_m"] for record in records],
            [record["lolo_predicted_depth_m"] for record in records],
            s=9,
            alpha=0.48,
            linewidth=0,
            color=SITE_COLORS[site],
            label=f"Site {site[-1].upper()} (n={len(records)})",
        )
    all_depths = np.asarray(
        [record["distance_m"] for record in analysis.range_records]
        + [record["lolo_predicted_depth_m"] for record in analysis.range_records]
    )
    lower = max(0.2, float(np.nanmin(all_depths)) * 0.82)
    upper = float(np.nanmax(all_depths)) * 1.18
    identity_line = np.geomspace(lower, upper, 250)
    ax_scatter.plot(identity_line, identity_line, color="#20242A", lw=1.0, ls="--", label="Identity")
    ax_scatter.set_xscale("log")
    ax_scatter.set_yscale("log")
    ax_scatter.set_xlim(METRIC_MIN_DEPTH_M * 0.92, METRIC_MAX_DEPTH_M * 1.08)
    ax_scatter.set_ylim(lower, upper)
    x_ticks = [0.5, 1, 2, 5, 10, 20]
    y_ticks = [value for value in (0.5, 1, 2, 5, 10, 20, 50, 100) if lower <= value <= upper]
    ax_scatter.set_xticks(x_ticks)
    ax_scatter.set_yticks(y_ticks)
    plain_log_formatter = FuncFormatter(lambda value, _: f"{value:g}")
    ax_scatter.xaxis.set_major_formatter(plain_log_formatter)
    ax_scatter.yaxis.set_major_formatter(plain_log_formatter)
    ax_scatter.xaxis.set_minor_formatter(NullFormatter())
    ax_scatter.yaxis.set_minor_formatter(NullFormatter())
    ax_scatter.set_xlabel("Cesium sparse true distance (m)")
    ax_scatter.set_ylabel("DA360 LOLO predicted depth (m)")
    ax_scatter.set_title("d  All in-range sparse comparisons (n = 833)", loc="left", fontweight="bold")
    ax_scatter.grid(which="both", color="#E2E8F0", lw=0.5)
    ax_scatter.legend(loc="upper left", ncol=2, fontsize=5.4, handletextpad=0.3, columnspacing=0.8)
    ax_scatter.text(
        0.98,
        0.04,
        "Pooled LOLO\n"
        f"median AbsRel = {analysis.pooled_metrics['median_abs_rel']:.2%}\n"
        f"p90 AbsRel = {analysis.pooled_metrics['p90_abs_rel']:.2%}",
        transform=ax_scatter.transAxes,
        ha="right",
        va="bottom",
        fontsize=6.1,
        bbox={"facecolor": "white", "alpha": 0.88, "edgecolor": "#CBD2D9", "pad": 2.4},
    )
    fig.suptitle(
        "DA360 depth versus Cesium sparse ray-cast truth\n"
        "Recorded sim-to-sim evidence; Cesium truth is sparse (8×16 candidate grid), not a dense GT map",
        x=0.01,
        ha="left",
        fontsize=9,
        fontweight="bold",
    )
    return save_figure(
        fig,
        output_dir / "figure2_da360_vs_cesium_sparse_truth",
        png_dpi=png_dpi,
        tiff_dpi=tiff_dpi,
    )


def capture_positions(rows: list[dict[str, Any]]) -> np.ndarray:
    positions = []
    cursor = 0.0
    previous_site = None
    for row in rows:
        if previous_site is not None and row["site"] != previous_site:
            cursor += 0.75
        positions.append(cursor)
        cursor += 1.0
        previous_site = row["site"]
    return np.asarray(positions, dtype=float)


def add_site_separators(ax: plt.Axes, positions: np.ndarray, rows: list[dict[str, Any]]) -> None:
    for index in range(1, len(rows)):
        if rows[index]["site"] != rows[index - 1]["site"]:
            midpoint = (positions[index - 1] + positions[index]) / 2
            ax.axvline(midpoint, color="#CBD2D9", lw=0.65, zorder=0)


def figure3_capture_scale_absrel(
    analysis: Analysis,
    output_dir: Path,
    *,
    png_dpi: int,
    tiff_dpi: int,
) -> list[Path]:
    rows = analysis.capture_rows
    positions = capture_positions(rows)
    labels = [
        f"{short_capture_label(row['capture_id'])}\nn={row['n_range_anchors']}" for row in rows
    ]
    fig, (ax_scale, ax_error) = plt.subplots(
        1,
        2,
        figsize=(7.2047, 3.5),
        constrained_layout=True,
        gridspec_kw={"width_ratios": (1.0, 1.22)},
    )

    scales = np.asarray([row["capture_fit_scale"] for row in rows], dtype=float) * 1e3
    for site in SITES:
        indices = np.asarray([index for index, row in enumerate(rows) if row["site"] == site])
        ax_scale.plot(
            positions[indices],
            scales[indices],
            color=SITE_COLORS[site],
            lw=1.0,
            alpha=0.65,
        )
        ax_scale.scatter(
            positions[indices],
            scales[indices],
            s=28,
            color=SITE_COLORS[site],
            edgecolor="white",
            linewidth=0.5,
            zorder=3,
            label=f"Site {site[-1].upper()}",
        )
    global_scale = analysis.pooled_scale * 1e3
    ax_scale.axhline(
        global_scale,
        color="#30343B",
        lw=0.9,
        ls="--",
        label=f"Pooled diagnostic fit ({global_scale:.3f})",
    )
    add_site_separators(ax_scale, positions, rows)
    ax_scale.set_xticks(positions, [short_capture_label(row["capture_id"]) for row in rows])
    ax_scale.tick_params(axis="x", labelrotation=45)
    for tick in ax_scale.get_xticklabels():
        tick.set_ha("right")
        tick.set_rotation_mode("anchor")
    ax_scale.set_ylabel("In-sample inverse-depth coefficient a (×10⁻³)")
    ax_scale.set_xlabel("Capture (site letter + try number)")
    ax_scale.set_ylim(0, max(scales) * 1.16)
    ax_scale.grid(axis="y", color="#E2E8F0", lw=0.55)
    ax_scale.set_title("Diagnostic scale varies across captures", loc="left", fontweight="bold")
    ax_scale.legend(loc="upper left", ncol=2, fontsize=5.4, columnspacing=0.8)
    ax_scale.text(
        0.98,
        0.95,
        f"max/min = {np.max(scales) / np.min(scales):.2f}×",
        transform=ax_scale.transAxes,
        ha="right",
        va="top",
        fontsize=6.2,
    )
    panel_label(ax_scale, "a")

    medians = np.asarray([row["lolo_median_abs_rel"] for row in rows])
    p90s = np.asarray([row["lolo_p90_abs_rel"] for row in rows])
    for x_position, median, p90 in zip(positions, medians, p90s):
        ax_error.plot([x_position, x_position], [median, p90], color="#B7BEC6", lw=0.8, zorder=1)
    ax_error.scatter(
        positions,
        medians,
        s=27,
        color="#4C78A8",
        edgecolor="white",
        linewidth=0.45,
        label="Median AbsRel",
        zorder=3,
    )
    ax_error.scatter(
        positions,
        p90s,
        marker="s",
        s=24,
        color="#D85858",
        edgecolor="white",
        linewidth=0.45,
        label="P90 AbsRel",
        zorder=3,
    )
    ax_error.axhline(
        MEDIAN_ABSREL_GATE,
        color="#4C78A8",
        lw=0.85,
        ls="--",
        label="Median gate (15%)",
    )
    ax_error.axhline(
        P90_ABSREL_GATE,
        color="#D85858",
        lw=0.85,
        ls=":",
        label="P90 gate (30%)",
    )
    add_site_separators(ax_error, positions, rows)
    ax_error.set_yscale("log")
    lower = min(float(np.min(medians)), MEDIAN_ABSREL_GATE) * 0.72
    upper = max(float(np.max(p90s)), P90_ABSREL_GATE) * 1.45
    ax_error.set_ylim(lower, upper)
    ax_error.yaxis.set_major_locator(LogLocator(base=10, subs=(1, 2, 5)))
    ax_error.yaxis.set_minor_formatter(NullFormatter())
    ax_error.yaxis.set_major_formatter(FuncFormatter(lambda value, _: f"{value:.0%}"))
    ax_error.set_xticks(positions, labels)
    ax_error.tick_params(axis="x", labelsize=5.5)
    ax_error.set_ylabel("LOLO absolute relative error")
    ax_error.set_xlabel("Capture / number of in-range anchors")
    ax_error.grid(axis="y", which="both", color="#E2E8F0", lw=0.55)
    ax_error.set_title("Every capture misses at least one error gate", loc="left", fontweight="bold")
    ax_error.legend(loc="upper left", ncol=2, fontsize=5.4, columnspacing=0.7)
    ax_error.text(
        0.99,
        0.035,
        "Vertical segments join median to p90; they are not confidence intervals",
        transform=ax_error.transAxes,
        ha="right",
        va="bottom",
        color="#5D6670",
        fontsize=5.6,
    )
    panel_label(ax_error, "b")

    fig.suptitle(
        "Capture-level inverse-depth scale drift and held-out DA360 error\n"
        "Panel a: in-sample diagnostic fits; panel b: leave-one-site-out sparse-anchor error (0.5–20 m)",
        x=0.01,
        ha="left",
        fontsize=9,
        fontweight="bold",
    )
    return save_figure(
        fig,
        output_dir / "figure3_capture_scale_absrel",
        png_dpi=png_dpi,
        tiff_dpi=tiff_dpi,
    )


def serializable(value: Any) -> Any:
    if value is None:
        return ""
    if isinstance(value, (np.floating, float)):
        return f"{float(value):.12g}"
    if isinstance(value, (np.integer, int)):
        return int(value)
    if isinstance(value, (np.bool_, bool)):
        return str(bool(value)).lower()
    return value


def write_csv(path: Path, fieldnames: list[str], rows: Iterable[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({field: serializable(row.get(field)) for field in fieldnames})


def write_source_data(analysis: Analysis, output_dir: Path) -> list[Path]:
    source_dir = output_dir / "source_data"
    figure1_rows = [
        {
            **record,
            "in_metric_range": (
                record["distance_m"] is not None
                and METRIC_MIN_DEPTH_M <= record["distance_m"] <= METRIC_MAX_DEPTH_M
            ),
        }
        for capture in analysis.captures
        for record in capture.records
    ]
    figure1_path = source_dir / "figure1_anchor_status.csv"
    write_csv(
        figure1_path,
        [
            "capture_id",
            "site",
            "col",
            "row",
            "u",
            "v",
            "status",
            "distance_m",
            "pred_disp",
            "in_metric_range",
        ],
        figure1_rows,
    )

    figure2_path = source_dir / "figure2_sparse_depth_comparison.csv"
    write_csv(
        figure2_path,
        [
            "capture_id",
            "site",
            "col",
            "row",
            "u",
            "v",
            "distance_m",
            "pred_disp",
            "lolo_scale",
            "lolo_predicted_depth_m",
            "abs_error_m",
            "lolo_abs_rel",
        ],
        analysis.range_records,
    )

    figure3_path = source_dir / "figure3_capture_metrics.csv"
    write_csv(
        figure3_path,
        [
            "capture_id",
            "site",
            "n_candidates",
            "n_hit",
            "n_no_hit",
            "n_pole_excluded",
            "n_valid_depth_samples",
            "n_range_anchors",
            "valid_anchor_fraction",
            "capture_fit_scale",
            "held_out_site_scale",
            "lolo_median_abs_rel",
            "lolo_p90_abs_rel",
        ],
        analysis.capture_rows,
    )
    return [figure1_path, figure2_path, figure3_path]


def write_qa(
    analysis: Analysis,
    output_dir: Path,
    reader: EvidenceReader,
    figure_paths: list[Path],
    source_paths: list[Path],
) -> Path:
    all_records = [record for capture in analysis.captures for record in capture.records]
    counts = {
        status: sum(record["status"] == status for record in all_records)
        for status in ("hit", "no_hit", "pole_excluded")
    }
    maximum_delta = max(analysis.validation_deltas.values())
    relative_outputs = [str(path.relative_to(output_dir)) for path in figure_paths]
    relative_sources = [str(path.relative_to(output_dir)) for path in source_paths]
    lines = [
        "# DA360/Cesium real-data figure QA",
        "",
        "## Figure contract",
        "",
        "- Figure 1 conclusion: the recorded 8×16 Cesium anchor process has explicit spatial coverage and failure modes across 12 captures.",
        "- Figure 1 archetype: image plate + quantitative validation.",
        "- Figure 2 conclusion: DA360 dense estimates can be compared only at recorded sparse Cesium ray-cast locations; the fixed cross-site scale has large held-out errors.",
        "- Figure 2 archetype: asymmetric mixed-modality figure.",
        "- Figure 3 conclusion: the in-sample inverse-depth coefficient varies across captures and leave-one-site-out errors miss the recorded acceptance gates.",
        "- Figure 3 archetype: quantitative grid.",
        "- Backend: Python/matplotlib only; final width 183 mm; editable SVG/PDF plus PNG and 600 dpi TIFF.",
        "",
        "## Input scope",
        "",
        f"- Input mode: `{reader.mode}`",
        f"- Input source: `{reader.source_label}`",
        f"- Captures: {len(analysis.captures)} (only the IDs listed in the fit report)",
        f"- Candidate anchor cells: {len(all_records):,}",
        f"- Cesium hits: {counts['hit']:,}",
        f"- No-hit cells: {counts['no_hit']:,}",
        f"- Pole-excluded cells: {counts['pole_excluded']:,}",
        f"- In-range comparison anchors: {len(analysis.range_records):,} (0.5–20 m)",
        f"- Candidate-cell yield: {counts['hit'] / len(all_records):.6%}",
        f"- Attempted-ray hit rate: {counts['hit'] / (counts['hit'] + counts['no_hit']):.6%}",
        "",
        "## Structural and numerical checks",
        "",
        "- All 12 RGB/raw/anchor/manifest bundles were present and paired by filename.",
        "- Manifest, raw metadata, and anchor identity fields agreed for every capture.",
        "- RGB/projection/raw array dimensions and relative-depth semantics were consistent.",
        "- Cesium tiles and all six panorama faces were reported ready at capture time.",
        "- Every capture accounted for all 128 cells in its recorded 8×16 grid.",
        "- The LOLO Huber scale-only calculation reproduced the fit-report counts and metrics.",
        f"- Largest absolute numerical delta among audited fit-report fields: {maximum_delta:.3g}.",
        "",
        "## Recomputed headline results",
        "",
        f"- Pooled fitted scale: {analysis.pooled_scale:.12g}",
        f"- Pooled LOLO median AbsRel: {analysis.pooled_metrics['median_abs_rel']:.6%}",
        f"- Pooled LOLO p90 AbsRel: {analysis.pooled_metrics['p90_abs_rel']:.6%}",
        "- Recorded acceptance outcome: failed.",
        "",
        "## Statistics and replicate definition",
        "",
        "- Figure 1 reports all recorded candidate cells; the top pole-excluded row is N/A in the attempted-ray heatmap.",
        "- Figure 2 uses 833 paired anchors. LOLO means leave one site out; the three captures at a site share one training-derived scale.",
        "- Figure 3 panel a is an in-sample diagnostic Huber fit per capture; coefficient a has units (1/m)/disparity and is not a depth multiplier.",
        "- Figure 3 panel b reports the median and p90 of anchor-level AbsRel within each capture.",
        "- Replicate unit: capture frame for capture summaries; anchors are nested spatial samples, not independent experimental replicates.",
        "- No repeated inference seeds are available, so no confidence interval, hypothesis test, multiple-comparison correction, or p value is claimed.",
        "",
        "## Image integrity",
        "",
        "- The example RGB is the recorded 134×67 ERP image, displayed without cropping or stitching and with nearest-neighbour rendering.",
        "- RGB brightness is reduced globally for marker visibility (78% in Figure 1; 50% in Figure 2); no local adjustment is applied.",
        "- DA360 depth is rendered directly from the recorded disparity using the held-out-site scale and a fixed 0.5–20 m display range with over/under colours.",
        "- Cesium distances remain discrete markers; no sparse-to-dense interpolation or synthetic ground-truth map is created.",
        "",
        "## Interpretation limits",
        "",
        "- These are recorded Cesium→DA360 sim-to-sim results, not real-world flight-sensor measurements.",
        "- Cesium truth is sparse ray-cast anchor distance, not a dense ground-truth depth image.",
        "- Panel 3 reports capture medians and p90 values without confidence intervals.",
        "- The quantitative error scope is restricted to Cesium anchors in 0.5–20 m.",
        "",
        "## Outputs",
        "",
    ]
    lines.extend(f"- `{path}`" for path in relative_outputs)
    lines.extend(["", "## Source data", ""])
    lines.extend(f"- `{path}`" for path in relative_sources)
    path = output_dir / "QA.md"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def main() -> None:
    args = parse_args()
    if args.png_dpi < 72 or args.tiff_dpi < 72:
        raise ValueError("export DPI values must be at least 72")
    apply_style()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    with choose_reader(args) as reader:
        fit_report = json_object(reader.read_fit_report(), "fit_report")
        captures = [load_capture(reader, capture_id) for capture_id in CAPTURE_IDS]
        analysis = analyze(captures, fit_report)
        source_paths = write_source_data(analysis, args.output_dir)
        figure_paths: list[Path] = []
        figure_paths.extend(
            figure1_anchor_sampling(
                analysis,
                args.output_dir,
                png_dpi=args.png_dpi,
                tiff_dpi=args.tiff_dpi,
            )
        )
        figure_paths.extend(
            figure2_sparse_truth_comparison(
                analysis,
                args.output_dir,
                png_dpi=args.png_dpi,
                tiff_dpi=args.tiff_dpi,
            )
        )
        figure_paths.extend(
            figure3_capture_scale_absrel(
                analysis,
                args.output_dir,
                png_dpi=args.png_dpi,
                tiff_dpi=args.tiff_dpi,
            )
        )
        qa_path = write_qa(
            analysis,
            args.output_dir,
            reader,
            figure_paths,
            source_paths,
        )

    print(f"Validated {len(captures)} recorded capture bundles")
    print(
        "Reproduced fit_report: "
        f"n={len(analysis.range_records)}, "
        f"median AbsRel={analysis.pooled_metrics['median_abs_rel']:.8f}, "
        f"p90 AbsRel={analysis.pooled_metrics['p90_abs_rel']:.8f}"
    )
    for path in figure_paths + source_paths + [qa_path]:
        print(path)


if __name__ == "__main__":
    main()
