"""Shared depth/mask validity and structured ERP dropout utilities."""

import cv2
import numpy as np


def valid_depth_mask(depth, mask_raw=None, threshold=127, minimum_depth=0.0):
    valid = np.isfinite(depth) & (depth >= minimum_depth)
    if mask_raw is not None:
        valid &= np.asarray(mask_raw) > threshold
    return valid


def fill_invalid_depth(depth, valid, mode="valid_mean"):
    if mode != "valid_mean":
        raise ValueError(f"Unsupported mask_fill_mode: {mode}")
    output = np.asarray(depth, dtype=np.float32).copy()
    if valid.any():
        fill = float(output[valid].mean())
    else:
        fill = 1.0
    output[~valid] = fill
    return output


def _wrapped_horizontal_distance(columns, center, width):
    direct = np.abs(columns - center)
    return np.minimum(direct, width - direct)


def _random_hole(height, width, width_range, height_range, rng):
    hole_width = max(1, int(round(width * rng.uniform(*width_range))))
    hole_height = max(1, int(round(height * rng.uniform(*height_range))))
    center_x = rng.uniform(0, width)
    center_y = rng.uniform(0, height)
    x_distance = _wrapped_horizontal_distance(
        np.arange(width, dtype=np.float32), center_x, width
    )
    y_distance = np.abs(np.arange(height, dtype=np.float32) - center_y)
    if rng.random() < 0.5:
        return (
            (y_distance[:, None] <= hole_height / 2.0)
            & (x_distance[None, :] <= hole_width / 2.0)
        )
    return (
        (x_distance[None, :] / max(hole_width / 2.0, 1.0)) ** 2
        + (y_distance[:, None] / max(hole_height / 2.0, 1.0)) ** 2
        <= 1.0
    )


def structured_mask_dropout(valid, settings, rng=None):
    """Add realistic block/band failures while respecting ERP horizontal wrap."""
    rng = np.random.default_rng() if rng is None else rng
    base_valid = np.asarray(valid, dtype=bool)
    if (
        not settings.get("enabled", False)
        or rng.random() >= float(settings["probability"])
    ):
        return base_valid.copy(), 0.0

    height, width = base_valid.shape
    ratio_min, ratio_max = map(float, settings["dropout_ratio"])
    if ratio_max <= 0.25:
        target = rng.uniform(ratio_min, ratio_max)
    elif ratio_min >= 0.25:
        target = rng.uniform(ratio_min, ratio_max)
    elif rng.random() < 0.8:
        target = rng.uniform(ratio_min, min(ratio_max, 0.25))
    else:
        target = rng.uniform(max(ratio_min, 0.25), ratio_max)
    keep = np.ones_like(base_valid)
    count_min, count_max = map(int, settings["hole_count"])
    planned_holes = int(rng.integers(count_min, count_max + 1))

    def added_ratio(candidate):
        return float(np.mean(base_valid & ~candidate))

    attempts = 0
    holes = 0
    while attempts < 64 and (holes < planned_holes or added_ratio(keep) < target):
        attempts += 1
        candidate = keep & ~_random_hole(
            height,
            width,
            settings["hole_width_fraction"],
            settings["hole_height_fraction"],
            rng,
        )
        if added_ratio(candidate) <= ratio_max:
            keep = candidate
            holes += 1

    if rng.random() < float(settings["band_probability"]):
        for _ in range(int(rng.integers(1, 3))):
            candidate = keep.copy()
            if rng.random() < 0.5:
                thickness = int(rng.integers(1, min(6, height) + 1))
                start = int(rng.integers(0, max(1, height - thickness + 1)))
                candidate[start:start + thickness, :] = False
            else:
                thickness = int(rng.integers(2, min(12, width) + 1))
                start = int(rng.integers(0, width))
                columns = (start + np.arange(thickness)) % width
                candidate[:, columns] = False
            if added_ratio(candidate) <= ratio_max:
                keep = candidate

    augmented = base_valid & keep
    minimum_valid = float(settings["min_valid_ratio"])
    if augmented.mean() < minimum_valid:
        return base_valid.copy(), 0.0
    ratio = float(np.mean(base_valid & ~augmented))
    if ratio < ratio_min:
        # Add one deterministic-width wrapped block to meet the configured floor.
        missing_columns = max(1, int(np.ceil((ratio_min - ratio) * width)))
        start = int(rng.integers(0, width))
        candidate = augmented.copy()
        candidate[:, (start + np.arange(missing_columns)) % width] = False
        candidate_ratio = float(np.mean(base_valid & ~candidate))
        if candidate.mean() >= minimum_valid and candidate_ratio <= ratio_max:
            augmented = candidate
            ratio = candidate_ratio
    return augmented, ratio


def mask_augmentation_settings(cfg):
    data = cfg._data
    return {
        "enabled": bool(data.get("mask_augment_enabled", False)),
        "probability": float(data.get("mask_augment_probability", 0.0)),
        "hole_count": list(data.get("mask_hole_count", [1, 1])),
        "hole_width_fraction": list(
            data.get("mask_hole_width_fraction", [0.02, 0.12])
        ),
        "hole_height_fraction": list(
            data.get("mask_hole_height_fraction", [0.03, 0.20])
        ),
        "band_probability": float(data.get("mask_band_probability", 0.0)),
        "dropout_ratio": list(data.get("mask_dropout_ratio", [0.05, 0.35])),
        "min_valid_ratio": float(data.get("mask_min_valid_ratio", 0.60)),
        "fill_mode": str(data.get("mask_fill_mode", "valid_mean")),
    }


def resize_mask(mask, shape):
    height, width = shape
    return cv2.resize(
        mask, (width, height), interpolation=cv2.INTER_NEAREST
    )
