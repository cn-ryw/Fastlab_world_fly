"""Readable, hash-free contracts for selectable YOPO strategies."""

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_two_named_yopo_strategies_keep_baseline_as_default():
    manifest = json.loads(
        (ROOT / "dependencies.versions.json").read_text(encoding="utf-8")
    )
    strategies = manifest["yopo_strategies"]
    assert set(strategies) == {"baseline", "d70_h30_epoch30"}
    assert strategies["baseline"]["checkpoint_name"] == "epoch10.pth"
    assert strategies["baseline"]["config"] == (
        "x5_cruise15_18m_a12_mask_wc3.yaml"
    )
    assert strategies["d70_h30_epoch30"]["checkpoint_name"] == "epoch30.pth"
    assert strategies["d70_h30_epoch30"]["config"] == (
        "d70_h30_cruise15_recovery.yaml"
    )


def test_launcher_exposes_explicit_strategy_selection_and_identity():
    source = (ROOT / "start-all.sh").read_text(encoding="utf-8")
    assert 'MINDCLOUD_YOPO_STRATEGY:-baseline' in source
    assert '--yopo-strategy "$YOPO_STRATEGY"' in source
    assert 'MINDCLOUD_YOPO_STRATEGY=$YOPO_STRATEGY' in source
    assert 'YOPO_MODEL_PATH=$YOPO_CONTAINER_MODEL' in source


def test_d70_profile_preserves_deployment_contract():
    profile = (
        ROOT / "third_party/YOPO/config/d70_h30_cruise15_recovery.yaml"
    ).read_text(encoding="utf-8")
    required = (
        "depth_max_m: 70.0",
        "radio_range: 15.0",
        "velocity: 16.0",
        "vel_max_train: 16.0",
        "acc_max_train: 12.0",
        "safety_loss_eval_points: 100",
        "wc: 3.0",
        "d0: 1.20",
        "r: 0.60",
    )
    for value in required:
        assert value in profile
