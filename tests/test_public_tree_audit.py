import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import audit_public_tree  # noqa: E402


def test_public_tree_has_no_forbidden_material():
    assert audit_public_tree.audit() == []


def test_audit_rules_detect_private_material_without_echoing_it():
    sample = "token_value = '" + "eyJabcdefghijk" + ".abcdefghijk" + ".abcdefghijk'"
    assert any(pattern.search(sample) for _, pattern in audit_public_tree.CONTENT_RULES)
