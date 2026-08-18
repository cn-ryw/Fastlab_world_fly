"""Static contract for the browser-side Cesium token setup flow."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
INDEX = (ROOT / "index.html").read_text(encoding="utf-8")
MAIN = (ROOT / "src" / "main.js").read_text(encoding="utf-8")
TOKEN_MODULE = (ROOT / "src" / "cesium-token.js").read_text(encoding="utf-8")


def test_missing_token_has_an_in_app_password_form():
    for element_id in (
        "cesium-token-overlay",
        "cesium-token-form",
        "cesium-token-input",
        "cesium-token-cancel",
        "cesium-token-error",
        "cesium-token-change",
    ):
        assert f'id="{element_id}"' in INDEX
        assert f"'{element_id}'" in MAIN
    assert 'type="password"' in INDEX
    assert "await requestCesiumIonToken()" in MAIN
    assert "persistAfterValidation" in MAIN
    assert "storeCesiumIonToken(tokenSetup.token)" in MAIN
    assert "requestCesiumIonToken(true)" in MAIN
    assert "startTilesMode(tokenSetup)" in MAIN
    assert "tokenSetupOverride || await requestCesiumIonToken()" in MAIN


def test_token_transport_is_storage_only():
    assert "localStorage" in TOKEN_MODULE
    assert "URLSearchParams" not in TOKEN_MODULE
    assert "location" not in TOKEN_MODULE
    assert "ionToken=" not in TOKEN_MODULE
