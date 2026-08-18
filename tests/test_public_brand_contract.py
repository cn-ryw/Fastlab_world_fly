"""Public FASTLab branding and legacy runtime compatibility contract."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def test_public_surfaces_use_fastlab_world_fly_brand():
    expected = {
        "README.md": "# FASTLab World Fly",
        "docs/USER_GUIDE.zh-CN.md": "# FASTLab World Fly 中文使用指南",
        "docs/ARCHITECTURE.zh-CN.md": "# FASTLab World Fly 系统架构",
        "CONTRIBUTING.md": "感谢参与 FASTLab World Fly",
        "NOTICE": "FASTLab World Fly\n",
        "index.html": "<title>FASTLab World Fly</title>",
    }
    for relative, marker in expected.items():
        assert marker in read(relative), f"{relative} is missing the public FASTLab brand"

    html = read("index.html")
    main = read("src/main.js")
    assert "Start FASTLab World Fly" in html
    assert "Start FASTLab World Fly" in main
    assert "Google 3D Tiles Flight</title>" not in html
    assert "Start Google 3D Tiles Flight" not in html
    assert "Start Google 3D Tiles Flight" not in main
    assert "FASTLab World Fly running at" in read("scripts/serve.py")
    assert "FASTLab World Fly container server listening" in read("scripts/server.js")


def test_public_assets_and_repository_links_use_new_slug():
    new_assets = (
        "docs/assets/demo/fastlab-world-fly-demo-overview.gif",
        "docs/assets/architecture/fastlab-world-fly-closed-loop-architecture-20260818.jpg",
        "docs/assets/architecture/fastlab-world-fly-system-architecture.svg",
        "docs/assets/architecture/fastlab-world-fly-system-architecture.png",
        "docs/assets/architecture/fastlab-world-fly-system-architecture.pdf",
        "docs/assets/architecture/fastlab-world-fly-system-architecture.html",
        "docs/assets/architecture/fastlab-world-fly-system-architecture.QA.md",
    )
    old_assets = (
        "docs/assets/demo/mindcloud-demo-overview.gif",
        "docs/assets/architecture/mindcloud-system-architecture.svg",
        "docs/assets/architecture/mindcloud-system-architecture.png",
        "docs/assets/architecture/mindcloud-system-architecture.pdf",
        "docs/assets/architecture/mindcloud-system-architecture.html",
        "docs/assets/architecture/mindcloud-system-architecture.QA.md",
    )
    assert all((ROOT / relative).is_file() for relative in new_assets)
    assert all(not (ROOT / relative).exists() for relative in old_assets)

    readme = read("README.md")
    assert "github.com/cn-ryw/Fastlab_world_fly" in readme
    assert "docs/assets/demo/fastlab-world-fly-demo-overview.gif" in readme
    assert (
        "docs/assets/architecture/fastlab-world-fly-closed-loop-architecture-20260818.jpg"
        in readme
    )
    assert "fastlab-world-fly-system-architecture" in readme
    assert "github.com/superboySB/MindCloud_World_Fly" in readme

    issue_config = read(".github/ISSUE_TEMPLATE/config.yml")
    assert "github.com/cn-ryw/Fastlab_world_fly/security/advisories/new" in issue_config
    assert "github.com/cn-ryw/MindCloud_World_Fly" not in issue_config

    architecture_svg = read(
        "docs/assets/architecture/fastlab-world-fly-system-architecture.svg"
    )
    assert 'xmlns="urn:fastlab-world-fly:architecture-diagram:1"' in architecture_svg
    assert "mindcloud.example" not in architecture_svg

    architecture_guide = read("docs/ARCHITECTURE.zh-CN.md")
    assert (
        "assets/architecture/fastlab-world-fly-closed-loop-architecture-20260818.jpg"
        in architecture_guide
    )


def test_internal_legacy_identifiers_remain_compatible():
    assert "MINDCLOUD_WEB_HOST" in read("start-all.sh")
    assert "mindcloud_cesium_ion_token" in read("src/cesium-token.js")
    assert "application/x-mindcloud-rgba8" in read("src/panorama-sensor.js")
    assert "application/x-mindcloud-rgba8" in read("scripts/da360_server.py")
    assert "/opt/mindcloud-da360" in read("Dockerfile.da360")


def test_google_tiles_attribution_is_explicitly_on_screen():
    world = read("src/cesium-world.js")
    html = read("index.html")
    assert "showCreditsOnScreen: true" in world
    assert "above the bottom-edge Cesium/Google credits" in html
    assert "bottom: 48px" in html
