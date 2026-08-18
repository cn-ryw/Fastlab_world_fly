# FASTLab World Fly 六节点架构图：准确性与发布 QA

## 1. 适用范围

本文只对应当前精简学术版六节点主图，不是项目所有功能、所有运行模式或所有异常分支的完整清单。

- 行为基线：Git commit f8ff396849e568ed21851991a64634819bec42c2（2026-08-16）。
- Canva design ID：DAHSij1Sid8。
- 规范画布：1920×1080 SVG；发布 PNG：3840×2160 RGB；发布 PDF：单页 960×540 pt。
- 图示范围：SO3 模式下的活动导航任务。
- SVG 中除三块项目原生截图外，所有模块、连线、箭头、文字与公式均为原生 SVG 矢量元素。
- 本图不把未画出的内容暗示成已验收能力，也不以插画代替真实项目素材。

当前交付物：

| 文件 | 角色 |
|---|---|
| docs/assets/architecture/fastlab-world-fly-system-architecture.svg | 规范源；含可检索文字、六节点拓扑和三块无损 crop |
| docs/assets/architecture/fastlab-world-fly-system-architecture.png | 2× 发布位图 |
| docs/assets/architecture/fastlab-world-fly-system-architecture.pdf | 单页发布 PDF |
| docs/assets/architecture/fastlab-world-fly-system-architecture.html | 只引用规范 SVG 的白底浏览器/打印壳 |
| docs/assets/architecture/fastlab-world-fly-system-architecture.QA.md | 本事实溯源与签核记录 |

### 1.1 明确的图外范围

下列内容存在于项目中，但本版为保持六节点主链清晰而没有画入；不得在验收时误称为图中可见：

- FPV、Drone (Easy)、Level 等手动控制旁路；
- preview、HUD、OSD、radar、FlightLogger；
- 离线标定采集、拟合、评测和性能数字；
- 端口、模型文件名、策略名、face 分片调度和浏览器性能 profile；
- 4 m 到达、T8L link-loss、collision、expiry 等详细异常状态机；
- 250 ms、200 Hz、速度/加速度/倾角等未在当前图面显示的控制数值。

这些事实可在项目文档和代码中查阅，但不属于本图的视觉断言。

## 2. 六节点主图事实溯源

### 2.1 节点

| 顺序 | 图中节点 | 图中保留的核心合同 | 代码或配置证据 |
|---:|---|---|---|
| 1 | 任务与导航会话 | 固定航点或 T8L 滚动航点；建立 Navigation Session；活动导航任务进入 SO3 自主闭环 | src/main.js:777-806,830-867；src/panorama-sensor.js:2771-2806 |
| 2 | 城市环境与全景采集 | CesiumJS 主视图与独立隐藏全景视图；冻结同一位姿后采集 6 个立方体视图 | src/main.js:1360-1387；src/cesium-world.js:431-533,1963-2015,2157-2356 |
| 3 | 原子全景观测 | WebGL ERP RGB 384×192；PerceptionFrame 将图像、采集位姿、actual/reference state 与 yaw 冻结成一个原子帧 | src/perception-frame.js:74-157；src/panorama-sensor.js:974-1015 |
| 4 | sim-to-sim 近似米制深度与局部规划 | DA360 pred_disp 经零偏置 scale-only 换算为近似深度；与 valid mask 一起进入 YOPO；72 个候选选最低代价 | scripts/da360_server.py:713-763,824-843；scripts/yopo_bridge.py:260-337 |
| 5 | 轨迹门禁与重构 | 核对身份、来源、时效与轨迹；无效候选不替换仍有效轨迹；应用时从实测状态重构完整时域 Poly5 | src/panorama-sensor.js:2125-2354；src/drone.js:808-879,974-996；src/yopo-trajectory.js:243-335 |
| 6 | SO3 控制与闭环动力学 | Poly5 参考进入 SO3 姿态/推力控制；固定步长积分与碰撞响应产生下一观测的状态反馈 | src/drone.js:1212-1306,2570-2714,2725-2830；src/fixed-step-scheduler.js:11-14 |

### 2.2 箭头拓扑

当前 SVG 有 6 条主闭环 connector 和 1 条独立 goal/request-join bus：

| 边 | 语义 | SVG path |
|---|---|---|
| 1 → 2 | 会话启动仿真闭环与采集 | M350 425 H382 |
| 2 → 3 | 冻结六面采集形成原子 ERP 观测 | M790 425 H822 |
| 3 → 4 | PerceptionFrame 组装为规划请求并进入组合 GPU 服务 | M1220 425 H1252 |
| 4 → 5 | 返回 axis-major 终端状态与轨迹时域 T | M1555 605 V690 |
| 5 → 6 | 合法候选重构为 Poly5，交给 SO3 与动力学 | M1260 810 H1228 |
| 6 → 2 | 实测位置、速度与姿态反馈到下一次冻结采集 | M600 690 V605 |
| 1 → 3 | 目标与会话标识仅在请求组装阶段合入 | M210 245 V216 H1025 V236 |

这 7 条线均为水平/垂直折线，箭头不穿过节点正文，彼此没有视觉交叉。

### 2.3 必须保留的语义边界

1. Goal 不属于 PerceptionFrame 的固有字段。PerceptionFrame 先冻结图像与状态，目标只在 frame.planningObservation(request.goal) 组装 /yopo/plan_full 请求时合入：src/perception-frame.js:147-157；src/panorama-sensor.js:1938-1947,2040-2055。
2. GPU 服务返回的不只是 9 维 axis-major endstate，还包括 traj_time T：scripts/combined_server.py:739-745；src/panorama-sensor.js:2322-2329。
3. 当前 Drone handoff 从应用时实测位置/速度与上一参考加速度重拟合完整时域 Poly5，initialTimeS 固定为 0；capture age 仅用于时效门禁，不执行 capture-time suffix fast-forward：src/drone.js:808-850。
4. 新候选在私有 tracker 中完成结构和完整区间极值校验，拒绝时不预先销毁仍有效旧轨迹：src/drone.js:829-879,974-996；src/yopo-trajectory.js:243-335。
5. 图中的闭环箭头只把动力学产生的实测位置、速度与姿态标为反馈量；控制器内部 reference state 仍会随下一 PerceptionFrame 一起冻结，但不是动力学实测量：src/drone.js:1460-1489；src/main.js:1370-1387。
6. 图中 D̂sim 是当前 sim-to-sim 链路的近似深度；运行时参数由外部标定文件确定，图面不声称跨场景或真实传感器适用性已经验证。

## 3. 图中数字与公式

当前六节点图只保留以下必要数字/符号：

| 图中内容 | 精确定义 | 证据 |
|---|---|---|
| 6 个立方体视图 | front/right/back/left/up/down 六方向；同一冻结位姿采集 | src/cesium-world.js:58-65,2157-2356 |
| ERP RGB 384×192 | 当前浏览器默认完整 ERP 尺寸 | src/panorama-sensor.js:101-102 |
| D̂sim = 1/(a·pdisp), b = 0 | 唯一允许部署的外部 scale-only 关系；非零外部 b 被拒绝 | scripts/da360_server.py:713-763,824-843 |
| 72 个候选轨迹 → 最低代价 | N=12×6×1；action_id=np.argmin(score_flat) | third_party/YOPO/config/traj_opt.yaml:32-45,68-71；scripts/yopo_bridge.py:327-337 |
| 终端状态 + 轨迹时域 T | axis-major [px,vx,ax, py,vy,ay, pz,vz,az] 与 traj_time | scripts/yopo_bridge.py:241-254,361-367；scripts/combined_server.py:739-745 |
| 完整时域 Poly5 | 每轴以初末 p/v/a 六个边界条件求五次多项式，应用时 initialTimeS=0 | src/yopo-trajectory.js:30-61,234-240；src/drone.js:808-850 |

图面没有写入 a 的数值、演示性能、规划频率或“已验证米制尺度”等超出当前证据边界的说法。

## 4. 原生截图溯源

### 4.1 源文件

| 源文件 | 源尺寸 | SHA-256 |
|---|---:|---|
| docs/assets/demo/poster-easy.jpg | 1280×720 | c1aaa0beab23a394272a8ebae5a649add8e43f038ff21dd8bbf81efba99c0681 |
| asset/display/20260703-005006.jpg | 1961×1176 | ee388c90cd698f0d9f5ed75a15481e7e2cce2692bb414cc7fe7b33fe1e89179e |

### 4.2 Crop 与内嵌结果

坐标均为半开区间 [x0,y0,x1,y1)。

| 用途 | crop | crop 尺寸 | SVG 显示尺寸 | 内嵌 PNG SHA-256 |
|---|---|---:|---:|---|
| CesiumJS 城市场景 | poster-easy.jpg [224,120,1082,603) | 858×483 | 350×197 | 363214028ca20338814c5e36ba8dad18e3b51a3fa9ee4bb7c4679e524e29c6c6 |
| ERP RGB | 20260703-005006.jpg [1644,768,1948,920) | 304×152 | 340×170 | 5a645f08a4a195b53427745785c9ac35c08a9ae53f58949b180ce02da6dbb577 |
| DA360 相对视差着色预览 | 20260703-005006.jpg [1644,926,1948,1078) | 304×152 | 230×115 | a08f61c6708c53d5f3e15d8175fffb1547abc91d0e3b66503e250911bbf7c60c |

验收结论：

- 三个内嵌 PNG 解码后均与对应源 JPG 的 RGB crop 逐像素相同。
- 只做矩形 crop 与版面缩放；未锐化、修补、扩图、内容感知填充或生成内部像素。
- ERP 与 DA360 显示框保持 2:1；城市场景显示框与 858:483 原 crop 比例近似一致。
- SVG metadata 和每个 image 的 data-source/data-crop 保留机器可读溯源。
- 可见页脚同时列出两个源文件名和 CesiumJS/Google Tiles attribution。
- DA360 图只能解释为“相对视差着色预览”；控制主链使用的是带 sim-to-sim scale-only 换算的近似深度。

## 5. 发布文件与导出

### 5.1 当前发布哈希

| 文件 | SHA-256 |
|---|---|
| fastlab-world-fly-system-architecture.svg | b735cbdb600a1dcf529b33035d800274afb85e2f95a837cff23fdb15686cd61e |
| fastlab-world-fly-system-architecture.png | dbd5ec07eb4c21fd00a0ff0cd0ebfef724d7f35bd53933aa786c3992bfc2a893 |
| fastlab-world-fly-system-architecture.pdf | 08db793b8a5d3b9805ad24b701a9c408caae93c262a27a8438437229aa7910d6 |
| fastlab-world-fly-system-architecture.html | 76cb062143a3d64dfc9013a826443c7ffe69edb3c7a6e17fed004cac9b2ef121 |

### 5.2 结构与尺寸

| 文件 | 验收结果 |
|---|---|
| SVG | XML 有效；width=1920、height=1080、viewBox=0 0 1920 1080；6 个 aria-label 节点、3 个 image |
| PNG | 3840×2160、RGB、无 alpha、Pillow info={}；外边界全部为 #FFFFFF |
| PDF | 1 页、960×540 pt、PDF 1.4；无 JavaScript、Custom Metadata 或 Metadata Stream |
| HTML | 白底；一个 img 引用规范 SVG；16:9 打印页；有描述性 alt |

### 5.3 可复现导出命令

以下命令写入 /tmp，不覆盖发布文件：

    ARCH_REPO="$(pwd)"
    google-chrome \
      --headless=new \
      --no-sandbox \
      --disable-gpu \
      --allow-file-access-from-files \
      --hide-scrollbars \
      --window-size=3840,2160 \
      --force-device-scale-factor=1 \
      --screenshot=/tmp/fastlab-world-fly-system-architecture.png \
      "file://$ARCH_REPO/docs/assets/architecture/fastlab-world-fly-system-architecture.html"

    google-chrome \
      --headless=new \
      --no-sandbox \
      --disable-gpu \
      --allow-file-access-from-files \
      --no-pdf-header-footer \
      --print-to-pdf=/tmp/fastlab-world-fly-system-architecture.pdf \
      "file://$ARCH_REPO/docs/assets/architecture/fastlab-world-fly-system-architecture.html"

不同 Chrome 进程可能产生字形抗锯齿或 PDF 时间戳差异，因此发布哈希用于锁定本次交付，不承诺跨进程 byte-identical。重导验收以尺寸、白底、结构、文字、crop 和全图目视一致为准。

## 6. 可执行验收

所有命令从仓库根目录执行。

### 6.1 XML、画布、节点与图片数

    xmllint --noout docs/assets/architecture/fastlab-world-fly-system-architecture.svg
    xmllint --xpath \
      'concat("root=", local-name(/*), " width=", /*/@width, " height=", /*/@height, " viewBox=", /*/@viewBox, " images=", count(//*[local-name()="image"]), " nodes=", count(//*[@aria-label]))' \
      docs/assets/architecture/fastlab-world-fly-system-architecture.svg

预期：

    root=svg width=1920 height=1080 viewBox=0 0 1920 1080 images=3 nodes=6

### 6.2 PNG 尺寸、纯白边界和真实 chunk

    python3 - <<'PY'
    from pathlib import Path
    from PIL import Image
    import struct

    path = Path("docs/assets/architecture/fastlab-world-fly-system-architecture.png")
    with Image.open(path) as image:
        assert image.size == (3840, 2160)
        assert image.mode == "RGB"
        assert image.info == {}
        white = (255, 255, 255)
        borders = [
            image.crop((0, 0, image.width, 1)),
            image.crop((0, image.height - 1, image.width, image.height)),
            image.crop((0, 0, 1, image.height)),
            image.crop((image.width - 1, 0, image.width, image.height)),
        ]
        assert all(
            all(pixel == white for pixel in border.getdata())
            for border in borders
        )

    data = path.read_bytes()
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    position = 8
    chunks = []
    while position < len(data):
        length = struct.unpack(">I", data[position:position + 4])[0]
        kind = data[position + 4:position + 8].decode("latin-1")
        payload = data[position + 8:position + 8 + length]
        chunks.append((kind, payload))
        position += 12 + length
        if kind == "IEND":
            break
    assert position == len(data)
    names = [kind for kind, _ in chunks]
    assert names.count("IHDR") == 1
    assert names.count("IDAT") == 299
    assert names.count("IEND") == 1
    assert not {"caBX", "iTXt", "tEXt", "zTXt", "eXIf"}.intersection(names)
    print("PNG OK:", (3840, 2160), "RGB", {"IHDR": 1, "IDAT": 299, "IEND": 1})
    PY

### 6.3 三个原生 crop 逐像素核对

    python3 - <<'PY'
    from pathlib import Path
    from PIL import Image, ImageChops
    import base64
    import hashlib
    import io
    import xml.etree.ElementTree as ET

    repo = Path(".").resolve()
    svg = repo / "docs/assets/architecture/fastlab-world-fly-system-architecture.svg"
    ns = {
        "svg": "http://www.w3.org/2000/svg",
        "xlink": "http://www.w3.org/1999/xlink",
    }
    root = ET.parse(svg).getroot()
    images = root.findall(".//svg:image", ns)
    assert len(images) == 3

    for index, element in enumerate(images, 1):
        href = element.attrib["href"]
        assert href == element.attrib["{http://www.w3.org/1999/xlink}href"]
        assert href.startswith("data:image/png;base64,")
        source = repo / element.attrib["data-source"]
        crop = tuple(map(int, element.attrib["data-crop"].split(",")))
        expected = Image.open(source).convert("RGB").crop(crop)
        raw = base64.b64decode(href.split(",", 1)[1], validate=True)
        embedded = Image.open(io.BytesIO(raw)).convert("RGB")
        assert embedded.size == expected.size
        assert ImageChops.difference(embedded, expected).getbbox() is None
        print(
            index,
            source.relative_to(repo),
            crop,
            embedded.size,
            hashlib.sha256(raw).hexdigest(),
        )
    PY

### 6.4 C2PA/JUMBF、生成式痕迹与敏感路径

不能直接对 SVG 全文件执行简单字符串搜索，因为 base64 高熵 payload 可能随机命中。下列检查解析 XML 后排除 data URI 字符串，同时检查三个内嵌 PNG 的实际 chunk；发布 PNG 另由 6.2 检查 caBX 和文本/EXIF chunk。

    python3 - <<'PY'
    from pathlib import Path
    import base64
    import re
    import subprocess
    import xml.etree.ElementTree as ET

    base = Path("docs/assets/architecture/fastlab-world-fly-system-architecture")
    root = ET.parse(base.with_suffix(".svg")).getroot()
    visible = []
    embedded = []
    for element in root.iter():
        visible.extend((element.tag, element.text or "", element.tail or ""))
        for key, value in element.attrib.items():
            local = key.rsplit("}", 1)[-1]
            if local in {"href", "src"} and value.startswith("data:"):
                embedded.append(value)
            else:
                visible.extend((key, value))

    documents = {
        "SVG(non-data)": "\n".join(visible).encode(),
        "HTML": base.with_suffix(".html").read_bytes(),
        "PDF strings": subprocess.run(
            ["strings", "-a", "-n", "8", str(base.with_suffix(".pdf"))],
            check=True,
            stdout=subprocess.PIPE,
        ).stdout,
    }
    patterns = {
        "C2PA/JUMBF":
            rb"c2pa|jumbf|content credentials|manifest[_ -]?store|claim[_ -]?generator",
        "generative provenance":
            rb"midjourney|dall[- ]?e|stable[ -]?diffusion|openai image|negative_prompt|prompt:",
        "sensitive path/token":
            rb"/(?:home|Users)/|[A-Za-z]:\\[A-Za-z]|sk-[A-Za-z0-9]|ionToken=",
    }
    for label, pattern in patterns.items():
        expression = re.compile(pattern, re.IGNORECASE)
        for owner, data in documents.items():
            assert not expression.search(data), (label, owner)

    def png_chunks(raw):
        assert raw[:8] == b"\x89PNG\r\n\x1a\n"
        position = 8
        chunks = []
        while position < len(raw):
            length = int.from_bytes(raw[position:position + 4], "big")
            kind = raw[position + 4:position + 8].decode("latin-1")
            payload = raw[position + 8:position + 8 + length]
            chunks.append((kind, payload))
            position += 12 + length
            if kind == "IEND":
                break
        assert position == len(raw)
        return chunks

    unique = list(dict.fromkeys(embedded))
    assert len(unique) == 3
    c2pa = re.compile(patterns["C2PA/JUMBF"], re.IGNORECASE)
    for uri in unique:
        raw = base64.b64decode(uri.split(",", 1)[1], validate=True)
        chunks = png_chunks(raw)
        names = [name for name, _ in chunks]
        assert not {"caBX", "iTXt", "tEXt", "zTXt", "eXIf"}.intersection(names)
        assert not c2pa.search(b"\n".join(payload for _, payload in chunks))

    assert not c2pa.search(base.with_suffix(".pdf").read_bytes())
    print("C2PA/JUMBF, generative provenance and sensitive path checks OK")
    PY

### 6.5 PDF 单页

    pdfinfo docs/assets/architecture/fastlab-world-fly-system-architecture.pdf

必须同时满足：

- Pages: 1
- Page size: 960 x 540 pts
- PDF version: 1.4
- Custom Metadata: no
- Metadata Stream: no
- JavaScript: no
- Encrypted: no

### 6.6 本地链接与素材引用

    python3 - <<'PY'
    from pathlib import Path
    from html.parser import HTMLParser
    import xml.etree.ElementTree as ET

    repo = Path(".").resolve()
    base = repo / "docs/assets/architecture"
    checked = []

    def check(owner, reference, repository_relative=False):
        if not reference or reference.startswith(
            ("data:", "#", "http://", "https://", "mailto:")
        ):
            return
        assert not reference.startswith(("file:", "/")), (owner, reference)
        target = (
            repo / reference
            if repository_relative
            else owner.parent / reference
        ).resolve()
        assert repo == target or repo in target.parents
        assert target.exists(), (owner, reference)
        checked.append((owner.relative_to(repo), reference, target.relative_to(repo)))

    class Parser(HTMLParser):
        def __init__(self, owner):
            super().__init__()
            self.owner = owner

        def handle_starttag(self, tag, attrs):
            attrs = dict(attrs)
            for key in ("href", "src"):
                if key in attrs:
                    check(self.owner, attrs[key])

    html = base / "fastlab-world-fly-system-architecture.html"
    parser = Parser(html)
    parser.feed(html.read_text(encoding="utf-8"))

    svg = base / "fastlab-world-fly-system-architecture.svg"
    root = ET.parse(svg).getroot()
    for element in root.iter():
        for key in (
            "href",
            "{http://www.w3.org/1999/xlink}href",
            "data-source",
        ):
            value = element.attrib.get(key)
            if value:
                check(svg, value, repository_relative=(key == "data-source"))

    print("local links OK:", checked)
    PY

### 6.7 关键文字与六节点拓扑

    python3 - <<'PY'
    from pathlib import Path
    import re
    import xml.etree.ElementTree as ET

    path = Path("docs/assets/architecture/fastlab-world-fly-system-architecture.svg")
    root = ET.parse(path).getroot()
    text = re.sub(
        r"\s+",
        " ",
        " ".join(value.strip() for value in root.itertext() if value.strip()),
    )

    required = [
        "FASTLab World Fly 自主导航闭环仿真架构",
        "图示范围 · SO3 模式下的活动导航任务",
        "目标与会话标识（在请求组装阶段合入）",
        "终端状态 + 轨迹时域 T",
        "实测位置、速度与姿态反馈",
        "目标仅在请求组装阶段合入",
        "近似深度 + valid mask",
        "YOPO · 72 个候选轨迹 → 最低代价",
        "无效候选不替换当前有效轨迹",
        "实测状态 → 完整时域 Poly5",
        "采集延迟仅用于时效判定",
        "该零偏置换算用于当前 sim-to-sim 链路",
        "运行时参数由外部标定文件确定",
        "跨场景与真实传感器适用性尚未验证",
    ]
    for item in required:
        assert item in text, item

    superseded = [
        "图示范围 · SO3 模式 + 有效目标",
        "实测与参考状态反馈",
        "YOPO · 72 candidates → 最低代价",
        "仓库发布基线未通过自动精度门禁",
        "不代表跨场景或真实传感器的米制精度",
        "DAHSiuzplC4",
    ]
    for item in superseded:
        assert item not in text, item

    formulas = [
        re.sub(r"\s+", " ", "".join(element.itertext())).strip()
        for element in root.iter()
        if "formula" in element.attrib.get("class", "")
    ]
    assert formulas == ["D̂sim = 1/(a·pdisp), b = 0"], formulas

    namespace = {"svg": "http://www.w3.org/2000/svg"}
    nodes = [
        element.attrib["aria-label"]
        for element in root.findall(".//svg:g[@aria-label]", namespace)
    ]
    assert nodes == [
        "任务与导航会话",
        "城市环境与全景采集",
        "原子全景观测与请求组装",
        "尺度标定深度与局部规划",
        "控制门禁与完整时域 Poly5",
        "SO3 控制、动力学与状态反馈",
    ]

    connectors = [
        element.attrib["d"]
        for element in root.findall('.//svg:path[@class="connector"]', namespace)
    ]
    assert connectors == [
        "M350 425 H382",
        "M790 425 H822",
        "M1220 425 H1252",
        "M1555 605 V690",
        "M1260 810 H1228",
        "M600 690 V605",
    ]
    buses = [
        element.attrib["d"]
        for element in root.findall('.//svg:path[@class="bus"]', namespace)
    ]
    assert buses == ["M210 245 V216 H1025 V236"]
    print("six-node topology and key text OK")
    PY

### 6.8 发布哈希

    sha256sum \
      docs/assets/architecture/fastlab-world-fly-system-architecture.svg \
      docs/assets/architecture/fastlab-world-fly-system-architecture.png \
      docs/assets/architecture/fastlab-world-fly-system-architecture.pdf \
      docs/assets/architecture/fastlab-world-fly-system-architecture.html

### 6.9 Git whitespace

QA 文件当前是新增文件，因此同时运行仓库差异检查和新增文件自身的 no-index 检查：

    git diff --check
    git diff --no-index --check /dev/null \
      docs/assets/architecture/fastlab-world-fly-system-architecture.QA.md \
      >/dev/null

第二条命令因“文件不同”可返回状态 1；验收标准是 stderr/终端没有 trailing whitespace、space before tab 或 blank line at EOF 报告。

## 7. 人工视觉验收

- [x] 画布纯白，无纹理、渐变、阴影、徽章、水印或生成式装饰。
- [x] 六个方角节点清晰：4 个上排、2 个下排。
- [x] 主闭环和独立 goal/request-join bus 方向正确，箭头无交叉且不穿过正文。
- [x] Goal 明确在请求组装阶段合入，没有被画成 PerceptionFrame 的内生字段。
- [x] GPU 输出明确包含终端状态与轨迹时域 T。
- [x] Gate 明确写出“无效候选不替换当前有效轨迹”和“采集延迟仅用于时效判定”。
- [x] 应用时轨迹重构明确为实测状态到完整时域 Poly5，没有 capture-time suffix 说法。
- [x] 三个 raster 都是项目原生截图 crop；ERP 与 DA360 预览保持 2:1。
- [x] DA360 图注为相对视差着色预览；主链只称 sim-to-sim 近似米制深度。
- [x] 标定边界明确说明运行时参数来自外部标定文件，且跨场景与真实传感器适用性尚未验证。
- [x] 页脚可见原生文件名以及 CesiumJS/Google Photorealistic 3D Tiles attribution。
- [x] 图面没有旧版诊断/离线泳道、性能卡、端口、旧哈希或未画分支的断言。

## 8. 最终签核记录

执行日期：2026-08-17（Asia/Shanghai）。

环境：

- Google Chrome 150.0.7871.114
- libxml2 2.9.14
- Python 3.12.3
- Pillow 10.2.0
- pdfinfo 24.02.0

| 检查 | 最终结果 |
|---|---|
| XML / SVG 结构 | 通过；1920×1080，viewBox 0 0 1920 1080，6 节点、3 images |
| PNG 尺寸与纯白 | 通过；3840×2160 RGB，info={}，四条外边界均为 #FFFFFF |
| PNG chunk | 通过；IHDR×1、IDAT×299、IEND×1，无 caBX/文本/EXIF chunk |
| 原生 crop | 通过；三个内嵌 PNG 与声明的原始 JPG crop 逐像素相同 |
| SVG/HTML/PDF C2PA/JUMBF | 通过；SVG 排除 data URI 后无标记，三个内嵌 PNG chunk 无标记，HTML/PDF 无标记 |
| PDF | 通过；1 页，960×540 pt，PDF 1.4，无 JavaScript/Custom Metadata/Metadata Stream |
| HTML / 本地链接 | 通过；HTML 只引用规范 SVG；三个 data-source 均存在且未逃逸仓库 |
| 关键文字 / 公式 | 通过；goal join、T、实测反馈、近似深度、72 个候选轨迹、Gate、Poly5 与标定边界均存在；上一版措辞均不存在 |
| 拓扑 | 通过；6 条闭环 connector + 1 条 goal/request-join bus，顺序和坐标与第 2.2 节一致 |
| 发布哈希 | 通过；与第 5.1 节一致 |
| 人工视觉 | 通过；见第 7 节 |
| git diff --check | 通过；无 whitespace error |

本记录只签核上述四个发布产物和六节点图面，不把图外范围升级为当前图的可见声明。
