#!/usr/bin/env python3
"""Request raw pred_disp from the DA360 /depth/raw endpoint and save as .npz.

Usage:
    python scripts/request_da360_raw.py <input.jpg> [--output out.npz] [--url http://127.0.0.1:5688/depth/raw]
"""
import argparse, io, json, os, sys, time
import numpy as np
import requests
from PIL import Image

DEFAULT_URL = os.environ.get("DA360_URL", "http://127.0.0.1:5688/depth/raw")


def main():
    parser = argparse.ArgumentParser(description="Request DA360 /depth/raw and save .npz")
    parser.add_argument("input", help="Input ERP RGB image (JPEG/PNG)")
    parser.add_argument("--output", "-o", default=None, help="Output .npz path (default: <input>_raw.npz)")
    parser.add_argument("--url", default=DEFAULT_URL, help=f"DA360 raw endpoint (default: {DEFAULT_URL})")
    args = parser.parse_args()

    img = Image.open(args.input).convert("RGB")
    print(f"Input: {args.input}  {img.size[0]}x{img.size[1]}")

    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=92)
    started = time.time()

    resp = requests.post(args.url, data=buf.getvalue(),
                         headers={"Content-Type": "image/jpeg"}, timeout=60)
    elapsed = (time.time() - started) * 1000
    if resp.status_code != 200:
        print(f"ERROR HTTP {resp.status_code}: {resp.text}", file=sys.stderr)
        sys.exit(1)

    out_path = args.output or os.path.splitext(args.input)[0] + "_raw.npz"
    with open(out_path, "wb") as f:
        f.write(resp.content)

    data = np.load(io.BytesIO(resp.content), allow_pickle=True)
    md = json.loads(str(data["metadata_json"]))
    print(f"Model: {md['model']}  Input: {md['width']}x{md['height']}")
    print(f"Latency: {elapsed:.1f} ms  Size: {len(resp.content)} bytes")
    print(f"pred_disp: {data['pred_disp'].dtype} {data['pred_disp'].shape}  [{data['pred_disp'].min():.4f}, {data['pred_disp'].max():.4f}]")
    print(f"Saved: {out_path}")


if __name__ == "__main__":
    main()
