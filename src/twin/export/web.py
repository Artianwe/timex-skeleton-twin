"""Build the single-file web app (3-D viewer + engineering dashboard).

Two variants from the same sources (web/app/*):
  * outputs/web/miyota_82S0_twin.html          - libraries from CDN (for publishing/sharing)
  * outputs/web/miyota_82S0_twin_offline.html  - libraries inlined from web/vendor (works with
                                                 no network, open by double-click)
Data (web/data/results.json, web/data/parts.json) is embedded, so no server is needed.
"""
from __future__ import annotations

from pathlib import Path

from ..params import ROOT

APP = ROOT / "web" / "app"
VENDOR = ROOT / "web" / "vendor"
CDN = [
    ("https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js", "https://cdn.jsdelivr.net/npm/three@0.128.0/build/three.min.js"),
    ("https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js", None),
    ("https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/renderers/CSS2DRenderer.js", None),
    ("https://cdnjs.cloudflare.com/ajax/libs/plotly.js/2.35.2/plotly-basic.min.js", "https://cdn.jsdelivr.net/npm/plotly.js-basic-dist-min@2.35.2/plotly-basic.min.js"),
]
VENDOR_FILES = ["three-r128.min.js", "OrbitControls-r128.js", "CSS2DRenderer-r128.js", "plotly-basic-2.35.2.min.js"]
JS_ORDER = ["sim.js", "viewer.js", "dashboard.js", "main.js"]


def _safe_json(text: str) -> str:
    # JSON embedded in <script type="application/json"> must not contain "</script"
    return text.replace("</", "<\\/")


def build(results: Path | None = None, geo: Path | None = None, out_dir: Path | None = None) -> dict:
    results = results or ROOT / "web" / "data" / "results.json"
    geo = geo or ROOT / "web" / "data" / "parts.json"
    out_dir = out_dir or ROOT / "outputs" / "web"
    out_dir.mkdir(parents=True, exist_ok=True)
    html = (APP / "index.html").read_text(encoding="utf-8")
    css = (APP / "style.css").read_text(encoding="utf-8")
    js = "\n".join((APP / f).read_text(encoding="utf-8") for f in JS_ORDER)
    html = html.replace("/*__CSS__*/", css).replace("/*__JS__*/", js)
    html = html.replace("/*__DATA__*/", _safe_json(results.read_text(encoding="utf-8")))
    html = html.replace("/*__GEO__*/", _safe_json(geo.read_text(encoding="utf-8")))
    cdn_tags = "\n".join(
        f'<script src="{u}"></script>' if not fb else
        f'<script src="{u}"></script>\n<script>if(!window.{"THREE" if "three" in u else "Plotly"}){{document.write(\'<script src="{fb}"><\\/script>\')}}</script>'
        for u, fb in CDN)
    vendor_tags = "\n".join(f"<script>{(VENDOR / f).read_text(encoding='utf-8')}</script>" for f in VENDOR_FILES)
    page = html.replace("<!--__LIBS__-->", cdn_tags)
    offline = html.replace("<!--__LIBS__-->", vendor_tags)
    p1 = out_dir / "miyota_82S0_twin.html"
    p1.write_text(page, encoding="utf-8")
    full = ("<!doctype html>\n<html lang=\"en\">\n<head>\n<meta charset=\"utf-8\">\n"
            "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1, viewport-fit=cover\">\n"
            "</head>\n<body>\n" + offline + "\n</body>\n</html>\n")
    p2 = out_dir / "miyota_82S0_twin_offline.html"
    p2.write_text(full, encoding="utf-8")
    return {"page": p1, "offline": p2, "size_page": p1.stat().st_size, "size_offline": p2.stat().st_size}


if __name__ == "__main__":
    print(build())
