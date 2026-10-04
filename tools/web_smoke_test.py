"""Headless smoke test of the offline web app: loads it, collects console errors, exercises the
controls and every dashboard tab, and saves screenshots to outputs/web/screens/.

Usage: python tools/web_smoke_test.py [--dark]
Author: Anwesh Ajitabh Dash
"""
from __future__ import annotations

import sys
from pathlib import Path

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[1]
PAGE = ROOT / "outputs" / "web" / "miyota_82S0_twin_offline.html"
OUT = ROOT / "outputs" / "web" / "screens"


def main(dark: bool = False, width: int = 1500, height: int = 920, tag: str = "") -> list[str]:
    OUT.mkdir(parents=True, exist_ok=True)
    errors: list[str] = []
    with sync_playwright() as p:
        b = p.chromium.launch(args=["--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader"])
        ctx = b.new_context(viewport={"width": width, "height": height}, color_scheme="dark" if dark else "light")
        pg = ctx.new_page()
        pg.on("console", lambda m: errors.append(f"{m.type}: {m.text}") if m.type in ("error", "warning") else None)
        pg.on("pageerror", lambda e: errors.append(f"pageerror: {e}"))
        pg.goto(PAGE.as_uri())
        pg.wait_for_timeout(3500)
        sfx = ("_dark" if dark else "") + tag
        pg.screenshot(path=str(OUT / f"01_live{sfx}.png"))
        pg.click("#t-labels")
        pg.click("#t-arrows")
        pg.click("#btn-next")
        pg.wait_for_timeout(400)
        pg.click("#btn-next")
        pg.wait_for_timeout(600)
        pg.screenshot(path=str(OUT / f"02_event_step{sfx}.png"))
        pg.click("#t-skeleton")
        pg.fill("#explode", "70")
        pg.dispatch_event("#explode", "input")
        pg.click("#btn-view-iso")
        pg.wait_for_timeout(800)
        pg.screenshot(path=str(OUT / f"03_exploded{sfx}.png"))
        for tab in ("esc", "time", "exp", "val", "par", "about"):
            pg.click(f"#tab-{tab}")
            pg.wait_for_timeout(1500)
            pg.screenshot(path=str(OUT / f"tab_{tab}{sfx}.png"))
        pg.click("#tab-live")
        pg.click('[data-speed="3600"]')
        pg.wait_for_timeout(2500)
        pg.screenshot(path=str(OUT / f"04_fast{sfx}.png"))
        b.close()
    return errors


if __name__ == "__main__":
    dark = "--dark" in sys.argv
    errs = main(dark=dark)
    if "--phone" in sys.argv:
        errs += main(dark=dark, width=400, height=860, tag="_phone")
    print("\n".join(errs) if errs else "no console errors")
