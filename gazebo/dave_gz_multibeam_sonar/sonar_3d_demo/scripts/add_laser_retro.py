#!/usr/bin/env python3
"""Add <laser_retro> tags to every model.sdf in ~/blender_models/.

Phase B of the material reflectivity plan. Idempotent: skips files
that already have a <laser_retro> tag. Inserts the tag inside each
<visual> block (must have exactly one visual per file — fails loudly
otherwise).

Reflectivity values are starting placeholders based on acoustic-impedance
ratios; they get calibrated against wetlab data in Phase C.
"""

import re
import sys
from pathlib import Path

# Starting reflectivity values per model.
# Phase C will refit these against wl_wetlab_apr22 data.
RETRO_VALUES = {
    # Metal targets — reference material
    "circle_metal": 1.0,
    "square_metal": 1.0,
    "triangle_metal": 1.0,
    "metal_board": 1.0,
    # Wood targets
    "circle_wood": 0.25,
    "square_wood": 0.25,
    "triangle_wood": 0.25,
    # PETG (3D-printed) targets
    "circle_petg": 0.10,
    "square_petg": 0.10,
    "triangle_petg": 0.10,
    # Other
    "brick": 0.40,
    "float": 0.05,
    # NOTE: ~/blender_models/wetlab_tank is NOT the tank used by
    # sonar_3d_demo. The actual tank model is WL_wetlab_tank, which
    # lives in src/dave/models/dave_object_models/description/.
    # Per the material_reflectivity_plan §5 ("tank walls flood noise floor"
    # mitigation), keep the live tank UNTAGGED so walls floor at mu.
    # Revisit if Phase C calibration needs wall returns.
}

VISUAL_CLOSE_PATTERN = re.compile(r"(\s*)</visual>")


def patch_sdf(sdf_path: Path, retro: float) -> str:
    """Return one of: 'added', 'skipped', 'multi-visual', 'no-visual'."""
    text = sdf_path.read_text()

    if "<laser_retro>" in text:
        return "skipped"

    visual_count = text.count("<visual ")
    if visual_count == 0:
        return "no-visual"
    if visual_count > 1:
        return "multi-visual"

    # Insert <laser_retro> just before </visual>, indented to match
    # the existing block.
    def insert(match: re.Match) -> str:
        indent = match.group(1)
        # Two-space deeper indent for the new child element
        inner_indent = indent + "  "
        return f"\n{inner_indent}<laser_retro>{retro}</laser_retro>{match.group(0)}"

    new_text, n = VISUAL_CLOSE_PATTERN.subn(insert, text, count=1)
    if n != 1:
        return "no-visual"

    sdf_path.write_text(new_text)
    return "added"


def main() -> int:
    root = Path.home() / "blender_models"
    if not root.is_dir():
        print(f"ERROR: {root} does not exist", file=sys.stderr)
        return 1

    results: dict[str, list[str]] = {
        "added": [], "skipped": [], "multi-visual": [], "no-visual": [], "missing": [],
    }
    for model_name, retro in RETRO_VALUES.items():
        sdf = root / model_name / "model.sdf"
        if not sdf.is_file():
            results["missing"].append(model_name)
            continue
        status = patch_sdf(sdf, retro)
        results[status].append(f"{model_name} (retro={retro})")

    for status, items in results.items():
        if not items:
            continue
        marker = "OK " if status in ("added", "skipped") else "!! "
        print(f"{marker}{status} ({len(items)}):")
        for it in items:
            print(f"   {it}")

    if results["multi-visual"] or results["no-visual"] or results["missing"]:
        print("\nNon-trivial cases above need manual review.", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
