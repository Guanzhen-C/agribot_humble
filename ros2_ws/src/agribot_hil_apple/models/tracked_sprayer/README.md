# Sprayer Material Atlas

`material_atlas.png` is an AI-generated diffuse colour texture created with the
built-in image generation tool on 2026-11-09 for this simulation. It is not the
original manufacturer's paint or a measured PBR material. No brand or logo is used.

Quadrants, as viewed in the PNG: white HDPE top-left, green coated steel top-right,
satin steel bottom-left, graphite polymer bottom-right. The Blender exporter uses
box-projected UVs inset into these quadrants, not a flat single-colour overlay.
The water tank is white, as requested. Chassis paint is a separate, scenario-local
grey material variant; shared vehicle and sensor exports are never overwritten.

Rebuild from the same private supplied STEP (FreeCAD and Blender required):

```bash
python3 scripts/prepare_work_material_parts.py ~/.cache/agribot/isobus/work_models/plant_protection
blender -b -noaudio --python scripts/blender_textured_work_model.py -- \
  ~/.cache/agribot/isobus/work_models/plant_protection/textured \
  --atlas models/tracked_sprayer/material_atlas.png
```

These commands refuse to overwrite an existing textured model. Archive that cache
first when explicitly rebuilding. `parts.json` preserves the STEP hash and the
material role of each solid; `appearance.json` records mesh, texture and geometry
bounds. Installation transform, uniform 0.55 display scale, bore positions,
collision model, mass and sensor configuration are not changed by texturing.

## Exact Generation Prompt

Use case: product-mockup. Asset type: a single production 3D material base-color texture atlas, square 1024x1024. Primary request: a perfectly flat, orthographic 2 by 2 texture atlas of four clean agricultural equipment surface materials, one material filling each exact quarter of the square. Top-left quarter: warm neutral WHITE rotational-moulded HDPE water tank plastic, subtle very fine polymer stipple, not yellow, mean light grey-white. Top-right quarter: dark forest green powder-coated steel paint, extremely fine even orange-peel coating texture. Bottom-left quarter: clean satin silver zinc-plated/brushed steel, subtle fine random brushed grain, no obvious diagonal bands. Bottom-right quarter: graphite black engineering polymer/rubber, very fine matte surface texture. Each quadrant is a seamless edge-to-edge flat surface swatch with no shapes or illustrated objects, no highlights, no shadows, no perspective, no borders, no gaps, no text, no labels, no logo, no rust, no damage, no large-scale lighting gradient. Very low-contrast realistic microsurface detail, physically plausible uniform diffuse base colour, intended to be mapped as albedo onto existing CAD geometry, NOT a rendered product and NOT a mood board.
