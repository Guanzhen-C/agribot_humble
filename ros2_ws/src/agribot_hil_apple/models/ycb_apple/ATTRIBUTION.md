# YCB Apple

Asset: YCB Object and Model Set, 013_apple, Google 16k textured scan.
Credit: Berk Calli et al.; Yale University, Carnegie Mellon University,
and University of California, Berkeley.

Source archive:
http://ycb-benchmarks.s3-website-us-east-1.amazonaws.com/data/google/013_apple_google_16k.tgz

License: Creative Commons Attribution 4.0 International.
https://creativecommons.org/licenses/by/4.0/

The source COLLADA and PNG are unchanged. At runtime the simulation creates
a derivative COLLADA with vertex and normal coordinates rigidly rotated to
place its measured stem axis along +Z, with the stem socket at the origin.
It updates the axis metadata and adds a separate small stem visual.
This is a textured scan of a benchmark object, not a new photogrammetry scan
of a living apple on the original orchard tree.

`background_stem_up.dae` is a derived background LOD of the same archive's OBJ
with the same rigid socket alignment and source texture coordinates. Blender's
decimate modifier reduces it to approximately 2,048 triangles; nearby selectable
task fruit retains the original full-resolution mesh. The LOD uses the same
CC BY 4.0 license and attribution.
