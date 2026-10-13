# Congress Street: first shared-material bridge slice

October 12, 2026. The Boston fixture now has one generated roadway slab with shared terrain/deck materials.
This establishes the structure rendering path. Detailed fitting to today's ungraded terrain is deferred at the
user's request: the terrain will be flattened/modified later. The saved height profile is a temporary demonstration.

![Congress Street prototype over Boston imagery](output/bridge-boston-congress/context-overview-final.jpg)

## What is implemented

`bridge_slice.py` selects three connected source ways from the frozen Boston structure GeoJSON, checks the
shared OSM nodes and finds one adjoining ground edge at each end. It exports a separate, approximately
159-meter glTF roadway deck, underside and fascia. The compiler's 7.4-meter lane-based roadway footprint is
preserved. The source's 22.9-meter whole-bridge width is retained separately; sidewalks and the full structure
are not reconstructed. The tagged movable span is shown closed, with no bascule mechanism or piers.

The top is solved once, then partitioned by the existing yellow marking polygons. Paint inherits each original
deck triangle's plane; asphalt occupies the remainder. There is no coplanar paint overlay or height bias.
This bridge has no source white marking polygons. Its center stripes stop about 20 cm before the source
way ends, including internal segment boundaries and landings; those source gaps are preserved.

`bridge_materials.py` generates original, deterministic 1024-square albedo and packed ORM maps for asphalt,
concrete, white paint and yellow paint. They use periodic grain, restrained color variation and distinct
roughness. Terrain and deck share the same asphalt/paint files, one-meter ENU scale and geographic origin.
The glTF UV convention accounts for the different image-row orientation. These are baseline materials;
normal maps, sparse flaking, TextureArena/MaterialArena integration and performance work remain deferred.

Ground material coverage continues two meters beneath each landing, inside the source deck footprint.
This explicit attachment region avoids an antialiased mask seam. It does not change elevation or establish
geometric attachment. The source GeoJSON remains unchanged; all receiver changes are derived copies in the
bridge output folder. No custom terrain geometry is generated.

## Temporary height assumptions

The manifest identifies heights as inferred WGS84 ellipsoid meters. A retained runtime snapshot of unchanged
ReadyMap terrain supplies the initial end cross-sections. A smooth, grade-constrained illustrative profile
supports a 0.75-meter slab, tapered over 40 meters at each end. The source end-cap directions are respected.

The demonstration's maximum centerline grade is 15%, reached near the eastern approach. This is a known
limitation of the temporary profile, not a claim about the real bridge. ReadyMap's coarse surface includes
existing crossing/urban elevation; clearance above that surface is not navigational or surveyed clearance.
The central underside is at least 1.03 meters above the sampled centerline terrain in this demonstration.
Neither this number nor a layer tag supplies a defensible real-world bridge height.

Further landing refinement and adaptive attachment have stopped. Earlier sample-fit diagnostics are retained
as historical evidence only. Future grading/profile integration should replace these assumptions rather than
preserve the current grade or make a production bridge fit this temporary DEM snapshot.

## Validation

The transient independent audit reads the written glTF buffers, transforms them back into the source frame,
and compares them with authoritative polygons. The final export contains 23,786 triangles and 71,358 vertices.

* No degenerate exported triangles; unit normals pass.
* Pavement footprint difference is 0.001052 square meters over 1,174.4 square meters. Float-coordinate slivers
  have no interior farther than 0.15 mm from their boundary; the check includes tiny internal partition slivers.
* Yellow paint footprint difference is 0.001901 square meters, confined within 0.15 mm of the source paint boundary.
* Paint-to-deck height error is below 0.003 mm; material texture phase matches at exported vertices.
* Source hashes and generated material hashes pass. The double-precision top partition area error is below
  0.00000000001 square meters. Boston, NYC and London source fixtures are not rewritten.
* Ordinary rendering and the GL4 east-landing view load the same final assets. Static adaptive-terrain views
  are diagnostic only; they do not prove stable attachment during LOD morphing or elevation revisions.
* Runtime layer checks reject LOD 14 coverage requests and accept LOD 15. Paired captures show no roads at 14
  and roads at 15. This validates the cutoff, not a finished fade or performance improvement.

The fixed reference uses 17-node terrain tiles, maximum LOD 19, range factor 40 and disabled morphing. The
sample probe verifies the actual hit tile level, because a maximum level alone does not force refinement.
Adaptive scenes use maximum LOD 22, range factor 7 and morphing. Views request 1200 by 1000 pixels, four
samples and logarithmic depth. Current captures are 803 by 699 logical pixels. No new engine build is required
for this slice; it uses the previously built Prestige material and triangulation paths.

## Inspect and reproduce

Run from the worktree using the existing Python environment with NumPy, SciPy, Shapely and pyproj:

```powershell
python tools/experiments/prestige_roads/bridge_slice.py build
./tools/experiments/prestige_roads/view_bridge.ps1 -Mode context
./tools/experiments/prestige_roads/view_bridge.ps1 -View deck
./tools/experiments/prestige_roads/view_bridge.ps1 -Mode dynamic -View east-landing -GL4
```

The launcher initializes `osgearth_shell.bat`. `build` consumes the retained terrain snapshot and checks
source hashes. `prepare` regenerates materials and query inputs; a new runtime terrain sample is required
if those inputs change. Runtime/export audit harnesses remain outside the repository, per project instructions.
The build marker prevents opening a partially regenerated package.

Assets, provenance, scenes and numerical results are in
[`output/bridge-boston-congress`](output/bridge-boston-congress/).
Useful views: [overview](output/bridge-boston-congress/context-overview-final.jpg),
[east landing / GL4](output/bridge-boston-congress/east-landing-final-gl4.jpg),
[deck detail](output/bridge-boston-congress/deck-final.jpg),
[LOD 14](output/bridge-boston-congress/cutoff-14.jpg) and
[LOD 15](output/bridge-boston-congress/cutoff-15.jpg).

This is one explicit structure adapter, not a general solution for elevated junctions, curved ramps or
stacked structures. Those cases and authored profile inputs are the useful next expansion; terrain fitting
waits for the later grading work. See the [staged plan](../../../docs/design/prestige-roads-rendering-plan.md).
