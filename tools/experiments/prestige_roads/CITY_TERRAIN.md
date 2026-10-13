# Boston and Pittsburgh: shared elevation for roads, buildings, and trees

Completed demo pass, 2026-10-10. Both city scenes compile and render; remaining geometric failures are explicit below.

The experiment composes real ReadyMap elevation, an inferred road design, a sparse road-owned grading
overlay, OSM building extrusions, and Prestige vegetation. Buildings and vegetation use native map
clamping against the same elevation stack that renders the ground. The source terrain is retained;
the grading raster overrides it only in the road influence area. There are no TerrainConstraints,
virtual textures, or osm2streets dependencies.

The regions are bounded city districts, not entire metropolitan areas:

- Boston: the Common, Public Garden, Back Bay, and adjacent downtown streets; approximately 2.37 km².
- Pittsburgh: downtown, the Allegheny crossings, and the North Shore; approximately 2.42 km².

| Exported result | Boston | Pittsburgh |
| --- | ---: | ---: |
| Selected road ways | 874 | 866 |
| Unique junction layouts | 229 | 133 |
| Lane connections | 925 | 739 |
| Crosswalks / stop bars | 332 / 200 | 275 / 78 |
| Building features | 2,609 | 549 |
| Mapped tree positions | 1,916 | 1,875 |
| Woodland polygons with inferred scatter | 8 | 40 |
| Road/detail triangles | 1,568,821 | 1,842,939 |
| Maximum significant pavement slope | 14.1913% | 14.1989% |
| Bridge/ground overlap screens passing / failing | 35 / 23 | 159 / 40 |
| Distinct exported ground vertices buried by grading | 0 | 62 |
| Largest inferred cut / fill | 13.55 / 9.01 m | 24.53 / 9.87 m |
| 95th percentile absolute grading change | 2.27 m | 3.34 m |

These are bounded experiments, not fully correct city road models. Boston's 1,695,465 exported pavement
vertex samples remain at least 0.1747 m above the bilinearly sampled grading. Pittsburgh has 228 buried
samples representing 62 distinct vertices, with a worst discrepancy of 3.73 m at a tight height transition.
The final bridge screens operate on overlap pairs, not unique bridges; some ambiguous approach overlaps
and crop-edge cases need classification as well as height design. Bridge-over-bridge clearance is not checked.
The complete audit summary is [city-terrain-summary.json](output/city-terrain-summary.json).

Native overview and pedestrian views were inspected in both cities. The representative street views
remain consistent at terrain SSE 2 and 8. This is a local visual check, not a global guarantee about terrain LOD.
The independent exported boundary errors are below 0.044 mm, and all exported positions and normals
are finite with unit normals. All six million grading pixels pass the declared influence-footprint check.

The original Manhattan, Alewife, and five-city source snapshots remain unchanged. The two new extracts
include service roads and tertiary links. Four bounded map-API requests per city retain full returned
way/node references, source versions, request provenance, and a merged snapshot hash. The merge is
not an atomic snapshot. Incomplete building/landcover relations are diagnosed rather than closed with
invented edges.

## Shared elevation contract

1. Sample the retained native ReadyMap elevation tiles and convert EGM96 orthometric heights to WGS84
   ellipsoid heights using osgEarth's geoid grid convention.
2. Infer connected centerline profiles. Source-connected nodes share heights; grade and selected
   bridge-crossing constraints act on the profiles. The DEM is a soft target.
3. Fit the connected pavement with fixed bridge-end ties and a face-gradient constraint. This preserves
   islands and tests the full surface, rather than accepting a road merely because its centerline fits.
4. Generate both absolute road meshes and a grading raster from the same ground-road triangles. Bridge
   decks are excluded from the grading raster. No-data outside the feathered influence area leaves the
   underlying ReadyMap layer in control.
5. `FeatureModel` clamps building base vertices through the map elevation pool. Prestige vegetation
   samples that same pool before placing models. Both request one-meter sampling resolution.

The source is advertised as 90-meter elevation. One-meter output and query spacing are interpolation,
not one-meter measurement accuracy. These road grades, clearance targets, dimensions, and earthworks
are visualization assumptions, not surveyed road engineering.

The prototype uses 9% ground-centerline and 8% bridge-centerline grade limits, a 14% pavement-gradient
target, and the existing 15% significant-face rejection guard. Profile crossings use 6.5 m of inferred
surface separation. A separate final-mesh audit checks actual deck underside clearance over overlapping
pavement. Passing the profile constraints does **not** certify final bridge clearance.

## Building and vegetation evidence

Building heights use a parseable mapped height first, then an explicit 3.2 m per mapped level inference,
then a recorded 10 m fallback. Mapped building parts replace overlapping parent footprints. Walls are
segmented before native clamping; roofs are flat. No building-pad flattening is introduced.

Mapped tree points retain their source identities and tags. Trees on compiled road or building
footprints are excluded and listed. Species and sizes come from shared illustrative vegetation assets.
Additional deterministic trees are permitted only within mapped woodland polygons; the distinction
between mapped positions and inferred scatter is exported. Incomplete source polygons and omitted
building dimensions remain in the context audit.

The new Prestige Flattening layer is not stacked on top of the road grading. It would independently
derive another target surface. Any later integration should consume the same solved road surface so
terrain, roads, and placement queries do not disagree.

The comparison reloads the map for each elevation variant. Live invalidation and reclamping after an
interactive grading edit are not tested by this pass.

## Compiler corrections exposed by the larger region

Pittsburgh rejected the first city pavement fit with a 165.85% worst significant-face grade. The 15%
guard remained in force. The failure was traced to incompatible ownership, rather than addressed by
weakening the acceptance check:

- Global corner rounding joined 36 separate raw ground-pavement components into 19. Rounding now
  protects foreign components and discards detached fillet debris. The source components remain 36;
  approximately 10,453 m² of artificial connecting pavement is removed in this source extract.
- Every bridge way end previously imposed a ground tie, including joins entirely between bridge
  segments. Only source-connected ground approaches now impose those ties, reducing the generated
  Pittsburgh cap constraints from 154 to 63 before vertex selection.
- Junction height influence now travels along ground source paths in design mode. A nearby junction
  on a different road family, or entirely on a bridge, cannot pull the ground surface toward its height.
- Seven explicitly recorded compatibility constraints coordinate near-touching deck ends where their
  inferred ground-approach footprints overlap. These are height assumptions and create no lane-graph
  edges. The independent final mesh audit still decides whether a crossing has sufficient clearance.

The original synthetic Manhattan and Alewife modes retain their junction-blending policy. Alewife's
checked clearances, cap boundaries, and pavement fit are unchanged after the source-connection fix.

Both repaired city pavement fits pass the unchanged rejection guard. Boston has 565,155 pavement
triangles and a 14.1913% maximum significant-face slope; Pittsburgh has 535,525 and 14.1989%.
Neither accepted fit has triangle area above 15%. This is surface acceptance, not clearance or
engineering acceptance. In particular, the inferred design can require substantial local earthworks
against the coarse source DEM; those differences are retained in the grading and terrain audits.

The expanded discovery resolves 230 candidate rows in Boston and 135 in Pittsburgh, corresponding to
229 and 133 unique layouts after compound-node entries are deduplicated. It also records 296 and
398 unresolved candidates, respectively. These counts describe candidate junction nodes, not a fraction
of streets that render. Close adjacent junctions requiring combined ownership dominate the remaining
diagnostics (171 in Boston and 236 in Pittsburgh). Their road surfaces still participate in the terrain
experiment, but their lane layouts are not represented as solved.

## Compile and view

Use the Python dependencies in `requirements.txt`, then run from the repository root:

```powershell
python tools/experiments/prestige_roads/compile.py --scene boston-region --terrain readymap --surface design
python tools/experiments/prestige_roads/city_context.py boston-region
python tools/experiments/prestige_roads/compile.py --scene pittsburgh-region --terrain readymap --surface design
python tools/experiments/prestige_roads/city_context.py pittsburgh-region

tools/experiments/prestige_roads/run.ps1 -RuntimeRoot C:/Users/gwald/.codex/worktrees/a436/repo `
    -Scene boston-region -Terrain readymap -Surface design -City -View city-overview
tools/experiments/prestige_roads/run.ps1 -RuntimeRoot C:/Users/gwald/.codex/worktrees/a436/repo `
    -Scene pittsburgh-region -Terrain readymap -Surface design -City -View city-overview
```

Use `city-park` and `city-street` for context views, or any generated intersection view. Street cameras
are derived from resolved lane approaches, with a pavement contact check, two-meter building clearance,
and an unobstructed building-footprint sightline. They replace manually chosen cameras that could land
inside a building after its asynchronous layer finished loading. Allow the context to finish loading
before inspecting a close view.

`city-bridge-check`
targets the worst measured bridge/ground overlap; it is a diagnostic view, not a clearance acceptance.
Where the independent export audit identifies buried vertices, `city-grading-check` targets that raster
failure. These diagnostic views deliberately retain failures rather than selecting only attractive streets.
`-Mode raw`
removes grading while keeping the same road design and clamping consumers; `-Mode graded` uses the
final composite surface. `-SSE 2`, `8`, and `32` select terrain detail. The launcher initializes
`osgearth_shell.bat` and refuses outputs marked by a failed vertical or pavement solve.

The native viewer uses the installed runtime in the other worktree without changing it. This pass
changes the Python experiment and scene descriptions; it does not require an engine rebuild.

## Validation and remaining limits

The completed smaller Pittsburgh integration check includes 234 building features, 463 mapped tree
points, and four woodland polygons. Its native street view uses the same camera and road mesh with
and without grading: natural terrain covers the far street; the grading layer exposes its pavement.
Buildings and vegetation are independently generated native consumers in both scenes.

| Graded ground | Natural ground |
| --- | --- |
| ![Graded native clamping](output/city-clamp-smoke-pittsburgh-street.jpg) | ![Natural ground comparison](output/city-clamp-smoke-pittsburgh-raw-street.jpg) |

Transient checks cover slope projection, impossible hard ties, connected profile clearance and grades,
recovery from failed solves, terrain sampling, geoid orientation, tile seams, source hashes, sparse
grading influence, building-height provenance, and the existing lane/control semantics. Manhattan's
29 layouts and original seven candidates still pass; Alewife's two checked bridge clearances remain
5.2482 m and 5.2915 m.

City scale also exposed repeated whole-city buffering inside every turn-curve trial. The final solver
clips to a guarded region containing every candidate vehicle envelope before building one tolerance
buffer. In a transient 6,943-component polygon benchmark, one original run took 43.24 s; the final
version took 0.031–0.035 s over three runs, with exactly identical selected paths and ribbon boundaries.
This is a kernel benchmark, not an end-to-end city speedup. The earlier hoisting-only change measured
43.33 s versus 2.32 s median over three runs each. Benchmark inputs and scripts remain outside the repo;
results are retained under `output/city-turn-*-benchmark.json`.

Road-surface refinement now samples new triangles in bounded batches and preserves untouched parent
heights. A transient real-Pittsburgh-sampler benchmark refined 800 input triangles into 25,600 triangles:
the median fell from 2.823 s to 0.835 s over three runs, with identical triangle order, XY, and heights.
The result is retained in `output/city-refinement-benchmark.json`.

Repeated paint/support buffers also use a bounded cache over immutable geometries. A transient complete
lane-audit benchmark, clearing the cache before each trial, measured 0.460 s versus 0.091 s median over
three runs, with an identical audit result. See `output/city-audit-buffer-benchmark.json`.

The final Boston area check exposed repeated densification of a fixed-precision polygon. That second
operation re-snapped previously inserted boundary points and changed the footprint by 0.000915 m².
Triangulation now clears the precision model before inserting points, preserving the already cleaned
boundary. The error falls to approximately 2.2e-9 m². The original 1e-5 m² acceptance threshold remains;
the check now runs before asset export and leaves a launcher-visible failure marker. Deliberately
omitted pavement is still rejected. Alewife's checked clearances remain unchanged.
The same precision correction applies to the separate boundary export; its Boston discrepancy falls
from 0.715 mm at one sample to below 0.044 mm after float glTF export.

Two further export corrections avoid repeated work without changing the geometry. Crosswalks reuse one
index over their unchanged support mesh, and stop bars reuse their ground inset. The paint benchmark
uses 535,525 actual city support triangles and ten paint calls: medians are 6.687 s before and 0.627 s
after, including index construction, with exactly identical output triangles. Urban layouts are exported
once after their paint is finalized instead of twice. The independent grading-footprint check uses an
indexed distance predicate: 10,000 Boston points take 1.440 s versus 0.0186 s median, with identical
classifications and explicit radius-boundary checks. The complete pixel checks also pass in both cities.
Results are retained in `output/city-final-export-regression.json` and `output/city-influence-benchmark.json`.

## What the combined scene establishes

The native overview and street views demonstrate buildings and vegetation consuming the same composite
elevation as the terrain. They do not establish global collision freedom, surveyed building heights,
bridge engineering, or successful live reclamping after an edit.

![Boston integrated overview](output/city-boston-overview.jpg)

![Pittsburgh integrated overview](output/city-pittsburgh-overview.jpg)

![Boston pedestrian view](output/city-boston-street-sse2.jpg)

![Pittsburgh pedestrian view](output/city-pittsburgh-street-sse2.jpg)

The raster also exposes a limitation of this first grading representation: closely spaced surfaces at
different heights can be blurred together by one-meter pixels and bilinear interpolation. This is a
measured failure, not a passed clearance test. The Pittsburgh diagnostic view shows the resulting tear:

![Pittsburgh local grading limitation](output/city-pittsburgh-grading-limit.jpg)

The next grading step should constrain the sampled final elevation against every ground-road surface,
refine resolution where necessary, and reject unresolved level ownership. A single height field cannot
represent vertically stacked ground at one horizontal location. Bridge and retaining-structure ownership
must be settled before treating those locations as ordinary terrain. Buildings need explicit pad or
foundation policy where large cuts reach their footprints; independent clamping alone does not design it.

The road mesh is resident, the lane solver still reports unsupported candidates, and bridge-over-bridge
design is not implemented. Building roofs, tree assets, retaining structures, and road materials are
illustrative. Final-mesh bridge failures remain explicit; they must not be hidden by a successful
centerline solve or a visually plausible distant view.
