# Roads on a shared graded terrain surface

October 12, 2026. The existing 2 km square Boston, NYC, and London fixtures now feed the Prestige
`GradingLayer` with paired centerlines and pavement footprints. ReadyMap is the base elevation source;
the road overlay supplies sparse absolute elevations at one native LOD. Asphalt and paint remain splatted
onto osgEarth's ordinary regular terrain grids. No custom terrain mesh is involved.

The junction crease found in the first pass is corrected. Junctions now use one common plane fitted from
their centerline terrain samples, shared with their explicitly connected approaches. Thirty-meter quintic
transitions return to the roads' longitudinal profiles while matching height and slope at the handoffs.
The NYC before/after comparison below retains the original failure. Bridge approaches, buildings, and
inland-water grading remain outside this pass.

## Data and elevation contract

`road_grading.py` reads the frozen compiler package, verifies its recorded input hashes, and writes
`output/road-grading-{city}/road-profiles.geojson`. Each feature pairs complete source centerline geometry
with its associated compiled pavement polygons in a GeometryCollection. The pavement masks retain holes.
The layer samples the preceding elevation stack along the centerlines, interpolates the longitudinal
profile, and applies it across the associated footprint. A five-meter shoulder blends to the base.

Accepted junctions use the approach IDs recorded by the compiler. Source-way corridors use their own
alignments. Rounded shared-network fill keeps its existing surface ownership; a nearest-ground-centerline
height association is explicitly labeled inferred. This does not invent an OSM connection or assign a new
road owner. All three exports have no profile-source IDs in common with their separate structure packages.
The original vectors, structures, markings, and ownership packages are unchanged.

Each derived feature now supplies ordered centerline IDs and a junction-selection flag. Common planes are
applied by those identities, not by proximity to a road. This prevents a nearby disconnected road from
receiving the correction. The thirty-meter profile transition changes height only; it does not enlarge
the five-meter terrain shoulder or fill no-data cells. Fits use complete junction footprints and fixed
source sampling, so adjacent output tiles agree. Public options changed, so the ABI version is now 191.

The opt-in `road_footprints` policy keeps ordinary line corridors and level pads available for other inputs.
It rejects a feature missing centerlines or polygons. Source geometry Z remains ignored; these are
terrain-derived profiles, not surveyed road elevations. Building pads and waterways need separate policies.

## One native grading LOD

All three scenes use `min_level=16`, `max_data_level=16`, and 257-square global-geodetic elevation tiles.
`max_level` stays unset so the layer continues participating in finer terrain requests. Native sample spacing
is about 1.2 meters north-south, and narrower east-west at these latitudes. Centerlines are sampled at up to
10-meter spacing, with a 30-meter smoothing radius and a five-meter source-elevation resolution request.
ReadyMap can supply coarser data; the requested spacing does not imply surveyed accuracy.

Higher terrain detail resamples native grading tiles. No-data cells permit the underlying elevation to
contribute. A finer tile wholly covered by parent grading can report no new detail, letting the terrain
pager retain its already graded parent. Native elevation LOD and the renderer's terrain LOD differ because
their grid sizes differ. The existing asphalt/paint activation remains LOD 15.

Native generation can repeat at the same LOD when requests are uncached. This prototype has not introduced
a persistent grading cache or a dependency-invalidation scheme; it does not recompute distinct native
grading levels for each finer terrain LOD. No performance claim is made.

## Validation

The prescribed `build.bat` build and install passed. The transient harness outside the repository passed
8,506 assertions in 19 cases, including existing corridor, polygon, width, and regional-fallback checks.
New cases cover paired masks, asymmetric widths, longitudinal slope, holes, cancellation, malformed input,
configuration round-trip, adjacent tile edges, finer base fallback, and native-generation LOD counting.
The latter observes only the configured source LOD while requests proceed five levels finer.
The junction regression reproduces the former ridge on known affine terrain and checks the common-plane
correction, feature-order invariance, exact native tile-edge agreement, malformed profile IDs, disconnected
road isolation, and matching first derivatives at both transition boundaries.

An independent live-scene probe sampled eight native tiles around the three close views. The audit compared
those samples against the frozen asphalt polygons, not the adapter's feature metadata. Every sampled pavement
interior had grading, and every sample more than 5.05 meters outside pavement was no-data. The small margin
avoids projection/boundary classification noise. Shared edges matched exactly.

| Measurement | Boston | NYC | London |
| --- | ---: | ---: | ---: |
| Paired grading features | 6,190 | 4,304 | 8,449 |
| Footprint difference from authoritative pavement, m² | 0.000167 | 0.000185 | 0.000000451 |
| Native grid samples audited | 33,282 | 33,282 | 66,564 |
| Pavement-interior samples | 6,988 | 4,508 | 12,656 |
| No-data samples beyond the shoulder | 20,678 | 22,694 | 38,921 |
| Shared-edge comparisons | 129 | 129 | 515 |
| Composed elevation samples | 626 | 626 | 626 |
| Maximum sampled road elevation change, m | 0.912 | 1.616 | 0.188 |
| Maximum outside-road composed difference, m | 0.00274 | 0.00149 | 0.00206 |

The millimeter-scale outside differences occur in elevation-pool quantization/resampling. The sparse native
overlay itself supplies no height there. These are bounded scene-center audits, not exhaustive city-wide
grade certification. Source and measurement details are in each output folder's `manifest.json`,
`validation.json`, `validation-tiles.csv`, and `validation-samples.csv`. No new regression files were added
to the repository.

The same native samples, at least three meters inside each camera junction, were also compared with a
best-fit plane. This measures the ridge separately from the junction's overall slope:

| Junction planarity | Boston | NYC | London |
| --- | ---: | ---: | ---: |
| Interior samples | 102 | 28 | 26 |
| Former maximum deviation, m | 0.403 | 0.702 | 0.193 |
| Corrected maximum deviation, m | 0.00635 | 0.000000457 | 0.00000215 |
| Corrected RMS deviation, m | 0.000654 | 0.000000274 | 0.00000109 |

Boston retains a small residual from overlapping neighboring influences. NYC and London are planar to
float-grid precision. These are numerical surface-continuity measurements, **not** elevation accuracy.
The terrain-derived plane grades are approximately 10.0%, 21.3%, and 4.4%, respectively. NYC's steep inferred
grade shows why coarse urban DEM samples must not be treated as surveyed road elevations. This correction
removes the blend-induced crease; it does not introduce road-class grade limits or a regional engineering
grade solver. Those need explicit policy and evidence, rather than silently capping source-derived slopes.

Each output folder retains `junction-before-*` evidence and `junction-validation.json`. The original
compiler and structure package hashes still match.

## Viewed results

All three close views were inspected in the default Rex renderer, using the same material assets and
terrain settings: 17-square terrain grids, SSE 8, maximum terrain LOD 22, range factor 7, four samples,
and logarithmic depth. Captures are 803 × 699 pixels including the window frame on this display.
This pass does not claim a moving-camera transition or GL4 comparison.

Boston:

![Boston centerline grading](output/road-grading-boston/after-close.jpg)

NYC before grading:

![NYC before grading](output/road-grading-nyc/before-close.jpg)

NYC with the former profile blend, showing the diagonal crease:

![NYC before the junction correction](output/road-grading-nyc/junction-before-after-close.jpg)

NYC with the corrected common junction plane, using the same camera:

![NYC graded junction](output/road-grading-nyc/after-close.jpg)

London:

![London centerline grading](output/road-grading-london/after-close.jpg)

## Reproduce and inspect

Use the experiment's existing Python environment with Shapely, NumPy, and pyproj, then:

```powershell
python tools/experiments/prestige_roads/road_grading.py boston nyc london
tools/experiments/prestige_roads/view_road_grading.ps1 -City boston -View close
tools/experiments/prestige_roads/view_road_grading.ps1 -City nyc -View close
tools/experiments/prestige_roads/view_road_grading.ps1 -City london -View close
```

The launcher initializes `osgearth_shell.bat`. Use `-Before` for ungraded ReadyMap, or `-View overview`
and `-View top` for alternate cameras. `-GL4` is available but was not part of this pass's visual evidence.
Shared asphalt and paint textures come from the completed Congress Street bridge material package.

The crease is now a retained regression case. Remaining quality work includes source-grade plausibility,
closely spaced junctions, and structure landing grades. Water-adjacent terrain treatment should follow an
explicit shared profile contract; elevations must not be inferred merely from OSM layer numbers.
