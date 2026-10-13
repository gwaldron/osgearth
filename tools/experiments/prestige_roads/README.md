# Prestige Roads: offline OSM experiments

This offline OSM compiler produces conventional glTF pavement, bridges, curbs, and markings for osgEarth.
It uses source node identity, tags, and restriction relations. No virtual texturing or osm2streets is involved.
Heights, pavement dimensions, and marking geometry are experimental, not surveyed conditions.

**Checkpoint:** [13 October 2026](CHECKPOINT.md) records the saved scope, validation, and local-only outputs.

**Sion city:** the [Sion dataset](SION_CITY.md) combines a 4 km by 3 km urban road package, bridges,
graded terrain, Swiss markings, and ReadyMap base imagery. Run
`view_interchange.ps1 -Scene sion-city -View overview` or `-View rhone`.

**Springfield interchange:** the [multilayer network experiment](SPRINGFIELD_INTERCHANGE.md) adds a separate
3 km square with source-connected ramps, shared constrained height profiles, and explicit terrain/structure
ownership. Run `view_interchange.ps1 -View stack`. Height and receiver assumptions remain attributed separately
from OSM tags; this does not replace the frozen city fixtures or the original Boston bridge demo.

**Shared bridge grading:** [Congress Street approaches and deck](BRIDGE_GRADING.md) now use one explicit
height profile. The deck stays separate geometry; sparse native-LOD-16 grids grade its connected approaches
and short landing support areas. Run `view_bridge_grading.ps1 -View west-landing -GL4`.
The central span does not grade the terrain beneath it. Heights and support dimensions remain inferred.

**Coverage update:** asphalt and paint masks now use R8, with unchanged vector rasterization and separate
material passes. The [R8 validation and upload results](SURFACE_SPLATTING.md#r8-coverage-validation-and-upload-measurement)
cover all three fixtures, legacy caches, both renderers, and the reduced upload payload.

**NYC combined demo:** [prestige_roads_nyc.earth](../../../tests/prestige_roads_nyc.earth) combines the
corrected native-LOD-16 grading, asphalt/paint, ReadyMap imagery, Prestige buildings, vegetation, and sky.
Run `view_prestige_nyc.ps1`; it launches `osgearth_imgui` with `--nvgl --ico --prestige-sky --samples 4`
and logarithmic depth. Viewpoints include the overview, repaired junction, and City Hall Park.
The scene uses the local building dataset at `D:/data/prestige/Prestige_ktx/models/prestige/` because
the public test endpoint lacks the NYC tile. Buildings retain authored elevations; vegetation samples
the composed terrain at a one-meter resolution request. Park scatter is explicitly inferred at 8% of
woodland density; compiled pavement, mapped buildings, water, and road/path buffers exclude vegetation.
The generated fixture/material packages remain required; this is not a standalone distributable dataset.

**Latest: [Three-city road grading](ROAD_GRADING.md), 2026-10-12.** Boston, NYC, and London now use
centerline-derived profiles bounded by their compiled pavement polygons. Sparse native LOD 16 elevation grids
sit above ReadyMap; finer terrain reuses them. Asphalt and markings share the resulting surface. Run
`road_grading.py boston nyc london` and `view_road_grading.ps1 -City boston -View close` (`-Before` for comparison).
The source vectors remain unchanged. The NYC junction crease is corrected with a common fitted plane and
smooth transitions on explicitly connected profiles; all three city close views and regressions pass.
The shared Boston bridge transition is now implemented separately; building and inland-water grading remain deferred.
The earlier level-pad adapter is
retained as `grading_demo.py` for historical comparison.

**Latest: [Congress Street bridge slice](BRIDGE_SLICE.md), 2026-10-12.** A separate roadway slab now shares
generated asphalt and paint materials with Prestige terrain splatting. It includes the underside and fascia,
source-connected landings and explicitly inferred heights. Use `view_bridge.ps1 -Mode context` or `-View deck`.
That original ungraded baseline is retained; the shared-profile grading experiment supersedes its landing fit.

**Latest: [terrain markings comparison](MARKINGS.md), 2026-10-12.** White/yellow paint now renders over asphalt
in all three fixtures. A raster-width correction disables the optional seam outline for paint. Terrain
coverage is the provisional ground backend; the GPU-clamped decal baseline has documented attachment failures.
Use `view_markings.ps1 -City boston|nyc|london` or `-View detail`. Arena integration remains deferred.

**Latest: [Prestige asphalt on terrain](SURFACE_SPLATTING.md), 2026-10-12.** The new
`prestige:surface_splatting` image layer applies repeating asphalt inside compiler pavement coverage on
ordinary ReadyMap terrain. It adds no road mesh or elevation alteration. All three fixtures have attributed
vector copies with explicit junction/shared-network ownership. Use `view_splatting.ps1 -City boston|nyc|london`
to inspect the first asphalt pass. Markings/structures remain the next renderer stages; performance tuning is deferred.

**Latest: [three rendering fixtures](RENDER_FIXTURES.md), 2026-10-12.** `city_fixtures.py` generates fresh
2 km by 2 km vector packages for central Boston, Lower Manhattan and central London, each with road
bridges. Ground markings/curbs, separate structure materials, source topology, evidence and fixed cameras
form the baseline for the new rendering architecture. `view_vectors.ps1 -Fixture boston|nyc|london`
opens a selected package. Blue-gray structure footprints in these views are 2D diagnostics with unknown heights.

**Latest: [separate structure vectors](STRUCTURE_VECTORS.md), 2026-10-12.** `vector_patch.py` now also
exports `structures.geojson` with source layer/level attribution and a separate source-node road graph.
Elevation and grade remain explicitly unknown. `structure_vectors.py` provides a second bounded Boston
sample covering the South Station highway ramps. Ground and structure features are not merged together.

**Latest: freshly compiled Boston vectors, 2026-10-12.** Run `vector_patch.py` to regenerate the 900 m
downtown patch from retained source OSM using the current compiler, without reusing geometry checkpoints.
Launch `view_vectors.ps1 -View overview` or `-View intersection`. The direct vector output contains 76 junction
layouts and 333 lane connections; [the manifest](output/boston-vectors-current/manifest.json) records source
and compiler hashes, marking checks, and compilation time. [Overview](output/boston-vectors-current/overview.svg)
and [junction](output/boston-vectors-current/intersection.svg) are also available as SVG plans.
The [triangulation correction](TRIANGULATION.md) fixes narrow paint and curb polygons lost by a float
conversion in osgEarth's Earcut adapter. The corrected default and a WeMesh comparison were checked
against the same GeoJSON; no vector regeneration or terrain changes were needed.

**Previous: [Boston gridded road grading](GRID_ROADS.md), 2026-10-11.** An ordinary elevation overlay
approximates flattened road corridors on osgEarth's regular terrain grid. The retained pavement,
markings, and curb bands render as ordinary terrain imagery. Buildings and trees use normal map
clamping. The custom terrain mesh and direct-sampling experiment has been removed.
Launch with `view_grid_roads.ps1 -View junction`; `-Mode raw` omits the grading for comparison.

**Previous: [Boston road imagery, 3 km radius](XYZ_RASTER.md), 2026-10-11.** The LOD 16 replacement uses an
ordinary global-geodetic XYZImageLayer: 451 tiles at 512 pixels, BC3 with mipmaps, totaling 150.40 MiB.
It includes full supported markings and inferred
curb/gutter bands. Explicit bridges and tunnels remain excluded. The previous LOD 14 tiles are deleted.
File, crop, and georeferencing checks pass; overview and close views were inspected in osgEarth.
Narrow yellow paint exhibits visible BC3 artifacts at close range. Launch with
`view_raster.ps1 -RuntimeRoot C:/Users/gwald/.codex/worktrees/a436/repo -View back-bay`.

**Previous: [Boston and Pittsburgh shared elevation demos](CITY_TERRAIN.md), 2026-10-10.** Two bounded downtown
regions combine ReadyMap elevation, road-owned grading, 362 junction layouts, 3,158 building features,
and 3,791 mapped tree positions. Native buildings and Prestige vegetation clamp to the same final map
elevation. Both pavement fits and boundary exports pass; bridge screens and a localized Pittsburgh
grading-raster failure remain explicit. Use `run.ps1` with `-Scene boston-region -Terrain readymap -Surface design`
and `-City -View city-overview`; the city report has complete commands for a Prestige-capable runtime.

**Previous: [Real ReadyMap terrain](REAL_TERRAIN.md), 2026-10-10.** Sion now compares fitted roads over untouched
terrain and a road-owned grading overlay, without TerrainConstraints. Pittsburgh exposes rejected grades;
bridge clearance remains an explicit diagnostic failure in both areas.

**Previous: [Physical lane roles and bus contraflow](LANE_ROLES.md), 2026-10-10.** Boston, Pittsburgh,
London, Paris, and Sion now produce 121 layouts and 540 private-car lane connections, with 424 crossings,
171 inferred stop bars, and 11 inferred BUS words. Physical reserved lanes retain their width and lane
numbers; source access and inference remain separate. One unsupported London allocation is withdrawn.
A follow-up fixes contraflow stop bars and withholds ambiguous combined signal/crossing placements;
`-Scene london -View contraflow-control` opens a corrected example.
All 104 surviving first-sweep city layouts and all 29 Manhattan layouts retain identical lane paths.
Compile with `compile.py --scene pittsburgh`; launch with `run.ps1 -Scene pittsburgh -View reserved-lanes`.

The [five-city stress-test report](CITIES.md) also covers left-hand traffic, source lane-connectivity
relations, narrow shared lanes, regional crossing styles, compound intersections, and stronger diagnostics.
A curvature penalty reduces Pittsburgh's bridge-approach spike while preserving hard deck ties and
Alewife clearance checks.

[Manhattan](MANHATTAN.md) now has 29 layouts and 120 lane connections. Its original seven candidates
remain covered, including the explicitly provisional, unpainted two-to-one turn allocation.
`run.ps1 -Scene manhattan -View mapped-lanes` opens the newly resolved source-connectivity case.

The remainder of this document records the Alewife experiment, which remains the default scene.

**Fifth pass, 2026-10-10:** automatic discovery resolves six junctions with 31 lane connections. A separate
180 m highway taper carries three lanes into two. Two two-way T-junctions include opposing traffic and turns.
The expanded crop retains eleven signal-based inferred stop bars and renders eleven tagged crosswalks.
Unsupported candidates have explicit diagnostics; there is no per-junction compiler configuration table.

![Three-to-two highway transition](output/pass5-lane-drop.jpg)

![Two-way junction in osgEarth](output/pass5-two-way.jpg)

![Two-way lane graph; colored paths are diagnostics, not painted stripes](output/junction-61322243-plan.png)

JPEGs prefixed `pass5-` record the fifth pass; the city report links later viewer captures.
Plan figures show lane paths before final crossing/control paint exclusions. The compiler regenerates assets,
plans, and maps; screenshots require the viewer. Generated output is ignored by Git.

## Run it

Requires Python 3.10+ and an installed osgEarth runtime with GDAL, glTF loading, and terrain constraints.
From the repository root in PowerShell:

```powershell
python -m venv tools/experiments/prestige_roads/.venv
tools/experiments/prestige_roads/.venv/Scripts/python.exe -m pip install -r tools/experiments/prestige_roads/requirements.txt
tools/experiments/prestige_roads/.venv/Scripts/python.exe tools/experiments/prestige_roads/compile.py
tools/experiments/prestige_roads/run.ps1 -RuntimeRoot C:/Users/gwald/.codex/worktrees/a436/repo -View lane-drop
```

The launcher calls `osgearth_shell.bat` before the viewer. `RuntimeRoot` defaults to this checkout; another
installed runtime can be selected without changing its sources. No engine C++ changes or rebuild were needed.
Close the experiment viewer before recompiling its input files. Compilation and viewing use local retained data.

Useful `-View` values: `lane-drop`, `lane-drop-street`, `two-way`, `two-way-street`, `intersection`,
`intersection-street`, `west-merge`, `south-merge`, `east-crossing`, `bridge`, `street`, and `crosswalk`.
Each resolved node also has a `junction-<OSM node ID>` view. Select `-SSE 2`, `8`, or `32` and
`-Mode replacement` or `overlay`. The current Alewife export has 138 maps: 23 viewpoints, two modes, and
three terrain-detail settings, including the new `surface-detail` and `surface-street` inspection views.
[viewpoints.json](output/viewpoints.json) lists them. Familiar camera aliases affect presentation only;
discovery and geometry never consult the alias table. Automatic touring is not a validated feature.
`compile.py --output <directory>` writes elsewhere; the launcher uses the default `output/` directory.

## Longitudinal lane drop

The original crop ended before the westbound lane count changed. The new context establishes this chain:

| Source way | Lanes | Evidence |
| --- | ---: | --- |
| `188033115` | 3 | `turn:lanes=||merge_to_left` |
| `39452964` | 3 | Same indication, highway bridge |
| `1311701320` | 3 | Same indication, final ground approach |
| `39452965` | 2 | Downstream lane count |

`transitions.py` follows shared endpoints and lane counts. It places an inferred 180 m taper at the end of the
final ground approach, clear of the bridge. Continuing paths are 1-to-1 and 2-to-2; lane 3 joins lane 2 before
the carriageway finishes narrowing. A quintic blend varies width and lane placement continuously. The disappearing
separator ends before its residual lane becomes too narrow. Paint follows the final pavement boundary.
Exact taper placement, length, shoulder allocation, and merge-arrow shape are assumptions, not mapped dimensions.
No gore is invented where the source shows a lane ending without a diverging branch.

Both left- and right-side single-lane-drop policies are tested; the rendered case drops the right lane.
Missing/branching continuation, conflicting counts/widths, insufficient length, structures, and qualified lane-change
policies produce diagnostics. The station-access-road merge remains unresolved because its continuation lies
outside the selected render context. This is not a general acceleration-lane merge solver.

- [Taper evidence, assumptions, paths, and checks](output/lane-drop-1311701320.json)
- [Taper paths in WGS84](output/lane-drop-1311701320.geojson)

## Automatic discovery and two-way traffic

`layouts.py` discovers junctions from shared OSM nodes, including interior way nodes. It splits two-way ways
locally into approach arms while retaining source IDs. Incoming and outgoing ports occupy right-hand-traffic
halves. Missing directional splits on two-lane roads are inferred as one lane each way. Excess mapped width
remains unallocated shoulder/parking pavement instead of becoming oversized driving lanes.

| Automatically resolved junction | Connections | Basis |
| --- | ---: | --- |
| Central, `2918219392` | 7 | Four mandatory-through restrictions; two-plus-one lane addition |
| Western merge, `13505628685` | 5 | Two directional signals; inferred lanes in a three-lane exit |
| Southern merge, `13433510068` | 4 | Two directional signals; shared exit, source-following bend |
| Eastern crossing, `61321054` | 3 | Two mandatory-through restrictions |
| Whittemore Avenue / Kassul Park, `61322243` | 6 | Two-way T, interior source-way node |
| Acorn Park Drive / Discovery Way East, `61332673` | 6 | Two-way T, interior source-way node |

Mouth distances derive from widths, signals, and available length. Neighboring candidates share a length budget
with a four-meter handoff allowance. Accepted domains are disjoint. Restrictions and stable topology ordering
determine precedence. Of 19 vehicle-junction candidates, six resolve and thirteen retain explicit reasons.

Two-way lane ribbons exclude same-arm U-turns, apply supported unconditional `no_*` and `only_*` restrictions,
and respect supported directional turn indications. Crossing/converging paths become conflicts, never extra
lane-graph transfers. These are geometric turn opportunities, not a complete legal-routing database.
Conditional, vehicle-qualified, and via-way restrictions require another policy; they are not simplified away.
Programming errors propagate; only explicit layout-precondition failures become unresolved-case diagnostics.

The original controlled merges still represent separately controlled streams sharing exit lanes. Signal phases
and priority remain unknown. A source-following fallback avoids cutting a lane path across an island.

- [Resolved/unresolved candidates](output/layout-discovery.json)
- [Whittemore/Kassul layout](output/junction-61322243-layout.json)
- [Acorn/Discovery layout](output/junction-61332673-layout.json)
- [Central junction](output/junction-2918219392-layout.json)
- [Western merge](output/junction-13505628685-layout.json)
- [Southern merge](output/junction-13433510068-layout.json)
- [Eastern crossing](output/junction-61321054-layout.json)

OSM distinguishes [lane indications](https://wiki.openstreetmap.org/wiki/Key:turn) from
[legal turn restrictions](https://wiki.openstreetmap.org/wiki/Relation:restriction).
Directional counts follow the [lanes documentation](https://wiki.openstreetmap.org/wiki/Key:lanes).

## Stop bars and crosswalks

Eleven directional signals have inferred 0.40 m solid bars, with source tags, positions, and placement offsets
retained. Maximum displacement is 0.132 m. The crop contains no separately mapped stop-line shapes or stop signs.
No stop control is invented at the two new T-junctions. A transient fixture verifies that a directional stop on
a two-way road marks only its incoming half. Signal phases, hardware, and actual paint presence remain unknown.

Sixteen tagged crossings yield five zebra, two explicit paired-line, and four generic marked crossings rendered
as inferred paired lines. Three explicitly unmarked crossings stay unpainted; two marked crossings lack a selected
vehicle road and remain unresolved. Orientation and dimensions are inferred. Footway alignment, curb ramps,
tactile paving, and surveyed placement remain open.

The viewer caught a centerline entering a crosswalk. All resolved white/yellow guidance now passes through shared
exclusions: crosswalk passage overlap is zero; numerical stop-exclusion overlap is below 0.001 square meters.
Bars remain separate from crossing passages. Two-way urban layouts omit isolated perimeter paint that would end
abruptly at their ownership boundaries.

- [Control evidence and inferred bars](output/traffic-controls.json)
- [Guidance exclusion checks](output/marking-audit.json)
- [Signal semantics](https://wiki.openstreetmap.org/wiki/Tag:highway=traffic_signals) and
  [explicit stop-line tagging](https://wiki.openstreetmap.org/wiki/Tag:road_marking=stop_line)

## Surface and verification

The approximately 782 by 356 m crop is `[-71.148, 42.3966, -71.1385, 42.3998]`. Full source ways extend beyond it.
Topology uses node identity, never projected line crossings. Highway lifts satisfy explicit bikeway clearance
constraints; other bridge lifts stay separate. A finite-element harmonic fit uses hard bridge-cap and underpass
constraints on connected pavement, avoiding smoothing across islands. Heights remain synthetic.

Geometry uses local metric calculations, double WGS84/ECEF conversion, and float glTF vertices relative to an
absolute anchor. A 1 mm overlay grid and conforming four-meter refinement control meshing. Paint clips to the
pavement triangles at an 18 mm offset. Terrain shares fitted boundary heights. Replacement maps use
`TerrainConstraint` with `has_elevation`, `remove_interior`, and minimum level 14; overlay maps omit the cutout.
Curbs, barriers, abutments, and the repeatable asphalt texture remain simple generated assets.

| Measurement | Fifth-pass result |
| --- | ---: |
| Selected OSM ways / bridges | 64 / 5 |
| Retained restriction relations / width overrides | 14 / 26 |
| Missing referenced road nodes | 0 |
| Preserved pavement holes | 13 |
| Pavement area | 38,351.90 square meters |
| Pavement triangles / total triangles | 75,704 / 261,509 |
| glTF vertex buffer | 25,104,864 bytes |
| Surface vertices / fixed vertices | 43,708 / 73 |
| Slope p95 / maximum on faces over 0.01 square meters | 5.23% / 9.36% |
| Highway deck clearances above bikeway | 5.248 m / 5.292 m |
| Exported float boundary disagreement | 0.017 mm over 11,744 samples |

The expanded crop adds two holes; the original eleven stay protected. Clearance checks cover overlapping rendered
triangles, including 0.7 m deck thickness and a 5 m experimental minimum. These are not surveyed dimensions or
design-standard compliance claims. One tunnel remains unrendered; restriction member way `168525335` is outside
the selected subset. The float boundary check measures export consistency, not runtime terrain cracking.

Transient tests verify reversed input order, right-hand traffic, reversed source-way direction, turn restrictions,
left/right drops, directional two-way stop bars, disjoint ownership, pavement support, and nine invalid cases.
T-junction minimum sampled path radii are 7.14 m and 8.70 m, above the 3 m prototype guard. This is not a vehicle
swept-path test. Connector endpoints coincide exactly and pass tangent and 1.8 m planar support-envelope checks.
Exported attributes are finite, normals are unit length, triangles have positive area, and 126 maps use absolute anchors.

- [Original junction after automatic discovery](output/pass5-intersection.jpg)
- [Measurements](output/metrics.json)
- [Layout/transition tests](output/next-pass-test-audit.json)
- [Export/boundary checks](output/seam-audit.json)

Viewer checks use the existing osgEarth 3.8.1 runtime in `a436`, without rebuilding it. Settled ground views compare
the T-junction at [SSE 2](output/pass5-two-way-street-sse2.jpg) and [SSE 32](output/pass5-two-way-street-sse32.jpg),
and the taper at [SSE 2](output/pass5-lane-drop-street-sse2.jpg) and [SSE 32](output/pass5-lane-drop-street-sse32.jpg).
The same-camera views remain coherent. A fresh compile reproduces geometry, boundaries, controls, and discovery
exports byte for byte. These checks do not prove moving-camera tile transitions or globe-scale behavior. The expanded crop
and new geometry are not a controlled performance comparison; no performance gain is claimed.

## Remaining work

Roads remain resident while terrain pages. Tile ownership, cross-tile junction assembly, coordinated road/terrain
residency, and distant raster coverage remain open. Paint subdivision is expensive and distant markings alias;
ordinary texture masks and LOD are the next rendering experiment, with measured quality and cost.

Layout coverage excludes uncontrolled acceleration merges, multiple simultaneous lane drops, mixed divided/two-way
turns, turn pockets, shared center turn lanes, roundabouts, conditional-policy evaluation, and junction clusters.
Unresolved junctions retain the earlier footprint approximation and suppress uncertain paint near the node.
Parking, sidewalks, curb ramps, regional marking rules, crossfall, surveyed heights, and detailed materials remain unfinished.

## Source and attribution

Map data: **Copyright OpenStreetMap contributors**, under the
[Open Database License](https://www.openstreetmap.org/copyright). OSM data/derived outputs keep their attribution
and data license; the repository code license does not replace it.

The original snapshot was retrieved 2026-10-09. The westward extract was retrieved 2026-10-10 from the
[OSM map API](https://api.openstreetmap.org/api/0.6/map?bbox=-71.158,42.397,-71.145,42.402).
`source/alewife-expanded.osm.gz` unions them by element type and ID, retaining the higher version. Coordinates,
IDs, versions, timestamps, tags, references, and relations remain; username, UID, and changeset attributes were removed.
[snapshots.json](source/snapshots.json) records source URLs, dates, hashes, and the union policy.
Combined SHA-256: `b084dba3cf57ed88e90034a76e9bff7204dc858288b3b66d690155fc6d49ae7c`.
Terrain/textures are deterministic procedural content. This is a bounded experiment, not a planet extract or
a photorealistic ground-to-globe road system.
