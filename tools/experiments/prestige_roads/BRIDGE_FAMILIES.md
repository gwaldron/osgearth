# Procedural bridge families and supports

The regional compiler plans bridge structures against the exact joined deck triangles before exporting railings.
It then emits decks, paint, remaining generic railings, and the accepted structural members.
Sion exercises mapped beam, truss, and cable-stayed attribution. The original road topology, deck footprints,
fat merges, and ground material coverage are retained. Structural members do not modify terrain.

## Source contract

`bridge_evidence.py` reads the retained source snapshot, including bridge outlines, complete multipolygon
relations, support objects, waterways, and railway corridors. Road ways and bridge outlines can both carry
`bridge:structure`. A unique outline must cover at least 90% of the road centerline and have a compatible
layer before supplying attribution. Conflicting families, overlapping outline associations, incomplete
relations, and clipped spans remain explicit diagnostics.

Sion's source includes:

* `way/1041055516`: a cable-stayed bridge outline containing both motorway carriageways, ways 155292030
  and 155292032. They share one tower/cable assembly rather than receiving duplicate towers per carriageway.
* `way/155292027`: the truss-tagged Passerelle de Vissigen cycleway.
* Beam outlines including Pont Sainte Marguerite and the eastern road crossing.

Untagged bridge ways receive an **inferred generic beam** family. Inferred raised approaches are not
automatically treated as bridges. Layer numbers retain their ordering meaning; they never become heights.
Mapped support locations and attributes are retained, but their exact association with multiple decks is
deferred. A mapped support that needs that association blocks generic replacement of its asset.

The tag interpretation follows the OSM documentation for
[bridge structure](https://wiki.openstreetmap.org/wiki/Key:bridge:structure) and
[bridge supports](https://wiki.openstreetmap.org/wiki/Key:bridge:support).

## Implemented templates

| Family | Basic geometry | Deliberate limits |
| --- | --- | --- |
| Beam / generic | Piers and end bearings beneath the existing slab | No deeper girder that consumes existing clearance |
| Truss | Parker polygonal top chords, Pratt webs, inclined end posts and deck gussets on side girders | Parker is an inferred default; Pratt variant also supported; no overhead cross-bracing |
| Arch | Paired through-arch ribs and hangers | Through-arch is explicitly an inferred variant, not implied by the tag |
| Cable-stayed | Paired towers, direct stays and anchorage blocks seated in slab-connected edge girders | Tower layout and all dimensions are inferred |
| Suspension | Paired towers, main cables and vertical hangers | Basic visual template, not a reconstruction of a landmark |

Floating, simple-suspension, unsupported types, movable mechanisms, strongly curved/branched spans, and
ambiguous support associations defer rather than silently acquiring the wrong structural family.
Arch and suspension pass transient synthetic geometry checks; they are not claimed as validated real-city
reconstructions in this Sion pass.

`bridge_families.py` attaches members to the exported deck triangles. Retained ReadyMap elevation provides
illustrative support bases with a half-meter embed. Candidate foundations avoid mapped water and railway
corridors. All members are checked against other roads' height/clearance corridors before emission.
The generator searches a bounded set of alternate pier/tower stations; it records rejected placements.
Both tower legs must succeed, otherwise that cable family is deferred atomically.

The base DEM includes some existing bridge surfaces and does not cover every outer Sion location retained
in this experiment. Supports with missing terrain or insufficient resolved height are omitted explicitly.
This is not a water-depth, foundation, abutment-wall, or retaining-wall solver.

## Sion result and inspection

The current pass emits 1,050 semantic member segments across 24 assets: one cable-stayed assembly, one
truss assembly, and 22 beam/fallback assets. The cable assembly has 4 tower legs, 2 crossbeams, 66 stays,
66 anchorage blocks, 676 edge-girder segments, and one accepted end bearing. The truss assembly has
60 edge-girder segments, 116 truss/chord/gusset members, and two end bearings. Another 22 assets are
deferred or have no supported placement. Segments along one girder are not separate physical girders.

The detailed reasons are in `output/sion-city/bridge-families.json`; source evidence and inferred dimensions
remain separate. The members add 9,680 triangles, while removing duplicate generic railings reduces the
complete scene from 571,331 to 545,303 triangles. Member centerlines and dimensions are inspectable in
`bridge-members.geojson`.

Review scenes:

* `output/sion-city/bridge-cable-stayed.earth`
* `output/sion-city/bridge-cable-anchors.earth` (close attachment view)
* `output/sion-city/bridge-truss.earth`
* `output/sion-city/bridge-supports.earth`

Launch through `view_interchange.ps1 -Scene sion-city -View bridge-cable-stayed`; the launcher initializes
osgEarth and uses NVGL, ICO, Prestige sky, and four samples. Google imagery and indexed FlatGeobuf coverage
remain in use. The original Sion views remain available.

## Family-owned deck edges

The inferred truss default is **Parker**, a Pratt arrangement with a polygonal top chord. Web diagonals
slope down toward midspan, and inclined end posts replace the former square-ended, alternating-web frame.
A parallel-top Pratt variant uses the same attachment system. This distinction follows
[NCDOT's historic truss descriptions](https://www.ncdot.gov/initiatives-policies/Transportation/bridges/historic-bridges/bridge-types/Pages/truss.aspx).
OSM maps the truss family here; the Parker subtype and all member dimensions are explicitly inferred.

Continuous side girders overlap each carriageway slab. Shared miter rings and end caps only at the two
ends of each sweep prevent cracks and overlapping interior caps between segments. Truss lower chords and gusset plates seat in these
girders. Cable stays terminate in separate anchorage blocks that overlap the concrete edge girder, with
one accepted block per stay. Each carriageway, including its median edge, gets its own longitudinal edge;
no member fills a divided-deck median. Girder axes sit 0.15 m outboard, their width is 0.8 m, and they overlap
the slab nominally by 0.25 m. Their bottom is 0.08 m below the deck and their top is 0.92 m above it.
These are visual template dimensions, not mapped or engineered dimensions.

**Persistent compiler policy:** an accepted truss or cable family owns its structural deck edges. Generic
railings are suppressed only where the replacement girders exist. Short transitions at the landings keep
the existing edge treatment. The end search is bounded to six meters and retains road-clearance exclusions.
An unsupported shape, failed primary member, or obstructed replacement defers the family atomically and
keeps its original generic railings. Arch-specific edge treatment remains a future extension.

Planning and emission share one triangulation, so structural attachments reference the same surface that
produces the deck materials. This pass removes 1,469.97 m of duplicate generic railing across the two Sion
assets. Five previous end stays are omitted because those endpoints lie outside the common supported
interval of the staggered carriageways; every retained stay has a seated anchor. The towers are unchanged.

`bridge-edge-validation.json` records the current checks. All 186 pavement/paint/underside/fascia mesh
primitives and the 38 unrelated railing meshes are bit-identical to the preceding delivery. Deck profiles,
grading profiles, and approved fat-merge vectors are byte-identical. The indexed runtime package and Google
imagery are retained without regeneration. All exported truss feet and stay anchors pass independent seating
checks, and member road/rail/water exclusions pass. Synthetic checks also exercise Parker and Pratt web
orientation, staggered divided decks, failure fallback, and preservation of unrelated approach railings.
The original Boston deck still regenerates byte-for-byte.

The prior 2,440 terrain samples remain valid because both the sampled grading and the crossing slab geometry
are unchanged. They were compared again to the current decoded meshes; minimum clearance remains 5.291 m.
No new terrain survey or native sampling run is claimed for this edge-only pass.

## Smooth cable-span profiles

The structural pass initially preserved every existing deck byte. Close inspection then exposed four and
five longitudinal slope reversals in the two cable-stayed carriageways: the previous profile followed
interior DEM variation across the river. `bridge_profiles.py` now derives a smooth profile for complete,
unbranched, mapped cable/suspension spans before the terrain receivers and decks are generated.

Both landings remain fixed. A common broad crown is solved across the physical bridge's carriageways,
with tangent end transitions and a straight central span. The crown is the minimum needed to satisfy
full-width road-crossing clearance with the existing interpolation allowance. Internal graph connections,
mutually stacked candidate spans, infeasible clearance, or excessive grade defer to the accepted network
profile. The original solve files remain intact; `bridge-profile-fairing.json` records the derived heights,
before/after station samples, attribution, dimensions, and endpoint locks.

For Sion, the common crown adds 1.105 m above the endpoint chords; the two maximum grades are 4.97% and
5.15% in the landing transitions. Both original endpoints remain exact. Dense profile samples retain at
least 5.3299 m below the 0.8 m slab at the affected mapped road crossing. These are inferred visual road
profiles, not surveyed bridge elevations or river clearances.

Only these two deck profiles and their two grading-apron rows change. The five urban coverage inputs and
all unrelated rendered deck meshes remain byte-identical; FlatGeobuf grading is repackaged from the updated
rows. The fat-merge polygons are unchanged.

![Longitudinal profiles](output/sion-city/bridge-profile-comparison.png)

## Validation and remaining work

Transient regressions outside the repository cover all five templates, outward member winding and volume,
finite/nondegenerate render triangles, water and road exclusions, conflicting attribution, unsupported and
movable types, profile wave removal, endpoint locks, foreign-road preservation, crossing clearance, and
infeasible/internal-junction deferral. The actual saved meshes are decoded independently to check member
corridors, normals, preserved footprints, deck paint, rail attachment, and stacked-road clearance.

`bridge-family-validation.json` records the member checks. `joined-export-validation.json` and
`terrain-validation.json` record the final mesh and native composed-terrain checks separately; their status
is authoritative. All 44 decoded-mesh crossing checks pass, with a minimum of 5.319 m; all 2,440 native
composed-terrain samples pass, with a minimum of 5.291 m. The prior network-blocked terrain recheck is
therefore resolved for this package. Captures are retained under `captures-bridge-families`.

Next detail passes can resolve mapped support association, pier caps and bearings, girders for beam bridges,
below-deck arch variants, arch-specific edges, landmark replacements, and water-aware foundations.
The package exporter also now initializes PROJ data explicitly and validates the saved WGS84 coordinate
reference. A GDAL conversion can exit successfully after failing to load its coordinate database; checking
geometry bytes alone did not catch that case. The viewer launcher supplies the same PROJ data path.

Regular-grid grading remains a separate component. No custom terrain mesh, virtual texturing, or arena
refactor is introduced here.
