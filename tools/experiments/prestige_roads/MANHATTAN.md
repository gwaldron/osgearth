# Prestige Roads: Manhattan urban intersections

The city stress-test pass retains the roughly 490 by 461 meter Tribeca patch and adds source-backed
lane connectivity at Greenwich / North Moore. The compiler now emits **29 layouts with 120 directed lane
connections**: fourteen three-way, fourteen four-way, and one five-way. **One layout remains provisional**
because OSM does not explain its two-to-one lane reduction. Its representative turn allocation remains an
explicit, unpainted hypothesis. The [five-city report](CITIES.md) describes the shared compiler changes.

The `pass7-` images below document the preceding pass. Current exports and the new `mapped-lanes` view
come from the city pass; the original seven cases retain their connectivity.

![Leonard / Varick / West Broadway, pedestrian-height viewer capture](output/manhattan/pass7-five-way-street-sse2.jpg)

![Five-arm junction, aerial viewer capture](output/manhattan/pass7-five-way.jpg)

These are actual osgEarth captures. The roads have synthetic elevations and inferred widths/curb returns.
The gray block interiors and sidewalk bands provide inspection context; buildings, parking, cycle tracks,
and surveyed sidewalk geometry are not modeled. This is an intersection-layout experiment, not a surveyed
reconstruction or a production traffic-routing network.

## Run

Use the Python environment and runtime setup in [README.md](README.md). From the repository root:

```powershell
python tools/experiments/prestige_roads/compile.py --scene manhattan
tools/experiments/prestige_roads/run.ps1 -RuntimeRoot C:/Users/gwald/.codex/worktrees/a436/repo `
    -Scene manhattan -View five-way -SSE 8
```

Use `five-way`, `fork`, `inferred-turn`, `turn-lanes`, `three-way`, or `four-way`; each has a `-street`
variant. `overview` shows the patch, and every emitted node also has a `junction-<node ID>` view.
There are 288 maps: 48 viewpoints, two terrain modes, and SSE 2/8/32. The launcher initializes `osgearth_shell.bat` before
running the installed viewer. No engine C++ changes or rebuild are required.

Close the experiment viewer before regenerating its inputs. Default Manhattan output is
`output/manhattan/`; `--output` can select another compilation directory. The launcher uses the default
directory. Alewife remains the default `--scene`/`-Scene` and retains its own outputs.

## The seven candidates

| Junction / OSM node | Result |
| --- | --- |
| Duane fork, `42430521` | One input lane branches to two one-lane exits; two alternative paths, no invented arrows or priority |
| Duane / West Broadway, `42430557` | Full approach context resolves the former crop failure; five lane paths |
| Hudson / North Moore, `42436336` | Full approach context; six lane paths, with unused receiving lanes explicitly reported |
| North Moore / West Broadway, `42445700` | Full approach context; three lane paths |
| Leonard / Varick / West Broadway, `42448162` | Five arms, seven lane paths and nine conflict pairs; mapped through arrow retained |
| Franklin / West Broadway, `42453059` | **Provisional:** four lane paths using the unmapped turn hypothesis below |
| Greenwich / Reade, `5812723036` | Full approach context; two lane paths |

The four boundary cases needed a larger render crop, not fabricated connectivity. The retained source
snapshot already contained the necessary context; no new download or junction-specific configuration was added.
The original eighteen junctions retain numerically identical lane paths. Across the full previous 28-layout
scene, 27 path sets remain identical; Duane / West Broadway retains its assignments but now uses the corrected
mouth allowance at a non-junction tag boundary.

The fork policy is deliberately bounded: one unindicated incoming lane and two outgoing single-lane arms
in the forward turn family. These are alternative routes from one lane, not two simultaneously occupied
input lanes. Multi-lane forks or ambiguous mapped arrows require another policy. Physical mouths separate
before independent cross sections are formed; overlapping ownership still fails explicitly.

At the five-arm junction, Varick's two lanes continue into West Broadway's two lanes, and its unindicated
right lane can turn onto Leonard. The separate West Broadway approach can continue or turn onto Leonard;
Leonard's incoming lane can continue or turn onto West Broadway. Those seven paths create nine crossing or
converging conflict pairs. Conflicts retain separate graph identities; signal timing and priority are unknown.

![Five-arm lane paths; colors are diagnostics, not road paint](output/manhattan/junction-42448162-plan.png)

At Franklin, West Broadway's mapped count changes from two lanes to one with no `turn:lanes` explanation.
The provisional layout lets the left lane leave onto Franklin and the right lane continue. This preserves
lane order without fusing coincident endpoints. It is **not evidence of a legally mandatory left turn**:
a missing turn tag, a real lane drop, or inaccurate counts remain possible. The exported
`allocation_inferences` records the source counts, receiving ways, unknown legal exclusivity, and missing
arrow evidence. No inferred left-turn arrow is painted. Explicit through indications, no side exit, and
larger unexplained reductions still reject this hypothesis. Source confirmation is required before treating
this particular allocation as ground truth or using it for traffic routing.

![Duane fork, viewer capture](output/manhattan/pass7-fork.jpg)

![Provisional Franklin layout; diagnostic paths](output/manhattan/junction-42453059-plan.png)

## The mapped turn-lane case

At **Varick and Franklin**, node `42453038`, the incoming Varick way `68675820` has three lanes and
`turn:lanes=left|through|`. The outgoing Varick way `420810621` has two lanes. The compiler produces:

| Incoming Varick lane | Assigned exit | Evidence and inference |
| --- | --- | --- |
| 1, leftmost | Franklin lane 1 | Mapped left indication; inferred receiving lane and curve |
| 2 | Varick lane 1 | Mapped through indication; inferred ordered assignment |
| 3, rightmost | Varick lane 2 | Unindicated lane; inferred continuing connection |

Two additional connections carry Franklin's incoming lane straight across and right onto Varick.
The compiler renders the mapped left and through arrows. It does **not** invent an arrow for the empty
third entry. The two continuing lanes remain separate; this is a lane leaving at a turn, not a three-to-two merge.

At **North Moore and Varick**, node `42445702`, the exit gains a lane whose downstream tag is `left||`.
The inferred allocation sends the two continuing input lanes into output lanes 2 and 3, leaving the new
left-turn lane distinct. This is a preference based on downstream indications, not a legal restriction.
The export explicitly reports the newly unfed output lane. Longitudinal entry into a new turn pocket and
lane-changing connectivity between junctions remain future work; no complete block-to-block lane graph is claimed.

![Lane paths at Varick and Franklin](output/manhattan/junction-42453038-plan.png)

Colored paths in this engineering figure are diagnostic lane centerlines, not painted guidance stripes.
The ordinary intersection box is left clear of invented internal lane paint.

## Layout policy

`urban.py` builds directional lane ports on three- through five-arm ground-level junctions at a single OSM node. It splits OSM ways at
interior junction nodes and follows compatible degree-two tag splits without discarding source IDs.
Explicit `lanes:forward`/`lanes:backward` values define two-way allocations. An unqualified even total is
split evenly as an explicit prototype assumption; ambiguous odd totals are rejected.

Lane numbering follows travel direction, left to right. Mapped left/right/through and compound indications
select rendered lane guidance. Unindicated edge lanes receive plausible turns; continuing groups retain
order. The bounded two-to-one hypothesis above is recorded separately; other unexplained reductions remain unresolved.
Simple unconditional via-node `only_*`/`no_*` restrictions are processed separately. This particular crop
has connectivity relations retained from the surrounding source but no applied turn-restriction relation
at its 29 emitted junctions; restriction behavior is checked with transient fixtures. Four via-node
connectivity relations now specify the four lane connections at Greenwich / North Moore. Parenthesized
lane-change possibilities are retained separately by the new parser; they do not create default paths.

Urban turns use tangent-continuous straight/arc/straight paths. The solver searches for a radius that fits
the rounded pavement, with a 3.5 m centerline-radius floor, then validates lane endpoints, tangent continuity,
lane ordering, and a 1.8 m passenger-car-width envelope. Through paths use cubics. Different incoming lanes
from the same approach cannot cross each other on their way out. Alternatives from the same input lane are branches; crossing or
converging streams from different approaches produce conflict records without extra graph connections.
These checks do not substitute for a vehicle wheelbase/overhang swept-path model.

Pavement starts with source-width footprints and inferred four-meter curb returns. The remaining fifteen
enclosed block interiors are protected. Supported lane ribbons and shared approach collars determine the
local replacement surface, which is fitted and exported with the same boundary used for terrain constraints.

## Controls, crosswalks, and paint

The expanded patch retains 107 crossing nodes and 29 traffic-control records. It renders **75 tagged marked
crossings** and **45 inferred stop bars**. Explicitly unmarked crossings stay
unpainted. Unsupported or missing marking styles also stay unpainted, but uniquely associated crossing
passages still reserve space so stop bars and lane paint do not obstruct them.

Shared-node junction signals provide junction-level control evidence. Applying that evidence to all incoming
approaches is recorded as an inference; a directional tag on a node shared by several ways does not uniquely
identify a signal head. Separately mapped directional stops retain their approach direction. Ambiguous or
unassociated controls remain in the export without guessed bars. No signal phase, priority schedule, signal-head
asset placement, or separately surveyed stop-line position is invented.

Bars are illustrative 0.40 m strokes, placed upstream of crossing passages on the incoming lanes only.
Their source nodes, per-approach scope, displacement, and inferred placement are exported independently.
Arrows move upstream as whole glyphs until they clear crossing and stop exclusions; the compiler does not
clip their heads to make them fit. Directional separators use white dashes and double yellow at the boundary
between opposing traffic. Geometry is draped onto the final mesh with the existing experimental 18 mm paint offset.

## Coverage and remaining cases

Of 32 candidates in the expanded crop, 29 emit layouts, including the provisional allocation above.
Three boundary candidates remain unresolved: nodes `42429829`, `42431617`, and `4143851142` reach beyond
the render crop. Greenwich / North Moore (`42436056`) now uses its four mapped lane-connectivity relations,
producing four connections and two inferred stop bars. Source assignments are explicitly distinguished from
inferred geometric paths in [its export](output/manhattan/junction-42436056-layout.json).
Crop ownership is checked before curve fitting so boundary failures are not misreported as bad curb geometry.
These are distinct from the original seven candidates.

Outgoing lanes without direct incoming connectors remain explicitly listed as `unfed_outgoing_lanes`.
Longitudinal turn-pocket entry and lane-changing connectivity between junctions remain open. Missing lane
counts, parking/cycleway allocation, larger or multi-lane clusters, reserved/conditional lanes, complete
connectivity-relation handling, and swept-vehicle design need additional policies. Slight/sharp arrow
indications currently share the ordinary left/right illustrative glyph and movement family.

The model is resident glTF. Terrain pages; the road geometry does not yet page or transition to a distant bake.
There is no virtual-texturing dependency. This pass makes no rendering-performance improvement claim.

## Verification and artifacts

Transient tests cover real-source lane assignments, reversed source enumeration, reversed digitization,
asymmetric directional counts, compound/empty turn indications, unconditional restrictions, a four-way fixture
with two lanes per direction, and a geometric dual-left-turn fixture. Follow-up checks cover all original
seven candidates, unchanged paths at the original eighteen, alternative fork branches, five-arm mouth
separation/conflicts, and the absence of invented arrows at Franklin. Fourteen deliberately invalid cases are
rejected, including reserved lanes, conflicting counts, conditional restrictions, malformed arrow arrays,
and an unexplained three-to-two drop. Tests remain outside the repository as requested.

The export checker independently reads glTF and verifies finite attributes, unit normals, positive triangle
areas, and agreement between rendered float positions and georeferenced pavement boundaries. The existing
Alewife geometry/restriction tests still pass. Its fresh city-pass compile retains six layouts, 31 connections,
and the 180 m taper; both deck clearances remain above 5 m. Bridge interfaces and collars now use the shared
city-pass surface improvements, so the complete Alewife mesh is no longer a byte-identical historical artifact.

Current export: **196,991 triangles**, 18,911,136 vertex-buffer bytes, and 15,712 boundary samples.
Maximum float boundary error is 0.0162 mm. All lane endpoints match exactly; the maximum car-width envelope outside pavement is zero. Final marking overlap with crossing passages and stop exclusions is zero.

Five-arm street views at terrain SSE 2 and 32 were inspected from the same camera. They show consistent road surfaces,
curbs, and markings. This is a static viewpoint comparison, not a measured moving-camera paging or popping test.

- [Original seven: before/after and provisional status](output/manhattan/seven-candidates-audit.json)
- [Geometry and compile metrics](output/manhattan/metrics.json)
- [Source records and tags](output/manhattan/source-records.json)
- [Layout discovery and unsupported cases](output/manhattan/layout-discovery.json)
- [Varick/Franklin lane layout](output/manhattan/junction-42453038-layout.json)
- [Mapped controls and inferred bars](output/manhattan/traffic-controls.json)
- [Marking exclusions](output/manhattan/marking-audit.json)
- [Transient test results](output/manhattan/urban-test-audit.json)
- [Current glTF and boundary verification](output/manhattan/city-export-audit.json)
- [Previous-pass viewer inspection record](output/manhattan/viewer-audit.json)
- [Fine-terrain five-arm capture](output/manhattan/pass7-five-way-street-sse2.jpg)
- [Coarse-terrain five-arm capture](output/manhattan/pass7-five-way-street-sse32.jpg)

Generated output and screenshots are ignored by Git. Source snapshots and compiler code are retained.

## Source and tag semantics

The retained `source/manhattan.osm.gz` snapshot came from the
[OSM map API](https://api.openstreetmap.org/api/0.6/map?bbox=-74.012,40.714,-74.003,40.722) on 2026-10-10.
It retains node identities, complete referenced ways, and relations; contributor-identifying `user`, `uid`,
and `changeset` attributes were removed. [Provenance](source/manhattan-source.json) records the URL, date,
and SHA-256 `931575fba322c569675f6925bf9c5adb1ee1926cf3b2eca327e3d457efaae8b9`.
Render crop: `(-74.01125, 40.7162, -74.00545, 40.72035)`; source context extends farther.
Data copyright OpenStreetMap contributors, [ODbL attribution/license](https://www.openstreetmap.org/copyright).

Tag interpretation follows OSM's primary documentation:
[lane counts](https://wiki.openstreetmap.org/wiki/Key:lanes),
[turn indications](https://wiki.openstreetmap.org/wiki/Key:turn),
[restriction relations](https://wiki.openstreetmap.org/wiki/Relation:restriction),
[lane connectivity](https://wiki.openstreetmap.org/wiki/Relation:connectivity), and
[traffic signals](https://wiki.openstreetmap.org/wiki/Tag:highway%3Dtraffic_signals).
In particular, empty/`none` lane indications do not prohibit a movement, and arrows are not a substitute for
legal restriction relations. Shared junction signals and precisely positioned directional signals are distinct
source representations; the export keeps those distinctions and the resulting assumptions visible.
