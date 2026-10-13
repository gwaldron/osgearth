# Prestige Roads: five-city compiler stress test

2026-10-10. This pass uses fresh retained OSM extracts from downtown Boston and Pittsburgh, London's
Bloomsbury/Holborn area, Paris around Bastille and the Marais, and central Sion. The compiler produces
**121 layouts and 540 private-car lane connections**, including two compound intersections. Those layouts
cover 123 of 299 candidate source nodes. There are **424 rendered marked crossings, 171 inferred stop bars,
and 11 inferred BUS words**. The [physical lane-role follow-up](LANE_ROLES.md) records the latest changes;
the first sweep established a baseline of 105 layouts and 480 connections.
The final control correction recovers three contraflow bars in London/Paris and withholds three ambiguous
Pittsburgh bars; the total remains 171. Details and source IDs are in the lane-role report.
Unsupported cases retain diagnostic reasons and their source evidence.

All scenes have controlled synthetic heights. These are experiments in road topology, lane layout,
surfaces, and markings, not surveyed reconstructions. Buildings, parking allocation, and cycle tracks
are not modeled. No engine C++ changes, virtual texturing, or osm2streets dependency were introduced.

![Boston: mapped turn arrows and inferred approach stop bars](output/boston/city-turn-lanes.jpg)

## Coverage

| Scene | Selected road ways | Initial layouts | Current layouts | Lane connections | Marked crossings | Inferred stop bars |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Boston | 137 | 24 | 26 | 114 | 60 | 48 |
| Pittsburgh | 129 | 11 | 14 | 81 | 95 | 43 |
| London | 290 | 25 | 32 | 140 | 57 | 35 |
| Paris | 238 | 7 | 33 | 121 | 165 | 36 |
| Sion | 114 | 10 | 16 | 84 | 47 | 9 |

Initial counts are a diagnostic baseline, **not a correctness benchmark**. The initial London run used
right-hand assumptions, and several initial layouts in Europe silently treated bus lanes as car lanes.
The compiler now separates physical lane roles from car eligibility, resolving source-supported allocations
and diagnosing ambiguous ones. A compound layout owns two source
junction nodes, so layout count and resolved-node count differ. Layouts still include explicitly exported
geometric inferences; acceptance does not certify traffic legality or swept-vehicle clearance.

The independent [export checks](output/london/city-export-audit.json) inspect the actual glTF buffers,
active Earth files, source records, and road/terrain boundaries. Each scene has its own `city-export-audit.json`,
`metrics.json`, `layout-discovery.json`, and `traffic-controls.json` under `output/<scene>/`.
[Combined checkpoint](output/city-stress-summary.json) and [viewer inspection record](output/city-viewer-audit.json)
record the exported results and distinguish final captures from earlier topology-pass images.

## Compiler changes driven by these extracts

**Traffic side and physical lane width.** London revealed hard-coded right-hand placement in lane ports,
center separators, and fallback stop bars. The scene now supplies a traffic side, with explicit per-way
overrides retained. Lane numbering remains left-to-right in the direction of travel. European scenes use an
inferred broken white center separator; this is a bounded visual style, not a complete national marking standard.
A mapped narrow lane gets its driving width before optional edge margins are allocated.
[OSM driving-side semantics](https://wiki.openstreetmap.org/wiki/Key:driving_side)

Sion also exposed 2.5-meter streets being expanded to the prototype two-lane default. Narrow mapped widths
can now imply one shared physical lane, with two opposed graph directions and no invented center divider.
This is recorded as an inference; passing priority is unknown. Explicit one-lane two-way ways receive the
same shared-pavement treatment. Source lane counts and inferred counts remain distinguishable.

**Access and reserved lanes.** Paris frequently uses `motor_vehicle=destination;permit`. Rejecting every
such way prevented physical layout generation. The compiler now retains those access conditions while
building geometry for authorized traffic. It does not turn them into public car routes. Conversely,
`lanes:psv`, `psv:lanes`, busway variants, and other vehicle-qualified lanes no longer silently become
ordinary car lanes. Their role allocation remains unresolved.
[Access hierarchy](https://wiki.openstreetmap.org/wiki/Key:access),
[reserved PSV lanes](https://wiki.openstreetmap.org/wiki/Key:lanes:psv)

**Restriction scope and source connectivity.** Unconditional restrictions are evaluated for a private
passenger car. Known vehicle-specific rules and exceptions are retained separately; bicycle-only rules do
not block a car layout. Relevant time conditions and unknown qualifiers still fail explicitly. A same-way
`no_u_turn` no longer removes ordinary continuation along that way. Roundabouts honor their implicit
one-way direction, although a general roundabout layout is still outside this pass.
[Restriction relations](https://wiki.openstreetmap.org/wiki/Relation:restriction),
[roundabout direction](https://wiki.openstreetmap.org/wiki/Tag:junction%3Droundabout)

The new `connectivity.py` parser gives supported via-node lane assignments precedence over geometric
inference and turn indications for that road pair. Parenthesized destinations are retained as lane-change
possibilities, not rendered as default paths. Contradictory restrictions, out-of-range lanes, conflicting
relations, and many-to-one default drops fail explicitly. This resolves Greenwich / North Moore in the
Manhattan regression scene: four source connectivity relations produce four connections and two additional
inferred stop bars. Manhattan now has **29 layouts, 120 connections, 75 marked crossings, and 45 stop bars**.
[OSM connectivity semantics](https://wiki.openstreetmap.org/wiki/Relation:connectivity)

**Tag boundaries and compound ownership.** A short source-way end is not necessarily a neighboring
intersection. A mouth can now use that approach up to a retained two-meter handoff collar instead of
arbitrarily taking only 42 percent of its length. True neighboring junctions retain the shared length budget.

`clusters.py` adds a bounded two-node solver for two close T-junctions, with one unindicated lane per
direction. External ports keep their original source-node roots. The solver traces directed reachability
through the internal source corridor and applies restrictions at both nodes before fitting geometry.
It preserves islands, rejects ownership overlap, and checks each path against its permitted source corridor.
This does not collapse neighboring nodes into an unconstrained all-to-all intersection.

- London: Red Lion Street / Eagle Street / Sandland Street, nodes `11544695` and `338501923`.
  Twelve connections; minimum sampled path radius 8.06 m.
- Sion: Avenue des Mayennets / Rue des Cedres and the short residential access opposite it,
  nodes `289356179` and `9308961940`. Twelve connections; minimum sampled path radius 5.62 m.
  The short northern street really ends at a mapped footway; its visible stub is not an invented continuation.

[London compound layout](output/london/junction-cluster-11544695-338501923-layout.json),
[Sion compound layout](output/sion/junction-cluster-289356179-9308961940-layout.json)

![Sion: compound intersection with yellow crosswalks](output/sion/city-compound-sse2.jpg)

**Crossings and controls.** London contains 50 nodes tagged with dotted crossing boundaries. That style is
now supported; 47 of those nodes have sufficient unambiguous geometry to render in this crop. Sion retains
explicit yellow crossing colors, recognizes legacy zebra tags, and uses a labeled Swiss color/pattern
inference where the source only confirms that paint exists. Explicit color tags take precedence. Unmarked
crossings stay unpainted; crossing passages remain reserved against stop bars and lane paint. Divider paint
also respects `lane_markings=no` inside resolved approaches.
[Crossing styles and colors](https://wiki.openstreetmap.org/wiki/Key:crossing:markings),
[legacy zebra tags](https://wiki.openstreetmap.org/wiki/Tag:crossing%3Dzebra),
[Swiss marking references](https://www.astra.admin.ch/de/signale)

A close London view caught a crossing on a harmless source-way split being rejected as ambiguous.
Two compatible end-to-end carriageway pieces can now supply one crossing cross-section, with both source
IDs retained. Branches, lane/width changes, sharp bends, and level changes still require explicit resolution.
This recovered 21 crossings across Boston, London, and Paris. One London stop bar is now withheld because
the newly recognized passage overlaps it; the exclusion remains explicit rather than painting over the crossing.

![London: dotted crossing boundaries after compatible way splits were joined](output/london/city-crossings-after.jpg)

**Surface robustness.** Boston found a plotting crash on non-polygon boundary fragments. Pittsburgh and
Sion found steep harmonic-fit spikes where a bridge was tied to only a few mesh vertices. Six-meter,
topologically connected approach collars spread those ties over pavement. Compatible degree-two bridge
interfaces also share their endpoint cross-section tangent. Twenty-two tested interfaces have zero
cross-section edge mismatch. Nearby roads without a source-node connection are not selected as approach collars.

The collars reduced Sion's failed surface maximum from 30.07 percent to 5.98 percent and Pittsburgh's
from 24.12 percent to 14.85 percent. The remaining Pittsburgh concentration led to a curvature penalty on
the connected pavement mesh. The fit minimizes height residual gradient, mass-weighted curvature, and
fidelity to the initial profiles, while keeping deck and underpass ties exact. The default curvature weight
is 4 square meters; setting it to zero reproduces the previous harmonic fit. The 15-percent rejection guard
was not relaxed.

Pittsburgh's final rebuilt maximum is **7.97 percent**; Sion's is **5.89 percent**. A transient comparison on
the same retained Pittsburgh mesh reproduces the improvement independently of layout changes. Tests also
check unchanged XY topology, exact hard ties, isolated components, planar terrain, and triangle-order
independence. The two Alewife deck-clearance regressions remain unchanged above 5 m (5.248 m and 5.292 m).
[Surface regression results](output/surface-curvature-tests.json)

![Pittsburgh: corrected bridge approach at street height](output/pittsburgh/city-surface-street-sse2.jpg)

The automatic `surface-detail` and `surface-street` cameras select the steepest significant pavement face
when its grade exceeds four percent. Pittsburgh's direct street view was inspected at terrain SSE 2 and 32;
both show a consistent approach. This is a static terrain-detail comparison, not a moving-camera paging test.

## Verification and limitations

Transient tests cover mirrored and shared lane geometry, tag-boundary ownership, fallback stop-bar sides,
access hierarchy, vehicle scope, explicit lane connectivity, malformed/conflicting source data, compound
reachability, one-way internal links, restriction propagation, and input-order independence. Existing turn,
merge, lane-drop, and original-seven-candidate checks pass. At the first-sweep checkpoint, Pittsburgh's ground and layout geometry also
matched across independent processes with different Python hash seeds; that probe predates lane-role support. Tests remain outside the repository.

All five city exports have finite attributes, unit normals, positive triangle areas, and matching boundary
vertices after float conversion. The maximum measured boundary error is below **0.022 mm**. The original
18 Manhattan paths remain numerically identical. During the first sweep, 27 of the preceding 28 path sets
remained identical; Duane / West Broadway kept its connectivity with a corrected tag-boundary mouth.
The subsequent role pass preserves all 29 Manhattan path sets and all 104 surviving first-sweep city layouts
exactly. It adds 17 layouts and withdraws one unsupported London allocation, as documented in the
[role comparison](output/lane-role-regression-comparison.json).

The largest remaining issues are bus movements, conditional policies, and ambiguous lane allocation; multi-lane or larger compound
junctions; longitudinal entry into turn pockets; general roundabouts; full vehicle swept paths; and production
elevation/crossfall fitting. All measured grades still use controlled synthetic elevations. The five city scenes
do not certify general bridge clearances: the existing explicit deck-clearance regression still covers the
Alewife highway/bikeway case. Roads remain resident meshes; moving-camera paging and globe-scale LOD are untested.

## Reproduce

Use the Python dependencies and installed runtime described in [README.md](README.md). Source snapshots
are retained, so compiling needs no network request.

```powershell
python tools/experiments/prestige_roads/compile.py --scene london
python tools/experiments/prestige_roads/compile.py --scene sion --survey
tools/experiments/prestige_roads/run.ps1 -RuntimeRoot C:/Users/gwald/.codex/worktrees/a436/repo `
    -Scene london -View compound -SSE 8
```

Other scene names are `boston`, `pittsburgh`, `paris`, `sion`, `manhattan`, and `alewife`. `--survey`
produces discovery diagnostics without meshes. `fetch_source.py <scene> ...` retrieves missing snapshots
sequentially and leaves existing snapshots intact. The launcher initializes `osgearth_shell.bat` and only
accepts views listed by the latest compile's `viewpoints.json`, avoiding stale maps from earlier runs.
Close the viewer before recompiling its scene. Useful views include `turn-lanes`, `four-way`, `three-way`,
`compound`, `mapped-lanes`, `reserved-lanes`, `crossing-dots`, `crossing-yellow`, and `surface-detail`, with
`-street` variants when listed for that scene.

## Retained source provenance

All five source extracts were retrieved on 2026-10-10. Complete returned way/node references and relation
members are retained; contributor user/uid/changeset attributes were removed. The rendering crops are smaller
than the source requests. Metadata records both bounds and element counts. Data copyright OpenStreetMap
contributors; [ODbL attribution and license](https://www.openstreetmap.org/copyright).

- **Boston:** [retained source](source/boston.osm.gz), [retrieval metadata](source/boston-source.json), [OSM request](https://api.openstreetmap.org/api/0.6/map?bbox=-71.0620000,42.3502000,-71.0510000,42.3574000). SHA-256 `07a3ccc441e4ea2f2845b717814f223ebb1f456783c5578e10a08e4e17f2e228`.
- **Pittsburgh:** [retained source](source/pittsburgh.osm.gz), [retrieval metadata](source/pittsburgh-source.json), [OSM request](https://api.openstreetmap.org/api/0.6/map?bbox=-80.0080000,40.4387000,-79.9960000,40.4460000). SHA-256 `e4ec93c35d6c8473d025fb25ecd8a6a4b2f9d9271bc4fdc6569dbb33a7740c21`.
- **London:** [retained source](source/london.osm.gz), [retrieval metadata](source/london-source.json), [OSM request](https://api.openstreetmap.org/api/0.6/map?bbox=-0.1255000,51.5140000,-0.1125000,51.5220000). SHA-256 `9a693a5c09c1b26dd9a8920ae8db639b9d573149699374503ae734e31bdfa421`.
- **Paris:** [retained source](source/paris.osm.gz), [retrieval metadata](source/paris-source.json), [OSM request](https://api.openstreetmap.org/api/0.6/map?bbox=2.3585000,48.8495000,2.3725000,48.8575000). SHA-256 `4af359ccc413a6541386f13f516164308b4ae100e804f3ec49fb31a150851c0c`.
- **Sion:** [retained source](source/sion.osm.gz), [retrieval metadata](source/sion-source.json), [OSM request](https://api.openstreetmap.org/api/0.6/map?bbox=7.3535000,46.2275000,7.3675000,46.2365000). SHA-256 `139bd09b4967950bd4448220ee0408f3301bfb23df9182ed5b0df3b9e57f651c`.
