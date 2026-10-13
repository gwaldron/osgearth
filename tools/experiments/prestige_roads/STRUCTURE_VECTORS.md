# Separate structure vectors

The vector compiler now keeps bridges and roads with nonzero or uninterpreted `layer`/`level`
evidence out of the ground pavement union. It exports these roads separately, without requiring a
vertical solution. Missing heights are **unknown**, not zero. The original OSM tags are retained
verbatim; neither `layer` nor `level` is multiplied by an assumed story height.

Run `vector_patch.py` to regenerate the downtown Boston ground and structure datasets together.
Run `structure_vectors.py` for the supplemental 1.2 km South Station highway-ramp sample.
Both read retained source OSM anew. No terrain data, grading, custom terrain mesh, or 3D mesh is generated.

| File | Content |
| --- | --- |
| `structures.geojson` | Separate per-way pavement and supported lane-paint polygons, with material, source ownership, raw layer/level/bridge/incline tags, inferred width basis, and topology edge references. Coordinates are 2D longitude/latitude. |
| `road-topology-edges.geojson` | Source centerlines split at shared OSM nodes, with endpoint IDs, forward/reverse incidence, raw layer/level tags, access evidence, and original node IDs. Includes adjacent ground roads and tunnels for connectivity context. |
| `road-topology-nodes.geojson` | Shared-node incidence and incoming/outgoing edge references. A projected crossing without a shared source node is not a junction. |
| `structure-source-evidence.json` | Full selected structure tags, retained restriction/connectivity relations, tunnel records, and width overrides. |

`elevation_m`, `grade_percent`, `vertical_profile`, and `vertical_datum` are null and `height_status`
is `unresolved`. A subsequent elevation solver can populate station-based profiles without changing
surface identity or topology. Source `incline` and `ele` evidence remains in source tags; it is not
mistaken for a solved road alignment. Floor-level lists such as `0;2;4` remain lists in their raw string
form. A road with only a relative-layer tag is called `layered_road`, not assumed to be a bridge.

Topology uses source way direction. Unsupported direction values are marked unknown and get no
invented directed incidence. This is physical source topology, not a legal turn graph: access restrictions,
turn restrictions, and lane-connectivity relations still require movement resolution at elevated junctions.
The bounded extract can omit adjoining ways, so its node records do not claim complete external context.
Tunnels participate in source topology but do not receive surface polygons.

The existing ground lane layout and paint algorithms are unchanged. In the downtown sample, separating
18 additional context ways with level evidence preserves all 76 junction layouts, all 333 lane connections
(the lane GeoJSON is identical), 156 crosswalks, and 80 stop bars. The first structure export contains 28
roads and 82 material polygons. Structure paint remains inside its own pavement.

The supplemental South Station sample contains 87 structure ways: 27 elevated ramp ways, 49 other
bridge ways, and 11 roads with layer/level evidence but no explicit bridge tag. The 896 material
polygons span source layers 1, 2, and 3; all pass validity, two-dimensional-coordinate, null-height,
and graph-reference checks. Its broader topology context includes 95 shared nodes between bridge
and non-tunnel/non-bridge ways. These are source connections, not newly inferred turn movements.

This pass supplies structure ownership and topology, not resolved elevated junction surfaces, bridge
undersides, supports, railings, curb solids, or a complete elevated marking layout. Supported ordinary
lane stripes reuse the existing street-paint policy and are suppressed near unresolved branches.
Overlapping structure polygons are kept separate so later geometry generation can resolve them using
their ownership and connectivity. The existing overview/intersection scenes still preview ground roads;
loading all structure polygons at zero altitude would misrepresent this dataset.

Seven transient regressions outside the repository cover raw attribution, null heights, classification,
projected crossings, coincident but distinct nodes, shared interior nodes, reversed/unknown direction,
and the fresh Boston export's geometry and referential integrity. These are compiler/data checks;
there is no new 3D structure rendering claim in this pass.
