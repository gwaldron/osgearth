# Three road-rendering fixtures

October 12, 2026. These are the input and vector-geometry baselines for the
[Prestige rendering plan](../../../docs/design/prestige-roads-rendering-plan.md).
The new asphalt splatting, marking backend, and generated bridge renderer are later stages.

## Regions

Each area is an exact 2,000 m by 2,000 m square in a local WGS84 azimuthal-equidistant projection.
The geographic export densifies its edges before reprojection; a degree-based bounding rectangle is
only a source-query envelope, not the render crop. The local square has an area of 4,000,000 square meters.

| Fixture | Center (longitude, latitude) | Source-backed bridge coverage |
| --- | --- | --- |
| Central Boston | -71.0565, 42.3515 | Fort Point bridges and the South Station highway ramps |
| Central NYC, Lower Manhattan | -73.9950, 40.7095 | Brooklyn and Manhattan bridges, FDR Drive and Brooklyn approaches |
| Central London | -0.1130, 51.5060 | Westminster, Waterloo and Blackfriars bridges |

Lower Manhattan gives the NYC fixture connected river crossings and approaches within the requested
small footprint. Some bridge approaches continue outside the square. Source ways retain their complete
returned node sequences; material output alone is clipped to the AOI.

Boston uses the retained Massachusetts source snapshot. NYC and London use nine serialized, bounded
OpenStreetMap map-API requests each, covering a 500 m halo outside the square. Source manifests record
request URLs, response hashes, retrieval times, object counts and the merged snapshot hash. Contributor
identifiers are removed. These are merged current object versions, not an atomic historical snapshot.
The compiler selects a 250 m working halo while retaining complete selected source ways.

## Reproduce and inspect

Use the experiment Python environment with its existing NumPy, Shapely and pyproj dependencies:

```powershell
python tools/experiments/prestige_roads/city_fixtures.py fetch boston nyc london
python tools/experiments/prestige_roads/city_fixtures.py compile boston nyc london
```

`fetch` retains existing source snapshots. `compile` replaces every geometry checkpoint for the selected
fixtures using current compiler code. A build-in-progress marker prevents the launcher opening partial
output. Successful manifests record both source and compiler hashes.

```powershell
./tools/experiments/prestige_roads/view_vectors.ps1 -Fixture boston -View overview
./tools/experiments/prestige_roads/view_vectors.ps1 -Fixture nyc -View bridge
./tools/experiments/prestige_roads/view_vectors.ps1 -Fixture london -View intersection
```

Each fixture has `overview`, `intersection` and `bridge` views. The launcher initializes
`osgearth_shell.bat` and uses the installed runtime with the corrected double-precision Earcut adapter.
The original 900 m Boston preview remains available with `-Fixture current`.
For empty optional materials, the launcher writes an `inspection-*.earth` display scene that omits those
layers. This avoids OGR's invalid-extent error for empty feature collections while retaining valid empty
GeoJSON files and the original generated scene. The viewport request is 1200 by 1000 with four MSAA samples.

| Output | Boston | NYC | London |
| --- | --- | --- | --- |
| Manifest | [Boston](output/render-fixture-boston/manifest.json) | [NYC](output/render-fixture-nyc/manifest.json) | [London](output/render-fixture-london/manifest.json) |
| Source inventory | [Boston](output/render-fixture-boston/source-inventory.json) | [NYC](output/render-fixture-nyc/source-inventory.json) | [London](output/render-fixture-london/source-inventory.json) |
| Vector overview | [Boston](output/render-fixture-boston/overview.svg) | [NYC](output/render-fixture-nyc/overview.svg) | [London](output/render-fixture-london/overview.svg) |
| Independent checks | [Boston](output/render-fixture-boston/validation.json) | [NYC](output/render-fixture-nyc/validation.json) | [London](output/render-fixture-london/validation.json) |

## Package and interpretation

* `aoi.geojson` records the crop and local projection. The manifest records locale, cameras, source,
  compiler version fingerprints, diagnostics and material counts.
* `asphalt.geojson`, `curb.geojson`, `gutter.geojson`, `white.geojson` and `yellow.geojson` contain the
  existing ground polygons. Empty color files are valid; London does not inherit US yellow centerlines.
* `structures.geojson` retains the existing combined structure/material packaging with source tags,
  surface IDs, raw levels, graph references and explicitly unknown elevations and grades.
* `road-topology-edges.geojson` and `road-topology-nodes.geojson` preserve source-node incidence.
  `lane-connectivity.geojson` contains separately inferred at-grade lane movements.
* Source, layout and marking evidence remain separate from inferred geometry. Unresolved junctions and
  partial boundary relations remain in diagnostics; the package does not claim every layout is solved.

The viewer shows gray ground pavement and blue-gray structure footprints on a flat reference globe.
This is a 2D diagnostic; its display ordering is not physical elevation or a solution for stacked decks.
The small display offsets belong only to the preview style. Structure GeoJSON coordinates remain 2D.
The `preview-structure-*.geojson` files are derived display subsets; `structures.geojson` is authoritative.

The general wrapper now carries the selected locale through junction construction, street paint,
directional stop bars and structure stripes. London uses left-hand traffic and white centerline defaults.
Country-wide marking completeness, red-route rules, box junctions and bridge-specific controls are not
claimed by these inherited prototype policies.

Ground material unions still lack final per-surface ownership. These fixtures establish reproducible
inputs and baseline geometry; they do not complete the Stage 0 render contract. Ownership attribution
must be added from the compiler's road/junction domains before the new rendering backends consume it.
Terrain grading and the separate Rebuild FlatteningLayer integration remain deferred.

## Validation results

All three fixtures compiled afresh and passed the independent export audit.

| Result | Boston | NYC | London |
| --- | ---: | ---: | ---: |
| Resolved junction layouts | 359 | 254 | 295 |
| Lane connections | 1,700 | 1,117 | 1,493 |
| Rendered crosswalk records | 478 | 795 | 245 |
| Inferred stop bars | 257 | 344 | 215 |
| Ground material polygons | 8,986 | 9,565 | 12,033 |
| Structure material polygons | 891 | 2,237 | 298 |
| Separate structure ways | 119 | 128 | 87 |
| Source-tagged bridge ways in AOI | 96 | 107 | 46 |
| Two-way lane ports checked | 918 | 578 | 1,276 |
| Unresolved junction-node records | 477 | 278 | 673 |

Ways are OSM segments, not counts of physical bridges. The structure category also contains source-level
evidence without a bridge tag. Unresolved node records are not directly comparable to layout counts:
compound layouts may cover several source nodes. Most unresolved records describe closely spaced junctions
requiring combined ownership. Reserved-lane, turning, lane-drop and other unsupported cases remain explicit.
NYC and London each retain two incomplete boundary restriction relations; Boston has none in this selection.

The independent audit reprojects exported GeoJSON into the declared metric frame. It checks AOI dimensions,
validity, crop boundaries, paint containment within its own pavement, structure/ground separation, unresolved
heights, unique graph IDs, cross-file references and lane placement by driving side. Geographic round-trip
boundary checks allow 1 mm; the compiler's existing local containment/exclusion area gate remains 0.001 m2.
All source and Python compiler hashes match the completed manifests. London exports no inferred US yellow
centerlines on either ground or structures.

A fresh rebuild of the earlier 900 m Boston preview produced byte-identical output for all five ground
materials, lane connectivity, structures, topology edges and topology nodes. Seven existing transient
structure/evidence/topology regression tests also pass. Test harnesses remain outside the repository.
No core rendering code changed in this fixture setup; the earlier triangulation precision correction is retained.

All three overviews and junction close-ups were inspected in osgEarth on the RTX 5080 runtime. Additional
close views checked the Congress Street bridge approach and London's mapped turn-lane arrows and stop bars.
The London supplemental camera is recorded in
[marking-inspection-camera.json](output/render-fixture-london/marking-inspection-camera.json).
This is a static vector baseline, not a terrain-attachment, frame-time, or dynamic-LOD acceptance test.

| Viewer capture | Boston | NYC | London |
| --- | --- | --- | --- |
| Overview | [Boston](output/render-fixture-boston/overview-viewer.png) | [NYC](output/render-fixture-nyc/overview-viewer.png) | [London](output/render-fixture-london/overview-viewer.png) |
| Junction | [Boston](output/render-fixture-boston/intersection-viewer.png) | [NYC](output/render-fixture-nyc/intersection-viewer.png) | [London](output/render-fixture-london/intersection-viewer.png) |
| Extra detail | [Congress Street bridge](output/render-fixture-boston/bridge-viewer.png) | — | [Mapped turn lanes](output/render-fixture-london/markings-viewer.png) |

The next implementation work is ground surface/marking ownership in the minimal render contract, followed
by the single-asphalt Prestige splatting experiment on the existing regular terrain. The same three fixtures
will then be used for marking-backend comparisons and bridge landing tests.

OpenStreetMap contributors, ODbL. Dimensions and some markings are inferred prototype values.
