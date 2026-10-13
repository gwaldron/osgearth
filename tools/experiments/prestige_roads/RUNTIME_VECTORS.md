# Indexed road vectors: Sion runtime package

October 13, 2026. Sion now reads **spatially indexed FlatGeobuf** for ground materials and grading.
The original GeoJSON remains available for inspection. Road compilation, heights, bridge meshes, ReadyMap
imagery, shaders, texture formats, material passes, and coverage cache policy are unchanged.

## What was slow

The six active GeoJSON files total 294,315,843 bytes. Each OGR cursor opens its own dataset for thread safety.
GeoJSON does not provide the indexed random access needed for these repeated regional queries. Additionally,
the largest asphalt polygon contains 189,156 vertices, so querying any part retrieves the whole feature.

The material query halo exposed a second problem. A two-meter buffer expands the request into neighboring
tile queries, and the generic feature cursor concatenates their results without deduplicating features.
Large polygons consequently arrived and were painted repeatedly. Besides wasted reads and rasterization,
this increased fractional edge coverage. The new filled-polygon scenes use a zero-meter query buffer.
This is appropriate for complete polygon footprints with `coverage_outline=false`; it is not a policy for
stroked centerlines, symbols, or other geometry that extends beyond its feature bounds.

## Packaging

`runtime_vectors.py` adds an indexed companion package without rebuilding the road design. Polygon features
over 2,048 vertices are intersected with a global-geodetic LOD-16 grid. Its boundaries coincide with whole
pixels of the existing 512-square coverage tiles at LOD 15 and above. Holes and disconnected pieces are
preserved. Disconnected pieces from the same source within a cell remain a compound polygon, avoiding
multiple alpha-compositing operations within one coverage pixel. The maximum asphalt feature is now 4,854
vertices. Small polygons, including all white/yellow marking features, retain their original coordinates.

Grading features are **not subdivided**. Their complete XYZ centerlines, footprint masks, offsets, and other
attributes remain paired in their original GeometryCollections. The binary index selects the relevant
features without changing profile construction. New runtime provenance attributes identify the original
feature and any fragment; indexed-file row numbers are not used as source road identity.

The package contains 843 asphalt, 21,800 white-paint, 1,830 yellow-paint, 3,534 curb, 3,703 gutter, and 5,385
grading features. Twenty-five empty asphalt features are omitted from the index and recorded explicitly;
they remain in the GeoJSON. The six binary files total **57,045,096 bytes**, about 81% less than the originals.

Packages live under `output/sion-city/runtime-vectors/`, in an immutable directory keyed by input and generator
hashes. Export verification checks decoded binary geometry, every non-null source attribute, source-to-fragment
coverage, and exact known XYZ coordinates. Validation reads WKB directly: writing validation geometry back
through GDAL's GeoJSON writer rounded some tiny valid slivers, despite the binary geometry being unchanged.
The original source hashes are checked again before switching the earth files. `city_network.py compose`
now creates and activates this companion package after the urban composition stage.

## Measured native tile-generation cost

Google Benchmark, three iterations per repetition and three repetitions; median real time. The harness uses
the production SurfaceSplattingLayer/FeatureImageLayer rasterizer and GradingLayer with feature L2 caching
disabled **before open**, and bypasses generated-tile caches. The OS file cache is warm. Material measurements
use the LOD-19 tile at the Sion street viewpoint. Grading uses the LOD-16 tile at the same location with a
deterministic base elevation, isolating vector delivery and grading computation from ReadyMap latency.

| Tile operation | Previous scene | GeoJSON, single query | Indexed runtime |
| --- | ---: | ---: | ---: |
| Asphalt | 4,097 ms | 431 ms | 9.44 ms |
| White paint | 1,715 ms | 190 ms | 6.02 ms |
| Yellow paint | 269 ms | 62.9 ms | 6.16 ms |
| Curb | 624 ms | 603 ms | 5.71 ms |
| Gutter | 654 ms | 659 ms | 5.86 ms |
| Grading | 639 ms | 639 ms | 78.0 ms |

The single-query control separates the data-format/subdivision improvement from removal of duplicate
queries and painting. For example, indexed asphalt is about **46 times faster than that control**, and
grading is about **8.2 times faster**. These are specific tile-generation measurements, not a claim about
whole-scene launch time, frame rate, or every location. Curb and gutter already had no explicit query halo;
their two GeoJSON columns are repeat measurements of equivalent settings.

## Correctness and visible behavior

* All nonempty decoded geometries are valid; subdivision preserves source footprints to floating-point
  clipping precision, with no coordinate quantization or simplification.
* Eighty real 512-square R8 tile comparisons cover asphalt, both paints, curbs, and gutters at four locations
  and LODs 15, 16, 19, and 21. With one query in both paths, only 93 of 20,971,520 pixels differ, each by one
  coverage step out of 255. All 32 paint tiles are byte-identical. Near-view asphalt tiles are also identical.
* Eight native grading tiles compare 528,392 samples: elevations and no-data cells are exactly unchanged.
* The actual ReadyMap-plus-grading scene repeats all 2,242 previous terrain samples with **zero elevation
  difference**. Minimum sampled road-crossing clearance remains **5.282771 m** against the 5.25 m target.
* Transient synthetic tests exercise holes and adjacent-cell preservation at Sion, NYC, and London coordinates.

The intentional visible correction is removal of the old duplicate-draw edge inflation; the coverage
comparison does not pretend that overpainting was correct. Asphalt boundaries and marking footprints remain
the compiler's polygons. No raster cache was enabled to hide generation costs. Coverage still begins at
LOD 15, and ReadyMap/network loading, terrain construction, and deck loading remain separate costs.

The updated NVGL/ICO scene was checked at the overview and the junction close view with Prestige sky and
four-sample antialiasing. The overview shows the city network, and the close view retains crisp turn arrows,
stop bars, stripes, curbs, and rounded corners. Captures are in `output/sion-city/captures-runtime-overview/`
and `captures-runtime-street/`. The interactive viewer is reopened on the updated overview.

Evidence: `output/sion-city/runtime-validation.json`, `runtime-benchmark.log`, `runtime-vectors.json`, and the
selected package's `manifest.json`. Test/benchmark harnesses remain outside the repository under the temporary
`prestige-runtime-vectors-validation` directory. No engine C++ or public ABI changed in this work.

## Reproduce

With the experiment Python dependencies available:

```powershell
python tools/experiments/prestige_roads/runtime_vectors.py `
    tools/experiments/prestige_roads/output/sion-city --activate
tools/experiments/prestige_roads/view_interchange.ps1 -Scene sion-city -View overview
```

Omit `--activate` to generate and validate a package without changing scene URLs. The exporter locates the
existing GDAL converter in this worktree, or accepts `--ogr2ogr` explicitly. The earth files enforce the
512-square, LOD-15-or-finer, exact-fill assumptions used for the subdivision boundary checks. GeoJSON is
still the inspection/debug output; FlatGeobuf is the rendering and grading input.
