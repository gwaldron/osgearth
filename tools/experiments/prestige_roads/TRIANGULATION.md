# Boston road polygon triangulation correction

2026-10-12. The GeoJSON polygons were intact, but osgEarth's default Earcut adapter narrowed their
double coordinates to floats before triangulating. For small geographic features, the polygon builder
selects a dominant plane from Earth-centered coordinates. Float spacing at those coordinate magnitudes
can exceed the width of a paint strip, collapsing vertices and producing missing or tapered triangles.

The correction in [Tessellator.cpp](../../../src/osgEarth/Tessellator.cpp) changes the two coordinate
accessors for `osg::Vec3d` to return `double`. Earcut already computes with doubles. The source GeoJSON,
road compiler, terrain, and rendering configuration are unchanged. There is no public ABI change.

![Corrected native road polygons](output/boston-vectors-current/earcut-fixed-native.jpg)

## Validation

A transient C++ reproduction places a 10 cm strip at large coordinate offsets. Before the correction,
one placement generated no triangles; after the correction it retains both triangles. A second fixture
passes all 2,307 Boston polygons through the native polygon builder and records the emitted vertices
and triangle indices. Tests and their temporary build inclusion were removed from the repository afterward.

An independent Shapely audit compares the union of those triangles with each source polygon in local
coordinates. All 2,307 pass with a 0.1 mm boundary allowance for the float draw vertices: maximum missing
interior area is numerical noise, approximately 1.6e-15 square meters, and outside area is zero.
The [saved footprint audit](output/boston-vectors-current/triangulation-footprint-audit.json) records the result.

The focused run passed 16 test cases, covering the new reproduction, actual Boston polygons, existing
feature geometry tests, and coordinate-system tests. The Windows build succeeded using `build.bat`.
A full unit-suite attempt exited in the elevation-pool portion without a test summary; its cause was
not diagnosed in this change, so this report does not claim a full-suite pass.

## WeMesh comparison

The existing WeMesh branch was tested against the same source polygons and inspected in osgEarth.
The comparison scene enables it with `render-max-tess-angle: 0.05`, whose default unit is degrees.
Neither this comparison nor the corrected default uses OSG's triangulator.

The table measures the sum of absolute per-polygon differences between source area and emitted triangle
area, in square meters. It is a geometric comparison, not a rendering-performance benchmark. Small residuals
include float draw-coordinate precision; the independent footprint audit above is the coverage check.

| Material | Original Earcut adapter | Corrected Earcut adapter | WeMesh comparison |
| --- | ---: | ---: | ---: |
| White paint | 183.9441 | 0.000112 | 0.2024 |
| Yellow paint | 416.9274 | 0.01193 | 0.02587 |
| Curbs | 1308.1604 | 0.04859 | 0.28230 |
| Gutters | 1325.6880 | 0.05855 | 0.59643 |
| Pavement | 447.1860 | 0.08107 | 23.05046 |

The corrected Earcut path remains the default for this preview because it preserves these footprints
more closely. This is a bounded comparison of the existing WeMesh settings, not a general judgment of
WeMesh for other meshing tasks. The alternative scene remains available for inspection.

- [Original screenshot](output/boston-vectors-current/triangulation-before.png)
- [Corrected default scene](output/boston-vectors-current/intersection.earth)
- [WeMesh comparison scene](output/boston-vectors-current/compare-wemesh-intersection.earth)
- [WeMesh native capture](output/boston-vectors-current/wemesh-native.jpg)

Launch the corrected default with `view_vectors.ps1 -View intersection` after building the updated engine.
The vector inputs remain the fresh output from `vector_patch.py`; regeneration is not required for this fix.
