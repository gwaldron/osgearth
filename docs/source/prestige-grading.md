# Terrain grading

The Prestige component is named `GradingLayer`, with factory tag `prestige:grading`.
It is intended for roads, building footprints and, eventually, inland waterways. The current algorithms
include terrain-derived corridors, footprint-bounded road profiles, and level pads. Waterway grading remains future work.
The older core `osgEarth::Contrib::FlatteningLayer` keeps its existing name and behavior.

`osgEarthPrestige::GradingLayer` generates terrain-derived road corridors and
level pads from GIS features. It is independent of the older
`osgEarth::Contrib::FlatteningLayer`. Enable the Prestige nodekit, include
`<osgEarthPrestige/GradingLayer>`, and link `osgEarth::osgEarthPrestige`.

For a whole-earth example, run `osgearth_imgui prestige_grading.earth` from
`tests` after setting up `osgearth_shell.bat` on Windows. The sample streams
ReadyMap imagery, elevation, and OSM roads, and opens at Stevens Canyon near
Mount Rainier. It infers road widths from OSM tags and uses twenty-meter shoulders,
five-meter profile samples, and thirty-meter smoothing. Bridges, tunnels, footpaths, and
highway areas are excluded. A Terrarium elevation alternative starts closed.

In **Tools > Map**, toggle **Prestige road grading** to compare terrain, or
turn off **ReadyMap imagery** to inspect the shaded surface. **Tools > Viewpoints** selects
the road close-up, Rainier overview, equatorial roads, New Orleans, and globe.
The online OSM tiles contain clipped road fragments: the demo does not yet
guarantee continuous smoothed profiles across those source-tile boundaries.

The implementation also corrects the shared tile-layer fallback conversion for
regional source profiles. A regional DEM's local maximum LOD must be converted
to the requesting map's LOD convention before selecting a parent tile. Without
this conversion, zooming beyond the DEM's native resolution can replace terrain
outside a flattened corridor with an excessively coarse source tile.

Load the nodekit and place the flattening layer **after** the elevation layers it
should modify:

```xml
<map>
    <libraries>osgEarthPrestige</libraries>

    <GDALElevation name="Terrain">
        <url>terrain.tif</url>
    </GDALElevation>

    <OGRFeatures name="Road centerlines">
        <url>roads.gpkg</url>
    </OGRFeatures>

    <prestige:grading name="Drivable terrain" features="Road centerlines">
        <line_width>10</line_width>
        <buffer_width>20</buffer_width>
        <sample_spacing>10</sample_spacing>
        <smoothing_distance>30</smoothing_distance>
        <elevation_resolution>5</elevation_resolution>
    </prestige:grading>
</map>
```

In C++, set the options and feature source before opening/adding the layer:

```cpp
osg::ref_ptr<osgEarthPrestige::GradingLayer> layer =
    new osgEarthPrestige::GradingLayer;
layer->setFeatureSource(roads);
layer->options().lineWidth() = osgEarth::Expression<double>(10.0);
layer->options().bufferWidth() = osgEarth::Expression<double>(20.0);
layer->options().smoothingDistance() = 30.0;
map->addLayer(layer);
```

## OSM road widths

The optional `osgEarthPrestige::OSMRoadWidthFilter` preprocesses OSM line features
in the feature source, before its tile cache. It writes `oe_road_width` in meters
and `oe_road_width_source` indicating the selected tag or fallback. Both the
flattening width expression and the road stroke read the same numeric attribute;
they do not run separate JavaScript width policies. The filter leaves original
OSM tags and geometry unchanged and does not annotate polygon features.

```xml
<filters>
    <!-- Place after any script that selects the surface roads to render. -->
    <prestige_osm_road_width attribute="oe_road_width" lane_width="3.5" max_width="40"/>
</filters>
```

Set `line_width` to `feature.properties.oe_road_width` and `max_line_width` to
at least the filter's `max_width`. Set the road style's `stroke-width` to that same
attribute expression and `stroke-width-units` to `meters`.

The first valid result wins:

1. `width`, then `width:carriageway`, then `est_width`, then a complete
   pipe-separated `width:lanes` list. Bare widths are meters; recognized length
   units and feet/inches such as `16'3"` are converted to meters.
2. `lanes` multiplied by `lane_width` (default 3.5 meters). If the total is
   absent or invalid, use `lanes:forward` plus `lanes:backward` and optional
   `lanes:both_ways`. Both directions must be supplied, except for an explicitly
   tagged one-way road. The total count is already for the whole way and is
   never doubled for two-way traffic.
3. Road-class estimates: one lane width for links; 5 meters for service roads;
   4.5 meters for living streets; 6 meters for residential/unclassified roads;
   and two lane widths for other road classes. These are demo assumptions,
   not measured widths or regional engineering standards.

Nonpositive, nonfinite, ambiguous, or oversized width tags fall through to the
next source. Lane counts must be whole numbers; missing lane widths are not
silently filled. Final fallback widths are bounded by `max_width`. The filter
ignores `maxwidth`, which limits vehicle width rather than measuring the road.
See the OSM definitions of [width](https://wiki.openstreetmap.org/wiki/Key:width)
and [lanes](https://wiki.openstreetmap.org/wiki/Key:lanes).

This preprocessing is specific to OSM roads and is independent of the generic
flattening layer. Building and airport polygons use their own footprints, and
the terrain transition buffer remains a separate setting. Inferring a road
width does not select an elevation policy for river polygons or other features.

## Surfaces

* **Lines and multilines:** sample the existing terrain along the centerline,
  smooth the longitudinal profile with a distance-weighted triangular filter,
  and flatten across the road width. Internal segment grades blend continuously
  at bends; overlapping roads blend into a common junction. A quintic shoulder
  transitions back to natural terrain with zero first and second derivatives at
  either edge of the transition.
* **Polygons, rings, and multipolygons:** flatten each component to a single
  terrain elevation sampled inside its footprint. The interior-point search
  handles concavity and holes. The shoulder also extends into holes; the part of
  a hole beyond the shoulder retains its original terrain.
* **Points and multipoints:** create round pads, using `line_width` as the pad
  diameter and sampling the terrain at each point.

Mixed multipart geometry is supported. Feature Z coordinates are ignored by default, and
source features are never modified. Distances are ground meters even when the
source or output uses geographic coordinates, Mercator, or projected feet.
Road stations, polygon anchors, and source sampling resolution are independent
of output tile boundaries and LOD. Centerline vertices are retained; intervals
are subdivided to at most `sample_spacing` meters.

## Options

| Option | Default | Meaning |
| --- | --- | --- |
| `features` | required | Referenced or embedded feature source. |
| `road_footprints` | false | Require each feature to pair complete centerlines with pavement polygons. Polygons mask the sampled longitudinal profiles instead of becoming level pads. |
| `profile_elevation` | terrain | `terrain` samples existing elevation; `absolute` consumes solved centerline Z in road-footprint mode. |
| `profile_transition` | 0 | Absolute mode only: quintic fade into preceding elevation at both complete centerline endpoints, in meters; range 0–1000. Zero disables the fade. |
| `profile_offset` | 0 | Absolute mode only: numeric feature expression giving a receiver height offset in meters after datum conversion. Must be finite and within plus/minus 10,000 meters. |
| `junction` | false | Boolean feature expression selecting common junction-plane fitting in road-footprint mode. |
| `profile_id_attribute` | empty | Attribute containing ordered, comma-separated centerline IDs. Required when junction fitting is selected; all road features must supply one ID per line component. |
| `junction_transition` | 30 | Distance in meters outside a junction footprint over which connected profiles blend back to their own grades; positive and at most 1,000. This does not expand terrain coverage. |
| `line_width` | 10 | Full road width or point-pad diameter, in meters. Supports numeric feature expressions. Zero disables line corridors. |
| `buffer_width` | 20 | Shoulder width on each side, in meters. Zero creates a hard boundary. Supports numeric feature expressions. |
| `max_line_width` | 100 | Upper bound on evaluated width; used to retrieve nearby features. |
| `max_buffer_width` | 100 | Upper bound on evaluated shoulder width; used to retrieve nearby features. |
| `sample_spacing` | 10 | Maximum spacing between road stations, in meters; range 1–100. |
| `smoothing_distance` | 30 | Physical radius of the longitudinal filter, in meters; range 0–1000. Zero disables profile smoothing. |
| `elevation_resolution` | 5 | Requested source-elevation resolution in meters; falls back to available coarser terrain. Must be positive. |
| `min_level` | 12 for the default global profile | First output LOD. Set this explicitly for coarse polygon work. Custom regional profiles retain the inherited LOD convention. |

## Compiler-owned road footprints

With `road_footprints=true`, each feature contains a GeometryCollection of one or more complete LineStrings
and one or more associated Polygons. Multipart lines/polygons are accepted in the default terrain mode.
In that mode profile heights come exclusively from terrain sampling along those centerlines;
footprint vertices and feature Z never supply elevations.
Polygons retain their holes, and their boundaries define the grading shoulder. A feature missing either
centerlines or polygons is rejected instead of silently flattening the polygon to one elevation.

In this mode `line_width` specifies a conservative **profile search diameter**, not pavement width.
It must cover the polygon and shoulder from its associated centerlines. `max_line_width` bounds it for
feature retrieval. Within each footprint the profile fully controls elevation; only the polygon shoulder
blends to the base. Without the optional junction policy, multiple associated approaches blend their
longitudinal profiles; that can form ridges where differently oriented roads impose different cross-slopes.
Disconnected or elevated roads must not be assigned to a ground footprint merely because they cross in plan.

For a common junction surface, set `junction` to a Boolean feature expression and `profile_id_attribute`
to the attribute containing ordered centerline IDs. Junction features and their approach-road features
must reference the same IDs and complete alignments. The layer fits one plane to terrain samples along
the junction's centerlines, within two meters of its pavement footprint. Station spacing is at most five
meters, with length weights to reduce digitization-density bias. A small regularization term stabilizes
collinear inputs without inventing an unconstrained transverse slope. Fewer than three valid samples
leave the original road-profile behavior in place.

Every identified member profile uses that same plane inside the junction. Outside it, a quintic transition
returns to the profile with zero blend derivative at both ends, matching height and slope at the handoffs.
Nearby junction influence bands blend continuously, with core interiors taking precedence over bands.
Disconnected roads are excluded by profile identity, not inferred from proximity. The correction changes
profile height only: existing pavement masks, holes, shoulders, and sparse no-data remain authoritative.
All controls are built before tile sampling and owned by that paging job. Fits use complete source
geometry, so changing output tiles does not change the fit.

These are terrain-derived planes, not surveyed grades. They can inherit implausible slopes from coarse
urban elevation. Closely spaced junction cores and complex interchanges still warrant explicit inspection;
this policy does not solve a regional engineering-grade optimization or superelevation design.

The road experiment's `road_grading.py` adapter pairs compiler source-way corridors and accepted junction
approaches with unchanged pavement coverage. Rounded fill retains shared network ownership; its nearest
ground-centerline height association is recorded explicitly as inferred.

For one native grading level, set `min_level` and `max_data_level` to the same value and leave `max_level`
unset. The three city scenes use global-geodetic LOD 16 and 257-square grids, approximately 1.2 meters
north-south sample spacing. Higher-detail terrain resamples these native tiles. No-data cells allow the
underlying elevation layer to participate. A finer tile entirely covered by parent grading can report no
new detail, allowing the terrain pager to retain its already graded parent. The terrain's 17-square render
grids and the elevation source's 257-square grids use different resolution mappings; native source LOD
does not directly equal visible terrain LOD.

```xml
<prestige:grading name="Road profiles" features="Compiled road profiles">
    <road_footprints>true</road_footprints>
    <junction>[grading_junction]</junction>
    <profile_id_attribute>source_way_ids</profile_id_attribute>
    <junction_transition>30</junction_transition>
    <line_width>[profile_search_width_m]</line_width>
    <max_line_width>500</max_line_width>
    <buffer_width>5</buffer_width>
    <max_buffer_width>5</max_buffer_width>
    <min_level>16</min_level>
    <max_data_level>16</max_data_level>
    <tile_size>257</tile_size>
</prestige:grading>
```

Evaluated widths are clamped to the declared maxima. Increase those maxima when
using wider roads or shoulders; each maximum is limited to 10,000 meters.
Negative or nonfinite feature widths skip that feature. The inherited elevation
tile size is respected (2–1024 samples per side, default 257).

## Solved road profiles and bridge approaches

Set `profile_elevation=absolute` with `road_footprints=true` when an offline compiler has already solved
the longitudinal profile. Each feature must contain one complete, continuous LineString and its paired
polygon footprint(s). Centerline Z supplies elevations in the feature SRS's vertical datum; without a
vertical datum, Z means ellipsoid meters. The layer converts to ellipsoid heights before applying the
optional `profile_offset`. Polygon Z still has no meaning.

Absolute mode evaluates piecewise-linear height at the nearest centerline station. It does not resample,
smooth or refit the supplied Z values. Multiple line components, junction-plane fitting, invalid heights,
and coincident XY stations with different heights are rejected. Zero is a valid height. Complete source
geometry and source heights participate in duplicate detection. Supply a well-sampled local alignment;
individual segments longer than 100 km are rejected.

`profile_transition` fades the profile's influence at the complete alignment's endpoints, including
inside its footprint, so a bounded solved approach can rejoin terrain-derived roads. The polygon and
its shoulder still bound coverage. Outside the influence, cells remain no-data; shoulder and endpoint
transitions sample only the preceding elevation stack. This is an elevation overlay on the existing
regular grid, not a new terrain mesh.

The bridge experiment exports the same centerline XYZ for both the deck and approach grading. Only
the ground approaches and small landing support areas receive grading; the central span does not.
A separate `profile_offset` of -0.03 meters beneath the short deck overlap prevents coincident terrain
and deck surfaces. This receiver policy is recorded separately from the solved road height. It is not
a structural clearance or a blanket adjustment to the bridge geometry.

```xml
<prestige:grading name="Solved approaches" features="Paired absolute profiles">
    <road_footprints>true</road_footprints>
    <profile_elevation>absolute</profile_elevation>
    <profile_transition>10</profile_transition>
    <profile_offset>[profile_offset_m]</profile_offset>
    <line_width>80</line_width>
    <max_line_width>80</max_line_width>
    <buffer_width>5</buffer_width>
    <max_buffer_width>5</max_buffer_width>
    <min_level>16</min_level>
    <max_data_level>16</max_data_level>
</prestige:grading>
```

Keep stacked receivers separate. An elevated profile must not grade terrain along the entire span,
and a road beneath a bridge must retain its own profile and footprint. This mode consumes a solved
profile; it does not infer bridge clearances, solve elevated junctions, or assign water elevations.

## Paging and accuracy

The layer queries a geographic halo large enough for the configured maximum
footprint, including neighboring source tiles and antimeridian splits. Road
geometry is indexed in double-precision Earth-centered coordinates. Raster
blocks without intersecting constraints are rejected before individual sample
queries. Only centerline/pad elevations and pixels in shoulders require source
elevation samples. Fully flattened pixels do not need a natural-terrain sample.

Each paging job owns its geometry, spatial index, and elevation working set. It
samples only open elevation layers preceding this layer in map order; an empty
source stack returns no tile. Cancellation or a processing failure publishes no
partial tile. Invalid profile samples are not interpolated through, and
unaffected pixels remain `NO_DATA_VALUE` so the underlying elevation stack is
preserved. The elevation pool's own fallback and missing-data resolution still
apply before this layer sees samples.

Output heights are above the ellipsoid; do not configure an output `vdatum`.
Source elevation layers may use vertical datums, which the elevation pool converts.

Persistent and layer-local output caching are disabled because the result
depends on other layers' elevation and on feature content. The elevation pool
still reuses source tiles. Configure the layer before opening it. After editing
source features, invalidate affected terrain using the application's normal
terrain-refresh mechanism; this layer does not watch external files for edits.

## Current limits

* Continuous smoothed profiles across source-tile boundaries require complete,
  unclipped geometries. Distinct clipped fragments with a shared nonzero feature
  ID are retained; only matching XY geometry (including components and polygon
  holes) in an equivalent horizontal SRS is deduplicated. Fragments are not yet
  stitched into continuous profiles across their clipped endpoints.
* Road segments follow short ellipsoid-projected chords. Individual input edges
  longer than 100 km are rejected; densify them upstream. Polygon vertices must
  lie within 100 km of their first vertex. Polygon edges use that component's
  local tangent plane and are intended for regional footprints such as buildings
  and airports.
* A tile request is limited to one million source samples. Transform failures,
  sampling failures, unsupported geometry sizes, and exceeded budgets return an
  error tile. Cancellation returns no tile.
* Terrain is a single-valued height field. Bridges, tunnels, and stacked roads
  require separate geometry. This layer smooths the road profile but does not
  enforce an engineering maximum grade, vertical-curve radius, crown, banking,
  drainage, or intersection topology.
* Terrain tessellation must resolve the desired road width. A narrow corridor
  cannot become drivable if the rendered terrain mesh spans it with a few large
  triangles. Source DEM resolution and elevation-pool quantization also limit
  achievable vertical accuracy.

## Validation

On Windows x64, `RelWithDebInfo`, the transient Google Benchmark harness measured
complete uncached 257-by-257 tile generation over the same warm, in-memory affine
terrain source. Each road contained 90 ten-meter segments in a 1024-meter square
tile. Both layers used ten-meter roads and twenty-meter shoulders. The new layer
used its default thirty-meter profile smoothing. Source elevation was limited to
LOD zero so both implementations sampled the same terrain data.

Median wall times from five repetitions, with at least 0.8 seconds per repetition:

| Roads in tile | Legacy | Prestige, smoothing enabled | Reduction |
| --- | ---: | ---: | ---: |
| 1 | 4.19 ms | 2.31 ms | 45% |
| 10 | 14.32 ms | 10.70 ms | 25% |
| 40 | 51.27 ms | 32.59 ms | 36% |

These are CPU tile-generation workloads, not network or frame-rate measurements.
Different feature density, source I/O, and terrain resolution will change the
result. The source and output caches were treated separately: source tiles were
warm, but every measured output tile was generated again.

The transient correctness harness passed 6,563 assertions across 13 cases:
affine-road agreement with the legacy layer; cross-road flatness; profile
smoothing; missing data; zero shoulders; cancellation before and during work;
empty source stacks; concurrent requests; shared tile edges; antimeridian edges;
point pads; polygon holes; continuous bends; random indexed-versus-exhaustive
queries; projected-foot coordinates; a buffered feature outside a tile at 75N;
nondefault tile size; and factory/configuration round trips. Tests and benchmarks
remain outside the repository, as required by the project's transient-test policy.

A live OSM regression reproduces the reported seam near Stevens Canyon at
46.76056 N, 121.65161 W. At output LOD 14, duplicate-ID filtering previously
discarded a road fragment in one parent tile, leaving 12 shared-edge samples
without flattening. Retaining distinct fragments makes both the coverage and
heights agree exactly at those samples; the corresponding LOD 18 comparison
also agrees exactly. A separate in-memory regression checks that distinct
same-ID fragments survive while repeated complete geometry is still removed.

The full existing suite could not complete: it contains a duplicate Chonk test
name, and, after temporarily distinguishing that name, the existing `glTF converts
channels and factors to shared PBR materials` test crashes with an access
violation. That crash also reproduces when the PBR test is run alone. The
temporary test-name change is not part of this implementation.
