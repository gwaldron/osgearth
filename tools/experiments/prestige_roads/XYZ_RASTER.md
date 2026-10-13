# Boston road imagery: LOD 16, 3 km radius

The raster experiment covers a 3,000 meter geodesic radius around longitude -71.080, latitude 42.350.
It reuses the complete compiled road geometry from the earlier 10 km square as surrounding context.
The previous LOD 14 tiles and their PNG intermediates have been deleted.
It bakes surface-road pavement, full supported markings, and shaded curbs into transparent 512 by 512 pixel
DDS tiles for an ordinary `XYZImageLayer`. The scene displays those tiles over ReadyMap imagery and elevation,
without grading the terrain.

**Complete, 2026-10-11:** all 451 LOD 16 tiles pass file, circular-crop, and georeferencing checks.
The overview, Back Bay, and close junction views were inspected in osgEarth over ReadyMap imagery/elevation.

![Native osgEarth overview of the circular road overlay](output/boston-metro-raster/native-lod16-overview.jpg)

[The portable atlas](output/boston-metro-raster/boston-lod16-atlas.png) decodes the actual DDS tiles onto a
neutral background. [The native-resolution image detail](output/boston-metro-raster/boston-lod16-intersection.png)
provides a closer look at the compressed paint and curb bands without the underlying aerial imagery.

## Tile layout and compression

The pyramid uses osgEarth's `global-geodetic` profile: two tiles across and one down at LOD 0, EPSG:4326,
with XYZ rows counted from the north. Each LOD doubles both dimensions. The pixel sizes below are at the
experiment's center; longitude and latitude have different ground scales.

| LOD | Tiles in region envelope | Approximate pixel size east by north |
| --- | ---: | ---: |
| 16 | 451 | 0.442 by 0.596 m |

Only LOD 16 is produced. Each pixel uses 32 by 32 coverage samples, with material priority resolved before
filtering premultiplied color and alpha. Bounded subwindows keep temporary memory small. Paint fades with
its pixel coverage instead of being widened. All five materials are cropped at subpixel resolution to the
same circular footprint. The circle uses
2,048 boundary segments with less than 0.0036 m radial error; its boundary vertices lie at a geodesic distance
of 3,000 m. Only tiles intersecting the circle are emitted, including transparent water tiles.

Every DDS contains BC3/DXT5 blocks, straight alpha, and a complete ten-level mip chain. Each file is 349,680
bytes including its 128-byte header. The 451-tile LOD 16 slice occupies 157,705,680 bytes (150.40 MiB).
Tiles are precompressed; no runtime texture encoder is required. Visible tiles are paged normally.
The existing pavement, lane layouts, markings, and curbs are rasterized again directly at the finer resolution.
The replacement tile bake took 255.8 seconds with eight workers; recompiling the source geometry was unnecessary.

The encoder corrects Pillow's compressed linear-size header and explicitly encodes nearest-palette BC3 alpha
blocks. This preserves fully transparent and fully opaque pixels and avoids the alpha distortion found in
the initial encoder probe. [Pillow documents its DDS encoding support](https://pillow.readthedocs.io/en/stable/handbook/image-file-formats.html#dds).

## Layer configuration

Place this layer after the background imagery. The relative URL assumes the earth file sits beside `tiles`.

```xml
<XYZImage name="Prestige Boston surface roads">
    <url>tiles/{z}/{x}/{y}.dds</url>
    <profile>global-geodetic</profile>
    <tile_size>512</tile_size>
    <min_level>16</min_level>
    <max_data_level>16</max_data_level>
    <invert_y>false</invert_y>
    <read_options>dds_flip</read_options>
    <texture_compression>none</texture_compression>
    <cache_policy usage="no_cache"/>
    <attribution>OpenStreetMap contributors / ODbL; road dimensions and paint geometry inferred</attribution>
</XYZImage>
```

`max_data_level` limits source data while allowing LOD 16 to remain visible when zoomed in further.
Setting `max_level` instead also limits visual range and hid the roads in the close native probe.
The overview explicitly selects `lod_method=screen_space`, `tile_pixel_size=16`, zero additional screen-space
error, and terrain `max_lod=16` so LOD 16 is requested while the whole circle is visible. Close views allow
finer terrain and inherit the LOD 16 imagery. This corrects the earlier overview, which set pixel size without
selecting screen-space subdivision. It does not add coarser road tiles.

`dds_flip` converts standard top-down DDS storage to OSG's texture convention. It does not invert the XYZ
row addresses. Without it, neighboring tiles had individually mirrored contents and broken road joins.
This behavior is explicit in the [OSG 3.6.5 DDS reader](https://github.com/openscenegraph/OpenSceneGraph/blob/OpenSceneGraph-3.6.5/src/osgPlugins/dds/ReaderWriterDDS.cpp).
`texture_compression=none` disables an additional encoding pass; osgEarth preserves images already compressed.

## Road compilation and evidence

The retained extract comes from the dated
[Geofabrik Massachusetts OSM download](https://download.geofabrik.de/north-america/us/massachusetts.html).
The local extraction retains 69,891 ways, 284,854 referenced nodes, and 1,089 restriction/connectivity relations,
including 250 meters of source context. Source versions, tags, and references are retained separately from
inferred widths and lane allocation. The snapshot and its hashes are in `source/boston-metro.osm.gz` and
`source/boston-metro.osm.json`.

The road reader selects 23,256 surface ways in the buffered context and excludes 923 explicitly tagged
bridge/tunnel ways. It retains the compiler's supported highway classes, including service roads and cycleways;
the much larger retained source also contains footways and other unrendered highway classes.
Bridges may still appear in the underlying aerial imagery.

The retained 10 km source-context square contains 21,693 surface ways, 8,223 unique resolved junction layouts, and 42,323 lane
connections. Joined pavement occupies approximately 14.99 square kilometers. Discovery diagnostics retain
individual candidate attempts; those records are not additional accepted layouts.

The complete source-context marking records contain 6,571 rendered crossings, 2,727 stop bars, 194 turn arrows,
and five BUS labels, in addition to longitudinal lane paint. These counts exclude duplicate cell records;
they are not totals for the smaller circle.
The stop bars are inferred at mapped controls; they are not claims that OSM surveyed each painted bar.

The new circular footprint intersects 8,347 surface ways. It contains 2,782 rendered crossing-node positions
and mapped control-node positions associated with 1,334 stop bars. Paint crossing the circle boundary is clipped
with the other materials, so these evidence counts describe source positions rather than independent painted polygons.

The planar backend reuses the existing road footprints, width transitions, urban junction solver, turn-lane
paint, source-tagged crossings, and inferred stop bars. Two-kilometer work cells use overlapping context.
The paint pass includes directional centerlines, lane dividers, mapped turn indications, crosswalks, stop bars,
inferred BUS words for mapped lane roles, and the existing taper/merge edge guidance. It does not replace these
with a road-centerline raster. Unsupported source schemas still retain the compiler's explicit withholding policy.
Each accepted junction belongs to one cell, and its whole domain is assembled before paint is clipped.
Unsupported junctions retain pavement and explicit diagnostics, with uncertain junction guidance withheld.
Curbs are inferred 0.22 m concrete bands outside the joined urban pavement boundary, with 0.25 m shaded
gutters inside it. They follow the final union, including islands, and do not cross joined intersection mouths.
They add image detail without height. Explicit `kerb=no` ways do not supply curb inference; motorway, trunk,
and cycleway classes do not supply it either. At this resolution narrow paint and curbs are subpixel detail.
The original terrain-fitting and triangle-export stages are unnecessary for this image-only output.

The result is illustrative pavement, not surveyed paint or a complete legal-routing model. Existing width,
lane, crossing-style, and signal assumptions remain visible in the exported evidence. A single image overlay
cannot provide vertical separation, curbs with height, engineered road grades, or a smooth road profile
independent of the terrain mesh.

![Native close view of the compressed junction overlay](output/boston-metro-raster/native-lod16-intersection.jpg)

At close range, BC3 compression leaves visible blocks in narrow, angled yellow paint. Curbs and the thinnest
stripes still occupy fractions of a pixel at LOD 16. The ReadyMap background is also coarse at this scale.
This bake preserves physical marking widths and records that visual limitation rather than widening the paint.

## Build and view

Install the experiment's Python requirements, then run from the repository root:

```powershell
python tools/experiments/prestige_roads/raster_roads.py fetch
python tools/experiments/prestige_roads/raster_roads.py geometry --workers 12
python tools/experiments/prestige_roads/raster_roads.py tiles --workers 8
tools/experiments/prestige_roads/view_raster.ps1 `
    -RuntimeRoot C:/Users/gwald/.codex/worktrees/a436/repo -View back-bay
```

Other views are `overview`, `intersection`, and `cambridge`. The launcher initializes `osgearth_shell.bat`.
The installed a436 runtime is used read-only; the compiler and outputs remain in c012. No engine C++ build
or new layer implementation is needed.

`fetch` reuses the retained source when present. `layouts` resumes the generated network and completed cell
checkpoints from the same source/configuration; use `geometry` for a fresh build after changing those inputs.
`paint` resumes the resolved network and skips completed marking cells. `render` assembles completed paint
with curb and gutter geometry. These checkpoints are only reusable with the same source and configuration.
`tiles` writes an in-progress marker that prevents the launcher from presenting a partial pyramid as complete.
Lossless PNGs in `build-png` are build intermediates; the runtime consumes only DDS files in `tiles`.
The large compiler checkpoints and lane-graph export are also build evidence. To copy the demo, retain the
earth files and their sibling `tiles` directory; the viewer does not load those compiler intermediates.

## Validation and compiler scaling

The circle regression independently checks geodesic distances, fractional boundary coverage for all five
materials, and unchanged interior pixels. All 451 DDS files pass checks for tile addresses, hashes, 512-pixel
dimensions, BC3 storage, ten mip levels, and coverage restricted to the 3 km radius within pixel-footprint
tolerance. The old LOD 14 directory is absent; only LOD 16 tiles remain.

All 1,000 independently sampled source road centers land on opaque pavement in the final DDS tiles.
Transparent and opaque alpha remain exact. Against the lossless reference images, mean alpha error is
0.16/255, maximum alpha error is 18/255, and coverage-weighted mean RGB error is 3.88/255. These are compression
errors rather than positional errors; the image metrics do not eliminate the local paint artifacts visible above.

An independent geometry audit sampled 1,499 curb points and 1,500 gutter points. Curbs remain outside
pavement within their 0.22 m band; gutters remain inside within 0.25 m. All 25 marking cells pass the crossing
and stop-bar exclusion check. The complete accepted-layout sequence matches the reference ownership result.
Explicit bridge and tunnel ways are absent from the render network.

The final evidence is retained beside the tiles in `tile-manifest.json`, `tile-audit.json`,
`georeferencing-audit.json`, `circle-crop-audit.json`, and `material-and-marking-audit.json`. Per-cell marking
evidence keeps source tags, inferred geometry, and withheld cases separately identifiable. The material audit
covers the unchanged source-context geometry; the new circular crop is audited independently.

The native osgEarth 3.8.1 / OSG 3.6.5 probe rendered BC3 tiles with transparency and continuous joins after
the configuration corrections above. A native read/write preserved DXT5 storage and all ten mip levels.
Grid checks cover the two-by-one geodetic root, north-origin row addresses, and parent/child tile coverage.

The larger region exposed repeated work in the existing compiler. These transient benchmarks use retained
Boston geometry; their speedups apply to the named operations, not to the whole city build.

| Operation | Before median | After median | Regression |
| --- | ---: | ---: | --- |
| Compound corridor support check | 55.74 ms | 17.81 ms | Zero area difference |
| 18 turn connectors and their candidate trials | 557.82 ms | 304.22 ms | Byte-identical selected paths and ribbons |
| 24 junction geometry audits | 6.515 s | 1.217 s | Identical audit results |
| 12 junctions, paint, and audits with compact pavement context | 18.026 s | 2.513 s | Identical paths and arrow evidence; paint/replacement area difference below 0.000001 m² |
| 700 spatial ownership decisions | 120.56 ms | 2.01 ms | Identical accepted indices |
| Rectangular window in 3.19 million pavement vertices | 7.844 s | 19.48 ms | Zero area difference |
| Street paint on 200 roads | 37.985 s | 24.568 s | Paint area difference below 0.000001 m² |

Compact pavement is an optional compiler path used by this raster experiment. Lane selection still sees the
original pavement. Stored paint support includes the complete approach alignments, junction domain, and lane
corridors; each accepted domain is then applied to the shared pavement. This avoids retaining a neighborhood
copy for every junction. The normal mesh compiler keeps its existing default path.

Manhattan retains all 29 layouts and 120 byte-identical lane paths; 29 deliberately damaged pavement cases
are still rejected. Alewife retains its six resolved junctions. Planar and mesh crossing coverage agree for
Alewife's eleven rendered crossings and Manhattan's 75, including the policy for explicitly unmarked crossings.
