# Boston roads on a regular terrain grid

The Boston prototype uses an ordinary gridded elevation overlay to flatten road corridors, then renders
the existing pavement, markings, and curb bands as terrain imagery. Buildings and trees clamp through
the normal map elevation stack. The custom terrain mesh and direct elevation sampling changes have
been removed; the engine is rebuilt with no tracked source changes.

The study area covers the Public Garden, Boston Common, and adjacent streets: approximately 1.4 by
1.1 km, with a 60 m context collar. Explicit bridges and tunnels are excluded. This is a bounded
prototype of the requested architecture, with approximate grading and no TerrainConstraints.

![Boston junction on ordinary gridded terrain](output/boston-gridded-roads/screenshots/graded-fine-junction.jpg)

## How it works

[grid_roads.py](grid_roads.py) reads the retained Boston planar compiler output and original OSM extract.
It samples ReadyMap along connected road centerlines every 2 m, holds the height approximately constant
across each road, and blends toward a common elevation near source-connected junctions. These are inferred
height targets; OSM source tags, IDs, relations, and width assumptions remain separately inspectable in
`source-evidence.json` and `centerlines.geojson`. The latter stores the natural profile samples before
junction blending.

The compiler rasterizes the pavement and curb footprint into an approximately 1 m regular grid, extends
the targets through a 2 m collar, and feathers back into ReadyMap over another 12 m. A small grid filter
rounds transitions between neighboring road targets. Outside that influence mask, the GeoTIFF has NoData,
allowing the base elevation layer to supply the terrain. There is no road surface triangulation in this pass.

The resulting `road-grading.tif` is an EPSG:4326 float32 raster with WGS84 ellipsoid heights. It contains
1,522 by 1,210 cells and occupies 3.33 MiB. The ReadyMap source is the retained 90 m elevation product,
converted from EGM96 with osgEarth's geoid grid; the finer output spacing adds control over road shape,
not new elevation measurements.

At runtime, `GDALElevation` places this raster above ReadyMap. Five ordinary `FeatureImage` layers draw
asphalt, curb, gutter, yellow paint, and white paint from cropped compiler polygons. They generate
512-pixel tiles on demand through LOD 19. Turn arrows, supported crossings and stop bars, lane separators,
and other retained markings use the same source polygons as the previous Boston raster experiment.
Curbs are material bands in this version, without vertical curb faces. No new splatting layer is needed
to evaluate the grading approach.

The scene includes 2,125 building features and 1,847 vegetation positions from the earlier Boston context.
Their normal clamping uses the combined elevation stack; there is no road-specific query path.

## Measurements and visual checks

The grid audit contains 956 transverse road spans, each covering the inner 60% of a supported road width.
These measure the exported grid, not every runtime terrain LOD or surveyed road conditions.

| Transverse grade | ReadyMap base | Graded grid |
| --- | ---: | ---: |
| Median | 3.91% | 0.010% |
| 95th percentile | 12.01% | 0.238% |

The grading modifies 1,012,069 cells; 277,730 cells belong to the pavement/curb footprint. Maximum cut
and fill are approximately 2.92 m and 3.26 m. The 95th-percentile change is 0.67 m. Smoothing departs from
the road target by 0.013 m at the 95th percentile, with a 0.91 m maximum near a height ownership transition.
That maximum is a remaining approximation, not an accepted engineering tolerance.

![An illustrative road cross-section measured from the grids](output/boston-gridded-roads/cross-section.png)

The independent export check validates all 1,841,620 raster cells, checks vector alignment at 10,000
sampled cell centers, verifies NoData away from road influence, and verifies original OSM, material,
and elevation hashes. All 12 scene variants use ordinary terrain and image layers. Its result is saved
in `output/boston-gridded-roads/independent-export-check.json`; the test script is transient outside the repo.

Native osgEarth inspection covered the district overview, Arlington Street junction, and a street view
beside Boston Common. The street view was compared with grading disabled and with both 33-by-33 and
17-by-17 terrain grids under different detail settings. Road imagery remains on the terrain in these
views; buildings and tree bases appear consistent with the graded ground. This does not certify every
building footprint or terrain paging transition.

![Boston street view on the graded grid](output/boston-gridded-roads/screenshots/graded-fine-street.jpg)

## Run it

From the repository root, use the existing installed c012 runtime:

```powershell
tools/experiments/prestige_roads/view_grid_roads.ps1 -View junction
tools/experiments/prestige_roads/view_grid_roads.ps1 -View overview
tools/experiments/prestige_roads/view_grid_roads.ps1 -View street
tools/experiments/prestige_roads/view_grid_roads.ps1 -View street -Mode raw
tools/experiments/prestige_roads/view_grid_roads.ps1 -View street -Detail coarse
```

The launcher initializes `osgearth_shell.bat`, checks inputs, and sets the existing material asset path.
To regenerate with the Python environment used by the road experiments, run `grid_roads.py`.
It reuses `source/boston-metro.osm.gz`, `source/boston-region-readymap`,
`output/boston-metro-raster/render-polygons.pkl`, and the previous Boston building/vegetation GeoJSON.
Its incomplete-build marker prevents launching partially regenerated output.

## Remaining work

This approach deliberately accepts regular-grid approximation: tight neighboring roads can disagree
in height, coarse terrain can soften cross-sections, and the source DEM cannot establish actual street
grades. The next useful refinement is improved target blending at those local transitions. The image
layers also retain ordinary raster resolution limits; this pass does not resolve distant paint aliasing
or establish production memory/performance budgets.

For inland water, the same gridded elevation producer can accept riverbed and bank targets with explicit
priorities where road and water influence overlap. Visible water levels must remain distinct from bed
elevation. No water grading or road/water conflict resolution is implemented here yet.
