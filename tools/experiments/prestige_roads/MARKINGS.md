# Prestige terrain markings: first comparison

October 12, 2026. The three fixed city fixtures now render their existing white/yellow marking polygons
over the asphalt prototype. The provisional ground backend is **paint coverage in the terrain material
shader**. The GPU depth-clamped mesh decal baseline failed the close-range attachment check.
This decision concerns that baseline, not every possible decal technique.

## Implemented slice

`marking_comparison.py` creates identical-camera mask and decal scenes for Boston, NYC and London,
including intersection, close paint, grazing and overview views at terrain screen-space errors 8 and 32.
The original vectors, ownership contract, source evidence and connections are unchanged. The script
selects actual white paint near each fixed junction for close views; it does not manufacture markings
or assign a turn-arrow meaning to an arbitrary polygon. Empty yellow output in London is skipped.

For the mask candidate, two instances of the new Prestige `SurfaceSplattingLayer` select white and yellow
paint. Each uses 512-pixel coverage pages from global-geodetic LOD 15 through 21, a two-meter query buffer and a
small constant paint material with nominal roughness 0.6, zero metal and full ambient occlusion visibility.
Asphalt keeps its independent repeating detail texture and coverage settings. Paint alpha comes solely
from the polygon mask and uses the existing terrain image-layer parent/child morphing.

At LOD 21, one coverage texel is approximately 1.86 cm north/south and 1.37 cm east/west in Boston,
1.42 cm east/west in NYC, and 1.16 cm east/west in London. These are geographic-grid estimates, not a
claim that every screen view is already displaying that finest level. Parent pages are coarser.
The mask stays an ordinary paged terrain layer; no regional baked color pyramid is generated.

The comparison mesh uses the same polygons through FeatureModel, corrected Earcut triangulation,
merged geometry, GPU terrain-depth clamping, zero vertical offset, depth testing, and polygon offset.
It derives a geometric normal after clamping and applies the same nominal paint roughness.
The tested angular subdivision limit is 0.00005 degrees (roughly 5.6 meters on Earth).
An initial ten-times-finer setting produced excessive geometry before display and was stopped.
This is a bounded attachment comparison, not a final decal renderer or a custom terrain mesh.

Both candidates use unchanged ReadyMap elevation and exclude the separately exported structure paint.
Unknown bridge heights remain unknown. No grading, terrain replacement, arena migration, or performance
tuning was added.

## Findings and correction

At the Boston intersection, the zero-offset GPU-clamped decals lost most stripes and stop bars. At the
close arrow view, they disappeared. A five-millimeter camera-depth bias did not restore them, and disabling
depth testing alone also did not restore them. The same geometry became intact with CPU elevation clamping
and depth testing disabled in a diagnostic scene. This isolates the observed failure to the GPU clamping
path rather than the polygon-to-triangle conversion; the exact internal cause is not yet resolved.
The CPU/no-depth scene is evidence only, not an accepted rendering solution.

The terrain mask displayed complete arrows, stripes and stop bars, plus NYC's crosswalks. A numerical
width audit then found that Blend2D's polygon rasterizer always added a one-pixel border to conceal seams.
That policy visibly/quantitatively inflated narrow paint and was inappropriate for this use.

The correction makes the existing PolygonSymbol `outline=false` suppress that implicit border. Defaults
remain unchanged. Surface splatting exposes `coverage_outline`, defaults it to true to preserve the asphalt
baseline, and explicitly sets it to false for paint. It still antialiases polygon boundaries; this is not an
analytic vector renderer and finite-resolution edge error remains.

## Validation

The runtime built and installed through `build.bat`. Tests and camera/audit harnesses remained transient
outside the repository. No performance claim is made.

* The default-outline raster output is byte-identical for all 24 retained Boston comparison tiles.
* The paint audit samples 60 adjacent 512×512 tiles at LODs 19, 20 and 21: 15,728,640 pixels.
  At LOD 21, all cities have zero interior missing pixels and zero exterior spill beyond a two-texel
  boundary band, using alpha thresholds 250 and 5. Across all levels there are zero exterior spill samples
  and two slightly reduced interior-alpha samples in London at LOD 20.
* Integrated alpha area is a separate, stricter diagnostic. The worst sampled LOD-21 errors changed from
  6.93% to 2.07% in Boston, 22.03% to 8.56% in NYC and 18.36% to 6.12% in London. The NYC maximum is about
  0.097 m² on 1.138 m² of clipped paint; London's maximum is about 0.0007 m² on a 0.0121 m² tip.
  These remaining edge/coverage errors are recorded rather than called exact vector equivalence.
* Boston's close and grazing views were inspected, including coarse/fine terrain settings. NYC and London
  were inspected using GL4. A transient six-position Boston sweep moved from 18 to 200 to 800 meters,
  back to 200, to a 55-meter grazing view, and back to 18 meters. The correct geographic target was supplied
  explicitly after an initial harness attempt used an uninitialized camera target.
* No compiler rebuild or connectivity changes were required. Source and attributed vector hashes remain
  in the fixture/contract and comparison manifests. The renderer consumes ground ownership only.

See [saved evidence](output/marking-validation/), including the before/after coverage audit, default-output
regression, build and viewer logs, and the following captures:

* [Boston corrected paint close-up](output/marking-validation/boston-mask-detail-8-final.png)
* [Boston original GPU-clamped decal failure](output/marking-validation/boston-decal-detail-8-zero-bias.png)
* [CPU-clamped geometry diagnostic](output/marking-validation/boston-map-clamped-no-depth-diagnostic.png)
* [Boston camera sweep, grazing after zoom](output/marking-validation/boston-tour-grazing-after-zoom.png)
* [NYC intersection](output/marking-validation/nyc-mask-intersection-final-gl4.png)
* [London intersection](output/marking-validation/london-mask-intersection-final-gl4.png)

The camera sweep and image checks are a bounded visual test. They do not establish exhaustive forced-LOD,
cold-loading, revision handoff, normal-map, shadow, picking, or wrong-deck correctness.

## Run

After generating the ownership/asphalt fixtures and building the runtime:

```powershell
python tools/experiments/prestige_roads/marking_comparison.py boston nyc london
./tools/experiments/prestige_roads/view_markings.ps1 -City boston -View detail
./tools/experiments/prestige_roads/view_markings.ps1 -City nyc -GL4
./tools/experiments/prestige_roads/view_markings.ps1 -City london -GL4
```

Use `-View grazing`, `-View overview`, or `-TerrainDetail 32` as needed. `-Backend decal` opens the documented
failed attachment baseline. The launcher initializes `osgearth_shell.bat`. Generated scenes and their
manifest are under each fixture's `marking-comparison` folder.

After the first comparison, the demo generators gained a shared minimum coverage LOD of 15 for asphalt
and paint. Regenerate `surface_splatting.py` scenes before this script to update both. The existing
image-layer gate skips lower-level coverage generation and rendering; reusable materials still initialize.
The original comparison images predate this cutoff. The later [bridge slice](BRIDGE_SLICE.md) includes runtime
request checks and paired LOD 14/15 views confirming the cutoff. A finished activation fade remains open.
The failed mesh-decal baseline has no equivalent cutoff and is not a distance-visibility comparison.

## Decision and remaining work

Use terrain paint coverage for the next ground-road visual slice. It follows the same rendered surface
as asphalt and requires no vertical separation. Fine paint remains a raster approximation with explicit
texel density, rather than being baked into the asphalt detail texture. Parent coverage provides the current
distant representation; there is no simultaneous decal/mask handoff or duplicate representation.

Retain mesh decals as a future option for exact grid attachment or solved bridge decks. Do not use large
offsets or disabled depth testing to accept the current GPU-clamped baseline. Unresolved work includes
subpixel edge fidelity, ownership-split antialiasing, coordinated page/revision publication, dedicated
paint normal/wear assets and wider movement/terrain tests. The later Congress Street prototype binds bridge
paint to the deck by partitioning the solved triangles; it does not project the paint onto underlying terrain.

The requested migration to TextureArena/MaterialArena is recorded in the implementation plan and remains
deferred. Material architecture and performance work can resume after this visual path is established.
