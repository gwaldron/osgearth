# Prestige Roads on real terrain

2026-10-10. Bounded ReadyMap experiments in Sion and Pittsburgh, using the retained OSM extracts.

**Use a road-owned grading overlay for fitted ground roads.** The same solved surface now drives the
road mesh and a sparse elevation overlay. Sion demonstrates this in osgEarth without TerrainConstraints.
Untouched terrain remains available as an explicit comparison. This is a ground-road feasibility result,
not a completed vertical-design solution: Pittsburgh's fit is rejected, and DEM-derived bridges fail clearance.

## What the viewer shows

Identical Sion camera, identical fitted road mesh, SSE 8. The foreground junction is mostly buried on
natural ReadyMap terrain:

![Untouched terrain clips the fitted road](output/sion-readymap-fit/raw-sse8-terrain-cut.jpg)

Adding the grading overlay recovers its pavement:

![The same road over its grading overlay](output/sion-readymap-fit/graded-sse8-terrain-cut.jpg)

The four-way street view keeps the surrounding Alpine terrain:

![Sion intersection with ReadyMap terrain](output/sion-readymap-fit/graded-sse2-four-way-street.jpg)

Matched four-way views at [SSE 2](output/sion-readymap-fit/graded-sse2-four-way-street.jpg) and
[SSE 32](output/sion-readymap-fit/graded-sse32-four-way-street.jpg) remain clear at that camera.
These are settled camera checks, not a guarantee against all paging transitions or distant-LOD intersections.

## Data and height contract

`fetch_elevation.py` retains the original ReadyMap TMS metadata and native elevation tiles with SHA-256
hashes. Pittsburgh needs one LOD-10 tile; Sion needs two. Coverage includes the complete selected source
alignments and a 450 m guard region, rather than stopping at the render crop. Requests are capped at
sixteen tiles. Existing snapshots are not overwritten. Source attribution is retained in each manifest.

ReadyMap describes this dataset as **90 m CGIAR SRTM Elevation Composite**. Native tiles contain 257
edge-to-edge node samples in geographic coordinates. The compiler bilinearly samples those nodes and
adds the EGM96 geoid separation using osgEarth's own `EGM96Grid.h`, whose hash is recorded. Meshes and
local grading output use WGS84 ellipsoid heights. At the scene origins the geoid corrections are
approximately -34.620 m in Pittsburgh and +51.131 m in Sion. Ignoring this would be a large placement error.

A one-meter output raster and two-meter road-profile stations interpolate this source; they do not add
measured terrain detail. Live viewer maps also load ReadyMap imagery and elevation for the surrounding
globe. Compilation uses retained source bytes; a future change to the live service could change that comparison.

The two surface policies are:

- `drape`: sample natural elevation at pavement vertices, keeping the mesh topology and OSM layout.
  This is a diagnostic of option (a), not a certified road. Skinny triangles crossing DEM slope changes
  can acquire extreme face slopes; broad terrain cross-slopes also become road cross-slopes.
- `fit`: sample road centerlines, resolve shared pavement with the existing curvature-regularized fit,
  and retain bridge-end ties. The existing 15% significant-face grade guard remains enforced. This
  experimental limit is not a regional road-design standard. Crossfall, superelevation, cut/fill limits,
  surveyed bridge supports, and a complete constrained vertical-alignment solver remain future work.

Both policies emit `raw` and `graded` maps. The useful finished ground-surface comparison here is **Sion,
fit, raw versus graded**. Drape scenes remain diagnostic even when their grading raster can be displayed.

## The grading overlay

`terrain.py` indexes the actual constrained pavement triangles, including their holes. It does not
re-triangulate across traffic islands. Inside the ground-road footprint it evaluates those exact faces;
outside, it extends the nearest boundary height and blends into natural terrain.

The sparse GeoTIFF uses ellipsoid meters, a one-meter collar, and an eighteen-meter quintic shoulder.
Pixels beyond the nineteen-meter influence distance are NoData, so the underlying elevation layer remains
in charge there. Small islands can lie entirely within overlapping shoulders; preserving their pavement
holes does not mean their terrain is untouched. Bridge deck triangles are excluded from grading.

The road skin has a 0.12 m visual offset; the bed target is 0.08 m below the solved surface. This leaves
about 0.20 m of headroom for raster interpolation. Curbs and sidewalk foundation faces extend down to
cover the near-ground edge. These dimensions are rendering assumptions, not surveyed construction.
Terrain triangulation at coarser LOD can still violate this headroom; the local raster is not an exact
terrain-mesh boundary constraint.

The overlay is an ordinary `GDALElevation` after ReadyMap elevation. No terrain removal, TerrainConstraint,
virtual texturing, or engine API change is used. The existing synthetic experiments still use their original
terrain path and remain the default.

## Measured outcomes

| Case | Result |
| --- | --- |
| Sion fitted pavement | Significant-face maximum slope 14.24%; face-count p95 8.26%. Ground surface accepted. |
| Sion fitted roads over natural terrain | Centroid estimates: 13.71% of pavement area buried; 11.28% more than 0.25 m above terrain. |
| Sion grading | -1.775 m maximum cut; +1.013 m maximum fill; p95 absolute change 0.575 m over active pixels. |
| Sion raster/road agreement | Every exported pavement vertex remains above bilinearly sampled grading; minimum separation about 0.190 m. |
| Sion direct DEM drape | Face-count p95 slope 10.38%; maximum significant-face slope 251.58%, exposing a tessellation/sampling failure. |
| Pittsburgh direct DEM drape | Face-count p95 slope 24.85%; maximum significant-face slope 127.17%; unsuitable as a road design. |
| Pittsburgh fitted pavement | Rejected: significant-face maximum slope 56.67%, with 6,949 m2 above 15%. No accepted fitted scene exported. |

Maximum-slope checks ignore faces below 0.01 m2; area diagnostics include them. The enormous drape
maxima are mesh-plane slopes, not claims that the actual streets have those grades. Smaller slivers remain
an issue even in the fitted Sion result: only 0.000692 m2 exceeds 15% there, below the significant-face guard.

Pittsburgh's transient parameter probe reduced fidelity and increased curvature regularization, but even
its strongest tested smoothing still reached 24.45%. One DEM-sampled approach profile reaches about 19%.
These are reasons to improve vertical alignment and source interpretation, not silently lower the guard.
The retained `surface-failure.json` makes rejection inspectable, and the launcher refuses that failed fit.

**Bridges are an explicit failure of the endpoint-only inference.** Sion's Rue de la Dixence deck overlaps
the lower Cour de la Gare surface: minimum underside clearance is -0.709 m. Pittsburgh has five overlapping
bridge/ground-road pairs below the inferred five-meter screen, including negative clearances. Full pavement
overlap catches an additional pair that centerline-crossing tests miss. Each record preserves both source
ways' tags and separates them from the assumed 0.7 m deck thickness and 5 m clearance screen. Bridge-over-
bridge clearance and surveyed support heights are not solved. The compiler does not certify these scenes
as complete road networks merely because their ground meshes compile.

## Relationship to the two flattening layers

I inspected the legacy `osgEarth::Contrib::FlatteningLayer` in this worktree and the new
`osgEarthPrestige::FlatteningLayer` in `C:/Users/gwald/.codex/worktrees/a436/repo` without editing either.
The new layer samples underlying terrain, smooths centerline profiles, blends junctions, and ignores feature
Z. Its polygon mode creates level pads. Feeding it the same OSM features would therefore produce a second,
independent elevation solution beneath the compiler's mesh. A junction polygon would not carry the
compiler's sloping triangle surface into that layer.

The useful integration contract is **one authoritative elevation surface**: datum, source revision,
at-grade footprint, junction triangles/profiles, shoulder policy, and bridge/tunnel exclusion. A future
Prestige flattening input could consume that surface or a shared sampler. Its paging and blending machinery
could then replace the experiment's raster export without recomputing different road heights. Do not stack
independent road flatteners over the same patch and hope their profiles coincide.

For this pass, ordinary GDAL elevation is sufficient to test that contract. The installed a436 runtime also
contains the regional elevation-profile fallback correction described in its Prestige flattening documentation;
that matters when viewing a local DEM beyond native resolution. This experiment did not patch the engine or
change public ABI.

## Reproduce

From the repository root, with the existing Python environment and `requirements.txt` dependencies:

```powershell
python tools/experiments/prestige_roads/fetch_elevation.py sion pittsburgh
python tools/experiments/prestige_roads/compile.py --scene sion --terrain readymap --surface fit
python tools/experiments/prestige_roads/compile.py --scene sion --terrain readymap --surface drape
python tools/experiments/prestige_roads/compile.py --scene pittsburgh --terrain readymap --surface drape
# Expected rejection; records the failed grade check:
python tools/experiments/prestige_roads/compile.py --scene pittsburgh --terrain readymap --surface fit

$runtime = 'C:/Users/gwald/.codex/worktrees/a436/repo'
./tools/experiments/prestige_roads/run.ps1 -RuntimeRoot $runtime -Scene sion -Terrain readymap -Mode raw -View terrain-cut
./tools/experiments/prestige_roads/run.ps1 -RuntimeRoot $runtime -Scene sion -Terrain readymap -Mode graded -View terrain-cut
./tools/experiments/prestige_roads/run.ps1 -RuntimeRoot $runtime -Scene sion -Terrain readymap -View four-way-street -SSE 2
```

`run.ps1` initializes `osgearth_shell.bat`. `-Surface drape` selects the diagnostic drape output directory;
otherwise fit is selected. Runtime commands require ReadyMap network access. No configure/build was needed
because this pass changes only the Python experiment, generated assets, maps, and its launcher.

Evidence lives alongside each scene in `metrics.json`, `terrain-audit.json`, `natural-terrain-audit.json`,
`bridge-terrain-audit.json`, and `terrain-export-audit.json`. See also the combined
[real-terrain checkpoint](output/real-terrain-summary.json). Sources live in `source/*-readymap`.

Transient checks outside the repository cover affine interpolation, geoid conventions, the native Sion tile
seam, hash corruption, exact triangle support and holes, sparse raster coverage, and independent glTF/raster
sampling. Existing city semantics, contraflow controls, Manhattan lane layouts, and synthetic bridge checks
remain covered. The Sion and Pittsburgh real-terrain exports retain their synthetic experiments' respective
16 and 14 intersection layouts exactly. Viewer checks use osgEarth 3.8.1 build 187 / OSG 3.6.5 on the installed
a436 runtime. Source snapshots, inference, diagnostic failures, and visual observations remain separate.

The next substantive step is a constrained vertical-alignment pass: solve approach grades, bridge support
heights and clearances together, then regenerate both road geometry and the grading overlay from that solution.
Better terrain alone will not supply all of that connectivity-dependent information.
