Release Notes
=============

Version 2.8 (September 2016)
---------------------------

* Disabled feature tesselation tiling in BuildGeometryFilter unless max_polygon_tiling_angle is explicitly set.  Cropping code was causing issues especially around the poles.  Need to come up with a more general solution in the future.
* Better support for osg::Fog in VirtualPrograms with FogEffect.  Implemented multiple fog modes.
* Always applying min_range and max_range in MPGeometry to prevent uniform leakage.
* Proper support for centroid clamping for MultiPolygons.
* New requirement to call open() on TileSources and Layers when creating at runtime.  This lets you explicitly get the Status of a layer and report errors to users.
* Fixes to EGM96 vertical datum grid.
* BUILD_OSGEARTH_EXAMPLES cmake option for disabling building examples.
* Added nearest sampling support for heightfields
* New feature_join for adding attributes from intersecting 
* osgearth_deformation demo
* Scatter filter support for pointsets.  Simply places models at each point in the PointSet.
* Performance optimizations when discarding features in javascript style selectors when returning null styles
* Feature geometry caching support
* New min_expiry_frames and min_expiry_time options to TerrainOptions.
* Proper createTile implementation for Rex engine.
* RocksDB cache plugin.
* New osgearth_server application (based on Poco networking libraries).  Serve up osgEarth tiles rendered on the GPU to your favorite web mapping tools like Leaflet, OpenLayers and Cesium!
* Packager now supports writing to MBTiles
* New osgearth_skyview example for drawing an "inside out" earth.  Turns out osgearth is a great photosphere viewer!
* Experimental WinInet support to replace CURL.  New osgearth_http test app.  
* Upgraded duktape to version 1.4.0
* Memory usage testing support (osgearth_viewer --monitor to enable)
* New osgearth_3pv utility application.
* Better support for pretiled datasets like TFS and Mapnik Vector Tiles in FeatureRasterSource (and agglite driver)
* Better support for node tethering in EarthManipulator
* Doxygen support
* New openstreetmap vector tiles demos (openstreetmap_buildings.earth and openstreetmap_full.earth)
* Support for Mapnik Vector Tiles datasets
* Fixed improper inversion of y tilekey in FeatureModelGraph and updated all drivers.
* CURLOPT_ENCODING support.  If you've built curl against zlib, proper HTTP headers for gzip and deflate will be added and automatically decompressed.
* New osgearth_splat example
* New osgEarthSplat NodeKit
* New "template" plugin based on NLTemplate that allows you to write templatized earth files
* Support for xi:include in earth files
* Minimum OpenSceneGraph version is 3.4.0
* Removed MINIZIP dependency
* New Triton and Silverlining NodeKits
* New feature_elevation driver that produces features from 
* New raster to feature driver for turning rasters to features
* 330 compatibiity default shader version for GLSL
* Normal mapping integrated into MP, removed normal map extension.
* TravisCI and Coverity support

Version 2.7 (July 2015)
---------------------------

* New ObjectIndex system for picking and selection
* New RTT-based picker that works for all geometry including GPU-modified geometry
* Extensions - modular code for extending the capabilities of osgEarth
* New procedural texture splatting extension
* Upgraded ShaderLoader for better modularization of VirtualProgram code
* New "elevation smoothing" property to MP terrain engine
* New support for default MapNodeOptions
* Logarithmic depth buffer lets you extend your near and far planes
* Better Triton and Silverlining support
* Overhaul of the elevation compositing engine and ElevationQuery utility
* New Raster Feature driver lets you generate features from raster data
* Attenuation and min/max range for image layers
* New shader-based geodetic graticule
* New day/night color filter
* Viewpoint: consolidation of look-ats and tethering
* New CoverageSymbol for rastering features into coverage data; agglite driver support
* New feature clustering and instancing algorithms for better performance and scalability
* Noise extension for creating a simplex noise sampler
* New TerrainShader extension lets you inject arbitrary shader code from an earth file
* VirtualProgram: specify all VP injection criteria with GLSL #pragmas
* Normal mapping extension with automatic edge-normalization
* Bump map extension for simple detail bumping
* Performance improvements based on GlowCode profiling results


Version 2.6 (October 2014)
--------------------------

Maintenance Release. Release notes TBD.

Version 2.5 (November 2013)
---------------------------

Terrain Engine

The terrain engine ("MP") has undergone many performance updates. We focused on geometry
optimization and GL state optimization, bypassing some the OSG mechnisms and going straight
to GL to make things as fast as possible.

MP has a new optional "incremental update" feature. By default, when you change the
map model (add/remove layers etc.) osgEarth will rebuild the terrain in its entirely. With
incremental update enabled, it will only rebuild tiles that are visible. Tiles not currently
visible (like those at lower LODs) don't update until they actually become visible.

Caching

Caching got a couple improvements. The cache seeder (osgearth_cache) is now multi-threaded
(as it the TMS packager utility). The filesystem cache also supports expiration policies
for cached items, including map tiles.

JavaScript

We updated osgEarth to work with the newest Google V8 JavaScript interpreter API. We also
now support JavaScriptCore as a JS interpreter for OSX/iOS devices (where V8 is not
available).

Terrain Effects

A new TerrainEffect API makes it easy to add custom shaders to the terrain. osgEarth has
several of these built in, including NormalMap, DetailTexture, LODBlending, and ContourMap.

New Drivers

There is a new Bing Maps driver. Bing requires an API key, which you can get at the Bing site.

We also added a new LibNOISE driver. It generates parametric noise that you can use as
terrain elevation data, or to add fractal detail to existing terrain, or to generate 
noise patterns for detail texturing.

Other Goodies

* Shared Layers allow access multiple samplers from a custom shader
* A new "AUTO_SCALE" render bin scales geometry to the screen without using an AutoTransform node.
* PlaceNodes and LabelNodes now support localized occlusion culling.
* The Controls utility library works on iOS/GLES now.


Version 2.4 (April 2013)
------------------------

* New "MP" terrain engine with better performance and support for unlimited image layers (now the default)
* Shader Composition - reworked the framework for more flexible control of vertex shaders
* EarthManipulator - support for mobile (multitouch) actions
* GPU clamping of feature geometry (ClampableNode)
* TMSBackFiller tool to generate low-res LODs from high-res data
* OceanSurface support for masking layer
* New RenderSymbol for draw control
* Fade-in control for feature layers
* OverlayDecorator - improvements in draping; eliminated jittering
* Added feature caching in FeatureSourceIndexNode
* ShaderGenerator - added support for more texture types
* Draping - moved draping/clamping control into Symbology (AltitudeSymbol)
* Lines - add units to "stroke-width", for values like "25m", also "stroke-min-pixels"
* PolygonizeLines operator with GPU auto-scaling
* New Documentation site (stored in the repo) at http://osgearth.readthedocs.org
* Decluttering - new "max_objects" property to limit number of drawables
* New ElevationLOD node
* SkyNode - added automatic ambient light calculation
* New DataScanner - build ImageLayers from a recursive file search
* Qt: new ViewWidget for use with a CompositeViewer
* Map: batch updates using the beginUpdate/endUpdate construct
* GLSL Color Filter: embed custom GLSL code directly in the earth file (glsl_filter.earth)
* Agglite: Support for "stroke-width" with units and min-pixels for rasterization
* Terrain options: force an elevation grid size with <elevation_tile_size>
* Better iOS support
* New "BYO" terrain engine lets you load an external model as your terrain
* New "first_lod" property lets you force a minimum LOD to start at
* Better support for tiled data layers
* Lots of bug fixes and performance improvements
* New documentation site stored in the osgEarth repo (docs.osgearth.org)
