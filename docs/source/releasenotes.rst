Release Notes
=============

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
