Release Notes
=============

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
