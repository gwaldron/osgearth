# osgEarth Layers

These are the public layer types native to osgEarth.

## Raster Data

| Data Source | Description |
| ----------- | ------------|
| [GDAL](gdal.md) | Loads any imagery format supported by the GDAL library, including GeoTIFF |
| [MBTiles](mbtiles.md) | Reads imagery tiles from an MBTiles (MapBox Tiles) database file |
| [TMS](tms.md) | Connects to a TMS (TileMapService) repository |
| [WMS](wms.md) | OGC Web Map Service server |
| [XYZ](xyz.md) | Reads data in standard XYZ format (no metadata) |
| [Composite](composite.md) | Combines multiple image layers into a single map layer |
| [ContourMap](contourmap.md) | Renders a colored representation of the elevation data in the map |
| [Microsoft Bing](bing.md) | ($) Connects to Microsoft Bing service. License key required |
| [Cesium Ion](cesiumion.md) | ($) Connects to a Cesium Ion server instance. License key required |
| [ESRI ArcGIS Server](arcgis.md) | ($) Connects to an ESRI ArcGIS Server instance |


## Vector Data

| Earth File        | Description                                                  |
| ----------------- | ------------------------------------------------------------ |
| [FeatureImage](featureimage.md) | Rasterizes vector data into an image layer                   |
| [FeatureModel](featuremodel.md) | Renders vector data as *OpenSceneGraph* geometry             |
| [TiledFeatureModel](tiledfeaturemodel.md) | Like a `FeatureModel` layer, but optimized for pre-tiled vector datasets |



## Miscellaneous Layers

| Earth File        | API Class              | Description                                                  |
| ----------------- | ---------------------- | ------------------------------------------------------------ |
| Annotations       | AnnotationLayer        | Holds a collection of annotation elements (like text labels, place nodes, or features) |
| Debug             | DebugImageLayer        | Renders metadata about each rendered map tile                |
| GeodeticGraticule | GeodeticGraticuleLayer | Display a simple latitude/longitude graticule                |
| MGRSGraticule     | MGRSGraticuleLayer     | Displays a simple MGRS graticule with labels                 |
| Model             | ModelLayer             | Loads and displays an external 3D model at a map location    |
| Ocean             | SimpleOceanLayer       | Renders a very simple ocean surface (requires the map to have bathymetry data) |
| Sky               | SimpleSkyLayer         | Renders a sky model with realistic lighting and shading      |
| TerrainConstraint | TerrainConstraintLayer | Alters the triangulation of the terrain skin to incorporate vector data; e.g., to represent ridgelines, coastlines, or to cut out an area where a custom terrain model will go |
| ThreeDTiles       | ThreeDTilesLayer       | Displays a 3D-Tiles dataset                                  |
| UTMGraticule      | UTMGraticuleLayer      | Displays a simple UTM graticule                              |
| Video             | VideoLayer             | Renders various video formats to a layer (using FFMPEG)      |
| Viewpoints        | ViewpointsLayer        | Pre-set viewpoints that a viewer application can display for the user |
| Wind              | WindLayer              | Incorporates a wind model (needs other layers that can use the data) |



## Feature Sources

| Earth File  | API Class        | Description                                                  |
| ----------- | ---------------- | ------------------------------------------------------------ |
| MVTFeatures | MVTFeatureSource | Mapnik Vector Tiles specification                            |
| OGRFeatures | OGRFeatureSource | Uses a GDAL/OGR vector driver to read feature data. This is the most common feature source for reading local data (e.g., ESRI Shapefile) |
| TFSFeatures | TFSFeatureSource | Reads vector features from a server according to the Tiled Feature Service specification (osgEarth proprietary) |
| WFSFeatures | WFSFeatureSource | OGC Web Feature Service specification (limited implementation) |
| XYZFeatures | XZYFeatureSource | Generic specification for reading tiled vector data from a server |