# Working with Data

osgEarth can load many different open standard data formats of imagery, elevation data, and vector features. Read on for tips on preparing your data to get the best results.

## Preparing your Data

osgEarth needs to chop the source data up into grids of "tiles" for display. You can usually load data directly from source with no pre-processing, but if the data is not optimized, it might be slow! So a little pre-processing is usually a good idea when using large datasets.

Load you data directly first. If it's fast enough, you are good to go! Otherwise, here are some tips to optimize your data for tiled access. There are two ways to approach it: optimizing a GeoTIFF, or building tilesets. We will also discuss how to combine a folder full of data into a single layer for use in osgEarth.

### Optimizing a GeoTIFF

GeoTIFF is the most common format for local imagery or elevation data. Here are some steps you can take to speed up osgEarth's access to GeoTIFFs.

#### Reproject your imagery

osgEarth will reproject your data on-the-fly if it does not have the necessary coordinate system. For instance, if you are trying to view a UTM image on a geodetic globe (epsg:4326), osgEarth needs to do that conversion on the fly -- but doing it offline beforehand will be faster.

You can use any tool you want to reproject your data such as GDAL, Global Mapper or ArcGIS.

For example, to reproject a UTM image to geodetic using ```gdal_warp```:
```
gdalwarp -t_srs epsg:4326 my_utm_image.tif my_wgs84_image.tif
```

#### Build internal tiles
Typically formats such as GeoTiff store their pixel data in scanlines. However, using a tiled dataset will be more efficient for osgEarth because of how it uses tiles internally.

To create a tiled GeoTiff using ```gdal_translate```, issue the following command:
```
gdal_translate -of GTiff -co TILED=YES input.tif output.tif
```

Take it a step further and use compression to save space. You can use internal JPEG compression if your data contains no transparency:
```
gdal_translate -of GTiff -co TILED=YES -co COMPRESS=JPG input.tif output.tif
```

#### Build overviews
Adding overviews (also called ''pyramids'' or ''rsets'') can sometimes increase the performance of a large data source in osgEarth. You can use the [gdaladdo](http://gdal.org/gdaladdo.html) utility to add overviews to a dataset:
```
gdaladdo -r average myimage.tif 2 4 8 16
```

#### Spatial indexing for feature data
Large vector feature datasets (e.g., shapefiles) will benefit greatly from a spatial index. Using the ```ogrinfo``` tool (included with GDAL/OGR binary distributions) you can create a spatial index for your vector data like so:

```
ogrinfo -sql "CREATE SPATIAL INDEX ON myfile" myfile.shp
```

For shapefiles, this will generate a ".qix" file that contains the spatial index information.

### Building Tilesets
Pre-tiling your imagery can speed up load time dramatically, especially over the network. In fact, if you want to serve your data over the network, this is the only way!

**[osgearth_conv](osgearth_conv.md)** is a command-line conversion tool that comes with osgEarth. One useful application of the tool is tile up a large GeoTIFF (or other input) in a tiled format. Note: this approach only works with drivers that support writing (MBTiles, TMS).

To make a portable MBTiles file:
```
osgearth_conv --in driver GDALImage --in url myLargeFile.tif
              --out driver MBTilesImage --out filename myData.mbtiles
              --out format jpg
```

If you want to serve tiles from a web server, use TMS:
```
osgearth_conv --in driver GDALImage --in url myLargeData.tif
              --out driver TMSImage --out url myLargeData/tms.xml
              --out format jpg
```

That will yield a folder (called "myLargeData" in this case) that you can deploy on the web behind any standard web server like Apache.

**Tip:** The `jpg` format does NOT support transparency. If your data has an alpha channel, use `png` instead.

Just type ```osgearth_conv``` for a full list of options. The `--in` and `--out` options correspond directly to properties you would normally include in an Earth file.

### Loading a Directory of Files

Sometimes the data you have will consist of lots of individual files that make up a single dataset. DTED elevation data is a common example of this. Instead of loading each individual file into osgEarth as a separate layer, it is best to combine them into one "virtual" dataset.

Use the GDAL [``gdalbuildvrt``](https://gdal.org/programs/gdalbuildvrt.html) utility to create a VRT. A VRT is a "virtual format" that combines multiple files into a single data source.

Say you have a folder full of ".dt1" files. You can create a single layer like so:
```
gdalbuildvrt output.vrt *.dt1
```
Now you can just load ``output.vrt`` directly in osgEarth, like so:
```
<GDALElevation name="My DTED data">
    <url>output.vrt</url>
    <vdatum>egm96</vdatum>
</GDALElevation>
```

## Where to Find Data

Help us add useful sources of freely available data to this list. Always check on attribution and distribution requirements from the provider when using 3rd party data!

**Raster data**

- [ReadyMap.org](http://readymap.org/) - 15m imagery, 90m elevation, and street tiles for osgEarth developers. Free for development and demo purposes only.
- [USGS National Map](http://nationalmap.gov/viewer.html) - Elevation, orthoimagery, hydrography, geographic names, boundaries, transportation, structures, and land cover products for the US.
- [NASA BlueMarble](http://visibleearth.nasa.gov/view_cat.php?categoryID=1484) - NASA's whole-earth imagery (including topography and bathymetry maps)
- [Natural Earth](http://www.naturalearthdata.com/) - Free vector and raster map data at various scales
- [Bing Maps](http://www.microsoft.com/maps/choose-your-bing-maps-API.aspx) - Microsoft's worldwide imagery and map data ($)

**Elevation data**

- [CGIAR](http://srtm.csi.cgiar.org/) - World 90m elevation data derived from SRTM and ETOPO (CGIAR European mirror)
- [GEBCO](http://www.gebco.net/) - General Bathymetry Chart of the Oceans

**Feature data**

- [OpenStreetMap](http://openstreetmap.org/) - Worldwide, community-sources street and land use data (vectors and rasterized tiles)
- [Natural Earth](http://www.naturalearthdata.com/) - Free vector and raster map data at various scales
- [DIVA-GIS](http://www.diva-gis.org/gData) - Free low-resolution vector data for any country

