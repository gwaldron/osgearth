# The Earth File

osgEarth uses the familiar **Map/Layer** paradigm for organizing data. The **Map** is comprised of a collection of **layers**. The renderer draws each visible layer one after the next, from bottom to top, to display the final scene. [You can see all the different layer types here](layers.md).

An **Earth File** is an XML file that describes the contents of a **Map** in osgEarth.

### My First Earth File

Here is a very simple earth file that you can find in the `tests` folder of the repository:

```xml
<Map name="Hello, World">
    <GDALImage name="World imagery">
        <url>../data/world.tif</url>
    </GDALImage>
</Map>
```

This map contains one layer that points to a local GeoTIFF file. In this case, the location is relative to the location of the earth file itself. You can see this map by running one of the osgEarth command line tools:

```
osgearth_imgui simple.earth
```

That's it! It is that easy to get a map up and running.

### Using Multiple Layers

You can add as many layers to your **Map** as you like. Here's an example called `hires-inset.earth` :

```xml
<Map name="Hello, World">    
    <!-- Worldwide image -->
    <GDALImage name="World">
        <url>../data/world.tif</url>
    </GDALImage>

    <!-- Higher resolution inset of Boston -->
    <GDALImage name="Boston">
        <url>../data/boston-inset-wgs84.tif</url>
    </GDALImage>
    
    <!-- Higher resolution inset of New York City -->
    <GDALImage name="New York">
        <url>../data/nyc-inset-wgs84.tif</url>
    </GDALImage>
</Map>
```

In this case, osgEarth will draw the "World" layer first, followed by "Boston" and finally "New York" on top. We also interspersed some XML comments in there.

### Adding Terrain Elevation Data

Now let's add some height field data, also knows as a **DEM** or Digital Elevation Model:

```xml
<Map name="ReadyMap">
    <TMSImage name="ReadyMap 15m Imagery">
        <url>http://readymap.org/readymap/tiles/1.0.0/7/</url>
    </TMSImage>

    <TMSElevation name="ReadyMap 90m Elevation">
        <url>http://readymap.org/readymap/tiles/1.0.0/116/</url>
        <vdatum>egm96</vdatum>
    </TMSElevation>

    <Viewpoints>
        <Viewpoint name="San Francisco, California">
            <heading>10/heading>
            <height>4500.0</height>
            <lat>37.5581</lat>
            <long>-122.334</long>
            <pitch>-34</pitch>
            <range>78000</range>
        </Viewpoint>
    </Viewpoints>
</Map>
```

A couple things going on here: first we see a `TMSImage` layer that loads imagery from a Tile Map Service layer over the Internet. Then we have some 90m digital elevation data coming from the same server.

Finally you can see a `Viewpoints` layer. This is not a visible layer at all! Instead if stores data - in this case, a set of pre-set viewpoints the user can navigate to. osgEarth stores all kinds of data as **layers**, not all of it visible. Non-visible layers can occur anywhere in the map. Their order or appearance does not matter.

### Drawing Vector Features

Finally, let's look at some vector feature data. This is GIS data in the form of points, lines, and polygons. osgEarth has various methods of displaying this data, but let's keep it simple for now. You can find this one in `feature_rasterize.earth` :

```xml
<Map name="Rasterize Vectors">
    <xi:include href="readymap_imagery.xml"/>
    
    <OGRFeatures name="world-data">
        <url>../data/world.shp</url>
    </OGRFeatures>
    
    <FeatureImage name="Countries" opacity="0.75">
        <features>world-data</features>
        <styles>        
            <style type="text/css">
                default {
                    fill:          #ff7700;
                    stroke:        #ffff00;
                    stroke-width:  5km;
                }
            </style>
        </styles>
    </FeatureImage>  
</Map>
```

First, we see the use of `<xi:include>`, a handy XML directive to include another XML file inline. We're using this to get our imagery.

Next is the `OGRFeatures` layer. This is a data layer - it won't be rendered directly - but rather it just points to an ESRI Shapefile we have stored locally.

Finally, the `FeatureImage` layer points at the data layer and describes how to draw it using a `StyleSheet`.

## More Examples

Please look in the `tests` folder of the repository for lots of examples of earth files. They range from very simple to quite complex and cover a wide range of the available functionality in osgEarth!

