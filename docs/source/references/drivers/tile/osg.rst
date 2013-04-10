OSG (OpenSceneGraph Loader)
===========================
This loader will use one of OpenSceneGraph's image plugins to load an image,
and then return tiles based on that image. Since the image will not have its
own SRS information, you are required to specify the geospatial profile.

It is rare that you will need this plugin; the GDAL driver will handle most
file types.

Example usage::

    <image driver="osg">
        <url>images/world.png</url>
        <profile>global-geodetic</profile>
    </image>
    
Properties:

    :url:      Location of the file to load.
    :profile:  Geospatial profile for the image. See Profiles_.
