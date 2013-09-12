Spatial References
==================

We specify locations on the Earth using *coordinates*, tuples of numbers that
pinpoint a particular place on the map at some level of precision. But just 
knowing the coordinates is not enough; you need to know how to interpret them.

  A **Spatial Reference** (SRS) maps a set of coordinates
  to a corresponding real location on the earth.
  
For example, given the coordinates of a location on the earth::

  (-121.5, 36.8, 2000.0)
  
Those numbers are meaningless unless you know how to use them.
So combine that with some reference information::

  Coordinate System Type: Geographic
  Units:                  Degrees
  Horizontal datum:       WGS84
  Vertical datum:         EGM96

Now you can figure out exactly where the point is on earth, where it is relative to
other points, and how to convert it to other representations.

Components of an SRS
--------------------

A *spatial reference*, or *SRS*, contains:

* `Coordinate System Type`_
* `Horizontal Datum`_
* `Vertical Datum`_
* `Projection`_

Coordinate System Type
~~~~~~~~~~~~~~~~~~~~~~

osgEarth supports three basic coordinate system types:

* **Geographic** - A whole-earth, ellipsoidal model. Coordinates are spherical angles
  in *degrees* (longitude and latitude). Examples include WGS84 and NAD83.
  (`Learn more <http://en.wikipedia.org/wiki/Geographic_coordinate_system>`_)
  
* **Projected** - A local coordinate system takes a limited region of the earth and
  "projects" it into a 2D cartesion (X,Y) plane. Examples include UTM, US State Plane,
  and Mercator.
  (`Learn more <http://en.wikipedia.org/wiki/Map_projection>`_.)
  
* **ECEF** - A whole earth, cartesian system. ECEF = Earth Centered Earth Fixed; it is
  a 3D cartesion system (X,Y,Z) with the origin (0,0,0) at the earth's center; the X-axis
  intersecting lat/long (0,0), the Y-axis intersecting lat/long (0,-90), and the Z-axis
  intersecting the north pole. ECEF is the native system in which osgEarth renders its
  graphics. (`Learn more <http://en.wikipedia.org/wiki/ECEF>`_)

Horizontal Datum
~~~~~~~~~~~~~~~~

A *datum* is a reference point (or set of points) against which geospatial 
measurements are made. The same location on earth can have different coordinates
depending on which datum is in use. There are two classes of datum:

  A **horizontal datum** measures positions on the earth. Since the earth is not
  a perfect sphere or even a perfect ellipsoid, particular datums are usually
  designed to approximate the shape of the earth in a particular region.
  Common datums include **WGS84** and **NAD83** in North America, and **ETR89**
  in Europe. 
  
Vertical Datum
~~~~~~~~~~~~~~
  
A **vertical datum** measures elevation. There are several classes of vertical
datum; osgEarth supports *geodetic* (based on an ellipsoid) and *geoid* (based
on a sample set of elevation points around the planet).

osgEarth has the following vertical datums built in:

* Geodetic - the default; osgEarth uses the Horizontal datum ellipsoid as a reference
* EGM84 geoid
* EGM96 geoid - commonly called *MSL*; used in DTED and KML
* EGM2008 geoid

By default, SRS's in osgEarth use a *geodetic* vertical datum; i.e., altitude is 
measured as "height above ellipsoid (HAE)".

Projection
~~~~~~~~~~

A *projected* SRS will also have a *Projection*. This is a mathematical formula 
for transforming a point on the ellipsoid into a 2D plane (and back).

osgEarth supports thousands of known projections (by way of the GDAL/OGR toolkit).
Notable ones include:

* UTM (Universal Transverse Mercator)
* Sterographic
* LCC (Lambert Conformal Conic)

Each has particular characteristics that makes it desirable for certain types of
applications. Please see `Map Projections`_ on Wikipedia to learn more.

.. _Map Projections:       http://en.wikipedia.org/wiki/Map_projection


SRS Representations
-------------------

There are many ways to define an SRS. osgEarth supports the following.

WKT (Well Known Text)
~~~~~~~~~~~~~~~~~~~~~

WKT is an OGC standard for describing a coordinate system. It is commonly 
found in a ".prj" file alongside a piece of geospatial data, like a shapefile
or an image.

Here is the WKT representation for the *UTM Zone 15N* projection::

    PROJCS["NAD_1983_UTM_Zone_15N",
        GEOGCS["GCS_North_American_1983",
            DATUM["D_North_American_1983",
                SPHEROID["GRS_1980",6378137.0,298.257222101]],
            PRIMEM["Greenwich",0.0],
            UNIT["Degree",0.0174532925199433]],
        PROJECTION["Transverse_Mercator"],
        PARAMETER["False_Easting",500000.0],
        PARAMETER["False_Northing",0.0],
        PARAMETER["Central_Meridian",-93.0],
        PARAMETER["Scale_Factor",0.9996],
        PARAMETER["Latitude_Of_Origin",0.0],
        UNIT["Meter",1.0]]
        
PROJ4
~~~~~

*PROJ4* is a map projections toolkit used by osgEarth and hundreds of other 
geospatial applications and toolkits. It has a shorthand represtation for 
describing an SRS. Here is the same SRS above, this time in PROJ4 format::

    +proj=utm +zone=15 +ellps=GRS80 +units=m +no_defs
    
PROJ4 has data tables for all the common components (like UTM zones and datums)
so you don't have to explicitly define everything like you do with WKT.

EPSG Codes
~~~~~~~~~~

The EPSG (the now-defunct European Petroleum Survey Group) established a table
of numerical codes for referencing well-known projections. You can browse a list
of there `here <http://spatialreference.org/ref/epsg>`_. osgEarth will accept
EPSG codes; again for the example above::

    epsg:26915
    
If you know the EPSG code it's a nice shorthand way to express it. OGR/PROJ4,
which osgEarth requires, includes a large table of EPSG codes.

Aliases
~~~~~~~

The last category is the *named SRS*. There are some SRS's that are so common
that we include shorthand notation for them. These include:

    :wgs84:              World Geographic Survey 1984 geographic system
    :spherical-mercator: Spherical mercator (commonly used in web mapping systems)
    :plate-carre:        WGS84 projected flat (X=longitude, Y=latitude)

    
    

Using Spatial References in osgEarth
------------------------------------

There are several ways to work with an SRS in osgEarth, but the easiest way it
to use the ``GeoPoint`` class. However let's look at creating an SRS first and
then move on to the class.

SpatialReference API
~~~~~~~~~~~~~~~~~~~~

The ``SpatialReference`` class represents an SRS. Lots of classes and functions in
osgEarth require an SRS. Here's how you create on in code::

    const SpatialReference* srs = SpatialReference::get("epsg:4326");
    
That will give you an SRS. The ``get()`` function will accept any of the SRS
representations we discussed above: WKT, PROJ4, EPSG, or Aliases.

If you need an SRS with a vertical datum, express that as a second parameter.
osgEarth support ``egm84``, ``egm96``, and ``egm2008``. Use it like this::

    srs = SpatialReference::get("epsg:4326", "egm96");

It's sometimes useful to be able to access an SRS's component types as well. For
example, every *projected* SRS has a base *geographic* SRS that it's based upon.
You can get this by calling::

    geoSRS = srs->getGeographicSRS();

If you're transforming a projected point to latitude/longitude, that's the output
SRS you will want.

You can also grab an ECEF SRS corresponding to any SRS, like so::

    ecefSRS = srs->getECEF();    

``SpatialReference`` has lots of functions for doing transformations, etc. Consult 
the header file for information on those. But in practice it is usually best to use
classes like ``GeoPoint`` instead of using ``SpatialReference`` directly.


GeoPoint API
~~~~~~~~~~~~

A ``GeoPoint`` is a georeferenced 2D or 3D point. ("Georeferenced" means that the
coordinate values are paired with an SRS - this means all the information necessary
to plot that point on the map is self-contained.) There are other "Geo" classes
including ``GeoExtent`` (a bounding box) and ``GeoCircle`` (a bounding circle).

Here is how you create a 2D ``GeoPoint``::

    GeoPoint point(srs, x, y);
    
You can also create a 3D ``GeoPoint`` with an altitude::

    GeoPoint point(srs, x, y, z, ALTMODE_ABSOLUTE);
    
The ``ALTMODE_ABSOLUTE`` is the *altitude mode*, and it required when you specify
a 3D coordinate:

    :``ALTMODE_ABSOLUTE``:  Z is relative to the SRS' vertical datum, i.e., 
                            height above ellipsoid or height above the geoid.
    :``ALTMODE_RELATIVE``:  Z is relative to the height of the terrain under
                            the point.

Now that you have your ``GeoPoint`` you can do transformations on it. Say you 
want to transform it to another SRS::

    GeoPoint point(srs, x, y);
    GeoPoint newPoint = point.transform(newSRS);
    
Here's a more concrete example. Say you have a point in latitude/longitude (WGS84)
and you need to express it in UTM Zone 15N::

    const SpatialReference* wgs84 = SpatialReference::get("wgs84");
    const SpatialReference* utm15 = SpatialReference::get("+proj=utm +zone=15 +ellps=GRS80 +units=m");
    ...
    GeoPoint wgsPoint( wgs84, -93.0, 34.0 );
    GeoPoint utmPoint = wgsPoint.transform( utm15 );
    
    if ( utmPoint.isValid() )
       // do something
       
Always check ``isValid()`` because not every point in one SRS can be transformed
into another SRS. UTM Zone 15, for example, is only defined for a 6-degree span
of longitude -- values too far outside this range might fail!

