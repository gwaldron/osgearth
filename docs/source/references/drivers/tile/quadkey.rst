QuadKey
=======
The QuadKey plugin is useful for reading web map tile repositories
that follow the Bing_ maps tile system.  It is assumed that the
dataset is in spherical-mercator with 2x2 tiles at the root just like Bing.

Example usage::

    <image name="imagery" driver="quadkey">
        <url>http://[1234].server.com/tiles/{key}.png</url>
    </image>

Creating the URL template:

    The square brackets [] indicate that osgEarth should "cycle through" the characters
    within, resulting in round-robin server requests. Some services require this.

    You will need to provide {key} template within the URL where osgEarth will
    insert the quadkey for the tile it's requesting.

Properties:

    :url:            Location of the tile repository (URL template -- see above)
    :profile:        Spatial profile of the repository
    :format:         If the format is not part of the URL itself, you can specify it here.

.. _Bing: http://msdn.microsoft.com/en-us/library/bb259689.aspx
