MBTiles
=========================
This plugin reads data from an `MBTiles`_ file, which is an SQLite3 database that contains all the tile data in a single table.  This driver requires that you build osgEarth with SQLite3 support.

Example usage::

    <image name="haiti" driver="mbtiles">
        <filename>../data/haiti-terrain-grey.mbtiles</filename>
		<format>jpg</format>
    </image>

Properties:

    :filename:          The filename of the MBTiles file
    :format:            The format of the imagery in the MBTiles file (jpeg, png, etc)

Also see:

    ``mb_tiles.earth`` sample in the repo ``tests`` folder


.. _MBTiles:  https://www.mapbox.com/developers/mbtiles/
