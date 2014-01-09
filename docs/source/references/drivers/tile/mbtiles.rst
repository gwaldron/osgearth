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
    :compute_levels:    Whether or not to automatically compute the valid levels of the MBTiles file.
                        By default this is true and will scan the table to determine the min/max.
                        This can take time when first loading the file so if you know the levels of your file 
                        up front you can set this to false and just use the min_level max_level settings of the tile source.
       
Also see:

    ``mb_tiles.earth`` sample in the repo ``tests`` folder
    

.. _MBTiles:  https://www.mapbox.com/developers/mbtiles/
