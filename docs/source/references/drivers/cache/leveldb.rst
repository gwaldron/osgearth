LevelDB Cache
=============
This plugin caches terrain tiles, feature vectors, and other data
to the local file system using the Google leveldb_ embedded key/value
store library.

Example usage::

    <map>
	    <options>
            <cache driver      = "leveldb"
                   path        = "c:/osgearth_cache"
                   max_size_mb = "500" />
            </cache>
			...
			
The ``leveldb`` cache stores each class of data in its own *bin*.
All bins are stored in the same directory, in the same database.
We do this so we can impose a size limit on the entire database. Each
record is timestamped; when the cache reaches the maximum size, it
starts removing the oldest records first to make room.
	
Cache access is asynchronous and multi-threaded, but you may only 
access a cache from one process at a time.
	
The actual format of cached data files is "black box" and may change
without notice. We do not intend for cached files to be used directly
or for other purposes.
    
Properties:

    :path:        Location of the root directory in which to store all cache
	              bins and data.
    :max_size_mb: Maximum size of the cache in megabytes. The size is taken
                  as a goal; there is no guarantee that the size of the cache
                  will always be less than this value, but the driver will do
                  its best to comply.

.. _leveldb: https://github.com/pelicanmapping/leveldb
