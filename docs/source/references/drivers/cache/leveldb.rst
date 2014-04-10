LevelDB Cache
=============
This plugin caches terrain tiles, feature vectors, and other data
to the local file system using the Google leveldb_ embedded key/value
store library.

Example usage::

    <map>
	    <options>
            <cache driver="leveldb">
	            <path>c:/osgearth_cache</path>
            </cache>
			...
			
Notes::

    The ``leveldb`` cache stores each class of data in its own ``bin``.
	Each ``bin`` has a separate directory under the root path. osgEarth
	controls the naming of these bins, but you can use the ``cache_id``
	property on map layers to customize the naming to some extent.
	
	Cache access is asynchronous and multi-threaded, but you may only 
	access a cache from one process at a time.
	
	The actual format of cached data files is "black box" and may change
	without notice. We do not intend for cached files to be used directly
	or for other purposes.
    
Properties:

    :path: Location of the root directory in which to store all cache
	       bins and data.

.. _leveldb: https://code.google.com/p/leveldb/
