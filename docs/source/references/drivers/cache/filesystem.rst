FileSystem Cache
================
This plugin caches terrain tiles, feature vectors, and other data
to the local file system in a hierarchy of folders. Each cached
data element is in a separate file, and may include an associated
metadata file.

Example usage::

    <map>
	    <options>
            <cache driver="filesystem">
	            <path>c:/osgearth_cache</path>
            </cache>
			...
			
Notes::

    The ``filesystem`` cache stores each class of data in its own ``bin``.
	Each ``bin`` has a separate directory under the root path. osgEarth
	controls the naming of these bins, but you can use the ``cache_id``
	property on map layers to customize the naming to some extent.
	
	This cache supports expiration, but does NOT support size limits --
	there is to way to cap the size of the cache.
	
	Cache access is serialized since we are reading and writing
	individual files on disk.
	
	Accessing the cache from more than one process at a time may cause
	corruption.
	
	The actual format of cached data files is "black box" and may change
	without notice. We do not intend for cached files to be used directly
	or for other purposes.
    
Properties:

    :path: Location of the root directory in which to store all cache
	       bins and files.
