Caching
=======
Depending on the nature of the source data, osgEarth may have to perform
some processing on it before it becomes a terrain tile. This may
include downloading, reprojection, cropping, mosacing, or compositing, to
name a few. These operations can become expensive. By setting up a cache,
you can direct osgEarth to store the result of the processing so that it
doesn't need to do it again the next time the same tile is needed.

    Note! osgEarth's cache uses an internal data storage representation that
    is not intended to be accessed through any public API. It's intended for
    use ONLY as a transient cache and not at a data publication format. The
    structure is subject to change at any time. If you want to publish a data 
    repository, consider the ``osgearth_package`` utility instead!


Setting up a Cache
------------------
You can set up a cache in your *earth file*. The following setup will
automatically activate caching on all your imagery and elevation layers::

    <map>
        <options>
            <cache type="filesystem">
                <path>folder_name</path>
            </cache>
            
In code this would look like this::

    FileSystemCacheOptions cacheOptions;
    cacheOptions.path() = ...;

    MapOptions mapOptions;
    mapOptions.cache() = cacheOptions;
    
Or, you can use an environment variable that will apply to all earth files. 
Keep in mind that this will *override* a cache setting in the earth file::

   set OSGEARTH_CACHE_DRIVER=leveldb
   set OSGEARTH_CACHE_PATH=folder_name

In code you can set a global cache in the osgEarth resgistry::

    osgEarth::Registry::instance()->setCache(...);
    osgEarth::Registry::instance()->setDefaultCachePolicy(...);


Caching Policies
----------------
Once you have a cache set up, osgEarth will use it by default for all your
imagery and elevation layers. If you want to override this behavior, you can
use a *cache policy*. A cache policy tells osgEarth if and how a certain object 
should utilize the cache.

In an *earth file* you can do this by using the ``cache_policy`` block. Here 
we apply it to the entire map::

    <map>
        <options>
            <cache_policy usage="cache_only"/>
            
Or you can apply a policy to a single layer::

    <image>
        <cache_policy usage="no_cache"/>
        ...
        

The values for cache policy *usage* are:

    :read_write:        The default. Use a cache if one is configured.
    :no_cache:          Even if a cache is in place, do not use it. Only read
                        directly from the data source.
    :cache_only:        If a cache if set up, ONLY use data in the cache; never go 
                        to the data source.

You can also direct the cache to expire objects. By default, cached data never expires,
but you can use the ``max_age`` property to tell it how long to treat an object as valid::

    <cache_policy max_age="3600"/>
    
Specify the maximum age in seconds. The example above will expire objects that are more
than one hour old.

Environment Variables
---------------------
Sometimes it's more convenient to control caching from the environment,
especially during development.

These variables override the cache policy properties:

    :OSGEARTH_NO_CACHE:      Enables a ``no_cache`` policy for any osgEarth map. (set to 1)
    :OSGEARTH_CACHE_ONLY:    Enabled a ``cache_only`` policy for any osgEarth map. (set to 1)
    :OSGEARTH_CACHE_MAX_AGE: Set the cache to expire objects more than this number of seconds old.

These are not part of the cache policy, but instead control a particular cache implementation.

    :OSGEARTH_CACHE_PATH:    Root folder for a cache. Setting this will enable caching for
                             whichever cache driver is active.
    :OSGEARTH_CACHE_DRIVER:  Set the name of the cache driver to use, e.g. ``filesystem`` or
                             ``leveldb``.

**Note**: environment variables *override* the cache settings in an *earth file*! See below.


Precedence of Cache Policy Settings
-----------------------------------
Since you can set caching policies in various places, we need to establish
precendence. Here are the rules.

- **Map settings**. This is a cache policy set in the ``Map`` object on in the 
  ``<map><options>`` block in an earth file. This sets the default cache policy for every
  layer in the map. This is the weakest policy setting; it can be overridden by any of
  the settings below.

- **Layer settings**. This is a cache policy set in a ``ImageLayer`` or ``ElevationLayer``
  object (or in the ``<map><image>`` or ``<map><elevation>`` block in an earth file).
  This will override the top-level setting in the Map, but it will NOT override a cache
  policy set by environment (see below). (It is also the ONLY way to override a driver
  policy hint (see below), but it is rare that you every need to do this.)

- **Environment variables**. These are read and stored in the Registry's
  ``overrideCachePolicy`` and they will override the settings in the map or in a layer.
  They will however NOT override driver policy hints.

- **Driver policy hints**. Sometimes a driver will tell osgEarth to *never* cache
  data that it provides, and osgEarth obeys. The only way to override this is to
  expressly set a caching policy on the layer itself. (You will rarely have to 
  worry about this.)


Seeding the Cache
-----------------
Sometimes it is useful to pre-seed your cache for a particular area of interest.
osgEarth provides a utility application called ``osgearth_cache`` to accomplish
this task. ``osgearth_cache`` will take an Earth file and populate any caches
it finds.

    Type ``osgearth_cache --help`` on the command line for usage information.

**Note**: The cache is a transient, "black box" designed to improve
performance in certain situations. It is not inteded as a distributable data
repository. In many cases you can move a cache folder from one environment to another
and it will work, but osgEarth does not *guarantee* such behavior.
