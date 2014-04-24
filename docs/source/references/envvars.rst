Environment Variables
=====================
This is a list of environment variables supported by osgEarth.

Caching:

    :OSGEARTH_CACHE_PATH:   Sets up a cache at the specified folder (path)
    :OSGEARTH_CACHE_ONLY:   Directs osgEarth to ONLY use the cache and no data sources (set to 1)
    :OSGEARTH_NO_CACHE:     Directs osgEarth to NEVER use the cache (set to 1)
    :OSGEARTH_CACHE_DRIVER: Sets the name of the plugin to use for caching (default is "filesystem")

Threading/Performance:

    :OSG_NUM_DATABASE_THREADS:      Sets the total number of threads that the OSG DatabasePager
                                    will use to load terrain tiles and feature data tiles.
    :OSG_NUM_HTTP_DATABASE_THREADS: Sets the number of threads in the Pager's thread pool (see
                                    above) that should be used for "high-latency" operations.
                                    (Usually this means operations that do not read data from
                                    the cache, or are expected to take more time than average.)

Debugging:

    :OSGEARTH_NOTIFY_LEVEL:     Similar to ``OSG_NOTIFY_LEVEL``, sets the verbosity for
                                console output. Values are ``DEBUG``, ``INFO``, ``NOTICE``,
                                and ``WARN``. Default is ``NOTICE``. (This is distinct from
                                OSG's notify level.)
    :OSGEARTH_MP_PROFILE:       Dumps verbose profiling and timing data about the terrain engine's
                                tile generator to the console.
    :OSGEARTH_MERGE_SHADERS:    Consolidate all shaders within a single shader program; this
                                is required for GLES (mobile devices) and is therefore useful
                                for testing. (set to 1).
    :OSGEARTH_DUMP_SHADERS:     Prints composed shader programs to the console (set to 1).

Rendering:

    :OSGEARTH_DEFAULT_FONT:       Name of the default font to use for text symbology
    :OSGEARTH_MIN_STAR_MAGNITUDE: Smallest star magnitude to use in SkyNode
    
Networking:

    :OSGEARTH_HTTP_DEBUG:                  Prints HTTP debugging messages (set to 1)
    :OSGEARTH_HTTP_TIMEOUT:                Sets an HTTP timeout (seconds)
    :OSG_CURL_PROXY:                       Sets a proxy server for HTTP requests (string)
    :OSG_CURL_PROXYPORT:                   Sets a proxy port for HTTP proxy server (integer)
    :OSGEARTH_PROXYAUTH:                   Sets proxy authentication information (username:password)
    :OSGEARTH_SIMULATE_HTTP_RESPONSE_CODE: Simulates HTTP errors (for debugging; set to HTTP response code)

Misc:

    :OSGEARTH_USE_PBUFFER_TEST: Directs the osgEarth platform Capabilities analyzer to
                                create a PBUFFER-based graphics context for collecting
                                GL support information. (set to 1)

