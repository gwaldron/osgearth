/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */

#include <osgEarth/Common>
#include <osgEarth/Version>

#ifdef OSGEARTH_EMBED_GIT_SHA
#    define GET_SHA osgEarthGitSHA1
#else
#    define GET_SHA ""
#endif

#if (OSGEARTH_DEVEL_VERSION > 0)
#    define isDevelopmentVersion " (DEVELOPMENT) "
#else
#    define isDevelopmentVersion " "
#endif

extern "C" {

    const char* osgEarthGetVersion()
    {
        static char osgearth_version[256];
        static int osgearth_version_init = 1;
        if (osgearth_version_init)
        {
            sprintf(osgearth_version,"%d.%d.%d build %d",
                OSGEARTH_MAJOR_VERSION,
                OSGEARTH_MINOR_VERSION,
                OSGEARTH_PATCH_VERSION,
                OSGEARTH_SOVERSION);

            osgearth_version_init = 0;
        }
    
        return osgearth_version;
    }

    const char* osgEarthGetSOVersion()
    {
        static char osgearth_soversion[32];
        static int osgearth_soversion_init = 1;
        if (osgearth_soversion_init)
        {
            sprintf(osgearth_soversion,"%d", OSGEARTH_SOVERSION);
            osgearth_soversion_init = 0;
        }
    
        return osgearth_soversion;
    }

    const char* osgEarthGetLibraryName()
    {
        return "osgEarth";
    }

}

osgEarth::Version osgEarth::parseVersion(const char* in)
{
    osgEarth::Version v;
    sscanf(in, "%d.%d.%d", &v.major, &v.minor, &v.patch);
    return v;
}