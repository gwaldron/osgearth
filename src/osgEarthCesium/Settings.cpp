/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
* MIT License
*/

#include "Settings"
#include "CesiumIon"

// TODO:  Replace this with the default key from Cesium
static std::string CESIUM_KEY = "";

namespace
{
    class ReadKey
    {
    public:
        ReadKey()
        {
            // Get the key from an environment variable
            const char* key = ::getenv("OSGEARTH_CESIUMION_KEY");
            if (key)
            {
                osgEarth::Cesium::setCesiumIonKey(std::string(key));
            }
        }
    };
}

static ReadKey s_READKEY;

std::string  osgEarth::Cesium::getCesiumIonKey()
{
    return CESIUM_KEY;
}

void osgEarth::Cesium::setCesiumIonKey(const std::string& key)
{
    CESIUM_KEY = key;
}

void osgEarth::Cesium::shutdown()
{
    CesiumIon::instance().shutdown();
}