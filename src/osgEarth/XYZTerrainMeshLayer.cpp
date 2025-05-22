/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "XYZTerrainMeshLayer"
#include <osgEarth/Registry>
#include <osgEarth/QuantizedMesh>
#include <osgEarth/Notify>

using namespace osgEarth;

#undef LC
#define LC "[XYZTerrainMeshLayer] "

//........................................................................

Config
XYZTerrainMeshLayer::Options::getConfig() const
{
    Config conf = super::getConfig();
    conf.set("url", _url);
    conf.set("format", _format);
    conf.set("invert_y", _invertY);
    conf.set("min_level", minLevel());
    conf.set("max_level", maxLevel());
    return conf;
}

void
XYZTerrainMeshLayer::Options::fromConfig(const Config& conf)
{
    conf.get("url", _url);
    conf.get("format", _format);
    conf.get("invert_y", _invertY);
    conf.get("min_level", minLevel());
    conf.get("max_level", maxLevel());
}

//........................................................................

REGISTER_OSGEARTH_LAYER(xyzterrainmesh, XYZTerrainMeshLayer);

OE_LAYER_PROPERTY_IMPL(XYZTerrainMeshLayer, URI, URL, url);
OE_LAYER_PROPERTY_IMPL(XYZTerrainMeshLayer, bool, InvertY, invertY);
OE_LAYER_PROPERTY_IMPL(XYZTerrainMeshLayer, std::string, Format, format);

void
XYZTerrainMeshLayer::init()
{
    TerrainMeshLayer::init();
}

void
XYZTerrainMeshLayer::setProfile(const Profile* profile)
{
    TerrainMeshLayer::setProfile(profile);

    if (profile)
    {
        // update the options for proper serialization
        options().profile() = profile->toProfileOptions();
    }
}

Status
XYZTerrainMeshLayer::openImplementation()
{
    Status parent = TerrainMeshLayer::openImplementation();
    if (parent.isError())
        return parent;

    if (!options().url().isSet())
    {
        return Status::Error(Status::ConfigurationError, "Valid URL template is required");
    }

    DataExtentList dataExtents;

    Status status = _driver.open(
        options().url().get(),
        options().format().get(),
        dataExtents,
        getReadOptions());

    if (status.isError())
        return status;

    if (!getProfile())
    {
        OE_INFO << LC << "No profile; assuming spherical-mercator" << std::endl;
        setProfile(Profile::create("spherical-mercator"));
    }

    if (dataExtents.empty())
    {
        DataExtent e(getProfile()->getExtent());
        // these copy the optional, retaining the set or unset state:
        e.minLevel() = options().minLevel();
        e.maxLevel() = options().maxLevel();
        dataExtents.emplace_back(e);
    }

    setDataExtents(dataExtents);

    return Status::NoError;
}

TileMesh
XYZTerrainMeshLayer::createTileImplementation(const TileKey& key, ProgressCallback* progress) const
{
    // Use the XYZ driver to read the raw data
    ReadResult r = _driver.read(
        options().url().get(),
        key,
        options().invertY() == true,
        progress,
        getReadOptions());

    if (r.succeeded())
    {
        // The XYZ driver returns an osg::Image for regular XYZ layers,
        // but for terrain mesh we need the raw binary data.
        // We'll need to get the data directly from the URI.
        
        unsigned x, y;
        key.getTileXY(x, y);
        unsigned cols = 0, rows = 0;
        key.getProfile()->getNumTiles(key.getLevelOfDetail(), cols, rows);
        unsigned inverted_y = rows - y - 1;

        if (options().invertY() == true)
        {
            y = inverted_y;
        }

        std::string url_template = options().url().get().full();
        
        // Apply XYZ template substitutions
        replaceIn(url_template, "${x}", Stringify() << x);
        replaceIn(url_template, "${y}", Stringify() << y);
        replaceIn(url_template, "${-y}", Stringify() << inverted_y);
        replaceIn(url_template, "${z}", Stringify() << key.getLevelOfDetail());

        // Legacy osgearth style:
        replaceIn(url_template, "{x}", Stringify() << x);
        replaceIn(url_template, "{y}", Stringify() << y);
        replaceIn(url_template, "{-y}", Stringify() << inverted_y);
        replaceIn(url_template, "{z}", Stringify() << key.getLevelOfDetail());

        URI tileURI(url_template, options().url().get().context());

        // Read the binary data directly
        ReadResult binaryResult = tileURI.readString(getReadOptions(), progress);
        if (binaryResult.succeeded())
        {
            std::string data = binaryResult.getString();
            auto mesh = QuantizedMeshReader::readFromString(key, data);
            
            // Apply constraints if any
            applyConstraints(key, mesh);
            
            return mesh;
        }
        else
        {
            OE_DEBUG << LC << "Failed to read terrain tile from " << tileURI.full() 
                     << ": " << binaryResult.errorDetail() << std::endl;
        }
    }
    else
    {
        OE_DEBUG << LC << "XYZ driver failed to read tile: " << r.errorDetail() << std::endl;
    }

    // Return invalid mesh on failure
    return TileMesh();
}