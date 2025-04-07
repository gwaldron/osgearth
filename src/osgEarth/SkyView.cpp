/* osgEarth
* Copyright 2008-2012 Pelican Mapping
* MIT License
*/
#include <osgEarth/SkyView>

#define LC "[SkyView] "

using namespace osgEarth;
using namespace osgEarth::Contrib;

//........................................................................

Config
SkyViewImageLayer::Options::getMetadata()
{
    return Config::readJSON(R"(
        { "name" : "SkyView",
          "properties" : [
            { "name": "image", "description" : "Image layer to render", "type" : "ImageLayer", "default" : "" },
          ]
        }
    )");
}

Config
SkyViewImageLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    imageLayer().set(conf, "image");
    return conf;
}

void
SkyViewImageLayer::Options::fromConfig(const Config& conf)
{
    imageLayer().get(conf, "image");
}

//........................................................................

REGISTER_OSGEARTH_LAYER(skyview, SkyViewImageLayer);

void
SkyViewImageLayer::init()
{
    ImageLayer::init();
}

Status
SkyViewImageLayer::openImplementation()
{
    Status parent = ImageLayer::openImplementation();
    if (parent.isError())
        return parent;

    Status imageStatus = options().imageLayer().open(getReadOptions());
    if (imageStatus.isError())
        return imageStatus;

    // copy over the profile
    setProfile(getImageLayer()->getProfile());

    return Status::NoError;
}

GeoImage
SkyViewImageLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
    if (getStatus().isError() || !getImageLayer())
        return GeoImage(getStatus());

    // We need to flip the image horizontally so that it's viewable from inside the globe.

    // Create a new key with the x coordinate flipped.
    unsigned int numRows, numCols;

    key.getProfile()->getNumTiles(key.getLevelOfDetail(), numCols, numRows);
    unsigned int tileX = numCols - key.getTileX() - 1;
    unsigned int tileY = key.getTileY();
    unsigned int level = key.getLevelOfDetail();

    TileKey flippedKey(level, tileX, tileY, key.getProfile());
    GeoImage image = getImageLayer()->createImageImplementation(flippedKey, progress);
    if (image.valid())
    {
        // If an image was read successfully, we still need to flip it horizontally
        
        osg::ref_ptr<osg::Image> flipped =
            osg::clone(image.getImage(), osg::CopyOp::DEEP_COPY_ALL);

        flipped->flipHorizontal();

        return GeoImage(flipped.get(), key.getExtent());
    }
    return GeoImage::INVALID;
}

Config
SkyViewImageLayer::getConfig() const
{
    Config c = ImageLayer::getConfig();
    return c;
}
