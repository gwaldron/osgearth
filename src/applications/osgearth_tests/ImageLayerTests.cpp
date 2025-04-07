/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/

#include <osgEarth/catch.hpp>

#include <osgEarth/ImageLayer>
#include <osgEarth/Registry>
#include <osgEarth/GDAL>

using namespace osgEarth;

TEST_CASE( "ImageLayers can be created" )
{
    GDALImageLayer* layer = new GDALImageLayer();
    layer->setName("World");
    layer->setURL("../data/world.tif");

    Status status = layer->open();
    REQUIRE( status.isOK() );

    SECTION("Profiles are correct")
    {
        const Profile* profile = layer->getProfile();
        REQUIRE(profile != nullptr);
        REQUIRE(profile->isEquivalentTo(Profile::create(Profile::GLOBAL_GEODETIC)));
    }

    SECTION("Images are read correctly")
    {
        TileKey key(0, 0, 0, layer->getProfile());
        GeoImage image = layer->createImage(key);
        REQUIRE(image.valid());
        REQUIRE(image.getImage()->s() == 256);
        REQUIRE(image.getImage()->t() == 256);
        REQUIRE(image.getExtent() == key.getExtent());
    }
}

TEST_CASE("Attribution works")
{
    std::string attribution = "Attribution test";

    GDALImageLayer* layer = new GDALImageLayer();
    layer->setURL("../data/world.tif");
    layer->setAttribution(attribution);

    Status status = layer->open();

    REQUIRE(status.isOK());
    REQUIRE(layer->getAttribution() == attribution);
}