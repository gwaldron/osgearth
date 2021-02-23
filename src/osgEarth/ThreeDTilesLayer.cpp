/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
 * http://osgearth.org
 *
 * osgEarth is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include <osgEarth/GeoTransform>
#include <osgEarth/FileUtils>
#include <osgEarth/Registry>
#include <osgEarth/Utils>
#include <osgEarth/ThreeDTilesLayer>

#define LC "[ThreeDTilesLayer] " << getName() << " : "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Contrib;
using namespace osgEarth::Contrib::ThreeDTiles;
using namespace osgEarth::Threading;

//------------------------------------------------------------------------

Config
ThreeDTilesLayer::Options::getConfig() const
{
    Config conf = VisibleLayer::Options::getConfig();
    conf.set("url", _url);
    conf.set("max_sse", _maximumScreenSpaceError);
    return conf;
}

void
ThreeDTilesLayer::Options::fromConfig( const Config& conf )
{
    _maximumScreenSpaceError.init(15.0f);
    conf.get("url", _url);
    conf.get("max_sse", _maximumScreenSpaceError);
}

//........................................................................

REGISTER_OSGEARTH_LAYER(threedtiles, ThreeDTilesLayer);

OE_LAYER_PROPERTY_IMPL(ThreeDTilesLayer, URI, URL, url);

ThreeDTilesLayer::~ThreeDTilesLayer()
{
    //nop
}

void
ThreeDTilesLayer::init()
{
    VisibleLayer::init();

    // Make sure the b3dm plugin is loaded
    std::string libname = osgDB::Registry::instance()->createLibraryNameForExtension("gltf");
    osgDB::Registry::instance()->loadLibrary(libname);
}

Status
ThreeDTilesLayer::openImplementation()
{
    Status parentStatus = VisibleLayer::openImplementation();
    if (parentStatus.isError())
        return parentStatus;

    osg::ref_ptr< osgDB::Options > readOptions = osgEarth::Registry::instance()->cloneOrCreateOptions(this->getReadOptions());

    ReadResult rr = _options->url()->readString(readOptions.get());
    if (rr.failed())
    {
        return Status(Status::ResourceUnavailable, Stringify() << "Error loading tileset: " << rr.errorDetail());
    }

    Tileset* tileset = Tileset::create(rr.getString(), _options->url()->full());
    if (!tileset)
    {
        return Status(Status::GeneralError, "Bad tileset");
    }

    _tilesetNode = new ThreeDTilesetNode(tileset, "", getSceneGraphCallbacks(), readOptions.get());
    _tilesetNode->setMaximumScreenSpaceError(*options().maximumScreenSpaceError());
    _tilesetNode->setOwnerName(getName());

    return STATUS_OK;
}

float
ThreeDTilesLayer::getMaximumScreenSpaceError() const
{
    return *options().maximumScreenSpaceError();
}

void
ThreeDTilesLayer::setMaximumScreenSpaceError(float maximumScreenSpaceError)
{
    options().maximumScreenSpaceError() = maximumScreenSpaceError;
    if (_tilesetNode)
    {
        _tilesetNode->setMaximumScreenSpaceError(maximumScreenSpaceError);
    }
}

osg::Node*
ThreeDTilesLayer::getNode() const
{
    return _tilesetNode.get();
}
