/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2018 Pelican Mapping
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
#include <osgEarth/TDTiles>
#include <osgEarth/Utils>
#include <osgEarth/Registry>
#include <osg/PagedLOD>
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>

using namespace osgEarth;

#define LC "[3DTiles] "

//........................................................................

#define PSEUDOLOADER_TILE_CONTENT_EXT ".osgearth_3dtiles_tile"
#define TAG_TILENODE "osgEarth::TDTiles::TileNode"

namespace osgEarth { namespace TDTiles
{
    struct TileContentPseudoloader : public osgDB::ReaderWriter
    {
        ReadResult readNode(const std::string& location, const osgDB::Options* options) const
        {
            if (location != PSEUDOLOADER_TILE_CONTENT_EXT)
                return ReadResult::FILE_NOT_HANDLED;

            osg::ref_ptr<TDTiles::TileNode> tileNode = OptionsData<TDTiles::TileNode>::get(options, TAG_TILENODE);
            if (!tileNode.valid())
                return ReadResult::FILE_NOT_FOUND;

            osg::ref_ptr<osg::Node> result = tileNode->readContent();
            return ReadResult(result.release());
        }
    };

    REGISTER_OSGPLUGIN(osgearth_3dtiles_tile, TileContentPseudoloader);
}}

//........................................................................

void
TDTiles::Asset::fromConfig(const Config& conf)
{
    conf.get("version", version());
    conf.get("tilesetVersion", tilesetVersion());
}

Config
TDTiles::Asset::getConfig() const
{
    Config conf;
    conf.set("version", version());
    conf.set("tilesetVersion", tilesetVersion());
    return conf;
}

//........................................................................

void
TDTiles::BoundingVolume::fromConfig(const Config& conf)
{
}

Config
TDTiles::BoundingVolume::getConfig() const
{
    Config conf;
    return conf;
}

//........................................................................

void
TDTiles::TileContent::fromConfig(const Config& conf)
{
    conf.get("boundingVolume", boundingVolume());
    conf.get("uri", uri());
}

Config
TDTiles::TileContent::getConfig() const
{
    Config conf;
    conf.set("boundingVolume", boundingVolume());
    conf.set("uri", uri());
    return conf;
}

//........................................................................

void
TDTiles::Tile::fromConfig(const Config& conf)
{
    conf.get("boundingVolume", boundingVolume());
    conf.get("viewerRequestVolume", viewerRequestVolume());
    conf.get("geometricError", geometricError());
    conf.get("refine", "replace", refine(), REFINE_REPLACE);
    conf.get("refine", "add", refine(), REFINE_ADD);
    conf.get("content", content());
}

Config
TDTiles::Tile::getConfig() const
{
    Config conf;
    conf.set("boundingVolume", boundingVolume());
    conf.set("viewerRequestVolume", viewerRequestVolume());
    conf.set("geometricError", geometricError());
    conf.set("refine", "replace", refine(), REFINE_REPLACE);
    conf.set("refine", "add", refine(), REFINE_ADD);
    conf.set("content", content());
    return conf;
}

//........................................................................

void
TDTiles::Tileset::fromConfig(const Config& conf)
{
    conf.get("asset", asset());
    conf.get("boundingVolume", boundingVolume());
    conf.get("geometricError", geometricError());
    if (conf.hasChild("root"))
        root() = new Tile(conf.child("root"));
}

Config
TDTiles::Tileset::getConfig() const
{
    Config conf;
    conf.set("asset", asset());
    conf.set("boundingVolume", boundingVolume());
    conf.set("geometricError", geometricError());
    if (root().valid())
        conf.set("root", root()->getConfig());
    return conf;
}

//........................................................................

TDTiles::TileNode::TileNode(TDTiles::Tile* tile, 
                            TDTiles::ContentHandler* handler, 
                            const osgDB::Options* readOptions,
                            bool preloadContent) :
    _tile(tile),
    _handler(handler),
    _readOptions(readOptions)
{
    // the transform to localize this tile:
    setMatrix(tile->transform().get());

    osg::PagedLOD* plod = new osg::PagedLOD();
    plod->setRangeMode(osg::LOD::PIXEL_SIZE_ON_SCREEN);
    float geometricError = tile->geometricError().getOrUse(0.0f);

    osg::ref_ptr<osgDB::Options> newOptions = Registry::instance()->cloneOrCreateOptions(readOptions);
    OptionsData<TDTiles::TileNode>::set(newOptions.get(), TAG_TILENODE, this);
    plod->setDatabaseOptions(newOptions.get());

    if (preloadContent)
    {
        // Load the content immediately:
        osg::ref_ptr<osg::Node> content = readContent();
        if (content.valid())
        {
            plod->addChild(content.get(), handler->getMaxScreenSpaceError(), FLT_MAX);
        }
    }
    else
    {
        // Set the radius to 1/2 the geometric error
        plod->setRadius(geometricError > 0.0f ? 0.5f*geometricError : -1.0f);
        plod->setFileName(0, PSEUDOLOADER_TILE_CONTENT_EXT);
        plod->setRange(0, handler->getMaxScreenSpaceError(), FLT_MAX);
    }


    addChild(plod);

    if (tile->children().size() > 0)
    {
        osg::Group* children = new osg::Group();
        plod->addChild(children);
        unsigned childrenIndex = plod->getNumChildren()-1u;

        for(std::vector<osg::ref_ptr<TDTiles::Tile> >::iterator i = tile->children().begin();
            i != tile->children().end();
            ++i)
        {
            TDTiles::Tile* childTile = i->get();
            if (childTile)
            {
                // create a new TileNode and immediately load its content
                TileNode* child = new TileNode(childTile, handler, readOptions, true);
                children->addChild(child);
            }
        }

        plod->setRange(childrenIndex, handler->getMaxScreenSpaceError(), FLT_MAX);
    }
}

osg::ref_ptr<osg::Node>
TDTiles::TileNode::readContent() const
{
    osg::ref_ptr<osg::Node> result;

    if (osgDB::getLowerCaseFileExtension(_tile->content()->uri()->full()) == "json")
    {
        // external tileset... handle somehow
        OE_INFO << LC << "Tile content might be an external dataset! Do something." << std::endl;
    }

    else if (_handler.valid())
    {
        // Custom handler? invoke it now
        result = _handler->createNode(_tile.get(), _readOptions.get());
    }

    return result;
}

//........................................................................

TDTiles::ContentHandler::ContentHandler() :
    _maxSSE(1.0f)
{
    //nop
}

osg::ref_ptr<osg::Node>
TDTiles::ContentHandler::createNode(TDTiles::Tile* tile, const osgDB::Options* readOptions) const
{
    osg::ref_ptr<osg::Node> result;

    OE_INFO << LC << "Loading content, URI = " << tile->content()->uri()->full() << std::endl;

    // default action: just try to load a node using OSG
    osgEarth::ReadResult rr = tile->content()->uri()->readNode(readOptions);
    if (rr.succeeded())
    {
        result = rr.releaseNode();
    }
    else
    {
        OE_WARN << LC << "Read error: " << rr.errorDetail() << std::endl;
    }
    return result;
}

void
TDTiles::ContentHandler::setMaxScreenSpaceError(float value)
{
    _maxSSE = value;
}

float
TDTiles::ContentHandler::getMaxScreenSpaceError() const
{
    return _maxSSE;
}

//........................................................................

TDTilesetGroup::TDTilesetGroup()
{
    setRangeMode(PIXEL_SIZE_ON_SCREEN);
    _handler = new TDTiles::ContentHandler();
}

TDTilesetGroup::TDTilesetGroup(TDTiles::ContentHandler* handler) :
    _handler(handler)
{
    setRangeMode(PIXEL_SIZE_ON_SCREEN);
    if (!_handler.valid())
        _handler = new TDTiles::ContentHandler();
}

void
TDTilesetGroup::setReadOptions(const osgDB::Options* value)
{
    _readOptions = value;
}

TDTiles::ContentHandler*
TDTilesetGroup::getContentHandler() const
{
    return _handler.get();
}

void
TDTilesetGroup::setTileset(TDTiles::Tileset* tileset)
{
    // clear out the node in preparation for a new tileset
    removeChildren(0, getNumChildren());

    _tileset = tileset;

    // Set up the root tile.
    if (_tileset->root().valid())
    {
        // create the root tile node and defer loading of its content:
        TDTiles::TileNode* tileNode = new TDTiles::TileNode(_tileset->root().get(), _handler.get(), _readOptions.get(), false);
        float minError = _tileset->geometricError().getOrUse(0.0f);
        float maxError = FLT_MAX;
        addChild(tileNode, minError, maxError);
    }
}

void
TDTilesetGroup::setTilesetURL(const URI& location)
{
    //TODO
    OE_WARN << "TODO" << std::endl;
}
