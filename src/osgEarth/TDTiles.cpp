/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#include <osgEarth/Async>
#include <osgEarth/Utils>
#include <osgEarth/Registry>
#include <osgEarth/URI>
#include <osgDB/FileNameUtils>

using namespace osgEarth;

#define LC "[3DTiles] "

//........................................................................

#define PSEUDOLOADER_TILE_CONTENT_EXT "osgearth_3dtiles_content"
#define PSEUDOLOADER_TILE_CHILDREN_EXT "osgearth_3dtiles_children"
#define PSEUDOLOADER_EXTERNAL_TILESET_EXT "osgearth_3dtiles_external_tileset"
#define TAG_INVOKER "osgEarth::TDTiles::Invoker"

namespace osgEarth { namespace TDTiles
{
    //! Operation that loads all of a tilenode's children.
    struct LoadChildren : public AsyncFunction
    {
        osg::observer_ptr<TDTiles::TileNode> _tileNode;
        LoadChildren(TDTiles::TileNode* tileNode) : _tileNode(tileNode) { }
        virtual ReadResult operator()() const
        {
            OE_DEBUG << LC << "LoadChildren" << std::endl;
            osg::ref_ptr<TDTiles::TileNode> tileNode;
            if (_tileNode.lock(tileNode))
            {
                return tileNode->loadChildren();
            }
            else return ReadResult(ReadResult::RESULT_NOT_FOUND);
        }
    };

    //! Operation that loads one child of a TileNode.
    struct LoadChild : public AsyncFunction
    {
        osg::observer_ptr<TDTiles::TileNode> _tileNode;
        unsigned _index;
        LoadChild(TDTiles::TileNode* tileNode, unsigned index) 
            : _tileNode(tileNode), _index(index) { }
        virtual ReadResult operator()() const
        {
            OE_DEBUG << LC << "LoadChild" << std::endl;
            osg::ref_ptr<TDTiles::TileNode> tileNode;
            if (_tileNode.lock(tileNode))
            {
                return tileNode->loadChild(_index);
            }
            else return ReadResult(ReadResult::RESULT_NOT_FOUND);
        }
    };

    //! Operation that loads the content of a TileNode (from a URI)
    struct LoadTileContent : public AsyncFunction
    {
        osg::observer_ptr<TDTiles::TileNode> _tileNode;
        LoadTileContent(TDTiles::TileNode* tileNode) : _tileNode(tileNode) { }
        virtual ReadResult operator()() const
        {
            OE_DEBUG << LC << "LoadTileContent" << std::endl;
            osg::ref_ptr<TDTiles::TileNode> tileNode;
            if (_tileNode.lock(tileNode))
            {
                return tileNode->loadContent();
            }
            else return ReadResult(ReadResult::RESULT_NOT_FOUND);
        }
    };

    //! Operation that loads an external tile set
    struct LoadExternalTileset : public AsyncFunction
    {
        osg::observer_ptr<TDTilesetGroup> _group;
        LoadExternalTileset(TDTilesetGroup* group) : _group(group) { }
        virtual ReadResult operator()() const
        {
            OE_DEBUG << LC << "LoadExternalTileset" << std::endl;
            osg::ref_ptr<TDTilesetGroup> group;
            if (_group.lock(group))
            {
                const URI& uri = group->getTilesetURL();
                OE_INFO << LC << "Loading external tileset " << uri.full() << std::endl;

                ReadResult r = uri.readString(group->getReadOptions());
                if (r.succeeded())
                {
                    TDTiles::Tileset* tileset = TDTiles::Tileset::create(r.getString(), uri.context());
                    if (tileset)
                    {
                        return group->loadRoot(tileset);
                    }
                    else
                    {
                        return ReadResult("Invalid tileset JSON");
                    }
                }
                return ReadResult("Failed to load external tileset");
            }
            else return ReadResult();
        }
    };
}}

//........................................................................

void
TDTiles::Asset::fromJSON(const Json::Value& value)
{
    if (value.isMember("version"))
        version() = value.get("version", "").asString();
    if (value.isMember("tilesetVersion"))
        tilesetVersion() = value.get("tilesetVersion", "").asString();
}

Json::Value
TDTiles::Asset::getJSON() const
{
    Json::Value value(Json::objectValue);
    if (version().isSet())
        value["version"] = version().get();
    if (tilesetVersion().isSet())
        value["tilesetVersion"] = tilesetVersion().get();
    return value;
}

//........................................................................

void
TDTiles::BoundingVolume::fromJSON(const Json::Value& value)
{
    if (value.isMember("region"))
    {
        const Json::Value& a = value["region"];
        if (a.isArray() && a.size() == 6)
        {
            Json::Value::const_iterator i = a.begin();
            region()->xMin() = (*i++).asDouble();
            region()->yMin() = (*i++).asDouble();
            region()->xMax() = (*i++).asDouble();
            region()->yMax() = (*i++).asDouble();
            region()->zMin() = (*i++).asDouble();
            region()->zMax() = (*i++).asDouble();
        }
        else OE_WARN << "Invalid region array" << std::endl;
    }

    if (value.isMember("sphere"))
    {
        const Json::Value& a = value["sphere"];
        if (a.isArray() && a.size() == 4)
        {
            Json::Value::const_iterator i = a.begin();
            sphere()->center().x() = (*i++).asDouble();
            sphere()->center().y() = (*i++).asDouble();
            sphere()->center().z() = (*i++).asDouble();
            sphere()->radius()     = (*i++).asDouble();
        }
    }
    if (value.isMember("box"))
    {
        const Json::Value& a = value["box"];
        if (a.isArray() && a.size() == 12)
        {
            Json::Value::const_iterator i = a.begin();
            osg::Vec3 center((*i++).asDouble(), (*i++).asDouble(), (*i++).asDouble());
            osg::Vec3 xvec((*i++).asDouble(), (*i++).asDouble(), (*i++).asDouble());
            osg::Vec3 yvec((*i++).asDouble(), (*i++).asDouble(), (*i++).asDouble());
            osg::Vec3 zvec((*i++).asDouble(), (*i++).asDouble(), (*i++).asDouble());
            box()->expandBy(center+xvec);
            box()->expandBy(center-xvec);
            box()->expandBy(center+yvec);
            box()->expandBy(center-yvec);
            box()->expandBy(center+zvec);
            box()->expandBy(center-zvec);
        }
        else OE_WARN << "Invalid box array" << std::endl;
    }
}

Json::Value
TDTiles::BoundingVolume::getJSON() const
{
    Json::Value value(Json::objectValue);

    if (region().isSet())
    {
        Json::Value a(Json::arrayValue);
        a.append(region()->xMin());
        a.append(region()->yMin());
        a.append(region()->xMax());
        a.append(region()->yMax());
        a.append(region()->zMin());
        a.append(region()->zMax());
        value["region"] = a;
    }
    else if (sphere().isSet())
    {
        Json::Value a(Json::arrayValue);
        a.append(sphere()->center().x());
        a.append(sphere()->center().y());
        a.append(sphere()->center().z());
        a.append(sphere()->radius());
        value["sphere"] = a;
    }
    else if (box().isSet())
    {
        OE_WARN << LC << "box not implemented" << std::endl;
    }
    return value;
}

osg::BoundingSphere
TDTiles::BoundingVolume::asBoundingSphere() const
{
    const SpatialReference* epsg4979 = SpatialReference::get("epsg:4979");
    if (!epsg4979)
        return osg::BoundingSphere();

    if (region().isSet())
    {
        GeoExtent extent(epsg4979,
            osg::RadiansToDegrees(region()->xMin()),
            osg::RadiansToDegrees(region()->yMin()),
            osg::RadiansToDegrees(region()->xMax()),
            osg::RadiansToDegrees(region()->yMax()));

        osg::BoundingSphered bs = extent.createWorldBoundingSphere(region()->zMin(), region()->zMax());
        return osg::BoundingSphere(bs.center(), bs.radius());
    }

    else if (sphere().isSet())
    {
        return sphere().get();
    }

    else if (box().isSet())
    {
        return osg::BoundingSphere(box()->center(), box()->radius());
    }

    return osg::BoundingSphere();
}

//........................................................................

void
TDTiles::TileContent::fromJSON(const Json::Value& value, LoadContext& lc)
{
    if (value.isMember("boundingVolume"))
        boundingVolume() = BoundingVolume(value.get("boundingVolume", Json::nullValue));
    if (value.isMember("uri"))
        uri() = URI(value.get("uri", "").asString(), lc._uc);
    if (value.isMember("url"))
        uri() = URI(value.get("url", "").asString(), lc._uc);
}

Json::Value
TDTiles::TileContent::getJSON() const
{
    Json::Value value(Json::objectValue);
    if (boundingVolume().isSet())
        value["boundingVolume"] = boundingVolume()->getJSON();
    if (uri().isSet())
        value["uri"] = uri()->base();
    return value;
}

//........................................................................

void
TDTiles::Tile::fromJSON(const Json::Value& value, LoadContext& uc)
{
    if (value.isMember("boundingVolume"))
        boundingVolume() = value["boundingVolume"];
    if (value.isMember("viewerRequestVolume"))
        viewerRequestVolume() = value["viewerRequestVolume"];
    if (value.isMember("geometricError"))
        geometricError() = value.get("geometricError", 0.0).asDouble();
    if (value.isMember("content"))
        content() = TileContent(value["content"], uc);

    if (value.isMember("refine"))
    {
        refine() = osgEarth::ciEquals(value["refine"].asString(), "add") ? REFINE_ADD : REFINE_REPLACE;
        uc._defaultRefine = refine().get();
    }
    else
    {
        refine() = uc._defaultRefine;
    }

    if (value.isMember("transform"))
    {
        const Json::Value& digits = value["transform"];
        double c[16];
        if (digits.isArray() && digits.size() == 16)
        {
            unsigned k=0;
            for(Json::Value::const_iterator i = digits.begin(); i != digits.end(); ++i)
                c[k++] = (*i).asDouble();
            transform() = osg::Matrix(c);
        }
    }

    if (value.isMember("children"))
    {
        const Json::Value& a = value["children"];
        if (a.isArray())
        {
            for (Json::Value::const_iterator i = a.begin(); i != a.end(); ++i)
            {
                osg::ref_ptr<Tile> tile = new Tile(*i, uc);
                children().push_back(tile.get());
            }
        }
    }
}

Json::Value
TDTiles::Tile::getJSON() const
{
    Json::Value value(Json::objectValue);

    if (boundingVolume().isSet())
        value["boundingVolume"] = boundingVolume()->getJSON();
    if (viewerRequestVolume().isSet())
        value["viewerRequestVolume"] = viewerRequestVolume()->getJSON();
    if (geometricError().isSet())
        value["geometricError"] = geometricError().get();
    if (refine().isSet())
        value["refine"] = (refine().get() == REFINE_ADD) ? "add" : "replace";
    if (content().isSet())
        value["content"] = content()->getJSON();


    if (!children().empty())
    {
        Json::Value collection(Json::arrayValue);
        for(unsigned i=0; i<children().size(); ++i)
        {
            Tile* child = children()[i].get();
            if (child)
            {
                collection.append(child->getJSON());
            }
        }
        value["children"] = collection;
    }

    return value;
}

//........................................................................

void
TDTiles::Tileset::fromJSON(const Json::Value& value, LoadContext& uc)
{
    if (value.isMember("asset"))
        asset() = Asset(value.get("asset", Json::nullValue));
    if (value.isMember("boundingVolume"))
        boundingVolume() = BoundingVolume(value.get("boundingVolume", Json::nullValue));
    if (value.isMember("geometricError"))
        geometricError() = value.get("geometricError", 0.0).asDouble();
    if (value.isMember("root"))
        root() = new Tile(value["root"], uc);
}

Json::Value
TDTiles::Tileset::getJSON() const
{
    Json::Value value(Json::objectValue);
    if (asset().isSet())
        value["asset"] = asset()->getJSON();
    if (boundingVolume().isSet())
        value["boundingVolume"] = boundingVolume()->getJSON();
    if (geometricError().isSet())
        value["geometricError"] = geometricError().get();
    if (root().valid())
        value["root"] = root()->getJSON();
    return value;
}

TDTiles::Tileset*
TDTiles::Tileset::create(const std::string& json, const URIContext& uc)
{
    Json::Reader reader;
    Json::Value root(Json::objectValue);
    if (!reader.parse(json, root, false))
        return NULL;

    LoadContext lc;
    lc._uc = uc;
    lc._defaultRefine = REFINE_REPLACE;

    return new TDTiles::Tileset(root, lc);
}

//........................................................................

TDTiles::TileNode::TileNode(TDTiles::Tile* tile, 
                            TDTiles::ContentHandler* handler, 
                            const osgDB::Options* readOptions) :
    _tile(tile),
    _handler(handler),
    _readOptions(readOptions)
{
    // the transform to localize this tile:
    if (tile->transform().isSet())
    {
        setMatrix(tile->transform().get());
    }

    // install a bounding volume:
    osg::BoundingSphere bs;
    if (tile->boundingVolume().isSet())
    {
        bs = tile->boundingVolume()->asBoundingSphere();

        // if both a transform and a region are set, assume the radius is
        // fine but the center point should be localized to the transform. -gw
        // (just guessing)
        if (tile->transform().isSet() && tile->boundingVolume()->region().isSet())
        {
            bs.center().set(0,0,0);
        }
    }
    // tag this object as the invoker of the paging request:
    osg::ref_ptr<osgDB::Options> newOptions = Registry::instance()->cloneOrCreateOptions(readOptions);
    OptionsData<TDTiles::TileNode>::set(newOptions.get(), TAG_INVOKER, this);

    // aka "maximum meters per pixel for which to use me"
    float geometricError = tile->geometricError().getOrUse(FLT_MAX);

    // actual content for this tile (optional)
    osg::ref_ptr<osg::Node> contentNode = loadContent();

    if (tile->refine() == REFINE_REPLACE)
    {
        //osg::ref_ptr<GeometricErrorPagedLOD> plod = new GeometricErrorPagedLOD(_handler.get());
        osg::ref_ptr<AsyncLOD> lod = new AsyncLOD();
        lod->setMode(AsyncLOD::MODE_GEOMETRIC_ERROR);
        lod->setPolicy(AsyncLOD::POLICY_REPLACE);

        if (bs.valid())
        {
            lod->setCenter(bs.center());
            lod->setRadius(bs.radius());
        }

        if (contentNode.valid())
        {
            lod->setName(_tile->content()->uri()->base());
            lod->addChild(contentNode, 0.0f, FLT_MAX);
        }

        if (tile->children().size() > 0)
        {
            // Async children as a group. All must load before replacing child 0.
            // TODO: consider a way to load each child asyncrhonously but still
            // block the refinement until all are loaded.
            lod->addChild(new LoadChildren(this), 0.0f, geometricError);
        }

        addChild(lod);
    }

    else // tile->refine() == REFINE_ADD
    {
        if (contentNode.valid())
        {
            addChild(contentNode.get());
        }

        for (unsigned i = 0; i < _tile->children().size(); ++i)
        {
            // Each tile gets its own async load since they don't depend on each other
            // nor do they depend on a parent loading first.
            osg::ref_ptr<AsyncLOD> lod = new AsyncLOD();
            lod->setMode(AsyncLOD::MODE_GEOMETRIC_ERROR);
            lod->setPolicy(AsyncLOD::POLICY_ACCUMULATE);

            TDTiles::Tile* childTile = _tile->children()[i].get();

            if (childTile->boundingVolume().isSet())
            {
                osg::BoundingSphere bs = childTile->boundingVolume()->asBoundingSphere();
                if (bs.valid())
                {
                    lod->setCenter(bs.center());
                    lod->setRadius(bs.radius());
                }
            }

            // backup plan if the child's BV isn't set - use parent BV
            else if (bs.valid())
            {
                lod->setCenter(bs.center());
                lod->setRadius(bs.radius());
            }

            // Load this child asynchronously:
            lod->addChild(new LoadChild(this, i), 0.0, geometricError);

            addChild(lod);
        }
    }
}

osg::ref_ptr<osg::Node>
TDTiles::TileNode::loadChildren() const
{
    osg::ref_ptr<osg::Group> children = new osg::Group();

    for (std::vector<osg::ref_ptr<TDTiles::Tile> >::iterator i = _tile->children().begin();
        i != _tile->children().end();
        ++i)
    {
        TDTiles::Tile* childTile = i->get();
        if (childTile)
        {
            // create a new TileNode:
            TileNode* child = new TileNode(childTile, _handler.get(), _readOptions.get());
            children->addChild(child);
        }
    }

    return children;
}

osg::ref_ptr<osg::Node>
TDTiles::TileNode::loadContent() const
{
    osg::ref_ptr<osg::Node> result;

    if (osgDB::getLowerCaseFileExtension(_tile->content()->uri()->base()) == "json")
    {
        // external tileset reference
        TDTilesetGroup* group = new TDTilesetGroup();
        group->setTilesetURL(_tile->content()->uri().get());
        result = group;
    }
    else if (_handler.valid())
    {
        // Custom handler? invoke it now
        result = _handler->createNode(_tile.get(), _readOptions.get());
    }

    return result;
}

osg::ref_ptr<osg::Node>
TDTiles::TileNode::loadChild(unsigned i) const
{
    TDTiles::Tile* child = _tile->children()[i].get();
    osg::ref_ptr<TDTiles::TileNode> node = new TDTiles::TileNode(
        child, _handler.get(), _readOptions.get());
    return node;
}

//........................................................................

TDTiles::ContentHandler::ContentHandler()
{
    //nop
}

osg::ref_ptr<osg::Node>
TDTiles::ContentHandler::createNode(TDTiles::Tile* tile, const osgDB::Options* readOptions) const
{
    osg::ref_ptr<osg::Node> result;

    // default action: just try to load a node using OSG
    if (tile->content().isSet() && 
        tile->content()->uri().isSet() &&
        !tile->content()->uri()->empty())
    {
        Registry::instance()->startActivity(tile->content()->uri()->base());

        osgEarth::ReadResult rr = tile->content()->uri()->readNode(readOptions);
        if (rr.succeeded())
        {
            result = rr.releaseNode();
        }
        else
        {
            OE_WARN << LC << "Read error: " << rr.errorDetail() << std::endl;
        }
        Registry::instance()->endActivity(tile->content()->uri()->base());
    }
    return result;
}

//........................................................................

TDTilesetGroup::TDTilesetGroup()
{
    _handler = new TDTiles::ContentHandler();
}

TDTilesetGroup::TDTilesetGroup(TDTiles::ContentHandler* handler) :
    _handler(handler)
{
    if (!_handler.valid())
    {
        _handler = new TDTiles::ContentHandler();
    }
}

void
TDTilesetGroup::setReadOptions(const osgDB::Options* value)
{
    _readOptions = value;
}

const osgDB::Options*
TDTilesetGroup::getReadOptions() const
{
    return _readOptions.get();
}

TDTiles::ContentHandler*
TDTilesetGroup::getContentHandler() const
{
    return _handler.get();
}

osg::ref_ptr<osg::Node>
TDTilesetGroup::loadRoot(TDTiles::Tileset* tileset) const
{
    osg::ref_ptr<osg::Node> result;

    // Set up the root tile.
    if (tileset->root().valid())
    {
        float maxMetersPerPixel = tileset->geometricError().getOrUse(FLT_MAX);

        // create the root tile node and defer loading of its content:
        osg::Node* tileNode = new TDTiles::TileNode(tileset->root().get(), _handler.get(), _readOptions.get());

        AsyncLOD* lod = new AsyncLOD();
        lod->setMode(AsyncLOD::MODE_GEOMETRIC_ERROR);
        lod->addChild(tileNode, 0.0, maxMetersPerPixel);

        result = lod;
    }

    return result;
}

void
TDTilesetGroup::setTileset(TDTiles::Tileset* tileset)
{
    // clear out the node in preparation for a new tileset
    removeChildren(0, getNumChildren());

    // create the root tile node and defer loading of its content:
    osg::ref_ptr<osg::Node> tileNode = loadRoot(tileset);
    if (tileNode.valid())
    {
        addChild(tileNode);
    }
}

void
TDTilesetGroup::setTilesetURL(const URI& location)
{
    _tilesetURI = location;

    // reset:
    removeChildren(0, getNumChildren());

    AsyncLOD* lod = new AsyncLOD();
    lod->setMode(AsyncLOD::MODE_GEOMETRIC_ERROR);
    lod->setName(location.base());
    lod->addChild(new TDTiles::LoadExternalTileset(this), 0.0, FLT_MAX);

    addChild(lod);
}

const URI&
TDTilesetGroup::getTilesetURL() const
{
    return _tilesetURI;
}
