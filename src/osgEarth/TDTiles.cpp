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
#include <osgEarth/Async>
#include <osgEarth/Utils>
#include <osgEarth/Registry>
#include <osgEarth/URI>
#include <osgEarth/OGRFeatureSource>
#include <osgEarth/FeatureCursor>
#include <osgEarth/ResampleFilter>
#include <osgEarth/FeatureNode>
#include <osgEarth/StyleSheet>
#include <osgEarth/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/WriteFile>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Contrib;
using namespace osgEarth::Contrib::TDTiles;

#define LC "[3DTiles] "

//........................................................................

#define PSEUDOLOADER_LOAD_TILE_CONTENT "osgearth_3dtiles_content"
#define PSEUDOLOADER_LOAD_ALL_TILE_CHILDREN "osgearth_3dtiles_load_tile_children"
#define PSEUDOLOADER_LOAD_ONE_TILE_CHILD "osgearth_3dtiles_load_tile_child"
#define PSEUDOLOADER_LOAD_EXTERNAL_TILESET "osgearth_3dtiles_load_external_tileset"

#define PSEUDOLOADER_TILESET_GROUP "osgearth_3dtiles_tileset_group"
#define PSEUDOLOADER_TILE_NODE "osgearth_3dtiles_tilenode"
#define PSEUDOLOADER_CHILD_INDEX "osgearth_3dtiles_tile_index"

namespace osgEarth { namespace Contrib { namespace TDTiles
{
    struct PagerPseudoLoader : public osgDB::ReaderWriter
    {
        PagerPseudoLoader()
        {
            supportsExtension(PSEUDOLOADER_LOAD_EXTERNAL_TILESET, PSEUDOLOADER_LOAD_EXTERNAL_TILESET);
            supportsExtension(PSEUDOLOADER_LOAD_ALL_TILE_CHILDREN, PSEUDOLOADER_LOAD_ALL_TILE_CHILDREN);
            supportsExtension(PSEUDOLOADER_LOAD_ONE_TILE_CHILD, PSEUDOLOADER_LOAD_ONE_TILE_CHILD);
            supportsExtension(PSEUDOLOADER_LOAD_TILE_CONTENT, PSEUDOLOADER_LOAD_TILE_CONTENT);
        }

        ReadResult readNode(const std::string& location, const osgDB::Options* rwOptions) const
        {
            std::string lcfe = osgDB::getLowerCaseFileExtension(location);

            if (lcfe == PSEUDOLOADER_LOAD_EXTERNAL_TILESET)
            {
                OE_WARN << "lcfe=[" << lcfe << "]" << std::endl;
                osg::ref_ptr<TDTilesetGroup> tilesetGroup = OptionsData<TDTilesetGroup>::get(rwOptions, PSEUDOLOADER_TILESET_GROUP);
                if (!tilesetGroup.valid())
                    return ReadResult("osgEarth: INTERNAL ERROR in PagerPseudoLoader (no tilesetGroup in options)");

                return tilesetGroup->loadExternal();
            }

            else if (lcfe == PSEUDOLOADER_LOAD_ALL_TILE_CHILDREN)
            {

                OE_WARN << "lcfe=[" << lcfe << "]" << std::endl;
                osg::ref_ptr<TDTiles::TileNode> tileNode = OptionsData<TDTiles::TileNode>::get(rwOptions, PSEUDOLOADER_TILE_NODE);
                if (!tileNode.valid())
                    return ReadResult("osgEarth: INTERNAL ERROR in PagerPseudoLoader (no tileNode in options)");

                return tileNode->loadChildren();
            }

            else if (lcfe == PSEUDOLOADER_LOAD_ONE_TILE_CHILD)
            {

                OE_WARN << "lcfe=[" << lcfe << "]" << std::endl;
                osg::ref_ptr<TDTiles::TileNode> tileNode = OptionsData<TDTiles::TileNode>::get(rwOptions, PSEUDOLOADER_TILE_NODE);
                if (!tileNode.valid())
                    return ReadResult("osgEarth: INTERNAL ERROR in PagerPseudoLoader (no tileNode in options)");

                unsigned index = 0u;
                if (!rwOptions->getUserValue(PSEUDOLOADER_CHILD_INDEX, index))
                    return ReadResult("osgEarth: INTERNAL ERROR in PagerPseudoLoader (no tile index provided)");

                return tileNode->loadChild(index);
            }

            else if (lcfe == PSEUDOLOADER_LOAD_TILE_CONTENT)
            {

                OE_WARN << "lcfe=[" << lcfe << "]" << std::endl;
                osg::ref_ptr<TDTiles::TileNode> tileNode = OptionsData<TDTiles::TileNode>::get(rwOptions, PSEUDOLOADER_TILE_NODE);
                if (!tileNode.valid())
                    return ReadResult("osgEarth: INTERNAL ERROR in PagerPseudoLoader (no tileNode in options)");

                return tileNode->loadContent();
            }

            return ReadResult::FILE_NOT_HANDLED;
        }
    };
    REGISTER_OSGPLUGIN(osgearth_pseudo_3dtiles, PagerPseudoLoader);
}}}

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
        refine() = osgEarth::ciEquals(value["refine"].asString(), "ADD") ? REFINE_ADD : REFINE_REPLACE;
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
        value["refine"] = (refine().get() == REFINE_ADD) ? "ADD" : "REPLACE";
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
        if (tile->transform().isSet() && tile->boundingVolume()->region().isSet())
        {
            // (actually don't - this was the old way but not the new way)
            //bs.center().set(0,0,0);
        }
    }

    // aka "maximum meters per pixel for which to use me"
    float geometricError = tile->geometricError().getOrUse(FLT_MAX);

    // actual content for this tile (optional)
    osg::ref_ptr<osg::Node> contentNode = loadContent();

    if (tile->refine() == REFINE_REPLACE)
    {
        osg::ref_ptr<osg::PagedLOD> lod = new osg::PagedLOD();
        lod->setRangeMode(lod->PIXEL_SIZE_ON_SCREEN);

        if (bs.valid())
        {
            lod->setCenter(bs.center());
            lod->setRadius(bs.radius());
        }

        if (contentNode.valid())
        {
            lod->setName(_tile->content()->uri()->base());
            //lod->addChild(contentNode, 0.0f, FLT_MAX);
            lod->addChild(contentNode, bs.valid() ? 1.0f : 0.0f, FLT_MAX);
        }

        if (tile->children().size() > 0)
        {
            osg::ref_ptr<osgDB::Options> local = Registry::instance()->cloneOrCreateOptions(readOptions);

            //lod->addChild(new LoadChildren(this), 0.0f, geometricError);
            unsigned index = getNumChildren();
            lod->setFileName(index, "." PSEUDOLOADER_LOAD_ALL_TILE_CHILDREN);
            //lod->setRange(1, geometricError, FLT_MAX); // TODO: revisit
            lod->setRange(index, bs.valid() ? 1.0f : 0.0f, FLT_MAX); 
            lod->setDatabaseOptions(local.get());
            OptionsData<TileNode>::set(local, PSEUDOLOADER_TILE_NODE, this);
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
            osg::ref_ptr<osg::PagedLOD> lod = new osg::PagedLOD();
            lod->setRangeMode(lod->PIXEL_SIZE_ON_SCREEN);
            // TODO: deal with POLICY_ACCUMULATE ...
                
            //osg::ref_ptr<AsyncLOD> lod = new AsyncLOD();
            //lod->setMode(AsyncLOD::MODE_GEOMETRIC_ERROR);
            //lod->setPolicy(AsyncLOD::POLICY_ACCUMULATE);

            TDTiles::Tile* childTile = _tile->children()[i].get();

            osg::BoundingSphere myBS;

            if (childTile->boundingVolume().isSet())
            {
                osg::BoundingSphere bs = childTile->boundingVolume()->asBoundingSphere();
                if (bs.valid())
                {
                    myBS = bs;
                    lod->setCenter(bs.center());
                    lod->setRadius(bs.radius());
                }
            }

            // backup plan if the child's BV isn't set - use parent BV
            else if (bs.valid())
            {
                myBS = bs;
                lod->setCenter(bs.center());
                lod->setRadius(bs.radius());
            }

            // Load this child asynchronously:
            osg::ref_ptr<osgDB::Options> local = Registry::instance()->cloneOrCreateOptions(readOptions);
            lod->setFileName(0, "." PSEUDOLOADER_LOAD_ONE_TILE_CHILD);
            lod->setRange(0, myBS.valid() ? 1.0f : 0.0f, FLT_MAX);
            lod->setDatabaseOptions(local.get());
            OptionsData<TileNode>::set(local.get(), PSEUDOLOADER_TILE_NODE, this);
            local->setUserValue(PSEUDOLOADER_CHILD_INDEX, (unsigned)i);

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
        group->setExternalTilesetURL(_tile->content()->uri().get(), getBound());
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
            result->setName(tile->content()->uri()->full());
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
        TDTiles::TileNode* tileNode = new TDTiles::TileNode(tileset->root().get(), _handler.get(), _readOptions.get());
        tileNode->setName("Tileset Root");

        //AsyncLOD* lod = new AsyncLOD();
        //lod->setMode(AsyncLOD::MODE_GEOMETRIC_ERROR);
        //lod->addChild(tileNode, 0.0, maxMetersPerPixel);
        //lod->setName("Root ALOD");

        osg::ref_ptr<osg::PagedLOD> lod = new osg::PagedLOD();
        lod->setName("TileSet Root");
        lod->setRangeMode(lod->PIXEL_SIZE_ON_SCREEN);
        lod->addChild(tileNode, 1.0f, FLT_MAX);
        
        //osg::ref_ptr<osgDB::Options> local = Registry::instance()->cloneOrCreateOptions(_readOptions.get());
        //OptionsData<TDTiles::TileNode>::set(local.get(), PSEUDOLOADER_TILE_NODE, tileNode);
        //lod->setDatabaseOptions(local.get());

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
TDTilesetGroup::setExternalTilesetURL(const URI& location, const osg::BoundingSphere& bound)
{
    _tilesetURI = location;

    // reset:
    removeChildren(0, getNumChildren());

    osg::PagedLOD* lod = new osg::PagedLOD();
    lod->setCenter(bound.center());
    lod->setRadius(bound.radius());
    lod->setName(location.base());
    lod->setRangeMode(lod->PIXEL_SIZE_ON_SCREEN);
    lod->setFileName(0, "." PSEUDOLOADER_LOAD_EXTERNAL_TILESET);
    lod->setRange(0, bound.valid() ? 1.0f : 0.0f, FLT_MAX);

    osg::ref_ptr<osgDB::Options> local = Registry::instance()->cloneOrCreateOptions(_readOptions.get());
    OptionsData<TDTilesetGroup>::set(local.get(), PSEUDOLOADER_TILESET_GROUP, this);
    lod->setDatabaseOptions(local.get());
    addChild(lod);
}

const URI&
TDTilesetGroup::getExternalTilesetURL() const
{
    return _tilesetURI;
}

osgDB::ReaderWriter::ReadResult
TDTilesetGroup::loadExternal() const
{
    const URI& uri = getExternalTilesetURL();
    OE_INFO << LC << "Loading external tileset " << uri.full() << std::endl;

    ReadResult r = uri.readString(getReadOptions());
    if (r.succeeded())
    {
        TDTiles::Tileset* tileset = TDTiles::Tileset::create(r.getString(), uri.full());
        if (tileset)
        {
            return loadRoot(tileset);
        }
        else
        {
            return osgDB::ReaderWriter::ReadResult("Invalid TileSet JSON");
        }
    }
    return osgDB::ReaderWriter::ReadResult(r.errorDetail());
}

//........................................................................

namespace osgEarth { namespace Contrib { namespace TDTiles
{
    struct Env
    {
        const Map* map;
        osg::ref_ptr<const Profile> profile;
        unsigned gridLOD;
        osg::ref_ptr<OGRFeatureSource> input;
        Query query;
        std::vector<float> geometricError;
        osg::ref_ptr<StyleSheet> sheet;
        std::vector<const Style*> style;
        unsigned maxDepth;
        URIContext uriContext;
        unsigned counter;
        GeoExtent extent;
        std::stringstream nameBuf;
        unsigned maxPointsPerTile;
        std::string format;
    };

    void buildContent(Env& env, const GeoExtent& dataExtent, const FeatureList& features, TDTiles::Tile* tile, int depth)
    {
        tile->boundingVolume()->region()->set(
            osg::DegreesToRadians(dataExtent.xMin()), osg::DegreesToRadians(dataExtent.yMin()), 0.0,
            osg::DegreesToRadians(dataExtent.xMax()), osg::DegreesToRadians(dataExtent.yMax()), 1.0);

        tile->geometricError() = env.geometricError[depth];

        std::string url = Stringify() << "data_" << env.counter++ << ".shp";

        // TODO: get the "refine" right
        tile->refine() = TDTiles::REFINE_ADD;

        bool writeGLTF = true;
        if (writeGLTF)
        {
            std::string u = Stringify() << url << "." << env.format;
            tile->content()->uri() = URI(u, env.uriContext);
            Session* session = new Session(env.map);
            session->setStyles(env.sheet.get());
            FilterContext fc(session, new FeatureProfile(dataExtent), dataExtent);
            GeometryCompiler gc;
            FeatureList copy = features; 
            osg::ref_ptr<osg::Node> node = gc.compile(copy, *env.style[depth], fc);
            osgDB::writeNodeFile(*node.get(), tile->content()->uri()->full()); 
        }

        else
        {
            URI uri(url, env.uriContext);
            tile->content()->uri() = uri;
            osg::ref_ptr<OGRFeatureSource> ogr = new OGRFeatureSource();
            ogr->setURL(uri);
            ogr->setOpenWrite(true);
            Status s = ogr->create(env.input->getFeatureProfile(), env.input->getSchema(), env.input->getGeometryType(), NULL);
            if (s.isOK())
            {
                for(FeatureList::const_iterator i = features.begin(); i != features.end(); ++i)
                {
                    ogr->insertFeature(i->get());
                }
                ogr->close();
            }
            else
            {
                OE_WARN << s.message() << std::endl;
            }
        }

        OE_INFO << "Wrote " << tile->content()->uri()->full() << std::endl;
    }

    void splitHorizontally(Env& env, const GeoExtent& dataExtent, TDTiles::Tile* parent)
    {
    }

    void splitVertically(Env& env, const GeoExtent& dataExtent, TDTiles::Tile* parent)
    {
    }

    void render(Env&, const GeoExtent&, TDTiles::Tile*, const FeatureList&, int);
    void split(Env& env, const GeoExtent& dataExtent, TDTiles::Tile* parent, const FeatureList& features, int depth);
    void populate(Env& env, const GeoExtent&, FeatureList&);
    void populate(Env& env, const GeoExtent&, const FeatureList&, FeatureList&);

    void split(Env& env, const GeoExtent& dataExtent, TDTiles::Tile* parent, const FeatureList& features, int depth)
    {
        if (features.empty())
            return;

        std::vector<double> xlist;
        std::vector<double> ylist;

        for(FeatureList::const_iterator i = features.begin(); i != features.end(); ++i)
        {
            const Feature* f = i->get();

            // find tile key containing centroid:
            const GeoExtent& ex = f->getExtent();
            double x, y;
            ex.getCentroid(x, y);

            xlist.push_back(x);
            ylist.push_back(y);
        }

        // sort the lists and find the medians.
        int i = xlist.size() / 2;
        std::sort(xlist.begin(), xlist.end());
        double xmedian = ((xlist.size() & 0x1) == 0) ? 0.5*(xlist[i - 1] + xlist[i]) : xlist[i - 1];

        i = ylist.size() / 2;
        std::sort(ylist.begin(), ylist.end());
        double ymedian = ((ylist.size() & 0x1) == 0) ? 0.5*(ylist[i - 1] + ylist[i]) : ylist[i - 1];

        GeoExtent corners[4];
        corners[0] = GeoExtent(dataExtent.getSRS(), dataExtent.west(), dataExtent.south(), xmedian, ymedian);
        corners[1] = GeoExtent(dataExtent.getSRS(), xmedian, dataExtent.south(), dataExtent.east(), ymedian);
        corners[2] = GeoExtent(dataExtent.getSRS(), dataExtent.west(), ymedian, xmedian, dataExtent.north());
        corners[3] = GeoExtent(dataExtent.getSRS(), xmedian, ymedian, dataExtent.east(), dataExtent.north());

        for (unsigned c = 0; c < 4; ++c)
        {
            FeatureList childFeatures;
            populate(env, corners[c], features, childFeatures);
            if (!childFeatures.empty())
            {
                TDTiles::Tile* tile = new TDTiles::Tile();
                render(env, corners[c], tile, childFeatures, depth);
                parent->children().push_back(tile);
            }
        }
    }

    struct TileMetadata
    {
        TileMetadata() : featureCount(0u), pointCount(0u) { }
        GeoExtent boundingExtent;
        unsigned featureCount;
        unsigned pointCount;
    };

    bool sortByArea(const osg::ref_ptr<Feature>& lhs, const osg::ref_ptr<Feature>& rhs)
    {
        return lhs->getExtent().area() > rhs->getExtent().area();
    };

    // queries all features whose centroid falls within the dataExtent
    // and populates a list of features transformed to geographic SRS.
    void populate(Env& env, const GeoExtent& dataExtent, FeatureList& features)
    {
        Query query;
        query.bounds() = dataExtent.transform(env.input->getFeatureProfile()->getSRS()).bounds();
        osg::ref_ptr<FeatureCursor> cursor = env.input->createFeatureCursor(query, NULL);
        while(cursor.valid() && cursor->hasMore())
        {
            Feature* f = cursor->nextFeature();
            if (f->getGeometry() == NULL)
                continue;

            f->transform(dataExtent.getSRS());
            osg::Vec3d c = f->getExtent().getCentroid();
            if (dataExtent.contains(c.x(), c.y()))
            {
                features.push_back(f);
            }
        }
        features.sort(sortByArea);
    }

    // queries all features whose centroid falls within the dataExtent
    // and populates a list of features transformed to geographic SRS.
    void populate(Env& env, const GeoExtent& dataExtent, const FeatureList& superset, FeatureList& subset)
    {
        for(FeatureList::const_iterator i = superset.begin(); i != superset.end(); ++i)
        {
            Feature* f = i->get();
            double x, y;
            f->getExtent().getCentroid(x, y);
            if (dataExtent.contains(x, y))
            {
                subset.push_back(f);
            }
        }
    }

    void render(Env& env, const GeoExtent& dataExtent, TDTiles::Tile* tile, const FeatureList& features, int depth)
    {
        if (depth < env.geometricError.size()-1)
        {
            tile->geometricError() = env.geometricError[depth];

            FeatureList thisFeatures;
            FeatureList childFeatures;
            int k=0;
            //int num = features.size()/3;  
            unsigned points = 0;
            for(FeatureList::const_iterator i = features.begin();
                i != features.end(); 
                ++i, ++k)
            {
                if (points > env.maxPointsPerTile)
                {
                    childFeatures.push_back(i->get());
                }
                else
                {
                    thisFeatures.push_back(i->get());
                    points += i->get()->getGeometry()->getTotalPointCount();
                }
            }

            buildContent(env, dataExtent, thisFeatures, tile, depth);

            if (!childFeatures.empty())
            {
                split(env, dataExtent, tile, childFeatures, depth+1);
            }
        }
        else
        {
            buildContent(env, dataExtent, features, tile, depth);
        }
    }

    // Build a grid of shapefiles at a particular profile LOD.
    Status buildGrid(Env& env, TDTiles::Tile* parent)
    {
        OE_INFO << LC << "Analyzing features and building grid..." << std::endl;

        typedef std::map<TileKey, osg::ref_ptr<OGRFeatureSource> > Sources;
        typedef std::map<TileKey, TileMetadata> TileMetadataMap;

        TileMetadataMap m;

        env.extent = GeoExtent(env.profile->getSRS());

        // Collect the TileKeys and bounding extents for the feature data.
        // These are usually close but not necessarily exact. The bounding extent
        // envelopes the true feature data and is used for culling; while the TileKey's
        // extent is static and is used for query.
        osg::ref_ptr<FeatureCursor> cursor = env.input->createFeatureCursor(Query(), 0L);
        while (cursor.valid() && cursor->hasMore())
        {
            // read the next feature and transform to output srs:
            Feature* feature = cursor->nextFeature();
            feature->transform(env.profile->getSRS());

            // find tile key containing centroid:
            const GeoExtent& featureExtent = feature->getExtent();
            double x, y;
            featureExtent.getCentroid(x, y);
            TileKey key = env.profile->createTileKey(x, y, env.gridLOD);

            TileMetadata& metadata = m[key];
            if (metadata.boundingExtent.isInvalid())
            {
                metadata.boundingExtent = GeoExtent(env.profile->getSRS());
            }
            metadata.boundingExtent.expandToInclude(featureExtent);
            metadata.featureCount++;
            metadata.pointCount += feature->getGeometry() ? feature->getGeometry()->getTotalPointCount() : 0;
        }

        // Now create a child Tile for each TileKey.
        OE_INFO << LC << "Building tiles..." << std::endl;
        for(TileMetadataMap::const_iterator i = m.begin();
            i != m.end();
            ++i)
        {
            const TileKey& key = i->first;
            const TileMetadata& metadata = i->second;

            TDTiles::Tile* tile = new TDTiles::Tile();

            tile->boundingVolume()->region()->set(
                osg::DegreesToRadians(metadata.boundingExtent.xMin()), osg::DegreesToRadians(metadata.boundingExtent.yMin()), 0.0,
                osg::DegreesToRadians(metadata.boundingExtent.xMax()), osg::DegreesToRadians(metadata.boundingExtent.yMax()), 1.0);

            // TODO: get the "refine" right

            parent->children().push_back(tile);

            env.extent.expandToInclude(metadata.boundingExtent);

            // Collect this cell's features to pass down the chain.
            FeatureList features;
            populate(env, key.getExtent(), features);
            render(env, key.getExtent(), tile, features, 0); // depth=0
        }

        parent->refine() = TDTiles::REFINE_ADD; // each tile will page in separately

        parent->boundingVolume()->region()->set(
            osg::DegreesToRadians(env.extent.xMin()), osg::DegreesToRadians(env.extent.yMin()), 0.0,
            osg::DegreesToRadians(env.extent.xMax()), osg::DegreesToRadians(env.extent.yMax()), 1.0);

        return Status::OK();
    }
}}}

TDTiles::TilesetFactory::TilesetFactory() :
_format("b3dm")
{
}

TDTiles::TilesetFactory::~TilesetFactory()
{
}

void
TDTiles::TilesetFactory::setMap(const Map* map)
{
    _map = map;
}

void
TDTiles::TilesetFactory::setStyleSheet(StyleSheet* sheet)
{
    _sheet = sheet;
}

void
TDTiles::TilesetFactory::setGeometryFormat(const std::string& format)
{
    _format = format;
    if (_format.empty())
        _format = "b3dm";
}

void
TDTiles::TilesetFactory::setURIContext(const URIContext& value)
{
    _uriContext = value;
}

TDTiles::Tileset*
TDTiles::TilesetFactory::create(OGRFeatureSource* source, 
                                const Query& query,
                                ProgressCallback* progress) const
{
    if (!_map || !_map->getSRS())
    {
        OE_WARN << "No map or map not open" << std::endl;
        return NULL;
    }

    if (!_sheet.valid())
    {
        OE_WARN << "Missing required stylesheet" << std::endl;
        return NULL;
    }

    osg::ref_ptr<TDTiles::Tile> root = new TDTiles::Tile();

    Env env;
    env.map = _map;
    env.input = source;
    env.format = _format;
    env.counter = 0;
    env.uriContext = _uriContext;
    env.profile = Profile::create("global-geodetic");
    env.gridLOD = 12u;
    env.maxPointsPerTile = 12500;
    env.sheet = _sheet;

    // extract the selectors from the style sheet to find the style
    // for each geometric error.
    std::map<float,const Style*> styles;
    for(StyleMap::const_iterator i = env.sheet->getStyles().begin();
        i != env.sheet->getStyles().end();
        ++i)
    {
        const Style& style = i->second;
        const RenderSymbol* render = style.get<RenderSymbol>();
        if (render && render->geometricError().isSet())
        {
            float error = render->geometricError()->as(Units::METERS);
            styles[error] = &style;
        }
    }

    if (styles.empty())
    {
        OE_WARN << "Found zero styles with a render-geometric-error" << std::endl;
        return NULL;
    }

    // copy styles (in error-order) into the env object:
    for(std::map<float,const Style*>::const_reverse_iterator i = styles.rbegin();
        i != styles.rend();
        ++i)
    {
        env.geometricError.push_back(i->first);
        env.style.push_back(i->second);
        OE_INFO << LC << "Added style \"" << i->second->getName() << "\" with geometric error " << i->first << std::endl;
    }

    root->geometricError() = env.geometricError[0] * 3.0f;

    // build the top-level grid of tiles:
    buildGrid(env, root);

    osg::ref_ptr<TDTiles::Tileset> tileset = new TDTiles::Tileset();
    tileset->root() = root.get();
    tileset->asset()->version() = "1.0";
    return tileset.release();
}

ThreeDTile::ThreeDTile(ThreeDTileset* tileset, TDTiles::Tile* tile, bool immediateLoad, osgDB::Options* options) :
    _tileset(tileset),
    _tile(tile),
    _requestedContent(false),
    _immediateLoad(immediateLoad),
    _firstVisit(true),
    _contentUnloaded(false),
    _options(options)
{
    // the transform to localize this tile:
    if (tile->transform().isSet())
    {
        setMatrix(tile->transform().get());
    }

    if (_immediateLoad && _tile->content().isSet())
    {
        //OE_NOTICE << "Immediately Loading " << _tile->content()->uri()->full() << std::endl;
        _content = _tile->content()->uri()->getNode();
    }

    /*
    if (_tile->content().isSet() && osgEarth::Strings::endsWith(_tile->content()->uri()->full(), ".b3dm"))
    {
        createBoundingSphere();
    }
    */

    if (_tile->children().size() > 0)
    {
        _children = new osg::Group;
        for (unsigned int i = 0; i < _tile->children().size(); ++i)
        {
            _children->addChild(new ThreeDTile(_tileset, _tile->children()[i], false, _options.get()));
        }

        if (_children->getNumChildren() == 0)
        {
            _children = 0;
        }
    }
}



osg::BoundingSphere ThreeDTile::computeBound() const
{
    if (_tile->boundingVolume().isSet())
    {
        return _tile->boundingVolume()->asBoundingSphere();
    }
}

bool ThreeDTile::hasContent()
{
    return _tile->content().isSet() && _tile->content()->uri().isSet();
}

bool ThreeDTile::isContentReady()
{
    resolveContent();
    return _content.valid();
}

void ThreeDTile::resolveContent()
{
    // Resolve the future 
    if (!_content.valid() && _requestedContent && _contentFuture.isAvailable())
    {
        _content = _contentFuture.get();
    }
}


namespace
{
    class LoadTilesetOperation : public osg::Operation
    {
    public:
        LoadTilesetOperation(ThreeDTileset* parentTileset, const std::string& url, osgDB::Options* options, osgEarth::Threading::Promise<osg::Node> promise) :
            _url(url),
            _promise(promise),
            _options(options),
            _parentTileset(parentTileset)
        {
        }

        void operator()(osg::Object*)
        {

            if (!_promise.isAbandoned())
            {
                osg::ref_ptr<ThreeDTilesetContent> tilesetNode;
                osg::ref_ptr< ThreeDTileset > parentTileset;
                _parentTileset.lock(parentTileset);
                if (parentTileset.valid())
                {
                    // load the tile set:
                    URI tilesetURI(_url);
                    ReadResult rr = tilesetURI.readString();

                    std::string fullPath = osgEarth::getAbsolutePath(_url);

                    osg::ref_ptr< TDTiles::Tileset> tileset = TDTiles::Tileset::create(rr.getString(), fullPath);
                    if (tileset)
                    {
                        tilesetNode = new ThreeDTilesetContent(parentTileset.get(), tileset, _options.get());
                    }
                }
                _promise.resolve(tilesetNode.get());
            }
        }

        osgEarth::Threading::Promise<osg::Node> _promise;
        osg::ref_ptr< osgDB::Options > _options;
        osg::observer_ptr< ThreeDTileset > _parentTileset;
        std::string _url;
    };

    Threading::Future<osg::Node> readTilesetAsync(ThreeDTileset* parentTileset, const std::string& url, osgDB::Options* options)
    {
        osg::ref_ptr<ThreadPool> threadPool;
        if (options)
        {
            threadPool = OptionsData<ThreadPool>::get(options, "threadpool");
        }

        Threading::Promise<osg::Node> promise;

        osg::ref_ptr< osg::Operation > operation = new LoadTilesetOperation(parentTileset, url, options, promise);

        if (operation.valid())
        {
            if (threadPool.valid())
            {
                threadPool->getQueue()->add(operation);
            }
            else
            {
                OE_WARN << "Immediately resolving async operation, please set a ThreadPool on the Options object" << std::endl;
                operation->operator()(0);
            }
        }

        return promise.getFuture();
    }
}


void ThreeDTile::requestContent(osgUtil::IncrementalCompileOperation* ico)
{
    if (!_content.valid() && !_requestedContent && hasContent())
    {
        if (osgEarth::Strings::endsWith(_tile->content()->uri()->base(), ".json"))
        {
            _contentFuture = readTilesetAsync(_tileset, _tile->content()->uri()->full(), _options.get());
        }
        else
        {
            _contentFuture = readNodeAsync(_tile->content()->uri()->full(), ico, _options.get());
        }
        _requestedContent = true;
    }
}

double ThreeDTile::getDistanceToTile(osgUtil::CullVisitor* cv)
{
    return (double)cv->getDistanceToViewPoint(getBound().center(), true) - getBound().radius();
}

double ThreeDTile::computeScreenSpaceError(osgUtil::CullVisitor* cv)
{
    double distance = osg::maximum(getDistanceToTile(cv), 0.0000001);
    double fovy, ar, zn, zf;
    cv->getCurrentCamera()->getProjectionMatrix().getPerspective(fovy, ar, zn, zf);
    double height = cv->getCurrentCamera()->getViewport()->height();
    double sseDenominator = 2.0 * tan(0.5 * osg::DegreesToRadians(fovy));
    double error = (*_tile->geometricError() * height) / (distance * sseDenominator);
    return error;
}

void ThreeDTile::unloadContent()
{
    if (_content)
    {
        // Don't unload the content of tiles that were loaded immediately.
        if (_immediateLoad)
        {
            return;
        }
        _content->releaseGLObjects(0);
        _firstVisit = true;
        _content = 0;
        _requestedContent = false;
        _contentFuture = Future<osg::Node>();
        _contentUnloaded = true;
    }
}

void ThreeDTile::updateTracking()
{
    if (_content.valid())
    {
        _tileset->touchTile(this);
    }
}

void ThreeDTile::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.UPDATE_VISITOR)
    {
        if (_content.valid())
        {
            _content->accept(nv);
        }

        if (_children.valid())
        {
            _children->accept(nv);
        }
    }
    else if (nv.getVisitorType() == nv.CULL_VISITOR)
    {
        osgUtil::CullVisitor* cv = nv.asCullVisitor();

        // Get the ICO so we can do incremental compiliation
        osgUtil::IncrementalCompileOperation* ico = 0;
        osgViewer::ViewerBase* viewerBase = dynamic_cast<osgViewer::ViewerBase*>(cv->getCurrentCamera()->getView());
        if (viewerBase)
        {
            ico = viewerBase->getIncrementalCompileOperation();
        }

        // This allows nodes to reload themselves
        requestContent(ico);
        resolveContent();

        // Compute the SSE
        double error = computeScreenSpaceError(cv);

        updateTracking();

        bool areChildrenReady = true;
        if (_children.valid())
        {
#if 1
            for (unsigned int i = 0; i < _children->getNumChildren(); i++)
            {
                osg::ref_ptr< ThreeDTile > childTile = dynamic_cast<ThreeDTile*>(_children->getChild(i));
                if (childTile.valid())
                {
                    childTile->updateTracking();

                    // Can we traverse the child?
                    if (childTile->hasContent() && !childTile->isContentReady())
                    {
                        childTile->requestContent(ico);
                        areChildrenReady = false;
                    }
                }
            }
#endif
        }
        else
        {
            areChildrenReady = false;
        }


        if (areChildrenReady && error > _tileset->getMaximumScreenSpaceError() && _children.valid() && _children->getNumChildren() > 0)
        {
            if (_content.valid() && _tile->refine().isSetTo(TDTiles::REFINE_ADD))
            {
                _content->accept(nv);
            }

            if (_children.valid())
            {
                _children->accept(nv);
            }
        }
        else
        {
            if (_content.valid())
            {
                _content->accept(nv);
            }
        }
    }

    osg::MatrixTransform::traverse(nv);
}

ThreeDTileset::ThreeDTileset(TDTiles::Tileset* tileset, osgDB::Options* options) :
    _tileset(tileset),
    _options(options),
    _maximumScreenSpaceError(15.0f)
{
    addChild(new ThreeDTilesetContent(this, tileset, _options.get()));
}

float ThreeDTileset::getMaximumScreenSpaceError() const
{
    return _maximumScreenSpaceError;
}

void ThreeDTileset::setMaximumScreenSpaceError(float maximumScreenSpaceError)
{
    _maximumScreenSpaceError = maximumScreenSpaceError;
}

osg::BoundingSphere ThreeDTileset::computeBound() const
{
    return _tileset->root()->boundingVolume()->asBoundingSphere();
}

void ThreeDTileset::touchTile(osg::Node* node)
{
    NodeSet::iterator itr = _deadTiles.find(node);
    if (itr != _deadTiles.end())
    {
        _deadTiles.erase(itr);
    }
    _liveTiles.insert(node);
}

void ThreeDTileset::startCull()
{
}

void ThreeDTileset::endCull()
{
    // We can erase all of the tiles that are in the dead set
    for (NodeSet::iterator itr = _deadTiles.begin(); itr != _deadTiles.end(); ++itr)
    {
        osg::ref_ptr< ThreeDTile > tile = dynamic_cast<ThreeDTile*>(itr->get());
        if (tile.valid())
        {
            tile->unloadContent();
        }
    }
    _deadTiles.clear();

    _liveTiles.swap(_deadTiles);
    _liveTiles.clear();
}



void ThreeDTileset::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.CULL_VISITOR)
    {
        startCull();
        osg::Group::traverse(nv);
        endCull();
    }
    else
    {
        osg::Group::traverse(nv);
    }
}

ThreeDTilesetContent::ThreeDTilesetContent(ThreeDTileset* tilesetNode, TDTiles::Tileset* tileset, osgDB::Options* options) :
    _tilesetNode(tilesetNode),
    _tileset(tileset),
    _options(options)
{
    // Set up the root tile.
    if (tileset->root().valid())
    {
        addChild(new ThreeDTile(_tilesetNode, tileset->root().get(), true, _options.get()));
    }
}

osg::BoundingSphere ThreeDTilesetContent::computeBound() const
{
    return _tileset->root()->boundingVolume()->asBoundingSphere();
}
