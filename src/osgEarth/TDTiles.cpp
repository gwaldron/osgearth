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
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>
#include <osg/PagedLOD>
#include <osg/Version>

using namespace osgEarth;

#define LC "[3DTiles] "

//........................................................................

#define PSEUDOLOADER_TILE_CONTENT_EXT ".osgearth_3dtiles_tile"
#define PSEUDOLOADER_TILE_CHILDREN_EXT ".osgearth_3dtiles_children"
#define TAG_TILENODE "osgEarth::TDTiles::TileNode"

namespace osgEarth { namespace TDTiles
{
    struct GeometricErrorPagedLOD : public osg::PagedLOD
    {
    public:
        GeometricErrorPagedLOD(ContentHandler* handler) : osg::PagedLOD(), _handler(handler), _refine(REFINE_REPLACE) { }
        TDTiles::RefinePolicy _refine;
        osg::ref_ptr<ContentHandler> _handler;

    public:

        void traverse(osg::NodeVisitor& nv)
        {
            // set the frame number of the traversal so that external nodes can find out how active this
            // node is.
            if (nv.getFrameStamp() &&
                nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
            {
                setFrameNumberOfLastTraversal(nv.getFrameStamp()->getFrameNumber());
            }

            double timeStamp = nv.getFrameStamp() ? nv.getFrameStamp()->getReferenceTime() : 0.0;
            unsigned int frameNumber = nv.getFrameStamp() ? nv.getFrameStamp()->getFrameNumber() : 0;
            bool updateTimeStamp = nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR;

            switch (nv.getTraversalMode())
            {
            case(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN):
                std::for_each(_children.begin(), _children.end(), osg::NodeAcceptOp(nv));
                break;
            case(osg::NodeVisitor::TRAVERSE_ACTIVE_CHILDREN):
            {
#if OSG_VERSION_GREATER_OR_EQUAL(3,5,6)
                osg::CullStack* cullStack = nv.asCullStack();
#else
                osg::CullStack* cullStack = dynamic_cast<osg::CullStack*>(&nv);
#endif

                if (cullStack && cullStack->getLODScale() > 0)
                {
                    float sizeInMeters = getBound().radius() * 2.0;
                    float sizeInPixels = cullStack->clampedPixelSize(getBound()) / cullStack->getLODScale();

                    int lastChildTraversed = -1;
                    bool needToLoadChild = false;
                    for (unsigned int i = 0; i < _rangeList.size(); ++i)
                    {
                        //float minMetersPerPixel = _rangeList[i].first;
                        float maxMetersPerPixel = _rangeList[i].second;

                        // adjust sizeInPixels for the current SSE:
                        float effectiveSizeInPixels = sizeInPixels / _handler->getMaxScreenSpaceError();
                        float effectiveMetersPerPixel = effectiveSizeInPixels > 0.0? sizeInMeters / effectiveSizeInPixels : 0.0f;

                        OE_DEBUG << getName() << ": px=" << sizeInPixels << ", eMPP=" << effectiveMetersPerPixel << ", maxMPP=" << maxMetersPerPixel << ", SSE=" << _handler->getMaxScreenSpaceError() << std::endl;

                        if (effectiveMetersPerPixel < maxMetersPerPixel)
                        {
                            if (i < _children.size())
                            {
                                if (updateTimeStamp)
                                {
                                    _perRangeDataList[i]._timeStamp = timeStamp;
                                    _perRangeDataList[i]._frameNumber = frameNumber;
                                }

                                if (_refine == TDTiles::REFINE_ADD)
                                {
                                    _children[i]->accept(nv);
                                }

                                lastChildTraversed = (int)i;
                            }
                            else
                            {
                                needToLoadChild = true;
                            }
                        }
                    }

                    if (_refine == REFINE_REPLACE && lastChildTraversed >= 0)
                    {
                        _children[lastChildTraversed]->accept(nv);
                    }

                    if (needToLoadChild)
                    {
                        unsigned int numChildren = _children.size();

                        // select the last valid child.
                        if (numChildren > 0 && ((int)numChildren - 1) != lastChildTraversed)
                        {
                            if (updateTimeStamp)
                            {
                                _perRangeDataList[numChildren - 1]._timeStamp = timeStamp;
                                _perRangeDataList[numChildren - 1]._frameNumber = frameNumber;
                            }
                            _children[numChildren - 1]->accept(nv);
                        }

                        // now request the loading of the next unloaded child.
                        if (!_disableExternalChildrenPaging &&
                            nv.getDatabaseRequestHandler() &&
                            numChildren < _perRangeDataList.size())
                        {
                            // compute priority from where abouts in the required range the distance falls.
                            float priority = 0.5f;

                            // modify the priority according to the child's priority offset and scale.
                            priority = _perRangeDataList[numChildren]._priorityOffset + priority * _perRangeDataList[numChildren]._priorityScale;

                            if (_databasePath.empty())
                            {
                                nv.getDatabaseRequestHandler()->requestNodeFile(_perRangeDataList[numChildren]._filename, nv.getNodePath(), priority, nv.getFrameStamp(), _perRangeDataList[numChildren]._databaseRequest, _databaseOptions.get());
                            }
                            else
                            {
                                // prepend the databasePath to the child's filename.
                                nv.getDatabaseRequestHandler()->requestNodeFile(_databasePath + _perRangeDataList[numChildren]._filename, nv.getNodePath(), priority, nv.getFrameStamp(), _perRangeDataList[numChildren]._databaseRequest, _databaseOptions.get());
                            }
                        }
                    }
                }


                break;
            }
            default:
                break;
            }
        }
    };

    struct TileChildrenPseudoloader : public osgDB::ReaderWriter
    {
        ReadResult readNode(const std::string& location, const osgDB::Options* options) const
        {
            if (location != PSEUDOLOADER_TILE_CHILDREN_EXT)
                return ReadResult::FILE_NOT_HANDLED;

            osg::ref_ptr<TDTiles::TileNode> tileNode = OptionsData<TDTiles::TileNode>::get(options, TAG_TILENODE);
            if (!tileNode.valid())
                return ReadResult::FILE_NOT_FOUND;

            osg::ref_ptr<osg::Node> result = tileNode->loadChildren();
            return ReadResult(result.release());
        }
    };

    REGISTER_OSGPLUGIN(osgearth_3dtiles_children, TileChildrenPseudoloader);

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
        value["refine"] = (refine().get() == REFINE_ADD)? "add" : "replace";
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
                            const osgDB::Options* readOptions,
                            bool preloadContent) :
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

    GeometricErrorPagedLOD* plod = new GeometricErrorPagedLOD(_handler.get()); //osg::PagedLOD();
    if (bs.valid())
    {
        plod->setCenter(bs.center());
        plod->setRadius(bs.radius());
    }

    osg::ref_ptr<osgDB::Options> newOptions = Registry::instance()->cloneOrCreateOptions(readOptions);
    OptionsData<TDTiles::TileNode>::set(newOptions.get(), TAG_TILENODE, this);
    plod->setDatabaseOptions(newOptions.get());

    // Will begin loading content immediately when in view.
    osg::ref_ptr<osg::Node> content = readContent();
    if (content.valid())
    {
        plod->setName(_tile->content()->uri()->base());
        plod->addChild(content, 0.0f, FLT_MAX);
        plod->setNumChildrenThatCannotBeExpired(1u);
    }

    addChild(plod);

    if (tile->children().size() > 0)
    {
        // aka "maximum meters per pixel for which to use me"
        float geometricError = tile->geometricError().getOrUse(FLT_MAX);

        unsigned childrenIndex = plod->getNumChildren();

        // Load the children when the geometric error exceeds that of this tile.
        plod->setFileName(childrenIndex, PSEUDOLOADER_TILE_CHILDREN_EXT);
        plod->setRange(childrenIndex, 0.0f, geometricError);
        plod->_refine = tile->refine().get();
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
            TileNode* child = new TileNode(childTile, _handler.get(), _readOptions.get(), true);
            children->addChild(child);
        }
    }

    return children;
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

    // default action: just try to load a node using OSG
    if (tile->content().isSet() && tile->content()->uri().isSet())
    {
        OE_INFO << LC << "Loading content, URI = " << tile->content()->uri()->full() << std::endl;
        osgEarth::ReadResult rr = tile->content()->uri()->readNode(readOptions);
        if (rr.succeeded())
        {
            result = rr.releaseNode();
        }
        else
        {
            OE_WARN << LC << "Read error: " << rr.errorDetail() << std::endl;
        }
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
        TDTiles::GeometricErrorPagedLOD* plod = new TDTiles::GeometricErrorPagedLOD(_handler.get());
        addChild(plod);        
        float maxMetersPerPixel = _tileset->geometricError().getOrUse(FLT_MAX);
        plod->setName("Tileset");
        plod->addChild(tileNode, 0.0f, maxMetersPerPixel);
    }
}

void
TDTilesetGroup::setTilesetURL(const URI& location)
{
    //TODO
    OE_WARN << "TODO" << std::endl;
}
