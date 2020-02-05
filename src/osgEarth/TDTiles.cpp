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
#include <osgEArth/Metrics>
#include <osgEarth/Registry>
#include <osgEarth/URI>
#include <osgEarth/OGRFeatureSource>
#include <osgEarth/FeatureCursor>
#include <osgEarth/ResampleFilter>
#include <osgEarth/FeatureNode>
#include <osgEarth/StyleSheet>
#include <osgEarth/LineDrawable>
#include <osgEarth/LabelNode>
#include <osgEarth/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/WriteFile>
#include <osg/CoordinateSystemNode>
#include <osgUtil/IncrementalCompileOperation>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Contrib::ThreeDTiles;

#define LC "[3DTiles] "

#define SENTRY_VALUE NULL

//........................................................................

namespace osgEarth { namespace Contrib { namespace ThreeDTiles
{
    class ThreeDTilesJSONReaderWriter : public osgDB::ReaderWriter
    {
    public:
        ThreeDTilesJSONReaderWriter()
        {
            supportsExtension("3dtiles", "3D-Tiles JSON TileSet");
        }

        virtual const char* className() const { return "3D-Tiles JSON TileSet"; }

        virtual ReadResult readObject(const std::string& location, const osgDB::Options* options) const
        {
            return readNode(location, options);
        }

        virtual ReadResult readNode(const std::string& location, const osgDB::Options* options) const
        {
            std::string ext = osgDB::getFileExtension(location);
            if (!acceptsExtension(ext))
                return ReadResult::FILE_NOT_HANDLED;

            std::string uristring = osgDB::getNameLessExtension(location);

            osgEarth::ReadResult rr = URI(uristring).readString(options);
            if (rr.failed())
                return ReadResult(rr.errorDetail());

            Tileset* tileset = Tileset::create(rr.getString(), uristring);
            if (!tileset)
                return ReadResult("Unable to parse tileset");

            // Clone the read options and if there isn't a ThreadPool create one.
            osg::ref_ptr< osgDB::Options > readOptions = osgEarth::Registry::instance()->cloneOrCreateOptions(options);
            osg::ref_ptr< ThreadPool > threadPool = OptionsData<ThreadPool>::get(readOptions.get(), "threadpool");
            if (!threadPool.valid())
            {
                unsigned int numThreads = 8;
                threadPool = new ThreadPool(numThreads);
                OptionsData<ThreadPool>::set(readOptions.get(), "threadpool", threadPool.get());
            }

            osg::ref_ptr<ThreeDTilesetNode> node = new ThreeDTilesetNode(tileset, readOptions.get());
            node->setMaximumScreenSpaceError(15.0f);
            return node.release();
        }
    };
    REGISTER_OSGPLUGIN(3dtiles, ThreeDTilesJSONReaderWriter)
}}}

//........................................................................

void
Asset::fromJSON(const Json::Value& value)
{
    if (value.isMember("version"))
        version() = value.get("version", "").asString();
    if (value.isMember("tilesetVersion"))
        tilesetVersion() = value.get("tilesetVersion", "").asString();
}

Json::Value
Asset::getJSON() const
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
BoundingVolume::fromJSON(const Json::Value& value)
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
BoundingVolume::getJSON() const
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
BoundingVolume::asBoundingSphere() const
{
    // Note: this should be epsg:4979 according to the 3D-Tiles spec,
    // but that only exists in very new versions of PROJ. For the
    // purposes of osgEarth there's no difference anyway
    const SpatialReference* srs = SpatialReference::get("epsg:4326");
    if (!srs)
        return osg::BoundingSphere();

    if (region().isSet())
    {
        GeoExtent extent(srs,
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
TileContent::fromJSON(const Json::Value& value, LoadContext& lc)
{
    if (value.isMember("boundingVolume"))
        boundingVolume() = BoundingVolume(value.get("boundingVolume", Json::nullValue));
    if (value.isMember("uri"))
        uri() = URI(value.get("uri", "").asString(), lc._uc);
    if (value.isMember("url"))
        uri() = URI(value.get("url", "").asString(), lc._uc);
}

Json::Value
TileContent::getJSON() const
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
Tile::fromJSON(const Json::Value& value, LoadContext& uc)
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
Tile::getJSON() const
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
Tileset::fromJSON(const Json::Value& value, LoadContext& uc)
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
Tileset::getJSON() const
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

Tileset*
Tileset::create(const std::string& json, const URIContext& uc)
{
    Json::Reader reader;
    Json::Value root(Json::objectValue);
    if (!reader.parse(json, root, false))
        return NULL;

    LoadContext lc;
    lc._uc = uc;
    lc._defaultRefine = REFINE_REPLACE;

    return new Tileset(root, lc);
}

//........................................................................

ThreeDTileNode::ThreeDTileNode(ThreeDTilesetNode* tileset, Tile* tile, bool immediateLoad, osgDB::Options* options) :
    _tileset(tileset),
    _tile(tile),
    _requestedContent(false),
    _immediateLoad(immediateLoad),
    _firstVisit(true),
    _contentUnloaded(false),
    _options(options),
    _trackerItrValid(false)
{
    OE_PROFILING_ZONE;
    if (_tile->content().isSet())
    {
        OE_PROFILING_ZONE_TEXT(_tile->content()->uri()->full().c_str());
    }

    // If this tile has content, store a URI Context to that relative-path external file
    // references (textures) will resolve correctly.
    if (_tile->content().isSet())
    {
        _options = Registry::instance()->cloneOrCreateOptions(options);
        URIContext(_tile->content()->uri()->full()).store(_options.get());
    }

    // the transform to localize this tile:
    if (tile->transform().isSet())
    {
        setMatrix(tile->transform().get());
    }

    if (_immediateLoad && _tile->content().isSet())
    {
        _content = _tile->content()->uri()->getNode(_options.get());
        OE_PROFILING_ZONE_TEXT("Immediate load");
    }

    if (_tile->children().size() > 0)
    {
        _children = new osg::Group;
        for (unsigned int i = 0; i < _tile->children().size(); ++i)
        {
            _children->addChild(new ThreeDTileNode(_tileset, _tile->children()[i].get(), false, _options.get()));
        }

        if (_children->getNumChildren() == 0)
        {
            _children = 0;
        }
    }
}

osg::BoundingSphere ThreeDTileNode::computeBound() const
{
    if (_tile->boundingVolume().isSet())
    {
        return _tile->boundingVolume()->asBoundingSphere();
    }
    else
    {
        return osg::Group::computeBound();
    }
}

bool ThreeDTileNode::hasContent()
{
    return _tile->content().isSet() && _tile->content()->uri().isSet();
}

osg::Node* ThreeDTileNode::getContent()
{
    return _content.get();
}

bool ThreeDTileNode::isContentReady()
{
    resolveContent();
    return _content.valid();
}

void ThreeDTileNode::resolveContent()
{
    // Resolve the future 
    if (!_content.valid() && _requestedContent && _contentFuture.isAvailable())
    {
        _content = _contentFuture.get();
    }
}


namespace
{
    class LoadTilesetOperation : public osg::Operation, public osgUtil::IncrementalCompileOperation::CompileCompletedCallback
    {
    public:
        LoadTilesetOperation(ThreeDTilesetNode* parentTileset, const URI& uri, osgDB::Options* options, osgEarth::Threading::Promise<osg::Node> promise) :
            _uri(uri),
            _promise(promise),
            _options(options),
            _parentTileset(parentTileset)
        {
        }

        void operator()(osg::Object*)
        {
            if (!_promise.isAbandoned())
            {   
                osg::ref_ptr<osgUtil::IncrementalCompileOperation> ico = OptionsData<osgUtil::IncrementalCompileOperation>::get(_options.get(), "osg::ico");
                osg::ref_ptr<ThreeDTilesetContentNode> tilesetNode;
                osg::ref_ptr<ThreeDTilesetNode> parentTileset;
                _parentTileset.lock(parentTileset);
                if (parentTileset.valid())
                {
                    // load the tile set:
                    ReadResult rr = _uri.readString(_options.get());

                    if (rr.failed())
                    {
                        OE_WARN << "Fail to read tileset \"" << _uri.full() << ": " << rr.errorDetail() << std::endl;
                    }

                    //std::string fullPath = osgEarth::getAbsolutePath(_url);

                    osg::ref_ptr<Tileset> tileset = Tileset::create(rr.getString(), _uri.full());
                    if (tileset)
                    {
                        tilesetNode = new ThreeDTilesetContentNode(parentTileset.get(), tileset.get(), _options.get());

                        if (tilesetNode.valid() && ico.valid())
                        {
                            OE_PROFILING_ZONE_NAMED("ICO compile");

                            osg::Node* contentNode = static_cast<ThreeDTileNode*>(tilesetNode->getChild(0))->getContent();
                            if (contentNode)
                            {
                                _compileSet = new osgUtil::IncrementalCompileOperation::CompileSet(contentNode);
                                _compileSet->_compileCompletedCallback = this;
                                ico->add(_compileSet.get());

                                // block until the compile completes, checking once and a while for
                                // an abandoned operation (to avoid deadlock)
                                while (!_block.wait(10)) // 10ms
                                {
                                    if (_promise.isAbandoned())
                                    {
                                        _compileSet->_compileCompletedCallback = NULL;
                                        ico->remove(_compileSet.get());
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
                _promise.resolve(tilesetNode.get());
            }
        }

        bool compileCompleted(osgUtil::IncrementalCompileOperation::CompileSet* compileSet)
        {
            // release the wait.
            _block.set();
            return true;
        }

        osgEarth::Threading::Promise<osg::Node> _promise;
        osg::ref_ptr< osgDB::Options > _options;
        osg::observer_ptr<ThreeDTilesetNode> _parentTileset;
        osg::ref_ptr<osgUtil::IncrementalCompileOperation::CompileSet> _compileSet;
        Threading::Event _block;
        URI _uri;
    };

    Threading::Future<osg::Node> readTilesetAsync(ThreeDTilesetNode* parentTileset, const URI& uri, osgDB::Options* options)
    {
        osg::ref_ptr<ThreadPool> threadPool;
        if (options)
        {
            threadPool = OptionsData<ThreadPool>::get(options, "threadpool");
        }

        Threading::Promise<osg::Node> promise;

        osg::ref_ptr< osg::Operation > operation = new LoadTilesetOperation(parentTileset, uri, options, promise);

        if (operation.valid())
        {
            if (threadPool.valid())
            {
                threadPool->getQueue()->add(operation.get());
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


void ThreeDTileNode::requestContent(osgUtil::IncrementalCompileOperation* ico)
{
    if (!_content.valid() && !_requestedContent && hasContent())
    {
        // if there's an ICO, install it:
        osg::ref_ptr<osgDB::Options> localOptions;
        if (ico)
        {
            localOptions = Registry::instance()->cloneOrCreateOptions(_options.get());
            OptionsData<osgUtil::IncrementalCompileOperation>::set(localOptions.get(), "osg::ico", ico);
        }
        else
        {
            localOptions = _options.get();
        }

        if (osgEarth::Strings::endsWith(_tile->content()->uri()->base(), ".json"))
        {
            _contentFuture = readTilesetAsync(_tileset, _tile->content()->uri().get(), localOptions.get());
        }
        else
        {
            _contentFuture = _tile->content()->uri()->readNodeAsync(localOptions.get(), NULL);
        }
        _requestedContent = true;
    }
}

double ThreeDTileNode::getDistanceToTile(osgUtil::CullVisitor* cv)
{
    return (double)cv->getDistanceToViewPoint(getBound().center(), true) - getBound().radius();
}

double ThreeDTileNode::computeScreenSpaceError(osgUtil::CullVisitor* cv)
{
    double distance = osg::maximum(getDistanceToTile(cv), 0.0000001);
    double fovy, ar, zn, zf;
    cv->getCurrentCamera()->getProjectionMatrix().getPerspective(fovy, ar, zn, zf);
    double height = cv->getCurrentCamera()->getViewport()->height();
    double sseDenominator = 2.0 * tan(0.5 * osg::DegreesToRadians(fovy));
    double error = (*_tile->geometricError() * height) / (distance * sseDenominator);
    return error;
}

void ThreeDTileNode::unloadContent()
{
    if (_content)
    {
        // Don't unload the content of tiles that were loaded immediately.
        if (_immediateLoad)
        {
            return;
        }
        //_content->releaseGLObjects(0);
        _firstVisit = true;
        _content = 0;
        _requestedContent = false;
        _contentFuture = Future<osg::Node>();
        _contentUnloaded = true;
    }
}

void ThreeDTileNode::updateTracking()
{
    if (_content.valid())
    {
        _tileset->touchTile(this);
    }
}

void ThreeDTileNode::traverse(osg::NodeVisitor& nv)
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
#if OSG_VERSION_LESS_THAN(3,6,0)
        osgUtil::CullVisitor *cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);
#else
        osgUtil::CullVisitor* cv = nv.asCullVisitor();
#endif

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
            for (unsigned int i = 0; i < _children->getNumChildren(); i++)
            {
                osg::ref_ptr< ThreeDTileNode > childTile = dynamic_cast<ThreeDTileNode*>(_children->getChild(i));
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
        }
        else
        {
            areChildrenReady = false;
        }

        if (areChildrenReady && error > _tileset->getMaximumScreenSpaceError() && _children.valid() && _children->getNumChildren() > 0)
        {
            if (_content.valid() && _tile->refine().isSetTo(REFINE_ADD))
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
    else if (nv.getVisitorType() == nv.INTERSECTION_VISITOR)
    {
        resolveContent();
        bool areChildrenReady = true;
        if (_children.valid())
        {
            for (unsigned int i = 0; i < _children->getNumChildren(); i++)
            {
                osg::ref_ptr< ThreeDTileNode > childTile = dynamic_cast<ThreeDTileNode*>(_children->getChild(i));
                if (childTile.valid())
                {
                    // Can we traverse the child?
                    if (childTile->hasContent() && !childTile->isContentReady())
                    {
                        areChildrenReady = false;
                    }
                }
            }
        }
        else
        {
            areChildrenReady = false;
        }

        if (areChildrenReady && _children.valid() && _children->getNumChildren() > 0)
        {
            if (_content.valid() && _tile->refine().isSetTo(REFINE_ADD))
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
}

ThreeDTilesetNode::ThreeDTilesetNode(Tileset* tileset, osgDB::Options* options) :
    _tileset(tileset),
    _options(options),
    _maximumScreenSpaceError(15.0f),
    _maxTiles(50)
{
    const char* c = ::getenv("OSGEARTH_3DTILES_CACHE_SIZE");
    if (c)
    {        
        setMaxTiles((unsigned)atoi(c));
    }

    _tracker.push_back(0);
    // Pointer to last element
    _sentryItr = --_tracker.end();

    addChild(new ThreeDTilesetContentNode(this, tileset, _options.get()));
}

unsigned int ThreeDTilesetNode::getMaxTiles() const
{
    return _maxTiles;
}

void ThreeDTilesetNode::setMaxTiles(unsigned int maxTiles)
{
    _maxTiles = maxTiles;
}

float ThreeDTilesetNode::getMaximumScreenSpaceError() const
{
    return _maximumScreenSpaceError;
}

void ThreeDTilesetNode::setMaximumScreenSpaceError(float maximumScreenSpaceError)
{
    _maximumScreenSpaceError = maximumScreenSpaceError;
}

osg::BoundingSphere ThreeDTilesetNode::computeBound() const
{
    return _tileset->root()->boundingVolume()->asBoundingSphere();    
}

void ThreeDTilesetNode::touchTile(osg::Node* node)
{
    ScopedMutexLock lock(_mutex);

    osg::ref_ptr< ThreeDTileNode > t = dynamic_cast<ThreeDTileNode*>(node);
    if (t.valid())
    {
        if (t->_trackerItrValid)
        {
            _tracker.erase(t->_trackerItr);
        }

        _tracker.push_back(t.get());
        t->_trackerItrValid = true;    
        t->_trackerItr = --_tracker.end();
    }
}

void ThreeDTilesetNode::startCull()
{
}

void ThreeDTilesetNode::endCull()
{
    OE_PROFILING_ZONE;

    osg::Timer_t startTime = osg::Timer::instance()->tick();
    osg::Timer_t endTime;

    ScopedMutexLock lock(_mutex);
    
    // Max time in ms to allocate to erasing tiles
    float maxTime = 1.0f;

    ThreeDTileNode::TileTracker::iterator itr = _tracker.begin();

    unsigned int numErased = 0;
    while (_tracker.size() > _maxTiles && itr != _sentryItr)
    {
        osg::ref_ptr< ThreeDTileNode > tile = dynamic_cast<ThreeDTileNode*>(itr->get());
        if (tile.valid())
        {
            tile->unloadContent();
            tile->_trackerItrValid = false;
            itr = _tracker.erase(itr);
            ++numErased;
        }        

        endTime = osg::Timer::instance()->tick();
        if (osg::Timer::instance()->delta_m(startTime, endTime) > maxTime)
        {
            break;
        }
    }

    // Erase the sentry and stick it at the end of the list
    _tracker.erase(_sentryItr);
    _tracker.push_back(0);    
    _sentryItr = --_tracker.end();    

    endTime = osg::Timer::instance()->tick();
}

void ThreeDTilesetNode::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.CULL_VISITOR)
    {
        startCull();
        osg::MatrixTransform::traverse(nv);
        endCull();
    }
    else
    {
        osg::MatrixTransform::traverse(nv);
    }
}

ThreeDTilesetContentNode::ThreeDTilesetContentNode(ThreeDTilesetNode* tilesetNode, Tileset* tileset, osgDB::Options* options) :
    _tilesetNode(tilesetNode),
    _tileset(tileset),
    _options(options)
{
    // Set up the root tile.
    if (tileset->root().valid())
    {
        addChild(new ThreeDTileNode(_tilesetNode, tileset->root().get(), true, _options.get()));
    }
}

osg::BoundingSphere ThreeDTilesetContentNode::computeBound() const
{
    return _tileset->root()->boundingVolume()->asBoundingSphere();
}
