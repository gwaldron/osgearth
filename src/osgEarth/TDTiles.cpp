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
#include <osgEarth/Metrics>
#include <osgEarth/TDTiles>
#include <osgEarth/Utils>
#include <osgEarth/Registry>
#include <osgEarth/URI>
#include <osgEarth/NodeUtils>
#include <osgEarth/FileUtils>
#include <osgEarth/NetworkMonitor>
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>
#include <osgUtil/IncrementalCompileOperation>
#include <osg/ShapeDrawable>
#include <osg/PolygonMode>
#include <osgEarth/LineDrawable>

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
            osg::ref_ptr< ThreadPool > threadPool = ThreadPool::get(readOptions.get());
            if (!threadPool.valid())
            {
                unsigned int numThreads = 2;
                threadPool = new ThreadPool("osgEarth.3DTiles", numThreads);
                threadPool->put(readOptions.get());
            }

            osg::ref_ptr<ThreeDTilesetNode> node = new ThreeDTilesetNode(tileset, "", NULL, readOptions.get());
            node->setMaximumScreenSpaceError(15.0f);
            return node.release();
        }
    };
    REGISTER_OSGPLUGIN(3dtiles, ThreeDTilesJSONReaderWriter);
} } }

//........................................................................

void
Asset::fromJSON(const Json::Value& value)
{
    if (value.isMember("version"))
        version() = value.get("version", "").asString();
    if (value.isMember("tilesetVersion"))
        tilesetVersion() = value.get("tilesetVersion", "").asString();
    if (value.isMember("gltfUpAxis"))
        gltfUpAxis() = value.get("gltfUpAxis", "").asString();

}

Json::Value
Asset::getJSON() const
{
    Json::Value value(Json::objectValue);
    if (version().isSet())
        value["version"] = version().get();
    if (tilesetVersion().isSet())
        value["tilesetVersion"] = tilesetVersion().get();
    if (gltfUpAxis().isSet())
        value["gltfUpAxis"] = gltfUpAxis().get();
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
            double values[12];
            unsigned int index = 0;
            for (Json::ValueConstIterator j = a.begin(); j != a.end(); ++j)
            {
                values[index] = (*j).asDouble();
                index++;
            }
            Json::Value::const_iterator i = a.begin();
            osg::Vec3d center(values[0], values[1], values[2]);
            osg::Vec3d xvec(values[3], values[4], values[5]);
            osg::Vec3d yvec(values[6], values[7], values[8]);
            osg::Vec3d zvec(values[9], values[10], values[11]);

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

osg::BoundingSphere
Tile::getBoundingSphere()
{
    osg::BoundingSphere bsphere = boundingVolume()->asBoundingSphere();

    // If the bounding volume is a box or a sphere we need to adjust it by the Tile's transform.
    if (boundingVolume()->box().isSet() || boundingVolume()->sphere().isSet())
    {
        // Taken directly from osg::Transform.
        osg::Matrixd l2w;
        l2w.preMult(transform().get());

        if (!bsphere.valid()) return bsphere;

        osg::BoundingSphere::vec_type xdash = bsphere._center;
        xdash.x() += bsphere._radius;
        xdash = xdash * l2w;

        osg::BoundingSphere::vec_type ydash = bsphere._center;
        ydash.y() += bsphere._radius;
        ydash = ydash * l2w;

        osg::BoundingSphere::vec_type zdash = bsphere._center;
        zdash.z() += bsphere._radius;
        zdash = zdash * l2w;

        bsphere._center = bsphere._center * l2w;

        xdash -= bsphere._center;
        osg::BoundingSphere::value_type sqrlen_xdash = xdash.length2();

        ydash -= bsphere._center;
        osg::BoundingSphere::value_type sqrlen_ydash = ydash.length2();

        zdash -= bsphere._center;
        osg::BoundingSphere::value_type sqrlen_zdash = zdash.length2();

        bsphere._radius = sqrlen_xdash;
        if (bsphere._radius < sqrlen_ydash) bsphere._radius = sqrlen_ydash;
        if (bsphere._radius < sqrlen_zdash) bsphere._radius = sqrlen_zdash;
        bsphere._radius = (osg::BoundingSphere::value_type)sqrt(bsphere._radius);
    }
    return bsphere;
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

    return new Tileset(root, lc);
}

static VirtualProgram* getOrCreateDebugVirtualProgram()
{
    char s_debugColoring[] =
        "#version " GLSL_VERSION_STR "\n"
        "#pragma import_defines(OE_3DTILES_DEBUG)\n"
        "uniform vec4 debugColor;\n"
        "void color( inout vec4 color ) \n"
        "{\n"
        "#ifdef OE_3DTILES_DEBUG \n"
        "    color = mix(debugColor, color, 0.5); \n"
        "#endif\n"
        "} \n";

    static osg::ref_ptr< VirtualProgram > s_debugProgram;
    if (!s_debugProgram.valid())
    {
        s_debugProgram = new VirtualProgram();
        s_debugProgram->setFunction("color", s_debugColoring, ShaderComp::LOCATION_FRAGMENT_LIGHTING);
    }
    return s_debugProgram.get();
}

//........................................................................

osg::Vec4
randomColor()
{
    float r = (float)rand() / (float)RAND_MAX;
    float g = (float)rand() / (float)RAND_MAX;
    float b = (float)rand() / (float)RAND_MAX;
    return osg::Vec4(r, g, b, 1.0f);
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
            // Get the currently active request layer and reuse it when the operator actually occurs, which will probably be on a different thread.
            _requestLayer = NetworkMonitor::getRequestLayer();
        }

        void operator()(osg::Object*)
        {
            NetworkMonitor::ScopedRequestLayer layerRequest(_requestLayer);
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

                        if (tilesetNode.valid())
                        {
                            ImageUtils::compressAndMipmapTextures(tilesetNode.get());

                            if (ico.valid())
                            {
                                OE_PROFILING_ZONE_NAMED("ICO compile");

                                osg::Node* contentNode = static_cast<ThreeDTileNode*>(tilesetNode->getChild(0))->getContent();
                                if (contentNode)
                                {
                                    _compileSet = new osgUtil::IncrementalCompileOperation::CompileSet(contentNode);
                                    _compileSet->_compileCompletedCallback = this;
                                    ico->add(_compileSet.get());

                                    unsigned int numTries = 0;
                                    // block until the compile completes, checking once and a while for
                                    // an abandoned operation (to avoid deadlock)
                                    while (!_block.wait(10)) // 10ms
                                    {
                                        // Limit the number of tries and give up after awhile to avoid the case where the ICO still has work to do but the application has exited.
                                        ++numTries;
                                        if (_promise.isAbandoned() || numTries == 1000)
                                        {
                                            _compileSet->_compileCompletedCallback = NULL;
                                            ico->remove(_compileSet.get());
                                            _compileSet = 0;
                                            break;
                                        }
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
            // Clear the _compileSet to avoid keeping a circular reference to the content.
            _compileSet = 0;
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
        std::string _requestLayer;
    };

    Threading::Future<osg::Node> readTilesetAsync(ThreeDTilesetNode* parentTileset, const URI& uri, osgDB::Options* options)
    {
        osg::ref_ptr<ThreadPool> threadPool;
        if (options)
        {
            threadPool = ThreadPool::get(options);
        }

        Threading::Promise<osg::Node> promise;

        osg::ref_ptr< osg::Operation > operation = new LoadTilesetOperation(parentTileset, uri, options, promise);

        if (operation.valid())
        {
            if (threadPool.valid())
            {
                threadPool->run(operation.get());
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

ThreeDTileNode::ThreeDTileNode(ThreeDTilesetNode* tileset, Tile* tile, bool immediateLoad, osgDB::Options* options) :
    _tileset(tileset),
    _tile(tile),
    _requestedContent(false),
    _immediateLoad(immediateLoad),
    _firstVisit(true),
    _options(options),
    _trackerItrValid(false),
    _lastCulledFrameNumber(0),
    _lastCulledFrameTime(0.0f)
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

    if (tile->refine().isSet())
    {
        _refine = *tile->refine();
    }

    _localBoundingSphere = tile->boundingVolume()->asBoundingSphere();

    if (_immediateLoad && _tile->content().isSet())
    {
        URIContext context = _tile->content()->uri()->context();
        if (!_tileset->getAuthorizationHeader().empty())
        {
            context.addHeader("authorization", _tileset->getAuthorizationHeader());
        }
        URI uri(_tile->content()->uri()->base(), context);


        if (osgEarth::Strings::endsWith(_tile->content()->uri()->base(), ".json"))
        {
            _content = readTilesetAsync(_tileset, uri, options)
                .get();
        }
        else
        {
            _content = uri.getNode(_options.get());
            ImageUtils::compressAndMipmapTextures(_content.get());
        }
        if (_content.valid())
        {
            _tileset->runPreMergeOperations(_content.get());
            _tileset->runPostMergeOperations(_content.get());
        }
        OE_PROFILING_ZONE_TEXT("Immediate load");
    }

    if (_tile->children().size() > 0)
    {
        _children = new osg::Group;
        for (unsigned int i = 0; i < _tile->children().size(); ++i)
        {
            ThreeDTileNode* child = new ThreeDTileNode(_tileset, _tile->children()[i].get(), false, _options.get());
            child->setParentTile(this);
            _children->addChild(child);
        }

        if (_children->getNumChildren() == 0)
        {
            _children = 0;
        }
    }

    _debugColor = randomColor();

    getOrCreateStateSet()->getOrCreateUniform("debugColor", osg::Uniform::FLOAT_VEC4)->set(_debugColor);

    computeBoundingVolume();

    createDebugBounds();
}

void ThreeDTileNode::setParentTile(ThreeDTileNode* parentTile)
{
    _parentTile = parentTile;
    // Inherit the parent's refine policy if this Tile's refine policy isn't set.
    if (parentTile && !_tile->refine().isSet())
    {
        _refine = parentTile->getRefine();
    }
}

void ThreeDTileNode::computeBoundingVolume()
{
    if (_tile->boundingVolume()->region().isSet())
    {
        const SpatialReference* srs = SpatialReference::get("epsg:4326");

        GeoExtent extent(srs,
            osg::RadiansToDegrees(_tile->boundingVolume()->region()->xMin()),
            osg::RadiansToDegrees(_tile->boundingVolume()->region()->yMin()),
            osg::RadiansToDegrees(_tile->boundingVolume()->region()->xMax()),
            osg::RadiansToDegrees(_tile->boundingVolume()->region()->yMax()));

        // For really large bounding regions just use the bounding sphere to avoid transformation issues.
        if (extent.width() >= 5.0 || extent.height() >= 5.0)
        {
            return;
        }

        GeoPoint centroid;
        extent.getCentroid(centroid);

        osg::Matrixd worldToLocal, localToWorld;
        centroid.createWorldToLocal(worldToLocal);
        centroid.createLocalToWorld(localToWorld);

        osg::Vec3d world;
        osg::BoundingBoxd bb;

        GeoPoint(srs, extent.west(), extent.south(), _tile->boundingVolume()->region()->zMin()).toWorld(world);
        bb.expandBy(world * worldToLocal);
        GeoPoint(srs, extent.east(), extent.south(), _tile->boundingVolume()->region()->zMin()).toWorld(world);
        bb.expandBy(world * worldToLocal);
        GeoPoint(srs, extent.east(), extent.north(), _tile->boundingVolume()->region()->zMin()).toWorld(world);
        bb.expandBy(world * worldToLocal);
        GeoPoint(srs, extent.west(), extent.north(), _tile->boundingVolume()->region()->zMin()).toWorld(world);

        GeoPoint(srs, extent.west(), extent.south(), _tile->boundingVolume()->region()->zMax()).toWorld(world);
        bb.expandBy(world * worldToLocal);
        GeoPoint(srs, extent.east(), extent.south(), _tile->boundingVolume()->region()->zMax()).toWorld(world);
        bb.expandBy(world * worldToLocal);
        GeoPoint(srs, extent.east(), extent.north(), _tile->boundingVolume()->region()->zMax()).toWorld(world);
        bb.expandBy(world * worldToLocal);
        GeoPoint(srs, extent.west(), extent.north(), _tile->boundingVolume()->region()->zMax()).toWorld(world);

        // Bounding region culling and display are supposed to ignore the matrix transform of the tile, but b/c of the nested
        // nature of the ThreeDTileNode class the MatrixTransform is going to be applied already by the time
        // that the bounding box culling code and display occur.  We multiply the localToWorld of the bounding box by the
        // inverse transform matrix to undo any tranformation done by the tile transform when displaying or using the bounding volume for culling.
        _boundingBoxLocalToWorld = getInverseMatrix() * localToWorld;
        _boundingBox = bb;
    }
    else if (_tile->boundingVolume()->box().isSet())
    {
        _boundingBox = _tile->boundingVolume()->box().get();
    }
}

osg::ref_ptr<osg::Geode> buildSphere(const double radius,
    const unsigned int rings,
    const unsigned int sectors,
    const osg::Vec4& color)
{
    osg::ref_ptr<osg::Geode>      sphereGeode = new osg::Geode;
    osg::ref_ptr<osg::Geometry>   sphereGeometry = new osg::Geometry;
    osg::ref_ptr<osg::Vec3Array>  sphereVertices = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec3Array>  sphereNormals = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec2Array>  sphereTexCoords = new osg::Vec2Array;
    osg::ref_ptr< osg::Vec4Array > sphereColors = new osg::Vec4Array;

    float const R = 1. / static_cast<float>(rings - 1);
    float const S = 1. / static_cast<float>(sectors - 1);

    sphereGeode->addDrawable(sphereGeometry);

    // Establish texture coordinates, vertex list, and normals
    for (unsigned int r(0); r < rings; ++r) {
        for (unsigned int s(0); s < sectors; ++s) {
            float const y = sin(-osg::PI_2 + osg::PI * r * R);
            float const x = cos(2 * osg::PI * s * S) * sin(osg::PI * r * R);
            float const z = sin(2 * osg::PI * s * S) * sin(osg::PI * r * R);

            //sphereTexCoords->push_back(osg::Vec2(s * R, r * R));

            sphereVertices->push_back(osg::Vec3(x * radius,
                y * radius,
                z * radius))
                ;
            sphereNormals->push_back(osg::Vec3(x, y, z));

        }
    }

    sphereGeometry->setVertexArray(sphereVertices.get());
    //sphereGeometry->setTexCoordArray(0, sphereTexCoords);
    sphereColors->push_back(color);
    sphereGeometry->setColorArray(sphereColors.get(), osg::Array::BIND_OVERALL);

    osg::ref_ptr<osg::DrawElementsUInt> faces = new osg::DrawElementsUInt(osg::PrimitiveSet::LINE_LOOP);
    sphereGeometry->addPrimitiveSet(faces.get());

    // Generate quads for each face.
    for (unsigned int r(0); r < rings - 1; ++r) {
        for (unsigned int s(0); s < sectors - 1; ++s) {
            // Corners of quads should be in CCW order.
            faces->push_back((r + 0) * sectors + (s + 0)); // ll
            faces->push_back((r + 0) * sectors + (s + 1)); // lr
            faces->push_back((r + 1) * sectors + (s + 1)); // ur

            faces->push_back((r + 0) * sectors + (s + 0)); // ll
            faces->push_back((r + 1) * sectors + (s + 1)); // ur
            faces->push_back((r + 1) * sectors + (s + 0)); // ul
        }
    }


    return sphereGeode;
}

void ThreeDTileNode::createDebugBounds()
{
    if (_tile->boundingVolume()->sphere().isSet())
    {
        osg::BoundingSphere bs = _localBoundingSphere;
        osg::MatrixTransform* mt = new osg::MatrixTransform;
        mt->setMatrix(osg::Matrixd::translate(bs.center()));
        mt->addChild(buildSphere(bs.radius(), 20, 20, _debugColor));
        _boundsDebug = mt;
    }
    else if (_boundingBox.valid())
    {
        const int index[24] = {
                0, 1, 1, 2, 2, 3, 3, 0,
                4, 5, 5, 6, 6, 7, 7, 4,
                0, 4, 1, 5, 2, 6, 3, 7
        };

        std::vector< osg::Vec3 > corners;
        LineDrawable* d = new LineDrawable(GL_LINES);
        d->setUseGPU(false);
        corners.push_back(osg::Vec3(_boundingBox.xMin(), _boundingBox.yMin(), _boundingBox.zMin()));
        corners.push_back(osg::Vec3(_boundingBox.xMax(), _boundingBox.yMin(), _boundingBox.zMin()));
        corners.push_back(osg::Vec3(_boundingBox.xMax(), _boundingBox.yMax(), _boundingBox.zMin()));
        corners.push_back(osg::Vec3(_boundingBox.xMin(), _boundingBox.yMax(), _boundingBox.zMin()));
        corners.push_back(osg::Vec3(_boundingBox.xMin(), _boundingBox.yMin(), _boundingBox.zMax()));
        corners.push_back(osg::Vec3(_boundingBox.xMax(), _boundingBox.yMin(), _boundingBox.zMax()));
        corners.push_back(osg::Vec3(_boundingBox.xMax(), _boundingBox.yMax(), _boundingBox.zMax()));
        corners.push_back(osg::Vec3(_boundingBox.xMin(), _boundingBox.yMax(), _boundingBox.zMax()));

        for (int i = 0; i < 24; ++i)
            d->pushVertex(corners[index[i]]);

        d->setColor(_debugColor);
        d->finish();

        osg::MatrixTransform* transform = new osg::MatrixTransform;
        transform->setMatrix(_boundingBoxLocalToWorld);
        transform->addChild(d);
        _boundsDebug = transform;
    }
}


osg::BoundingSphere ThreeDTileNode::computeBound() const
{
    return _tile->getBoundingSphere();
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
        _content = _contentFuture.release();

        if (_content.valid())
        {
            // Assign the parent node if we just loaded a tileset
            ThreeDTilesetContentNode* tilesetContentNode = dynamic_cast<ThreeDTilesetContentNode*>(_content.get());
            if (tilesetContentNode)
            {
                ThreeDTileNode* tileNode = tilesetContentNode->getTileNode();
                if (tileNode)
                {
                    tileNode->setParentTile(this);
                }
            }

            _tileset->runPreMergeOperations(_content.get());
            _tileset->runPostMergeOperations(_content.get());
        }
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

        URIContext context = _tile->content()->uri()->context();
        if (!_tileset->getAuthorizationHeader().empty())
        {
            context.addHeader("authorization", _tileset->getAuthorizationHeader());
        }

        URI uri(_tile->content()->uri()->base(), context);

        NetworkMonitor::ScopedRequestLayer layerRequest(_tileset->getOwnerName());

        if (osgEarth::Strings::endsWith(_tile->content()->uri()->base(), ".json"))
        {
            _contentFuture =
                readTilesetAsync(_tileset, uri, localOptions.get());
        }
        else
        {
            _contentFuture = uri
                .readNodeAsync(localOptions.get(), NULL);
        }

        _requestedContent = true;
    }
}

double ThreeDTileNode::getDistanceToTile(osgUtil::CullVisitor* cv)
{
    osg::BoundingSphere bs = _localBoundingSphere;
    return (double)cv->getDistanceToViewPoint(bs.center(), true) - bs.radius();
}

double ThreeDTileNode::computeScreenSpaceError(osgUtil::CullVisitor* cv)
{
    double distance = osg::maximum(getDistanceToTile(cv), 0.0000001);
    const osg::Matrix& proj = cv->getCurrentCamera()->getProjectionMatrix();
    if (proj(3,3)==0.0) // perspective
    {
        double height = cv->getCurrentCamera()->getViewport()->height();
        return (*_tile->geometricError() * height) / (distance * _tileset->getSSEDenominator());
    }
    else // orthographic
    {
        const osg::Viewport* vp = cv->getCurrentCamera()->getViewport();
        double L, R, B, T, N, F;
        proj.getOrtho(L, R, B, T, N, F);
        double pixelSize = osg::maximum(T-B, R-L) / osg::maximum(vp->width(), vp->height());
        return (*_tile->geometricError()) / pixelSize;
    }
}

bool ThreeDTileNode::unloadContent()
{
    // Don't unload the content of tiles that were loaded immediately.  This shouldn't be called as they aren't tracked, but just in case.
    if (_immediateLoad)
    {
        return false;
    }

    if (_content.valid())
    {
        if (_content.valid())
        {
            _content->releaseGLObjects();
            _content = 0;
        }
    }

    _firstVisit = true;
    _content = 0;
    _requestedContent = false;
    _contentFuture = Future<osg::Node>();

    return true;
}

void ThreeDTileNode::resizeGLObjectBuffers(unsigned int maxSize)
{
    osg::MatrixTransform::resizeGLObjectBuffers(maxSize);

    if (_content.valid())
    {
        _content->resizeGLObjectBuffers(maxSize);
    }

    if (_children.valid())
    {
        _children->resizeGLObjectBuffers(maxSize);
    }
}

void ThreeDTileNode::releaseGLObjects(osg::State* state) const
{
    if (_content.valid())
    {
        _content->releaseGLObjects(state);
    }

    if (_children.valid())
    {
        _children->releaseGLObjects(state);
    }

    if (_boundsDebug.valid())
    {
        _boundsDebug->releaseGLObjects(state);
    }
}

unsigned int ThreeDTileNode::getLastCulledFrameNumber() const
{
    return _lastCulledFrameNumber;
}

float ThreeDTileNode::getLastCulledFrameTime() const
{
    return _lastCulledFrameTime;
}

void ThreeDTileNode::updateTracking(osgUtil::CullVisitor* cv)
{
    // Update tracking if this node wasn't immediately loaded.  Tiles that were immediately loaded are expected to be tracked by their parent.
    if (hasContent() && !_immediateLoad)
    {
        _tileset->touchTile(this);
        _lastCulledFrameNumber = cv->getFrameStamp()->getFrameNumber();
        _lastCulledFrameTime = cv->getFrameStamp()->getReferenceTime();
    }
}

void ThreeDTileNode::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.CULL_VISITOR)
    {
        osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(&nv);

        if (_boundingBox.valid())
        {
            osg::ref_ptr< osg::RefMatrix > refMatrix = new osg::RefMatrix(*cv->getModelViewMatrix());
            refMatrix->preMult(_boundingBoxLocalToWorld);
            cv->pushModelViewMatrix(refMatrix.get(), getReferenceFrame());
            bool culled = cv->isCulled(_boundingBox);
            cv->popModelViewMatrix();

            if (culled)
            {
                return;
            }
        }

        // Get the ICO so we can do incremental compiliation
        osgUtil::IncrementalCompileOperation* ico = 0;
        osgViewer::View* osgView = dynamic_cast<osgViewer::View*>(cv->getCurrentCamera()->getView());
        if (osgView)
        {
            ico = osgView->getDatabasePager()->getIncrementalCompileOperation();
        }

        // This allows nodes to reload themselves
        requestContent(ico);
        resolveContent();

        // Compute the SSE
        double error = computeScreenSpaceError(cv);

        updateTracking(cv);

        bool areChildrenReady = true;
        if (_children.valid())
        {
            for (unsigned int i = 0; i < _children->getNumChildren(); i++)
            {
                osg::ref_ptr< ThreeDTileNode > childTile = dynamic_cast<ThreeDTileNode*>(_children->getChild(i));
                if (childTile.valid())
                {
                    childTile->updateTracking(cv);

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
            if (_content.valid() && _refine == REFINE_ADD)
            {
                _content->accept(nv);
            }

            if (_tileset->getShowBoundingVolumes() && _boundsDebug.valid())
            {
                _boundsDebug->accept(nv);
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

            if (_tileset->getShowBoundingVolumes() && _boundsDebug.valid())
            {
                _boundsDebug->accept(nv);
            }
        }
    }
    else if (nv.getTraversalMode() == osg::NodeVisitor::TRAVERSE_ACTIVE_CHILDREN)
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
            if (_content.valid() && _refine == REFINE_ADD)
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
    else if (nv.getTraversalMode() == osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
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

}

ThreeDTilesetNode::ThreeDTilesetNode(Tileset* tileset, const std::string& authorizationHeader, SceneGraphCallbacks* sceneGraphCallbacks, osgDB::Options* options) :
    _tileset(tileset),
    _options(options),
    _maximumScreenSpaceError(15.0f),
    _maxTiles(50),
    _showBoundingVolumes(false),
    _showColorPerTile(false),
    _maxAge(5.0f),
    _lastExpiredFrame(0),
    _authorizationHeader(authorizationHeader),
    _sgCallbacks(sceneGraphCallbacks),
	_sseDenominator(1.0)
{
    ADJUST_UPDATE_TRAV_COUNT(this, +1);
    const char* c = ::getenv("OSGEARTH_3DTILES_CACHE_SIZE");
    if (c)
    {
        setMaxTiles((unsigned)atoi(c));
    }

    c = ::getenv("OSGEARTH_3DTILES_MAX_AGE");
    if (c)
    {
        setMaxAge((float)atof(c));
    }

    _tracker.push_back(0);
    // Pointer to last element
    _sentryItr = --_tracker.end();

    _debugVP = getOrCreateDebugVirtualProgram();
    getOrCreateStateSet()->setAttribute(_debugVP.get());
    if (_showColorPerTile)
    {
        getOrCreateStateSet()->setDefine("OE_3DTILES_DEBUG", osg::StateAttribute::ON);
    }

    // If the gltfUpAxis property is set to z we don't need to do the y up to z up transformation
    // so we set an option string telling the gltf loader to not apply the transformation for this tileset.
    if (tileset->asset().isSet() && osgEarth::toLower(*tileset->asset()->gltfUpAxis()) == "z")
    {
        if (!_options.valid())
        {
            _options = new osgDB::Options;
        }
        std::string optString = _options->getOptionString();
        optString += " gltfZUp";
        _options->setOptionString(optString);
    }

    addChild(new ThreeDTilesetContentNode(this, tileset, _options.get()));
}

const std::string&
ThreeDTilesetNode::getOwnerName() const
{
    return _ownerName;
}

void
ThreeDTilesetNode::setOwnerName(const std::string& value)
{
    _ownerName = value;
}


unsigned int ThreeDTilesetNode::getMaxTiles() const
{
    return _maxTiles;
}

void ThreeDTilesetNode::setMaxTiles(unsigned int maxTiles)
{
    _maxTiles = maxTiles;
}

float ThreeDTilesetNode::getMaxAge() const
{
    return _maxAge;
}

void ThreeDTilesetNode::setMaxAge(float maxAge)
{
    _maxAge = maxAge;
}

float ThreeDTilesetNode::getMaximumScreenSpaceError() const
{
    return _maximumScreenSpaceError;
}

void ThreeDTilesetNode::setMaximumScreenSpaceError(float maximumScreenSpaceError)
{
    _maximumScreenSpaceError = maximumScreenSpaceError;
}

bool ThreeDTilesetNode::getShowBoundingVolumes() const
{
    return _showBoundingVolumes;
}

void ThreeDTilesetNode::setShowBoundingVolumes(bool showBoundingVolumes)
{
    _showBoundingVolumes = showBoundingVolumes;
}

bool ThreeDTilesetNode::getColorPerTile() const
{
    return _showColorPerTile;
}

void ThreeDTilesetNode::setColorPerTile(bool colorPerTile)
{
    if (_showColorPerTile != colorPerTile)
    {
        _showColorPerTile = colorPerTile;
        if (_showColorPerTile)
        {
            getOrCreateStateSet()->setDefine("OE_3DTILES_DEBUG", osg::StateAttribute::ON);
        }
        else
        {
            getOrCreateStateSet()->setDefine("OE_3DTILES_DEBUG", osg::StateAttribute::OFF);
        }
    }
}

double ThreeDTilesetNode::getSSEDenominator() const
{
	return _sseDenominator;
}

SceneGraphCallbacks* ThreeDTilesetNode::getSceneGraphCallbacks(SceneGraphCallbacks* callbacks)
{
    return _sgCallbacks.get();
}

void ThreeDTilesetNode::setSceneGraphCallbacks(SceneGraphCallbacks* callbacks)
{
    _sgCallbacks = callbacks;
}

void
ThreeDTilesetNode::runPreMergeOperations(osg::Node* node)
{
    if (_sgCallbacks.valid())
    {
        _sgCallbacks->firePreMergeNode(node);
    }
}

void
ThreeDTilesetNode::runPostMergeOperations(osg::Node* node)
{
    if (_sgCallbacks.valid())
    {
        _sgCallbacks->firePostMergeNode(node);
    }
}

void ThreeDTilesetNode::touchTile(ThreeDTileNode* node)
{
    ScopedMutexLock lock(_mutex);
    if (node->_trackerItrValid)
    {
        _tracker.erase(node->_trackerItr);
    }

    _tracker.push_back(node);
    node->_trackerItrValid = true;
    node->_trackerItr = --_tracker.end();
}

void ThreeDTilesetNode::expireTiles(const osg::NodeVisitor& nv)
{
    OE_PROFILING_ZONE;

    unsigned int frameNumber = nv.getFrameStamp()->getFrameNumber();
    float frameTime = nv.getFrameStamp()->getReferenceTime();

    osg::Timer_t startTime = osg::Timer::instance()->tick();
    osg::Timer_t endTime;

    ScopedMutexLock lock(_mutex);

    // Max time in ms to allocate to erasing tiles
    float maxTime = 2.0f;

    ThreeDTileNode::TileTracker::iterator itr = _tracker.begin();

    unsigned int numErased = 0;
    unsigned int numSkipped = 0;
    while (_tracker.size() > _maxTiles && itr != _sentryItr)
    {
        osg::ref_ptr< ThreeDTileNode > tile = dynamic_cast<ThreeDTileNode*>(itr->get());
        if (tile.valid())
        {
            float age = frameTime - tile->getLastCulledFrameTime();
            bool canUnload = age >= _maxAge;
            if (canUnload && tile->unloadContent())
            {
                tile->_trackerItrValid = false;
                itr = _tracker.erase(itr);
                ++numErased;
            }
            else
            {
                numSkipped++;
                ++itr;
            }
        }

        endTime = osg::Timer::instance()->tick();
        if (osg::Timer::instance()->delta_m(startTime, endTime) > maxTime)
        {
            break;
        }
    }

#if 0
    if (numErased > 0 || numSkipped > 0)
    {
        OE_NOTICE << "Erased " << numErased << " and skipped " << numSkipped << " in " << osg::Timer::instance()->delta_m(startTime, endTime) << "ms" << std::endl;
    }
    OE_NOTICE << "Tiles in memory " << _tracker.size() << " max tiles=" << _maxTiles << std::endl;
#endif

    // Erase the sentry and stick it at the end of the list
    _tracker.erase(_sentryItr);
    _tracker.push_back(0);
    _sentryItr = --_tracker.end();
}

void ThreeDTilesetNode::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.UPDATE_VISITOR)
    {
        // Check to make sure expireTiles is only called once per frame, even if the UpdateVisitor is sent down multiple times.
        // This can happen if the node has multiple parents.
        if (nv.getFrameStamp()->getFrameNumber() > _lastExpiredFrame)
        {
            expireTiles(nv);
            _lastExpiredFrame = nv.getFrameStamp()->getFrameNumber();
        }
    }
	else if (nv.getVisitorType() == nv.CULL_VISITOR)
	{
		osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(&nv);
		const osg::Matrix& proj = cv->getCurrentCamera()->getProjectionMatrix();
		double height = cv->getCurrentCamera()->getViewport()->height();
		double fovy, ar, zn, zf;
		proj.getPerspective(fovy, ar, zn, zf);
		_sseDenominator = 2.0 * tan(0.5 * osg::DegreesToRadians(fovy));
	}

    osg::Group::traverse(nv);
}

ThreeDTilesetContentNode::ThreeDTilesetContentNode(ThreeDTilesetNode* tilesetNode, Tileset* tileset, osgDB::Options* options) :
    _tilesetNode(tilesetNode),
    _tileset(tileset),
    _options(options),
    _tileNode(0)
{
    // Set up the root tile.
    if (tileset->root().valid())
    {
        _tileNode = new ThreeDTileNode(_tilesetNode, tileset->root().get(), true, _options.get());
        addChild(_tileNode);
    }
}

ThreeDTileNode* ThreeDTilesetContentNode::getTileNode()
{
    return _tileNode;
}