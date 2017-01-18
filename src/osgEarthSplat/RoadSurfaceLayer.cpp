/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include "RoadSurfaceLayer"
#include <osgEarth/Utils>
#include <osgEarth/Map>
#include <osgEarth/TileRasterizer>
#include <osgEarth/ShaderUtils>
#include <osgEarthFeatures/FilterContext>
#include <osgEarthFeatures/GeometryCompiler>
#include <osgDB/WriteFile>

using namespace osgEarth;
using namespace osgEarth::Splat;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

#define LC "[RoadSurfaceLayer] "



RoadSurfaceLayer::RoadSurfaceLayer(const RoadSurfaceLayerOptions& options) :
ImageLayer(&_localOptionsConcrete),
_localOptions(&_localOptionsConcrete),
_localOptionsConcrete(options)
{
    ImageLayer::init();

    if (getName().empty())
        setName("Road surface");
}

const Status&
RoadSurfaceLayer::open()
{
    // assert a feature source:
    if (!options().featureSourceOptions().isSet())
        return setStatus(Status::Error(Status::ConfigurationError, "Missing required feature source"));

    // create and attempt to open that feature source:
    _features = FeatureSourceFactory::create(options().featureSourceOptions().get());
    if (!_features)
        return setStatus(Status::Error(Status::ServiceUnavailable, "Cannot load feature source"));

    setStatus(_features->open());

    if (getStatus().isOK())
        return ImageLayer::open();
    else
        return getStatus();
}

void
RoadSurfaceLayer::addedToMap(const Map* map)
{   
    if (_features.valid())
    {
        // create a session for feature processing based in the Map:
        _session = new Session(map, new StyleSheet(), _features.get(), getReadOptions());
        _session->setResourceCache(new ResourceCache());

        if (options().style().isSet())
            _session->styles()->addStyle(options().style().get());
        else
            OE_WARN << LC << "No style available\n";
    }
    else
    {
        OE_WARN << LC << "Added to map before opening features\n";
    }
}

void
RoadSurfaceLayer::removedFromMap(const Map* map)
{
    _session = 0L;
}

osg::Texture*
RoadSurfaceLayer::createTexture(const TileKey& key, ProgressCallback* progress)
{
    //OE_INFO << LC << key.str() << std::endl;

    osg::Texture2D* tex = 0L;
    
    osg::ref_ptr<GeoNode> node = createNode(key, progress);
    if (node.valid())
    {
        tex = new osg::Texture2D();
        tex->setTextureSize(256, 256);
        tex->setInternalFormat(GL_RGBA);
        tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
        tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR); //LINEAR_MIPMAP_LINEAR);
        tex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
        tex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
        tex->setResizeNonPowerOfTwoHint(false);
        tex->setMaxAnisotropy(4.0f);
        tex->setUnRefImageDataAfterApply(true);
        tex->setName(key.str());
        tex->setUserData(node.get());
    }
    return tex;
}

GeoNode*
RoadSurfaceLayer::createNode(const TileKey& key, ProgressCallback* progress)
{        
    // Resolve the list of tile keys that intersect the incoming extent:
    std::vector<TileKey> featureKeys;
    if (_features->getFeatureProfile() && _features->getFeatureProfile()->getProfile())
    {
        _features->getFeatureProfile()->getProfile()->getIntersectingTiles(key, featureKeys);    
    }
    else
    {
        featureKeys.push_back(key);
    }

    const FeatureProfile* featureProfile = _features->getFeatureProfile();
    const SpatialReference* featureSRS = featureProfile->getSRS();

    FeatureList features;

    std::set<TileKey> fkeys;

    for (int i = 0; i < featureKeys.size(); ++i)
    {        
        if (featureKeys[i].getLOD() > featureProfile->getMaxLevel())
            fkeys.insert(featureKeys[i].createAncestorKey(featureProfile->getMaxLevel()));
    }

    for (std::set<TileKey>::const_iterator i = fkeys.begin(); i != fkeys.end(); ++i)
    {
        Query query;
        query.tileKey() = *i;

        osg::ref_ptr<FeatureCursor> cursor = _features->createFeatureCursor(query);
        if (cursor.valid())
        {
            cursor->fill(features);
        }
    }

    // Create the output extent in feature SRS:
    GeoExtent outputExtent = key.getExtent();
    FilterContext fc(_session.get(), _features->getFeatureProfile(), outputExtent);
    
    // By default, the geometry compiler will use the Session's Map SRS at the output SRS
    // for feature data. We want a projected output so we can take an overhead picture of it.
    // So set the output SRS to mercator instead.
    if (key.getExtent().getSRS()->isGeographic())
    {
        fc.setOutputSRS(SpatialReference::get("spherical-mercator"));
        outputExtent = outputExtent.transform(fc.getOutputSRS());
    }

    // turn off the shader generation:
    GeometryCompilerOptions geomOptions;
    geomOptions.shaderPolicy() = SHADERPOLICY_DISABLE;

    // compile the features into a node.
    GeometryCompiler compiler(geomOptions);
    osg::ref_ptr<osg::Node> node = compiler.compile(features, options().style().get(), fc);
    
    if (node && node->getBound().valid())
    {
        // return that new node.
        return new GeoNode(node.release(), outputExtent);
    }
    else
    {
        return 0L;
    }
}
