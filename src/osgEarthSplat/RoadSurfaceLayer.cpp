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


REGISTER_OSGEARTH_LAYER(road_surface, RoadSurfaceLayer);

RoadSurfaceLayer::RoadSurfaceLayer() :
ImageLayer(&_optionsConcrete),
_options(&_optionsConcrete)
{
    init();
}

RoadSurfaceLayer::RoadSurfaceLayer(const RoadSurfaceLayerOptions& options) :
ImageLayer(&_optionsConcrete),
_options(&_optionsConcrete),
_optionsConcrete(options)
{
    init();
}

void
RoadSurfaceLayer::init()
{
    setTileSourceExpected(false);

    setProfile(Profile::create("global-geodetic"));

    // Create a rasterizer for rendering nodes to images.
    _rasterizer = new TileRasterizer(); 

    ImageLayer::init();

    if (getName().empty())
        setName("Road surface");
}

const Status&
RoadSurfaceLayer::open()
{
    // assert a feature source:
    if (!options().features().isSet() && !options().featureSourceLayer().isSet())
    {
        return setStatus(Status::Error(Status::ConfigurationError, "Missing required feature source"));
    }

    if (options().features().isSet())
    {
        // create and attempt to open that feature source:
        osg::ref_ptr<FeatureSource> features = FeatureSourceFactory::create(options().features().get());
        if (features.valid())
        {
            setStatus(features->open());

            if (getStatus().isOK())
            {
                setFeatureSource(features.get());
            }
        }
        else
        {
            return setStatus(Status::Error(Status::ServiceUnavailable, "Cannot load feature source"));
        }
    }


    if (getStatus().isOK())
        return ImageLayer::open();
    else
        return getStatus();
}

void
RoadSurfaceLayer::addedToMap(const Map* map)
{
    // create a session for feature processing based in the Map,
    // but don't set the feature source yet.
    _session = new Session(map, new StyleSheet(), 0L, getReadOptions());
    _session->setResourceCache(new ResourceCache());
    
    if (options().style().isSet())
    {
        _session->styles()->addStyle(options().style().get());
    }
    else
    {
        OE_WARN << LC << "No style available\n";
        setStatus(Status::Error(Status::ConfigurationError, "No styles provided"));
    }

    if (options().featureSourceLayer().isSet())
    {
        _layerListener.listen(
            map,
            options().featureSourceLayer().get(),
            this,
            &RoadSurfaceLayer::setFeatureSourceLayer);
    }
    else if (!_features.valid())
    {
        setStatus(Status::Error(Status::ConfigurationError, "No features"));
    }
}

void
RoadSurfaceLayer::removedFromMap(const Map* map)
{
    _session = 0L;
}

osg::Node*
RoadSurfaceLayer::getNode() const
{
    // adds the Rasterizer to the scene graph so we can rasterize tiles
    return _rasterizer.get();
}

void
RoadSurfaceLayer::setFeatureSource(FeatureSource* fs)
{
    if (fs != _features.get())
    {
        _features = fs;
        if (_features.valid())
        {
            setStatus(_features->getStatus());
        }
    }
}

void
RoadSurfaceLayer::setFeatureSourceLayer(FeatureSourceLayer* layer)
{
    if (layer && layer->getStatus().isError())
    {
        setStatus(Status::Error(Status::ResourceUnavailable, "Feature source layer is unavailable; check for error"));
        return;
    }

    if (layer)
        OE_INFO << LC << "Feature source layer is \"" << layer->getName() << "\"\n";

    setFeatureSource(layer ? layer->getFeatureSource() : 0L);
}

GeoImage
RoadSurfaceLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress)
{
    if (getStatus().isError() || !_features.valid())
        return GeoImage::INVALID;

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
        else
            fkeys.insert(featureKeys[i]);
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
        // Schedule the rasterization and get the future:
        Threading::Future<osg::Image> image = _rasterizer->push(node.release(), getTileSize(), outputExtent);
        // Wait for rasterization to complete and return the image.
        return GeoImage(image.get(), key.getExtent());
    }
    else
    {
        return GeoImage::INVALID;
    }
}
