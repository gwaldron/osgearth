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
ImageLayer(ImageLayerOptions())
{
    //nop
}

RoadSurfaceLayer::RoadSurfaceLayer(const ConfigOptions& options) :
ImageLayer(options),
RoadSurfaceLayerOptions(options)
{
    //nop
}

const Status&
RoadSurfaceLayer::open()
{
    if (!featureSourceOptions().isSet())
        return setStatus(Status::Error(Status::ConfigurationError, "Missing required feature source"));

    _features = FeatureSourceFactory::create(featureSourceOptions().get());
    if (!_features)
        return setStatus(Status::Error(Status::ServiceUnavailable, "Cannot load feature source"));

    osg::ref_ptr<const Map> map;
    OptionsData<const Map>::lock(getReadOptions(), "osgEarth.Map", map);
    if (!map.valid())
        return setStatus(Status::Error(Status::AssertionFailure, "const Map not found in OptionsData<>"));

    // create a session for feature processing
    _session = new Session(map.get(), new StyleSheet(), _features.get(), getReadOptions());
    _session->setResourceCache(new ResourceCache());

    if (style().isSet())
        _session->styles()->addStyle(style().get());
    else
        OE_WARN << LC << "No style available\n";

    setStatus(_features->open());

    if (getStatus().isOK())
        ImageLayer::open();
    
    //OE_INFO << LC << "Status = " << getStatus().message() << "\n";
    return getStatus();
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
    // all points > innerRadius and <= outerRadius and smoothstep blended
    double outerRadius = outerWidth().get() * 0.5;

    // adjust those values based on latitude in a geographic map
    if (key.getExtent().getSRS()->isGeographic())
    {
        double latMid = fabs(key.getExtent().yMin() + key.getExtent().height()*0.5);
        double metersPerDegAtEquator = (key.getExtent().getSRS()->getEllipsoid()->getRadiusEquator() * 2.0 * osg::PI) / 360.0;
        double metersPerDegree = metersPerDegAtEquator * cos(osg::DegreesToRadians(latMid));
        //innerRadius = innerRadius / metersPerDegree;
        outerRadius = outerRadius / metersPerDegree;
    }

    // Expand the query bounds to encompass any feature within outerRadius of the tile:
    Bounds queryBounds = key.getExtent().bounds();
    queryBounds.expandBy(queryBounds.xMin() - outerRadius, queryBounds.yMin() - outerRadius, 0);
    queryBounds.expandBy(queryBounds.xMax() + outerRadius, queryBounds.yMax() + outerRadius, 0);
    queryBounds.transform(key.getExtent().getSRS(), _features->getFeatureProfile()->getSRS());

    // Use that to query the feature source:
    Query query;
    query.bounds() = queryBounds;
    osg::ref_ptr<FeatureCursor> cursor = _features->createFeatureCursor(query);
    if (!cursor.valid() || !cursor->hasMore())
        return 0L;

    GeoExtent outputExtent = key.getExtent();
    FilterContext fc(_session.get(), _features->getFeatureProfile(), outputExtent);
    if (key.getExtent().getSRS()->isGeographic())
    {
        fc.setOutputSRS(SpatialReference::get("spherical-mercator"));
        outputExtent = outputExtent.transform(fc.getOutputSRS());
    }

    GeometryCompilerOptions options;
    options.shaderPolicy() = SHADERPOLICY_DISABLE;

    GeometryCompiler compiler(options);
    osg::ref_ptr<osg::Node> node = compiler.compile(cursor, style().get(), fc);

    //if (node)
    //    osgDB::writeNodeFile(*node, "out.osgb");

    if (node && node->getBound().valid())
    {
        return new GeoNode(node.release(), outputExtent);
    }
    else
    {
        return 0L;
    }
}
