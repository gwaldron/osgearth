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
#include "RoadSurfaceLayer"
#include <osgEarth/Utils>
#include <osgEarth/Map>
#include <osgEarth/TileRasterizer>
#include <osgEarth/VirtualProgram>
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

    // Generate Mercator tiles by default.
    setProfile(Profile::create("global-geodetic"));
    //setProfile(Profile::create("spherical-mercator"));

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
    _session = new Session(map, options().styles().get(), 0L, getReadOptions());
    _session->setResourceCache(new ResourceCache());

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

typedef std::vector< std::pair< Style, FeatureList > > StyleToFeatures;

void addFeatureToMap(Feature* feature, const Style& style, StyleToFeatures& map)
{
    bool added = false;

    if (!style.getName().empty())
    {
        // Try to find the style by name
        for (int i = 0; i < map.size(); i++)
        {
            if (map[i].first.getName() == style.getName())
            {
                map[i].second.push_back(feature);
                added = true;
                break;
            }
        }
    }

    if (!added)
    {
        FeatureList list;
        list.push_back( feature );
        map.push_back(std::pair< Style, FeatureList>(style, list));
    }                                
}

void sortFeaturesIntoStyleGroups(StyleSheet* styles, FeatureList& features, FilterContext &context, StyleToFeatures& map)
{
    if ( styles == 0L )
        return;

    if ( styles->selectors().size() > 0 )
    {
        for( StyleSelectorList::const_iterator i = styles->selectors().begin(); i != styles->selectors().end(); ++i )
        {
            const StyleSelector& sel = *i;

            if ( sel.styleExpression().isSet() )
            {
                // establish the working bounds and a context:
                StringExpression styleExprCopy(  sel.styleExpression().get() );

                for (FeatureList::iterator itr = features.begin(); itr != features.end(); ++itr)
                {
                    Feature* feature = itr->get();

                    const std::string& styleString = feature->eval( styleExprCopy, &context );
                    if (!styleString.empty() && styleString != "null")
                    {
                        // resolve the style:
                        Style combinedStyle;

                        // if the style string begins with an open bracket, it's an inline style definition.
                        if ( styleString.length() > 0 && styleString[0] == '{' )
                        {
                            Config conf( "style", styleString );
                            conf.setReferrer( sel.styleExpression().get().uriContext().referrer() );
                            conf.set( "type", "text/css" );
                            combinedStyle = Style(conf);
                        }

                        // otherwise, look up the style in the stylesheet. Do NOT fall back on a default
                        // style in this case: for style expressions, the user must be explicity about 
                        // default styling; this is because there is no other way to exclude unwanted
                        // features.
                        else
                        {
                            const Style* selectedStyle = styles->getStyle(styleString, false);
                            if ( selectedStyle )
                                combinedStyle = *selectedStyle;
                        }

                        if (!combinedStyle.empty())
                        {
                            addFeatureToMap( feature, combinedStyle, map);
                        }                                
                    }
                }
            }
        }
    }
    else
    {
        const Style* style = styles->getDefaultStyle();
        for (FeatureList::iterator itr = features.begin(); itr != features.end(); ++itr)
        {
            Feature* feature = itr->get();
            addFeatureToMap( feature, *style, map);
        }        
    }
}

GeoImage
RoadSurfaceLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress)
{
    if (getStatus().isError())    
    {
        return GeoImage::INVALID;
    }
    
    if (!_features.valid())
    {
        setStatus(Status(Status::ServiceUnavailable, "No feature source"));
        return GeoImage::INVALID;
    }

    const FeatureProfile* featureProfile = _features->getFeatureProfile();
    if (!featureProfile)
    {
        setStatus(Status(Status::ConfigurationError, "Feature profile is missing"));
        return GeoImage::INVALID;
    }

    const SpatialReference* featureSRS = featureProfile->getSRS();
    if (!featureSRS)
    {
        setStatus(Status(Status::ConfigurationError, "Feature profile has no SRS"));
        return GeoImage::INVALID;
    }

    // If the feature source has a tiling profile, we are going to have to map the incoming
    // TileKey to a set of intersecting TileKeys in the feature source's tiling profile.
    GeoExtent featureExtent = key.getExtent().transform(featureSRS);
    GeoExtent queryExtent = featureExtent;

    if (!featureExtent.isValid())
        return GeoImage::INVALID;

    // Buffer the incoming extent, if requested.
    if (options().featureBufferWidth().isSet())
    {
        GeoExtent geoExtent = queryExtent.transform(featureSRS->getGeographicSRS());
        double latitude = geoExtent.getCentroid().y();
        double buffer = SpatialReference::transformUnits(options().featureBufferWidth().get(), featureSRS, latitude);
        queryExtent.expand(buffer, buffer);
    }
    

    FeatureList features;

    if (featureProfile->getProfile())
    {
        // Resolve the list of tile keys that intersect the incoming extent.
        std::vector<TileKey> intersectingKeys;
        featureProfile->getProfile()->getIntersectingTiles(queryExtent, key.getLOD(), intersectingKeys);

        std::set<TileKey> featureKeys;
        for (int i = 0; i < intersectingKeys.size(); ++i)
        {        
            if (intersectingKeys[i].getLOD() > featureProfile->getMaxLevel())
                featureKeys.insert(intersectingKeys[i].createAncestorKey(featureProfile->getMaxLevel()));
            else
                featureKeys.insert(intersectingKeys[i]);
        }

        // Query and collect all the features we need for this tile.
        for (std::set<TileKey>::const_iterator i = featureKeys.begin(); i != featureKeys.end(); ++i)
        {
            Query query;        
            query.tileKey() = *i;

            osg::ref_ptr<FeatureCursor> cursor = _features->createFeatureCursor(query, progress);
            if (cursor.valid())
            {
                cursor->fill(features);
            }
        }
    }
    else
    {
        // Set up the query; bounds must be in the feature SRS:
        Query query;
        query.bounds() = queryExtent.bounds();

        // Run the query and fill the list.
        osg::ref_ptr<FeatureCursor> cursor = _features->createFeatureCursor(query, progress);
        if (cursor.valid())
        {
            cursor->fill(features);
        }
    }

    if (!features.empty())
    {
        // Create the output extent:
        GeoExtent outputExtent = key.getExtent();

        const SpatialReference* keySRS = outputExtent.getSRS();
        osg::Vec3d pos(outputExtent.west(), outputExtent.south(), 0);
        osg::ref_ptr<const SpatialReference> srs = keySRS->createTangentPlaneSRS(pos);
        outputExtent = outputExtent.transform(srs.get());

        FilterContext fc(_session.get(), featureProfile, featureExtent);
        fc.setOutputSRS(outputExtent.getSRS());

        // compile the features into a node.
        GeometryCompiler compiler;

        StyleToFeatures map;
        sortFeaturesIntoStyleGroups(options().styles().get(), features, fc, map);
        osg::ref_ptr< osg::Group > group;
        if (!map.empty())
        {
            group = new osg::Group;
            for (unsigned int i = 0; i < map.size(); i++)
            {
                osg::ref_ptr<osg::Node> node = compiler.compile(map[i].second, map[i].first, fc);
                if (node.valid() && node->getBound().valid())
                {
                    group->addChild( node );
                }
            }
        }

        if (group && group->getBound().valid())
        {
            Threading::Future<osg::Image> imageFuture;

            // Schedule the rasterization and get the future.
            imageFuture = _rasterizer->push(group.release(), getTileSize(), outputExtent);

            // Block until the image is ready.
            // NULL means there was nothing to render.
            osg::Image* image = imageFuture.release();
            if (image)
            {
                return GeoImage(image, key.getExtent());
            }
        }
    }

    return GeoImage::INVALID;
}
