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
#include <osgEarthFeatures/FeatureTileSource>
#include <osgEarth/Registry>
#include <osgDB/WriteFile>
#include <osg/Notify>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

#define LC "[FeatureTileSource] "

/*************************************************************************/

FeatureTileSourceOptions::FeatureTileSourceOptions( const ConfigOptions& options ) :
TileSourceOptions( options ),
_geomTypeOverride( Geometry::TYPE_UNKNOWN )
{
    fromConfig( _conf );
}

Config
FeatureTileSourceOptions::getConfig() const
{
    Config conf = TileSourceOptions::getConfig();

    conf.updateObjIfSet( "features", _featureOptions );
    conf.updateObjIfSet( "styles", _styles );

    if ( _geomTypeOverride.isSet() ) {
        if ( _geomTypeOverride == Geometry::TYPE_LINESTRING )
            conf.update( "geometry_type", "line" );
        else if ( _geomTypeOverride == Geometry::TYPE_POINTSET )
            conf.update( "geometry_type", "point" );
        else if ( _geomTypeOverride == Geometry::TYPE_POLYGON )
            conf.update( "geometry_type", "polygon" );
    }

    return conf;
}

void
FeatureTileSourceOptions::mergeConfig( const Config& conf )
{
    TileSourceOptions::mergeConfig( conf );
    fromConfig( conf );
}

void
FeatureTileSourceOptions::fromConfig( const Config& conf )
{
    conf.getObjIfSet( "features", _featureOptions );

    conf.getObjIfSet( "styles", _styles );
    
    std::string gt = conf.value( "geometry_type" );
    if ( gt == "line" || gt == "lines" || gt == "linestring" )
        _geomTypeOverride = Geometry::TYPE_LINESTRING;
    else if ( gt == "point" || gt == "pointset" || gt == "points" )
        _geomTypeOverride = Geometry::TYPE_POINTSET;
    else if ( gt == "polygon" || gt == "polygons" )
        _geomTypeOverride = Geometry::TYPE_POLYGON;
}

/*************************************************************************/

FeatureTileSource::FeatureTileSource( const TileSourceOptions& options ) :
TileSource  ( options ),
_options    ( options.getConfig() ),
_initialized( false )
{
    if ( _options.featureSource().valid() )
    {
        _features = _options.featureSource().get();
    }

    else if ( _options.featureOptions().isSet() )
    {
        _features = FeatureSourceFactory::create( _options.featureOptions().value() );
        if ( !_features.valid() )
        {
            OE_WARN << LC << "Failed to create FeatureSource from options" << std::endl;
        }
    }
}

Status 
FeatureTileSource::initialize(const osgDB::Options* readOptions)
{
    if ( !getProfile() )
    {
        setProfile( osgEarth::Registry::instance()->getGlobalGeodeticProfile() );
    }            

    if ( !_features.valid() )
        return Status::Error(Status::ServiceUnavailable, "No feature source");

    // attempt to open the feature source:
    const Status& sourceStatus = _features->open(readOptions);
    if (sourceStatus.isError())
        return sourceStatus;

    // Try to fill the DataExtent list using the FeatureProfile
    const FeatureProfile* featureProfile = _features->getFeatureProfile();
    if (featureProfile != NULL)
    {
        if (featureProfile->getProfile() != NULL)
        {
            // Use specified profile's GeoExtent
            getDataExtents().push_back(DataExtent(featureProfile->getProfile()->getExtent()));
        }
        else if (featureProfile->getExtent().isValid() == true)
        {
            // Use FeatureProfile's GeoExtent
            getDataExtents().push_back(DataExtent(featureProfile->getExtent()));
        }
    }

    // Create a session for feature processing. No map.
    _session = new Session( 0L, _options.styles().get(), _features.get(), readOptions );

    _initialized = true;

    return Status::OK();
}

void
FeatureTileSource::setFeatureSource( FeatureSource* source )
{
    if ( !_initialized )
    {
        _features = source;
    }
    else
    {
        OE_WARN << LC << "Illegal: cannot set FeatureSource after intitialization ( " << getName() << ")" << std::endl;
    }
}

osg::Image*
FeatureTileSource::createImage( const TileKey& key, ProgressCallback* progress )
{
    if ( !_features.valid() || !_features->getFeatureProfile() )
        return 0L;

    // style data
    const StyleSheet* styles = _options.styles();

    // implementation-specific data
    osg::ref_ptr<osg::Referenced> buildData = createBuildData();

    // allocate the image.
    osg::ref_ptr<osg::Image> image = allocateImage();
    if ( !image.valid() )
    {
        image = new osg::Image();
        image->allocateImage( getPixelsPerTile(), getPixelsPerTile(), 1, GL_RGBA, GL_UNSIGNED_BYTE );
    }

    preProcess( image.get(), buildData.get() );

    Query defaultQuery;
    defaultQuery.tileKey() = key;

    // figure out if and how to style the geometry.
    if ( _features->hasEmbeddedStyles() )
    {

        // Each feature has its own embedded style data, so use that:
        osg::ref_ptr<FeatureCursor> cursor = _features->createFeatureCursor(defaultQuery);
        while( cursor.valid() && cursor->hasMore() )
        {
            osg::ref_ptr< Feature > feature = cursor->nextFeature();
            if ( feature )
            {
                FeatureList list;
                list.push_back( feature );

                renderFeaturesForStyle(
                    _session.get(),
                    *feature->style(),
                    list,
                    buildData.get(),
                    key.getExtent(),
                    image.get() );
            }
        }
    }
    else if ( styles )
    {
        if ( styles->selectors().size() > 0 )
        {
            for( StyleSelectorList::const_iterator i = styles->selectors().begin(); i != styles->selectors().end(); ++i )
            {
                const StyleSelector& sel = *i;

                if ( sel.styleExpression().isSet() )
                {
                    const FeatureProfile* featureProfile = _features->getFeatureProfile();

                    // establish the working bounds and a context:
                    FilterContext context( _session.get(), featureProfile);
                    StringExpression styleExprCopy(  sel.styleExpression().get() );

                    FeatureList features;
                    getFeatures(defaultQuery, key.getExtent(), features);
                    if (!features.empty())
                    {
                        for (FeatureList::iterator itr = features.begin(); itr != features.end(); ++itr)
                        {
                            Feature* feature = itr->get();

                            const std::string& styleString = feature->eval( styleExprCopy, &context );
                            if (!styleString.empty() && styleString != "null")
                            {
                                // resolve the style:
                                Style combinedStyle;

                                // if the style string begins with an open bracket, it's an inline style definition.
                                if ( styleString.length() > 0 && styleString.at(0) == '{' )
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
                                    const Style* selectedStyle = _session->styles()->getStyle(styleString, false);
                                    if ( selectedStyle )
                                        combinedStyle = *selectedStyle;
                                }

                                if (!combinedStyle.empty())
                                {
                                    FeatureList list;
                                    list.push_back( feature );

                                    renderFeaturesForStyle(
                                        _session.get(),
                                        combinedStyle,
                                        list,
                                        buildData.get(),
                                        key.getExtent(),
                                        image.get() );
                                }
                            }
                        }
                    }                    
                }
                else
                {
                    const Style* style = styles->getStyle( sel.getSelectedStyleName() );
                    Query query = sel.query().get();
                    query.tileKey() = key;
                    queryAndRenderFeaturesForStyle( *style, query, buildData.get(), key.getExtent(), image.get() );
                }
            }
        }
        else
        {
            const Style* style = styles->getDefaultStyle();
            queryAndRenderFeaturesForStyle( *style, defaultQuery, buildData.get(), key.getExtent(), image.get() );
        }
    }
    else
    {
        queryAndRenderFeaturesForStyle( Style(), defaultQuery, buildData.get(), key.getExtent(), image.get() );
    }

    // final tile processing after all styles are done
    postProcess( image.get(), buildData.get() );

    return image.release();
}


bool
FeatureTileSource::queryAndRenderFeaturesForStyle(const Style&     style,
                                                  const Query&     query,
                                                  osg::Referenced* data,
                                                  const GeoExtent& imageExtent,
                                                  osg::Image*      out_image)
{   
    // Get the features
    FeatureList features;
    getFeatures(query, imageExtent, features );
    if (!features.empty())
    {
        // Render them.
        return renderFeaturesForStyle( _session.get(), style, features, data, imageExtent, out_image );
    }
    return false;
}

void
FeatureTileSource::getFeatures(const Query& query, const GeoExtent& imageExtent, FeatureList& features)
{
    // first we need the overall extent of the layer:
    const GeoExtent& featuresExtent = getFeatureSource()->getFeatureProfile()->getExtent();
    
    // convert them both to WGS84, intersect the extents, and convert back.
    GeoExtent featuresExtentWGS84 = featuresExtent.transform( featuresExtent.getSRS()->getGeographicSRS() );
    GeoExtent imageExtentWGS84 = imageExtent.transform( featuresExtent.getSRS()->getGeographicSRS() );
    GeoExtent queryExtentWGS84 = featuresExtentWGS84.intersectionSameSRS( imageExtentWGS84 );
    if ( queryExtentWGS84.isValid() )
    {
        GeoExtent queryExtent = queryExtentWGS84.transform( featuresExtent.getSRS() );

        // incorporate the image extent into the feature query for this style:
        Query localQuery = query;
        localQuery.bounds() = 
            query.bounds().isSet() ? query.bounds()->unionWith( queryExtent.bounds() ) :
            queryExtent.bounds();

        // now copy the resulting feature set into a list, converting the data
        // types along the way if a geometry override is in place:
        while (features.empty())
        {
            // query the feature source:
            osg::ref_ptr<FeatureCursor> cursor = _features->createFeatureCursor( localQuery );

            while( cursor.valid() && cursor->hasMore() )
            {
                Feature* feature = cursor->nextFeature();
                Geometry* geom = feature->getGeometry();
                if ( geom )
                {
                    // apply a type override if requested:
                    if (_options.geometryTypeOverride().isSet() &&
                        _options.geometryTypeOverride() != geom->getComponentType() )
                    {
                        geom = geom->cloneAs( _options.geometryTypeOverride().value() );
                        if ( geom )
                            feature->setGeometry( geom );
                    }
                }
                if ( geom )
                {
                    features.push_back( feature );
                }
            }

            // If we didn't get any features and we have a tilekey set, try falling back.
            if (features.empty() && localQuery.tileKey().isSet())
            {
                localQuery.tileKey() = localQuery.tileKey().get().createParentKey();
                if (!localQuery.tileKey()->valid())
                {
                    // We fell back all the way to lod 0 and got nothing, so bail.
                    break;
                }
            }
            else
            {
                // Just bail, we didn't get any features and aren't using tilekeys
                break;
            }
        }
    }
}

