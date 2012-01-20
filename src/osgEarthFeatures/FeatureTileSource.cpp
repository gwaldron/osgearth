/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
    //if ( conf.hasChild("features") )
    //    _featureOptions->merge( ConfigOptions(conf.child("features")) );

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

void 
FeatureTileSource::initialize(const osgDB::Options* dbOptions,
                              const Profile*        overrideProfile )
{
    if (overrideProfile)
    {
        //If we were given a profile, take it on.
        setProfile(overrideProfile);
    }
    else
    {
        //Assume it is global-geodetic
        setProfile( osgEarth::Registry::instance()->getGlobalGeodeticProfile() );
    }            

    if ( _features.valid() )
    {
        _features->initialize( dbOptions );

#if 0 // removed this as it was screwing up the rasterizer (agglite plugin).. not sure there's any reason to do this anyway
        if (_features->getFeatureProfile())
        {
            setProfile( Profile::create(_features->getFeatureProfile()->getSRS(),
                                    _features->getFeatureProfile()->getExtent().xMin(), _features->getFeatureProfile()->getExtent().yMin(),
                                    _features->getFeatureProfile()->getExtent().xMax(), _features->getFeatureProfile()->getExtent().yMax()));

        }
#endif
    }
    else
    {
        OE_WARN << LC << "No FeatureSource provided; nothing will be rendered (" << getName() << ")" << std::endl;
    }

    _initialized = true;
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
	osg::ref_ptr<osg::Image> image = new osg::Image();
	image->allocateImage( getPixelsPerTile(), getPixelsPerTile(), 1, GL_RGBA, GL_UNSIGNED_BYTE );

    preProcess( image.get(), buildData.get() );

    // figure out if and how to style the geometry.
    if ( _features->hasEmbeddedStyles() )
    {
        // Each feature has its own embedded style data, so use that:
        osg::ref_ptr<FeatureCursor> cursor = _features->createFeatureCursor( Query() );
        while( cursor.valid() && cursor->hasMore() )
        {
            Feature* feature = cursor->nextFeature();
            if ( feature )
            {
                FeatureList list;
                list.push_back( feature );
                renderFeaturesForStyle( 
                    *feature->style(), list, buildData.get(),
                    key.getExtent(), image.get() );
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
                const Style* style = styles->getStyle( sel.getSelectedStyleName() );
                queryAndRenderFeaturesForStyle( *style, sel.query().value(), buildData.get(), key.getExtent(), image.get() );
            }
        }
        else
        {
            const Style* style = styles->getDefaultStyle();
            queryAndRenderFeaturesForStyle( *style, Query(), buildData.get(), key.getExtent(), image.get() );
        }
    }
    else
    {
        queryAndRenderFeaturesForStyle( Style(), Query(), buildData.get(), key.getExtent(), image.get() );
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
    // first we need the overall extent of the layer:
    const GeoExtent& featuresExtent = getFeatureSource()->getFeatureProfile()->getExtent();
    
    // convert them both to WGS84, intersect the extents, and convert back.
    GeoExtent featuresExtentWGS84 = featuresExtent.transform( featuresExtent.getSRS()->getGeographicSRS() );
    GeoExtent imageExtentWGS84 = imageExtent.transform( featuresExtent.getSRS()->getGeographicSRS() );
    GeoExtent queryExtentWGS84 = featuresExtentWGS84.intersectionSameSRS( imageExtentWGS84.bounds() );
    if ( queryExtentWGS84.isValid() )
    {
        GeoExtent queryExtent = queryExtentWGS84.transform( featuresExtent.getSRS() );

	    // incorporate the image extent into the feature query for this style:
        Query localQuery = query;
        localQuery.bounds() = query.bounds().isSet()?
		    query.bounds()->unionWith( queryExtent.bounds() ) :
		    queryExtent.bounds();

        // query the feature source:
        osg::ref_ptr<FeatureCursor> cursor = _features->createFeatureCursor( localQuery );

        // now copy the resulting feature set into a list, converting the data
        // types along the way if a geometry override is in place:
        FeatureList cellFeatures;
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
                cellFeatures.push_back( feature );
            }
        }

        //OE_NOTICE
        //    << "Rendering "
        //    << cellFeatures.size()
        //    << " features in ("
        //    << queryExtent.toString() << ")"
        //    << std::endl;

	    return renderFeaturesForStyle( style, cellFeatures, data, imageExtent, out_image );
    }
    else
    {
        return false;
    }
}

