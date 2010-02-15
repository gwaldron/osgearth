/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osgEarthFeatures/Styling>
#include <osgEarth/Registry>
#include <osg/Notify>

using namespace osgEarth;
using namespace osgEarth::Features;

#define PROP_FEATURES      "features"
#define PROP_GEOMETRY_TYPE "geometry_type"
#define PROP_TILE_SIZE     "tile_size"


FeatureTileSource::FeatureTileSource( const PluginOptions* options ) :
TileSource( options ),
_geomTypeOverride( Geometry::TYPE_UNKNOWN ),
_tileSize( 256 )
{
    const Config& conf = options->config();

    // the data source from which to pull features:
    _features = FeatureSourceFactory::create( conf.child( PROP_FEATURES ) );
    if ( !_features.valid() )
    {
        osg::notify( osg::WARN ) << "[osgEarth] FeatureModelSource - no valid feature source provided" << std::endl;
    }

    // force a particular geometry type
    if ( conf.hasValue( PROP_GEOMETRY_TYPE ) )
    {
        // geometry type override: the config can ask that input geometry
        // be interpreted as a particular geometry type
        std::string gt = conf.value( PROP_GEOMETRY_TYPE );
        if ( gt == "line" || gt == "lines" || gt == "linestrip" )
            _geomTypeOverride = Geometry::TYPE_LINESTRING;
        else if ( gt == "point" || gt == "points" || gt == "pointset" )
            _geomTypeOverride = Geometry::TYPE_POINTSET;
        else if ( gt == "polygon" || gt == "polygons" )
            _geomTypeOverride = Geometry::TYPE_POLYGON;
    }

    // custom tile size?
    _tileSize = conf.value<int>( PROP_TILE_SIZE, _tileSize );

    // load up the style catalog.
    StyleReader::readLayerStyles( this->getName(), conf, _styleCatalog );
}

void 
FeatureTileSource::initialize( const std::string& referenceURI, const Profile* overrideProfile)
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
        _features->initialize( referenceURI );
}

osg::Image*
FeatureTileSource::createImage( const TileKey* key, ProgressCallback* progress )
{
    if ( !_features.valid() || !_features->getFeatureProfile() )
        return 0L;

    // implementation-specific data
    osg::ref_ptr<osg::Referenced> buildData = createBuildData();

	// allocate the image.
	osg::ref_ptr<osg::Image> image = new osg::Image();
	image->allocateImage( getPixelsPerTile(), getPixelsPerTile(), 1, GL_RGBA, GL_UNSIGNED_BYTE );

    preProcess( image.get(), buildData.get() );

    // figure out which rule to use to style the geometry.
    StyledLayer layer;
    bool hasStyledLayer = _styleCatalog.getNamedLayer( this->getName(), layer );

    if ( hasStyledLayer )
    {
        //osg::notify(osg::NOTICE) << "Styled layer def found" << std::endl;

        // The catalog contains style data for this source, so use it:
        for( StyleList::iterator i = layer.styles().begin(); i != layer.styles().end(); ++i )
        {
            const Style& style = *i;
			queryAndRenderFeaturesForStyle( style, buildData.get(), key->getGeoExtent(), image.get() );
        }
    }
    else if ( _features->hasEmbeddedStyles() )
    {
        //osg::notify(osg::NOTICE) << "Using embedded style info" << std::endl;

        // Each feature has its own embedded style data, so use that:
        osg::ref_ptr<FeatureCursor> cursor = _features->createFeatureCursor( Query() );
        while( cursor->hasMore() )
        {
            Feature* feature = cursor->nextFeature();
            if ( feature )
            {
                FeatureList list;
                list.push_back( feature );
                renderFeaturesForStyle( 
					feature->style().get(), list, buildData.get(),
					key->getGeoExtent(), image.get() );
            }
        }
    }
    else
    {
        //osg::notify(osg::NOTICE) << "[osgEarth] " << getName() << ": no styles found for '" << this->getName() << "'" << std::endl;

        // There is no style data, so use the default.
		queryAndRenderFeaturesForStyle( Style(), buildData.get(), key->getGeoExtent(), image.get() );
    }

    // final tile processing after all styles are done
    postProcess( image.get(), buildData.get() );

	return image.release();
}


bool
FeatureTileSource::queryAndRenderFeaturesForStyle(const Style& style,
												  osg::Referenced* data,
												  const GeoExtent& imageExtent,
												  osg::Image* out_image)
{
    osg::Group* styleGroup = 0L;

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
        Query query = style.query().value();
        query.bounds() = query.bounds().isSet()?
		    query.bounds()->unionWith( queryExtent.bounds() ) :
		    queryExtent.bounds();

        // query the feature source:
        osg::ref_ptr<FeatureCursor> cursor = _features->createFeatureCursor( query );

        // now copy the resulting feature set into a list, converting the data
        // types along the way if a geometry override is in place:
        FeatureList cellFeatures;
        while( cursor->hasMore() )
        {
            Feature* feature = cursor->nextFeature();
            Geometry* geom = feature->getGeometry();
            if ( geom )
            {
                // apply a type override if requested:
                if ( _geomTypeOverride.isSet() && _geomTypeOverride.get() != geom->getComponentType() )
                {
                    geom = geom->cloneAs( _geomTypeOverride.get() );
                    if ( geom )
                        feature->setGeometry( geom );
                }
            }
            if ( geom )
            {
                cellFeatures.push_back( feature );
            }
        }

        //osg::notify(osg::NOTICE)
        //    << "[osgEarth] Rendering "
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

