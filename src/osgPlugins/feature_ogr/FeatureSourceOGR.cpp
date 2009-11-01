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

#include <osgEarth/Registry>
#include <osgEarthFeatures/FeatureSource>
#include "FeatureCursorOGR"

#include <osg/Notify>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>

#include <ogr_api.h>

using namespace osgEarth;
using namespace osgEarthFeatures;

#define OGR_SCOPED_LOCK GDAL_SCOPED_LOCK

#define PROPERTY_URL "url"
#define PROPERTY_OGR_DRIVER "ogr_driver"
#define PROPERTY_GEOMETRY_TYPE "geometry_type" // "line", "point", "polygon"

/**
 * A FeatureSource that reads features from an OGR driver.
 */
class FeatureSourceOGR : public FeatureSource
{
public:
    FeatureSourceOGR( const PluginOptions* options ) : FeatureSource( options ),
      _ready( false ),
      _dsHandle( 0L ),
      _layerHandle( 0L ),
      _supportsRandomRead( false ),
      _geomTypeOverride( FeatureProfile::GEOM_UNKNOWN )
    {
        const Config& conf = getOptions()->config();

        _url = conf.value( PROPERTY_URL );
        _ogrDriver = conf.value( PROPERTY_OGR_DRIVER );

        std::string gt = conf.value( PROPERTY_GEOMETRY_TYPE );
        if ( gt == "line" || gt == "lines" )
            _geomTypeOverride = FeatureProfile::GEOM_LINE;
        else if ( gt == "point" || gt == "points" )
            _geomTypeOverride = FeatureProfile::GEOM_POINT;
        else if ( gt == "polygon" || gt == "polygons" )
            _geomTypeOverride = FeatureProfile::GEOM_POLYGON;
    }

    /** Destruct the object, cleaning up and OGR handles. */
    virtual ~FeatureSourceOGR()
    {       
        OGR_SCOPED_LOCK;

        if ( _layerHandle )
        {
            // OGR_L_SyncToDisk( _layerHandle ); // for writing only
            _layerHandle = 0L;
        }

        if ( _dsHandle )
        {
            OGRReleaseDataSource( _dsHandle );
            _dsHandle = 0L;
        }
    }

    void initialize( const std::string& referenceURI, const Profile* overrideProfile )
    {
        // never called????
    }

    /** Called once at startup to create the profile for this feature set. Successful profile
        creation implies that the datasource opened succesfully. */
    FeatureProfile* createFeatureProfile()
    {
        FeatureProfile* result = 0L;

        OGR_SCOPED_LOCK;

        OGRSFDriverH ogrDriverHandle = 0L;
        if ( !_ogrDriver.empty() )
        {
            ogrDriverHandle = OGRGetDriverByName( _ogrDriver.c_str() );
        }

	    _dsHandle = OGROpenShared( _url.c_str(), 0, &ogrDriverHandle ); // last param is the driver spec..TODO!!
	    if ( _dsHandle )
	    {
		    _layerHandle = OGR_DS_GetLayer( _dsHandle, 0 ); // default to layer 0 for now
            if ( _layerHandle )
            {
                _supportsRandomRead = OGR_L_TestCapability( _layerHandle, OLCRandomRead ) == TRUE;

                // WARN the user if we load an ESRI-style LCC SRS, in which the PROJECTION["Lambert_Conformal_Conic"]
                // should really be Lambert_Conformal_Conic_1SP or _2SP.
                //OGR_SpatialReference* ogr_srs = dynamic_cast<OGR_SpatialReference*>( getSRS() );
                //if ( ogr_srs )
                //{
                //    if ( ogr_srs->getAttrValue( "PROJECTION", 0 ) == "Lambert_Conformal_Conic" )
                //    {
                //        osgGIS::notify(osg::WARN)
                //            << std::endl
                //            << "***WANRING*** SRS has an invalid ESRI-style LCC projection ... transformations may not work"
                //            << std::endl
                //            << "SRS = " << ogr_srs->getWKT() << std::endl
                //            << std::endl;
                //    }
                //}

                // extract the SRS.
                OGRSpatialReferenceH srHandle = OGR_L_GetSpatialRef( _layerHandle );
                if ( srHandle )
                {
                    osg::ref_ptr<SpatialReference> srs = SpatialReference::createFromHandle( srHandle );

                    // read the first feature to determine the geometry type and dimension.
                    OGR_L_ResetReading( _layerHandle );
                    OGRFeatureH testFeatureHandle = OGR_L_GetNextFeature( _layerHandle );
                    if ( testFeatureHandle )
                    {
                        OGRGeometryH geomHandleRef = OGR_F_GetGeometryRef( testFeatureHandle ); // no need to destroy "refs"
                	    if ( geomHandleRef )
                        {
                            OGRwkbGeometryType wkb_type = OGR_G_GetGeometryType( geomHandleRef );

                            FeatureProfile::GeometryType geomType = FeatureProfile::GEOM_UNKNOWN;

                            if ( 
                                wkb_type == wkbLineString ||
                                wkb_type == wkbLineString25D ||
                                wkb_type == wkbMultiLineString ||
                                wkb_type == wkbMultiLineString25D )
                            {
                                geomType = FeatureProfile::GEOM_LINE;
                            }
                            else if (
                                wkb_type == wkbMultiPoint ||
                                wkb_type == wkbMultiPoint25D ||
                                wkb_type == wkbPoint ||
                                wkb_type == wkbPoint25D )
                            {
                                geomType = FeatureProfile::GEOM_POINT;
                            }
                            else if (
                                wkb_type == wkbMultiPolygon ||
                                wkb_type == wkbMultiPolygon25D ||
                                wkb_type == wkbPolygon ||
                                wkb_type == wkbPolygon25D )
                            {
                                geomType = FeatureProfile::GEOM_POLYGON;
                            }
                            else // unsupported type.
                            {
                                osg::notify( osg::WARN ) << "[osgEarth::FeatureSourceOGR] Unsupported WKB shape type:" << wkb_type << std::endl;
                            }
                           
                            bool multiGeom = wkb_type == wkbMultiPolygon || wkb_type == wkbMultiPolygon25D;

                            // still need to do the above to determine the multiGeom flag..
                            if ( _geomTypeOverride != FeatureProfile::GEOM_UNKNOWN )
                                geomType = _geomTypeOverride;

                            // extract the dimensionality of the geometry:
                            int dim = OGR_G_GetCoordinateDimension( geomHandleRef );

                            // if all went well, make the new profile.
                            if ( srs.valid() && dim >= 2 && dim <= 3 && geomType != FeatureProfile::GEOM_UNKNOWN )
                            {
                                result = new FeatureProfile( srs.get(), geomType, dim, multiGeom );
                                //osg::notify(osg::NOTICE) << "_geomTypeOverride = " << _geomTypeOverride << std::endl;
                            }

                        }
                        OGR_F_Destroy( testFeatureHandle );
                    }
                }
            }
	    }

        return result;
    }

    GeoExtent createDataExtent()
    {
        return GeoExtent::INVALID;
    }


    FeatureCursor* createCursor( const FeatureQuery& query ) const
    {
        return new FeatureCursorOGR( _layerHandle, getFeatureProfile(), query );
    }

protected:

    // closes any open OGR objects and releases the handles
    bool cleanup()
    {
        OGR_SCOPED_LOCK;

        if ( _layerHandle )
        {
            // OGR_L_SyncToDisk( _layerHandle ); // for writing only
            _layerHandle = 0L;
        }

        if ( _dsHandle )
        {
            OGRReleaseDataSource( _dsHandle );
            _dsHandle = 0L;
        }

        return true;
    }

private:
    std::string _url;
    std::string _ogrDriver;
    OGRDataSourceH _dsHandle;
    OGRLayerH _layerHandle;
    bool _supportsRandomRead;
    bool _ready;
    FeatureProfile::GeometryType _geomTypeOverride;
};


class ReaderWriterFeatureSourceOGR : public osgDB::ReaderWriter
{
public:
    ReaderWriterFeatureSourceOGR()
    {
        supportsExtension( "osgearth_feature_ogr", "OGR feature driver for osgEarth" );
    }

    virtual const char* className()
    {
        return "OGR Feature Reader";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return new FeatureSourceOGR( static_cast<const PluginOptions*>(options) );
    }
};

REGISTER_OSGPLUGIN(osgearth_feature_ogr, ReaderWriterFeatureSourceOGR)

