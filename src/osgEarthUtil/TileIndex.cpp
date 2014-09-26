/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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
#include <osgEarth/FileUtils>
#include <osgEarthUtil/TileIndex>
#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>
#include <ogr_api.h>
#include <osgEarthFeatures/OgrUtils>
#include <osgDB/FileUtils>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Drivers;
using namespace osgEarth::Features;
using namespace std;

#define OGR_SCOPED_LOCK GDAL_SCOPED_LOCK

TileIndex::TileIndex()
{
}

TileIndex::~TileIndex()
{

}

TileIndex*
    TileIndex::load(const std::string& filename)
{        
    if (!osgDB::fileExists( filename ) )
    {
        return 0;
    }

    //Load up an index file
    OGRFeatureOptions featureOpt;
    featureOpt.url() = filename;        
    featureOpt.buildSpatialIndex() = true;
    featureOpt.openWrite() = true;

    osg::ref_ptr< FeatureSource> features = FeatureSourceFactory::create( featureOpt );        
    if (!features.valid())
    {
        OE_NOTICE << "Can't load " << filename << std::endl;
        return 0;
    }
    features->initialize();
    features->getFeatureProfile();    

    TileIndex* index = new TileIndex();
    index->_features = features.get();
    index->_filename = filename;
    return index;
}

TileIndex*
    TileIndex::create( const std::string& filename, const osgEarth::SpatialReference* srs )
{
    // Make sure the registry is loaded since that is where the OGR/GDAL registration happens
    osgEarth::Registry::instance();

    OGR_SCOPED_LOCK;

    OGRSFDriverH driver = OGRGetDriverByName( "ESRI Shapefile" );    

    //Create the datasource itself.
    OGRDataSourceH dataSource = OGR_Dr_CreateDataSource( driver, filename.c_str(), NULL );
    if (dataSource == NULL)
    {
        OE_WARN << "failed to create " << filename.c_str() << std::endl;
        return 0;
    }

    //Create the layer
    OGRLayerH layer = OGR_DS_CreateLayer( dataSource, "index", (OGRSpatialReferenceH)srs->getHandle(), wkbPolygon, NULL );

    //Create the attribute name to use for the filename
    OGRFieldDefnH  field = OGR_Fld_Create("location", OFTString);
    OGR_L_CreateField( layer, field, TRUE);

    OGR_DS_Destroy( dataSource );

    return load( filename );
}


void
    TileIndex::getFiles(const osgEarth::GeoExtent& extent, std::vector< std::string >& files)
{            
    files.clear();
    osgEarth::Symbology::Query query;    

    GeoExtent transformed = extent.transform( _features->getFeatureProfile()->getSRS() );
    query.bounds() = transformed.bounds();
    osg::ref_ptr< osgEarth::Features::FeatureCursor> cursor = _features->createFeatureCursor( query );

    while (cursor->hasMore())
    {
        osg::ref_ptr< osgEarth::Features::Feature> feature = cursor->nextFeature();
        if (feature.valid())
        {
            std::string location = getFullPath(_filename, feature->getString("location"));
            files.push_back( location );
        }
    }    
}

bool TileIndex::add( const std::string& filename, const GeoExtent& extent )
{       
    osg::ref_ptr< Polygon > polygon = new Polygon();
    polygon->push_back( osg::Vec3d(extent.bounds().xMin(), extent.bounds().yMin(), 0) );
    polygon->push_back( osg::Vec3d(extent.bounds().xMax(), extent.bounds().yMin(), 0) );
    polygon->push_back( osg::Vec3d(extent.bounds().xMax(), extent.bounds().yMax(), 0) );
    polygon->push_back( osg::Vec3d(extent.bounds().xMin(), extent.bounds().yMax(), 0) );
    polygon->push_back( osg::Vec3d(extent.bounds().xMin(), extent.bounds().yMin(), 0) );
   
    osg::ref_ptr< Feature > feature = new Feature( polygon, extent.getSRS()  );
    feature->set("location", filename );
    
    const SpatialReference* wgs84 = SpatialReference::create("epsg:4326");
    feature->transform( wgs84 );

    return _features->insertFeature( feature.get() );    
    return true;
}
