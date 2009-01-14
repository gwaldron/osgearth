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

#include <osgEarth/GeocentricTileBuilder>
#include <osgEarth/PlateCarre>
#include <osgEarth/Mercator>
#include <osgEarth/TerrainTileEdgeNormalizerUpdateCallback>

#include <osg/Image>
#include <osg/Timer>
#include <osg/Notify>
#include <osg/PagedLOD>
#include <osg/ClusterCullingCallback>
#include <osg/CoordinateSystemNode>
#include <osg/Version>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgTerrain/Terrain>
#include <osgTerrain/TerrainTile>
#include <osgTerrain/Locator>
#include <osgTerrain/GeometryTechnique>
#include <sstream>
#include <stdlib.h>

using namespace osgEarth;

//#define WGS84_WKT "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],TOWGS84[0,0,0,0,0,0,0],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9108\"]],AXIS["Lat",NORTH],AXIS["Long",EAST],AUTHORITY["EPSG","4326"]]


GeocentricTileBuilder::GeocentricTileBuilder( 
    MapConfig* map, 
    const std::string& url_template,
    const osgDB::ReaderWriter::Options* global_options ) :
TileBuilder( map, url_template, global_options )
{
    //NOP   
}


osg::Node*
GeocentricTileBuilder::createCap(const double &min_lat, const double &max_lat, const osg::Vec4ub &color)
{
    double min_lon = -180.0;
    double max_lon = 180.0;

    osgTerrain::TerrainTile* tile = new osgTerrain::TerrainTile();

    osgTerrain::Locator* locator = new osgTerrain::Locator();
    locator->setCoordinateSystemType( osgTerrain::Locator::GEOCENTRIC );
	locator->setTransform(getTransformFromExtents(
        osg::DegreesToRadians( min_lon ),
        osg::DegreesToRadians( min_lat ),
        osg::DegreesToRadians( max_lon ),
        osg::DegreesToRadians( max_lat )));

    osg::HeightField *hf = new osg::HeightField();
    hf->allocate(32,32);
    for(unsigned int i=0; i<hf->getHeightList().size(); i++ ) hf->getHeightList()[i] = 0.0;

    hf->setOrigin( osg::Vec3d( min_lon, min_lat, 0.0 ) );
    hf->setXInterval( (max_lon - min_lon)/(double)(hf->getNumColumns()-1) );
    hf->setYInterval( (max_lat - min_lat)/(double)(hf->getNumRows()-1) );
    hf->setBorderWidth( 0 );

    osgTerrain::HeightFieldLayer* hf_layer = new osgTerrain::HeightFieldLayer();
    hf_layer->setLocator( locator );
    hf_layer->setHeightField( hf );

    osg::Image *image = new osg::Image();
    image->allocateImage(1,1,1, GL_RGBA, GL_UNSIGNED_BYTE);
    unsigned char *data = image->data(0,0);
    memcpy(data, color.ptr(), 4);

    osgTerrain::ImageLayer* img_layer = new osgTerrain::ImageLayer( image );
    img_layer->setLocator( locator );
    tile->setColorLayer( 0, img_layer );

    tile->setLocator( locator );
    tile->setTerrainTechnique( new osgTerrain::GeometryTechnique() );
    tile->setElevationLayer( hf_layer );
    tile->setRequiresNormals( true );


    osg::EllipsoidModel* ellipsoid = locator->getEllipsoidModel();
    double x, y, z;
    ellipsoid->convertLatLongHeightToXYZ(
        osg::DegreesToRadians( (max_lat+min_lat)/2.0 ),
        osg::DegreesToRadians( (max_lon+min_lon)/2.0 ),
        0.0,
        x, y, z );

    osg::Vec3d centroid( x, y, z );

    double sw_x, sw_y, sw_z;
    ellipsoid->convertLatLongHeightToXYZ(
        osg::DegreesToRadians( min_lat ),
        osg::DegreesToRadians( min_lon ),
        0.0,
        sw_x, sw_y, sw_z );

    double max_range = 1e10;
    double radius = (centroid-osg::Vec3d(sw_x,sw_y,sw_z)).length();
    double min_range = radius * map->getMinTileRangeFactor();

    //Set the skirt height of the heightfield
    hf->setSkirtHeight(radius * map->getSkirtRatio());

    osg::Vec3d normal = centroid;
    normal.normalize();
    // dot product: 0 = orthogonal to normal, -1 = equal to normal
    float deviation = -radius/locator->getEllipsoidModel()->getRadiusPolar();

    osg::ClusterCullingCallback* ccc = new osg::ClusterCullingCallback();
    ccc->set( centroid, normal, deviation, radius );
    tile->setCullCallback( ccc );


    return tile;

}

bool
GeocentricTileBuilder::addChildren( osg::Group* tile_parent, const TileKey* key )
{
    if (key->getLevelOfDetail() == 0)
    {
        const osgEarth::TileGridProfile& profile = key->getProfile();
        
        //Extend the caps out so they slightly overlap neighboring tiles to hide seams
        double cap_offset = 0.1;
        
        //Draw a "cap" on the bottom of the earth to account for missing tiles
        if (profile.yMin() > -90)
        {
            tile_parent->addChild(createCap(-90, profile.yMin()+cap_offset,  map->getSouthCapColor()));
        }

        //Draw a "cap" on the top of the earth to account for missing tiles
        if (profile.yMax() < 90)
        {   
            tile_parent->addChild(createCap(profile.yMax()-cap_offset, 90, map->getNorthCapColor()));
        }
    }

    osg::ref_ptr<osg::Node> q0 = createQuadrant(key->getSubkey(0));
    osg::ref_ptr<osg::Node> q1 = createQuadrant(key->getSubkey(1));
    osg::ref_ptr<osg::Node> q2;
    osg::ref_ptr<osg::Node> q3;

    bool allQuadrantsCreated = (q0.valid() && q1.valid());

    if ( key->getLevelOfDetail() > 0 || dynamic_cast<const MercatorTileKey*>( key ) )
    {
        q2 = createQuadrant( key->getSubkey( 2 ));
        q3 = createQuadrant( key->getSubkey( 3 ));
        allQuadrantsCreated = (allQuadrantsCreated && q2.valid() && q3.valid());
    }

    if (allQuadrantsCreated)
    {
        if (q0.valid()) tile_parent->addChild(q0.get());
        if (q1.valid()) tile_parent->addChild(q1.get());
        if (q2.valid()) tile_parent->addChild(q2.get());
        if (q3.valid()) tile_parent->addChild(q3.get());
    }
    else
    {
        osg::notify(osg::INFO) << "Couldn't create all 4 quadrants for " << key->str() << " time to stop subdividing!" << std::endl;
    }
    return allQuadrantsCreated;
}

osg::Node*
GeocentricTileBuilder::createQuadrant( const TileKey* key)
{
    double min_lon, min_lat, max_lon, max_lat;
    if ( !key->getGeoExtents( min_lon, min_lat, max_lon, max_lat ) )
    {
        osg::notify( osg::WARN ) << "GET EXTENTS FAILED!" << std::endl;
        return NULL;
    }

    ImageTileList image_tiles;

    //TODO: select/composite:
    //Create the images for the tile
    if ( image_sources.size() > 0 )
    {
        //Add an image from each image source
        for (unsigned int i = 0; i < image_sources.size(); ++i)
        {
            osg::Image *image = 0;
            if (key->getLevelOfDetail() >= image_sources[i]->getMinLevel() &&
                key->getLevelOfDetail() <= image_sources[i]->getMaxLevel())
            {
                image = image_sources[i]->readImage(key);
            }
            image_tiles.push_back(ImageTileKeyPair(image, key));
        }
    }

    //Create the heightfield for the tile
    osg::ref_ptr<osg::HeightField> hf = NULL;
    //TODO: select/composite.
    if ( heightfield_sources.size() > 0 )
    {
        hf = heightfield_sources[0]->readHeightField(key);
    }

    //Determine if we've created any images
    unsigned int numValidImages = 0;
    for (unsigned int i = 0; i < image_tiles.size(); ++i)
    {
        if (image_tiles[i].first.valid()) numValidImages++;
    }

    //If we couldn't create any imagery of heightfields, bail out
    if (!hf.valid() && (numValidImages == 0))
    {
        osg::notify(osg::INFO) << "Could not create any imagery or heightfields for " << key->str() <<".  Not building tile" << std::endl;
        return NULL;
    }
   
    //Try to interpolate any missing imagery from parent tiles
    for (unsigned int i = 0; i < image_sources.size(); ++i)
    {
        if (!image_tiles[i].first.valid())
        {
            if (key->getLevelOfDetail() >= image_sources[i]->getMinLevel() &&
                key->getLevelOfDetail() <= image_sources[i]->getMaxLevel())
            {
                //osg::Timer_t start = osg::Timer::instance()->tick();
                if (!createValidImage(image_sources[i].get(), key, image_tiles[i]))
                {
                    osg::notify(osg::INFO) << "Could not get valid image from image source " << i << " for TileKey " << key->str() << std::endl;
                }
                else
                {
                    //osg::Timer_t end = osg::Timer::instance()->tick();
                    //osg::notify(osg::NOTICE) << "TimeToImterpolateImagery: " << osg::Timer::instance()->delta_m(start,end) << std::endl; 
                    osg::notify(osg::INFO) << "Interpolated imagery from image source " << i << " for TileKey " << key->str() << std::endl;
                }
            }
        }
    }

    //Fill in missing heightfield information from parent tiles
    if (!hf.valid())
    {
        //We have no heightfield sources, 
        if (heightfield_sources.size() == 0)
        {
            //Make any empty heightfield if no heightfield source is specified
            hf = new osg::HeightField();
            hf->allocate( 8, 8 );
            for(unsigned int i=0; i<hf->getHeightList().size(); i++ )
                hf->getHeightList()[i] = 0.0; //(double)((::rand() % 10000) - 5000);
        }
        else
        {
            //osg::Timer_t start = osg::Timer::instance()->tick();
            hf = createValidHeightField(heightfield_sources[0].get(), key);
            if (!hf.valid())
            {
                osg::notify(osg::WARN) << "Could not get valid heightfield for TileKey " << key->str() << std::endl;
                return NULL;
            }
            else
            {
                //osg::Timer_t end = osg::Timer::instance()->tick();
                //osg::notify(osg::NOTICE) << "TimeToImterpolateHeightField: " << osg::Timer::instance()->delta_m(start,end) << std::endl; 
                osg::notify(osg::INFO) << "Interpolated heightfield TileKey " << key->str() << std::endl;
            }
        }
    }
           
    osgTerrain::Locator* locator = new osgTerrain::Locator();
    locator->setCoordinateSystemType( osgTerrain::Locator::GEOCENTRIC );
	locator->setTransform( getTransformFromExtents(
		osg::DegreesToRadians( min_lon ),
        osg::DegreesToRadians( min_lat ),
        osg::DegreesToRadians( max_lon ),
        osg::DegreesToRadians( max_lat ) ));

    hf->setOrigin( osg::Vec3d( min_lon, min_lat, 0.0 ) );
    hf->setXInterval( (max_lon - min_lon)/(double)(hf->getNumColumns()-1) );
    hf->setYInterval( (max_lat - min_lat)/(double)(hf->getNumRows()-1) );
    hf->setBorderWidth( 0 );

    osgTerrain::HeightFieldLayer* hf_layer = new osgTerrain::HeightFieldLayer();
    hf_layer->setLocator( locator );
    hf_layer->setHeightField( hf.get() );

    osgTerrain::TerrainTile* tile = new osgTerrain::TerrainTile();
    tile->setTileID(key->getTileId());

    //Attach an updatecallback to normalize the edges of TerrainTiles.
    tile->setUpdateCallback(new TerrainTileEdgeNormalizerUpdateCallback());
    tile->setDataVariance(osg::Object::DYNAMIC);

    tile->setLocator( locator );
    tile->setTerrainTechnique( new osgTerrain::GeometryTechnique() );
    tile->setElevationLayer( hf_layer );
    tile->setRequiresNormals( true );

    //Assign the terrain system to the TerrainTile
    tile->setTerrain( terrain.get() );

    int layer = 0;
    for (unsigned int i = 0; i < image_tiles.size(); ++i)
    {
        if (image_tiles[i].first.valid())
        {
            double img_min_lon, img_min_lat, img_max_lon, img_max_lat;
            image_tiles[i].second->getGeoExtents(img_min_lon, img_min_lat, img_max_lon, img_max_lat);

            //Specify a new locator for the color with the coordinates of the TileKey that was actually used to create the image
            osg::ref_ptr<osgTerrain::Locator> img_locator = new osgTerrain::Locator;
            img_locator->setCoordinateSystemType( osgTerrain::Locator::GEOCENTRIC );
			img_locator->setTransform( getTransformFromExtents(
                osg::DegreesToRadians( img_min_lon ),
                osg::DegreesToRadians( img_min_lat ),
                osg::DegreesToRadians( img_max_lon ),
                osg::DegreesToRadians( img_max_lat )));

            // use a special image locator to warp the texture coords for mercator tiles :)
            // WARNING: TODO: this will not persist upon export....we need a nodekit.
            if ( dynamic_cast<const MercatorTileKey*>( key ) )
                img_locator = new MercatorLocator(*img_locator.get(), image_tiles[i].first->s(), image_tiles[i].second->getLevelOfDetail() );
            osgTerrain::ImageLayer* img_layer = new osgTerrain::ImageLayer( image_tiles[i].first.get());
            img_layer->setLocator( img_locator.get());
#if (OPENSCENEGRAPH_MAJOR_VERSION == 2 && OPENSCENEGRAPH_MINOR_VERSION < 7)
            img_layer->setFilter( osgTerrain::Layer::LINEAR );
#else
            img_layer->setMinFilter(osg::Texture::LINEAR_MIPMAP_LINEAR);
            img_layer->setMagFilter(osg::Texture::LINEAR);
#endif
            tile->setColorLayer( layer, img_layer );
            layer++;
        }
    }
    
    osg::EllipsoidModel* ellipsoid = locator->getEllipsoidModel();
    double x, y, z;
    ellipsoid->convertLatLongHeightToXYZ(
        osg::DegreesToRadians( (max_lat+min_lat)/2.0 ),
        osg::DegreesToRadians( (max_lon+min_lon)/2.0 ),
        0.0,
        x, y, z );
    
    osg::Vec3d centroid( x, y, z );

    double sw_x, sw_y, sw_z;
    ellipsoid->convertLatLongHeightToXYZ(
        osg::DegreesToRadians( min_lat ),
        osg::DegreesToRadians( min_lon ),
        0.0,
        sw_x, sw_y, sw_z );

    double max_range = 1e10;
    double radius = (centroid-osg::Vec3d(sw_x,sw_y,sw_z)).length();
    double min_range = radius * map->getMinTileRangeFactor();

    //Set the skirt height of the heightfield
    hf->setSkirtHeight(radius * map->getSkirtRatio());

    osg::Vec3d normal = centroid;
    normal.normalize();
    // dot product: 0 = orthogonal to normal, -1 = equal to normal
    float deviation = -radius/locator->getEllipsoidModel()->getRadiusPolar();
            
    osg::ClusterCullingCallback* ccc = new osg::ClusterCullingCallback();
    ccc->set( centroid, normal, deviation, radius );
    tile->setCullCallback( ccc );

    osg::PagedLOD* plod = new osg::PagedLOD();
    plod->setCenter( centroid );
    plod->addChild( tile, min_range, max_range );
    plod->setFileName( 1, createURI( key ) );
    plod->setRange( 1, 0.0, min_range );

    return plod;
}

osg::CoordinateSystemNode*
GeocentricTileBuilder::createCoordinateSystemNode() const
{
    osg::CoordinateSystemNode* csn = new osg::CoordinateSystemNode();
    csn->setEllipsoidModel( new osg::EllipsoidModel() );
    csn->setCoordinateSystem( "+init=epsg:4326" );
    csn->setFormat( "PROJ4" );
    return csn;
}

