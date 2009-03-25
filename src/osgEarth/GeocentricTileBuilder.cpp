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
#include <osgEarth/Mercator>
#include <osgEarth/TerrainTileEdgeNormalizerUpdateCallback>
#include <osgEarth/MultiImage>
#include <osgEarth/ImageUtils>

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
#include <string.h>
#include <stdlib.h>

using namespace osgEarth;

//#define WGS84_WKT "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],TOWGS84[0,0,0,0,0,0,0],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9108\"]],AXIS["Lat",NORTH],AXIS["Long",EAST],AUTHORITY["EPSG","4326"]]

void getLatLonExtents(const TileKey* key, double &min_lon, double &min_lat, double &max_lon, double &max_lat)
{
    key->getGeoExtents(min_lon, min_lat, max_lon, max_lat);
    if (key->isMercator())
    {
        Mercator::metersToLatLon(min_lon, min_lat, min_lat, min_lon);
        Mercator::metersToLatLon(max_lon, max_lat, max_lat, max_lon);
    }
}


GeocentricTileBuilder::GeocentricTileBuilder( 
    MapConfig* map, 
    const osgDB::ReaderWriter::Options* global_options ) :
TileBuilder( map, global_options )
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
    if ( key->getLevelOfDetail() == 0 )
    {
        const osgEarth::TileGridProfile& profile = key->getProfile();
        double minY = profile.yMin();
        double maxY = profile.yMax();

        //Convert meters to lat/lon if the profile is Mercator
        if (profile.getProfileType() == TileGridProfile::GLOBAL_MERCATOR)
        {
            double lat,lon;
            Mercator::metersToLatLon(0, minY, lat, lon);
            minY = lat;
            Mercator::metersToLatLon(0, maxY, lat, lon);
            maxY = lat;
        }
        
        //Extend the caps out so they slightly overlap neighboring tiles to hide seams
        double cap_offset = 0.1;
        
        //Draw a "cap" on the bottom of the earth to account for missing tiles
        if (minY > -90)
        {
            tile_parent->addChild(createCap(-90, minY+cap_offset,  map->getSouthCapColor()));
        }

        //Draw a "cap" on the top of the earth to account for missing tiles
        if (maxY < 90)
        {   
            tile_parent->addChild(createCap(maxY-cap_offset, 90, map->getNorthCapColor()));
        }
    }

    return TileBuilder::addChildren( tile_parent, key );
}

osg::Node*
GeocentricTileBuilder::createQuadrant( const TileKey* key)
{
    double min_lon, min_lat, max_lon, max_lat;
    getLatLonExtents( key, min_lon, min_lat, max_lon, max_lat);

    ImageTileList image_tiles;

    //TODO: select/composite:
    //Create the images for the tile
    if ( image_sources.size() > 0 )
    {
        //Add an image from each image source
        for (unsigned int i = 0; i < image_sources.size(); ++i)
        {
            osg::Image *image = 0;
            if (image_sources[i]->isKeyValid(key))
            {
                image = createImage(key, image_sources[i].get());    
            }
            image_tiles.push_back(ImageTileKeyPair(image, key));
        }
    }

    bool hasElevation = false;

    //Create the heightfield for the tile
    osg::ref_ptr<osg::HeightField> hf = NULL;
    //TODO: select/composite.
    if ( heightfield_sources.size() > 0 )
    {
        if (heightfield_sources[0]->isKeyValid(key))
        {
            hf = createHeightField(key, heightfield_sources[0].get());
            hasElevation = hf.valid();
        }
    }

    //Determine if we've created any images
    unsigned int numValidImages = 0;
    for (unsigned int i = 0; i < image_tiles.size(); ++i)
    {
        if (image_tiles[i].first.valid()) numValidImages++;
    }


    //If we couldn't create any imagery or heightfields, bail out
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
            if (image_sources[i]->isKeyValid(key))
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
                hasElevation = true;
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
    if (hasElevation && map->getNormalizeEdges())
    {
        tile->setUpdateCallback(new TerrainTileEdgeNormalizerUpdateCallback());
        tile->setDataVariance(osg::Object::DYNAMIC);
    }

    tile->setLocator( locator );
    tile->setTerrainTechnique( new osgTerrain::GeometryTechnique() );
    tile->setElevationLayer( hf_layer );
    tile->setRequiresNormals( true );

    //Assign the terrain system to the TerrainTile
    if (terrain.valid())
    {
      tile->setTerrain( terrain.get() );
    }

    int layer = 0;
    for (unsigned int i = 0; i < image_tiles.size(); ++i)
    {
        if (image_tiles[i].first.valid())
        {
            double img_min_lon, img_min_lat, img_max_lon, img_max_lat;
            getLatLonExtents(image_tiles[i].second.get(), img_min_lon, img_min_lat, img_max_lon, img_max_lat);

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

            // TODO: perhaps we can ask the key/profile to create a locator instead of explicity
            // checking for mercator..

            if ( key->isMercator() )
                img_locator = new MercatorLocator(*img_locator.get(), image_tiles[i].first->s(), image_tiles[i].second->getLevelOfDetail() );

            osgTerrain::ImageLayer* img_layer = new osgTerrain::ImageLayer( image_tiles[i].first.get());
            img_layer->setLocator( img_locator.get());

            //Turn on linear texture minification if the image is not a power of two due
            //to the fact that some drivers have issues with npot mip mapping
            if (!ImageUtils::isPowerOfTwo(image_tiles[i].first.get()))
            {
#if (OPENSCENEGRAPH_MAJOR_VERSION == 2 && OPENSCENEGRAPH_MINOR_VERSION < 7)
                img_layer->setFilter( osgTerrain::Layer::LINEAR );
#else
                img_layer->setMinFilter(osg::Texture::LINEAR);
#endif
            }
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
            
    osg::ClusterCullingCallback* ccc = createClusterCullingCallback(tile, ellipsoid);
    tile->setCullCallback( ccc );

    // see if we need to keep subdividing:
    osg::Node* result = tile;
    if ( hasMoreLevels( key ) )
    {
        osg::PagedLOD* plod = new osg::PagedLOD();
        plod->setCenter( centroid );
        plod->addChild( tile, min_range, max_range );
        plod->setFileName( 1, createURI( key ) );
        plod->setRange( 1, 0.0, min_range );
        result = plod;
    }
    else 
    {
        //osg::notify(osg::NOTICE) << "[osgEarth] Stopping at level " << key->getLevelOfDetail() << std::endl;
    }

    return result;
}

osg::CoordinateSystemNode*
GeocentricTileBuilder::createCoordinateSystemNode() const
{
    osg::CoordinateSystemNode* csn = new osg::CoordinateSystemNode();

    // TODO: set the CORRECT ellipsoid model, based on the profile/srs:
    csn->setEllipsoidModel( new osg::EllipsoidModel() );

    _dataProfile.applyTo( csn );

    return csn;
}

osg::ClusterCullingCallback*
GeocentricTileBuilder::createClusterCullingCallback(osgTerrain::TerrainTile* tile, osg::EllipsoidModel* et)
{
    //This code is a very slightly modified version of the DestinationTile::createClusterCullingCallback in VirtualPlanetBuilder.
    osg::HeightField* grid = ((osgTerrain::HeightFieldLayer*)tile->getElevationLayer())->getHeightField();
    if (!grid) return 0;

    double globe_radius = et ? et->getRadiusPolar() : 1.0;
    unsigned int numColumns = grid->getNumColumns();
    unsigned int numRows = grid->getNumRows();


    double midLong = grid->getOrigin().x()+grid->getXInterval()*((double)(numColumns-1))*0.5;
    double midLat = grid->getOrigin().y()+grid->getYInterval()*((double)(numRows-1))*0.5;
    double midZ = grid->getOrigin().z();

    double midX,midY;
    et->convertLatLongHeightToXYZ(osg::DegreesToRadians(midLat),osg::DegreesToRadians(midLong),midZ, midX,midY,midZ);

    osg::Vec3 center_position(midX,midY,midZ);

    osg::Vec3 center_normal(midX,midY,midZ);
    center_normal.normalize();
    
    osg::Vec3 transformed_center_normal = center_normal;

    unsigned int r,c;
    
    // populate the vertex/normal/texcoord arrays from the grid.
    double orig_X = grid->getOrigin().x();
    double delta_X = grid->getXInterval();
    double orig_Y = grid->getOrigin().y();
    double delta_Y = grid->getYInterval();
    double orig_Z = grid->getOrigin().z();


    float min_dot_product = 1.0f;
    float max_cluster_culling_height = 0.0f;
    float max_cluster_culling_radius = 0.0f;

    for(r=0;r<numRows;++r)
    {
        for(c=0;c<numColumns;++c)
        {
            double X = orig_X + delta_X*(double)c;
            double Y = orig_Y + delta_Y*(double)r;
            double Z = orig_Z + grid->getHeight(c,r);
            double height = Z;

            et->convertLatLongHeightToXYZ(osg::DegreesToRadians(Y),osg::DegreesToRadians(X),Z,
                                         X,Y,Z);

            osg::Vec3d v(X,Y,Z);
            osg::Vec3 dv = v - center_position;
            double d = sqrt(dv.x()*dv.x() + dv.y()*dv.y() + dv.z()*dv.z());
            double theta = acos( globe_radius/ (globe_radius + fabs(height)) );
            double phi = 2.0 * asin (d*0.5/globe_radius); // d/globe_radius;
            double beta = theta+phi;
            double cutoff = osg::PI_2 - 0.1;
            
            //log(osg::INFO,"theta="<<theta<<"\tphi="<<phi<<" beta "<<beta);
            if (phi<cutoff && beta<cutoff)
            {

                float local_dot_product = -sin(theta + phi);
                float local_m = globe_radius*( 1.0/ cos(theta+phi) - 1.0);
                float local_radius = static_cast<float>(globe_radius * tan(beta)); // beta*globe_radius;
                min_dot_product = osg::minimum(min_dot_product, local_dot_product);
                max_cluster_culling_height = osg::maximum(max_cluster_culling_height,local_m);      
                max_cluster_culling_radius = osg::maximum(max_cluster_culling_radius,local_radius);
            }
            else
            {
                //log(osg::INFO,"Turning off cluster culling for wrap around tile.");
                return 0;
            }
        }
    }
    

    // set up cluster cullling, 
    osg::ClusterCullingCallback* ccc = new osg::ClusterCullingCallback;

    ccc->set(center_position + transformed_center_normal*max_cluster_culling_height ,
             transformed_center_normal, 
             min_dot_product,
             max_cluster_culling_radius);

    return ccc;
}

