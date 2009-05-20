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

#include <osgEarth/GeocentricMap>
#include <osgEarth/Mercator>
#include <osgEarth/Cube>
#include <osgEarth/TerrainTileEdgeNormalizerUpdateCallback>
#include <osgEarth/Compositing>
#include <osgEarth/ImageUtils>
#include <osgEarth/EarthTerrainTechnique>

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
using namespace OpenThreads;


GeocentricMap::GeocentricMap( const MapConfig& mapConfig ) :
MapEngine( mapConfig )
{
    //NOP   
}

osg::Node*
GeocentricMap::createQuadrant( const TileKey* key )
{
    osg::notify(osg::INFO) << "[osgEarth::GeocentricMap::createQuadrant] Begin " << key->str() << std::endl;
    osg::notify(osg::INFO) << "[osgEarth::GeocentricMap::createQuadrant] Waiting for lock..." << std::endl;
    ScopedReadLock lock( getLayersMutex() );
    osg::notify(osg::INFO) << "[osgEarth::GeocentricMap::createQuadrant] Obtained Lock" << std::endl;

    double min_lon, min_lat, max_lon, max_lat;
    key->getGeoExtent().getBounds(min_lon, min_lat, max_lon, max_lat);

    GeoImageList image_tiles;

    //Collect the image layers
    ImageLayerList imageLayers;
    getImageLayers( imageLayers );

    ElevationLayerList elevationLayers;
    getElevationLayers( elevationLayers );

    bool empty_map = imageLayers.size() == 0 && elevationLayers.size() == 0;

    //TODO: select/composite:
    //Create the images for the tile
    if ( imageLayers.size() > 0 )
    {
        //Add an image from each image source
        for (unsigned int i = 0; i < imageLayers.size(); ++i)
        {
            GeoImage* image = NULL;
            //osg::Image *image = 0;
            if (imageLayers[i]->getTileSource()->isKeyValid( key ) )
            {
                image = createGeoImage( key, imageLayers[i]->getTileSource() );                
            }
            image_tiles.push_back(image);
        }
    }

    bool hasElevation = false;

    //Create the heightfield for the tile
    osg::ref_ptr<osg::HeightField> hf;
    //TODO: select/composite.
    if ( elevationLayers.size() > 0 )
    {
        hf = createHeightField( key, false );
        //hf = _elevationManager->getHeightField(key, 0, 0, false);
        hasElevation = hf.valid();
    }

    //Determine if we've created any images
    unsigned int numValidImages = 0;
    for (unsigned int i = 0; i < image_tiles.size(); ++i)
    {
        if (image_tiles[i].valid()) numValidImages++;
    }


    //If we couldn't create any imagery or heightfields, bail out
    if (!hf.valid() && (numValidImages == 0) && !empty_map)
    {
        osg::notify(osg::INFO) << "Could not create any imagery or heightfields for " << key->str() <<".  Not building tile" << std::endl;
        return NULL;
    }
   
    //Try to interpolate any missing image layers from parent tiles
    for (unsigned int i = 0; i < imageLayers.size(); ++i)
    {
        if (!image_tiles[i].valid())
        {
            if (imageLayers[i]->getTileSource()->isKeyValid(key))
            {
                GeoImage* image = createValidGeoImage(imageLayers[i]->getTileSource(), key);
                if (image)
                {
                    osg::notify(osg::INFO) << "[osgEarth::GeocentricMap] Using fallback image for image source " << imageLayers[i]->getName() << " for TileKey " << key->str() << std::endl;
                    image_tiles[i] = image;
                }
                else
                {
                    osg::notify(osg::INFO) << "[osgEarth::GeocentricMap] Could not get valid image from image source " << imageLayers[i]->getName() << " for TileKey " << key->str() << std::endl;
                }
            }
        }
    }

    //Fill in missing heightfield information from parent tiles
    if (!hf.valid())
    {
        //We have no heightfield sources, 
        if (elevationLayers.size() == 0)
        {
            hf = createEmptyHeightField( key );
        }
        else
        {
            //Try to get a heightfield again, but this time fallback on parent tiles
            hf = createHeightField( key, true );
            if (!hf.valid())
            {
                osg::notify(osg::WARN) << "Could not get valid heightfield for TileKey " << key->str() << std::endl;
                return NULL;
            }
            else
            {
                hasElevation = true;
            }
        }
    }

    osg::ref_ptr<osgTerrain::Locator> locator = key->getProfile()->getSRS()->createLocator(
        min_lon, min_lat, max_lon, max_lat );

    // TESTING.
    //if ( key->getProfile()->getSRS()->getName() == "Square Polar" )
    //    locator = new SquarePolarLocator( *locator.get() );
    locator->setCoordinateSystemType( osgTerrain::Locator::GEOCENTRIC );

    bool isCube = dynamic_cast<CubeFaceLocator*>(locator.get()) != NULL;

    //if (isCube)
    //{
    //    locator->setTransform( getTransformFromExtents(
    //        min_lon,
    //        min_lat,
    //        max_lon,
    //        max_lat));
    //}
    //else
    //{
    //    locator->setTransform( getTransformFromExtents(
    //        osg::DegreesToRadians( min_lon ),
    //        osg::DegreesToRadians( min_lat ),
    //        osg::DegreesToRadians( max_lon ),
    //        osg::DegreesToRadians( max_lat ) ) );
    //}

    //hf->setOrigin( osg::Vec3d( min_lon, min_lat, 0.0 ) );
    //hf->setXInterval( (max_lon - min_lon)/(double)(hf->getNumColumns()-1) );
    //hf->setYInterval( (max_lat - min_lat)/(double)(hf->getNumRows()-1) );
    //hf->setBorderWidth( 0 );

    osgTerrain::HeightFieldLayer* hf_layer = new osgTerrain::HeightFieldLayer();
    hf_layer->setLocator( locator );
    hf_layer->setHeightField( hf.get() );

    osgTerrain::TerrainTile* tile = new osgTerrain::TerrainTile();
    tile->setTileID(key->getTileId());

    //Attach an updatecallback to normalize the edges of TerrainTiles.
    if (hasElevation && _mapConfig.getNormalizeEdges())
    {
        tile->setUpdateCallback(new TerrainTileEdgeNormalizerUpdateCallback());
        tile->setDataVariance(osg::Object::DYNAMIC);
    }

    tile->setLocator( locator );
    //tile->setTerrainTechnique( new osgTerrain::GeometryTechnique() );
    tile->setTerrainTechnique( new osgEarth::EarthTerrainTechnique );
    tile->setElevationLayer( hf_layer );
    tile->setRequiresNormals( true );
    tile->setDataVariance(osg::Object::DYNAMIC);

    //Assign the terrain system to the TerrainTile
    osgTerrain::Terrain* terrain = _terrains[key->getFace()].get();
    if ( terrain )
    {
        tile->setTerrain( terrain );
    }

    int layer = 0;
    for (unsigned int i = 0; i < image_tiles.size(); ++i)
    {
        if (image_tiles[i].valid())
        {
            double img_min_lon, img_min_lat, img_max_lon, img_max_lat;

            //Specify a new locator for the color with the coordinates of the TileKey that was actually used to create the image
            osg::ref_ptr<osgTerrain::Locator> img_locator; // = key->getProfile()->getSRS()->createLocator();
			
            GeoImage* geo_image = image_tiles[i].get();

            // Use a special locator for mercator images (instead of reprojecting)
            if ( geo_image->getSRS()->isMercator() )
            {
                GeoExtent geog_ext = image_tiles[i]->getExtent().transform(image_tiles[i]->getExtent().getSRS()->getGeographicSRS());
                geog_ext.getBounds(img_min_lon, img_min_lat, img_max_lon, img_max_lat);
                img_locator = key->getProfile()->getSRS()->createLocator( img_min_lon, img_min_lat, img_max_lon, img_max_lat );
                img_locator = new MercatorLocator( *img_locator.get(), geo_image->getExtent() );
                //Transform the mercator extents to geographic
            }
            else
            {
                image_tiles[i]->getExtent().getBounds(img_min_lon, img_min_lat, img_max_lon, img_max_lat);
                img_locator = key->getProfile()->getSRS()->createLocator( img_min_lon, img_min_lat, img_max_lon, img_max_lat );
            }

            img_locator->setCoordinateSystemType( osgTerrain::Locator::GEOCENTRIC );

            //TODO:  Check for cube grid here?
            //img_locator->setTransform( getTransformFromExtents(
            //    osg::DegreesToRadians( img_min_lon ),
            //    osg::DegreesToRadians( img_min_lat ),
            //    osg::DegreesToRadians( img_max_lon ),
            //    osg::DegreesToRadians( img_max_lat )));


            //osgTerrain::ImageLayer* img_layer = new osgTerrain::ImageLayer( geo_image->getImage() );
            osgTerrain::ImageLayer* img_layer = new osgEarthImageLayer( imageLayers[i]->getId(), geo_image->getImage() );


            img_layer->setLocator( img_locator.get());

            tile->setColorLayer( layer, img_layer );
            layer++;
        }
    }
    
    osg::EllipsoidModel* ellipsoid = locator->getEllipsoidModel();


    osg::BoundingSphere bs = tile->getBound();
    double max_range = 1e10;
    double radius = bs.radius();
    double min_range = radius * _mapConfig.getMinTileRangeFactor();

    //Set the skirt height of the heightfield
    hf->setSkirtHeight(radius * _mapConfig.getSkirtRatio());

    if (!isCube)
    {
        //TODO:  Work on cluster culling computation for cube faces
        osg::ClusterCullingCallback* ccc = createClusterCullingCallback(tile, ellipsoid);
        tile->setCullCallback( ccc );
    }

    // see if we need to keep subdividing:
    osg::PagedLOD* plod = new osg::PagedLOD();
    plod->setCenter( bs.center() );
    plod->addChild( tile, min_range, max_range );
    plod->setFileName( 1, createURI( key ) );
    plod->setRange( 1, 0.0, min_range );

    osg::notify(osg::INFO) << "[osgEarth::GeocentricMap::createQuadrant] End" << std::endl;

    return plod;
}

osg::CoordinateSystemNode*
GeocentricMap::createCoordinateSystemNode() const
{
    return getProfile()->getSRS()->createCoordinateSystemNode();
}

osg::ClusterCullingCallback*
GeocentricMap::createClusterCullingCallback(osgTerrain::TerrainTile* tile, osg::EllipsoidModel* et)
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

