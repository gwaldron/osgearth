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

#include <osgEarth/ProjectedTileBuilder>
#include <osgEarth/Mercator>
#include <osgEarth/TerrainTileEdgeNormalizerUpdateCallback>
#include <osgEarth/MultiImage>
#include <osg/Image>
#include <osg/Notify>
#include <osg/PagedLOD>
#include <osg/CoordinateSystemNode>
#include <osg/Version>
#include <osgDB/ReadFile>
#include <osgTerrain/Terrain>
#include <osgTerrain/TerrainTile>
#include <osgTerrain/Locator>
#include <osgTerrain/GeometryTechnique>
#include <sstream>
#include <stdlib.h>

using namespace osgEarth;

void getExtents(const TileKey* key, double &min_x, double &min_y, double &max_x, double &max_y, bool reproject_mercator)
{
    key->getGeoExtents(min_x, min_y, max_x, max_y);
    if (key->isMercator() && reproject_mercator)
    {
        Mercator::metersToLatLon(min_x, min_y, min_y, min_x);
        Mercator::metersToLatLon(max_x, max_y, max_y, max_x);
    }
}

ProjectedTileBuilder::ProjectedTileBuilder( 
    MapConfig* _map,
    const std::string& _url_template,
    const osgDB::ReaderWriter::Options* _global_options ) :
TileBuilder( _map, _url_template, _global_options )
{
    //NOP
}


osg::Node*
ProjectedTileBuilder::createQuadrant( const TileKey* key )
{
    double xmin, ymin, xmax, ymax;
    getExtents(key, xmin, ymin, xmax, ymax, getMapConfig()->getReprojectMercatorToGeodetic());

    //osg::notify(osg::NOTICE) << "DataGridProfile " << _dataProfile.xMin() << ", " << _dataProfile.yMin() << ", " << _dataProfile.xMax() << ", " << _dataProfile.yMax() << std::endl;

    ImageTileList image_tiles;

    //Create the images
    std::vector<osg::ref_ptr<osg::Image> > images;
    //TODO: select/composite:
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
            hf = heightfield_sources[0]->createHeightField(key);
            if (hf.valid()) hasElevation = true;
        }
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
 
          if (image_sources[i]->isKeyValid(key))
          {
            if (!createValidImage(image_sources[i].get(), key, image_tiles[i]))
            {
              osg::notify(osg::INFO) << "Could not get valid image from image source " << i << " for TileKey " << key->str() << std::endl;
            }
            else
            {
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
            hf = createValidHeightField(heightfield_sources[0].get(), key);
            if (!hf.valid())
            {
                osg::notify(osg::WARN) << "Could not get valid heightfield for TileKey " << key->str() << std::endl;
                return NULL;
            }
            else
            {
                hasElevation = true;
                osg::notify(osg::INFO) << "Interpolated heightfield TileKey " << key->str() << std::endl;
            }
        }
    }

    //Scale the heightfield elevations from meters to degrees
    if (_dataProfile.getProfileType() != TileGridProfile::PROJECTED)
    {
        scaleHeightFieldToDegrees(hf.get());
    }

    osgTerrain::Locator* geo_locator = new osgTerrain::Locator();

    osgTerrain::Locator::CoordinateSystemType coordinateSystem = (_dataProfile.getProfileType() == TileGridProfile::PROJECTED) ? osgTerrain::Locator::PROJECTED : osgTerrain::Locator::GEOGRAPHIC;
    geo_locator->setCoordinateSystemType( coordinateSystem );
	geo_locator->setTransform( getTransformFromExtents( xmin, ymin, xmax, ymax ) );
    
    hf->setOrigin( osg::Vec3d( xmin, ymin, 0.0 ) );
    hf->setXInterval( (xmax - xmin)/(double)(hf->getNumColumns()-1) );
    hf->setYInterval( (ymax - ymin)/(double)(hf->getNumRows()-1) );
    hf->setBorderWidth( 0 );
    hf->setSkirtHeight( 0 );

    osgTerrain::HeightFieldLayer* hf_layer = new osgTerrain::HeightFieldLayer();
    hf_layer->setLocator( geo_locator );
    hf_layer->setHeightField( hf.get() );

    osgTerrain::TerrainTile* tile = new osgTerrain::TerrainTile();
    tile->setLocator( geo_locator );
    tile->setTerrainTechnique( new osgTerrain::GeometryTechnique() );
    tile->setElevationLayer( hf_layer );
    tile->setRequiresNormals( true );
    tile->setTileID(key->getTileId());

    if (hasElevation && map->getNormalizeEdges())
    {
        //Attach an updatecallback to normalize the edges of TerrainTiles.
        tile->setUpdateCallback(new TerrainTileEdgeNormalizerUpdateCallback());
        tile->setDataVariance(osg::Object::DYNAMIC);
    }

    //Assign the terrain system to the TerrainTile
    tile->setTerrain( terrain.get() );

    int layer = 0;
    for (unsigned int i = 0; i < image_tiles.size(); ++i)
    {
        if (image_tiles[i].first.valid())
        {
            double img_xmin, img_ymin, img_xmax, img_ymax;
            getExtents(image_tiles[i].second.get(), img_xmin, img_ymin, img_xmax, img_ymax, getMapConfig()->getReprojectMercatorToGeodetic());

            //Specify a new locator for the color with the coordinates of the TileKey that was actually used to create the image
            osg::ref_ptr<osgTerrain::Locator> img_locator = new osgTerrain::Locator;
            img_locator->setCoordinateSystemType( coordinateSystem );
			img_locator->setTransform( getTransformFromExtents(img_xmin, img_ymin,img_xmax, img_ymax));

            if ( key->isMercator() && getMapConfig()->getReprojectMercatorToGeodetic())
            {
                img_locator = new MercatorLocator(*img_locator.get(), image_tiles[i].first->s(), image_tiles[i].second->getLevelOfDetail() );
            }

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
    
    osg::Vec3d centroid( (xmax+xmin)/2.0, (ymax+ymin)/2.0, 0 );

    double max_range = 1e10;
    double radius = (centroid-osg::Vec3d(xmin,ymin,0)).length();
    double min_range = radius * map->getMinTileRangeFactor();

    //Set the skirt height of the heightfield
    hf->setSkirtHeight(radius * map->getSkirtRatio());
   
    //osg::PagedLOD* plod = new osg::PagedLOD();
    //plod->setCenter( centroid );
    //plod->addChild( tile, min_range, max_range );
    //plod->setFileName( 1, createURI( key ) );
    //plod->setRange( 1, 0.0, min_range );

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

    return result;
}

osg::CoordinateSystemNode*
ProjectedTileBuilder::createCoordinateSystemNode() const
{
    osg::CoordinateSystemNode* csn = new osg::CoordinateSystemNode();

    // OSG wants a null ellipsoid for projected datasets...
    csn->setEllipsoidModel( NULL );

    _dataProfile.applyTo( csn );

    //if (_dataProfile.getProfileType() == TileGridProfile::GLOBAL_GEODETIC ||
    //    _dataProfile.getProfileType() == TileGridProfile::GLOBAL_MERCATOR)
    //{   
    //    csn->setCoordinateSystem( "+proj=eqc +lat_ts=0 +lon_0=0 +x_0=0 +y_0=0" );
    //    csn->setFormat( "PROJ4" );
    //}
    //else
    //{
    //    //TODO:  Set format
    //    csn->setCoordinateSystem( _dataProfile.srs() );
    //}

    return csn;
}

void
ProjectedTileBuilder::scaleHeightFieldToDegrees(osg::HeightField *hf)
{
    //The number of degrees in a meter at the equator
    float scale = 1.0f/111319.0f;
    if (hf)
    {
        for (unsigned int i = 0; i < hf->getHeightList().size(); ++i)
        {
            hf->getHeightList()[i] *= scale;
        }
    }
    else
    {
        osg::notify(osg::WARN) << "scaleHeightFieldToDegrees heightfield is NULL" << std::endl;
    }
}
