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

#include <osgEarth/ProjectedMap>
#include <osgEarth/Mercator>
#include <osgEarth/TerrainTileEdgeNormalizerUpdateCallback>
#include <osgEarth/Compositing>
#include <osgEarth/ImageUtils>
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

ProjectedMap::ProjectedMap( 
    MapConfig* mapConfig,
    const osgDB::ReaderWriter::Options* global_options ) :
Map( mapConfig,global_options )
{
    //NOP
}


osg::Node*
ProjectedMap::createQuadrant( const TileKey* key )
{
    double xmin, ymin, xmax, ymax;
    key->getGeoExtent().getBounds(xmin, ymin, xmax, ymax);

    bool empty_map = _image_sources.size() == 0 && _heightfield_sources.size() == 0;

    //osg::notify(osg::NOTICE) << "DataGridProfile " << _dataProfile.xMin() << ", " << _dataProfile.yMin() << ", " << _dataProfile.xMax() << ", " << _dataProfile.yMax() << std::endl;

    GeoImageList image_tiles;

    //TODO: select/composite:
    if ( _image_sources.size() > 0 )
    {
        //Add an image from each image source
        for (unsigned int i = 0; i < _image_sources.size(); ++i)
        {
            GeoImage* image = NULL;
            if (_image_sources[i]->isKeyValid(key))
            {
                image = createGeoImage(key, _image_sources[i].get());
            }
            image_tiles.push_back(image);
        }
    }


    bool hasElevation = false;
    //Create the heightfield for the tile
    osg::ref_ptr<osg::HeightField> hf = NULL;
    if ( _elevationManager.valid() && _heightfield_sources.size() > 0 )
    {
        hf = _elevationManager->getHeightField(key, 0, 0, false);
        hasElevation = hf.valid();
    }

    //Determine if we've created any images
    unsigned int numValidImages = 0;
    for (unsigned int i = 0; i < image_tiles.size(); ++i)
    {
        if (image_tiles[i].valid()) numValidImages++;
    }

    // If we couldn't create any imagery of heightfields, bail out
    if (!hf.valid() && (numValidImages == 0))
    {
        osg::notify(osg::INFO) << "Could not create any imagery or heightfields for " << key->str() <<".  Not building tile" << std::endl;
        return NULL;
    }
   
    //Try to interpolate any missing _image_sources from parent tiles
    for (unsigned int i = 0; i < _image_sources.size(); ++i)
    {
        if (!image_tiles[i].valid())
        {
            if (_image_sources[i]->isKeyValid(key))
            {
                GeoImage* image = createValidGeoImage(_image_sources[i].get(), key);
                if (image)
                {
                    osg::notify(osg::INFO) << "[osgEarth::ProjectedMap] Using fallback image for image source " << _image_sources[i]->getName() << " for TileKey " << key->str() << std::endl;
                    image_tiles.push_back(image);
                }
                else
                {
                    osg::notify(osg::INFO) << "[osgEarth::ProjectedMap] Could not get valid image from image source " << _image_sources[i]->getName() << " for TileKey " << key->str() << std::endl;
                }
            }
        }
    }

    //Fill in missing heightfield information from parent tiles
    if (!hf.valid())
    {
        //We have no heightfield sources, 
        if (_heightfield_sources.size() == 0)
        {
            //Make any empty heightfield if no heightfield source is specified
            hf = new osg::HeightField();
            hf->allocate( 16, 16 );
            for(unsigned int i=0; i<hf->getHeightList().size(); i++ )
                hf->getHeightList()[i] = 0.0; //(double)((::rand() % 10000) - 5000);
        }
        else
        {
            //Try to get a heightfield again, but this time fallback on parent tiles
            hf = _elevationManager->getHeightField(key, 0, 0, true);
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

    //Scale the heightfield elevations from meters to degrees
    if ( getProfile()->getSRS()->isGeographic() )
    {
        scaleHeightFieldToDegrees( hf.get() );
    }

    osgTerrain::Locator* geo_locator = getProfile()->getSRS()->createLocator();
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

    if (hasElevation && _mapConfig->getNormalizeEdges())
    {
        //Attach an updatecallback to normalize the edges of TerrainTiles.
        tile->setUpdateCallback(new TerrainTileEdgeNormalizerUpdateCallback());
        tile->setDataVariance(osg::Object::DYNAMIC);
    }

    //Assign the terrain system to the TerrainTile
    if (_terrain.valid())
    {
        tile->setTerrain( _terrain.get() );
    }

    int layer = 0;
    for (unsigned int i = 0; i < image_tiles.size(); ++i)
    {
        if (image_tiles[i].valid())
        {
            double img_xmin, img_ymin, img_xmax, img_ymax;

            //Specify a new locator for the color with the coordinates of the TileKey that was actually used to create the image
            osg::ref_ptr<osgTerrain::Locator> img_locator = getProfile()->getSRS()->createLocator();

            GeoImage* geo_image = image_tiles[i].get();
            //Special case for when the map is geographic and the image is Mercator
            if ( getProfile()->getSRS()->isGeographic() && geo_image->getSRS()->isMercator() )
            {
                img_locator = new MercatorLocator( *img_locator.get(), geo_image->getExtent() );
                //Transform the mercator extents to geographic
                image_tiles[i]->getExtent().transform(image_tiles[i]->getExtent().getSRS()->getGeographicSRS()).getBounds(img_xmin, img_ymin, img_xmax, img_ymax);
            }
            else
            {
              image_tiles[i]->getExtent().getBounds(img_xmin, img_ymin, img_xmax, img_ymax);
            }
            img_locator->setTransform( getTransformFromExtents(img_xmin, img_ymin,img_xmax, img_ymax));

            osgTerrain::ImageLayer* img_layer = new osgTerrain::ImageLayer( geo_image->getImage() );
            img_layer->setLocator( img_locator.get() );

            //Turn on linear texture minification if the image is not a power of two due
            //to the fact that some drivers have issues with npot mip mapping
            if (!ImageUtils::isPowerOfTwo( geo_image->getImage() )) //image_tiles[i].first.get()))
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
    
    osg::Vec3d centroid( (xmax+xmin)/2.0, (ymax+ymin)/2.0, 0 );

    double max_range = 1e10;
    double radius = (centroid-osg::Vec3d(xmin,ymin,0)).length();
    double min_range = radius * _mapConfig->getMinTileRangeFactor();

    //Set the skirt height of the heightfield
    hf->setSkirtHeight(radius * _mapConfig->getSkirtRatio());

    // see if we need to keep subdividing:
    osg::Node* result = tile;
    if ( hasMoreLevels( key ) || empty_map )
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
ProjectedMap::createCoordinateSystemNode() const
{
    osg::CoordinateSystemNode* csn = getProfile()->getSRS()->createCoordinateSystemNode();

    // Setting the ellipsoid to NULL indicates that the CS should be interpreted 
    // as PROJECTED instead of GEOGRAPHIC.
    csn->setEllipsoidModel( NULL );

    return csn;
}

void
ProjectedMap::scaleHeightFieldToDegrees(osg::HeightField *hf)
{
    //The number of degrees in a meter at the equator
    //TODO: adjust this calculation based on the actual EllipsoidModel.
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
