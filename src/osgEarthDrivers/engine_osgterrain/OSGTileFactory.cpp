/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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
#include "OSGTileFactory"
#include "TerrainNode"
#include "StreamingTerrainNode"
#include "FileLocationCallback"
#include "TransparentLayer"

#include <osgEarth/Map>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/Registry>
#include <osgEarth/ImageUtils>
#include <osgEarth/TileSource>

#include <osg/Image>
#include <osg/Notify>
#include <osg/PagedLOD>
#include <osg/ClusterCullingCallback>
#include <osg/CoordinateSystemNode>
#include <osgFX/MultiTextureControl>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgTerrain/Locator>
#include <osgTerrain/GeometryTechnique>
#include <OpenThreads/ReentrantMutex>
#include <sstream>
#include <stdlib.h>

using namespace OpenThreads;
using namespace osgEarth_engine_osgterrain;
using namespace osgEarth;
using namespace osgEarth::Drivers;

#define LC "[OSGTileFactory] "

/*****************************************************************************/

namespace
{
    //TODO: get rid of this, and move it to the CustomTerrain CULL traversal....?????
    struct PopulateStreamingTileDataCallback : public osg::NodeCallback
    {
        PopulateStreamingTileDataCallback( const MapFrame& mapf ) : _mapf(mapf) { }

        void operator()( osg::Node* node, osg::NodeVisitor* nv )
        {
            if ( nv->getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
            {
                if ( node->asGroup()->getNumChildren() > 0 )
                {
                    StreamingTile* tile = static_cast<StreamingTile*>( node->asGroup()->getChild(0) );
                    tile->servicePendingImageRequests( _mapf, nv->getFrameStamp()->getFrameNumber() );
                }
            }
            traverse( node, nv );
        }

        const MapFrame& _mapf;
    };
}

/*****************************************************************************/

OSGTileFactory::OSGTileFactory(unsigned int engineId,
                               const MapFrame& cull_thread_mapf,
                               const OSGTerrainOptions& props ) :
osg::Referenced( true ),
_engineId( engineId ),
_cull_thread_mapf( cull_thread_mapf ),
_terrainOptions( props )
{
    LoadingPolicy::Mode mode = _terrainOptions.loadingPolicy()->mode().value();
}

const OSGTerrainOptions& 
OSGTileFactory::getTerrainOptions() const
{
    return _terrainOptions;
}

std::string
OSGTileFactory::createURI( unsigned int id, const TileKey& key )
{
    std::stringstream ss;
    ss << key.str() << "." <<id<<".osgearth_osgterrain_tile";
    std::string ssStr;
    ssStr = ss.str();
    return ssStr;
}

// Make a MatrixTransform suitable for use with a Locator object based on the given extents.
// Calling Locator::setTransformAsExtents doesn't work with OSG 2.6 due to the fact that the
// _inverse member isn't updated properly.  Calling Locator::setTransform works correctly.
osg::Matrixd
OSGTileFactory::getTransformFromExtents(double minX, double minY, double maxX, double maxY) const
{
    osg::Matrixd transform;
    transform.set(
        maxX-minX, 0.0,       0.0, 0.0,
        0.0,       maxY-minY, 0.0, 0.0,
        0.0,       0.0,       1.0, 0.0,
        minX,      minY,      0.0, 1.0); 
    return transform;
}

osg::Node*
OSGTileFactory::createSubTiles( const MapFrame& mapf, TerrainNode* terrain, const TileKey& key, bool populateLayers )
{
    TileKey k0 = key.createChildKey(0);
    TileKey k1 = key.createChildKey(1);
    TileKey k2 = key.createChildKey(2);
    TileKey k3 = key.createChildKey(3);

    bool hasValidData = false;
    bool validData;

    bool fallback = false;
    osg::ref_ptr<osg::Node> q0 = createTile( mapf, terrain, k0, populateLayers, true, fallback, validData);
    if (!hasValidData && validData) hasValidData = true;

    osg::ref_ptr<osg::Node> q1 = createTile( mapf, terrain, k1, populateLayers, true, fallback, validData );
    if (!hasValidData && validData) hasValidData = true;

    osg::ref_ptr<osg::Node> q2 = createTile( mapf, terrain, k2, populateLayers, true, fallback, validData );
    if (!hasValidData && validData) hasValidData = true;

    osg::ref_ptr<osg::Node> q3 = createTile( mapf, terrain, k3, populateLayers, true, fallback, validData );
    if (!hasValidData && validData) hasValidData = true;

    if (!hasValidData)
    {
        OE_DEBUG << LC << "Couldn't create any quadrants for " << key.str() << " time to stop subdividing!" << std::endl;
        return NULL;
    }

    osg::Group* tile_parent = new osg::Group();

    fallback = true;
    //Fallback on tiles if we couldn't create any
    if (!q0.valid())
    {
        q0 = createTile( mapf, terrain, k0, populateLayers, true, fallback, validData);
    }

    if (!q1.valid())
    {
        q1 = createTile( mapf, terrain, k1, populateLayers, true, fallback, validData);
    }

    if (!q2.valid())
    {
        q2 = createTile( mapf, terrain, k2, populateLayers, true, fallback, validData);
    }

    if (!q3.valid())
    {        
        q3 = createTile( mapf, terrain, k3, populateLayers, true, fallback, validData);
    }

    tile_parent->addChild( q0.get() );
    tile_parent->addChild( q1.get() );
    tile_parent->addChild( q2.get() );
    tile_parent->addChild( q3.get() );
    return tile_parent;
}

bool
OSGTileFactory::createValidGeoImage(ImageLayer* layer,
                                    const TileKey& key,
                                    GeoImage& out_image,
                                    TileKey&  out_actualTileKey,
                                    ProgressCallback* progress)
{
    //TODO:  Redo this to just grab images from the parent TerrainTiles
    //Try to create the image with the given key
    out_actualTileKey = key;

    while (out_actualTileKey.valid())
    {
        if ( layer->isKeyValid(out_actualTileKey) )
        {
            out_image = layer->createImage( out_actualTileKey, progress );
            if ( out_image.valid() )
            {
                return true;
            }
        }
        out_actualTileKey = out_actualTileKey.createParentKey();
    }
    return false;
}

bool
OSGTileFactory::hasMoreLevels( Map* map, const TileKey& key )
{
    //Threading::ScopedReadLock lock( map->getMapDataMutex() );

    bool more_levels = false;

    ImageLayerVector imageLayers;
    map->getImageLayers( imageLayers );

    for ( ImageLayerVector::const_iterator i = imageLayers.begin(); i != imageLayers.end(); i++ )
    {
        const ImageLayerOptions& opt = i->get()->getImageLayerOptions();

        if ( !opt.maxLevel().isSet() || key.getLevelOfDetail() < (unsigned int)*opt.maxLevel() )
        {
            more_levels = true;
            break;
        }
    }
    if ( !more_levels )
    {
        ElevationLayerVector elevLayers;
        map->getElevationLayers( elevLayers );

        for( ElevationLayerVector::const_iterator j = elevLayers.begin(); j != elevLayers.end(); j++ )
        {
            const ElevationLayerOptions& opt = j->get()->getElevationLayerOptions();

            if ( !opt.maxLevel().isSet() || key.getLevelOfDetail() < (unsigned int)*opt.maxLevel() )
            //if ( !j->get()->maxLevel().isSet() || key.getLevelOfDetail() < j->get()->maxLevel().get() )
            {
                more_levels = true;
                break;
            }
        }
    }

    return more_levels;
}

osg::HeightField*
OSGTileFactory::createEmptyHeightField( const TileKey& key, unsigned numCols, unsigned numRows )
{
    return HeightFieldUtils::createReferenceHeightField( key.getExtent(), numCols, numRows );
}

void
OSGTileFactory::addPlaceholderImageLayers(Tile* tile, Tile* ancestorTile )
{
    if ( !ancestorTile )
    {
        //OE_NOTICE << "No ancestorTile for key " << key.str() << std::endl;
        return;
    }        

    // Now if we have a valid ancestor tile, go through and make a temporary tile consisting only of
    // layers that exist in the new map layer image list as well.
    //int layer = 0;
    ColorLayersByUID colorLayers;
    ancestorTile->getCustomColorLayers( colorLayers );
    tile->setCustomColorLayers( colorLayers );
}


void
OSGTileFactory::addPlaceholderHeightfieldLayer(StreamingTile* tile,
                                               StreamingTile* ancestorTile,
                                               GeoLocator*    defaultLocator,
                                               const TileKey& key,
                                               const TileKey& ancestorKey)
{
    osgTerrain::HeightFieldLayer* newHFLayer = 0L;

    if ( ancestorTile && ancestorKey.valid() )
    {
        osg::ref_ptr<osgTerrain::HeightFieldLayer> ancestorLayer;
        {
            Threading::ScopedReadLock sharedLock( ancestorTile->getTileLayersMutex() );
            ancestorLayer = dynamic_cast<osgTerrain::HeightFieldLayer*>(ancestorTile->getElevationLayer());
        }

        if ( ancestorLayer.valid() )
        {
            osg::ref_ptr<osg::HeightField> ancestorHF = ancestorLayer->getHeightField();
            if ( ancestorHF.valid() )
            {
                osg::HeightField* newHF = HeightFieldUtils::createSubSample(
                    ancestorHF.get(),
                    ancestorKey.getExtent(),
                    key.getExtent());

                newHFLayer = new osgTerrain::HeightFieldLayer( newHF );
                newHFLayer->setLocator( defaultLocator );

                // lock to set the elevation layerdata:
                {
                    Threading::ScopedWriteLock exclusiveLock( tile->getTileLayersMutex() );
                    tile->setElevationLayer( newHFLayer );                
                    tile->setElevationLOD( ancestorTile->getElevationLOD() );
                }
            }
        }
    }

    // lock the tile to write the elevation data.
    {
        Threading::ScopedWriteLock exclusiveLock( tile->getTileLayersMutex() );

        if ( !newHFLayer )
        {
            newHFLayer = new osgTerrain::HeightFieldLayer();
            newHFLayer->setHeightField( createEmptyHeightField( key, 8, 8 ) );
            newHFLayer->setLocator( defaultLocator );
            tile->setElevationLOD( -1 );
        }

        if ( newHFLayer )
        {
            tile->setElevationLayer( newHFLayer );
        }
    }
}


osgTerrain::HeightFieldLayer*
OSGTileFactory::createPlaceholderHeightfieldLayer(osg::HeightField* ancestorHF,
                                                  const TileKey&    ancestorKey,
                                                  const TileKey&    key,
                                                  GeoLocator*       keyLocator )
{
    osgTerrain::HeightFieldLayer* hfLayer = NULL;

    osg::HeightField* newHF = HeightFieldUtils::createSubSample(
        ancestorHF,
        ancestorKey.getExtent(),
        key.getExtent() );

    newHF->setSkirtHeight( ancestorHF->getSkirtHeight() / 2.0 );

    hfLayer = new osgTerrain::HeightFieldLayer( newHF );
    hfLayer->setLocator( keyLocator );

    return hfLayer;
}

osg::Node*
OSGTileFactory::createTile(const MapFrame&  mapf, 
                           TerrainNode*         terrain, 
                           const TileKey&   key, 
                           bool             populateLayers, 
                           bool             wrapInPagedLOD, 
                           bool             fallback,
                           bool&            out_validData )
{
    if ( populateLayers )
    {        
        return createPopulatedTile( mapf, terrain, key, wrapInPagedLOD, fallback, out_validData);
    }
    else
    {
        //Placeholders always contain valid data
        out_validData = true;

        return createPlaceholderTile(
            mapf, 
            static_cast<StreamingTerrainNode*>(terrain),
            key );
    }
}



osg::Node*
OSGTileFactory::createPlaceholderTile(const MapFrame&   mapf,
                                      StreamingTerrainNode* terrain,
                                      const TileKey&    key )
{
    // Start out by finding the nearest registered ancestor tile, since the placeholder is
    // going to be based on inherited data. Note- the ancestor may not be the immediate
    // parent, b/c the parent may or may not be in the scene graph.
    TileKey ancestorKey = key.createParentKey();
    osg::ref_ptr<StreamingTile> ancestorTile;
    while( !ancestorTile.valid() && ancestorKey.valid() )
    {
        terrain->getTile( ancestorKey.getTileId(), ancestorTile );
        if ( !ancestorTile.valid() )
            ancestorKey = ancestorKey.createParentKey();
    }
    if ( !ancestorTile.valid() )
    {
        OE_WARN << LC << "cannot find ancestor tile for (" << key.str() << ")" <<std::endl;
        return 0L;
    }

    OE_DEBUG << LC << "Creating placeholder for " << key.str() << std::endl;

    const MapInfo& mapInfo = mapf.getMapInfo();

    bool hasElevation = mapf.elevationLayers().size() > 0;

    // Build a "placeholder" tile.
    double xmin, ymin, xmax, ymax;
    key.getExtent().getBounds( xmin, ymin, xmax, ymax );

    // A locator will place the tile on the globe:
    osg::ref_ptr<GeoLocator> locator = GeoLocator::createForKey( key, mapInfo );

    // The empty tile:
    StreamingTile* tile = new StreamingTile( key, locator.get(), terrain->getQuickReleaseGLObjects() );
    tile->setTerrainTechnique( terrain->cloneTechnique() );
    tile->setVerticalScale( _terrainOptions.verticalScale().value() );
    tile->setDataVariance( osg::Object::DYNAMIC );
    //tile->setLocator( locator.get() );    

    // Generate placeholder imagery and elevation layers. These "inherit" data from an
    // ancestor tile.
    {
        //Threading::ScopedReadLock parentLock( ancestorTile->getTileLayersMutex() );
        addPlaceholderImageLayers     ( tile, ancestorTile.get() );
        addPlaceholderHeightfieldLayer( tile, ancestorTile.get(), locator.get(), key, ancestorKey );
    }

    // calculate the switching distances:
    osg::BoundingSphere bs = tile->getBound();
    double max_range = 1e10;
    double radius = bs.radius();
    double min_range = radius * _terrainOptions.minTileRangeFactor().get();

    // Set the skirt height of the heightfield
    osgTerrain::HeightFieldLayer* hfLayer = static_cast<osgTerrain::HeightFieldLayer*>(tile->getElevationLayer());
    if (!hfLayer)
    {
        OE_WARN << LC << "Warning: Couldn't get hfLayer for " << key.str() << std::endl;
    }
    hfLayer->getHeightField()->setSkirtHeight(radius * _terrainOptions.heightFieldSkirtRatio().get() );

    // In a Plate Carre tesselation, scale the heightfield elevations from meters to degrees
    if ( mapInfo.isPlateCarre() && hfLayer->getHeightField() )
        HeightFieldUtils::scaleHeightFieldToDegrees( hfLayer->getHeightField() );

    bool markTileLoaded = false;

    if ( _terrainOptions.loadingPolicy()->mode().get() != LoadingPolicy::MODE_STANDARD )
    {
        markTileLoaded = true;
        tile->setHasElevationHint( hasElevation );
    }

    // install a tile switcher:
    tile->attachToTerrain( terrain );
    //tile->setTerrain( terrain );
    //terrain->registerTile( tile );

    osg::Node* result = 0L;

    // create a PLOD so we can keep subdividing:
    osg::PagedLOD* plod = new osg::PagedLOD();
    plod->setCenter( bs.center() );
    plod->addChild( tile, min_range, max_range );

    if (key.getLevelOfDetail() < getTerrainOptions().maxLOD().get())
    {
        plod->setFileName( 1, createURI( _engineId, key ) ); //map->getId(), key ) );
        plod->setRange( 1, 0.0, min_range );
    }
    else
    {
        plod->setRange( 0, 0, FLT_MAX );
    }

#if 0 //USE_FILELOCATIONCALLBACK
    osgDB::Options* options = Registry::instance()->cloneOrCreateOptions();
    options->setFileLocationCallback( new FileLocationCallback);
    plod->setDatabaseOptions( options );
#endif

    result = plod;

    // Install a callback that will load the actual tile data via the pager.
    result->addCullCallback( new PopulateStreamingTileDataCallback( _cull_thread_mapf ) );

    // Install a cluster culler (FIXME for cube mode)
    //bool isCube = map->getMapOptions().coordSysType() == MapOptions::CSTYPE_GEOCENTRIC_CUBE;
    if ( mapInfo.isGeocentric() && !mapInfo.isCube() )
    {
        osg::ClusterCullingCallback* ccc = createClusterCullingCallback( tile, locator->getEllipsoidModel() );
        result->addCullCallback( ccc );
    }     

    return result;
}

namespace
{
    struct GeoImageData
    {
        GeoImageData() : _layerUID(-1) , _imageTileKey(0,0,0,0L) { }
        GeoImage _image;
        UID      _layerUID;
        TileKey  _imageTileKey;
    };
}


osg::Node*
OSGTileFactory::createPopulatedTile(const MapFrame&  mapf, 
                                    TerrainNode*         terrain, 
                                    const TileKey&   key, 
                                    bool             wrapInPagedLOD, 
                                    bool             fallback, 
                                    bool&            validData )
{    
    const MapInfo& mapInfo = mapf.getMapInfo();
    bool isPlateCarre = !mapInfo.isGeocentric() && mapInfo.isGeographicSRS();

    typedef std::vector<GeoImageData> GeoImageDataVector;
    GeoImageDataVector image_tiles;

    // Collect the image layers
    bool empty_map = mapf.imageLayers().size() == 0 && mapf.elevationLayers().size() == 0;

    // Create the images for the tile
    for( ImageLayerVector::const_iterator i = mapf.imageLayers().begin(); i != mapf.imageLayers().end(); ++i )
    {
        ImageLayer* layer = i->get();
        GeoImageData imageData;

        // Only try to create images if the key is valid
        if ( layer->isKeyValid( key ) )
        {
            imageData._image = layer->createImage( key );
            imageData._layerUID = layer->getUID();
            imageData._imageTileKey = key;
        }

        // always push images, even it they are empty, so that the image_tiles vector is one-to-one
        // with the imageLayers() vector.
        image_tiles.push_back( imageData );
    }

    bool hasElevation = false;

    //Create the heightfield for the tile
    osg::ref_ptr<osg::HeightField> hf;
    if ( mapf.elevationLayers().size() > 0 )
    {
        mapf.getHeightField( key, false, hf, 0L);     
    }

    //If we are on the first LOD and we couldn't get a heightfield tile, just create an empty one.  Otherwise you can run into the situation
    //where you could have an inset heightfield on one hemisphere and the whole other hemisphere won't show up.
    if ( mapInfo.isGeocentric() && key.getLevelOfDetail() <= 1 && !hf.valid())
    {
        hf = createEmptyHeightField( key );
    }
    hasElevation = hf.valid();

    //Determine if we've created any images
    unsigned int numValidImages = 0;
    for (unsigned int i = 0; i < image_tiles.size(); ++i)
    {
        if (image_tiles[i]._image.valid()) numValidImages++;
    }


    //If we couldn't create any imagery or heightfields, bail out
    if (!hf.valid() && (numValidImages == 0) && !empty_map)
    {
        OE_DEBUG << LC << "Could not create any imagery or heightfields for " << key.str() <<".  Not building tile" << std::endl;
        validData = false;

        //If we're not asked to fallback on previous LOD's and we have no data, return NULL
        if (!fallback)
        {
            return NULL;
        }
    }
    else
    {
        validData = true;
    }

    //Try to interpolate any missing image layers from parent tiles
    for (unsigned int i = 0; i < mapf.imageLayers().size(); i++ )
    {
        if (!image_tiles[i]._image.valid())
        {
            if (mapf.getImageLayerAt(i)->isKeyValid(key))
            {
                //If the key was valid and we have no image, then something possibly went wrong with the image creation such as a server being busy.
                createValidGeoImage(mapf.getImageLayerAt(i), key, image_tiles[i]._image, image_tiles[i]._imageTileKey);
            }

            //If we still couldn't create an image, either something is really wrong or the key wasn't valid, so just create a transparent placeholder image
            if (!image_tiles[i]._image.valid())
            {
                //If the image is not valid, create an empty texture as a placeholder
                image_tiles[i]._image = GeoImage(ImageUtils::createEmptyImage(), key.getExtent());
                image_tiles[i]._imageTileKey = key;
            }
        }
    }

    //Fill in missing heightfield information from parent tiles
    if (!hf.valid())
    {
        //We have no heightfield sources, 
        if ( mapf.elevationLayers().size() == 0 )
        {
            hf = createEmptyHeightField( key );
        }
        else
        {
            //Try to get a heightfield again, but this time fallback on parent tiles
            if ( mapf.getHeightField( key, true, hf, 0L ) )
            {
                hasElevation = true;
            }
            else
            {
                //We couldn't get any heightfield, so just create an empty one.
                hf = createEmptyHeightField( key );
            }
        }
    }


    // In a Plate Carre tesselation, scale the heightfield elevations from meters to degrees
    if ( isPlateCarre )
    {
        HeightFieldUtils::scaleHeightFieldToDegrees( hf.get() );
    }

    osg::ref_ptr<GeoLocator> locator = GeoLocator::createForKey( key, mapInfo );
    osgTerrain::HeightFieldLayer* hf_layer = new osgTerrain::HeightFieldLayer();
    hf_layer->setLocator( locator.get() );
    hf_layer->setHeightField( hf.get() );

    bool isStreaming = 
        _terrainOptions.loadingPolicy()->mode() == LoadingPolicy::MODE_SEQUENTIAL ||
        _terrainOptions.loadingPolicy()->mode() == LoadingPolicy::MODE_PREEMPTIVE;

    Tile* tile = terrain->createTile( key, locator.get() );
    tile->setTerrainTechnique( terrain->cloneTechnique() );
    tile->setVerticalScale( _terrainOptions.verticalScale().value() );
    //tile->setLocator( locator.get() );
    tile->setElevationLayer( hf_layer );
    //tile->setRequiresNormals( true );
    tile->setDataVariance(osg::Object::DYNAMIC);

    //Assign the terrain system to the TerrainTile.
    //It is very important the terrain system is set while the MapConfig's sourceMutex is locked.
    //This registers the terrain tile so that adding/removing layers are always in sync.  If you don't do this
    //you can end up with a situation where the database pager is waiting to merge a tile, then a layer is added, then
    //the tile is finally merged and is out of sync.

    double min_units_per_pixel = DBL_MAX;

#if 0
    // create contour layer:
    if (map->getContourTransferFunction() != NULL)
    {
        osgTerrain::ContourLayer* contourLayer(new osgTerrain::ContourLayer(map->getContourTransferFunction()));

        contourLayer->setMagFilter(_terrainOptions.getContourMagFilter().value());
        contourLayer->setMinFilter(_terrainOptions.getContourMinFilter().value());
        tile->setCustomColorLayer(layer,contourLayer); //TODO: need layerUID, not layer index here -GW
        ++layer;
    }
#endif

    for (unsigned int i = 0; i < image_tiles.size(); ++i)
    {
        if (image_tiles[i]._image.valid())
        {
            const GeoImage& geo_image = image_tiles[i]._image;

            double img_xmin, img_ymin, img_xmax, img_ymax;
            geo_image.getExtent().getBounds( img_xmin, img_ymin, img_xmax, img_ymax );

            //Specify a new locator for the color with the coordinates of the TileKey that was actually used to create the image
            osg::ref_ptr<GeoLocator> img_locator = key.getProfile()->getSRS()->createLocator( 
                img_xmin, img_ymin, img_xmax, img_ymax,
                isPlateCarre );

            if ( mapInfo.isGeocentric() )
                img_locator->setCoordinateSystemType( osgTerrain::Locator::GEOCENTRIC );

            tile->setCustomColorLayer( CustomColorLayer(
                mapf.getImageLayerAt(i),
                geo_image.getImage(),
                img_locator.get(),
                key.getLevelOfDetail(),
                key) );

            double upp = geo_image.getUnitsPerPixel();

            // Scale the units per pixel to degrees if the image is mercator (and the key is geo)
            if ( geo_image.getSRS()->isMercator() && key.getExtent().getSRS()->isGeographic() )
                upp *= 1.0f/111319.0f;

            min_units_per_pixel = osg::minimum(upp, min_units_per_pixel);
        }
    }

    osg::BoundingSphere bs = tile->getBound();
    double maxRange = 1e10;
    double radius = bs.radius();
#if 0
    //Compute the min range based on the actual bounds of the tile.  This can break down if you have very high resolution
    //data with elevation variations and you can run out of memory b/c the elevation change is greater than the actual size of the tile so you end up
    //inifinitely subdividing (or at least until you run out of data or memory)
    double minRange = bs.radius() * _terrainOptions.minTileRangeFactor().value();
#else        
    //double origMinRange = bs.radius() * _options.minTileRangeFactor().value();        
    //Compute the min range based on the 2D size of the tile
    GeoExtent extent = tile->getKey().getExtent();        
    GeoPoint lowerLeft(extent.getSRS(), extent.xMin(), extent.yMin(), 0.0, ALTMODE_ABSOLUTE);
    GeoPoint upperRight(extent.getSRS(), extent.xMax(), extent.yMax(), 0.0, ALTMODE_ABSOLUTE);
    osg::Vec3d ll, ur;
    lowerLeft.toWorld( ll );
    upperRight.toWorld( ur );
    double minRange = (ur - ll).length() / 2.0 * _terrainOptions.minTileRangeFactor().value();        
#endif



    // a skirt hides cracks when transitioning between LODs:
    hf->setSkirtHeight(radius * _terrainOptions.heightFieldSkirtRatio().get() );

    // for now, cluster culling does not work for CUBE rendering
    //bool isCube = mapInfo.isCube(); //map->getMapOptions().coordSysType() == MapOptions::CSTYPE_GEOCENTRIC_CUBE;
    if ( mapInfo.isGeocentric() && !mapInfo.isCube() )
    {
        //TODO:  Work on cluster culling computation for cube faces
        osg::ClusterCullingCallback* ccc = createClusterCullingCallback(tile, locator->getEllipsoidModel() );
        tile->setCullCallback( ccc );
    }

    // Wait until now, when the tile is fully baked, to assign the terrain to the tile.
    // Placeholder tiles might try to locate this tile as an ancestor, and access its layers
    // and locators...so they must be intact before making this tile available via setTerrain.
    //
    // If there's already a placeholder tile registered, this will be ignored. If there isn't,
    // this will register the new tile.
    tile->attachToTerrain( terrain );
    //tile->setTerrain( terrain );
    //terrain->registerTile( tile );

    if ( isStreaming && key.getLevelOfDetail() > 0 )
    {
        static_cast<StreamingTile*>(tile)->setHasElevationHint( hasElevation );
    }

    osg::Node* result = 0L;

    if (wrapInPagedLOD)
    {
        // create a PLOD so we can keep subdividing:
        osg::PagedLOD* plod = new osg::PagedLOD();
        plod->setCenter( bs.center() );
        plod->addChild( tile, minRange, maxRange );

        std::string filename = createURI( _engineId, key ); //map->getId(), key );

        //Only add the next tile if it hasn't been blacklisted
        bool isBlacklisted = osgEarth::Registry::instance()->isBlacklisted( filename );
        if (!isBlacklisted && key.getLevelOfDetail() < (unsigned int)getTerrainOptions().maxLOD().value() && validData )
        {
            plod->setFileName( 1, filename  );
            plod->setRange( 1, 0.0, minRange );
        }
        else
        {
            plod->setRange( 0, 0, FLT_MAX );
        }

#if USE_FILELOCATIONCALLBACK
        osgDB::Options* options = Registry::instance()->cloneOrCreateOptions();
        options->setFileLocationCallback( new FileLocationCallback() );
        plod->setDatabaseOptions( options );
#endif
        result = plod;

        if ( isStreaming )
            result->addCullCallback( new PopulateStreamingTileDataCallback( _cull_thread_mapf ) );
    }
    else
    {
        result = tile;
    }

    return result;
}

CustomColorLayerRef*
OSGTileFactory::createImageLayer(const MapInfo&    mapInfo,
                                 ImageLayer*       layer,
                                 const TileKey&    key,
                                 ProgressCallback* progress)
{
    if ( !layer )
        return 0L;

    GeoImage geoImage;

    //If the key is valid, try to get the image from the MapLayer
    bool keyValid = layer->isKeyValid( key );
    if ( keyValid )
    {
        geoImage = layer->createImage(key, progress);
    }
    else
    {
        //If the key is not valid, simply make a transparent tile
        geoImage = GeoImage(ImageUtils::createEmptyImage(), key.getExtent());
    }

    if (geoImage.valid())
    {
        osg::ref_ptr<GeoLocator> imgLocator = GeoLocator::createForKey( key, mapInfo );

        if ( mapInfo.isGeocentric() )
            imgLocator->setCoordinateSystemType( osgTerrain::Locator::GEOCENTRIC );

        return new CustomColorLayerRef( CustomColorLayer(
            layer,
            geoImage.getImage(),
            imgLocator.get(),
            key.getLevelOfDetail(),
            key) );
    }

    return NULL;
}

osgTerrain::HeightFieldLayer* 
OSGTileFactory::createHeightFieldLayer( const MapFrame& mapf, const TileKey& key, bool exactOnly )
{
    const MapInfo& mapInfo = mapf.getMapInfo();
    bool isPlateCarre = !mapInfo.isGeocentric() && mapInfo.isGeographicSRS();

    // try to create a heightfield at native res:
    osg::ref_ptr<osg::HeightField> hf;
    if ( !mapf.getHeightField( key, !exactOnly, hf, 0L ) )
    {
        if ( exactOnly )
            return NULL;
        else
            hf = createEmptyHeightField( key );
    }

    // In a Plate Carre tesselation, scale the heightfield elevations from meters to degrees
    if ( isPlateCarre )
    {
        HeightFieldUtils::scaleHeightFieldToDegrees( hf.get() );
    }

    osgTerrain::HeightFieldLayer* hfLayer = new osgTerrain::HeightFieldLayer( hf.get() );

    GeoLocator* locator = GeoLocator::createForKey( key, mapInfo );
    hfLayer->setLocator( locator );

    return hfLayer;
}

osg::ClusterCullingCallback*
OSGTileFactory::createClusterCullingCallback(Tile* tile, osg::EllipsoidModel* et)
{
    //This code is a very slightly modified version of the DestinationTile::createClusterCullingCallback in VirtualPlanetBuilder.
    osg::HeightField* grid = ((osgTerrain::HeightFieldLayer*)tile->getElevationLayer())->getHeightField();
    if (!grid) return 0;

    float verticalScale = 1.0f;
    Tile* customTile = dynamic_cast<Tile*>(tile);
    if (customTile)
    {
        verticalScale = customTile->getVerticalScale();
    }

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
            double Z = orig_Z + grid->getHeight(c,r) * verticalScale;
            double height = Z;

            et->convertLatLongHeightToXYZ(
                osg::DegreesToRadians(Y), osg::DegreesToRadians(X), Z,
                X, Y, Z);

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

    osg::ClusterCullingCallback* ccc = new osg::ClusterCullingCallback;

    ccc->set(center_position + transformed_center_normal*max_cluster_culling_height ,
        transformed_center_normal, 
        min_dot_product,
        max_cluster_culling_radius);

    return ccc;
}
