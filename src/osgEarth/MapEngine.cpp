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

#include <osgEarth/MapEngine>
#include <osgEarth/DirectReadTileSource>
#include <osgEarth/Caching>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/Compositing>
#include <osgEarth/Registry>
#include <osgEarth/ImageUtils>
#include <osgEarth/TileSourceFactory>
#include <osgEarth/EarthTerrainTechnique>
#include <osgEarth/ElevationManager>
#include <osgEarth/TerrainTileEdgeNormalizerUpdateCallback>

#include <osg/Image>
#include <osg/Notify>
#include <osg/PagedLOD>
#include <osg/ClusterCullingCallback>
#include <osg/CoordinateSystemNode>
#include <osgFX/MultiTextureControl>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgTerrain/Terrain>
#include <osgTerrain/TerrainTile>
#include <osgTerrain/Locator>
#include <osgTerrain/GeometryTechnique>
#include <OpenThreads/ReentrantMutex>
#include <sstream>
#include <stdlib.h>

using namespace OpenThreads;
using namespace osgEarth;


/*****************************************************************************/


class TileSwitcher : public osg::Group
{
public:
    TileSwitcher( VersionedTile* tile, const TileKey* key, VersionedTerrain* terrain ) :
    _terrain( terrain ),
    _keyStr( key->str() )
    {
        _loaded = false;
        tile->setTerrainRevision( terrain->getRevision() );
        osg::Group::addChild(tile);
    }

    virtual ~TileSwitcher() {
    }

    // simply checks whether this loadgroup's tile is out of revision, and if so,
    // flags it for reloading.
    void checkTileRevision()
    {
        if (_loaded && 
            getNumChildren() > 0 &&
            _terrain.valid() && 
            static_cast<VersionedTile*>( getChild(0) )->getTerrainRevision() != _terrain->getRevision() )
        {
            //osg::notify(osg::NOTICE) << "Tile " << _keyStr << " is obselete" << std::endl;
            _loaded = false;
        }
    }

    // only called by the database pager when the POPULATED tile get merged:
    virtual bool addChild( osg::Node* node )
    {
        if ( !_loaded )
        {
            _loaded = true;
            
            // this will remove the old tile AND unregister it with the terrain:
            removeChildren( 0, getNumChildren() );

            VersionedTile* tile = static_cast<VersionedTile*>( node );

            tile->setTerrain( 0L );             // unregisters with the old one
            tile->setTerrain( _terrain.get() ); // registers with the new one
        }
        return osg::Group::addChild( node );
    }

    bool _loaded;
    std::string _keyStr;
    osg::observer_ptr<VersionedTerrain> _terrain;
};


/*****************************************************************************/


struct TileDataLoaderCallback : public osg::NodeCallback
{
    TileDataLoaderCallback( Map* map, const TileKey* key)
    {
        std::stringstream buf;
        buf << key->str() << "." << map->getId() << ".earth_tile_data";
        _filename = buf.str();
    }

    virtual void operator()( osg::Node* node, osg::NodeVisitor* nv )
    {
        if ( nv->getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
        {
            TileSwitcher* switcher = static_cast<TileSwitcher*>( node );
            switcher->checkTileRevision();

            if ( !switcher->_loaded )
            {
                VersionedTile* tile = static_cast<VersionedTile*>( switcher->getChild(0) );
                float priority = -(99.0f - (float)(tile->getTileID().level));
                nv->getDatabaseRequestHandler()->requestNodeFile(
                    _filename, switcher, priority, nv->getFrameStamp(), _databaseRequest );
            }
            else
            {
                _databaseRequest = 0L;
            }
        }
        traverse( node, nv );
    }

    std::string _filename;
    osg::ref_ptr<osg::Referenced> _databaseRequest;
};


/*****************************************************************************/

MapEngine::MapEngine()
{
    //nop
}

MapEngine::MapEngine( const MapEngineProperties& props ) :
_engineProps( props )
{
    //nop
}

MapEngine::~MapEngine()
{
    //nop
}

const MapEngineProperties& 
MapEngine::getEngineProperties() const
{
    return _engineProps;
}

std::string
MapEngine::createURI( unsigned int id, const TileKey* key )
{
    std::stringstream ss;
    ss << key->str() << "." <<id<<".earth_tile";
    return ss.str();
}

// Make a MatrixTransform suitable for use with a Locator object based on the given extents.
// Calling Locator::setTransformAsExtents doesn't work with OSG 2.6 due to the fact that the
// _inverse member isn't updated properly.  Calling Locator::setTransform works correctly.
osg::Matrixd
MapEngine::getTransformFromExtents(double minX, double minY, double maxX, double maxY) const
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
MapEngine::createNode( Map* map, VersionedTerrain* terrain, const TileKey* key, bool populateLayers )
{
    osg::ref_ptr<osg::Group> parent = new osg::Group;
    if ( !addChildren( map, terrain, parent.get(), key, populateLayers ))
    {
        parent = 0;
    }
    return parent.release();
}

GeoImage*
MapEngine::createValidGeoImage(TileSource* tileSource, const TileKey* key)
{
    //Try to create the image with the given key
    osg::ref_ptr<const TileKey> image_key = key;

    osg::ref_ptr<GeoImage> geo_image;

    while (image_key.valid())
    {
        if ( tileSource->isKeyValid(image_key.get()) )
        {
            geo_image = createGeoImage( image_key.get(), tileSource );
            if (geo_image.valid()) return geo_image.release();
        }
        image_key = image_key->createParentKey();
    }
    return 0;
}

bool
MapEngine::hasMoreLevels( Map* map, const TileKey* key )
{
    OpenThreads::ScopedReadLock lock( map->getMapDataMutex() );

    bool more_levels = false;
    int max_level = 0;

    for ( MapLayerList::const_iterator i = map->getImageMapLayers().begin(); i != map->getImageMapLayers().end(); i++ )
    {
//        if ( i->get()->maxLevel().isSet() && key->getLevelOfDetail() < i->get()->maxLevel().get() )
        if ( !i->get()->maxLevel().isSet() || key->getLevelOfDetail() < i->get()->maxLevel().get() )
        {
            more_levels = true;
            break;
        }
    }
    if ( !more_levels )
    {
        for( MapLayerList::const_iterator j = map->getHeightFieldMapLayers().begin(); j != map->getHeightFieldMapLayers().end(); j++ )
        {
//            if ( j->get()->maxLevel().isSet() && key->getLevelOfDetail() < j->get()->maxLevel().get() )
            if ( !j->get()->maxLevel().isSet() || key->getLevelOfDetail() < j->get()->maxLevel().get() )
            {
                more_levels = true;
                break;
            }
        }
    }

    return more_levels;
}

bool
MapEngine::addChildren(Map* map,
                       VersionedTerrain* terrain,
                       osg::Group* tile_parent,
                       const TileKey* key,
                       bool populateLayers )
{
    bool all_quadrants_created = false;

    osg::ref_ptr<osg::Node> q0, q1, q2, q3;

    osg::ref_ptr<TileKey> k0 = key->getSubkey(0);
    osg::ref_ptr<TileKey> k1 = key->getSubkey(1);
    osg::ref_ptr<TileKey> k2 = key->getSubkey(2);
    osg::ref_ptr<TileKey> k3 = key->getSubkey(3);

    q0 = createTile( map, terrain, k0.get(), populateLayers );
    q1 = createTile( map, terrain, k1.get(), populateLayers );
    q2 = createTile( map, terrain, k2.get(), populateLayers );
    q3 = createTile( map, terrain, k3.get(), populateLayers );

    all_quadrants_created = (q0.valid() && q1.valid() && q2.valid() && q3.valid());

    if (all_quadrants_created)
    {
        if (q0.valid()) tile_parent->addChild(q0.get());
        if (q1.valid()) tile_parent->addChild(q1.get());
        if (q2.valid()) tile_parent->addChild(q2.get());
        if (q3.valid()) tile_parent->addChild(q3.get());
    }
    else
    {
        osg::notify(osg::INFO) << "[osgEarth::MapEngine] Couldn't create all quadrants for " << key->str() << " time to stop subdividing!" << std::endl;
    }
    return all_quadrants_created;
}


GeoImage*
MapEngine::createGeoImage(const TileKey* mapKey, TileSource* source)
{
    GeoImage* result = NULL;
    const Profile* mapProfile = mapKey->getProfile();

    //If the key profile and the source profile exactly match, simply request the image from the source
    if ( mapProfile->isEquivalentTo( source->getProfile() ) )
    {
        osg::Image* image = source->createImageWrapper( mapKey );
        if ( image )
        {
            result = new GeoImage( image, mapKey->getGeoExtent() );
        }
    }

    // Otherwise, we need to process the tiles.
    else
    {
        Compositor comp;
        osg::ref_ptr<GeoImage> mosaic = comp.mosaicImages( mapKey, source );

        if ( mosaic.valid() )
        {
            // whether to use the fast-path mercator locator. If so, DO NOT reproject the imagery here.
            bool useMercatorFastPath =
                mosaic->getSRS()->isMercator() &&
                mapKey->getProfile()->getSRS()->isGeographic() &&
                _engineProps.getUseMercatorLocator();

            if ( !mosaic->getSRS()->isEquivalentTo( mapKey->getProfile()->getSRS()) && !useMercatorFastPath )
            {
                //We actually need to reproject the image.  Note:  The GeoImage::reprojection function will automatically
                //crop the image to the correct extents, so there is no need to crop after reprojection.
                result = mosaic->reproject( mapKey->getProfile()->getSRS(), &mapKey->getGeoExtent() );
            }
            else
            {
                // crop to fit the map key extents
                GeoExtent clampedMapExt = source->getProfile()->clampAndTransformExtent( mapKey->getGeoExtent() );
                if ( clampedMapExt.width() * clampedMapExt.height() > 0 )
                    result = mosaic->crop(clampedMapExt);
                else
                    result = NULL;
            }
        }
    }

    return result;
}

bool
MapEngine::isCached(Map* map, const osgEarth::TileKey *key)
{
    OpenThreads::ScopedReadLock lock( map->getMapDataMutex() );

    const Profile* mapProfile = key->getProfile();

    //Check the imagery layers
    for( MapLayerList::const_iterator i = map->getImageMapLayers().begin(); i != map->getImageMapLayers().end(); i++ )
    {
        MapLayer* layer = i->get();
        std::vector< osg::ref_ptr< const TileKey > > keys;

        if ( map->getProfile()->isEquivalentTo( layer->getTileSource()->getProfile() ) )
        {
            keys.push_back( key );
        }
        else
        {
            layer->getTileSource()->getProfile()->getIntersectingTiles( key, keys );
        }

        for (unsigned int j = 0; j < keys.size(); ++j)
        {
            if ( layer->getTileSource()->isKeyValid( keys[j].get() ) )
            {
                if ( !layer->getTileSource()->isCached( keys[j].get() ) )
                {
                    return false;
                }
            }
        }
    }

    //Check the elevation layers
    for( MapLayerList::const_iterator i = map->getHeightFieldMapLayers().begin(); i != map->getHeightFieldMapLayers().end(); i++ )
    {
        MapLayer* layer = i->get();
        std::vector< osg::ref_ptr< const TileKey > > keys;

        if ( map->getProfile()->isEquivalentTo( layer->getTileSource()->getProfile() ) )
        {
            keys.push_back( key );
        }
        else
        {
            layer->getTileSource()->getProfile()->getIntersectingTiles( key, keys );
        }

        for (unsigned int j = 0; j < keys.size(); ++j)
        {
            if ( layer->getTileSource()->isKeyValid( keys[j].get() ) )
            {
                if ( !layer->getTileSource()->isCached( keys[j].get() ) )
                {
                    return false;
                }
            }
        }
    }

    return true;
}



osg::HeightField*
MapEngine::createHeightField( Map* map, const TileKey* key, bool fallback )
{   
    // dont' need this here??
    //OpenThreads::ScopedReadLock lock( map->getMapDataMutex() );

    osg::ref_ptr< ElevationManager > em = new ElevationManager;

    for( MapLayerList::const_iterator i = map->getHeightFieldMapLayers().begin(); i != map->getHeightFieldMapLayers().end(); i++ )
    {
        em->getElevationSources().push_back( i->get()->getTileSource() );
    }
    return em->createHeightField( key, 0, 0, fallback );
}

osg::HeightField*
MapEngine::createEmptyHeightField( const TileKey* key )
{
    //Get the bounds of the key
    double minx, miny, maxx, maxy;
    key->getGeoExtent().getBounds(minx, miny, maxx, maxy);

    osg::HeightField *hf = new osg::HeightField();
    hf->allocate( 16, 16 );
    for(unsigned int i=0; i<hf->getHeightList().size(); i++ )
        hf->getHeightList()[i] = 0.0;

    hf->setOrigin( osg::Vec3d( minx, miny, 0.0 ) );
    hf->setXInterval( (maxx - minx)/(double)(hf->getNumColumns()-1) );
    hf->setYInterval( (maxy - miny)/(double)(hf->getNumRows()-1) );
    hf->setBorderWidth( 0 );
    return hf;
}

void
MapEngine::addPlaceholderImageLayers(VersionedTile* tile,
                                     VersionedTile* ancestorTile,
                                     const MapLayerList& imageMapLayers,
                                     GeoLocator* defaultLocator,
                                     const TileKey* key)
{
    if ( !ancestorTile )
        return;

    // Now if we have a valid ancestor tile, go through and make a temporary tile consisting only of
    // layers that exist in the new map layer image list as well.
    int layer = 0;
    for( unsigned int j=0; j<ancestorTile->getNumColorLayers(); j++ )
    {
        osgTerrain::ImageLayer* ancestorLayer = static_cast<osgTerrain::ImageLayer*>(ancestorTile->getColorLayer(j));

        const std::string& layerName = ancestorLayer->getName();
        for( MapLayerList::const_iterator i = imageMapLayers.begin(); i != imageMapLayers.end(); i++ )
        {
            if ( i->get()->getName() == layerName )
            {                    
                GeoLocator* newImageLocator = 0L;

                GeoLocator* ancestorLocator = dynamic_cast<GeoLocator*>( ancestorLayer->getLocator() );
                if ( ancestorLocator )
                {
                    newImageLocator = ancestorLocator->cloneAndCrop( *defaultLocator, key->getGeoExtent() );
                    //newImageLocator = new CroppingLocator(
                    //    *defaultLocator,
                    //    ancestorLocator->getDataExtent(),
                    //    key->getGeoExtent() );
                }
                else
                {
                    newImageLocator = defaultLocator;
                }

                osg::Image* ancestorImage = ancestorLayer->getImage();

                osgTerrain::ImageLayer* img_layer = new TransparentLayer(ancestorImage, i->get());
                img_layer->setLocator( newImageLocator );
                img_layer->setName( layerName );

                tile->setColorLayer( layer++, img_layer );
                break;
            }
        }
    }
}


void
MapEngine::addPlaceholderHeightfieldLayer(VersionedTile* tile,
                                          VersionedTile* ancestorTile,
                                          GeoLocator* defaultLocator,
                                          const TileKey* key,
                                          const TileKey* ancestorKey)
{
    if ( ancestorTile && ancestorKey )
    {
        osgTerrain::HeightFieldLayer* ancestorLayer = static_cast<osgTerrain::HeightFieldLayer*>(ancestorTile->getElevationLayer());
        if ( ancestorLayer && ancestorLayer->getHeightField() )
        {   
            osg::HeightField* ancestorHF = ancestorLayer->getHeightField();
            osg::HeightField* hf = new osg::HeightField();
            hf->allocate( ancestorHF->getNumColumns(), ancestorHF->getNumRows() );
            hf->setXInterval( 0.5f * ancestorHF->getXInterval() );
            hf->setYInterval( 0.5f * ancestorHF->getYInterval() );

            float hx = 0.5f * (float)ancestorHF->getNumColumns();
            float hy = 0.5f * (float)ancestorHF->getNumRows();

            float x0 = key->getGeoExtent().xMin() > ancestorKey->getGeoExtent().xMin()? hx : 0.0;
            float y0 = key->getGeoExtent().yMin() > ancestorKey->getGeoExtent().yMin()? hy : 0.0;
            
            for( float r=y0; r<y0+hy; r+=0.5f )
            {
                for( float c=x0; c<x0+hx; c+=0.5f )
                {
                    float h = HeightFieldUtils::getHeightAtPixel( ancestorHF, c, r );
                    hf->setHeight( (unsigned int)((c-x0)*2.0f), (unsigned int)((r-y0)*2.0f), h );
                }
            }

            const osg::Vec3d& anOrig = ancestorHF->getOrigin();
            osg::Vec3d orig;
            orig.x() = x0 == 0.0f? anOrig.x() : anOrig.x() + hx * ancestorHF->getXInterval();
            orig.y() = y0 == 0.0f? anOrig.y() : anOrig.y() + hy * ancestorHF->getYInterval();
            orig.z() = anOrig.z();
            hf->setOrigin( orig );

            osgTerrain::HeightFieldLayer* hfLayer = new osgTerrain::HeightFieldLayer( hf );
            hfLayer->setLocator( defaultLocator );
            tile->setElevationLayer( hfLayer );
        }
    }

    if ( !tile->getElevationLayer() )
    {
        osgTerrain::HeightFieldLayer* hfLayer = new osgTerrain::HeightFieldLayer();
        hfLayer->setHeightField( createEmptyHeightField( key ) );
        hfLayer->setLocator( defaultLocator );
        tile->setElevationLayer( hfLayer );
    }
}


osg::Node*
MapEngine::createTile( Map* map, VersionedTerrain* terrain, const TileKey* key, bool populateLayers )
{
    if ( populateLayers )
        return createPopulatedTile( map, terrain, key );
    else
        return createPlaceholderTile( map, terrain, key );
}


osg::Node*
MapEngine::createPlaceholderTile( Map* map, VersionedTerrain* terrain, const TileKey* key )
{
    ScopedReadLock lock( map->getMapDataMutex() );

    bool isProjected = map->getCoordinateSystemType() == Map::CSTYPE_PROJECTED;
    bool isPlateCarre = isProjected && map->getProfile()->getSRS()->isGeographic();
    bool isGeocentric = !isProjected;

    const MapLayerList& imageMapLayers = map->getImageMapLayers();
    const MapLayerList& hfMapLayers = map->getHeightFieldMapLayers();

    bool hasElevation = hfMapLayers.size() > 0;

    // Build a "placeholder" tile.
    double xmin, ymin, xmax, ymax;
    key->getGeoExtent().getBounds( xmin, ymin, xmax, ymax );

    // A locator will place the tile on the globe:
    osg::ref_ptr<GeoLocator> locator = key->getProfile()->getSRS()->createLocator(
        xmin, ymin, xmax, ymax, isPlateCarre );

    if ( isGeocentric )
        locator->setCoordinateSystemType( osgTerrain::Locator::GEOCENTRIC );

    // The empty tile:
    VersionedTile* tile = new VersionedTile();
    tile->setTileID( key->getTileId() );
    tile->setRequiresNormals( true );
    tile->setDataVariance(osg::Object::DYNAMIC);
    tile->setLocator( locator.get() );

    // Attach an updatecallback to normalize the edges of TerrainTiles.
    if ( hasElevation && _engineProps.getNormalizeEdges() )
    {
        tile->setUpdateCallback(new TerrainTileEdgeNormalizerUpdateCallback());
        tile->setDataVariance(osg::Object::DYNAMIC);
    }

    // Now generate imagery and elevation placeholders:
    osg::ref_ptr<const TileKey> ancestorKey = key;
    VersionedTile* ancestorTile = 0L;
    std::string indent = "";
    
    while( !ancestorTile && ancestorKey.valid() )
    {
        ancestorKey = ancestorKey->createParentKey();
        if ( ancestorKey.valid() )
        {
            osgTerrain::TileID tid = ancestorKey->getTileId();
            ancestorTile = static_cast<VersionedTerrain*>(terrain)->getVersionedTile( ancestorKey->getTileId() );
        }
    }

    // install placeholder image and heightfield layers.
    addPlaceholderImageLayers( tile, ancestorTile, imageMapLayers, locator.get(), key );
    addPlaceholderHeightfieldLayer( tile, ancestorTile, locator.get(), key, ancestorKey.get() );
    

    // calculate the switching distances:
    //osg::EllipsoidModel* ellipsoid = locator->getEllipsoidModel();
    osg::BoundingSphere bs = tile->getBound();
    double max_range = 1e10;
    double radius = bs.radius();
    double min_range = radius * _engineProps.getMinTileRangeFactor();

    // Set the skirt height of the heightfield
    osgTerrain::HeightFieldLayer* hfLayer = static_cast<osgTerrain::HeightFieldLayer*>(tile->getElevationLayer());
    hfLayer->getHeightField()->setSkirtHeight(radius * _engineProps.getSkirtRatio());
                
    // In a Plate Carre tesselation, scale the heightfield elevations from meters to degrees
    if ( isPlateCarre && hfLayer->getHeightField() )
        HeightFieldUtils::scaleHeightFieldToDegrees( hfLayer->getHeightField() );

    // install a tile switcher:
    tile->setTerrainRevision( static_cast<VersionedTerrain*>(terrain)->getRevision() );
    TileSwitcher* switcher = new TileSwitcher( tile, key, static_cast<VersionedTerrain*>(terrain) );

    // Install a cluster culler (FIXME for cube mode)
    bool isCube = map->getCoordinateSystemType() == Map::CSTYPE_GEOCENTRIC_CUBE;
    if ( isGeocentric && !isCube )
    {
        osg::ClusterCullingCallback* ccc = createClusterCullingCallback( tile, locator->getEllipsoidModel() );
        switcher->addCullCallback( ccc );
    }
     
    // Install a callback that will load the actual tile data via the pager (this must be added
    // after the cluster culler)
    switcher->addCullCallback( new TileDataLoaderCallback( map, key ) );

    // register the temporary tile with the terrain:
    tile->setTerrain( terrain );

    osg::Node* result = 0L;

    // create a PLOD so we can keep subdividing:
    osg::PagedLOD* plod = new osg::PagedLOD();
    plod->setCenter( bs.center() );
    plod->addChild( switcher, min_range, max_range );
    plod->setFileName( 1, createURI( map->getId(), key ) );
    plod->setRange( 1, 0.0, min_range );

#if USE_FILELOCATIONCALLBACK
    osgDB::Options* options = new osgDB::Options;
    options->setFileLocationCallback( new osgEarth::FileLocationCallback);
    plod->setDatabaseOptions( options );
#endif

    result = plod;

    return result;
}


osg::Node*
MapEngine::createPopulatedTile( Map* map, VersionedTerrain* terrain, const TileKey* key )
{
    ScopedReadLock lock( map->getMapDataMutex() );

    bool isProjected = map->getCoordinateSystemType() == Map::CSTYPE_PROJECTED;
    bool isPlateCarre = isProjected && map->getProfile()->getSRS()->isGeographic();
    bool isGeocentric = !isProjected;

    //double min_lon, min_lat, max_lon, max_lat;
    //key->getGeoExtent().getBounds(min_lon, min_lat, max_lon, max_lat);
    double xmin, ymin, xmax, ymax;
    key->getGeoExtent().getBounds( xmin, ymin, xmax, ymax );

    GeoImageList image_tiles;

    const MapLayerList& imageMapLayers = map->getImageMapLayers();
    const MapLayerList& hfMapLayers = map->getHeightFieldMapLayers();

    // Collect the image layers
    bool empty_map = imageMapLayers.size() == 0 && hfMapLayers.size() == 0;

    // Create the images for the tile
    for( MapLayerList::const_iterator i = imageMapLayers.begin(); i != imageMapLayers.end(); i++ )
    {
        GeoImage* image = NULL;
        TileSource* source = i->get()->getTileSource();
		//Only create images if the key is valid
        if ( source->isKeyValid( key ) )
        {
            image = createGeoImage( key, source );                
        }
        image_tiles.push_back(image);
    }

    bool hasElevation = false;

    //Create the heightfield for the tile
    osg::ref_ptr<osg::HeightField> hf;
    if ( hfMapLayers.size() > 0 )
    {
        hf = createHeightField( map, key, false );
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
        osg::notify(osg::INFO) << "[osgEarth::MapEngine] Could not create any imagery or heightfields for " << key->str() <<".  Not building tile" << std::endl;
        return NULL;
    }
   
    //Try to interpolate any missing image layers from parent tiles
    for (unsigned int i = 0; i < imageMapLayers.size(); i++ )
    {
        if (!image_tiles[i].valid())
        {
            TileSource* source = imageMapLayers[i]->getTileSource();
			GeoImage* image = NULL;
            if (source->isKeyValid(key))
            {
				//If the key was valid and we have no image, then something possibly went wrong with the image creation such as a server being busy.
                image = createValidGeoImage(source, key);
            }

			//If we still couldn't create an image, either something is really wrong or the key wasn't valid, so just create a transparent placeholder image
			if (!image)
			{
				//If the image is not valid, create an empty texture as a placeholder
				image = new GeoImage(ImageUtils::getEmptyImage(), key->getGeoExtent());
			}

			//Assign the new image to the proper place in the list
			image_tiles[i] = image;
        }
    }

    //Fill in missing heightfield information from parent tiles
    if (!hf.valid())
    {
        //We have no heightfield sources, 
        if ( hfMapLayers.size() == 0 )
        {
            hf = createEmptyHeightField( key );
        }
        else
        {
            //Try to get a heightfield again, but this time fallback on parent tiles
            hf = createHeightField( map, key, true );
            if (!hf.valid())
            {
                osg::notify(osg::WARN) << "[osgEarth::MapEngine] Could not get valid heightfield for TileKey " << key->str() << std::endl;
                return NULL;
            }
            else
            {
                hasElevation = true;
            }
        }
    }


    // In a Plate Carre tesselation, scale the heightfield elevations from meters to degrees
    if ( isPlateCarre )
    {
        HeightFieldUtils::scaleHeightFieldToDegrees( hf.get() );
    }

    osg::ref_ptr<GeoLocator> locator = key->getProfile()->getSRS()->createLocator(
        xmin, ymin, xmax, ymax, isPlateCarre );

    if ( isGeocentric )
        locator->setCoordinateSystemType( osgTerrain::Locator::GEOCENTRIC );

    osgTerrain::HeightFieldLayer* hf_layer = new osgTerrain::HeightFieldLayer();
    hf_layer->setLocator( locator.get() );
    hf_layer->setHeightField( hf.get() );

    VersionedTile* tile = new VersionedTile();
    tile->setTileID(key->getTileId());

    tile->setLocator( locator.get() );
    tile->setElevationLayer( hf_layer );
    tile->setRequiresNormals( true );
    tile->setDataVariance(osg::Object::DYNAMIC);

    //Attach an updatecallback to normalize the edges of TerrainTiles.
    if (hasElevation && _engineProps.getNormalizeEdges())
    {
        tile->setUpdateCallback(new TerrainTileEdgeNormalizerUpdateCallback());
        tile->setDataVariance(osg::Object::DYNAMIC);
    }

    //Assign the terrain system to the TerrainTile.
    //It is very important the terrain system is set while the MapConfig's sourceMutex is locked.
    //This registers the terrain tile so that adding/removing layers are always in sync.  If you don't do this
    //you can end up with a situation where the database pager is waiting to merge a tile, then a layer is added, then
    //the tile is finally merged and is out of sync.

	double min_units_per_pixel = DBL_MAX;

    int layer = 0;
    for (unsigned int i = 0; i < image_tiles.size(); ++i)
    {
        if (image_tiles[i].valid())
        {
            double img_xmin, img_ymin, img_xmax, img_ymax;

            //Specify a new locator for the color with the coordinates of the TileKey that was actually used to create the image
            osg::ref_ptr<GeoLocator> img_locator;
			
            GeoImage* geo_image = image_tiles[i].get();

            // Use a special locator for mercator images (instead of reprojecting)
            if ( map->getProfile()->getSRS()->isGeographic() && geo_image->getSRS()->isMercator() && _engineProps.getUseMercatorLocator() )
            {
                GeoExtent geog_ext = image_tiles[i]->getExtent().transform(image_tiles[i]->getExtent().getSRS()->getGeographicSRS());
                geog_ext.getBounds( img_xmin, img_ymin, img_xmax, img_ymax );
                img_locator = key->getProfile()->getSRS()->createLocator( img_xmin, img_ymin, img_xmax, img_ymax );
                img_locator = new MercatorLocator( *img_locator.get(), geo_image->getExtent() );
            }
            else
            {
                image_tiles[i]->getExtent().getBounds( img_xmin, img_ymin, img_xmax, img_ymax );

                img_locator = key->getProfile()->getSRS()->createLocator( 
                    img_xmin, img_ymin, img_xmax, img_ymax, isPlateCarre );
            }

            if ( isGeocentric )
                img_locator->setCoordinateSystemType( osgTerrain::Locator::GEOCENTRIC );

			osgTerrain::ImageLayer* img_layer = new TransparentLayer(geo_image->getImage(), imageMapLayers[i].get());
            img_layer->setName( imageMapLayers[i]->getName() );
            img_layer->setLocator( img_locator.get());

			double upp = geo_image->getUnitsPerPixel();

			// Scale the units per pixel to degrees if the image is mercator (and the key is geo)
            if ( geo_image->getSRS()->isMercator() && key->getGeoExtent().getSRS()->isGeographic() )
                upp *= 1.0f/111319.0f;

			min_units_per_pixel = osg::minimum(upp, min_units_per_pixel);

            tile->setColorLayer( layer, img_layer );
            layer++;
        }
    }

	double width = key->getGeoExtent().width();	
	if (min_units_per_pixel == DBL_MAX) min_units_per_pixel = width/256.0;
	double min_range = (width / min_units_per_pixel) * _engineProps.getMinTileRangeFactor(); 

    osg::BoundingSphere bs = tile->getBound();
    double max_range = 1e10;
    double radius = bs.radius();

    // a skirt hides cracks when transitioning between LODs:
    hf->setSkirtHeight(radius * _engineProps.getSkirtRatio());

    // for now, cluster culling does not work for CUBE rendering
    bool isCube = map->getCoordinateSystemType() == Map::CSTYPE_GEOCENTRIC_CUBE;
    if ( isGeocentric && !isCube )
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
    tile->setTerrain( terrain );

    // Set the tile's revision to the current terrain revision
    tile->setTerrainRevision( static_cast<VersionedTerrain*>(terrain)->getRevision() );

    if ( _engineProps.getPreemptiveLOD() )
    {
        // if this was a deferred load, all we need is the populated tile.
        return tile;
    }
    else
    {
        // if we are doing immediate load, we need to house the new tile in a PLOD that will
        // queue up its subtiles.
        osg::PagedLOD* plod = new osg::PagedLOD();
        plod->setCenter( bs.center() );
		plod->addChild(tile, 0, min_range);
        plod->setFileName( 1, createURI( map->getId(), key ) );
		plod->setRangeMode( osg::LOD::PIXEL_SIZE_ON_SCREEN );
		plod->setRange(1, min_range, FLT_MAX);
        
#if USE_FILELOCATIONCALLBACK
        osgDB::Options* options = new osgDB::Options;
        options->setFileLocationCallback( new osgEarth::FileLocationCallback);
        plod->setDatabaseOptions( options );
#endif
    
        return plod;
    }
}

osg::ClusterCullingCallback*
MapEngine::createClusterCullingCallback(osgTerrain::TerrainTile* tile, osg::EllipsoidModel* et)
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