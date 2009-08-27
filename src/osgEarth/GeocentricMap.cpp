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
#include <osgEarth/Locators>
#include <osgEarth/Cube>
#include <osgEarth/TerrainTileEdgeNormalizerUpdateCallback>
#include <osgEarth/Compositing>
#include <osgEarth/ImageUtils>
#include <osgEarth/EarthTerrainTechnique>
#include <osgEarth/MultiPassTerrainTechnique>
#include <osgEarth/FileLocationCallback>
#include <osgEarth/FindNode>
#include <osgEarth/VersionedTerrain>

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
#include <OpenThreads/Thread>
#include <sstream>
#include <string.h>
#include <stdlib.h>

using namespace osgEarth;
using namespace OpenThreads;

GeocentricMapEngine::GeocentricMapEngine( const MapEngineProperties& props ) :
MapEngine( props )
{
    //NOP   
}

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
            osg::notify(osg::NOTICE) << "Tile " << _keyStr << " is obselete" << std::endl;
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

            // synchronize the tile to the terrain:
            tile->setTerrainRevision( _terrain->getRevision() );

            //static_cast<osgTerrain::TerrainTile*>(node)->setTerrain( 0L );
            //static_cast<osgTerrain::TerrainTile*>(node)->setTerrain( _terrain.get() );
        }
        return osg::Group::addChild( node );
    }

    bool _loaded;
    std::string _keyStr;
    osg::observer_ptr<VersionedTerrain> _terrain;
};


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
                //osgTerrain::TerrainTile* tile = static_cast<osgTerrain::TerrainTile*>(node);
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


osg::Node*
GeocentricMapEngine::createQuadrant(Map* map, osgTerrain::Terrain* terrain, const TileKey* key, bool populateLayers)
{
    if ( populateLayers )
        return createPopulatedTile( map, terrain, key );
    else
        return createPlaceholderTile( map, terrain, key );
}

osg::Node*
GeocentricMapEngine::createPlaceholderTile(Map* map, osgTerrain::Terrain* terrain, const TileKey* key )
{
    ScopedReadLock lock( map->getMapDataMutex() );

    const MapLayerList& imageMapLayers = map->getImageMapLayers();
    const MapLayerList& hfMapLayers = map->getHeightFieldMapLayers();

    bool hasElevation = hfMapLayers.size() > 0;

    // Build a "placeholder" tile.
    double min_lon, min_lat, max_lon, max_lat;
    key->getGeoExtent().getBounds(min_lon, min_lat, max_lon, max_lat);

    // A locator will place the tile on the globe:
    osg::ref_ptr<GeoLocator> locator = key->getProfile()->getSRS()->createLocator(
        min_lon, min_lat, max_lon, max_lat );
    locator->setCoordinateSystemType( osgTerrain::Locator::GEOCENTRIC );

    // The empty tile:
    //osgTerrain::TerrainTile* tile = new osgTerrain::TerrainTile();
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

    //tile->setTerrainTechnique( new osgEarth::EarthTerrainTechnique );
	//tile->setTerrainTechnique( new osgEarth::MultiPassTerrainTechnique());

    // An empty heightfield as a placeholder.
    // TODO: populate by sampling the parent tile.
    osgTerrain::HeightFieldLayer* hf_layer = new osgTerrain::HeightFieldLayer();
    hf_layer->setHeightField( createEmptyHeightField( key ) );
    hf_layer->setLocator( locator.get() );
    tile->setElevationLayer( hf_layer );

    // Now generate imagery placeholders:
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

            if ( ancestorTile && !ancestorTile->isUpToDate() )
            {
                ancestorTile = 0L;
                //if ( ancestorTile->getTerrainRevision() != static_cast<VersionedTerrain*>(terrain)->getRevision() )
                //    ancestorTile = 0L; // keep looking
            }

            //if ( !ancestorTile ) {
            //    osg::notify(osg::NOTICE)
            //        << indent
            //        << "Failed to find ancestor tile (" << ancestorKey->str() << ")" << std::endl;
            //    indent = indent + "  ";
            //}
        }
    }
    
    int layer = 0;
    for( MapLayerList::const_iterator i = imageMapLayers.begin(); i != imageMapLayers.end(); i++ )
    {
        osgTerrain::ImageLayer* img_layer = 0L;

        if ( ancestorTile )
        {
            GeoLocator* newImageLocator = 0L;

            osgTerrain::ImageLayer* ancestorLayer = static_cast<osgTerrain::ImageLayer*>(ancestorTile->getColorLayer(layer));
            GeoLocator* ancestorLocator = dynamic_cast<GeoLocator*>( ancestorLayer->getLocator() );
            if ( ancestorLocator )
            {
                newImageLocator = new CroppingLocator(
                    *(locator.get()),
                    ancestorLocator->getDataExtent(),
                    key->getGeoExtent() );
            }
            else
            {
                newImageLocator = locator.get();
            }

            osg::Image* ancestorImage = ancestorLayer->getImage();

            //img_layer = new osgTerrain::ImageLayer( ancestorImage );
			img_layer = new TransparentLayer(ancestorImage, i->get());
            img_layer->setLocator( newImageLocator );
        }
        else
        {
            //osg::notify(osg::NOTICE) << "[osgEarth] Could not find ancestor tile for key " << key->str() << std::endl;
            //img_layer = new osgTerrain::ImageLayer( ImageUtils::getEmptyImage() );
			img_layer = new TransparentLayer( ImageUtils::getEmptyImage(), i->get());
            img_layer->setLocator( locator.get() );	
        }   

        tile->setColorLayer( layer++, img_layer );        
    }   

    // finish off the tile and put it under a new PLOD.
    osg::EllipsoidModel* ellipsoid = locator->getEllipsoidModel();

    osg::BoundingSphere bs = tile->getBound();
    double max_range = 1e10;
    double radius = bs.radius();
    double min_range = radius * _engineProps.getMinTileRangeFactor();

    // Set the skirt height of the heightfield
    hf_layer->getHeightField()->setSkirtHeight(radius * _engineProps.getSkirtRatio());

    // register the temporary tile with the terrain:
    tile->setTerrain( terrain );
    tile->setTerrainRevision( static_cast<VersionedTerrain*>(terrain)->getRevision() );

    TileSwitcher* switcher = new TileSwitcher( tile, key, static_cast<VersionedTerrain*>(terrain) );

    // TEMPORARY TODO FIXME
    bool isCube = dynamic_cast<CubeFaceLocator*>(locator.get()) != NULL;
    if (!isCube)
    {
        //TODO:  Work on cluster culling computation for cube faces
        osg::ClusterCullingCallback* ccc = createClusterCullingCallback(tile, ellipsoid);
        switcher->addCullCallback( ccc );
    }

    // This callback will load the actual tile data via the database pager:
    switcher->addCullCallback( new TileDataLoaderCallback( map, key ) ); //, loadGroup ) );
    //tile->addCullCallback( new TileDataLoaderCallback( map, key, loadGroup ) );

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

    return plod;
}

osg::Node*
GeocentricMapEngine::createPopulatedTile(Map* map, osgTerrain::Terrain* terrain, const TileKey* key )
{
    //osg::notify(osg::INFO) << "[osgEarth::GeocentricMap::createQuadrant] Begin " << key->str() << std::endl;
    //osg::notify(osg::INFO) << "[osgEarth::GeocentricMap::createQuadrant] Waiting for lock..." << std::endl;
    ScopedReadLock lock( map->getMapDataMutex() );
    //osg::notify(osg::INFO) << "[osgEarth::GeocentricMap::createQuadrant] Obtained Lock" << std::endl;

    double min_lon, min_lat, max_lon, max_lat;
    key->getGeoExtent().getBounds(min_lon, min_lat, max_lon, max_lat);

    GeoImageList image_tiles;

    const MapLayerList& imageMapLayers = map->getImageMapLayers();
    const MapLayerList& hfMapLayers = map->getHeightFieldMapLayers();

    //Collect the image layers
    bool empty_map = imageMapLayers.size() == 0 && hfMapLayers.size() == 0;

    //Create the images for the tile
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
    //TODO: select/composite.
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
        osg::notify(osg::INFO) << "[osgEarth::GeocentricMapEngine] Could not create any imagery or heightfields for " << key->str() <<".  Not building tile" << std::endl;
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
                osg::notify(osg::WARN) << "[osgEarth::GeocentricMap] Could not get valid heightfield for TileKey " << key->str() << std::endl;
                return NULL;
            }
            else
            {
                hasElevation = true;
            }
        }
    }

    osg::ref_ptr<GeoLocator> locator = key->getProfile()->getSRS()->createLocator(
        min_lon, min_lat, max_lon, max_lat );

    locator->setCoordinateSystemType( osgTerrain::Locator::GEOCENTRIC );

    osgTerrain::HeightFieldLayer* hf_layer = new osgTerrain::HeightFieldLayer();
    hf_layer->setLocator( locator.get() );
    hf_layer->setHeightField( hf.get() );

    //osgTerrain::TerrainTile* tile = new osgTerrain::TerrainTile();
    VersionedTile* tile = new VersionedTile();
    tile->setTileID(key->getTileId());

    //Attach an updatecallback to normalize the edges of TerrainTiles.
    if (hasElevation && _engineProps.getNormalizeEdges())
    {
        tile->setUpdateCallback(new TerrainTileEdgeNormalizerUpdateCallback());
        tile->setDataVariance(osg::Object::DYNAMIC);
    }

    tile->setLocator( locator.get() );
    //tile->setTerrainTechnique( new osgTerrain::GeometryTechnique() );
    //tile->setTerrainTechnique( new osgEarth::EarthTerrainTechnique );
	//tile->setTerrainTechnique( new osgEarth::MultiPassTerrainTechnique());
    tile->setElevationLayer( hf_layer );
    tile->setRequiresNormals( true );
    tile->setDataVariance(osg::Object::DYNAMIC);

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
            double img_min_lon, img_min_lat, img_max_lon, img_max_lat;

            //Specify a new locator for the color with the coordinates of the TileKey that was actually used to create the image
            osg::ref_ptr<GeoLocator> img_locator;
			
            GeoImage* geo_image = image_tiles[i].get();

            // Use a special locator for mercator images (instead of reprojecting)
            if ( geo_image->getSRS()->isMercator() && _engineProps.getUseMercatorLocator() )
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

            //osgTerrain::ImageLayer* img_layer = new osgTerrain::ImageLayer( geo_image->getImage() );
			osgTerrain::ImageLayer* img_layer = new TransparentLayer(geo_image->getImage(), imageMapLayers[i].get());
            img_layer->setLocator( img_locator.get());

			double upp = geo_image->getUnitsPerPixel();
			//Scale the units per pixel to degrees if the image is mercator
			if (geo_image->getSRS()->isMercator()) upp *= 1.0f/111319.0f;

			min_units_per_pixel = osg::minimum(upp, min_units_per_pixel);
            tile->setColorLayer( layer, img_layer );
            layer++;
        }
    }
    
    osg::EllipsoidModel* ellipsoid = locator->getEllipsoidModel();

	double width = key->getGeoExtent().width();	
	if (min_units_per_pixel == DBL_MAX) min_units_per_pixel = width/256.0;
	double min_range = (width / min_units_per_pixel) * _engineProps.getMinTileRangeFactor(); 

    osg::BoundingSphere bs = tile->getBound();
    double max_range = 1e10;
    double radius = bs.radius();

	//osg::notify(osg::NOTICE) << "Width= " << width << " upp=" << min_units_per_pixel << " min_range=" << min_range << std::endl;

    //Set the skirt height of the heightfield
    hf->setSkirtHeight(radius * _engineProps.getSkirtRatio());

    // TEMPORARY
    bool isCube = dynamic_cast<CubeFaceLocator*>(locator.get()) != NULL;
    if (!isCube)
    {
        //TODO:  Work on cluster culling computation for cube faces
        osg::ClusterCullingCallback* ccc = createClusterCullingCallback(tile, ellipsoid);
        tile->setCullCallback( ccc );
    }
    
    // Wait until now, when the tile is fully baked, to assign the terrain to the tile.
    // Placeholder tiles might try to locate this tile as an ancestor, and access its layers
    // and locators...so they must be intact before making this tile available via setTerrain.
    //
    // If there's already a placeholder tile registered, this will be ignored. If there isn't,
    // this will register the new tile.
    tile->setTerrain( terrain );

    if ( _engineProps.getDeferTileDataLoading() ) //&& key->getLevelOfDetail() > 1 )
    {
        // if this was a deferred load, all we need is the populated tile.
        return tile;
    }
    else
    {
        // see if we need to keep subdividing:
        osg::PagedLOD* plod = new osg::PagedLOD();
        plod->setCenter( bs.center() );
        //plod->addChild( tile, min_range, max_range );
		plod->addChild(tile, 0, min_range);
        plod->setFileName( 1, createURI( map->getId(), key ) );
        //plod->setRange( 1, 0.0, min_range );
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
GeocentricMapEngine::createClusterCullingCallback(osgTerrain::TerrainTile* tile, osg::EllipsoidModel* et)
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

