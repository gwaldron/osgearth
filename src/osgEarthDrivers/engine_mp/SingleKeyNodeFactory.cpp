/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2015 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include "SingleKeyNodeFactory"
#include "DynamicLODScaleCallback"
#include "FileLocationCallback"
#include "TilePagedLOD"
#include "TileGroup"

#include <osgEarth/Registry>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/Progress>
#include <osgEarth/Containers>
#include <osgEarth/Horizon>

#include <osgUtil/CullVisitor>

using namespace osgEarth::Drivers::MPTerrainEngine;
using namespace osgEarth;

#define LC "[SingleKeyNodeFactory] "

namespace
{
    /**
     * Culling callback for terrain tiles.
     *
     * It performs a horizon-visibility test against the highest four corners of
     * the tile's bounding box.

     * This is an alternative to the old cluster culling callback.
     */
    struct HorizonTileCuller : public osg::NodeCallback
    {
        Horizon    _horizonPrototype;
        osg::Vec3d _points[4];

        HorizonTileCuller(const SpatialReference* srs, const osg::BoundingBox& bbox, const osg::Matrix& m)
        {
            _horizonPrototype.setEllipsoid(*srs->getEllipsoid());

            // Adjust the horizon ellipsoid based on the minimum Z value of the tile;
            // necessary because a tile that's below the ellipsoid (ocean floor, e.g.)
            // may be visible even if it doesn't pass the horizon-cone test. In such
            // cases we need a more conservative ellipsoid.
            double zMin = bbox.corner(0).z();
            if ( zMin < 0.0 )
            {
                _horizonPrototype.setEllipsoid(osg::EllipsoidModel(
                    srs->getEllipsoid()->getRadiusEquator() + zMin,
                    srs->getEllipsoid()->getRadiusPolar()   + zMin));
            }
            

            // consider the uppermost 4 points of the tile-aligned bounding box.
            // (the last four corners of the bbox are the "zmax" corners.)
            for(unsigned i=0; i<4; ++i)
            {
                _points[i] = bbox.corner(4+i) * m;
            }
        }

        void operator()(osg::Node* node, osg::NodeVisitor* nv)
        {
            // Clone the horizon object to support multiple cull threads
            // (since we call setEye with the current node visitor eye point)
            Horizon horizon(_horizonPrototype);

            // Since each terrain tile has an aboslute reference frame, 
            // there is no need to transform the eyepoint:
            horizon.setEye(nv->getViewPoint()); // * osg::computeLocalToWorld(nv->getNodePath()));
            
            for(unsigned i=0; i<4; ++i)
            {                   
                if ( !horizon.occludes(_points[i]) )
                {
                    traverse(node, nv);
                    break;
                }
            }
        }
    };
}


SingleKeyNodeFactory::SingleKeyNodeFactory(const Map*                    map,
                                           TileModelFactory*             modelFactory,
                                           TileModelCompiler*            modelCompiler,
                                           TileNodeRegistry*             liveTiles,
                                           TileNodeRegistry*             deadTiles,
                                           const MPTerrainEngineOptions& options,
                                           UID                           engineUID,
                                           TerrainTileNodeBroker*        tileNodeBroker ) :
_frame           ( map ),
_modelFactory    ( modelFactory ),
_modelCompiler   ( modelCompiler ),
_liveTiles       ( liveTiles ),
_deadTiles       ( deadTiles ),
_options         ( options ),
_engineUID       ( engineUID ),
_tileNodeBroker  ( tileNodeBroker )
{
    _debug = _options.debug() == true;
}

unsigned
SingleKeyNodeFactory::getMinimumRequiredLevel()
{
    // highest required level in the map:
    unsigned minLevel = _frame.getHighestMinLevel();

    return _options.minLOD().isSet() ?
        std::max( _options.minLOD().value(), minLevel ) :
        minLevel;
}

//Experimental: this speeds up tile loading a lot; it's sort of an alternative
// to the OSG_MAX_PAGEDLOD cache.
//#define EXPERIMENTAL_TILE_NODE_CACHE
#ifdef EXPERIMENTAL_TILE_NODE_CACHE
namespace
{
    typedef LRUCache<TileKey, osg::ref_ptr<TileNode> > TileNodeCache;
    TileNodeCache cache(true, 16384);
}
#endif

osg::Node*
SingleKeyNodeFactory::createTile(TileModel*        model,
                                 bool              setupChildrenIfNecessary,
                                 ProgressCallback* progress)
{
#ifdef EXPERIMENTAL_TILE_NODE_CACHE
    osg::ref_ptr<TileNode> tileNode;
    TileNodeCache::Record rec;
    cache.get(model->_tileKey, rec);
    if ( rec.valid() )
    {
        tileNode = rec.value().get();
    }
    else
    {
        tileNode = _modelCompiler->compile( model, _frame );
        cache.insert(model->_tileKey, tileNode);
    }
#else
    // compile the model into a node:
    TileNode* tileNode = _modelCompiler->compile(model, _frame, progress);
    tileNode->setEngineUID( _engineUID );
#endif

    // see if this tile might have children.
    bool prepareForChildren =
        setupChildrenIfNecessary &&
        model->_tileKey.getLOD() < *_options.maxLOD();

    osg::Node* result = 0L;

    if ( prepareForChildren )
    {
        osg::BoundingSphere bs = tileNode->getBound();
        TilePagedLOD* plod = new TilePagedLOD( _engineUID, _liveTiles, _deadTiles );
        plod->setCenter  ( bs.center() );
        plod->addChild   ( tileNode );
        plod->setFileName( 1, Stringify() << tileNode->getKey().str() << "." << _engineUID << ".osgearth_engine_mp_tile" );
        plod->setDebug   ( _debug );

        if ( _options.rangeMode().value() == osg::LOD::DISTANCE_FROM_EYE_POINT )
        {
            //Compute the min range based on the 2D size of the tile
            GeoExtent extent = model->_tileKey.getExtent();
            double radius = 0.0;
            
#if 0
            // Test code to use the equitorial radius so that all of the tiles at the same level
            // have the same range.  This will make the poles page in more appropriately.
            if (_frame.getMapInfo().isGeocentric())
            {
                GeoExtent equatorialExtent(
                extent.getSRS(),
                extent.west(),
                -extent.height()/2.0,
                extent.east(),
                extent.height()/2.0 );
                radius = equatorialExtent.getBoundingGeoCircle().getRadius();
            }
            else
#endif
            {
                GeoPoint lowerLeft(extent.getSRS(), extent.xMin(), extent.yMin(), 0.0, ALTMODE_ABSOLUTE);
                GeoPoint upperRight(extent.getSRS(), extent.xMax(), extent.yMax(), 0.0, ALTMODE_ABSOLUTE);
                osg::Vec3d ll, ur;
                lowerLeft.toWorld( ll );
                upperRight.toWorld( ur );
                radius = (ur - ll).length() / 2.0;
            }
          
            float minRange = (float)(radius * _options.minTileRangeFactor().value());

            plod->setRange( 0, minRange, FLT_MAX );
            plod->setRange( 1, 0, minRange );
            plod->setRangeMode( osg::LOD::DISTANCE_FROM_EYE_POINT );
        }
        else
        {
            // the *2 is because we page in 4-tile sets, not individual tiles.
            float size = 2.0f * _options.tilePixelSize().value();
            plod->setRange( 0, 0.0f, size );
            plod->setRange( 1, size, FLT_MAX );
            plod->setRangeMode( osg::LOD::PIXEL_SIZE_ON_SCREEN );
        }
        
        // Install a tile-aligned bounding box in the pager node itself so we can do
        // visibility testing before paging in subtiles.
        plod->setChildBoundingBoxAndMatrix(
            1,
            tileNode->getTerrainBoundingBox(),
            tileNode->getMatrix() );

        // DBPager will set a priority based on the ratio range/maxRange.
        // This will offset that number with a full LOD #, giving LOD precedence.
        // Experimental.
        //plod->setPriorityScale( 1, model->_tileKey.getLOD()+1 );

#if USE_FILELOCATIONCALLBACK
        osgDB::Options* options = plod->getOrCreateDBOptions();
        options->setFileLocationCallback( new FileLocationCallback() );
#endif
        
        result = plod;

        // this one rejects back-facing tiles:
        if ( _frame.getMapInfo().isGeocentric() && _options.clusterCulling() == true )
        {

#if 1
            osg::HeightField* hf =
                model->_elevationData.getHeightField();

            result->addCullCallback( HeightFieldUtils::createClusterCullingCallback(
                hf,
                tileNode->getKey().getProfile()->getSRS()->getEllipsoid(),
                *_options.verticalScale() ) );
#else
            // This works, but isn't quite as tight at the cluster culler.
            // Re-evaluate down the road.
            result->addCullCallback( new HorizonTileCuller(
                _frame.getMapInfo().getSRS(),
                tileNode->getTerrainBoundingBox(),
                tileNode->getMatrix(), tileNode->getKey() ) );
#endif
        }
    }
    else
    {
        result = tileNode;
    }

    return result;
}


osg::Node*
SingleKeyNodeFactory::createNode(const TileKey&    key, 
                                 bool              accumulate,
                                 bool              setupChildren,
                                 ProgressCallback* progress )
{
    if ( progress && progress->isCanceled() )
        return 0L;

    _frame.sync();
    
    OE_START_TIMER(create_model);

    osg::ref_ptr<TileModel> model[4];
    for(unsigned q=0; q<4; ++q)
    {
        if ( progress && progress->isCanceled() )
            return 0L;
        
        TileKey child = key.createChildKey(q);
        _modelFactory->createTileModel( child, _frame, accumulate, model[q], progress );

        // if any one of the TileModel creations fail, we will be unable to build
        // this quadtile. So goodbye.
        if ( !model[q].valid() )
        {
            OE_DEBUG << LC << "Bailed on key " << key.str() << " due to a NULL model." << std::endl;
            return 0L;
        }
    }

    if (progress)
        progress->stats()["create_tilemodel_time"] += OE_STOP_TIMER(create_model);

    bool makeTile;

    // If this is a request for a root tile, make it no matter what.
    if ( key.getLOD() == 0 || (key.getLOD()-1) == _options.firstLOD().value() )
    {
        makeTile = true;
    }

    // If there's a minimum LOD set, and we haven't reached it yet, make the tile.
    else if ( key.getLOD() <= getMinimumRequiredLevel() )
    {
        makeTile = true;
    }

    // Otherwise, only make the tile if at least one quadrant has REAL data
    // (not fallback data).
    else
    {
        makeTile = false;
        for(unsigned q=0; q<4; ++q)
        {
            if ( model[q]->hasRealData() )
            {
                makeTile = true;
                break;
            }
        }
    }
    
    if ( progress && progress->isCanceled() )
        return 0L;

    OE_START_TIMER(compile_tile);

    osg::ref_ptr<osg::Group> quad;

    if ( makeTile )
    {
        if ( _options.incrementalUpdate() == true )
        {
            quad = new TileGroup(key, _engineUID, _liveTiles.get(), _deadTiles.get());
        }
        else
        {
            quad = new osg::Group();
        }

        for( unsigned q=0; q<4; ++q )
        {
            osg::ref_ptr<osg::Node> tile = createTile(model[q].get(), setupChildren, progress);
            _tileNodeBroker->notifyOfTerrainTileNodeCreation( model[q]->_tileKey, tile.get() );
            quad->addChild( tile.get() );
        }
    }

    if (progress)
        progress->stats()["compile_tilemodel_time"] += OE_STOP_TIMER(compile_tile);

    return quad.release();
}
