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
#include "SingleKeyNodeFactory"
#include "DynamicLODScaleCallback"
#include "FileLocationCallback"
#include "TilePagedLOD"
#include "TileGroup"

#include <osgEarth/Registry>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/Progress>

using namespace osgEarth::Drivers::MPTerrainEngine;
using namespace osgEarth;
using namespace OpenThreads;

#define LC "[SingleKeyNodeFactory] "


SingleKeyNodeFactory::SingleKeyNodeFactory(const Map*                    map,
                                           TileModelFactory*             modelFactory,
                                           TileModelCompiler*            modelCompiler,
                                           TileNodeRegistry*             liveTiles,
                                           TileNodeRegistry*             deadTiles,
                                           const MPTerrainEngineOptions& options,
                                           UID                           engineUID ) :
_frame           ( map ),
_modelFactory    ( modelFactory ),
_modelCompiler   ( modelCompiler ),
_liveTiles       ( liveTiles ),
_deadTiles       ( deadTiles ),
_options         ( options ),
_engineUID       ( engineUID )
{
    //nop
}


osg::Node*
SingleKeyNodeFactory::createTile(TileModel* model, bool setupChildrenIfNecessary)
{
    // compile the model into a node:
    TileNode* tileNode = _modelCompiler->compile( model, _frame );

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

        if ( _options.rangeMode().value() == osg::LOD::DISTANCE_FROM_EYE_POINT )
        {
            //Compute the min range based on the 2D size of the tile
            GeoExtent extent = model->_tileKey.getExtent();
            GeoPoint lowerLeft(extent.getSRS(), extent.xMin(), extent.yMin(), 0.0, ALTMODE_ABSOLUTE);
            GeoPoint upperRight(extent.getSRS(), extent.xMax(), extent.yMax(), 0.0, ALTMODE_ABSOLUTE);
            osg::Vec3d ll, ur;
            lowerLeft.toWorld( ll );
            upperRight.toWorld( ur );
            double radius = (ur - ll).length() / 2.0;
            float minRange = (float)(radius * _options.minTileRangeFactor().value());

            plod->setRange( 0, minRange, FLT_MAX );
            plod->setRange( 1, 0, minRange );
            plod->setRangeMode( osg::LOD::DISTANCE_FROM_EYE_POINT );
        }
        else
        {
            plod->setRange( 0, 0.0f, _options.tilePixelSize().value() );
            plod->setRange( 1, _options.tilePixelSize().value(), FLT_MAX );
            plod->setRangeMode( osg::LOD::PIXEL_SIZE_ON_SCREEN );
        }



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
            osg::HeightField* hf =
                model->_elevationData.getHeightField();

            result->addCullCallback( HeightFieldUtils::createClusterCullingCallback(
                hf,
                tileNode->getKey().getProfile()->getSRS()->getEllipsoid(),
                *_options.verticalScale() ) );
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
        _modelFactory->createTileModel( child, _frame, model[q], progress );

        // if any one of the TileModel creations fail, we will be unable to build
        // this quadtile. So goodbye.
        if ( !model[q].valid() )
        {
            OE_INFO << LC << "Bailed on key " << key.str() << " due to a NULL model." << std::endl;
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

    // If there's a minLOD set, and we haven't reached it yet, make the tile.
    else if ( _options.minLOD().isSet() && key.getLOD() < _options.minLOD().value() )
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
            quad->addChild( createTile(model[q].get(), setupChildren) );
        }
    }

    if (progress)
        progress->stats()["compile_tilemodel_time"] += OE_STOP_TIMER(compile_tile);

    return quad.release();
}
