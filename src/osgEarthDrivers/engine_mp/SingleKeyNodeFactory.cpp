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

using namespace osgEarth_engine_mp;
using namespace osgEarth;
using namespace OpenThreads;

#define LC "[SingleKeyNodeFactory] "

namespace
{
    struct MyProgressCallback : public osgEarth::ProgressCallback
    {
        osg::observer_ptr<osg::PagedLOD> _plod;

        MyProgressCallback( osg::PagedLOD* plod )
            : _plod(plod) { }

        bool isCanceled() const
        {
            if ( _canceled )
                return true;

            if ( !_plod.valid() )
            {
                _canceled = true;
                OE_INFO << "CANCEL, plod = null." << std::endl;
            }
            else
            {
                osg::ref_ptr<osg::PagedLOD> plod;
                if ( _plod.lock(plod) )
                {
                    osg::ref_ptr<osg::Referenced> dbr = plod->getDatabaseRequest( 1 );
                    if ( !dbr.valid() || dbr->referenceCount() < 2 )
                    {
                        _canceled = true;
                        OE_INFO << "CANCEL, REFCOUNT = " << dbr->referenceCount() << std::endl;
                    }
                }
                else
                {
                    _canceled = true;
                    OE_INFO << "CANCEL, plod = null." << std::endl;
                }
            }

            return _canceled;
        }
    };
}


SingleKeyNodeFactory::SingleKeyNodeFactory(const Map*                    map,
                                           TileModelFactory*             modelFactory,
                                           TileModelCompiler*            modelCompiler,
                                           TileNodeRegistry*             liveTiles,
                                           TileNodeRegistry*             deadTiles,
                                           const MPTerrainEngineOptions& options,
                                           TerrainNode*                  terrain,
                                           UID                           engineUID ) :
_frame           ( map ),
_modelFactory    ( modelFactory ),
_modelCompiler   ( modelCompiler ),
_liveTiles       ( liveTiles ),
_deadTiles       ( deadTiles ),
_options         ( options ),
_terrain         ( terrain ),
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
        //Compute the min range based on the 2D size of the tile
        osg::BoundingSphere bs = tileNode->getBound();
        GeoExtent extent = model->_tileKey.getExtent();
        GeoPoint lowerLeft(extent.getSRS(), extent.xMin(), extent.yMin(), 0.0, ALTMODE_ABSOLUTE);
        GeoPoint upperRight(extent.getSRS(), extent.xMax(), extent.yMax(), 0.0, ALTMODE_ABSOLUTE);
        osg::Vec3d ll, ur;
        lowerLeft.toWorld( ll );
        upperRight.toWorld( ur );
        double radius = (ur - ll).length() / 2.0;
        float minRange = (float)(radius * _options.minTileRangeFactor().value());

        TilePagedLOD* plod = new TilePagedLOD( _engineUID, _liveTiles, _deadTiles );
        plod->setCenter  ( bs.center() );
        plod->addChild   ( tileNode );
        plod->setRange   ( 0, minRange, FLT_MAX );
        plod->setFileName( 1, Stringify() << tileNode->getKey().str() << "." << _engineUID << ".osgearth_engine_mp_tile" );
        plod->setRange   ( 1, 0, minRange );

#if USE_FILELOCATIONCALLBACK
        osgDB::Options* options = Registry::instance()->cloneOrCreateOptions();
        options->setFileLocationCallback( new FileLocationCallback() );
        plod->setDatabaseOptions( options );
#endif
        
        result = plod;
    }
    else
    {
        result = tileNode;
    }

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
    
    osg::ref_ptr<TileModel> model[4];
    for(unsigned q=0; q<4; ++q)
    {
        TileKey child = key.createChildKey(q);
        _modelFactory->createTileModel( child, _frame, model[q] );
    }

    bool subdivide =
        _options.minLOD().isSet() && 
        key.getLOD() < _options.minLOD().value();

    if ( !subdivide )
    {
        for(unsigned q=0; q<4; ++q)
        {
            if ( model[q]->hasRealData() )
            {
                subdivide = true;
                break;
            }
        }
    }

    osg::ref_ptr<osg::Group> quad;

    if ( subdivide )
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

    return quad.release();
}
