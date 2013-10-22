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
#include "SerialKeyNodeFactory"
#include "DynamicLODScaleCallback"
#include "FileLocationCallback"
#include "TilePagedLOD"

#include <osgEarth/Registry>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/Progress>
#include <osg/PagedLOD>
#include <osg/CullStack>
#include <osg/Uniform>

#include <osgEarth/MapNode>

using namespace osgEarth_engine_mp;
using namespace osgEarth;
using namespace OpenThreads;

#define LC "[SerialKeyNodeFactory] "

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


SerialKeyNodeFactory::SerialKeyNodeFactory(const Map*                    map,
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
SerialKeyNodeFactory::createTile(TileModel* model, bool setupChildrenIfNecessary)
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

        osgDB::Options* dbOptions = Registry::instance()->cloneOrCreateOptions();

        // tell the tile group it *must* subdivide if we are trying to reach a minLOD.
        bool forceSubdivide =
            _options.minLOD().isSet() && model->_tileKey.getLOD() < *_options.minLOD();

        TileGroup* plod = new TileGroup(tileNode, _engineUID, _liveTiles.get(), _deadTiles.get(), dbOptions);
        plod->setForceSubdivide( forceSubdivide );
        plod->setSubtileRange( minRange );


#if USE_FILELOCATIONCALLBACK
        dbOptions->setFileLocationCallback( new FileLocationCallback() );
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
SerialKeyNodeFactory::createRootNode( const TileKey& key )
{
    osg::ref_ptr<TileModel> model;

    _frame.sync();

    _modelFactory->createTileModel( key, _frame, model );
    return createTile( model.get(), true );
}


osg::Node*
SerialKeyNodeFactory::createNode(const TileKey&    key, 
                                 bool              setupChildren,
                                 ProgressCallback* progress )
{
    osg::ref_ptr<TileModel> model;

    if ( progress && progress->isCanceled() )
        return 0L;

    _frame.sync();

    _modelFactory->createTileModel(key, _frame, model);

    if ( progress && progress->isCanceled() )
        return 0L;

    return createTile( model.get(), setupChildren );
}
