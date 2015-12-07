/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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
#include "EngineContext"
#include "TileNodeRegistry"
#include <osgEarth/TraversalData>
#include <osgEarth/CullingUtils>

using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;

#define LC "[EngineContext] "

//#define PROFILE 1

//..............................................................


EngineContext::EngineContext(const Map*                     map,
                             TerrainEngineNode*             terrainEngine,
                             GeometryPool*                  geometryPool,
                             Loader*                        loader,
                             Unloader*                      unloader,
                             TileNodeRegistry*              liveTiles,
                             TileNodeRegistry*              deadTiles,
                             const RenderBindings&          renderBindings,
                             const RexTerrainEngineOptions& options,
                             const SelectionInfo&           selectionInfo,
                             TilePatchCallbacks&            tilePatchCallbacks) :
_frame         ( map ),
_terrainEngine ( terrainEngine ),
_geometryPool  ( geometryPool ),
_loader        ( loader ),
_unloader      ( unloader ),
_liveTiles     ( liveTiles ),
_deadTiles     ( deadTiles ),
_renderBindings( renderBindings ),
_options       ( options ),
_selectionInfo ( selectionInfo ),
_tilePatchCallbacks( tilePatchCallbacks )
{
    //NOP
}

void
EngineContext::startCull(osgUtil::CullVisitor* cv)
{
    _tick = osg::Timer::instance()->tick();
    _tilesLastCull = _liveTiles->size();

#ifdef PROFILE
    _progress = new ProgressCallback();
#endif
}

double
EngineContext::getElapsedCullTime() const
{
    osg::Timer_t now = osg::Timer::instance()->tick();
    return osg::Timer::instance()->delta_s(_tick, now);
}

namespace {
    struct UnloadDormantTiles : public TileNodeRegistry::Operation {
        const osg::FrameStamp* _fs;
        Unloader* _unloader;
        UnloadDormantTiles(const osg::FrameStamp* fs, Unloader* unloader) : _fs(fs), _unloader(unloader) { }
        void operator()(TileNodeRegistry::TileNodeMap& tiles) {
            for(TileNodeRegistry::TileNodeMap::iterator i = tiles.begin(); i != tiles.end(); ++i) {
                TileNode* tile = i->second.get();
                if ( tile->areSubTilesDormant(_fs) ) {
                    _unloader->unloadChildren(tile->getTileKey());
                }
            }
        }
    };
}

void
EngineContext::endCull(osgUtil::CullVisitor* cv)
{
    double tms = 1000.0 * getElapsedCullTime();
    int tileDelta = _liveTiles->size() - _tilesLastCull;
    if ( tileDelta != 0 )
    {
        OE_DEBUG << LC << "Live tiles = " << _liveTiles->size() << "; cull time = " << tms << " ms" << ", tile delta = " << tileDelta << "; avt = " << tms/(double)tileDelta << " ms\n";
    }

    if ( progress() )
    {
        OE_NOTICE << "Stats:\n";
        for(ProgressCallback::Stats::const_iterator i = _progress->stats().begin(); i != _progress->stats().end(); ++i)
        { 
            OE_NOTICE << "    " << i->first << " = " << i->second << std::endl;
        }
    }  

#if 0 // render bin printout
    Config c = CullDebugger().dumpRenderBin(cv->getCurrentRenderBin());
    OE_NOTICE << c.toJSON(true) << std::endl << std::endl;
#endif

    //Forceably unload all dormat tiles -- this works, but it too slow to run
    // every frame.
    //UnloadDormantTiles op(cv->getFrameStamp(), _unloader);
    //_liveTiles->run(op);
}

bool
EngineContext::maxLiveTilesExceeded() const
{
    return _liveTiles->size() > _options.expirationThreshold().get();
}

osg::Uniform*
EngineContext::getOrCreateMatrixUniform(const std::string& name, const osg::Matrixf& m)
{
    // Unique key for this uniform include the scale, the x/y bias, and the name ID.
    osg::Vec4f key(m(0,0),m(3,0),m(3,1),(float)osg::Uniform::getNameID(name));

    MatrixUniformMap::iterator i = _matrixUniforms.find(key);
    if ( i != _matrixUniforms.end() )
    {
        return i->second.get();
    }
    
    osg::Uniform* u = new osg::Uniform(name.c_str(), m);
    _matrixUniforms[key] = u;

    return u;
}

void
EngineContext::invokeTilePatchCallbacks(osgUtil::CullVisitor* cv,
                                        const TileKey&        tileKey,
                                        osg::StateSet*        tileStateSet,
                                        osg::Node*            tilePatch)
{
    for(TilePatchCallbacks::iterator i = _tilePatchCallbacks.begin();
        i != _tilePatchCallbacks.end();
        ++i)
    {
        i->get()->cull(cv, tileKey, tileStateSet, tilePatch);
    }
}
