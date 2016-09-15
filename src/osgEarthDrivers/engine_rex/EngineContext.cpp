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
#include <osgEarth/Registry>

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
_renderBindings( renderBindings ),
_options       ( options ),
_selectionInfo ( selectionInfo ),
_tilePatchCallbacks( tilePatchCallbacks ),
_tick(0),
_tilesLastCull(0)
{
    _expirationRange2 = _options.expirationRange().get() * _options.expirationRange().get();
}

const MapFrame& EngineContext::getMapFrame()
{
    if (_frame.needsSync())
        _frame.sync();

    return _frame;
}

void
EngineContext::unloadChildrenOf(const TileNode* tile)
{
   _tilesWithChildrenToUnload.push_back( tile->getTileKey() );
   OE_INFO << LC << "Unload children of: " << tile->getTileKey().str() << "\n";
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

namespace
{
    struct Scanner : public TileNodeRegistry::ConstOperation
    {
        enum Policy {
            POLICY_FIND_ALL,
            POLICY_FIND_SOME,
            POLICY_FIND_ONE
        };

        std::vector<TileKey>& _keys;
        const osg::FrameStamp* _stamp;
        Policy _policy;

        Scanner(std::vector<TileKey>& keys, const osg::FrameStamp* stamp) : _keys(keys), _stamp(stamp)
        {
            _policy = POLICY_FIND_ALL;
        }

        void operator()(const TileNodeRegistry::TileNodeMap& tiles) const
        {
            if ( tiles.empty() ) return;
            unsigned f = _stamp->getFrameNumber(), s = tiles.size();

            switch (_policy)
            {
                case POLICY_FIND_ALL:
                {
                    for (TileNodeRegistry::TileNodeMap::const_iterator i = tiles.begin(); i != tiles.end(); ++i)
                    {
                        const TileNode* tile = i->second.tile.get();
                        if (tile->areSubTilesDormant(_stamp))
                            _keys.push_back(i->first);
                    }
                }
                break;

                case POLICY_FIND_ONE:
                {
                    const TileNode* tile = tiles.at(f%s);
                    if (tile->areSubTilesDormant(_stamp))
                    {
                        _keys.push_back(tile->getTileKey());
                    }
                }
                break;

                default:
                case POLICY_FIND_SOME:
                {
                    for(unsigned i=0; i<4; ++i) {
                        const TileNode* tile = tiles.at((f+i)%s);
                        if ( tile->areSubTilesDormant(_stamp) )
                            _keys.push_back( tile->getTileKey() );
                    }
                }
            }
        }
    };
}

void
EngineContext::endCull(osgUtil::CullVisitor* cv)
{
    if ( progress() )
    {
        double tms = 1000.0 * getElapsedCullTime();

        OE_NOTICE << "Stats:\n";
        double totalCull = getElapsedCullTime();
        OE_NOTICE << "  TOTAL TIME = " << tms << " ms ... live tiles = " << _liveTiles->size() << std::endl;
        for(ProgressCallback::Stats::const_iterator i = _progress->stats().begin(); i != _progress->stats().end(); ++i)
        { 
            if ( osgEarth::endsWith(i->first, "_count") )
            {
                OE_NOTICE << "    " << i->first << " = " << (int)i->second << std::endl;
            }
            else
            {
                OE_NOTICE << "    " << i->first << " = " << std::setprecision(5) << (1000.0*i->second) << " ms (" << 
                    std::setprecision(2) << 100.0*i->second/totalCull << "%)" << std::endl;
            }
        }
    }  

#if 0 // render bin printout
    Config c = CullDebugger().dumpRenderBin(cv->getCurrentRenderBin());
    OE_NOTICE << c.toJSON(true) << std::endl << std::endl;
#endif

    Scanner scanner(_tilesWithChildrenToUnload, cv->getFrameStamp());
    _liveTiles->run( scanner );

    if ( !_tilesWithChildrenToUnload.empty() )
    {        
        getUnloader()->unloadChildren( _tilesWithChildrenToUnload );
        _tilesWithChildrenToUnload.clear();
    }

    Registry::instance()->startActivity("REX live tiles", Stringify()<<_liveTiles->size());
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
