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
#include "LoadTileData"
#include <osgEarth/TerrainEngineNode>
#include <osg/NodeVisitor>

using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;

#define LC "[LoadTileData] "

namespace
{
    // Visitor that recalculates the sampler inheritance matrices in a graph.
    struct RecalculateMatrices : public osg::NodeVisitor
    {
        RecalculateMatrices(const RenderBindings& bindings)
            : _bindings(bindings)
        {
            setTraversalMode( TRAVERSE_ALL_CHILDREN );
        }

        void apply(osg::Group& node)
        {
            bool changed = true;

            TileNode* tilenode = dynamic_cast<TileNode*>(&node);
            if ( tilenode )
            {
                changed = tilenode->inheritState( tilenode->getParentTile(), _bindings );
            }

            if ( changed )
            {
                traverse(node);
            }
        }

        const RenderBindings& _bindings;
    };
}

//............................................................................


LoadTileData::LoadTileData(TileNode* tilenode, TileGroupFactory* context) :
_tilenode(tilenode),
_context(context)
{
    //nop
}


void
LoadTileData::invoke()
{
    osg::ref_ptr<TileNode> tilenode;
    if ( _tilenode.lock(tilenode) )
    {
        _model = _context->getEngine()->createTileModel(
            _context->getMapFrame(),
            tilenode->getTileKey(),
            0L,   //_factory->getLiveTiles(),
            0L ); // progress
    }
}

void
LoadTileData::apply()
{
    if ( _model.valid() && _model->containsNewData() )
    {
        osg::ref_ptr<TileNode> tilenode;
        if ( _tilenode.lock(tilenode) )
        {
            const RenderBindings& bindings = _context->getRenderBindings();

            osg::StateSet* stateSet = tilenode->getOrCreateStateSet();

            for(TerrainTileImageLayerModelVector::iterator i = _model->colorLayers().begin();
                i != _model->colorLayers().end();
                ++i)
            {
                TerrainTileImageLayerModel* layerModel = i->get();
                if ( layerModel && layerModel->getTexture() )
                {
                    const SamplerBinding& colorBinding = bindings.front(); // TODO

                    stateSet->setTextureAttribute(
                        colorBinding.unit(),
                        layerModel->getTexture() );

                    stateSet->addUniform(
                        new osg::Uniform(colorBinding.matrixName().c_str(),
                        osg::Matrixf::identity()));
                    
                    // TODO.... multipass textures.
                    break;                            
                }
            }

            // Update existing inheritance matrices as necessary.
            RecalculateMatrices recalc( bindings );
            tilenode->accept( recalc );

            // TODO: elevation, normal map, etc....

            // Mark as complete. TODO: per-data requests will do something different.
            tilenode->setDirty( false );

            // Delete the model immediately
            _model = 0L;
        }
    }
}
