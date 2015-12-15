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
#include "MPTexture"

using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;

#define LC "[MPTexture] "

MPTexture::MPTexture() :
osg::Texture2D()
{
    //nop
}

void
MPTexture::setLayer(const ImageLayer* layer, osg::Texture* tex, int order)
{
    Passes::iterator insertionPoint = _passes.end();

    // If the layer already exists as a pass, update it
    for(Passes::iterator pass = _passes.begin(); pass != _passes.end(); ++pass)
    {
        if ( pass->_layer.get() == layer )
        {
            pass->_texture = tex;
            pass->_ownsTexture = true;
            pass->_textureMatrix = osg::Matrixf::identity();
            return;
        }
        else if ( order < pass->_order )
        {
            insertionPoint = pass;
        }            
    }

    // Layer didn't already exist; add it (in the proper order)
    Pass& pass = *_passes.insert( insertionPoint, Pass() );
    pass._layer   = layer;
    pass._texture = tex;
    pass._parentTexture = tex;
    pass._order   = order;
    pass._ownsTexture = true;
}

void
MPTexture::merge(MPTexture* rhs)
{
    if ( rhs )
    {
        for(Passes::const_iterator pass = rhs->getPasses().begin(); pass != rhs->getPasses().end(); ++pass)
        {
            setLayer( pass->_layer.get(), pass->_texture.get(), pass->_order );
        }
    }
}

void
MPTexture::inheritState(MPTexture* parent, const osg::Matrixf& scaleBias)
{
    if ( parent )
    {
        // First time around, copy the parent's data in whole, and then scale/bias
        // the texture matrix to the appropriate quadrant
        if ( _passes.empty() )
        {
            // copy it:
            _passes = parent->getPasses();

            for(Passes::iterator pass = _passes.begin(); pass != _passes.end(); ++pass)
            {
                // scale and bias the texture matrix; then the new parent texture just
                // points to the main texture until the main texture gets replaced later.
                pass->_textureMatrix.preMult( scaleBias );
                pass->_parentTexture = pass->_texture.get();
                pass->_parentTextureMatrix = pass->_textureMatrix;
                pass->_ownsTexture = false;
            }
        }

        // If this object already has data, recalculate the sub-matrixing on all
        // existing textures and parent-textures:
        else
        {
            for(Passes::const_iterator parentPass = parent->getPasses().begin(); parentPass != parent->getPasses().end(); ++parentPass)
            {
                for(Passes::iterator pass = _passes.begin(); pass != _passes.end(); ++pass)
                {
                    if ( parentPass->_layer.get() == pass->_layer.get())
                    {
                        // If the texture matrix is non-identity, that means we are inheriting from another tile.
                        // In this case, re-copy from the parent (in case it changed) and recalculate the
                        // texture matrix.
                        if ( !pass->_textureMatrix.isIdentity() )
                        {
                            pass->_texture = parentPass->_texture.get();
                            pass->_textureMatrix = parentPass->_textureMatrix;
                            pass->_textureMatrix.preMult( scaleBias );
                        }
                        
                        // Update the parent texture always.
                        if ( parentPass->_texture.valid() )
                        {
                            pass->_parentTexture = parentPass->_texture.get();
                            pass->_parentTextureMatrix = parentPass->_textureMatrix;
                            pass->_parentTextureMatrix.preMult( scaleBias );
                        }
                        else
                        {
                            pass->_parentTexture = pass->_texture.get();
                            pass->_parentTextureMatrix = pass->_textureMatrix;
                        }
                    }
                }
            }
        }
    }
}

void 
MPTexture::compileGLObjects(osg::State& state) const
{
    for(Passes::const_iterator pass = _passes.begin(); pass != _passes.end(); ++pass)
    {
        if ( pass->_texture.valid() ) //&& pass->_ownsTexture )
        {
            pass->_texture->apply(state);
        }
    }
}

void
MPTexture::resizeGLObjectBuffers(unsigned int maxSize)
{
    for(Passes::const_iterator pass = _passes.begin(); pass != _passes.end(); ++pass)
    {
        if ( pass->_texture.valid() && pass->_ownsTexture )
        {
            pass->_texture->resizeGLObjectBuffers(maxSize);
        }
    }
}

void
MPTexture::releaseGLObjects(osg::State* state) const
{
    for(Passes::const_iterator pass = _passes.begin(); pass != _passes.end(); ++pass)
    {
        // check the refcount since textures are shared across MPTexture instances.
        if ( pass->_texture.valid() && pass->_texture->referenceCount() == 1 )
        {
            pass->_texture->releaseGLObjects(state);
        }
    }
}
