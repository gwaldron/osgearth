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
MPTexture::setLayer(const ImageLayer* layer, osg::Texture* tex)
{
    for(Passes::iterator pass = _passes.begin(); pass != _passes.end(); ++pass)
    {
        if ( pass->_layer.get() == layer )
        {
            pass->_texture = tex;
            pass->_ownsTexture = true;
            pass->_matrix = osg::Matrixf::identity();
            return;
        }
    }

    _passes.push_back(Pass());
    Pass& pass = _passes.back();
    pass._layer   = layer;
    pass._texture = tex;
    pass._ownsTexture = true;
}

void
MPTexture::merge(MPTexture* rhs)
{
    if ( rhs )
    {
        for(Passes::const_iterator pass = rhs->getPasses().begin(); pass != rhs->getPasses().end(); ++pass)
        {
            setLayer( pass->_layer.get(), pass->_texture.get() );
        }
    }
}

void
MPTexture::inheritState(MPTexture* parent, const osg::Matrixf& scaleBias)
{
    if ( parent )
    {
        // First time around, just copy the parent's passes and sub-matrix all textures.
        if ( _passes.empty() )
        {
            _passes = parent->getPasses();
            for(Passes::iterator pass = _passes.begin(); pass != _passes.end(); ++pass)
            {
                pass->_matrix.preMult(scaleBias);
                pass->_ownsTexture = false;
            }
        }

        // If we already have data, recalculate the sub-matrixing on existing textures.
        else
        {
            for(Passes::const_iterator i = parent->getPasses().begin(); i != parent->getPasses().end(); ++i)
            {
                const Pass& parentPass = *i;
                for(Passes::iterator j = _passes.begin(); j != _passes.end(); ++j)
                {
                    if ( parentPass._layer.get() == j->_layer.get() && !j->_ownsTexture )
                    {
                        j->_matrix = parentPass._matrix;
                        j->_matrix.preMult( scaleBias );
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
        if ( pass->_texture.valid() && pass->_ownsTexture )
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
