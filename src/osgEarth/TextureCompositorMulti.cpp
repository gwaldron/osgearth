/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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

#include <osgEarth/TextureCompositorMulti>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osg/Texture3D>
#include <vector>

using namespace osgEarth;

#define LC "[TextureCompositorMultiTexture] "

//------------------------------------------------------------------------

// Records the information about a layer's texture space
struct LayerTexRegion
{
    LayerTexRegion() :
        _px(0), _py(0),
        _pw(256), _ph(256),
        _tx(0.0f), _ty(0.0f),
        _tw(1.0f), _th(1.0f),
        _xoffset(0.0f), _yoffset(0.0f),
        _xscale(1.0f), _yscale(1.0f)
    {
        //nop
    }

    // pixel coordinates of layer in the composite image:
    int _px, _py, _pw, _ph;

    // texture coordinates of layer in the composite image:
    float _tx, _ty, _tw, _th;

    // texture scale and offset for this region:
    float _xoffset, _yoffset, _xscale, _yscale;
};
typedef std::vector<LayerTexRegion> LayerTexRegionList;

//------------------------------------------------------------------------

static char s_source_vertMain[] =

    "varying vec3 normal, lightDir, halfVector; \n"

    "void main(void) \n"
    "{ \n"
    "    gl_TexCoord[0] = gl_MultiTexCoord0; \n"
    "    gl_Position = ftransform(); \n"
    "} \n";

//------------------------------------------------------------------------

static char s_source_fragMain[] =

    "    gl_FragColor = gl_Color; \n"
    "} \n";

//------------------------------------------------------------------------

TextureCompositorMultiTexture::TextureCompositorMultiTexture( bool useGPU ) :
_useGPU( useGPU )
{
}

osg::StateSet*
TextureCompositorMultiTexture::createStateSet( const GeoImageVector& layerImages, const GeoExtent& tileExtent ) const
{
    osg::StateSet* stateSet = new osg::StateSet();
    //TODO
    return stateSet;
}

osg::Program*
TextureCompositorMultiTexture::createProgram() const
{
    osg::Program* program = new osg::Program();
    program->addShader( new osg::Shader( osg::Shader::VERTEX, s_source_vertMain ) );
    program->addShader( new osg::Shader( osg::Shader::FRAGMENT, s_source_fragMain ) );
    return program;
}
