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

#include <osgEarthUtil/AlphaEffect>
#include <osgEarth/Registry>
#include <osgEarth/StringUtils>
#include <osgEarth/VirtualProgram>

using namespace osgEarth;
using namespace osgEarth::Util;

namespace
{
    const char fragment[] =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"
        "uniform float oe_alphaeffect_alpha;\n"
        "void oe_alphaeffect_fragment(inout vec4 color) {\n"
        "    color = color * oe_alphaeffect_alpha;\n"
        "}\n";
}

AlphaEffect::AlphaEffect()
{
    init();
}

AlphaEffect::AlphaEffect(osg::StateSet* stateset)
{
    init();
    attach( stateset );
}

void
AlphaEffect::init()
{
    _alphaUniform = new osg::Uniform(osg::Uniform::FLOAT, "oe_alphaeffect_alpha");
    _alphaUniform->set( 1.0f );
}

AlphaEffect::~AlphaEffect()
{
    detach();
}

void
AlphaEffect::setAlpha(float value)
{
    _alphaUniform->set( value );
}

float
AlphaEffect::getAlpha() const
{
    float value;
    _alphaUniform->get(value);
    return value;
}

void
AlphaEffect::attach(osg::StateSet* stateset)
{
    detach();

    if ( stateset )
    {
        _stateset = stateset;
        VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
        vp->setFunction( "oe_alphaeffect_fragment", fragment, ShaderComp::LOCATION_FRAGMENT_COLORING, 2 );
        stateset->addUniform( _alphaUniform.get() );
    }
}

void
AlphaEffect::detach()
{
    osg::ref_ptr<osg::StateSet> stateset;
    if ( _stateset.lock(stateset) )
    {
        detach( stateset );
        _stateset = 0L;
    }
}

void
AlphaEffect::detach(osg::StateSet* stateset)
{
    if ( stateset )
    {
        stateset->removeUniform( _alphaUniform.get() );
        VirtualProgram* vp = VirtualProgram::get( stateset );
        if ( vp )
        {
            vp->removeShader( "oe_alphaeffect_fragment" );
        }
    }
}
