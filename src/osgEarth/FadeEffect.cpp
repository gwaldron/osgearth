/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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
#include <osgEarth/FadeEffect>
#include <osgEarth/ShaderComposition>

using namespace osgEarth;

//--------------------------------------------------------------------

namespace
{
    char* vertexShader =
        "#version " GLSL_VERSION_STR "\n"
        "uniform float osgearth_FadeEffect_duration; \n"
        "uniform float osgearth_FadeEffect_startTime; \n"
        "uniform float osg_FrameTime; \n"
        "varying float osgearth_FadeEffect_opacity; \n"

        "void vertFadeEffect() \n"
        "{ \n"
        "    float t = (osg_FrameTime-osgearth_FadeEffect_startTime)/osgearth_FadeEffect_duration; \n"
        "    osgearth_FadeEffect_opacity = clamp( t, 0.0, 1.0 ); \n"
        "} \n";

    char* fragmentShader = 
        "#version " GLSL_VERSION_STR "\n"
        "varying float osgearth_FadeEffect_opacity; \n"

        "void fragFadeEffect( inout vec4 color ) \n"
        "{ \n"
        "    color.a *= osgearth_FadeEffect_opacity; \n"
        "} \n";
}

//--------------------------------------------------------------------

osg::Uniform*
FadeEffect::createStartTimeUniform()
{
    return new osg::Uniform( osg::Uniform::FLOAT, "osgearth_FadeEffect_startTime" );
}

FadeEffect::FadeEffect()
{
    osg::StateSet* ss = this->getOrCreateStateSet();

    VirtualProgram* vp = new VirtualProgram();

    vp->setFunction( "vertFadeEffect", vertexShader,   ShaderComp::LOCATION_VERTEX_POST_LIGHTING );
    vp->setFunction( "fragFadeEffect", fragmentShader, ShaderComp::LOCATION_FRAGMENT_PRE_LIGHTING );

    ss->setAttributeAndModes( vp, osg::StateAttribute::ON );

    _fadeDuration = new osg::Uniform( osg::Uniform::FLOAT, "osgearth_FadeEffect_duration" );
    _fadeDuration->set( 1.0f );
    ss->addUniform( _fadeDuration );

    ss->setMode( GL_BLEND, 1 );
}

void
FadeEffect::setFadeDuration( float seconds )
{
    _fadeDuration->set(seconds);
}

float
FadeEffect::getFadeDuration() const
{
    float value = 0.0f;
    _fadeDuration->get(value);
    return value;
}
