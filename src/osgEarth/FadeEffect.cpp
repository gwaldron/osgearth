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
#include <osgEarth/VirtualProgram>
#include <osgUtil/CullVisitor>

using namespace osgEarth;

//--------------------------------------------------------------------

namespace
{
    char* FadeEffectVertexShader =
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

    char* FadeEffectFragmentShader = 
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

    vp->setFunction( "vertFadeEffect", FadeEffectVertexShader,   ShaderComp::LOCATION_VERTEX_POST_LIGHTING );
    vp->setFunction( "fragFadeEffect", FadeEffectFragmentShader, ShaderComp::LOCATION_FRAGMENT_PRE_LIGHTING );

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

//--------------------------------------------------------------------

namespace
{
    char* FadeLODFragmentShader = 
        "#version " GLSL_VERSION_STR "\n"
        "varying float osgearth_FadeLOD_opacity; \n"
        "void fragFadeLOD( inout vec4 color ) \n"
        "{ \n"
        "    color.a *= osgearth_FadeLOD_opacity; \n"
        "} \n";
}

//--------------------------------------------------------------------

#undef  LC
#define LC "[FadeLOD] "

FadeLOD::FadeLOD() :
_minPixelExtent( 0.0f ),
_maxPixelExtent( FLT_MAX ),
_minFadeExtent ( 0.0f ),
_maxFadeExtent ( 0.0f )
{
    VirtualProgram* vp = new VirtualProgram();

    vp->setFunction(
        "fragFadeLOD",
        FadeLODFragmentShader,
        ShaderComp::LOCATION_FRAGMENT_PRE_LIGHTING );

    osg::StateSet* ss = getOrCreateStateSet();

    ss->setAttributeAndModes( vp, osg::StateAttribute::ON );
}


void
FadeLOD::traverse( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType() == nv.CULL_VISITOR )
    {
        osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);
        PerViewData& data = _perViewData.get(cv);
        if ( !data._opacity.valid() )
        {
            data._opacity = new osg::Uniform(osg::Uniform::FLOAT, "osgearth_FadeLOD_opacity");
            data._stateSet = new osg::StateSet();
            data._stateSet->addUniform( data._opacity.get() );
        }
        
        float p = cv->clampedPixelSize(getBound()) / cv->getLODScale();
        
        float opacity;

        if ( p < _minPixelExtent )
            opacity = 0.0f;
        else if ( p < _minPixelExtent + _minFadeExtent )
            opacity = (p - _minPixelExtent) / _minFadeExtent;
        else if ( p < _maxPixelExtent - _maxFadeExtent )
            opacity = 1.0f;
        else if ( p < _maxPixelExtent )
            opacity = (_maxPixelExtent - p) / _maxFadeExtent;
        else
            opacity = 0.0f;

        data._opacity->set( opacity );

        //OE_INFO << LC << "r = " << getBound().radius() << ", p = " << p << ", o = " << opacity << std::endl;

        cv->pushStateSet( data._stateSet.get() );
        osg::Group::traverse( nv );
        cv->popStateSet();
    }
    else
    {
        osg::Group::traverse( nv );
    }
}


#if 0
void 
FadeLOD::setMinPixelExtent( float value )
{
    osg::Vec4f value;
    _params->get( value );
    value[0] = value;
    _params->set( value );
}

float 
FadeLOD::getMinPixelExtent() const
{
    osg::Vec4f value;
    _params->get( value );
    return value[0];
}

void 
FadeLOD::setMaxPixelExtent( float value )
{
    osg::Vec4f value;
    _params->get( value );
    value[1] = value;
    _params->set( value );
}

float 
FadeLOD::getMaxPixelExtent() const
{
    osg::Vec4f value;
    _params->get( value );
    return value[1];
}

void
FadeLOD::setMinFadeExtent( float value )
{
    osg::Vec4f value;
    _params->get( value );
    value[2] = value;
    _params->set( value );
}

float
FadeLOD::getMinFadeExtent() const
{
    osg::Vec4f value;
    _params->get( value );
    return value[2];
}

void
FadeLOD::setMaxFadeExtent( float value )
{
    osg::Vec4f value;
    _params->get( value );
    value[3] = value;
    _params->set( value );
}

float
FadeLOD::getMaxFadeExtent() const
{
    osg::Vec4f value;
    _params->get( value );
    return value[3];
}
#endif
