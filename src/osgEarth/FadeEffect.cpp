/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/CullingUtils>

using namespace osgEarth;

//--------------------------------------------------------------------

FadeOptions::FadeOptions(const Config& conf) :
_duration      ( 1.0f ),
_maxRange      ( FLT_MAX ),
_attenDist     ( 1000.0f )
{
    conf.getIfSet( "duration",             _duration );
    conf.getIfSet( "max_range",            _maxRange );
    conf.getIfSet( "attenuation_distance", _attenDist );
}

Config
FadeOptions::getConfig() const
{
    Config conf("fading");
    conf.addIfSet( "duration",             _duration );
    conf.addIfSet( "max_range",            _maxRange );
    conf.addIfSet( "attenuation_distance", _attenDist );
    return conf;
}

//--------------------------------------------------------------------

namespace
{
    const char* FadeEffectVertexShader =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"

        "uniform float oe_fadeeffect_duration; \n"
        "uniform float oe_fadeeffect_startTime; \n"
        "uniform float oe_fadeeffect_maxRange; \n"
        "uniform float oe_fadeeffect_attenDist; \n"
        "uniform float osg_FrameTime; \n"

        "varying float oe_fadeeffect_opacity; \n"

        "void oe_vertFadeEffect(inout vec4 VertexView) \n"
        "{ \n"
        "    float t = (osg_FrameTime-oe_fadeeffect_startTime)/oe_fadeeffect_duration; \n"
        "    float r = (oe_fadeeffect_maxRange - (-VertexView.z))/oe_fadeeffect_attenDist; \n"
        "    oe_fadeeffect_opacity = clamp(t, 0.0, 1.0) * clamp(r, 0.0, 1.0); \n"
        "} \n";

    const char* FadeEffectFragmentShader = 
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"

        "varying float oe_fadeeffect_opacity; \n"

        "void oe_fragFadeEffect( inout vec4 color ) \n"
        "{ \n"
        "    color.a *= oe_fadeeffect_opacity; \n"
        "} \n";
}

//--------------------------------------------------------------------

osg::Uniform*
FadeEffect::createStartTimeUniform()
{
    return new osg::Uniform( osg::Uniform::FLOAT, "oe_fadeeffect_startTime" );
}

FadeEffect::FadeEffect()
{
    osg::StateSet* ss = this->getOrCreateStateSet();

    if ( Registry::capabilities().supportsGLSL() )
    {
        VirtualProgram* vp = new VirtualProgram();

        vp->setFunction( "oe_vertFadeEffect", FadeEffectVertexShader,   ShaderComp::LOCATION_VERTEX_VIEW, 0.5f );
        vp->setFunction( "oe_fragFadeEffect", FadeEffectFragmentShader, ShaderComp::LOCATION_FRAGMENT_COLORING, 0.5f );

        ss->setAttributeAndModes( vp, osg::StateAttribute::ON );

        _fadeDuration = ss->getOrCreateUniform( "oe_fadeeffect_duration", osg::Uniform::FLOAT );
        _fadeDuration->set( 1.0f );

        _maxRange = ss->getOrCreateUniform( "oe_fadeeffect_maxRange", osg::Uniform::FLOAT );
        _maxRange->set( FLT_MAX );

        _attenDist = ss->getOrCreateUniform( "oe_fadeeffect_attenDist", osg::Uniform::FLOAT );
        _attenDist->set( 0.0f );
    }

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

void
FadeEffect::setMaxRange(float value)
{
    _maxRange->set( value );
}

float
FadeEffect::getMaxRange() const
{
    float value = 0.0f;
    _maxRange->get( value );
    return value;
}

void
FadeEffect::setAttenuationDistance(float value)
{
    _attenDist->set( value );
}

float
FadeEffect::getAttenuationDistance() const
{
    float value = 0.0f;
    _attenDist->get( value );
    return value;
}

//--------------------------------------------------------------------

namespace
{
    const char* FadeLODFragmentShader = 
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"

        "varying float oe_FadeLOD_opacity; \n"
        "void oe_fragFadeLOD( inout vec4 color ) \n"
        "{ \n"
        "    color.a *= oe_FadeLOD_opacity; \n"
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
    if ( Registry::capabilities().supportsGLSL() )
    {
        VirtualProgram* vp = new VirtualProgram();

        vp->setFunction(
            "oe_fragFadeLOD",
            FadeLODFragmentShader,
            osgEarth::ShaderComp::LOCATION_FRAGMENT_COLORING,
            0.5f );

        osg::StateSet* ss = getOrCreateStateSet();

        ss->setAttributeAndModes( vp, osg::StateAttribute::ON );
    }
}


void
FadeLOD::traverse( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType() == nv.CULL_VISITOR )
    {
        osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);
        PerViewData& data = _perViewData.get(cv);
        if ( !data._opacity.valid() )
        {
            data._opacity = new osg::Uniform(osg::Uniform::FLOAT, "oe_FadeLOD_opacity");
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

