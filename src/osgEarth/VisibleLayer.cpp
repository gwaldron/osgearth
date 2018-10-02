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
#include <osgEarth/VisibleLayer>
#include <osgEarth/VirtualProgram>
#include <osg/BlendFunc>

using namespace osgEarth;

#define LC "[VisibleLayer] Layer \"" << getName() << "\" "

//------------------------------------------------------------------------

VisibleLayerOptions::VisibleLayerOptions() :
LayerOptions()
{
    setDefaults();
    fromConfig( _conf ); 
}

VisibleLayerOptions::VisibleLayerOptions(const ConfigOptions& co) :
LayerOptions(co)
{
    setDefaults();
    fromConfig(_conf);
}

void
VisibleLayerOptions::setDefaults()
{
    _visible.init( true );
    _opacity.init( 1.0f );
    _minRange.init( 0.0 );
    _maxRange.init( FLT_MAX );
    _blend.init( BLEND_INTERPOLATE );
}

Config
VisibleLayerOptions::getConfig() const
{
    Config conf = LayerOptions::getConfig();
    conf.set( "visible", _visible);
    conf.set( "opacity", _opacity);
    conf.set( "min_range", _minRange );
    conf.set( "max_range", _maxRange );
    conf.set( "blend", "interpolate", _blend, BLEND_INTERPOLATE );
    conf.set( "blend", "modulate", _blend, BLEND_MODULATE );
    return conf;
}

void
VisibleLayerOptions::fromConfig(const Config& conf)
{
    conf.get( "visible", _visible );
    conf.get( "opacity", _opacity);
    conf.get( "min_range", _minRange );
    conf.get( "max_range", _maxRange );
    conf.get( "blend", "interpolate", _blend, BLEND_INTERPOLATE );
    conf.get( "blend", "modulate", _blend, BLEND_MODULATE );
}

void
VisibleLayerOptions::mergeConfig(const Config& conf)
{
    LayerOptions::mergeConfig(conf);
    fromConfig(conf);
}

//........................................................................

VisibleLayer::VisibleLayer(VisibleLayerOptions* optionsPtr) :
Layer(optionsPtr ? optionsPtr : &_optionsConcrete),
_options(optionsPtr ? optionsPtr : &_optionsConcrete)
{
    //nop
}

VisibleLayer::~VisibleLayer()
{
    //nop
}

void
VisibleLayer::init()
{
    Layer::init();
}

const Status&
VisibleLayer::open()
{
    if (options().visible().isSet())
    {
        setVisible(options().visible().get());
    }

    if (options().opacity().isSet())
    {
        setOpacity(options().opacity().get());
    }

    if (options().blend().isSetTo(options().BLEND_INTERPOLATE))
    {
        getOrCreateStateSet()->setAttributeAndModes(
            new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA),
            osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

        setOpacity(options().opacity().get());
    }
    else if (options().blend().isSetTo(options().BLEND_MODULATE))
    {
        getOrCreateStateSet()->setAttributeAndModes(
            new osg::BlendFunc(GL_DST_COLOR, GL_ZERO),
            osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

        initializeBlending();
    }
    return Layer::open();
}

void
VisibleLayer::setVisible(bool value)
{
    options().visible() = value;

    // if this layer has a scene graph node, toggle its node mask
    osg::Node* node = getNode();
    if (node)
        node->setNodeMask(value? ~0 : 0);

    fireCallback(&VisibleLayerCallback::onVisibleChanged);
}

bool
VisibleLayer::getVisible() const
{
    return options().visible().get();
}

namespace
{
    const char* opacityInterpolateFS =
        "#version " GLSL_VERSION_STR "\n"
        "uniform float oe_VisibleLayer_opacity; \n"
        "void oe_VisibleLayer_setOpacity(inout vec4 color) { \n"
        "    color.a *= oe_VisibleLayer_opacity; \n"
        "} \n";

    const char* opacityModulateFS =
        "#version " GLSL_VERSION_STR "\n"
        "uniform float oe_VisibleLayer_opacity; \n"
        "void oe_VisibleLayer_setOpacity(inout vec4 color) { \n"    
        "    vec3 rgbHi = color.rgb * 2.3/oe_VisibleLayer_opacity; \n"
        "    color.rgb = mix(vec3(1), rgbHi, oe_VisibleLayer_opacity); \n"
        "} \n";
}

void
VisibleLayer::initializeBlending()
{
    if (!_opacityU.valid())
    {
        osg::StateSet* stateSet = getOrCreateStateSet();

        _opacityU = new osg::Uniform("oe_VisibleLayer_opacity", options().opacity().get());
        stateSet->addUniform(_opacityU.get());

        VirtualProgram* vp = VirtualProgram::getOrCreate(stateSet);

        if (options().blend() == VisibleLayerOptions::BLEND_MODULATE)
            vp->setFunction("oe_VisibleLayer_setOpacity", opacityModulateFS, ShaderComp::LOCATION_FRAGMENT_COLORING, 1.1f);
        else
            vp->setFunction("oe_VisibleLayer_setOpacity", opacityInterpolateFS, ShaderComp::LOCATION_FRAGMENT_COLORING, 1.1f);
    }
}

void
VisibleLayer::setOpacity(float value)
{
    options().opacity() = value;
    initializeBlending();
    _opacityU->set(value);
    fireCallback(&VisibleLayerCallback::onOpacityChanged);
}

float
VisibleLayer::getOpacity() const
{
    return options().opacity().get();
}

void
VisibleLayer::setMinVisibleRange( float minVisibleRange )
{
    options().minVisibleRange() = minVisibleRange;
    fireCallback( &VisibleLayerCallback::onVisibleRangeChanged );
}

float
VisibleLayer::getMinVisibleRange() const
{
    return options().minVisibleRange().get();
}

void
VisibleLayer::setMaxVisibleRange( float maxVisibleRange )
{
    options().maxVisibleRange() = maxVisibleRange;
    fireCallback( &VisibleLayerCallback::onVisibleRangeChanged );
}

float
VisibleLayer::getMaxVisibleRange() const
{
    return options().maxVisibleRange().get();
}

void
VisibleLayer::fireCallback(VisibleLayerCallback::MethodPtr method)
{
    for (CallbackVector::iterator i = _callbacks.begin(); i != _callbacks.end(); ++i)
    {
        VisibleLayerCallback* cb = dynamic_cast<VisibleLayerCallback*>(i->get());
        if (cb) (cb->*method)(this);
    }
}
