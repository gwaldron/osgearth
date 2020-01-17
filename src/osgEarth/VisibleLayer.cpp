/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
    _attenuationRange.init(0.0f);
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
    conf.set( "attenuation_range", _attenuationRange );
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
    conf.get( "attenuation_range", _attenuationRange );
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

    initializeBlending();

    if (options().minVisibleRange().isSet() || options().maxVisibleRange().isSet())
    {
        initializeMinMaxRangeOpacity();
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
    // Shader that just copies the uniform value into a stage global/output
    const char* opacityVS =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"
        "uniform float oe_VisibleLayer_opacityUniform; \n"
        "out float oe_layer_opacity; \n"
        "void oe_VisibleLayer_initOpacity(inout vec4 vertex) \n"
        "{ \n"
        "    oe_layer_opacity = oe_VisibleLayer_opacityUniform; \n"
        "} \n";

    // Shader that incorporates range-based opacity (min/max range with attenuation)
    const char* rangeOpacityVS =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"
        "#pragma import_defines(OE_DISABLE_RANGE_OPACITY) \n"
        "uniform vec3 oe_VisibleLayer_ranges; \n"
        "float oe_layer_opacity; \n"
        
        "void oe_VisibleLayer_applyMinMaxRange(inout vec4 vertexView) \n"
        "{ \n"
        "  #ifndef OE_DISABLE_RANGE_OPACITY \n"
        "    float minRange = oe_VisibleLayer_ranges[0]; \n"
        "    float maxRange = oe_VisibleLayer_ranges[1]; \n"
        "    float attRange = oe_VisibleLayer_ranges[2]; \n"

        "    float range = max(-vertexView.z, 0.0); \n"

        "    float maxOpaqueRange = maxRange-attRange; \n"
        "    float minOpaqueRange = minRange+attRange; \n"
        "    float rangeOpacity = \n"
        "        minRange >= maxRange ? 1.0 : \n"
        "        range >= maxRange || range <= minRange ? 0.0 : \n"
        "        range > maxOpaqueRange ? 1.0-((range-maxOpaqueRange)/(maxRange-maxOpaqueRange)) : \n"
        "        range < minOpaqueRange && minRange > 0.0 ? ((range-minRange)/(minOpaqueRange-minRange)) : \n"
        "        1.0; \n"

        //atten as a factor instead of a range:
        //"    float maxOpaqueRange = maxRange-((maxRange-minRange)*attenFactor); \n"
        //"    float minOpaqueRange = minRange+(minRange*attenFactor); \n"
        //"    float rangeOpacity = \n"
        //"        minRange >= maxRange ? 1.0 : \n"
        //"        range >= maxRange || range <= minRange ? 0.0 : \n"
        //"        range > maxOpaqueRange && maxRange < MAX_MAX_RANGE ? 1.0-((range-maxOpaqueRange)/(maxRange-maxOpaqueRange)) : \n"
        //"        range < minOpaqueRange && minRange > 0.0 ? ((range-minRange)/(minOpaqueRange-minRange)) : \n"
        //"        1.0; \n"

        "    oe_layer_opacity *= rangeOpacity; \n"
        "  #endif \n"
        "} \n";

    // Shader that calculates a modulation color based on the "opacity", i.e. intensity
    const char* opacityInterpolateFS =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"
        "in float oe_layer_opacity; \n"
        "void oe_VisibleLayer_setOpacity(inout vec4 color) \n"
        "{ \n"
        "    color.a *= oe_layer_opacity; \n"
        "} \n";

    // Shader that calculates a modulation color based on the "opacity", i.e. intensity
    const char* opacityModulateFS =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"
        "#define OE_MODULATION_EXPOSURE 2.35 \n"
        "in float oe_layer_opacity; \n"
        "void oe_VisibleLayer_setOpacity(inout vec4 color) \n"
        "{ \n"
        "    vec3 rgbHi = oe_layer_opacity > 0.0? color.rgb * float( OE_MODULATION_EXPOSURE )/oe_layer_opacity : vec3(1); \n"
        "    color.rgb = mix(vec3(1), rgbHi, oe_layer_opacity); \n"
        "    color.a = 1.0; \n"
        "    oe_layer_opacity = 1.0; \n"
        "} \n";
}

void
VisibleLayer::initializeBlending()
{
    if (!_opacityU.valid())
    {
        osg::StateSet* stateSet = getOrCreateStateSet();

        _opacityU = new osg::Uniform("oe_VisibleLayer_opacityUniform", (float)options().opacity().get());
        stateSet->addUniform(_opacityU.get());

        VirtualProgram* vp = VirtualProgram::getOrCreate(stateSet);
        vp->setName("VisibleLayer");

        vp->setFunction("oe_VisibleLayer_initOpacity", opacityVS, ShaderComp::LOCATION_VERTEX_MODEL);

        if (options().blend() == VisibleLayerOptions::BLEND_MODULATE)
        {
            vp->setFunction("oe_VisibleLayer_setOpacity", opacityModulateFS, ShaderComp::LOCATION_FRAGMENT_COLORING, 1.1f);

            stateSet->setAttributeAndModes(
                new osg::BlendFunc(GL_DST_COLOR, GL_ZERO),
                osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
        }
        else
        {
            // In this case the fragment shader of the layer is responsible for
            // incorporating the final value of oe_layer_opacity.
            if (options().blend().isSetTo(options().BLEND_INTERPOLATE))
            {
                stateSet->setAttributeAndModes(
                    new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA),
                    osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE );
            }
        }        
    }
}

void
VisibleLayer::initializeMinMaxRangeOpacity()
{
    initializeBlending();

    if (!_rangeU.valid())
    {
        osg::StateSet* stateSet = getOrCreateStateSet();

        VirtualProgram* vp = VirtualProgram::getOrCreate(stateSet);
        vp->setName("VisibleLayer");

        vp->setFunction("oe_VisibleLayer_applyMinMaxRange", rangeOpacityVS, ShaderComp::LOCATION_VERTEX_VIEW);

        _rangeU = new osg::Uniform("oe_VisibleLayer_ranges", osg::Vec3f(
            (float)options().minVisibleRange().get(),
            (float)options().maxVisibleRange().get(),
            (float)options().attenuationRange().get()));

        stateSet->addUniform(_rangeU.get());
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
    initializeMinMaxRangeOpacity();
    options().minVisibleRange() = minVisibleRange;
    _rangeU->set(osg::Vec3f(
        (float)options().minVisibleRange().get(),
        (float)options().maxVisibleRange().get(),
        (float)options().attenuationRange().get()));
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
    initializeMinMaxRangeOpacity();
    options().maxVisibleRange() = maxVisibleRange;
    _rangeU->set(osg::Vec3f(
        (float)options().minVisibleRange().get(),
        (float)options().maxVisibleRange().get(),
        (float)options().attenuationRange().get()));
    fireCallback( &VisibleLayerCallback::onVisibleRangeChanged );
}

float
VisibleLayer::getMaxVisibleRange() const
{
    return options().maxVisibleRange().get();
}

void
VisibleLayer::setAttenuationRange(float value)
{
    initializeMinMaxRangeOpacity();
    options().attenuationRange() = value;
    _rangeU->set(osg::Vec3f(
        (float)options().minVisibleRange().get(),
        (float)options().maxVisibleRange().get(),
        (float)options().attenuationRange().get()));
}

float
VisibleLayer::getAttenuationRange() const
{
    return options().attenuationRange().get();
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

void
VisibleLayer::installDefaultOpacityShader()
{
    if (options().blend() == options().BLEND_INTERPOLATE)
    {
        VirtualProgram* vp = VirtualProgram::getOrCreate(getOrCreateStateSet());
        vp->setName("VisibleLayer");
        vp->setFunction("oe_VisibleLayer_setOpacity", opacityInterpolateFS, ShaderComp::LOCATION_FRAGMENT_COLORING, 1.1f);
    }
}
