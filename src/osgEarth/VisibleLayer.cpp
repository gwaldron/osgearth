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
}

Config
VisibleLayerOptions::getConfig() const
{
    Config conf = LayerOptions::getConfig();
    conf.set("visible", _visible);
    conf.set("opacity", _opacity);
    return conf;
}

void
VisibleLayerOptions::fromConfig(const Config& conf)
{
    conf.getIfSet( "visible", _visible );
    conf.getIfSet("opacity", _opacity);
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
    return Layer::open();
}

void
VisibleLayer::setVisible(bool value)
{
    options().visible() = value;

    // if this layer has a scene graph node, toggle its node mask
    osg::Node* node = getOrCreateNode();
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
    const char* opacityFS =
        "#version " GLSL_VERSION_STR "\n"
        "uniform float oe_visibleLayer_opacity; \n"
        "void oe_visibleLayer_setOpacity(inout vec4 color) { \n"
        "    color.a *= oe_visibleLayer_opacity; \n"
        "} \n";
}

void
VisibleLayer::setOpacity(float value)
{
    options().opacity() = value;

    // On-demand installation of the opacity shader and rendering hint,
    // since they do incur a small performance penalty.
    if (value < 1.0f)
    {
        if (!_opacityU.valid())
        {
            osg::StateSet* stateSet = getOrCreateStateSet();
            _opacityU = new osg::Uniform("oe_visibleLayer_opacity", value);
            stateSet->addUniform(_opacityU.get());
            VirtualProgram* vp = VirtualProgram::getOrCreate(stateSet);
            vp->setFunction("oe_visibleLayer_setOpacity", opacityFS, ShaderComp::LOCATION_FRAGMENT_COLORING, 1.1f);
            // NOTE: do not alter the render bin here - it will screw up terrain rendering order!
        }
    }
    
    if (_opacityU.valid())
    {
        _opacityU->set(value);
    }

    fireCallback(&VisibleLayerCallback::onOpacityChanged);
}

float
VisibleLayer::getOpacity() const
{
    return options().opacity().get();
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
