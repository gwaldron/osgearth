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

using namespace osgEarth;

#define LC "[VisibleLayer] Layer \"" << getName() << "\" "

//------------------------------------------------------------------------

VisibleLayerOptions::VisibleLayerOptions() :
LayerOptions()
{
    setDefaults();
    mergeConfig( _conf ); 
}

VisibleLayerOptions::VisibleLayerOptions(const ConfigOptions& co) :
LayerOptions(co)
{
    setDefaults();
    mergeConfig(_conf);
}

void
VisibleLayerOptions::setDefaults()
{
    _visible.init( true );
}

Config
VisibleLayerOptions::getConfig() const
{
    Config conf = LayerOptions::getConfig();
    conf.updateIfSet( "visible", _visible );
    return conf;
}

void
VisibleLayerOptions::mergeConfig(const Config& conf)
{
    conf.getIfSet( "visible", _visible );
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
VisibleLayer::setVisible(bool value)
{
    if (options().visible().get() != value)
    {
        mutableOptions().visible() = value;

        // if this layer has a scene graph node, toggle its node mask
        osg::Node* node = getNode();
        if (node)
            node->setNodeMask(value? ~0 : 0);

        fireCallback(&Callback::onVisibleChanged);
    }
}

bool
VisibleLayer::getVisible() const
{
    return options().visible().get();
}

void
VisibleLayer::fireCallback(CallbackMethodPtr method)
{
    for (CallbackVector::iterator i = _callbacks.begin(); i != _callbacks.end(); ++i)
    {
        Callback* cb = dynamic_cast<Callback*>(i->get());
        if (cb) (cb->*method)(this);
    }
}
