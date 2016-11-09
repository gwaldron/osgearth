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
#include "MonitorExtension"
#include "MonitorUI"

#include <osgEarthFeatures/FeatureSource>
#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>
#include <osgEarthAnnotation/FeatureNode>
#include <osgEarthSymbology/Style>

#include <osgGA/GUIEventHandler>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;
using namespace osgEarth::Monitor;

#define LC "[Monitor] "

namespace
{
    struct EventFrame : public osgGA::GUIEventHandler
    {
        EventFrame(MonitorExtension* ext) : _ext(ext) { }

        bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
        {
            if (ea.getEventType() == ea.FRAME)
            {
                _ext->frame(aa.asView()->getFrameStamp());
            }
            return false;
        }

        MonitorExtension* _ext;
    };
}

MonitorExtension::MonitorExtension()
{
    ctor();
}

MonitorExtension::MonitorExtension(const ConfigOptions& options)
{
    ctor();
}

MonitorExtension::~MonitorExtension()
{
    // nop
}

void
MonitorExtension::ctor()
{
    OE_INFO << LC << "loaded\n";
    _ui = new MonitorUI();
}


bool
MonitorExtension::connect(MapNode* mapNode)
{
    OE_INFO << LC << "connected\n";
    if ( mapNode )
    {
        _mapNode = mapNode;
        _mapNode->addEventCallback(new EventFrame(this));
    }
    
    return true;
}

bool
MonitorExtension::disconnect(MapNode* mapNode)
{
    OE_INFO << LC << "disconnected\n";

    return true;
}

bool
MonitorExtension::connect(Control* control)
{
    Container* container = dynamic_cast<Container*>(control);
    if ( container && _ui.valid() )
    {
        container->addControl( _ui.get() );
    }

    return true;
}

bool
MonitorExtension::disconnect(Control* control)
{
    Container* container = dynamic_cast<Container*>(control);
    if ( container && _ui.valid() )
    {
        container->removeChild( _ui.get() );
    }
    return true;
}

void
MonitorExtension::frame(const osg::FrameStamp* fs)
{
    if (_ui.valid())
        _ui->update(fs);
}