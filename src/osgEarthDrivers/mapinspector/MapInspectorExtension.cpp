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
#include "MapInspectorExtension"
#include "MapInspectorUI"

#include <osgEarthFeatures/FeatureSource>
#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>
#include <osgEarthAnnotation/FeatureNode>
#include <osgEarthSymbology/Style>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;
using namespace osgEarth::MapInspector;

#define LC "[MapInspector] "


REGISTER_OSGEARTH_EXTENSION(osgearth_mapinspector, MapInspectorExtension)


MapInspectorExtension::MapInspectorExtension()
{
    ctor();
}

MapInspectorExtension::MapInspectorExtension(const ConfigOptions& options)
{
    ctor();
}

MapInspectorExtension::~MapInspectorExtension()
{
    // nop
}

void
MapInspectorExtension::ctor()
{
    OE_INFO << LC << "loaded\n";
    _ui = new MapInspectorUI();
}


void 
MapInspectorExtension::onMapModelChanged(const MapModelChange& change)
{
    osg::ref_ptr<MapNode> mapNode;
    _mapNode.lock(mapNode);
    static_cast<MapInspectorUI*>(_ui.get())->reinit(mapNode.get());
}

bool
MapInspectorExtension::connect(MapNode* mapNode)
{
    OE_INFO << LC << "connected\n";
    if ( mapNode )
    {
        _mapNode = mapNode;
        _mapNode->getMap()->addMapCallback(this);
        static_cast<MapInspectorUI*>(_ui.get())->reinit(mapNode);
    }
    
    return true;
}

bool
MapInspectorExtension::disconnect(MapNode* mapNode)
{
    OE_INFO << LC << "disconnected\n";

    if ( mapNode )
        mapNode->getMap()->removeMapCallback(this);

    static_cast<MapInspectorUI*>(_ui.get())->reinit(0L);
    return true;
}

bool
MapInspectorExtension::connect(Control* control)
{
    Container* container = dynamic_cast<Container*>(control);
    if ( container && _ui.valid() )
    {
        container->addControl( _ui.get() );
    }

    return true;
}

bool
MapInspectorExtension::disconnect(Control* control)
{
    Container* container = dynamic_cast<Container*>(control);
    if ( container && _ui.valid() )
    {
        container->removeChild( _ui.get() );
    }
    return true;
}
