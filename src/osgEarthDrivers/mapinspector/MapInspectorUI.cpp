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
#include "MapInspectorUI"
#include <osgEarth/GeoData>
#include <osgEarthAnnotation/FeatureNode>
#include <osgEarthAnnotation/LabelNode>
#include <osgEarthFeatures/Feature>
#include <osgEarthSymbology/Style>
#include <osgEarthSymbology/Geometry>

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace osgEarth::MapInspector;

namespace ui = osgEarth::Util::Controls;

#define LC "[MapInspectorUI] "

MapInspectorUI::MapInspectorUI()
{
    //nop
}

void
MapInspectorUI::reinit(MapNode* mapNode)
{
    if ( !_annos.valid() )
        _annos = new osg::Group();

    _annos->removeChildren(0, _annos->getNumChildren());

    this->clearControls();

    if ( mapNode )
    {
        // install annotation group as neccesary
        if (_annos->getNumParents() == 0 || _annos->getParent(0) != mapNode)
        {
            if ( _annos->getNumParents() > 0 )
            {
                _annos->getParent(0)->removeChild(_annos.get());
            }
            mapNode->addChild( _annos.get() );
        }
        
        Map* map = mapNode->getMap();

        for(int i=0; i<map->getNumElevationLayers(); ++i)
        {
            ElevationLayer* layer = map->getElevationLayerAt(i);
            addTerrainLayer( layer, mapNode );
        }

        for(int i=0; i<map->getNumImageLayers(); ++i)
        {
            ImageLayer* layer = map->getImageLayerAt(i);
            addTerrainLayer( layer, mapNode );
        }

        for(int i=0; i<map->getNumModelLayers(); ++i)
        {
            ModelLayer* layer = map->getModelLayerAt(i);
            addModelLayer( layer, mapNode );
        }
    }
    else
    {
        OE_INFO << LC << "MapNode is null\n";
    }
}

void
MapInspectorUI::addTerrainLayer(TerrainLayer* layer,
                                MapNode*      mapNode)
{
    const Color colors[6] = {
        Color::White,
        Color::Yellow,
        Color::Cyan,
        Color::Lime,
        Color::Red,
        Color::Magenta
    };
    Color color = colors[(int)layer->getUID()%6];

    osg::ref_ptr<MultiGeometry> collection = new MultiGeometry();

    DataExtentList exlist;
    if (layer->getDataExtents(exlist))
    {
        for(DataExtentList::const_iterator i = exlist.begin(); i != exlist.end(); ++i)
        {
            const DataExtent& e = *i;
            Polygon* p = new Polygon();
            p->push_back( e.xMin(), e.yMin() );
            p->push_back( e.xMax(), e.yMin() );
            p->push_back( e.xMax(), e.yMax() );
            p->push_back( e.xMin(), e.yMax() );
            collection->add( p );
        }

        // poly:
        {
            Style style;
            style.getOrCreate<LineSymbol>()->stroke()->color() = color;
            style.getOrCreate<LineSymbol>()->stroke()->width() = 2;
            style.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
            style.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_DRAPE;
            style.getOrCreate<RenderSymbol>()->lighting() = false;

            Feature* feature = new Feature(collection.get(), layer->getProfile()->getSRS(), style);
            FeatureNode* node = new FeatureNode( mapNode, feature );
            _annos->addChild( node );
        }

        // label:
        std::string text = 
            !layer->getName().empty()? layer->getName() :
            Stringify() << "Layer " << layer->getUID();

        {
            Style style;
            TextSymbol* ts = style.getOrCreate<TextSymbol>();
            ts->halo()->color().set(0,0,0,1);
            ts->declutter() = true;
            ts->alignment() = TextSymbol::ALIGN_CENTER_CENTER;
        
            osg::Vec2d center = collection->getBounds().center2d();
            GeoPoint position(layer->getProfile()->getSRS(), center.x(), center.y(), 0.0, ALTMODE_ABSOLUTE);

            LabelNode* label = new LabelNode(mapNode, position, text, style);
            _annos->addChild( label );
        }

        unsigned r = this->getNumRows();
        setControl(0, r, new ui::LabelControl(text, color));
    }
}

void
MapInspectorUI::addModelLayer(ModelLayer* layer,
                              MapNode*    mapNode)
{
}
