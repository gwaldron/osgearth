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
#include "SimpleSkyOptions"
#include "SimpleSkyNode"
#include <osgDB/FileNameUtils>
#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarth/NodeUtils>
#include <osgEarthUtil/Controls>
#include <osgEarthUtil/ExampleResources>

#define LC "[SimpleSkyDriver] "

using namespace osgEarth::Util;
namespace ui = osgEarth::Util::Controls;

namespace osgEarth { namespace SimpleSky
{
    class SimpleSkyExtension : public Extension,
                               public ExtensionInterface<MapNode>,
                               public ExtensionInterface<osg::View>,
                               public ExtensionInterface<ui::Control>,
                               public SimpleSkyOptions,
                               public SkyNodeFactory
    {
    public:
        META_OE_Extension(osgEarth, SimpleSkyExtension, sky_simple);

        // CTORs
        SimpleSkyExtension() { }

        SimpleSkyExtension(const ConfigOptions& options) :
            SimpleSkyOptions(options) { }

    public: // Extension

        const ConfigOptions& getConfigOptions() const { return *this; }


    public: // ExtensionInterface<MapNode>

        bool connect(MapNode* mapNode)
        {
            _skynode = createSkyNode();
            if (mapNode->getMapSRS()->isProjected())
            {
                GeoPoint refPoint;
                mapNode->getMap()->getProfile()->getExtent().getCentroid(refPoint);
                _skynode->setReferencePoint(refPoint);
            }                
            osgEarth::insertParent(_skynode.get(), mapNode);
            return true;
        }

        bool disconnect(MapNode* mapNode)
        {
            osgEarth::removeGroup(_skynode.get());
            _skynode = 0L;
            return true;
        }

    public: // ExtensionInterface<osg::View>

        bool connect(osg::View* view)
        {
            if (view && _skynode.valid())
            {
                _skynode->attach(view, 0);
            }
            return true;
        }

        bool disconnect(osg::View* view)
        {
            //todo
            return true;
        }


    public: // ExtensionInterface<Control>

        bool connect(ui::Control* control)
        {
            ui::Container* container = dynamic_cast<ui::Container*>(control);
            if (container)
                _ui = container->addControl(SkyControlFactory::create(_skynode.get()));
            return true;
        }

        bool disconnect(ui::Control* control)
        {
            ui::Container* container = dynamic_cast<ui::Container*>(control);
            if (container && _ui.valid())
                container->removeChild(_ui.get());
            return true;
        }

    public: // SkyNodeFactory

        SkyNode* createSkyNode() {
            return new SimpleSkyNode(*this);
        }


    protected: // Object

        // DTOR
        virtual ~SimpleSkyExtension() { }


    private:
        osg::ref_ptr<ui::Control> _ui;
        osg::ref_ptr<SkyNode> _skynode;
    };

    REGISTER_OSGEARTH_EXTENSION(osgearth_sky_simple, SimpleSkyExtension)

} } // namespace osgEarth::SimpleSky
