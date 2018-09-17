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
#include <osgDB/ReaderWriter>
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>
#include <osgDB/FileUtils>

#include "SimpleOceanNode"
#include "SimpleOceanOptions"
#include <osgEarthUtil/Controls>

#undef  LC
#define LC "[SimpleOceanExtension] "

using namespace osgEarth;
using namespace osgEarth::Util;
namespace ui = osgEarth::Util::Controls;

namespace osgEarth { namespace SimpleOcean
{
    /**
     * Extension that lets you load a SimpleOcean from an earth file.
     */
    class SimpleOceanExtension : public Extension,
                                 public ExtensionInterface<MapNode>,
                                 public ExtensionInterface<ui::Control>,
                                 public SimpleOceanOptions,
                                 public OceanNodeFactory
    {
    public:
        META_OE_Extension(osgEarth, SimpleOceanExtension, simple_oecan);

        SimpleOceanExtension() { }

        SimpleOceanExtension(const ConfigOptions& options) :
            SimpleOceanOptions(options) { }

        const ConfigOptions& getConfigOptions() const { return *this; }

    public: // ExtensionInterface<MapNode>

        bool connect(MapNode* mapNode)
        {
            _oceanNode = createOceanNode(mapNode);
            mapNode->addChild(_oceanNode.get());
            return true;
        }

        bool disconnect(MapNode* mapNode)
        {
            if (mapNode && _oceanNode.valid())
                mapNode->removeChild(_oceanNode.get());
            return true;
        }

    public: // ExtensionInterface<Control>

        bool connect(ui::Control* control);

        bool disconnect(ui::Control* control) {
            return true;
        }

    public: // OCeanNodeFactory

        OceanNode* createOceanNode(MapNode* mapNode) {
            return new SimpleOceanNode(*this, mapNode);
        }

    protected:
        virtual ~SimpleOceanExtension() { }

        osg::ref_ptr<OceanNode> _oceanNode;
    };

    REGISTER_OSGEARTH_EXTENSION(osgearth_ocean_simple, SimpleOceanExtension);

} } // namespace osgEarth::Drivers::SimpleOcean





using namespace osgEarth::SimpleOcean;

namespace
{
    struct ChangeSeaLevel : public ui::ControlEventHandler
    {
        ChangeSeaLevel( OceanNode* ocean ) : _ocean(ocean) { }
        OceanNode* _ocean;
        virtual void onValueChanged( ui::Control* control, float value )
        {
            _ocean->setSeaLevel( value );
        }
    };

    struct ChangeSeaAlpha : public ui::ControlEventHandler
    {
        ChangeSeaAlpha( OceanNode* ocean ) : _ocean(ocean) { }
        OceanNode* _ocean;
        virtual void onValueChanged( ui::Control* control, float value )
        {
            _ocean->setAlpha( value );
        }
    };
}

bool
SimpleOceanExtension::connect(ui::Control* control)
{
    ui::Container* container = dynamic_cast<ui::Container*>(control);
    if (container && _oceanNode.valid())
    {
        ui::VBox* main = new ui::VBox();

        ui::HBox* sealLevelBox = main->addControl(new ui::HBox());
        sealLevelBox->setChildVertAlign( ui::Control::ALIGN_CENTER );
        sealLevelBox->setChildSpacing( 10 );
        sealLevelBox->setHorizFill( true );

        sealLevelBox->addControl( new ui::LabelControl("Sea Level: ", 16) );

        ui::HSliderControl* mslSlider = sealLevelBox->addControl(new ui::HSliderControl( -250.0f, 250.0f, 0.0f ));
        mslSlider->setBackColor( Color::Gray );
        mslSlider->setHeight( 12 );
        mslSlider->setHorizFill( true, 200 );
        mslSlider->addEventHandler( new ChangeSeaLevel(_oceanNode.get()) );

        ui::HBox* alphaBox = main->addControl(new ui::HBox());
        alphaBox->setChildVertAlign( ui::Control::ALIGN_CENTER );
        alphaBox->setChildSpacing( 10 );
        alphaBox->setHorizFill( true );
    
        alphaBox->addControl( new ui::LabelControl("Sea Alpha: ", 16) );

        ui::HSliderControl* alphaSlider = alphaBox->addControl(new ui::HSliderControl( 0.0, 1.0, 1.0));
        alphaSlider->setBackColor( Color::Gray );
        alphaSlider->setHeight( 12 );
        alphaSlider->setHorizFill( true, 200 );
        alphaSlider->addEventHandler( new ChangeSeaAlpha(_oceanNode.get()) );

        container->addControl(main);
    }
    else
    {
        OE_WARN << LC << "Ocean node is unavailable; cannot create the UI.\n";
    }

    return true;
}