/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2015 Pelican Mapping
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
#include <osgEarthUtil/Controls>

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
        META_Object(osgearth_sky_simple, SimpleSkyExtension);

        // CTORs
        SimpleSkyExtension() { }

        SimpleSkyExtension(const ConfigOptions& options) :
            SimpleSkyOptions(options) { }

    public: // Extension

        const ConfigOptions& getConfigOptions() const { return *this; }


    public: // ExtensionInterface<MapNode>

        bool connect(MapNode* mapNode)
        {
            _skynode = createSkyNode(mapNode->getMap()->getProfile());
            osgEarth::insertParent(_skynode.get(), mapNode);
            return true;
        }

        bool disconnect(MapNode* mapNode)
        {
            //todo
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

        bool connect(ui::Control* control);

        bool disconnect(ui::Control* control)
        {
            return true;
        }

    public: // SkyNodeFactory

        SkyNode* createSkyNode(const Profile* profile) {
            return new SimpleSkyNode(profile->getSRS());
        }


    protected: // Object

        SimpleSkyExtension(const SimpleSkyExtension& rhs, const osg::CopyOp& op) { }

        // DTOR
        virtual ~SimpleSkyExtension() { }


    private:
        osg::ref_ptr<SkyNode> _skynode;
    };

    REGISTER_OSGEARTH_EXTENSION(osgearth_sky_simple, SimpleSkyExtension)

} } // namespace osgEarth::SimpleSky




using namespace osgEarth::SimpleSky;

namespace
{
    struct SkyHoursSlider : public ui::ControlEventHandler
    {
        SkyHoursSlider(SkyNode* sky) : _sky(sky)  { }
        SkyNode* _sky;
        void onValueChanged(ui::Control* control, float value )
        {
            DateTime d = _sky->getDateTime();
            _sky->setDateTime(DateTime(d.year(), d.month(), d.day(), value));
        }
    };

    struct SkyMonthSlider : public ui::ControlEventHandler
    {
        SkyMonthSlider(SkyNode* sky) : _sky(sky)  { }
        SkyNode* _sky;
        void onValueChanged(ui::Control* control, float value )
        {
            DateTime d = _sky->getDateTime();            
            _sky->setDateTime(DateTime(d.year(), (int)value, d.day(), d.hours()));
        }
    };

    struct SkyYearSlider : public ui::ControlEventHandler
    {
        SkyYearSlider(SkyNode* sky) : _sky(sky)  { }
        SkyNode* _sky;
        void onValueChanged(ui::Control* control, float value )
        {
            DateTime d = _sky->getDateTime();            
            _sky->setDateTime(DateTime((int)value, d.month(), d.day(), d.hours()));
        }
    };

    struct AmbientBrightnessHandler : public ui::ControlEventHandler
    {
        AmbientBrightnessHandler(SkyNode* sky) : _sky(sky) { }

        SkyNode* _sky;

        void onValueChanged(ui::Control* control, float value )
        {
            _sky->setMinimumAmbient(osg::Vec4(value,value,value,1));
        }
    };
}

bool
SimpleSkyExtension::connect(ui::Control* control)
{
    ui::Container* container = dynamic_cast<ui::Container*>(control);
    if (container)
    {
        ui::Grid* grid = new ui::Grid();
        grid->setChildVertAlign( ui::Control::ALIGN_CENTER );
        grid->setChildSpacing( 10 );
        grid->setHorizFill( true );
    
        DateTime dt = _skynode->getDateTime();

        int r=0;
        grid->setControl( 0, r, new ui::LabelControl("Time (Hours UTC): ", 16) );
        ui::HSliderControl* skyHoursSlider = grid->setControl(1, r, new ui::HSliderControl( 0.0f, 24.0f, dt.hours() ));
        skyHoursSlider->setHorizFill( true, 250 );
        skyHoursSlider->addEventHandler( new SkyHoursSlider(_skynode.get()) );
        grid->setControl(2, r, new ui::LabelControl(skyHoursSlider) );
    
        ++r;
        grid->setControl( 0, r, new ui::LabelControl("Month: ", 16) );
        ui::HSliderControl* skyMonthSlider = grid->setControl(1, r, new ui::HSliderControl( 0.0f, 11.0f, dt.month() ));
        skyMonthSlider->setHorizFill( true, 250 );
        skyMonthSlider->addEventHandler( new SkyMonthSlider(_skynode.get()) );
        grid->setControl(2, r, new ui::LabelControl(skyMonthSlider) );
    
        ++r;
        grid->setControl( 0, r, new ui::LabelControl("Year: ", 16) );
        ui::HSliderControl* skyYearSlider = grid->setControl(1, r, new ui::HSliderControl( 1970.0f, 2061.0f, dt.year() ));
        skyYearSlider->setHorizFill( true, 250 );
        skyYearSlider->addEventHandler( new SkyYearSlider(_skynode.get()) );
        grid->setControl(2, r, new ui::LabelControl(skyYearSlider) );

        ++r;
        grid->setControl(0, r, new ui::LabelControl("Min.Ambient: ", 16) );
        ui::HSliderControl* ambient = grid->setControl(1, r, new ui::HSliderControl(0.0f, 1.0f, _skynode->getSunLight()->getAmbient().r()));
        ambient->addEventHandler( new AmbientBrightnessHandler(_skynode.get()) );
        grid->setControl(2, r, new ui::LabelControl(ambient) );

        container->addControl(grid);
    }
    return true;
}
