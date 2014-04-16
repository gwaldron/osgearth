/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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

#include <typeinfo>

#include <osg/Notify>
#include <osgDB/ReadFile>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgEarth/Registry>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/Controls>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthSymbology/Color>

using namespace osgEarth::Symbology;
using namespace osgEarth::Util::Controls;


void createControls( ControlCanvas* );
ImageControl* s_imageControl;


int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);       
    osgViewer::Viewer viewer(arguments);

    osg::Group* root = new osg::Group();
    osg::Node* node = osgEarth::Util::MapNodeHelper().load(arguments, &viewer);
    if ( node )
        root->addChild( node );

    // create a surface to house the controls
    ControlCanvas* cs = ControlCanvas::get( &viewer );

    viewer.setSceneData( root );
    viewer.setCameraManipulator( new osgEarth::Util::EarthManipulator );

    // create some controls.
    createControls( cs );

    return viewer.run();
}

struct MyClickHandler : public ControlEventHandler
{
    void onClick( Control* control, const osg::Vec2f& pos, int mouseButtonMask )
    {
        OE_NOTICE << "You clicked at (" << pos.x() << ", " << pos.y() << ") within the control."
            << std::endl;
    }
};

static LabelControl* s_sliderLabel;

struct MySliderHandler : public ControlEventHandler
{
    void onValueChanged( Control* control, float value )
    {
        std::stringstream buf;
        buf << (int)value;
        std::string str;
        str = buf.str();
        s_sliderLabel->setText( str );
    }
};

struct RotateImage : public ControlEventHandler
{
    void onValueChanged( Control* control, float value )
    {
        s_imageControl->setRotation( Angular(value) );
    }
};

struct SetImageOpacity : public ControlEventHandler
{
    void onValueChanged( Control* control, float value )
    {
        s_imageControl->setOpacity( value );
    }
};

void
createControls( ControlCanvas* cs )
{
    // a container centered on the screen, containing an image and a text label.
    {
        VBox* center = new VBox();
        center->setBorderColor( 1, 1, 1, 1 );
        center->setBackColor( .6,.5,.4,0.5 );
        center->setPadding( 10 );
        center->setHorizAlign( Control::ALIGN_CENTER );
        center->setVertAlign( Control::ALIGN_CENTER );

        // Add an image:
        osg::ref_ptr<osg::Image> image = osgDB::readImageFile("http://demo.pelicanmapping.com/rmweb/readymap_logo.png");
        if ( image.valid() )
        {
            s_imageControl = new ImageControl( image.get() );
            s_imageControl->setHorizAlign( Control::ALIGN_CENTER );
            s_imageControl->setFixSizeForRotation( true );
            center->addControl( s_imageControl );
            center->setHorizAlign( Control::ALIGN_CENTER );
        }

        // Add a text label:
        LabelControl* label = new LabelControl( "osgEarth Controls Toolkit" );
        label->setFont( osgEarth::Registry::instance()->getDefaultFont() );
        label->setFontSize( 24.0f );
        label->setHorizAlign( Control::ALIGN_CENTER );
        label->setMargin( 5 );
        center->addControl( label );

        // Rotation slider
        HBox* rotateBox = new HBox();
        rotateBox->setChildVertAlign( Control::ALIGN_CENTER );
        rotateBox->setHorizFill( true );
        rotateBox->setBackColor( Color::Blue );
        {
            rotateBox->addControl( new LabelControl("Rotate: ") );

            HSliderControl* rotateSlider = new HSliderControl( -180.0, 180.0, 0.0 );
            rotateSlider->addEventHandler( new RotateImage() );
            rotateSlider->setHeight( 8.0f );
            rotateSlider->setHorizFill( true );
            rotateBox->addControl( rotateSlider );
        }
        center->addControl( rotateBox );

        // Opacity slider:
        HBox* opacityBox = new HBox();
        opacityBox->setChildVertAlign( Control::ALIGN_CENTER );
        opacityBox->setHorizFill( true );
        opacityBox->setBackColor( Color::Green );
        {
            opacityBox->addControl( new LabelControl("Opacity: ") );

            HSliderControl* opacitySlider = new HSliderControl( 0.0, 1.0, 1.0 );
            opacitySlider->addEventHandler( new SetImageOpacity() );
            opacitySlider->setHeight( 8.0f );
            opacitySlider->setHorizFill( true, 200 );
            opacityBox->addControl( opacitySlider );
        }
        center->addControl( opacityBox );

        cs->addControl( center );
    }

    // a simple vbox with absolute positioning in the upper left with two text labels.
    {
        VBox* ul = new VBox();
        ul->setPosition( 20, 20 );
        ul->setPadding( 10 );
        {
            LabelControl* title = new LabelControl( "Upper left control", 22, osg::Vec4f(1,1,0,1) );
            ul->addControl( title );

            LabelControl* content = new LabelControl( "Here is some text in the upper left control" );
            ul->addControl( content );

            HBox* c2 = new HBox();
            c2->setChildSpacing( 10 );
            {
                HSliderControl* slider = new HSliderControl( 0, 100 );
                slider->setBackColor( .6,0,0,1 );
                slider->setHeight( 25 );
                slider->setWidth( 300 );
                slider->addEventHandler( new MySliderHandler() );
                c2->addControl( slider );

                s_sliderLabel = new LabelControl();
                s_sliderLabel->setVertAlign( Control::ALIGN_CENTER );
                c2->addControl( s_sliderLabel );        
            }
            ul->addControl( c2 );

            HBox* c3 = new HBox();
            c3->setHorizAlign( Control::ALIGN_CENTER );
            c3->setChildSpacing( 10 );
            {
                HBox* c4 = new HBox();
                c4->setChildSpacing( 5 );
                {
                    c4->addControl( new CheckBoxControl( true ) );
                    c4->addControl( new LabelControl( "Checkbox 1" ) );
                }
                c3->addControl( c4 );

                HBox* c5 = new HBox();
                c5->setChildSpacing( 5 );
                {
                    c5->addControl( new CheckBoxControl( false ) );
                    c5->addControl( new LabelControl( "Checkbox 2" ) );
                }
                c3->addControl( c5 );
            }
            ul->addControl( c3 );
        }
        cs->addControl( ul );

        ul->addEventHandler( new MyClickHandler );
    }

    // a centered hbox container along the bottom on the screen.
    {
        HBox* bottom = new HBox();
        bottom->setBackColor(0,0,0,0.5);        
        bottom->setMargin( 10 );
        bottom->setChildSpacing( 145 );
        bottom->setVertAlign( Control::ALIGN_BOTTOM );
        bottom->setHorizAlign( Control::ALIGN_CENTER );

        for( int i=0; i<4; ++i )
        {
            LabelControl* label = new LabelControl();
            std::stringstream buf;
            buf << "Label_" << i;
            std::string str;
            str = buf.str();
            label->setText( str );
            label->setMargin( 10 );
            label->setBackColor( 1,1,1,0.4 );
            bottom->addControl( label );

            label->setActiveColor(1,.3,.3,1);
            label->addEventHandler( new MyClickHandler );
        }

        cs->addControl( bottom );
    }
}
