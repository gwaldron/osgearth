/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2010 Pelican Mapping
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
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/Controls>

using namespace osgEarthUtil::Controls;


void createControls( ControlCanvas* );

int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);       
    osgViewer::Viewer viewer(arguments);

    osg::Group* root = new osg::Group();
    osg::Node* node = osgDB::readNodeFiles( arguments );
    if ( node )
        root->addChild( node );

    // create a surface to house the controls
    ControlCanvas* cs = new ControlCanvas( &viewer );
    root->addChild( cs );

    viewer.setSceneData( root );
    viewer.setCameraManipulator( new osgEarthUtil::EarthManipulator );

    // create some controls.
    createControls( cs );

    return viewer.run();
}

struct MyClickHandler : public ControlEventHandler
{
    void onClick( Control* control, int mouseButtonMask )
    {
        OE_NOTICE << "Thank you for clicking on " << typeid(control).name()
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
        std::string str = buf.str();
        s_sliderLabel->setText( str );
    }
};

void
createControls( ControlCanvas* cs )
{
    // a container centered on the screen, containing an image and a text label.
    {
        VBox* center = new VBox();
        center->setFrame( new RoundedFrame() );
        center->getFrame()->setBackColor( 1,1,1,0.5 );
        center->setPadding( 10 );
        center->setHorizAlign( ALIGN_CENTER );
        center->setVertAlign( ALIGN_CENTER );

        // Add an image:
        osg::Image* image = osgDB::readImageFile( "http://pelicanmapping.com/pelican.png" );
        if ( image ) {
            ImageControl* imageCon = new ImageControl( image );
            center->addControl( imageCon );
            center->setHorizAlign( ALIGN_CENTER );
        }

        // Add a text label:
        LabelControl* label = new LabelControl( "osgEarth Controls Toolkit - new in osgEarth 1.4" );
        label->setFont( osgText::readFontFile( "arialbd.ttf" ) );
        label->setFontSize( 24.0f );
        label->setHorizAlign( ALIGN_CENTER );
        center->addControl( label );

        cs->addControl( center );
    }

    // a simple vbox with absolute positioning in the upper left with two text labels.
    {
        VBox* ul = new VBox();
        ul->setFrame( new Frame() );
        ul->setPosition( 20, 20 );
        ul->setPadding( 10 );
        {
            LabelControl* title = new LabelControl( "Upper left control", 22, osg::Vec4f(1,1,0,1) );
            ul->addControl( title );

            LabelControl* content = new LabelControl( "Here is some text in the upper left control" );
            ul->addControl( content );

            HBox* c2 = new HBox();
            c2->setSpacing( 10 );
            {
                HSliderControl* slider = new HSliderControl( 0, 100 );
                slider->setBackColor( .6,0,0,1 );
                slider->setHeight( 25 );
                slider->setWidth( 300 );
                slider->addEventHandler( new MySliderHandler() );
                c2->addControl( slider );

                s_sliderLabel = new LabelControl();
                s_sliderLabel->setVertAlign( ALIGN_CENTER );
                c2->addControl( s_sliderLabel );        
            }
            ul->addControl( c2 );

            HBox* c3 = new HBox();
            c3->setHorizAlign( ALIGN_CENTER );
            c3->setSpacing( 10 );
            {
                HBox* c4 = new HBox();
                c4->setSpacing( 5 );
                {
                    c4->addControl( new CheckBoxControl( true ) );
                    c4->addControl( new LabelControl( "Checkbox 1" ) );
                }
                c3->addControl( c4 );

                HBox* c5 = new HBox();
                c5->setSpacing( 5 );
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
        bottom->setFrame( new RoundedFrame() );
        bottom->getFrame()->setBackColor(0,0,0,0.5);
        bottom->setMargin( 10 );
        bottom->setSpacing( 145 );
        bottom->setVertAlign( ALIGN_BOTTOM );
        bottom->setHorizAlign( ALIGN_CENTER );

        for( int i=0; i<4; ++i )
        {
            LabelControl* label = new LabelControl();
            std::stringstream buf;
            buf << "Label_" << i;
            label->setText( buf.str() );
            label->setMargin( 10 );
            label->setBackColor( 1,1,1,0.4 );
            bottom->addControl( label );

            label->setActiveColor(1,.3,.3,1);
            label->addEventHandler( new MyClickHandler );
        }

        cs->addControl( bottom );
    }
}
