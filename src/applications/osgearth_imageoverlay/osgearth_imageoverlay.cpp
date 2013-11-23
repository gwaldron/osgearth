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

#include <osg/Notify>
#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/MapNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/Controls>
#include <osgEarth/Utils>
#include <osgEarth/VirtualProgram>

#include <osg/ImageStream>
#include <osgDB/FileNameUtils>

#include <osgEarthAnnotation/ImageOverlay>
#include <osgEarthAnnotation/ImageOverlayEditor>

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;

static Grid* s_layerBox = NULL;

osg::Node*
createControlPanel( osgViewer::View* view )
{
    ControlCanvas* canvas = ControlCanvas::get( view );

    // the outer container:
    s_layerBox = new Grid();
    s_layerBox->setBackColor(0,0,0,0.5);
    s_layerBox->setMargin( 10 );
    s_layerBox->setPadding( 10 );
    s_layerBox->setChildSpacing( 10 );
    s_layerBox->setChildVertAlign( Control::ALIGN_CENTER );
    s_layerBox->setAbsorbEvents( true );
    s_layerBox->setVertAlign( Control::ALIGN_BOTTOM );



    canvas->addControl( s_layerBox );    
    return canvas;
}

int
usage( const std::string& msg )
{
    OE_NOTICE << msg << std::endl;
    OE_NOTICE << "USAGE: osgearth_imageoverlay file.earth" << std::endl;
    OE_NOTICE << "   --image <file> xmin ymin xmax ymax : An image to overlay and it's bounds" << std::endl;        
    OE_NOTICE << "   --vert                             : Move individual verts when editing" << std::endl;

        
    return -1;
}

struct OpacityHandler : public ControlEventHandler
{
    OpacityHandler( ImageOverlay* overlay ) : _overlay(overlay) { }
    void onValueChanged( Control* control, float value ) {
        _overlay->setAlpha( value );
    }
    ImageOverlay* _overlay;
};

struct EnabledHandler : public ControlEventHandler
{
    EnabledHandler( ImageOverlay* overlay ) :  _overlay(overlay) { }
    void onValueChanged( Control* control, bool value ) {
        _overlay->setNodeMask( value ? ~0 : 0 );
    }
    ImageOverlay* _overlay;
};



struct EditHandler : public ControlEventHandler
{
    EditHandler( ImageOverlay* overlay, osgViewer::Viewer* viewer, osg::Node* editor) :
      _overlay(overlay),
      _viewer(viewer),
      _editor(editor){ }

    void onClick( Control* control, int mouseButtonMask ) {        
        if (_editor->getNodeMask() != ~0)
        {
            static_cast<LabelControl*>(control)->setText( "Finish" );
            _editor->setNodeMask(~0);
        }
        else
        {
            static_cast<LabelControl*>(control)->setText( "Edit" );
            _editor->setNodeMask(0);
        }
    }

    ImageOverlay* _overlay;
    osgViewer::Viewer* _viewer;
    osg::Node* _editor;
};

struct ChangeImageHandler : public ControlEventHandler
{
    ChangeImageHandler( osg::Image* image, ImageOverlay* overlay, ImageControl* preview) :
      _image(image),
      _overlay(overlay),
      _preview(preview){ }

    void onClick( Control* control, int mouseButtonMask ) {
        _overlay->setImage( _image.get() );
        _preview->setImage( _image.get() );
    }
    ImageOverlay* _overlay;
    osg::ref_ptr< osg::Image > _image;
    osg::ref_ptr< ImageControl> _preview;
};

struct UpdateLabelCallback : public ImageOverlay::ImageOverlayCallback
{
    UpdateLabelCallback(LabelControl* label, ImageOverlay* overlay, ImageOverlay::ControlPoint controlPoint):
      _label(label),
      _overlay(overlay),
      _controlPoint(controlPoint)
    {

    }

    virtual void onOverlayChanged()
    {
        osg::Vec2d location = _overlay->getControlPoint( _controlPoint );
        std::stringstream ss;
        ss << location.y() << ", " << location.x();
        std::string str;
        str = ss.str();
        _label->setText( str );
    }
    

    osg::ref_ptr< LabelControl > _label;
    osg::ref_ptr< ImageOverlay > _overlay;
    ImageOverlay::ControlPoint _controlPoint;
};



int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osg::DisplaySettings::instance()->setMinimumNumStencilBits( 8 );


    std::vector< std::string > imageFiles;
    std::vector< Bounds > imageBounds;

    //Read in the image files
    std::string filename;
    Bounds bounds;
    while (arguments.read("--image", filename, bounds.xMin(), bounds.yMin(), bounds.xMax(), bounds.yMax()))
    {
        imageFiles.push_back( filename );
        imageBounds.push_back( bounds );
    }

    if (imageFiles.empty())
    {
      imageFiles.push_back("../data/osgearth.gif");
      imageBounds.push_back( Bounds(-100, 30, -90, 40) );
    }
 

    bool moveVert = arguments.read("--vert");

    // load the .earth file from the command line.
    osg::Node* earthNode = osgDB::readNodeFiles( arguments );
    if (!earthNode)
        return usage( "Unable to load earth model." );

    osgViewer::Viewer viewer(arguments);
    
    EarthManipulator* manip = new EarthManipulator();
    viewer.setCameraManipulator( manip );

    osg::Group* root = new osg::Group();
    root->addChild( earthNode );

    //Create the control panel
    root->addChild( createControlPanel(&viewer) );

    viewer.setSceneData( root );
    
    osgEarth::MapNode* mapNode = osgEarth::MapNode::findMapNode( earthNode );
    if ( mapNode )
    {

        for (unsigned int i = 0; i < imageFiles.size(); i++)
        {
            std::string imageFile = imageFiles[i];
            //Read the image file and play it if it's a movie
            osg::Image* image = osgDB::readImageFile(imageFile);
            if (image)
            {
                osg::ImageStream* is = dynamic_cast<osg::ImageStream*>(image);
                if (is)
                {
                    is->play();
                }
            }

            //Create a new ImageOverlay and set it's bounds
            //ImageOverlay* overlay = new ImageOverlay(mapNode->getMap()->getProfile()->getSRS()->getEllipsoid(), image);        
            ImageOverlay* overlay = new ImageOverlay(mapNode);
            overlay->setImage( image );
            overlay->setBounds(imageBounds[i]);
            
            root->addChild( overlay );


            //Create a new ImageOverlayEditor and set it's node mask to 0 to hide it initially
            osg::Node* editor = new ImageOverlayEditor( overlay, moveVert);
            editor->setNodeMask( 0 );
            root->addChild( editor );      
            
            // Add an image preview
            ImageControl* imageCon = new ImageControl( image );
            imageCon->setSize( 64, 64 );
            imageCon->setVertAlign( Control::ALIGN_CENTER );
            s_layerBox->setControl( 0, i, imageCon );            


            //Add some controls        
            CheckBoxControl* enabled = new CheckBoxControl( true );
            enabled->addEventHandler( new EnabledHandler(overlay) );
            enabled->setVertAlign( Control::ALIGN_CENTER );
            s_layerBox->setControl( 1, i, enabled );

            //The overlay name
            LabelControl* name = new LabelControl( osgDB::getSimpleFileName( imageFile) );      
            name->setVertAlign( Control::ALIGN_CENTER );
            s_layerBox->setControl( 2, i, name );

            // an opacity slider
            HSliderControl* opacity = new HSliderControl( 0.0f, 1.0f, overlay->getAlpha() );
            opacity->setWidth( 125 );
            opacity->setHeight( 12 );
            opacity->setVertAlign( Control::ALIGN_CENTER );
            opacity->addEventHandler( new OpacityHandler(overlay) );
            s_layerBox->setControl( 3, i, opacity );

            // Add a text label:
            LabelControl* edit = new LabelControl( "Edit" );        
            edit->setVertAlign( Control::ALIGN_CENTER );
            edit->addEventHandler(new EditHandler(overlay, &viewer, editor));
            s_layerBox->setControl(4, i, edit );
        }        
    }

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());    
    viewer.addEventHandler(new osgViewer::LODScaleHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    viewer.addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));

    return viewer.run();
}
