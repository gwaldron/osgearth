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

#include <osg/ImageStream>
#include <osgDB/FileNameUtils>
#include <osg/Version>
#include <osgEarth/Version>

#include <osgEarthUtil/ImageOverlay>
#if OSG_MIN_VERSION_REQUIRED(2,9,6)
#include <osgEarthUtil/ImageOverlayEditor>
#endif

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;

static Grid* s_layerBox = NULL;
static Grid* s_imageBox = NULL;
static Grid* s_coordInfo = NULL;

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

    s_imageBox = new Grid();
    s_imageBox->setHorizAlign(Control::ALIGN_RIGHT);
    s_imageBox->setBackColor(0,0,0,0.5);
    s_imageBox->setMargin( 10 );
    s_imageBox->setPadding( 10 );
    s_imageBox->setChildSpacing( 10 );
    s_imageBox->setChildVertAlign( Control::ALIGN_CENTER );
    s_imageBox->setAbsorbEvents( true );
    s_imageBox->setVertAlign( Control::ALIGN_BOTTOM );

    s_coordInfo = new Grid();
    s_coordInfo->setHorizAlign(Control::ALIGN_LEFT);
    s_coordInfo->setBackColor(0,0,0,0.5);
    s_coordInfo->setMargin( 10 );
    s_coordInfo->setPadding( 10 );
    s_coordInfo->setChildSpacing( 10 );
    s_coordInfo->setChildVertAlign( Control::ALIGN_CENTER );
    s_coordInfo->setAbsorbEvents( true );
    s_coordInfo->setVertAlign( Control::ALIGN_TOP );

    canvas->addControl( s_layerBox );
    canvas->addControl( s_imageBox );
    canvas->addControl( s_coordInfo );

    return canvas;
}

int
usage( const std::string& msg )
{
    OE_NOTICE << msg << std::endl;
    OE_NOTICE << "USAGE: osgearth_imageoverlay file.earth" << std::endl;
    OE_NOTICE << "   --image                         : The image to overlay" << std::endl;    
    OE_NOTICE << "   --bounds xmin ymin xmax ymax    : The bounds of the overlay image" << std::endl;    
    OE_NOTICE << "   --vert                          : Move individual verts when editing" << std::endl;

        
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
#if OSG_MIN_VERSION_REQUIRED(2,9,6)
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

#else
        OE_NOTICE << "Use OSG 2.9.6 or greater to use editing" << std::endl;
#endif
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
        _overlay->setImage( _image );
        _preview->setImage( _image );
    }
    ImageOverlay* _overlay;
    osg::ref_ptr< osg::Image > _image;
    osg::ref_ptr< ImageControl> _preview;
};

void addImage(osg::Image* image, ImageOverlay* overlay, ImageControl* preview)
{
    static unsigned int row = 0;
    // Add an image:                
    ImageControl* imageCon = new ImageControl( image );
    imageCon->setSize( 128, 128 );
    imageCon->setVertAlign( Control::ALIGN_CENTER );
    s_imageBox->setControl( 0, row++, imageCon );     
    imageCon->addEventHandler(new ChangeImageHandler(image, overlay, preview));
}

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
        _label->setText( ss.str() );
    }
    

    osg::ref_ptr< LabelControl > _label;
    osg::ref_ptr< ImageOverlay > _overlay;
    ImageOverlay::ControlPoint _controlPoint;
};

void addCoordInfo(ImageOverlay* overlay, ImageOverlay::ControlPoint point)
{

    static unsigned int row = 0;
    Grid *grid = new Grid();
    //grid->setBackColor(0,0,0,0.5);
    grid->setMargin( 10 );
    grid->setPadding( 10 );
    grid->setChildSpacing( 10 );
    grid->setChildVertAlign( Control::ALIGN_CENTER );
    grid->setAbsorbEvents( true );
    grid->setVertAlign( Control::ALIGN_BOTTOM );

    std::string name;
    switch (point)
    {
    case ImageOverlay::CONTROLPOINT_CENTER:
        name = "Center:";
        break;
    case ImageOverlay::CONTROLPOINT_LOWER_LEFT:
        name = "Lower Left:";
        break;
    case ImageOverlay::CONTROLPOINT_LOWER_RIGHT:
        name = "Lower Right:";
        break;
    case ImageOverlay::CONTROLPOINT_UPPER_LEFT:
        name = "Upper Left:";
        break;
    case ImageOverlay::CONTROLPOINT_UPPER_RIGHT:
        name = "Upper Right:";
        break;
    }

    LabelControl* lbl = new LabelControl( name );      
    lbl->setVertAlign( Control::ALIGN_CENTER );
    grid->setControl(0, 0, lbl);



    LabelControl* coords = new LabelControl(  );      
    osg::Vec2d location = overlay->getControlPoint( point );
    std::stringstream ss;
    ss << location.y() << ", " << location.x();
    coords->setText( ss.str() );
    coords->setVertAlign( Control::ALIGN_CENTER );
    grid->setControl(1, 0, coords);
    overlay->addCallback(new UpdateLabelCallback(coords, overlay, point));

    s_coordInfo->setControl( 0, row++, grid );  
}



int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osg::DisplaySettings::instance()->setMinimumNumStencilBits( 8 );


    //Read in the image to overlay
    std::string imageFile = "../data/osgearth.gif";
    while (arguments.read("--image", imageFile));
    
    //Read in the bounds
    Bounds bounds(-100, 30, -90, 40);
    while (arguments.read("--bounds", bounds.xMin(), bounds.yMin(), bounds.xMax(), bounds.yMax()));

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
        ImageOverlay* overlay = new ImageOverlay();        
        overlay->setImage( image );
        overlay->setBounds(bounds);

        //Create a new ModelLayer so we can overlay it on the earth
        osgEarth::ModelLayer* modelLayer = new osgEarth::ModelLayer("overlay",overlay);
        modelLayer->setOverlay( true );
        mapNode->getMap()->addModelLayer( modelLayer );

        //Create a new ImageOverlayEditor and set it's node mask to 0 to hide it initially
#if OSG_MIN_VERSION_REQUIRED(2,9,6)
        osg::Node* editor = new ImageOverlayEditor( overlay, mapNode->getMap()->getProfile()->getSRS()->getEllipsoid(), mapNode );
#else
        //Just make an empty group for pre-2.9.6
        osg::Node* editor = new osg::Group;
#endif
        editor->setNodeMask( 0 );
        root->addChild( editor );      

        addCoordInfo(overlay, ImageOverlay::CONTROLPOINT_CENTER);
        addCoordInfo(overlay, ImageOverlay::CONTROLPOINT_UPPER_LEFT);
        addCoordInfo(overlay, ImageOverlay::CONTROLPOINT_UPPER_RIGHT);
        addCoordInfo(overlay, ImageOverlay::CONTROLPOINT_LOWER_LEFT);
        addCoordInfo(overlay, ImageOverlay::CONTROLPOINT_LOWER_RIGHT);

        // Add an image preview
        ImageControl* imageCon = new ImageControl( image );
        imageCon->setSize( 64, 64 );
        imageCon->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 0, 0, imageCon );            


        //Add some controls        
        CheckBoxControl* enabled = new CheckBoxControl( true );
        enabled->addEventHandler( new EnabledHandler(overlay) );
        enabled->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 1, 0, enabled );

        //The overlay name
        LabelControl* name = new LabelControl( osgDB::getSimpleFileName( imageFile) );      
        name->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 2, 0, name );

        // an opacity slider
        HSliderControl* opacity = new HSliderControl( 0.0f, 1.0f, overlay->getAlpha() );
        opacity->setWidth( 125 );
        opacity->setHeight( 12 );
        opacity->setVertAlign( Control::ALIGN_CENTER );
        opacity->addEventHandler( new OpacityHandler(overlay) );
        s_layerBox->setControl( 3, 0, opacity );

        // Add a text label:
        LabelControl* edit = new LabelControl( "Edit" );        
        edit->setVertAlign( Control::ALIGN_CENTER );
        edit->addEventHandler(new EditHandler(overlay, &viewer, editor));
        s_layerBox->setControl(4, 0, edit );


        //Add some images that the user can tinker with
        addImage(osgDB::readImageFile("../data/icon.png"), overlay, imageCon);
        addImage(osgDB::readImageFile("../data/tree.gif"), overlay, imageCon);
        addImage(osgDB::readImageFile("../data/osgearth.gif"), overlay, imageCon);
        addImage(image, overlay, imageCon);
    }

    // osgEarth benefits from pre-compilation of GL objects in the pager. In newer versions of
    // OSG, this activates OSG's IncrementalCompileOpeartion in order to avoid frame breaks.
    viewer.getDatabasePager()->setDoPreCompile( true );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());    
    viewer.addEventHandler(new osgViewer::LODScaleHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    viewer.addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));

    return viewer.run();
}
