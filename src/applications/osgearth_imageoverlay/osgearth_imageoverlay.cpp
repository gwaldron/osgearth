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

#include <osg/ImageStream>
#include <osgDB/FileNameUtils>

#include <osgEarthUtil/ImageOverlay>
#include <osgEarthUtil/ImageOverlayEditor>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;

static Grid* s_layerBox = NULL;
static ImageOverlayEditor* s_editor = NULL;

osg::Node*
createControlPanel( osgViewer::View* view )
{
    ControlCanvas* canvas = new ControlCanvas( view );

    // the outer container:
    s_layerBox = new Grid();
    s_layerBox->setBackColor(0,0,0,0.5);
    s_layerBox->setMargin( 10 );
    s_layerBox->setPadding( 10 );
    s_layerBox->setSpacing( 10 );
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
    OE_NOTICE << "   --image                         : The image to overlay" << std::endl;    
    OE_NOTICE << "   --bounds xmin ymin xmax ymax    : The bounds of the overlay image" << std::endl;    
    OE_NOTICE << "   --vert                          : Move individual verts when editing" << std::endl;

        
    return -1;
}

struct ToggleEditHandler : public osgGA::GUIEventHandler
{
    ToggleEditHandler( ImageOverlay* overlay, osg::Group* editorGroup, bool moveVert)
        : _overlay(overlay),
          _editorGroup(editorGroup),
          _moveVert(moveVert){ }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        osgViewer::Viewer* viewer = static_cast<osgViewer::Viewer*>(&aa);
        if ( ea.getEventType() == ea.KEYDOWN )
        {
            //Hit 'e' to toggle editing
            if (ea.getKey() == 'e')
            {
                if (_editor.valid())
                {
                    viewer->removeEventHandler( _editor.get() );
                    _editor = 0;
                }
                else
                {
                    _editor = new ImageOverlayEditor(_overlay.get(), _editorGroup.get() );
                    _editor->setMoveVert( _moveVert );
                    viewer->addEventHandler( _editor.get() );
                }
                return true;
            }
        }
        return false;
    }

    osg::ref_ptr< ImageOverlayEditor > _editor;
    osg::ref_ptr< ImageOverlay > _overlay;
    osg::ref_ptr< osg::Group > _editorGroup;
    bool _moveVert;
};

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
    EditHandler( ImageOverlay* overlay, osgViewer::Viewer* viewer, osg::Group* editGroup) :
      _overlay(overlay),
      _viewer(viewer),
      _editGroup(editGroup){ }

    void onClick( Control* control, int mouseButtonMask ) {
        if (!s_editor)
        {
            static_cast<LabelControl*>(control)->setText( "Finish" );
            s_editor = new ImageOverlayEditor(_overlay, _editGroup );                    
            _viewer->addEventHandler( s_editor );
        }
        else
        {
            static_cast<LabelControl*>(control)->setText( "Edit" );
            if (s_editor)
            {
                _viewer->removeEventHandler( s_editor );
                s_editor = 0;
            }
        }
    }
    ImageOverlay* _overlay;
    osgViewer::Viewer* _viewer;
    osg::Group* _editGroup;
};



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
    root->addChild( createControlPanel( &viewer ) );
    
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
        ImageOverlay* overlay = new ImageOverlay(mapNode->getMap()->getProfile()->getSRS()->getEllipsoid(), image);        
        overlay->setBounds(bounds);

        //Create a new ModelLayer so we can overlay it on the earth
        osgEarth::ModelLayer* modelLayer = new osgEarth::ModelLayer("overlay",overlay);
        modelLayer->setOverlay( true );
        mapNode->getMap()->addModelLayer( modelLayer );

        //Create a group for the editor to stick it's controls
        osg::Group* editorGroup = new osg::Group;
        root->addChild( editorGroup );       

        // Add an image:                
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
        edit->addEventHandler(new EditHandler(overlay, &viewer, editorGroup));
        s_layerBox->setControl(4, 0, edit );
    }

    // osgEarth benefits from pre-compilation of GL objects in the pager. In newer versions of
    // OSG, this activates OSG's IncrementalCompileOpeartion in order to avoid frame breaks.
    viewer.getDatabasePager()->setDoPreCompile( true );

    viewer.setSceneData( root );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());    
    viewer.addEventHandler(new osgViewer::LODScaleHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    viewer.addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));

    return viewer.run();
}
