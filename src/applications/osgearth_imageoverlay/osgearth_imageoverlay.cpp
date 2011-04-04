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

#include <osg/ImageStream>

#include <osgEarthUtil/ImageOverlay>
#include <osgEarthUtil/ImageOverlayEditor>

using namespace osgEarth;
using namespace osgEarth::Util;

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
        
        viewer.addEventHandler( new ToggleEditHandler(overlay, editorGroup, moveVert ) );
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
