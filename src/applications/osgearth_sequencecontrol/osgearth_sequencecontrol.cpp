/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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

#include <osgViewer/Viewer>
#include <osgEarth/MapNode>
#include <osgEarth/TimeControl>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/EarthManipulator>

namespace ui = osgEarth::Util::Controls;


int
usage(char** argv, const char* msg)
{
    OE_WARN << argv[0] << ": " << msg << std::endl;
    return -1;
}


// Updates the timestamp label once per frame.
struct UpdateLabel : public osg::Operation
{
    osgEarth::SequenceControl* _sc;
    ui::LabelControl*          _label;
    unsigned                   _prevIndex;

    UpdateLabel(osgEarth::SequenceControl* sc, ui::LabelControl* label)
        : osg::Operation("updatelabel", true), _sc(sc), _label(label), _prevIndex(INT_MAX) { }

    void operator()(osg::Object* obj)
    {
        osgViewer::View* view = dynamic_cast<osgViewer::View*>(obj);
        unsigned index = _sc->getCurrentSequenceFrameIndex(view->getFrameStamp());
        if ( index >= 0 && index != _prevIndex )
        {
            const std::vector<osgEarth::SequenceFrameInfo>& frames = _sc->getSequenceFrameInfo();
            _label->setText( frames[index].timeIdentifier );
            _prevIndex = index;
        }
    }
};


int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osgViewer::Viewer viewer(arguments);
    viewer.setCameraManipulator( new osgEarth::Util::EarthManipulator() );

    // load an earth file from the command line.
    osg::Node* node = osgEarth::Util::MapNodeHelper().load( arguments, &viewer );
    osgEarth::MapNode* mapNode = osgEarth::MapNode::get(node);
    if ( !mapNode )
        return usage(argv, "Unable to load an earth file.");
    viewer.setSceneData( node );

    // find the first layer with sequence control.
    bool found = false;
    osgEarth::ImageLayerVector layers;
    mapNode->getMap()->getImageLayers( layers );
    for( osgEarth::ImageLayerVector::const_iterator i = layers.begin(); i != layers.end(); ++i )
    {
        // get the sequence control interface for the layer, if there is one.
        osgEarth::ImageLayer*      layer = i->get();
        osgEarth::SequenceControl* sc = layer->getSequenceControl();
        if ( sc )
        {
            // found one, so make a label for it:
            ui::LabelControl* label = new ui::LabelControl("Time");
            label->setFontSize( 24.0f );
            label->setBackColor ( 0, 0, 0, 0.5 );
            label->setHorizAlign( ui::Control::ALIGN_CENTER );
            label->setVertAlign ( ui::Control::ALIGN_TOP );
            ui::ControlCanvas::getOrCreate( &viewer )->addControl( label );

            // make sure the sequence is playing:
            sc->playSequence();

            // add a callback to update the label with the sequence time id
            viewer.addUpdateOperation( new UpdateLabel(sc, label) );
            found = true;
            break;
        }
    }

    if (!found)
    {
        return usage(argv, "Your earth file does not contain any sequenced layers...bye!");
    }

    return viewer.run();
}
