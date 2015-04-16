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

#include <osg/Notify>
#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/MapNode>
#include <osgEarth/ShaderLoader>
#include <osgEarth/VirtualProgram>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/Controls>
#include <osgEarthUtil/FeatureQueryTool>

#define LC "[feature_query] "

using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;

//-----------------------------------------------------------------------

/**
 * Creates a simple user interface for the demo.
 */
Container* createUI()
{
    VBox* vbox = new VBox();
    vbox->setVertAlign( Control::ALIGN_TOP );
    vbox->setHorizAlign( Control::ALIGN_LEFT );
    vbox->addControl( new LabelControl("Feature Query Demo", Color::Yellow) );
    vbox->addControl( new LabelControl("Click on a feature to see its attributes.") );
    return vbox;
}

//-----------------------------------------------------------------------

/**
 * Feautre highlighting shader snippets.
 */
const char* vertexShader = OE_MULTILINE(
    #version 130\n
    uniform int fid_highlight;
    in uint fid_attr;
    out vec4 mixColor;
    void featureQueryVertex(inout vec4 vertex)
    {
        if ( fid_attr == uint(fid_highlight) )
            mixColor = vec4(0, 1, 1, 0.5);
        else
            mixColor = vec4(0);
    }
);

const char* fragmentShader = OE_MULTILINE(
    in vec4 mixColor;
    void featureQueryFragment(inout vec4 color)
    {
        color.rgb = mix(color.rgb, mixColor.rgb, mixColor.a);
    }
);

//-----------------------------------------------------------------------

/**
 * A custom InputPredicate that determines whether to perform a feature query.
 * By default, a left-click starts a query. This custom predicate will query
 * upon each MOVE event; i.e. as you move the mouse without clicking.
 */
class HoverPredicate : public FeatureQueryTool::InputPredicate
{
public:
     bool accept(const osgGA::GUIEventAdapter& ea)
     {
         return ea.getHandled() == false && ea.getEventType() == ea.MOVE;
     }
};

//-----------------------------------------------------------------------

/**
 * Query callback that will install the highlighting shader and a control uniform.
 * The FeatureQueryTool will invoke the "onHit" method when it finds a feature
 * intersection, and the "onMiss" when an intersection fails to return a feature.
 */
class HighlightingCallback : public FeatureQueryTool::Callback
{
public:
    HighlightingCallback(osg::StateSet* stateSet)
    {
        _fidUniform = new osg::Uniform("fid_highlight", (int)-1);
        stateSet->addUniform( _fidUniform );

        VirtualProgram* vp = VirtualProgram::getOrCreate(stateSet);
        vp->setFunction( "featureQueryVertex", vertexShader, ShaderComp::LOCATION_VERTEX_MODEL );
        vp->setFunction( "featureQueryFragment", fragmentShader, ShaderComp::LOCATION_FRAGMENT_COLORING );
        vp->addBindAttribLocation( "fid_attr", FeatureSourceIndexNode::IndexAttrLocation );
    }

    void onHit(FeatureSourceIndexNode* index, FeatureID fid, const EventArgs& args)
    {
        _fidUniform->set( (int)fid );
    }

    void onMiss(const EventArgs& args)
    {
        _fidUniform->set( (int)-1 );
    }

private:
    osg::Uniform* _fidUniform;
};


//-----------------------------------------------------------------------

/**
 * Query Callback that displays the targeted feature's attributes in a
 * user interface grid control.
 */

class ReadoutCallback : public FeatureQueryTool::Callback
{
public:
    ReadoutCallback(ControlCanvas* container) : _lastFID( ~0 )
    {
        _grid = new Grid();
        _grid->setBackColor( Color(Color::Black,0.7f) );
        container->addControl( _grid );
    }

    void onHit(FeatureSourceIndexNode* index, FeatureID fid, const EventArgs& args)
    {
        const Feature* f = 0L;
        if ( !index || !index->getFeature(fid, f) )
        {
            _grid->setVisible( false );
        }
        else if ( !_grid->visible() || fid != _lastFID )
        {
            _grid->clearControls();
            unsigned r=0;

            _grid->setControl( 0, r, new LabelControl("FID", Color::Red) );
            _grid->setControl( 1, r, new LabelControl(Stringify()<<fid, Color::White) );
            ++r;

            const AttributeTable& attrs = f->getAttrs();
            for( AttributeTable::const_iterator i = attrs.begin(); i != attrs.end(); ++i, ++r )
            {
                _grid->setControl( 0, r, new LabelControl(i->first, 14.0f, Color::Yellow) );
                _grid->setControl( 1, r, new LabelControl(i->second.getString(), 14.0f, Color::White) );
            }
            if ( !_grid->visible() )
                _grid->setVisible( true );
        
            _lastFID = fid;
        }
        args._aa->requestRedraw();
    }

    void onMiss(const EventArgs& args)
    {
        _grid->setVisible(false);
        args._aa->requestRedraw();
    }

    Grid*     _grid;
    FeatureID _lastFID;
};

//------------------------------------------------------------------------

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    // a basic OSG viewer
    osgViewer::Viewer viewer(arguments);

    // install our default manipulator (do this before using MapNodeHelper)
    viewer.setCameraManipulator( new EarthManipulator() );

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags
    osg::Group* root = MapNodeHelper().load( arguments, &viewer, createUI() );
    if ( root )
    {
        viewer.setSceneData( root );

        // configure the near/far so we don't clip things that are up close
        viewer.getCamera()->setNearFarRatio(0.00002);

        MapNode* mapNode = MapNode::findMapNode( root );
        if ( mapNode )
        {
            // Install the query tool.
            FeatureQueryTool* tool = new FeatureQueryTool( mapNode );
            viewer.addEventHandler( tool );

            // Install our custom highlighting effect.
            osg::StateSet* modelsStateSet = mapNode->getModelLayerGroup()->getOrCreateStateSet();
            tool->addCallback( new HighlightingCallback(modelsStateSet) );

            // Install a readout for feature metadata.
            ControlCanvas* canvas = ControlCanvas::getOrCreate(&viewer);
            tool->addCallback( new ReadoutCallback(canvas) );

            // Install a query-on-hover predicate.
            tool->setInputPredicate( new HoverPredicate() );
        }

        return viewer.run();
    }
    else
    {
        OE_NOTICE 
            << "\nUsage: " << argv[0] << " file.earth" << std::endl
            << MapNodeHelper().usage() << std::endl;
    }
}
