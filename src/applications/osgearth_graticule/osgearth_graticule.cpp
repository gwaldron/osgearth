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
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include <osg/Notify>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgEarth/MapNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/MouseCoordsTool>
#include <osgEarthUtil/MGRSFormatter>
#include <osgEarthUtil/LatLongFormatter>

#include <osgEarthUtil/GeodeticGraticule>
#include <osgEarthUtil/MGRSGraticule>
#include <osgEarthUtil/UTMGraticule>
#include <osgEarthUtil/GraticuleNode>

using namespace osgEarth::Util;

int
usage( const std::string& msg )
{
    OE_NOTICE 
        << msg << std::endl
        << "USAGE: osgearth_graticule [options] file.earth" << std::endl
        << "   --geodetic            : display a geodetic (lat/long) graticule" << std::endl
        << "   --utm                 : display a UTM graticule" << std::endl
        << "   --mgrs                : display an MGRS graticule" << std::endl
        << "   --shader              : display a geodetic graticule using the glsl shaders" << std::endl;
    return -1;
}

//------------------------------------------------------------------------

struct ToggleGraticuleHandler : public ControlEventHandler
{
    ToggleGraticuleHandler( GraticuleNode* graticule ) : _graticule( graticule ) { }

    void onValueChanged( Control* control, bool value )
    {
        _graticule->setVisible( value );
    }

    GraticuleNode* _graticule;
};

struct OffsetGraticuleHandler : public ControlEventHandler
{
    OffsetGraticuleHandler( GraticuleNode* graticule, const osg::Vec2f& offset ) :
        _graticule( graticule ),
        _offset(offset)
    {
        //nop
    }

    void onClick( Control* control, const osg::Vec2f& pos, int mouseButtonMask )
    {
        _graticule->setCenterOffset( _graticule->getCenterOffset() + _offset );
    }

    osg::Vec2f _offset;
    GraticuleNode* _graticule;
};

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osgViewer::Viewer viewer(arguments);

    // parse command line:
    bool isUTM = arguments.read("--utm");
    bool isMGRS = arguments.read("--mgrs");
    bool isGeodetic = arguments.read("--geodetic");

    bool isShader = !isUTM && !isMGRS && !isGeodetic;

    // load the .earth file from the command line.
    MapNode* mapNode = MapNode::load( arguments );
    if ( !mapNode )
        return usage( "Failed to load a map from the .earth file" );

    // install our manipulator:
    viewer.setCameraManipulator( new EarthManipulator() );

    // root scene graph:
    osg::Group* root = new osg::Group();
    root->addChild( mapNode );

    GraticuleNode* graticuleNode = 0;

    Formatter* formatter = 0L;
    if ( isUTM )
    {
        UTMGraticule* gr = new UTMGraticule( mapNode );
        root->addChild( gr );
        formatter = new MGRSFormatter();
    }
    else if ( isMGRS )
    {
        MGRSGraticule* gr = new MGRSGraticule( mapNode );
        root->addChild( gr );
        formatter = new MGRSFormatter();
    }
    else if ( isGeodetic )
    {
        GeodeticGraticule* gr = new GeodeticGraticule( mapNode );
        GeodeticGraticuleOptions o = gr->getOptions();
        o.lineStyle()->getOrCreate<LineSymbol>()->stroke()->color().set(1,0,0,1);
        gr->setOptions( o );
        root->addChild( gr );
        formatter = new LatLongFormatter();
    }
    else
    {
        graticuleNode = new GraticuleNode( mapNode );
        root->addChild( graticuleNode );
    }

   
    // mouse coordinate readout:
    ControlCanvas* canvas = new ControlCanvas();
    root->addChild( canvas );
    VBox* vbox = new VBox();
    canvas->addControl( vbox );


    LabelControl* readout = new LabelControl();
    vbox->addControl( readout );

    if (graticuleNode)
    {
        HBox* toggleBox = vbox->addControl( new HBox() );
        toggleBox->setChildSpacing( 5 );
        CheckBoxControl* toggleCheckBox = new CheckBoxControl( true );
        toggleCheckBox->addEventHandler( new ToggleGraticuleHandler( graticuleNode ) );
        toggleBox->addControl( toggleCheckBox );
        LabelControl* labelControl = new LabelControl( "Show Graticule" );
        labelControl->setFontSize( 24.0f );
        toggleBox->addControl( labelControl  );

        HBox* offsetBox = vbox->addControl( new HBox() );
        offsetBox->setChildSpacing( 5 );
        osg::Vec4 activeColor(1,.3,.3,1);

        offsetBox->addControl(new LabelControl("Adjust Labels"));

        double adj = 10.0;
        LabelControl* left = new LabelControl("Left");
        left->addEventHandler(new OffsetGraticuleHandler(graticuleNode, osg::Vec2f(-adj, 0.0)) );
        offsetBox->addControl(left);
        left->setActiveColor(activeColor);

        LabelControl* right = new LabelControl("Right");
        right->addEventHandler(new OffsetGraticuleHandler(graticuleNode, osg::Vec2f(adj, 0.0)) );
        offsetBox->addControl(right);
        right->setActiveColor(activeColor);

        LabelControl* down = new LabelControl("Down");
        down->addEventHandler(new OffsetGraticuleHandler(graticuleNode, osg::Vec2f(0.0, -adj)) );
        offsetBox->addControl(down);
        down->setActiveColor(activeColor);

        LabelControl* up = new LabelControl("Up");
        up->addEventHandler(new OffsetGraticuleHandler(graticuleNode, osg::Vec2f(0.0, adj)) );
        offsetBox->addControl(up);
        up->setActiveColor(activeColor);


    }

    MouseCoordsTool* tool = new MouseCoordsTool( mapNode );
    tool->addCallback( new MouseCoordsLabelCallback(readout, formatter) );
    viewer.addEventHandler( tool );

    // finalize setup and run.
    viewer.setSceneData( root );
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgViewer::ThreadingHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    return viewer.run();
}
