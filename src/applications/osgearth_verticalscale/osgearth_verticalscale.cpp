/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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

/**
 * This sample shows how to use osgEarth's built-in elevation data attributes
 * to adjust the terrain's vertical scale in real time.
 */
#include <osg/Notify>
#include <osgViewer/Viewer>
#include <osgEarth/ShaderComposition>
#include <osgEarth/Registry>
#include <osgEarth/TerrainEngineNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/Controls>

using namespace osgEarth;
using namespace osgEarth::Util;

//-------------------------------------------------------------------------

// In the vertex shader, we use a vertex attribute that's genreated by the
// terrain engine. In this example it's called "osgearth_elevData" but you 
// can give it any name you want, as long as it's bound to the proper
// attribute location (see code). 
//
// The attribute contains a vec4 which holds the unit "up vector" in 
// indexes[0,1,2] and the original raw height in index[3].
//
// Here, we use the vertical scale uniform to move the vertex up or down
// along its up vector, thereby scaling the terrain's elevation. The code
// is intentionally verbose for clarity.

const char* vertexShader =
    "attribute vec4  osgearth_elevData; \n"
    "uniform   float verticalScale;     \n"

    "void applyVerticalScale() \n"
    "{ \n"
    "    vec3  upVector = osgearth_elevData.xyz;                     \n"
    "    float elev     = osgearth_elevData.w;                       \n"
    "    vec3  offset   = upVector * elev * (verticalScale - 1.0);   \n"
    "    vec4  vertex   = gl_Vertex + vec4(offset/gl_Vertex.w, 0.0); \n"
    "    gl_Position    = gl_ModelViewProjectionMatrix * vertex;     \n"
    "} \n";


// Build the stateset necessary for scaling elevation data.
osg::StateSet* createStateSet()
{
    osg::StateSet* stateSet = new osg::StateSet();

    // Install the shaders. We also bind osgEarth's elevation data attribute, which the 
    // terrain engine automatically generates at the specified location.
    VirtualProgram* vp = new VirtualProgram();
    vp->installDefaultColoringAndLightingShaders();
    vp->setFunction( "applyVerticalScale", vertexShader, ShaderComp::LOCATION_VERTEX_PRE_LIGHTING );
    vp->addBindAttribLocation( "osgearth_elevData", osg::Drawable::ATTRIBUTE_6 );
    stateSet->setAttributeAndModes( vp, osg::StateAttribute::ON );

    return stateSet;
};


// Build a slider to adjust the vertical scale
osgEarth::Util::Controls::Control* createUI( osg::Uniform* scaler )
{
    using namespace osgEarth::Util::Controls;

    struct ApplyVerticalScale : public ControlEventHandler {
        osg::Uniform* _u;
        ApplyVerticalScale(osg::Uniform* u) : _u(u) { }
        void onValueChanged(Control*, float value) {
            _u->set( value );
        }
    };

    HBox* hbox = new HBox();
    hbox->setChildVertAlign( Control::ALIGN_CENTER );
    hbox->addControl( new LabelControl("Scale:") );
    HSliderControl* slider = hbox->addControl( new HSliderControl(0.0, 5.0, 1.0, new ApplyVerticalScale(scaler)) );
    slider->setHorizFill( true, 200 );
    hbox->addControl( new LabelControl(slider) );

    return hbox;
}


int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc, argv);

    // create a viewer:
    osgViewer::Viewer viewer(arguments);

    // Tell osgEarth to use the "quadtree" terrain driver by default.
    // Elevation data attribution is only available in this driver!
    osgEarth::Registry::instance()->setDefaultTerrainEngineDriverName( "quadtree" );

    // install our default manipulator (do this before calling load)
    viewer.setCameraManipulator( new EarthManipulator() );

    osg::Uniform* verticalScale = new osg::Uniform(osg::Uniform::FLOAT, "verticalScale");
    verticalScale->set( 1.0f );
    osgEarth::Util::Controls::Control* ui = createUI( verticalScale );

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Node* node = MapNodeHelper().load( arguments, &viewer, ui );
    if ( node )
    {
        MapNode* mapNode = MapNode::findMapNode(node);
        if ( !mapNode )
            return -1;

        if ( mapNode->getMap()->getNumElevationLayers() == 0 )
            OE_WARN << "No elevation layers! Scaling will be very boring." << std::endl;

        // install the shader program and install our controller uniform:
        osg::Group* root = new osg::Group();
        root->setStateSet( createStateSet() );
        root->getStateSet()->addUniform( verticalScale );
        root->addChild( node );

        viewer.setSceneData( root );
        viewer.run();
    }
    else
    {
        OE_NOTICE 
            << "\nUsage: " << argv[0] << " file.earth" << std::endl
            << MapNodeHelper().usage() << std::endl;
    }

    return 0;
}
