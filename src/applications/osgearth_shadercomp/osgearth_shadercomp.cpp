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
 * This sample demonstrates a simple use of osgEarth's shader composition framework.
 *
 * By default, osgEarth uses GL shaders to render the terrain. Shader composition is a
 * mechanism by which you can inject custom shader code into osgEarth's shader program
 * pipeline. This gets around the problem of having to duplicate shader code in order 
 * to add functionality.
 */
#include <osg/Notify>
#include <osgDB/ReadFile>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarth/ShaderComposition>
#include <osgEarth/Registry>
#include <osgEarthUtil/Controls>

using namespace osgEarth;
using namespace osgEarth::Util::Controls;



int usage( const std::string& msg )
{    
    OE_NOTICE
        << msg << "\n\n"
        << "USAGE: osgearth_shadercomp <earthfile> \n"
        << "           [--test1]    : Run the function injection test \n"
        << "           [--test2]    : Run the shader nesting test \n"
        << "           [--test3]    : Run the OVERRIDE mode test \n"
        << "           [--test4]    : Run the PROTECTED mode test \n"
        << std::endl;

    return -1;
}

//-------------------------------------------------------------------------

// Simple function injection test -- turns the earth gray with a haze.
namespace TEST_1
{
    char s_hazeVertShader[] =
        "varying vec3 v_pos; \n"
        "void setup_haze() \n"
        "{ \n"
        "    v_pos = vec3(gl_ModelViewMatrix * gl_Vertex); \n"
        "} \n";

    char s_hazeFragShader[] =
        "varying vec3 v_pos; \n"
        "void apply_haze(inout vec4 color) \n"
        "{ \n"
        "    float dist = clamp( length(v_pos)/10000000.0, 0, 0.75 ); \n"
        "    color = mix(color, vec4(0.5, 0.5, 0.5, 1.0), dist); \n"
        "} \n";

    osg::StateAttribute* createHaze()
    {
        osgEarth::VirtualProgram* vp = new osgEarth::VirtualProgram();
        vp->setFunction( "setup_haze", s_hazeVertShader, osgEarth::ShaderComp::LOCATION_VERTEX_POST_LIGHTING );
        vp->setFunction( "apply_haze", s_hazeFragShader, osgEarth::ShaderComp::LOCATION_FRAGMENT_POST_LIGHTING );
        return vp;
    }

    osg::Group* run( osg::Node* earth )
    {    
        osg::Group* g = new osg::Group();
        g->addChild( earth );
        g->getOrCreateStateSet()->setAttributeAndModes( createHaze(), osg::StateAttribute::ON );
        return g;
    }
}

//-------------------------------------------------------------------------

// TESTS natual VP shader nesting. Even though a reddening shader is applied,
// the built-in applyLighting shader should override it and take precedence.
// Normal lighting should be used.
namespace TEST_2
{
    char s_source[] =
        "void osgearth_frag_applyTexturing( inout vec4 color ) { \n"
        "    color.r = 1.0; \n"
        "} \n";

    osg::Group* run( osg::Node* earth )
    {    
        osg::Group* g1 = new osg::Group();
        g1->addChild( earth );

        osgEarth::VirtualProgram* vp = new osgEarth::VirtualProgram();
        vp->setShader( "osgearth_frag_applyTexturing", new osg::Shader(osg::Shader::FRAGMENT, s_source) );
        g1->getOrCreateStateSet()->setAttributeAndModes( vp, osg::StateAttribute::ON );

        return g1;
    }
}

//-------------------------------------------------------------------------

// TESTS the use of the OVERRIDE qualifier. Same as TEST_2 shader, but the "reddening"
// shader should pre-empty the built-in shader because of the OVERRIDE mode.
namespace TEST_3
{
    char s_source[] =
        "void osgearth_frag_applyTexturing( inout vec4 color ) { \n"
        "    color = vec4(1.0, 0.0, 0.0, 1.0); \n"
        "} \n";

    osg::Group* run( osg::Node* earth )
    {    
        osg::Group* g1 = new osg::Group();
        g1->addChild( earth );

        osgEarth::VirtualProgram* vp = new osgEarth::VirtualProgram();

        // NOTE the use of OVERRIDE; this prevents subordinate VPs from replacing the 
        // function (unless marked as PROTECTED).
        vp->setShader( 
            "osgearth_frag_applyTexturing",
            new osg::Shader(osg::Shader::FRAGMENT, s_source),
            osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE );

        g1->getOrCreateStateSet()->setAttributeAndModes( vp, osg::StateAttribute::ON );

        return g1;
    }
}

//-------------------------------------------------------------------------

// TESTS the use of the PROTECTED qualifier. Same as TEST_3 shader, but the "reddening"
// shader (which has its OVERRIDE mode set) has a subordinate shader with its PROTECTED
// mode set, which nullifies the effect of the OVERRIDE.
namespace TEST_4
{
    char s_source_1[] =
        "void osgearth_frag_applyTexturing( inout vec4 color ) { \n"
        "    color = vec4(1.0, 0.0, 0.0, 1.0); \n"
        "} \n";
    
    char s_source_2[] =
        "void osgearth_frag_applyTexturing( inout vec4 color ) { \n"
        "    color = vec4(0.0, 0.0, 1.0, 1.0); \n"
        "} \n";

    osg::Group* run( osg::Node* earth )
    {    
        // Insert a Shader in OVERRIDE mode:
        osg::Group* g1 = new osg::Group();
        osgEarth::VirtualProgram* vp1 = new osgEarth::VirtualProgram();
        vp1->setShader( 
            "osgearth_frag_applyTexturing",
            new osg::Shader(osg::Shader::FRAGMENT, s_source_1),
            osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE );
        g1->getOrCreateStateSet()->setAttributeAndModes( vp1, osg::StateAttribute::ON );

        // Insert a subordinate shader in PROTECTED mode:
        osg::Group* g2 = new osg::Group();
        osgEarth::VirtualProgram* vp2 = new osgEarth::VirtualProgram();
        vp2->setShader(
            "osgearth_frag_applyTexturing",
            new osg::Shader(osg::Shader::FRAGMENT, s_source_2),
            osg::StateAttribute::ON | osg::StateAttribute::PROTECTED | osg::StateAttribute::OVERRIDE );
        g2->getOrCreateStateSet()->setAttributeAndModes( vp2, osg::StateAttribute::ON );

        g1->addChild( g2 );
        g2->addChild( earth );
        return g1;
    }
}


//-------------------------------------------------------------------------

int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    bool test1 = arguments.read("--test1");
    bool test2 = arguments.read("--test2");
    bool test3 = arguments.read("--test3");
    bool test4 = arguments.read("--test4");

    osg::Node* earthNode = osgDB::readNodeFiles( arguments );
    if (!earthNode)
    {
        return usage( "Unable to load earth model." );
    }

    osgViewer::Viewer viewer(arguments);
    viewer.setCameraManipulator( new osgEarth::Util::EarthManipulator() );

    LabelControl* label = new LabelControl();
    ControlCanvas::get(&viewer,true)->addControl(label);

    if ( test1 )
    {
        viewer.setSceneData( TEST_1::run(earthNode) );
        label->setText( "Function injection test: the map should appear hazy." );
    }
    else if ( test2 )
    {
        viewer.setSceneData( TEST_2::run(earthNode) );
        label->setText( "Shader nesting test: the map should appear normally." );
    }
    else if ( test3 )
    {
        viewer.setSceneData( TEST_3::run(earthNode) );
        label->setText( "Shader override test: the map should appear red." );
    }
    else if ( test4 )
    {
        viewer.setSceneData( TEST_4::run(earthNode) );
        label->setText( "Shader protected test: the map should appear blue." );
    }
    else
    {
        return usage("Please select a test to run.");
    }

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgViewer::ThreadingHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
