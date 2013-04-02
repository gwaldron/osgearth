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
#include <osgEarth/VirtualProgram>
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
        << "           [--test5]    : Run the Program state set text \n"
        << std::endl;

    return -1;
}

//-------------------------------------------------------------------------

// Simple function injection test -- turns the earth gray with a haze.
namespace TEST_1
{
    char s_hazeVertShader[] =
        "#version " GLSL_VERSION_STR "\n"
        "varying vec3 v_pos; \n"
        "void setup_haze(inout vec4 VertexVIEW) \n"
        "{ \n"
        "    v_pos = vec3(VertexVIEW); \n"
        "} \n";

    char s_hazeFragShader[] =
        "#version " GLSL_VERSION_STR "\n"
        "varying vec3 v_pos; \n"
        "void apply_haze(inout vec4 color) \n"
        "{ \n"
        "    float dist = clamp( length(v_pos)/1e7, 0.0, 0.75 ); \n"
        "    color = mix(color, vec4(0.5, 0.5, 0.5, 1.0), dist); \n"
        "} \n";

    osg::StateAttribute* createHaze()
    {
        osgEarth::VirtualProgram* vp = new osgEarth::VirtualProgram();
        vp->setFunction( "setup_haze", s_hazeVertShader, osgEarth::ShaderComp::LOCATION_VERTEX_VIEW );
        vp->setFunction( "apply_haze", s_hazeFragShader, osgEarth::ShaderComp::LOCATION_FRAGMENT_LIGHTING );
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

namespace TEST_5
{
    char s_vert[] =
        "#version " GLSL_VERSION_STR "\n"
        "void main() { \n"
        "    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex; \n"
        "} \n";
    char s_frag[] =
        "#version " GLSL_VERSION_STR "\n"
        "void main() { \n"
        "    gl_FragColor = vec4(1.0,0.0,0.0,1.0); \n"
        "} \n";
    char s_vp[] =
        "#version " GLSL_VERSION_STR "\n"
        "void test( inout vec4 color ) { color = vec4(1.0,0.0,0.0,1.0); } \n";

    osg::Geode* makeGeom( float v )
    {
        osg::Geode* geode = new osg::Geode();
        osg::Geometry* geom = new osg::Geometry();
        osg::Vec3Array* verts = new osg::Vec3Array();
        verts->push_back( osg::Vec3(v-1, 0, 0) );
        verts->push_back( osg::Vec3(v+1, 0, 0) );
        verts->push_back( osg::Vec3(  0, 0, 2) );
        geom->setVertexArray( verts );
        geom->setUseDisplayList(false);
        geom->setUseVertexBufferObjects(true);
        osg::Vec4Array* colors = new osg::Vec4Array();
        colors->push_back( osg::Vec4(0,0,1,1) );
        geom->setColorArray(colors);
        geom->setColorBinding(osg::Geometry::BIND_OVERALL);
        geom->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES,0,3));
        geode->addDrawable(geom);
        return geode;
    }

    osg::Group* run()
    {
        osg::Node* n1 = makeGeom( -5 );
        osg::Node* n2 = makeGeom(  5 );

#if 0
        osg::Program* p = new osg::Program();
        p->addShader( new osg::Shader(osg::Shader::VERTEX,   s_vert) );
        p->addShader( new osg::Shader(osg::Shader::FRAGMENT, s_frag) );
#else
        VirtualProgram* p = new VirtualProgram();
        p->setFunction("test", s_vp, ShaderComp::LOCATION_FRAGMENT_LIGHTING);
#endif

        n1->getOrCreateStateSet()->setAttributeAndModes( p, 1 );

        osg::Group* root = new osg::Group();
        root->getOrCreateStateSet()->setRenderBinDetails( 0, "TraversalOrderBin" );
        root->getOrCreateStateSet()->setMode(GL_LIGHTING,0);

        root->addChild( n1 );
        root->addChild( n2 );

        return root;
    }
}


//-------------------------------------------------------------------------

int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osgViewer::Viewer viewer(arguments);

    bool test1 = arguments.read("--test1");
    bool test5 = arguments.read("--test5");
    bool ok    = test1 || test5; 

    if ( !ok )
    {
        return usage("");
    }

    if ( !test5 )
    {
        osg::Node* earthNode = osgDB::readNodeFiles( arguments );
        if (!earthNode)
        {
            return usage( "Unable to load earth model." );
        }

        viewer.setCameraManipulator( new osgEarth::Util::EarthManipulator() );

        LabelControl* label = new LabelControl();
        if ( !test5 )
            ControlCanvas::get(&viewer,true)->addControl(label);

        if ( test1 )
        {
            viewer.setSceneData( TEST_1::run(earthNode) );
            label->setText( "Function injection test: the map should appear hazy." );
        }
    }
    else // if ( test5 )
    {
        viewer.setSceneData( TEST_5::run() );
    }

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgViewer::ThreadingHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
