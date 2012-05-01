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

osg::StateAttribute* createHaze();

int usage( const std::string& msg )
{    
    OE_NOTICE
        << msg << std::endl
        << "USAGE: osgearth_shadercomp <earthfile>" << std::endl;

    return -1;
}

int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    osg::Node* earthNode = osgDB::readNodeFiles( arguments );
    if (!earthNode)
    {
        return usage( "Unable to load earth model." );
    }

    osg::Group* root = new osg::Group();
    root->addChild( earthNode );
    
    osgViewer::Viewer viewer(arguments);
    viewer.setCameraManipulator( new osgEarth::Util::EarthManipulator() );
    viewer.setSceneData( root );

    // inject the haze shader components:
    root->getOrCreateStateSet()->setAttributeAndModes( createHaze(), osg::StateAttribute::ON );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgViewer::ThreadingHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}

//-------------------------------------------------------------------------

static char s_hazeVertShader[] =
    "varying vec3 v_pos; \n"
    "void setup_haze() \n"
    "{ \n"
    "    v_pos = vec3(gl_ModelViewMatrix * gl_Vertex); \n"
    "} \n";

static char s_hazeFragShader[] =
    "varying vec3 v_pos; \n"
    "void apply_haze(inout vec4 color) \n"
    "{ \n"
    "    float dist = clamp( length(v_pos)/10000000.0, 0, 0.75 ); \n"
    "    color = mix(color, vec4(0.5, 0.5, 0.5, 1.0), dist); \n"
    "} \n";


osg::StateAttribute*
createHaze()
{
    osgEarth::VirtualProgram* vp = new osgEarth::VirtualProgram();

    vp->setFunction( "setup_haze", s_hazeVertShader, osgEarth::ShaderComp::LOCATION_VERTEX_POST_LIGHTING );
    vp->setFunction( "apply_haze", s_hazeFragShader, osgEarth::ShaderComp::LOCATION_FRAGMENT_POST_LIGHTING );

    return vp;
}
