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
 * This sample shows how to apply a detail texture to osgEarth's terrain.
 */
#include <osg/Notify>
#include <osgViewer/Viewer>
#include <osgEarth/VirtualProgram>
#include <osgEarth/Registry>
#include <osgEarth/URI>
#include <osgEarth/TerrainEngineNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/Controls>
#include <osg/Image>
#include <osg/Texture>
#include <osg/Uniform>

using namespace osgEarth;
using namespace osgEarth::Util;

//-------------------------------------------------------------------------

const char* vertexShader =
    "uniform vec3  oe_tilekey; \n"
    "varying vec4  oe_layer_tilec; \n"
    "uniform float dt_L0; \n"
    "varying vec2  dt_texcoord; \n"

    "int dt_ipow(in int x, in int y) { \n"
    "   int r = 1; \n"
    "   while( y > 0 ) { \n"
    "       r *= x; \n"
    "       --y; \n"
    "   } \n"
    "   return r; \n"
    "}\n"

    "void setupDetailTex(inout vec4 VertexMODEL) \n"
    "{ \n"
    "    float L = oe_tilekey.z; \n"

    "    float dL = L - dt_L0; \n"
    "    float twoPowDeltaL = float(dt_ipow(2, int(abs(dL)))); \n"
    "    float factor = dL >= 0.0 ? twoPowDeltaL : 1.0/twoPowDeltaL; \n"

    "    vec2 a = floor(oe_tilekey.xy / factor); \n"
    "    vec2 b = a * factor; \n"
    "    vec2 c = (a+1.0) * factor; \n"
    "    vec2 offset = (oe_tilekey.xy-b)/(c-b); \n"
    "    vec2 scale = vec2(1.0/factor); \n"

    "    dt_texcoord = (oe_layer_tilec.st * scale) + offset; \n"
    "} \n";

const char* fragmentShader =
    "uniform vec3      oe_tilekey; \n"
    "uniform float     dt_L0; \n"
    "uniform sampler2D dt_tex; \n"
    "uniform float     dt_effect; \n"
    "varying vec2      dt_texcoord; \n"
    "void applyDetailTex(inout vec4 color) \n"
    "{ \n"
    "    if ( oe_tilekey.z >= dt_L0 ) \n"
    "    { \n"
    "        vec4 texel = texture2D(dt_tex, dt_texcoord); \n"
    "        if ( oe_tilekey.z >= dt_L0+3.0 ) \n"
    "        { \n"
    "            texel = mix(texel, texture2D(dt_tex, dt_texcoord*12.0), 0.5); \n"
    "        } \n"
    "        texel.rgb -= 0.5; \n"
    "        color.rgb = clamp( color.rgb + (texel.rgb*dt_effect), 0.0, 1.0 ); \n"
    "    } \n"
    "} \n";


// Build the stateset necessary for scaling elevation data.
osg::StateSet* createStateSet()
{
    osg::StateSet* stateSet = new osg::StateSet();

    VirtualProgram* vp = new VirtualProgram();
    vp->setFunction( "setupDetailTex", vertexShader, ShaderComp::LOCATION_VERTEX_MODEL );
    vp->setFunction( "applyDetailTex", fragmentShader, ShaderComp::LOCATION_FRAGMENT_COLORING );
    stateSet->setAttributeAndModes( vp, osg::StateAttribute::ON );
    stateSet->getOrCreateUniform( "dt_effect", osg::Uniform::FLOAT )->set( 1.0f );

    return stateSet;
};


int 
usage(const char* msg)
{
    OE_NOTICE << msg << std::endl;
    return 0;
}


int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc, argv);

    // create a viewer:
    osgViewer::Viewer viewer(arguments);

    // Tell osgEarth to use the "mp" terrain driver by default.
    osgEarth::Registry::instance()->setDefaultTerrainEngineDriverName( "mp" );

    // install our default manipulator (do this before calling load)
    viewer.setCameraManipulator( new EarthManipulator() );

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Node* node = MapNodeHelper().load( arguments, &viewer );
    if ( node )
    {
        MapNode* mapNode = MapNode::findMapNode(node);
        if ( !mapNode )
            return -1;

        // install the shader program and install our controller uniform:
        osg::Group* root = new osg::Group();
        osg::StateSet* ss = createStateSet();
        root->setStateSet( ss );

        osg::Image* image = URI("../data/noise3.png").getImage();
        if ( !image )
            return usage( "Failed to load image" );

        osg::Texture2D* dtex = new osg::Texture2D(image);
        dtex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
        dtex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
        ss->setTextureAttributeAndModes(1, dtex, 1);
        ss->getOrCreateUniform("dt_tex", osg::Uniform::SAMPLER_2D)->set(1);
        ss->getOrCreateUniform("dt_L0", osg::Uniform::FLOAT)->set(10.0f);

        root->addChild( node );

        viewer.setSceneData( root );
        viewer.run();
    }
    else
    {
        return usage("no earth file");
    }

    return 0;
}
