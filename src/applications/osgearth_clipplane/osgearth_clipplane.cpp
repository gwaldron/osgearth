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

#include <osgViewer/Viewer>
#include <osgEarth/Notify>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarth/CullingUtils>
#include <osgEarth/VirtualProgram>
#include <osgEarth/MapNode>
#include <osgEarth/CullingUtils>
#include <osg/ClipNode>
#include <osg/ClipPlane>

#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;

/**
 * Demonstrates the use of an osg::ClipNode and osg::ClipPlane
 * objects to do clipping based on the visible horizon. This technique
 * can be useful when you want to draw geometry with depth testing
 * disabled, but you don't want it showing through the earth.
 *
 * Try this with "feature_clip_plane.earth" for a demo.
 */
int
usage(const char* name)
{
    OE_NOTICE 
        << "\nUsage: " << name << " feature_clip_plane.earth" << std::endl
        << MapNodeHelper().usage() << std::endl;

    return 0;
}

// Vertex shader to activate clip planes in GLSL:
const char* clipvs =
    "#version " GLSL_VERSION_STR "\n"
    "void oe_clip_vert(inout vec4 vertex_view) { \n"
    "   gl_ClipVertex = vertex_view; \n"
    "}\n";

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    // help?
    if ( arguments.read("--help") )
        return usage(argv[0]);

    // set up a viewer:
    osgViewer::Viewer viewer(arguments);
    viewer.getDatabasePager()->setUnrefImageDataAfterApplyPolicy( false, false );

    // install our default manipulator (do this before calling load)
    viewer.setCameraManipulator( new EarthManipulator() );

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Node* node = MapNodeHelper().load( arguments, &viewer );
    if ( node )
    {
        // Make a root group:
        osg::Group* root = new osg::Group();
        root->addChild( node );
        viewer.setSceneData( root );

        // Install a ClipNode. The ClipNode establishes positional state so it
        // doesn't need to parent anything. In this case it needs to be at the
        // top of the scene graph since out clip plane calculator assumes 
        // you're in world space.
        osg::ClipNode* clipNode = new osg::ClipNode();
        root->addChild( clipNode );
        
        // By default, the clip node will activate any clip planes you add to it
        // for its subgraph. Our clip node doesn't parent anything, but we include
        // this to demonstrate how you would disable that:
        clipNode->getOrCreateStateSet()->setMode(GL_CLIP_PLANE0, 0);
        
        // Create a ClipPlane we will use to clip to the visible horizon:
        osg::ClipPlane* cp = new osg::ClipPlane();
        clipNode->addClipPlane( cp );

        // This cull callback will recalcuate the position of the clipping plane
        // each frame based on the camera.
        const osgEarth::SpatialReference* srs = osgEarth::MapNode::get(node)->getMapSRS();
        clipNode->addCullCallback( new ClipToGeocentricHorizon(srs, cp) );

        // We also need a shader that will activate clipping in GLSL.
        VirtualProgram* vp = VirtualProgram::getOrCreate(root->getOrCreateStateSet());
        vp->setFunction("oe_clip_vert", clipvs, ShaderComp::LOCATION_VERTEX_VIEW, 0.5f);

        // Now everything is set up. The last thing to do is: anywhere in your
        // scene graph that you want to activate the clipping plane, set the 
        // corresponding mode on, like so:
        //
        // node->getOrCreateStateSet()->setMode(GL_CLIP_PLANE0, osg::StateAttribute::ON);
        //
        // If you are using symbology, you can use RenderSymbol::clipPlane(). Or in 
        // the earth file, for example:
        //
        //    render-depth-test: false;
        //    render-clip-plane: 0;

        return viewer.run();
    }
    else
    {
        return usage(argv[0]);
    }
}
