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
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/HorizonClipPlane>
#include <osgEarth/MapNode>

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

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    // help?
    if ( arguments.read("--help") )
        return usage(argv[0]);

    // set up a viewer:
    osgViewer::Viewer viewer(arguments);
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
        
        // Grab the map node.
        osgEarth::MapNode* mapNode = osgEarth::MapNode::get(node);

        // Create a horizon clip plane and install it on the map node.
        HorizonClipPlane* hcp = new HorizonClipPlane();
        mapNode->addCullCallback(hcp);

        // Optional: set a custom clip plane number. Default is zero.
        // If you call this, you must do so before calling installShaders.
        hcp->setClipPlaneNumber(0u);

        // Next, install the shaders somewhere. Usually this is the 
        // same node on which you installed the callback.
        hcp->installShaders(mapNode->getOrCreateStateSet());

        // Now everything is set up. The last thing to do is: anywhere in your
        // scene graph that you want to activate the clipping plane, set the 
        // corresponding mode on, like so:
        //
        // node->getOrCreateStateSet()->setMode(GL_CLIP_DISTANCE0, osg::StateAttribute::ON);
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
