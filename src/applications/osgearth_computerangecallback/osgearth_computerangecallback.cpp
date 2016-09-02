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
#include <osgEarth/TerrainEngineNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>

#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;

int
usage(const char* name)
{
    OE_NOTICE 
        << "\nUsage: " << name << " file.earth" << std::endl
        << MapNodeHelper().usage() << std::endl;

    return 0;
}

struct MyComputeRangeCallback : public osgEarth::ComputeRangeCallback
{
    virtual float operator()(osg::Node* node, osg::NodeVisitor& nv)
    {
        osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);
        if (cv)
        {
            // This code assumes that the range_mode is PIXEL_SIZE_ON_SCREEN and computes the pixel size on screen 
            // without just using the vertical field of view.
            float distance = nv.getDistanceToViewPoint(node->getBound().center(),true);
            float radius = node->getBound().radius();

            double fov, ar, near, far;
            cv->getCurrentCamera()->getProjectionMatrixAsPerspective(fov, ar, near,far);

            osg::Viewport* viewPort = cv->getCurrentCamera()->getViewport();
            double angularSize = osg::RadiansToDegrees( 2.0*atan(radius/distance) );
            double dpp = osg::maximum(fov, 1.0e-17) / viewPort->height();
            float pixelSize = angularSize / dpp;
            return pixelSize;
        }

        // Fall back to the base calculation.
        return -1.0;
    }
};

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    // help?
    if ( arguments.read("--help") )
        return usage(argv[0]);

    // create a viewer:
    osgViewer::Viewer viewer(arguments);

    // Tell the database pager to not modify the unref settings
    viewer.getDatabasePager()->setUnrefImageDataAfterApplyPolicy( false, false );

    // thread-safe initialization of the OSG wrapper manager. Calling this here
    // prevents the "unsupported wrapper" messages from OSG
    osgDB::Registry::instance()->getObjectWrapperManager()->findWrapper("osg::Image");

    // install our default manipulator (do this before calling load)
    viewer.setCameraManipulator( new EarthManipulator(arguments) );

    // disable the small-feature culling
    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

    // set a near/far ratio that is smaller than the default. This allows us to get
    // closer to the ground without near clipping. If you need more, use --logdepth
    viewer.getCamera()->setNearFarRatio(0.0001);


    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Node* node = MapNodeHelper().load( arguments, &viewer );
    if ( node )
    {
        viewer.setSceneData( node );

        // Set a custom ComputeRangeCallback on the MapNode
        MapNode* mapNode = MapNode::findMapNode(node);
        mapNode->getTerrainEngine()->setComputeRangeCallback( new MyComputeRangeCallback() );


        while(!viewer.done())
        {
            viewer.frame();
        }
    }
    else
    {
        return usage(argv[0]);
    }
}
