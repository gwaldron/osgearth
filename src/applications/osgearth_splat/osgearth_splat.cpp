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
#include <osgEarth/VirtualProgram>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>

#include <osgEarthSplat/LandCoverTerrainEffect>
#include <osgEarthSplat/SplatExtension>

#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Splat;

int
usage(const char* name)
{
    OE_NOTICE 
        << "\nUsage: " << name << " file.earth" << std::endl
        << MapNodeHelper().usage() << std::endl;

    return 0;
}

const char* color_landcover =
    "#version 120\n"
    "void color_landcover(inout vec4 color) \n"
    "{ \n"
    "    color = vec4(1,0,0,1);\n"
    "} \n";

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

        // Get the MapNode
        MapNode* mapNode = MapNode::findMapNode( node );

        // Find the Splat Extension
        SplatExtension* splatExtension = mapNode->getExtension<SplatExtension>();
        if (splatExtension)
        {
            OE_NOTICE << "Found Splat Extension" << std::endl;
        }

        LandCoverTerrainEffect* landCoverEffect = mapNode->getTerrainEngine()->getEffect<LandCoverTerrainEffect>();
        if (landCoverEffect)
        {
            OE_NOTICE << "Found landcover terrain effect" << std::endl;

            for (Zones::const_iterator zoneItr = landCoverEffect->getZones().begin();
                zoneItr != landCoverEffect->getZones().end();
                ++zoneItr)
            {
                // Get the StateSet for each of the LandCoverLayers
                for (LandCoverLayers::iterator landCoverItr = zoneItr->get()->getLandCover()->getLayers().begin();
                    landCoverItr != zoneItr->get()->getLandCover()->getLayers().end();
                    ++landCoverItr)
                {
                    // Get the stateset for the layer.
                    osg::StateSet* stateset = landCoverItr->get()->getOrCreateStateSet();

                    // Get the VirtualProgram for this layer.
                    VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);

                    // Make the "tree" layer all red.
                    if (landCoverItr->get()->getName() == "trees")
                    {                                         
                        vp->setFunction( "color_landcover", color_landcover, ShaderComp::LOCATION_FRAGMENT_LIGHTING);
                    }
                }
            }
        }

        viewer.setSceneData( node );
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
