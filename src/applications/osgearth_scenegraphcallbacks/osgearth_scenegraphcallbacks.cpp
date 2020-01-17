/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2019 Pelican Mapping
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
#include <osgEarth/MapNode>
#include <osgEarth/SceneGraphCallback>

#define LC "[scenegraphcallbacks] "

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

class MyCallback : public SceneGraphCallback
{
public:
    void onPreMergeNode(osg::Node* node, osg::Object* sender)
    {
        Layer* layer = static_cast<Layer*>(sender);
        OE_NOTICE << "Layer " << layer->getName() << " pre-merge node " << node->getName() << std::endl;
    }

    void onPostMergeNode(osg::Node* node, osg::Object* sender)
    {
        Layer* layer = static_cast<Layer*>(sender);
        OE_NOTICE << "Layer " << layer->getName() << " post-merge node " << node->getName() << std::endl;
    }

    void onRemoveNode(osg::Node* node, osg::Object* sender)
    {
        Layer* layer = static_cast<Layer*>(sender);
        OE_NOTICE << "Layer " << layer->getName() << " remove node " << node->getName() << std::endl;
    }
};

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osgViewer::Viewer viewer(arguments);
    viewer.setCameraManipulator( new EarthManipulator(arguments) );

    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if ( node )
    {
        MapNode* mapNode = MapNode::get(node);
        if (!mapNode)
            return -1;

        MyCallback* myCallback = new MyCallback();

        // Install a callback on each loaded layer.
        LayerVector layers;
        mapNode->getMap()->getLayers(layers);
        for (LayerVector::iterator layer = layers.begin(); layer != layers.end(); ++layer)
        {
            layer->get()->getSceneGraphCallbacks()->add( myCallback );
        }

        viewer.setSceneData( node );
        return viewer.run();
    }
    else
    {
        return usage(argv[0]);
    }

    return 0;
}
