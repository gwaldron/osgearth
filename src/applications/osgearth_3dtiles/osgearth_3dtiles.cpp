/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2018 Pelican Mapping
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
#include <osgEarth/ThreadingUtils>
#include <osgEarth/TDTiles>
#include <iostream>

#define LC "[3dtiles test] "

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


int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    // help?
    if ( arguments.read("--help") )
        return usage(argv[0]);

    // create a viewer:
    osgViewer::Viewer viewer(arguments);
    viewer.setCameraManipulator( new EarthManipulator(arguments) );
    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if ( node )
    {
        MapNode* mapNode = MapNode::get(node);

        const SpatialReference* srs = mapNode->getMapSRS()->getGeographicSRS();
        GeoPoint point(srs, -77.0, 34.5);
        osg::Matrix matrix;
        point.createLocalToWorld(matrix);
        
        TDTiles::Tile* tile = new TDTiles::Tile();
        tile->transform() = matrix;
        tile->content()->uri() = URI("../data/red_flag.osg.1000.scale");
        tile->geometricError() = 2000.0; // i.e., diameter of the node

        TDTiles::Tileset* tileset = new TDTiles::Tileset();
        tileset->root() = tile;

        OE_NOTICE << std::endl
            << tileset->getConfig().toJSON(true)
            << std::endl;

        TDTilesetGroup* root = new TDTilesetGroup();
        root->setTileset(tileset);
        root->setReadOptions(mapNode->getMap()->getReadOptions());
        mapNode->addChild(root);

        viewer.setSceneData( node );
        return viewer.run();
    }
    else
    {
        return usage(argv[0]);
    }

    return 0;
}