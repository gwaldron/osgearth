/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2015 Pelican Mapping
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

/**
 * This app demonstrates the use of TerrainEngine::createTile(), which lets
 * you create the geometry for an arbitrary terrain tile that you can use for
 * external purposes.
 */

#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgUtil/LineSegmentIntersector>
#include <osgEarth/MapNode>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/ElevationQuery>
#include <osgEarth/StringUtils>
#include <osgEarth/Terrain>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/Controls>
#include <osgEarthUtil/LatLongFormatter>
#include <osg/TriangleFunctor>
#include <osgDB/WriteFile>
#include <iomanip>

using namespace osgEarth;
using namespace osgEarth::Util;

static MapNode*       s_mapNode     = 0L;
static osg::Group*    s_root        = 0L;

struct CollectTriangles
{
    CollectTriangles()
    {
        verts = new osg::Vec3Array();
    }
    inline void operator () (const osg::Vec3& v1,const osg::Vec3& v2,const osg::Vec3& v3, bool treatVertexDataAsTemporary)
    {
        verts->push_back(v1);
        verts->push_back(v2);
        verts->push_back(v3);
    }

    osg::ref_ptr< osg::Vec3Array > verts;
};

struct CollectTrianglesVisitor : public osg::NodeVisitor
{
    CollectTrianglesVisitor():
        osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
    {
        _vertices = new osg::Vec3dArray();
    }

    void apply(osg::Transform& transform)
    {
        osg::Matrix matrix;
        if (!_matrixStack.empty()) matrix = _matrixStack.back();

        transform.computeLocalToWorldMatrix(matrix,this);

        pushMatrix(matrix);

        traverse(transform);

        popMatrix();
    }

    void apply(osg::Geode& geode)
    {
        for(unsigned int i=0; i<geode.getNumDrawables(); ++i)
        {
            osg::TriangleFunctor<CollectTriangles> triangleCollector;
            geode.getDrawable(i)->accept(triangleCollector);
            for (unsigned int j = 0; j < triangleCollector.verts->size(); j++)
            {
                osg::Matrix& matrix = _matrixStack.back();
                _vertices->push_back((*triangleCollector.verts)[j] * matrix);
            }
        }
    }

    osg::Node* buildNode()
    {
        osg::Geometry* geom = new osg::Geometry;
        osg::Vec3Array* verts = new osg::Vec3Array;
        geom->setVertexArray(verts);
        osg::Vec4ubArray* colors = new osg::Vec4ubArray(1);
        (*colors)[0] = osg::Vec4ub(255,0,0,255);
        geom->setColorArray(colors);
        geom->setColorBinding(osg::Geometry::BIND_OVERALL);

        bool first = true;
        osg::Vec3d anchor;

        for (unsigned int i = 0; i < _vertices->size(); i++)
        {
            if (first)
            {
                anchor = (*_vertices)[i];
                first = false;
            }

            verts->push_back((*_vertices)[i] - anchor);
        }

        OSG_NOTICE << "Building scene with " << verts->size() << " verts" << std::endl;

        osg::MatrixTransform* mt = new osg::MatrixTransform;
        mt->setMatrix(osg::Matrixd::translate(anchor));

        osg::Geode* geode = new osg::Geode;
        geode->addDrawable(geom);
        geom->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, 0, verts->size()));
        mt->addChild(geode);
        mt->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
        mt->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        return mt;
    }

    inline void pushMatrix(osg::Matrix& matrix) { _matrixStack.push_back(matrix); }

    inline void popMatrix() { _matrixStack.pop_back(); }

    typedef std::vector<osg::Matrix> MatrixStack;
    osg::ref_ptr<osg::Vec3dArray>  _vertices;
    MatrixStack _matrixStack;
};





// An event handler that will create a tile that can be used for intersections
struct CreateTileHandler : public osgGA::GUIEventHandler
{
    CreateTileHandler()
    {
    }

    void update( float x, float y, osgViewer::View* view )
    {
        bool yes = false;

        // look under the mouse:
        osg::Vec3d world;
        osgUtil::LineSegmentIntersector::Intersections hits;
        if ( view->computeIntersections(x, y, hits) )
        {
            world = hits.begin()->getWorldIntersectPoint();

            // convert to map coords:
            GeoPoint mapPoint;
            mapPoint.fromWorld( s_mapNode->getMapSRS(), world );

            TileKey key = s_mapNode->getMap()->getProfile()->createTileKey(mapPoint.x(), mapPoint.y(), 13);
            OE_NOTICE << "Creating tile " << key.str() << std::endl;
            osg::ref_ptr<osg::Node> node = s_mapNode->getTerrainEngine()->createTile(key);
            if (node.valid())
            {
                OE_NOTICE << "Created tile for " << key.str() << std::endl;
                CollectTrianglesVisitor v;
                node->accept(v);

                osg::ref_ptr<osg::Node> output = v.buildNode();
                osgDB::writeNodeFile( *output.get(), "createtile.osgt" );
                OE_NOTICE << "Wrote tile to createtile.osgt\n";
                //osgDB::writeNodeFile(v.buildNode(
                //s_root->addChild(v.buildNode());
            }
            else
            {
                OE_NOTICE << "Failed to create tile for " << key.str() << std::endl;
            }
        }
    }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH && ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
        {
            osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());
            update( ea.getX(), ea.getY(), view );
        }

        return false;
    }
};


int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    osgViewer::Viewer viewer(arguments);

    s_mapNode = MapNode::load(arguments);
    if ( !s_mapNode )
    {
        OE_WARN << "Unable to load earth file." << std::endl;
        return -1;
    }

    s_root = new osg::Group();
    viewer.setSceneData( s_root );

    // install the programmable manipulator.
    viewer.setCameraManipulator( new osgEarth::Util::EarthManipulator() );

    // The MapNode will render the Map object in the scene graph.
    s_root->addChild( s_mapNode );

    // An event handler that will respond to mouse clicks:
    viewer.addEventHandler( new CreateTileHandler() );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
