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
#include <osgEarth/StringUtils>
#include <osgEarth/Terrain>
#include <osgEarth/GeoTransform>
#include <osgEarth/EarthManipulator>
#include <osgEarth/Controls>
#include <osgEarth/ExampleResources>
#include <osg/TriangleFunctor>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <iomanip>
#include <sstream>

using namespace osgEarth;
using namespace osgEarth::Util;

static MapNode*       s_mapNode     = nullptr;
static osg::Group*    s_root        = nullptr;
static osg::ref_ptr< osg::Node >  marker = osgDB::readNodeFile("../data/red_flag.osg");

TileKey makeTileKey(const std::string& str, const Profile* profile)
{
    std::istringstream stream(str);
    unsigned lod = 0, x = 0, y = 0;
    stream >> lod;
    if (stream.fail() || stream.peek() != '/')
        return TileKey();
    stream.ignore(1);
    stream >> x;
    if (stream.fail() || stream.peek() != '/')
        return TileKey();
    stream.ignore(1);
    stream >> y;
    if (stream.fail())
        return TileKey();
    return TileKey(lod, x, y, profile);
}

struct CollectTriangles
{
    CollectTriangles()
    {
        verts = new osg::Vec3Array();
    }
#if OSG_VERSION_LESS_THAN(3,5,6)
    inline void operator () (const osg::Vec3& v1,const osg::Vec3& v2,const osg::Vec3& v3, bool treatVertexDataAsTemporary)
#else
    inline void operator () (const osg::Vec3& v1,const osg::Vec3& v2,const osg::Vec3& v3)
#endif
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

    void apply(osg::Drawable& drawable) override
    {
        osg::TriangleFunctor<CollectTriangles> triangleCollector;
        drawable.accept(triangleCollector);
        for (unsigned int j = 0; j < triangleCollector.verts->size(); j++)
        {
            osg::Matrix& matrix = _matrixStack.back();
            osg::Vec3d v = (*triangleCollector.verts)[j];
            _vertices->push_back(v * matrix);
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
        geode->setCullingActive( false );
        geom->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, 0, verts->size()));
        mt->addChild(geode);
        //mt->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
        mt->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        mt->getOrCreateStateSet()->setRenderBinDetails(99, "RenderBin");

        osg::BoundingSphere bs = mt->getBound();
        bs.radius() = bs.radius() * 100;
        mt->setInitialBound(bs);
        mt->dirtyBound();

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
        : _tileLOD(15), _refLOD(0), _tileFlags(TerrainEngineNode::CREATE_TILE_INCLUDE_ALL)
    {
    }


    CreateTileHandler(osg::ArgumentParser& arguments)
        : CreateTileHandler()
    {
        // new tile LOD for interactive use
        while (arguments.read("--tilelod", _tileLOD))
            ;
        // Create a standalone tile explicitly, for debugging various problems
        while (arguments.read("--tilekey", _keyString))
            ;
        while (arguments.read("--reflod", _refLOD))
            ;
        bool tilesWithMasks = false;
        bool tilesWithoutMasks = false;
        while (arguments.read("--with-masks", tilesWithMasks))
            ;
        while (arguments.read("--without-masks", tilesWithoutMasks))
            ;
        if (tilesWithMasks)
        {
            _tileFlags = TerrainEngineNode::CREATE_TILE_INCLUDE_TILES_WITH_MASKS;
        }
        if (tilesWithoutMasks)
        {
            _tileFlags = TerrainEngineNode::CREATE_TILE_INCLUDE_TILES_WITHOUT_MASKS;
        }
    }
    
    osg::Node* makeCustomTile(const TileKey& key)
    {
        Map* map = s_mapNode->getMap();
        osg::ref_ptr<TerrainTileModel> model
            = s_mapNode->getTerrainEngine()->createTileModel(map, key, _manifest, nullptr);
        osg::ref_ptr<osg::Node> node =
            s_mapNode->getTerrainEngine()->createStandaloneTile(model.get(), _tileFlags, _refLOD, key);
        if (node.valid())
        {
            // Extract the triangles from the node that was created
            // and do our own rendering.
            // Simulates what you would do when passing in the triangles to a physics engine.
            OE_NOTICE << "Created tile for " << key.str() << std::endl;
            CollectTrianglesVisitor v;
            node->accept(v);
            return v.buildNode();
        }
        else
        {
            return nullptr;
        }
    }

    void update( float x, float y, osgViewer::View* view )
    {
        // look under the mouse:
        osg::Vec3d world;
        osgUtil::LineSegmentIntersector::Intersections hits;
        TileKey key;
        GeoPoint mapPoint;

        if ( view->computeIntersections(x, y, hits) )
        {
            world = hits.begin()->getWorldIntersectPoint();

            // convert to map coords:
            mapPoint.fromWorld( s_mapNode->getMapSRS(), world );

            // Depending on the level of detail key you request, you will get a mesh that should line up exactly with the highest resolution mesh that the terrain engine will draw.
            // At level 15 that is a 257x257 heightfield.  If you select a higher lod, the mesh will be less dense.
            key = s_mapNode->getMap()->getProfile()->createTileKey(mapPoint.x(), mapPoint.y(), _tileLOD);
            OE_NOTICE << "Creating tile " << key.str() << std::endl;
        }
        osg::ref_ptr<osg::Node> node;
        if (key.valid() && (node = makeCustomTile(key)))
        {
            if (_node.valid())
            {
                s_root->removeChild( _node.get() );
            }

            osg::Group* group = new osg::Group;
                
            // Show the actual mesh.
            group->addChild( node.get() );

            _node = group;
            // Clamp the marker to the intersection of the triangles created by osgEarth.  This should line up with the mesh that is actually rendered.
            double z = 0.0;
            s_mapNode->getTerrain()->getHeight( node.get(), s_mapNode->getMapSRS(), mapPoint.x(), mapPoint.y(), &z);

            GeoTransform* xform = new GeoTransform();
            xform->setPosition( osgEarth::GeoPoint(s_mapNode->getMapSRS(),mapPoint.x(),  mapPoint.y(), z, ALTMODE_ABSOLUTE) );
            xform->addChild( marker.get() );
            group->addChild( xform );

            s_root->addChild( _node.get() );
        }
        else
        {
            OE_NOTICE << "Failed to create tile for " << key.str() << std::endl;
        }
    }

    void update(const std::string& tileKeyString)
    {
        TileKey key = makeTileKey(tileKeyString, s_mapNode->getMap()->getProfile());
        osg::ref_ptr<osg::Node> node;
        if (key.valid() && (node = makeCustomTile(key)))
        {
            if (_node.valid())
            {
                s_root->removeChild(_node.get());
            }
            s_root->addChild(node.get());
            _node = node;
            return;
        }
        OE_NOTICE << "Failed to create tile for " << key.str() << std::endl;
    }
    
    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        if (ea.getEventType() == osgGA::GUIEventAdapter::FRAME && !_keyString.empty())
        {
            update(_keyString);
            _keyString.clear();
        }
        else if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH
                 && ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON
                 && (ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_SHIFT))
        {
            osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());
            update( ea.getX(), ea.getY(), view );
        }
        return false;
    }

    osg::ref_ptr< osg::Node > _node;
    CreateTileManifest _manifest; // Empty should be fine
    std::string _keyString;
    unsigned _tileLOD;
    unsigned _refLOD;
    TerrainEngineNode::CreateTileFlags _tileFlags;
};


int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osgEarth::initialize();

    osg::ref_ptr<CreateTileHandler> createTileHandler(new CreateTileHandler(arguments));
    osgViewer::Viewer viewer(arguments);

    s_root = new osg::Group();
    
    // install the programmable manipulator.
    viewer.setCameraManipulator( new osgEarth::Util::EarthManipulator() );

    // The MapNode will render the Map object in the scene graph.
    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    s_mapNode = MapNode::findMapNode(node);
    s_root->addChild( node );

    // disable the small-feature culling
    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

    // set a near/far ratio that is smaller than the default. This allows us to get
    // closer to the ground without near clipping. If you need more, use --logdepth
    viewer.getCamera()->setNearFarRatio(0.0001);
    viewer.getCamera()->setComputeNearFarMode( osg::CullSettings::COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES );


    // An event handler that will respond to mouse clicks:
    viewer.addEventHandler( createTileHandler.get() );

    viewer.setSceneData( s_root );

    return viewer.run();
}
