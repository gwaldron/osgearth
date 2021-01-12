/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2020 Pelican Mapping
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
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/MapNode>
#include <osgEarth/Metrics>
#include <osgEarth/GLUtils>
#include <osgEarth/LineDrawable>
#include <osgEarth/DrapeableNode>
#include <iostream>
#include <h3/h3api.h>

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

class PagedNode : public osg::Group
{
public:
    typedef std::function<osg::Node*(Cancelable*)> LoadFunction;
    Future<osg::Node> _future;

    PagedNode(const LoadFunction& loader)
    {
        Job<osg::Node> job(loader);
        _future = job.schedule();
    }

    PagedNode(const std::string& arena, const LoadFunction& loader)
    {
        Job<osg::Node> job(loader);
        _future = job.schedule(arena);
    }

    void traverse(osg::NodeVisitor& nv) override
    {
        if (_future.isAvailable())
        {
            _future.get()->accept(nv);
        }
    }
};

template<typename INDEX>
class PagedGraph : public osg::Group
{
public:
    //typedef H3Index INDEX; // TODO: replace with template parameter
    //typedef INDEX Cell;
    typedef std::list<INDEX> Tracker;
    std::unordered_map<INDEX, osg::ref_ptr<osg::Node>> _children;
    std::unordered_map<INDEX, typename Tracker::iterator> _trackerLUT;
    Tracker _tracker;
    typename Tracker::iterator _sentryptr;
    Mutex _mutex;
    const INDEX SENTRY_VALUE = 0;

    PagedGraph()
        : osg::Group()
    {
        _tracker.push_front(SENTRY_VALUE);
        _sentryptr = _tracker.begin();

        setCullingActive(false);
        setNumChildrenRequiringUpdateTraversal(1u);
    }

    //! Override this to create the new.
    //! This is a temporary implementation; the method should be pure virtual.
    virtual osg::Node* createNode(INDEX index) const = 0;
    
    // TODO: pure virtual. Implements the spatial search.
    virtual int search(std::vector<INDEX>& results, osg::NodeVisitor& nv) const = 0;

    //! Mark indices as active OR add new indices
    void tick(const std::vector<INDEX>& indices, osg::NodeVisitor& nv)
    {
        ScopedMutexLock lock(_mutex);

        for(const auto& index : indices)
        {
            auto iter = _trackerLUT.find(index);

            if (iter != _trackerLUT.end()) // existing
            {
                // move the tracker to the front of the list (ahead of the sentry).
                // Once a cull traversal is complete, all visited tiles will be
                // in front of the sentry, leaving all non-visited tiles behind it.
                _tracker.erase(iter->second);
            }

            else // new
            {
                // add it to the tracker:
                _children[index] = createNode(index);
            }

            _tracker.push_front(index);
            _trackerLUT[index] = _tracker.begin();

            _children[index]->accept(nv);
        }
    }

    //! Get rid of indices that are no longer active
    void purge()
    {
        std::list<osg::ref_ptr<osg::Node>> purged;
        {
            ScopedMutexLock lock(_mutex);

            // Collect all incides following the sentry and purge them.
            Tracker::iterator i = _sentryptr;
            Tracker::iterator tmp_iter;
            for (++i; i != _tracker.end(); ++i)
            {
                INDEX& index = *i;
                tmp_iter = i;
                --i;
                purged.push_back(_children[index]);
                _children.erase(index);
                _trackerLUT.erase(index);
                _tracker.erase(tmp_iter);
            }

            // reset the sentry for the next frame.
            _tracker.erase(_sentryptr);
            _tracker.push_front(SENTRY_VALUE);
            _sentryptr = _tracker.begin();
        }

        // release resources..
        for (auto& node : purged)
        {
            node->releaseGLObjects(nullptr);
        }

        // put them in a secondary cache for a while...?

        // invoke callbacks...?
    }

    void traverse(osg::NodeVisitor& nv) override
    {
        if (nv.getVisitorType() == nv.CULL_VISITOR)
        {
            // do a spatial index search and tick() all the resulting cells
            // to keep them alive. Then traverse each child.
            std::vector<INDEX> cullset;
            if (search(cullset, nv) > 0)
            {
                tick(cullset, nv);
            }
        }

        else if (nv.getVisitorType() == nv.UPDATE_VISITOR)
        {
            // purge dead cells (cells that fall after the sentry)
            purge();
        }
    }
};


class H3PagedGraph : public PagedGraph<H3Index>
{
public:

    int _cell_res;
    double _search_radius;
    
    H3PagedGraph() : PagedGraph<H3Index>()
    {
        _cell_res = 2;
        _search_radius = 500000.0;
    }

    osg::Node* createNode(H3Index index) const override
    {
        return new PagedNode("H3", [index](Cancelable* state)
            {
                LineDrawable* lines = new LineDrawable(GL_LINES);
                lines->setColor(Color::Red);
                lines->setLineWidth(4.0f);

                GeoBoundary boundary;
                h3ToGeoBoundary(index, &boundary);
                GeoPoint p_wgs84(SpatialReference::get("WGS84"));
                osg::Vec3d world;
                for (int j = 0; j < boundary.numVerts; ++j)
                {
                    for (int n = 0; n < 2; ++n)
                    {
                        int r = (j + n) % boundary.numVerts;
                        p_wgs84.x() = radsToDegs(boundary.verts[r].lon);
                        p_wgs84.y() = radsToDegs(boundary.verts[r].lat);
                        p_wgs84.z() = 10000;
                        p_wgs84.toWorld(world);
                        lines->pushVertex(world);
                    }
                }
                lines->finish();

                // Artifical sleep to show the paging
                //std::this_thread::sleep_for(std::chrono::milliseconds(40));

                return lines;
            });
    }

    int search(std::vector<H3Index>& results, osg::NodeVisitor& nv) const override
    {
        // find the lat/long under the camera
        osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);
        osg::Vec3d eye(0, 0, 0);
        eye = eye * cv->getCurrentCamera()->getInverseViewMatrix();

        GeoPoint p_wgs84;
        p_wgs84.fromWorld(SpatialReference::get("WGS84"), eye);

        GeoCoord h3_coord;
        h3_coord.lat = degsToRads(p_wgs84.y());
        h3_coord.lon = degsToRads(p_wgs84.x());

        // Resolve the H3 cell containing the lat/long:
        H3Index h3_index = geoToH3(&h3_coord, _cell_res);

        // Compute k from search radius at the current resolution:
        int k = (int)ceil(_search_radius / edgeLengthM(_cell_res));

        // Allocate space for the radial search:
        int maxNumCells = maxKringSize(k);
        H3Index* cells = new H3Index[maxNumCells];

        // Find all cells withing "k" cells of the original cell:
        kRing(h3_index, k, cells);

        for (int i = 0; i < maxNumCells; ++i)
        {
            if (cells[i] != 0)
            {
                results.push_back(cells[i]);
            }
        }

        delete[] cells;

        return results.size();
    }
};


#if 1
int
main(int argc, char** argv)
{
    osgEarth::initialize();

    osg::ArgumentParser arguments(&argc, argv);

    if (arguments.read("--help"))
        return usage(argv[0]);

    std::cout
        << "Press A/S to change cell resolution" << std::endl
        << "Press Z/X to change search radius" << std::endl;

    osgViewer::Viewer viewer(arguments);

    // install an event handler
    EventRouter* router = new EventRouter();
    viewer.addEventHandler(router);

    // configure our job arena to use 4 threads.
    JobArena::setSize("H3", 4u);

    // and a camera manip
    viewer.setCameraManipulator(new EarthManipulator(arguments));

    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if (node)
    {
        viewer.setSceneData(node);

        H3PagedGraph* graph = new H3PagedGraph();

        MapNode::get(node)->addChild(graph);

        // adjust h3 cell resolution
        router->onKeyPress(router->KEY_A, [graph]() { ++graph->_cell_res; });
        router->onKeyPress(router->KEY_S, [graph]() { graph->_cell_res = std::max(graph->_cell_res - 1, 0); });

        // adjust search radius
        router->onKeyPress(router->KEY_Z, [graph]() { graph->_search_radius *= 2.0; });
        router->onKeyPress(router->KEY_X, [graph]() { graph->_search_radius = std::max(graph->_search_radius/2.0, 100.0); });

        return Metrics::run(viewer);
    }
    else
    {
        return usage(argv[0]);
    }
}

#else
int
main(int argc, char** argv)
{
    osgEarth::initialize();

    osg::ArgumentParser arguments(&argc,argv);

    if ( arguments.read("--help") )
        return usage(argv[0]);

    std::cout
        << "Press A/S to change cell resolution" << std::endl
        << "Press Z/X to change search radius" << std::endl;

    osgViewer::Viewer viewer(arguments);

    // install an event handler
    EventRouter* router = new EventRouter();
    viewer.addEventHandler(router);

    // and a camera manip
    viewer.setCameraManipulator( new EarthManipulator(arguments) );
    
    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if ( node )
    {
        viewer.setSceneData( node );

        // H3 index resolution. 0 is the lower (largest cell size)
        int cell_res = 2;

        // Search radius around the mouse in meters
        double search_radius = 500000.0;

        // Set up a graphic to render the cells
        LineDrawable* lines = new LineDrawable(GL_LINES);
        lines->setColor(Color::Red);
        lines->setLineWidth(6.0f);
        DrapeableNode* drape = new DrapeableNode();
        drape->addChild(lines);
        MapNode::get(node)->addChild(drape);

        // When the mouse moves, plot the cells within the search radius
        router->onMove([&](osg::View* view, float x, float y)
            {
                // Find the lat/long of the point under the mouse:
                MapNode* mapnode = MapNode::get(node);
                osg::Vec3d world;
                mapnode->getTerrain()->getWorldCoordsUnderMouse(view, x, y, world);
                GeoPoint p_map;
                p_map.fromWorld(mapnode->getMapSRS(), world);
                GeoPoint p_wgs84 = p_map.transform(mapnode->getMapSRS()->getGeographicSRS());
                
                // Popluate the H3 structure:
                GeoCoord h3_coord;
                h3_coord.lat = degsToRads(p_wgs84.y());
                h3_coord.lon = degsToRads(p_wgs84.x());
                
                // Resolve the H3 cell containing the lat/long:
                H3Index h3_index = geoToH3(&h3_coord, cell_res);

                // Compute k from search radius at the current resolution:
                int k = (int)ceil(search_radius / edgeLengthM(cell_res));

                // Allocate space for the radial search:
                int maxNumCells = maxKringSize(k);
                H3Index* cells = new H3Index[maxNumCells];

                // Find all cells withing "k" cells of the original cell:
                kRing(h3_index, k, cells);

                // Rebuild the visual:
                lines->clear();
                GeoBoundary boundary;
                for (int i = 0; i < maxNumCells; ++i)
                {
                    H3Index& cell = cells[i];
                    if (cell > 0) // cell==0 means an empty slot, so ignore it
                    {
                        h3ToGeoBoundary(cell, &boundary);
                        for (int j = 0; j < boundary.numVerts; ++j)
                        {
                            for (int n = 0; n < 2; ++n)
                            {
                                int r = (j + n) % boundary.numVerts;
                                p_wgs84.x() = radsToDegs(boundary.verts[r].lon);
                                p_wgs84.y() = radsToDegs(boundary.verts[r].lat);
                                p_map = p_wgs84.transform(mapnode->getMapSRS());
                                p_map.toWorld(world);
                                lines->pushVertex(world);
                            }
                        }
                    }
                }
                lines->finish();

                // Clean up
                delete[] cells;
            });


        // adjust h3 cell resolution
        router->onKeyPress(router->KEY_A, [&cell_res]() { ++cell_res; });
        router->onKeyPress(router->KEY_S, [&cell_res]() { cell_res = std::max(cell_res - 1, 0); });

        // adjust search radius
        router->onKeyPress(router->KEY_Z, [&search_radius]() { search_radius *= 2.0; });
        router->onKeyPress(router->KEY_X, [&search_radius]() { search_radius /= 2.0; });

        return Metrics::run(viewer);
    }
    else
    {
        return usage(argv[0]);
    }
}
#endif
