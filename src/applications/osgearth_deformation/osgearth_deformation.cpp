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
#include <osgEarth/VerticalDatum>
#include <osgEarth/Registry>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/Controls>
#include <osgEarthUtil/LatLongFormatter>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthSymbology/Geometry>
#include <osgEarthSymbology/GeometryRasterizer>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthFeatures/Feature>
#include <osgEarthSymbology/GeometryFactory>
#include <iomanip>


using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Util;
using namespace osgEarth::Symbology;


class Deformation
{
public:
    Deformation(Feature* feature, double offset):
      _feature(feature),
          _offset(offset)
      {
      }

      osg::ref_ptr< Feature > _feature;
      double _offset;
};

typedef std::vector< Deformation > DeformationList;



class DeformationTileSource : public TileSource
{
public:

    


    // Constructor that takes the user-provided options.
    DeformationTileSource() : TileSource(TileSourceOptions())
    {
    }

    // Called by the terrain engine when a layer using this driver is first added.
    Status initialize(const osgDB::Options* dbOptions)
    {
        if ( !getProfile() )
        {
            setProfile( Registry::instance()->getGlobalGeodeticProfile() );
        }
        return STATUS_OK;
    }

    // Tells the layer not to cache data from this tile source.
    CachePolicy getCachePolicyHint(const Profile* profile) const 
    {
        return CachePolicy::NO_CACHE;
    }

    osg::HeightField* createHeightField(const TileKey& key, ProgressCallback* progress)
    {
        if (key.getLevelOfDetail() < 3)
        {
            return 0;
        }
        OpenThreads::ScopedLock< OpenThreads::Mutex > lk(_mutex);
        DeformationList intersectingDeformations;
        {
            for (DeformationList::iterator itr = _deformations.begin(); itr != _deformations.end(); ++itr)
            {                
                GeoExtent ext(itr->_feature->getSRS(), itr->_feature->getGeometry()->getBounds());
                if (key.getExtent().intersects(ext, false))
                {
                    intersectingDeformations.push_back( *itr );
                }
            }
        }

        //OE_NOTICE << "Checking " << intersectingDeformations.size() << " of " << _deformations.size() << " deformations " << std::endl;

        if (intersectingDeformations.empty())
        {
            return 0;
        }

        

        //Get the extents of the tile
        double xmin, ymin, xmax, ymax;
        key.getExtent().getBounds(xmin, ymin, xmax, ymax);

        int tileSize = 257;

        //Only allocate the heightfield if we actually intersect any features.
        osg::ref_ptr<osg::HeightField> hf = new osg::HeightField;
        hf->allocate(tileSize, tileSize);
        for (unsigned int i = 0; i < hf->getHeightList().size(); ++i) hf->getHeightList()[i] = NO_DATA_VALUE;

        // Iterate over the output heightfield and sample the data that was read into it.
        double dx = (xmax - xmin) / (tileSize-1);
        double dy = (ymax - ymin) / (tileSize-1);



        


        for (int c = 0; c < tileSize; ++c)
        {
            double geoX = xmin + (dx * (double)c);
            for (int r = 0; r < tileSize; ++r)
            {
                double geoY = ymin + (dy * (double)r);

                float h = NO_DATA_VALUE;

                for (DeformationList::iterator itr = intersectingDeformations.begin(); itr != intersectingDeformations.end(); ++itr)
                {
                    osgEarth::Symbology::Polygon* boundary = dynamic_cast<osgEarth::Symbology::Polygon*>(itr->_feature->getGeometry());
                    //OE_NOTICE << "Checking " << itr->_feature->getGeoJSON() << std::endl;

                    if (!boundary)
                    {
                        OE_WARN << "NOT A POLYGON" << std::endl;
                    }
                    else
                    {
                        GeoPoint geo(SpatialReference::create("wgs84"), geoX, geoY, 0.0, ALTMODE_ABSOLUTE);
                        if ( boundary->contains2D(geo.x(), geo.y()) )
                        {
                            if (h == NO_DATA_VALUE)
                            {
                                h = itr->_offset;
                            }
                            else
                            {
                                h += itr->_offset;
                            }
                        }
                    }
                }
                hf->setHeight(c, r, h);
                
            }
        }
        return hf.release();
    }

  

    void addDeformation(const Deformation& deformation)
    {
        OpenThreads::ScopedLock< OpenThreads::Mutex > lk(_mutex);
        _deformations.push_back( deformation );
    }

    virtual int getPixelsPerTile() const
    {
        return 257;
    }

    OpenThreads::Mutex _mutex;
    std::vector< Deformation > _deformations;
};

static MapNode*       s_mapNode     = 0L;
static DeformationTileSource* s_deformations = 0L;

enum Tool
{
    TOOL_CIRCLE,
    TOOL_RECTANGLE
};

struct DeformationHandler : public osgGA::GUIEventHandler 
{
    DeformationHandler()
        : _mouseDown( false ),
          _terrain  ( s_mapNode->getTerrain() ),
          _query    ( s_mapNode->getMap() ),
          _deformationsAdded(0),
          _framesSinceLastUpdate(0),
          _tool(TOOL_RECTANGLE)
    {
        _map = s_mapNode->getMap();
        _query.setMaxTilesToCache(10);
        _query.setFallBackOnNoData( false );
        _path.push_back( s_mapNode->getTerrainEngine() );
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
            mapPoint.fromWorld( _terrain->getSRS(), world );

            OE_NOTICE << "Got hit " << mapPoint.x() << ", " << mapPoint.y() << std::endl;

            GeometryFactory factory(SpatialReference::create("wgs84"));
            Geometry* geom = 0;
            double radius = 500;
            if (_tool == TOOL_RECTANGLE)
            {
                geom = factory.createRectangle(mapPoint.vec3d(), radius,radius);
            }
            else if (_tool == TOOL_CIRCLE)
            {
                geom = factory.createCircle(mapPoint.vec3d(), radius);
            }
            
            Feature* feature = new Feature(geom, SpatialReference::create("wgs84"));
            OE_NOTICE << "Adding deformation " << feature->getGeoJSON() << std::endl;
            s_deformations->addDeformation(Deformation(feature, -100));
            _deformationsAdded++;
            osgEarth::Registry::instance()->clearBlacklist();
            s_mapNode->getTerrainEngine()->dirtyTerrain();            
        }
    }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        if (ea.getEventType() == osgGA::GUIEventAdapter::MOVE ||
            ea.getEventType() == osgGA::GUIEventAdapter::DRAG)
        {
            if (_mouseDown)
            {
                osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());
                update( ea.getX(), ea.getY(), view );
            }
        }
        if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH && ea.getButton() == osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON)
        {
            _mouseDown = true;
            osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());
            update( ea.getX(), ea.getY(), view );
        }
        if (ea.getEventType() == osgGA::GUIEventAdapter::RELEASE && ea.getButton() == osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON)
        {
            _mouseDown = false;
        }
        /*
        else if (ea.getEventType() == osgGA::GUIEventAdapter::FRAME)
        {

            _framesSinceLastUpdate++;
            if (_deformationsAdded > 0 && _framesSinceLastUpdate > 5)
            {
                _framesSinceLastUpdate = 0;
                _deformationsAdded = 0;
                s_mapNode->getTerrainEngine()->dirtyTerrain();
            }
        }
        */
        else if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
        {
            if (ea.getKey() == 'k')
            {
                if (_tool == TOOL_RECTANGLE)
                {
                    OE_NOTICE << "Switching to circle tool" << std::endl;
                    _tool = TOOL_CIRCLE;
                }
                else
                {
                    OE_NOTICE << "Switching to rectangle tool" << std::endl;
                    _tool = TOOL_RECTANGLE;
                }
            }
        }

        return false;
    }

    const Map*       _map;
    const Terrain*   _terrain;
    bool             _mouseDown;
    ElevationQuery   _query;
    osg::NodePath    _path;
    int _framesSinceLastUpdate;
    int _deformationsAdded;
    Tool _tool;
};


int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    osgViewer::Viewer viewer(arguments);

    s_mapNode = 0L;
    osg::Node* earthFile = MapNodeHelper().load(arguments, &viewer);
    if (earthFile)
        s_mapNode = MapNode::get(earthFile);

    if ( !s_mapNode )
    {
        OE_WARN << "Unable to load earth file." << std::endl;
        return -1;
    }

    s_deformations = new DeformationTileSource();
    ElevationLayerOptions opt;
    opt.offset() = true;
    opt.cachePolicy() = CachePolicy::NO_CACHE;
    opt.name() = "deformation";
    opt.cacheId() = "deformation";
    ElevationLayer* layer = new ElevationLayer(opt, s_deformations);
    s_mapNode->getMap()->addElevationLayer(layer);

    osg::Group* root = new osg::Group();
    viewer.setSceneData( root );
    
    // install the programmable manipulator.
    viewer.setCameraManipulator( new osgEarth::Util::EarthManipulator() );

    // The MapNode will render the Map object in the scene graph.
    root->addChild( earthFile );

    viewer.addEventHandler( new DeformationHandler() );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
