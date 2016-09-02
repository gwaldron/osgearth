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
#include <osgEarthAnnotation/FeatureNode>
#include <osgEarthSymbology/GeometryFactory>
#include <iomanip>


using namespace osgEarth;
using namespace osgEarth::Annotation;
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

typedef std::map< TileKey, osg::ref_ptr< osg::HeightField > > HeightFieldMap;


/**
* Applies the deformation to the heightfield with the given key.
*/
static void deformHeightField(const Deformation& deformation, const TileKey& key, osg::HeightField* heightField)
{   
    osgEarth::Symbology::Polygon* boundary = dynamic_cast<osgEarth::Symbology::Polygon*>(deformation._feature->getGeometry());
    if (!boundary)
    {
        OE_WARN << "NOT A POLYGON" << std::endl;
        return;
    }

    //Get the extents of the tile
    double xmin, ymin, xmax, ymax;
    key.getExtent().getBounds(xmin, ymin, xmax, ymax);

    int tileSize = heightField->getNumColumns();

    // Iterate over the output heightfield and sample the data that was read into it.
    double dx = (xmax - xmin) / (tileSize-1);
    double dy = (ymax - ymin) / (tileSize-1);     


    const SpatialReference* srs = SpatialReference::create("wgs84");

    for (int c = 0; c < tileSize; ++c)
    {
        double geoX = xmin + (dx * (double)c);
        for (int r = 0; r < tileSize; ++r)
        {
            double geoY = ymin + (dy * (double)r);

            GeoPoint geo(srs, geoX, geoY, 0.0, ALTMODE_ABSOLUTE);
            if ( boundary->contains2D(geo.x(), geo.y()) )
            {
                heightField->setHeight(c, r, deformation._offset);
            }                
        }
    }
}

static void applyBlast(GeoPoint& center, double radius, double offset, const TileKey& key, osg::HeightField* heightField)
{
    //Get the extents of the tile
    double xmin, ymin, xmax, ymax;
    key.getExtent().getBounds(xmin, ymin, xmax, ymax);

    int tileSize = heightField->getNumColumns();

    // Iterate over the output heightfield and sample the data that was read into it.
    double dx = (xmax - xmin) / (tileSize-1);
    double dy = (ymax - ymin) / (tileSize-1);     

    const SpatialReference* srs = SpatialReference::create("wgs84");

    for (int c = 0; c < tileSize; ++c)
    {
        double geoX = xmin + (dx * (double)c);
        for (int r = 0; r < tileSize; ++r)
        {
            double geoY = ymin + (dy * (double)r);

            GeoPoint geo(srs, geoX, geoY, center.z(), ALTMODE_ABSOLUTE);

            double distance = geo.distanceTo( center );
            double ratio = distance / radius;
            if (ratio <= 1.0)
            {
                double weight = 1.0 - osg::clampBetween(ratio, 0.0, 1.0);
                if (weight > 0)
                {
                    float h = center.z() + offset * weight;
                    heightField->setHeight(c, r, h);
                }
            }
            
            /*


            if ( boundary->contains2D(geo.x(), geo.y()) )
            {
                float h = heightField->getHeight( c, r );
                if (h == NO_DATA_VALUE)
                {
                    //h = deformation._offset;
                }
                else
                {
                    //h += deformation._offset;                    
                    h = deformation._offset;
                }
                heightField->setHeight(c, r, h);
            } 
            */
        }
    }
}

/*
void getHeightFieldsForDeformation(const Deformation& deformation, MapFrame& map, unsigned int level, HeightFieldMap& results)
{
    // Get the extent of the deformation feature.
    GeoExtent extent(deformation._feature->getSRS(), deformation._feature->getGeometry()->getBounds());

    TileKey ll = map.getProfile()->createTileKey(extent.xMin(), extent.yMin(), level);
    TileKey ur = map.getProfile()->createTileKey(extent.xMax(), extent.yMax(), level);        

    for (unsigned int c = ll.getTileX(); c <= ur.getTileX(); c++)
    {
        for (unsigned int r = ur.getTileY(); r <= ll.getTileY(); r++)
        {
            TileKey key(level, c, r, map.getProfile());
            //Allocate a new heightfield
            osg::ref_ptr< osg::HeightField > hf = new osg::HeightField;
            hf->allocate(257, 257);
            for (unsigned int i = 0; i < hf->getHeightList().size(); ++i) hf->getHeightList()[i] = NO_DATA_VALUE;
            // We don't actually need to populate it unless we are doing some logic on it, right?
            //map.populateHeightField(hf, key, false, 0);
            results[ key ] = hf.get();            
        }
    }
}
*/




class DeformationTileSource : public TileSource
{
public: 
    // Constructor that takes the user-provided options.
    DeformationTileSource(const TileSourceOptions& options) : TileSource(options)
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
        OpenThreads::ScopedLock< OpenThreads::Mutex > lk(_mutex);

        // See if we have a neightfield in the cache
        HeightFieldMap::iterator itr = _heightfields.find( key );
        if (itr != _heightfields.end())
        {
            osg::HeightField* hf = itr->second.get();
            return hf;
        }
        return 0;        
    }

    /*
    void addDeformation(const Deformation& deformation)
    {
        OpenThreads::ScopedLock< OpenThreads::Mutex > lk(_mutex);
        _deformations.push_back( deformation );

        // Get the heightfields that might be effect by the deformation
        HeightFieldMap heightfields;
        for (unsigned int i = 5; i < 10; i++)
        {
            getOrCreateHeightFieldsForDeformation( deformation, i, heightfields);
        }

        OE_NOTICE << "Applying deformation to " << heightfields.size() << " heightfields" << std::endl;

        // Actually apply the deformation
        for (HeightFieldMap::iterator itr = heightfields.begin(); itr != heightfields.end(); ++itr)
        {
            deformHeightField( deformation, itr->first, itr->second.get());
        }
    }
    */

    void addHeightField(const TileKey& key, osg::HeightField* heightfield)
    {
        OpenThreads::ScopedLock< OpenThreads::Mutex > lk(_mutex);
        _heightfields[ key ] = heightfield;
    }

    virtual int getPixelsPerTile() const
    {
        return 257;
    }

    /**
    * Gets or creates the heightfields that would be effected by the given deformation
    */
    void getOrCreateHeightFieldsForDeformation(const Deformation& deformation, unsigned int level, HeightFieldMap& results)
    {
        OpenThreads::ScopedLock< OpenThreads::Mutex > lk(_mutex);

        // Get the extent of the deformation feature.
        GeoExtent extent(deformation._feature->getSRS(), deformation._feature->getGeometry()->getBounds());

        TileKey ll = getProfile()->createTileKey(extent.xMin(), extent.yMin(), level);
        TileKey ur = getProfile()->createTileKey(extent.xMax(), extent.yMax(), level);        

        for (unsigned int c = ll.getTileX(); c <= ur.getTileX(); c++)
        {
            for (unsigned int r = ur.getTileY(); r <= ll.getTileY(); r++)
            {
                TileKey key(level, c, r, getProfile());

                osg::ref_ptr< osg::HeightField > hf;

                HeightFieldMap::iterator itr = _heightfields.find( key );
                if (itr == _heightfields.end())
                {
                    //Allocate a new heightfield
                    hf = new osg::HeightField;
                    hf->allocate(getPixelsPerTile(), getPixelsPerTile());
                    for (unsigned int i = 0; i < hf->getHeightList().size(); ++i) hf->getHeightList()[i] = NO_DATA_VALUE;
                    _heightfields[ key ] = hf.get();
                    results[ key ] = hf.get();
                }
                else
                {                
                    results[ key ] = itr->second.get();
                }
            }
        }
    }






    OpenThreads::Mutex _mutex;
    std::vector< Deformation > _deformations;

    HeightFieldMap _heightfields;

};

static MapNode*       s_mapNode     = 0L;
static DeformationTileSource* s_deformations = 0L;

enum Tool
{
    TOOL_CIRCLE,
    TOOL_RECTANGLE,
    TOOL_BLAST
};

struct DeformationHandler : public osgGA::GUIEventHandler 
{
    DeformationHandler(osg::Group* root)
        : _mouseDown( false ),
        _terrain  ( s_mapNode->getTerrain() ),
        _tool(TOOL_CIRCLE),
        _root(root),
        _offset(-100.0f),
        _radius(100.0),
        _query( s_mapNode->getMap() )
    {
        _map = s_mapNode->getMap();
        _query.setMaxTilesToCache(10);
        _query.setFallBackOnNoData( false );
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
            mapPoint.z() = 0;

            // do an elevation query:
            double query_resolution = 0; // max.
            double out_hamsl        = 0.0;
            double out_resolution   = 0.0;

            bool ok = _query.getElevation( 
                mapPoint,
                out_hamsl,
                query_resolution, 
                &out_resolution );
            mapPoint.z() = out_hamsl;
            _mapPoint = mapPoint;

            

            GeometryFactory factory(SpatialReference::create("wgs84"));
            Geometry* geom = 0;
            if (_tool == TOOL_RECTANGLE)
            {
                geom = factory.createRectangle(mapPoint.vec3d(), _radius,_radius);
            }
            else if (_tool == TOOL_CIRCLE || _tool == TOOL_BLAST)
            {
                geom = factory.createCircle(mapPoint.vec3d(), _radius);
            }

            Feature* feature = new Feature(geom, SpatialReference::create("wgs84"));

            if (_featureNode.valid())
            {
                _root->removeChild( _featureNode );
                _featureNode = 0;
            }

            Style style;
            style.getOrCreateSymbol<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
            style.getOrCreateSymbol<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_DRAPE;
            if (_tool != TOOL_BLAST)
            {
                style.getOrCreateSymbol<PolygonSymbol>()->fill()->color() = Color(Color::Cyan, 0.5);
            }
            else
            {
                style.getOrCreateSymbol<PolygonSymbol>()->fill()->color() = Color(Color::Red, 0.5);
            }

            _featureNode = new FeatureNode( s_mapNode,
                feature,
                style);
            _root->addChild( _featureNode );
        }
    }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        if (ea.getEventType() == osgGA::GUIEventAdapter::MOVE ||
            ea.getEventType() == osgGA::GUIEventAdapter::DRAG)
        {
            osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());
            update( ea.getX(), ea.getY(), view );
            if (_mouseDown)
            {                
                applyDeformation();
            }
        }
        if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH && ea.getButton() == osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON)
        {
            _mouseDown = true;
            osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());
            update( ea.getX(), ea.getY(), view );
            applyDeformation();
        }
        if (ea.getEventType() == osgGA::GUIEventAdapter::RELEASE && ea.getButton() == osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON)
        {
            _mouseDown = false;
        }
        else if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
        {
            if (ea.getKey() == 'k')
            {
                if (_tool == TOOL_CIRCLE)
                {
                    OE_NOTICE << "Switching to rectangle tool" << std::endl;
                    _tool = TOOL_RECTANGLE;
                }
                else if (_tool == TOOL_RECTANGLE)
                {
                    OE_NOTICE << "Switching to blast tool" << std::endl;
                    _tool = TOOL_BLAST;
                }
                else if (_tool == TOOL_BLAST)
                {
                    OE_NOTICE << "Switching to circle tool" << std::endl;
                    _tool = TOOL_CIRCLE;
                }
                return true;
            }
            else if (ea.getKey() == 'o')
            {
                _offset -= 10;
                OE_NOTICE << "Offset " << _offset << std::endl;
                return true;
            }
            else if (ea.getKey() == 'O')
            {
                _offset += 10;
                OE_NOTICE << "Offset " << _offset << std::endl;
                return true;
            }
            else if (ea.getKey() == 'r')
            {
                _radius = osg::clampAbove(_radius - 10.0, 1.0);
                OE_NOTICE << "Radius = " << _radius << std::endl;
            }
            else if (ea.getKey() == 'R')
            {
                _radius += 10;
                OE_NOTICE << "Radius = " << _radius << std::endl;
            }
        }

        return false;
    }

    void applyDeformation()
    {
        if (_featureNode)
        {
            Feature* feature = _featureNode->getFeature();
            OE_NOTICE << "Adding deformation " << feature->getGeoJSON() << std::endl;

            Deformation deformation(feature, _offset);
            // Instead of applying the deformation, we are going to add a heightfield ourselves.
            //s_deformations->addDeformation(Deformation(feature, _offset));


            MapFrame frame(s_mapNode->getMap());
            // Get the heightfields that might be effect by the deformation
            HeightFieldMap heightfields;
            for (unsigned int i = 0; i < 14; i++)
            {
                //getHeightFieldsForDeformation(deformation, frame, i, heightfields);
                s_deformations->getOrCreateHeightFieldsForDeformation(deformation, i, heightfields);
            }

            OE_NOTICE << "Deforming " << heightfields.size() << std::endl;

            // Actually apply the deformation
            for (HeightFieldMap::iterator itr = heightfields.begin(); itr != heightfields.end(); ++itr)
            {
                // Deform heightfield based on a polygon
                if (_tool == TOOL_CIRCLE || _tool == TOOL_RECTANGLE)
                {
                    // Set the offset to be absolute since we aren't using offset heightfield anymore.
                    deformation._offset = _mapPoint.z() + _offset;
                    deformHeightField( deformation, itr->first, itr->second.get());
                }
                else if (_tool == TOOL_BLAST)
                {
                    // Apply a simple blast radius
                    applyBlast(_mapPoint, _radius, -_radius, itr->first, itr->second);
                }
                s_deformations->addHeightField( itr->first, itr->second.get());
            }

            osgEarth::Registry::instance()->clearBlacklist();
            s_deformations->getBlacklist()->clear();
            s_mapNode->getTerrainEngine()->dirtyTerrain();            
        }
    }

    const Map*       _map;
    const Terrain*   _terrain;
    bool             _mouseDown;
    Tool _tool;
    osg::Group* _root;
    float _offset;
    double _radius;
    GeoPoint _mapPoint;
    osg::ref_ptr < FeatureNode > _featureNode;
    ElevationQuery   _query;
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

    TileSourceOptions tileSourceOptions;
    tileSourceOptions.L2CacheSize() = 0;
    s_deformations = new DeformationTileSource(tileSourceOptions);
    s_deformations->open();

    ElevationLayerOptions elevationOpt;
    //elevationOpt.offset() = true;
    elevationOpt.name() = "deformation";
    // This is the only way to get the l2 cache size to pass down even though we're not actually creating a tilesource from the options.
    elevationOpt.driver() = tileSourceOptions;

    ElevationLayer* layer = new ElevationLayer(elevationOpt, s_deformations);
    layer->open();
    s_mapNode->getMap()->addElevationLayer(layer);

    osg::Group* root = new osg::Group();
    viewer.setSceneData( root );

    // install the programmable manipulator.
    viewer.setCameraManipulator( new osgEarth::Util::EarthManipulator() );

    // The MapNode will render the Map object in the scene graph.
    root->addChild( earthFile );

    viewer.addEventHandler( new DeformationHandler(root) );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
