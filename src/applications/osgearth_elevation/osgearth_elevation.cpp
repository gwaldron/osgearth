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

#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgUtil/LineSegmentIntersector>
#include <osgEarth/MapNode>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/StringUtils>
#include <osgEarth/Terrain>
#include <osgEarth/VerticalDatum>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/Controls>
#include <osgEarthUtil/LatLongFormatter>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthAnnotation/ModelNode>
#include <iomanip>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;
using namespace osgEarth::Annotation;

static MapNode*       s_mapNode     = 0L;
static LabelControl*  s_posLabel    = 0L;
static LabelControl*  s_vdaLabel    = 0L;
static LabelControl*  s_mslLabel    = 0L;
static LabelControl*  s_haeLabel    = 0L;
static LabelControl*  s_egm96Label  = 0L;
static LabelControl*  s_mapLabel    = 0L;
static LabelControl*  s_resLabel    = 0L;
static ModelNode*     s_marker      = 0L;


// An event handler that will print out the elevation at the clicked point
struct QueryElevationHandler : public osgGA::GUIEventHandler 
{
    QueryElevationHandler()
        : _mouseDown( false ),
          _terrain  ( s_mapNode->getTerrain() )
    {
        _map = s_mapNode->getMap();
        _path.push_back( s_mapNode->getTerrainEngine() );
        _envelope = _map->getElevationPool()->createEnvelope(_map->getSRS(), 20u);
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

            // do an elevation query:
            double query_resolution  = 0.0;  // max.
            double actual_resolution = 0.0;
            float elevation          = 0.0f;

            std::pair<float, float> result = _envelope->getElevationAndResolution(
                mapPoint.x(), mapPoint.y());

            elevation = result.first;
            actual_resolution = result.second;

            if ( elevation != NO_DATA_VALUE )
            {
                // convert to geodetic to get the HAE:
                mapPoint.z() = elevation;
                GeoPoint mapPointGeodetic( s_mapNode->getMapSRS()->getGeodeticSRS(), mapPoint );

                static LatLongFormatter s_f;

                s_posLabel->setText( Stringify()
                    << std::fixed << std::setprecision(2) 
                    << s_f.format(mapPointGeodetic.y(), true)
                    << ", " 
                    << s_f.format(mapPointGeodetic.x(), false) );

                if (s_mapNode->getMapSRS()->isGeographic())
                {
                    double metersPerDegree = s_mapNode->getMapSRS()->getEllipsoid()->getRadiusEquator() / 360.0;
                    actual_resolution *= metersPerDegree * cos(osg::DegreesToRadians(mapPoint.y()));
                }

                s_mslLabel->setText( Stringify() << elevation << " m" );
                s_haeLabel->setText( Stringify() << mapPointGeodetic.z() << " m" );
                s_resLabel->setText( Stringify() << actual_resolution << " m" );

                double egm96z = mapPoint.z();

                VerticalDatum::transform(
                    mapPointGeodetic.getSRS()->getVerticalDatum(),
                    VerticalDatum::get("egm96"),
                    mapPointGeodetic.y(),
                    mapPointGeodetic.x(),
                    egm96z);
                
                s_egm96Label->setText(Stringify() << egm96z << " m");

                yes = true;
            }

            // now get a normal ISECT HAE point.
            GeoPoint isectPoint;
            isectPoint.fromWorld( _terrain->getSRS()->getGeodeticSRS(), world );
            s_mapLabel->setText( Stringify() << isectPoint.alt() << " m");

            // and move the marker.
            s_marker->setPosition(mapPoint);

            // normal test.
            osg::Quat q;
            q.makeRotate(osg::Vec3(0,0,1), hits.begin()->getLocalIntersectNormal());
            s_marker->setLocalRotation(q);
        }

        if (!yes)
        {
            s_posLabel->setText( "-" );
            s_mslLabel->setText( "-" );
            s_haeLabel->setText( "-" );
            s_resLabel->setText( "-" );
            s_egm96Label->setText("-");
        }
    }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        if (ea.getEventType() == ea.DOUBLECLICK &&
            ea.getButton() == ea.LEFT_MOUSE_BUTTON)
        {
            osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());
            update( ea.getX(), ea.getY(), view );
            return true;
        }

        return false;
    }

    const Map*       _map;
    const Terrain*   _terrain;
    bool             _mouseDown;
    osg::NodePath    _path;
    osg::ref_ptr<ElevationEnvelope> _envelope;
};


struct ClickToRemoveElevation : public ControlEventHandler
{
    void onClick(Control*)
    {
        Map* map = s_mapNode->getMap();
        ElevationLayerVector layers;
        map->getLayers(layers);
        map->beginUpdate();
        for (ElevationLayerVector::iterator i = layers.begin(); i != layers.end(); ++i) {
            map->removeLayer(i->get());
        }
        map->endUpdate();
    }
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

    osg::Group* root = new osg::Group();
    viewer.setSceneData( root );
    
    // install the programmable manipulator.
    viewer.setCameraManipulator( new osgEarth::Util::EarthManipulator() );

    // The MapNode will render the Map object in the scene graph.
    root->addChild( earthFile );

    // Make the readout:
    Grid* grid = new Grid();
    grid->setBackColor(osg::Vec4(0,0,0,0.5));
    int r=0;
    grid->setControl(0,r++,new LabelControl("Double-click to sample elevation", osg::Vec4(1,1,0,1)));
    grid->setControl(0,r++,new LabelControl("Coords (Lat, Long):"));
    grid->setControl(0,r++,new LabelControl("Map vertical datum:"));
    grid->setControl(0,r++,new LabelControl("Height above geoid:"));
    grid->setControl(0,r++,new LabelControl("Height above ellipsoid:"));
    grid->setControl(0,r++,new LabelControl("Scene graph intersection:"));
    grid->setControl(0,r++,new LabelControl("EGM96 elevation:"));
    grid->setControl(0,r++,new LabelControl("Query resolution:"));
    grid->setControl(0, r++, new ButtonControl("Click to remove all elevation data", new ClickToRemoveElevation()));

    r = 1;
    s_posLabel = grid->setControl(1,r++,new LabelControl(""));
    s_vdaLabel = grid->setControl(1,r++,new LabelControl(""));
    s_mslLabel = grid->setControl(1,r++,new LabelControl(""));
    s_haeLabel = grid->setControl(1,r++,new LabelControl(""));
    s_mapLabel = grid->setControl(1,r++,new LabelControl(""));
    s_egm96Label = grid->setControl(1,r++,new LabelControl(""));
    s_resLabel = grid->setControl(1,r++,new LabelControl(""));

    
    Style markerStyle;
    markerStyle.getOrCreate<ModelSymbol>()->url()->setLiteral("../data/axes.osgt.64.scale");
    markerStyle.getOrCreate<ModelSymbol>()->autoScale() = true;
    s_marker = new ModelNode(s_mapNode, markerStyle);
    //s_marker->setMapNode( s_mapNode );
    //s_marker->setIconImage(osgDB::readImageFile("../data/placemark32.png"));
    s_marker->setDynamic(true);
    s_mapNode->addChild( s_marker );

    const SpatialReference* mapSRS = s_mapNode->getMapSRS();
    s_vdaLabel->setText( mapSRS->getVerticalDatum() ? 
        mapSRS->getVerticalDatum()->getName() : 
        Stringify() << "geodetic (" << mapSRS->getEllipsoid()->getName() << ")" );

    ControlCanvas* canvas = ControlCanvas::get(&viewer);
    canvas->addControl( grid );

    // An event handler that will respond to mouse clicks:
    viewer.addEventHandler( new QueryElevationHandler() );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
