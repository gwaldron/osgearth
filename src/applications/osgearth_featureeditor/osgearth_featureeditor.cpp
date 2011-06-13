/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2010 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include <osg/Notify>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/GUIEventHandler>
#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>

#include <osgEarthSymbology/Style>
#include <osgEarthSymbology/GeometrySymbol>
#include <osgEarthFeatures/FeatureModelGraph>
#include <osgEarthFeatures/FeatureListSource>

#include <osgEarthDrivers/gdal/GDALOptions>
#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>
#include <osgEarthDrivers/agglite/AGGLiteOptions>
#include <osgEarthDrivers/model_feature_geom/FeatureGeomModelOptions>
#include <osgEarthDrivers/model_feature_stencil/FeatureStencilModelOptions>

#include <osgEarthUtil/Draggers>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers;
using namespace osgEarth::Symbology;
using namespace osgEarth::Util;

static int s_fid = 0;

class MoveFeatureDraggerCallback : public osgManipulator::DraggerCallback
{
public:
    MoveFeatureDraggerCallback(Feature* feature, FeatureSource* source, const Map* map, int point):
      _feature(feature),
      _source(source),
      _map(map),
      _point(point)
      {}

      osg::Vec2d getLocation(const osg::Matrixd& matrix)
      {
          osg::Vec3d trans = matrix.getTrans();
          double lat, lon, height;
          _map->getProfile()->getSRS()->getEllipsoid()->convertXYZToLatLongHeight(trans.x(), trans.y(), trans.z(), lat, lon, height);
          return osg::Vec2d(osg::RadiansToDegrees(lon), osg::RadiansToDegrees(lat));
      }


      virtual bool receive(const osgManipulator::MotionCommand& command)
      {
          switch (command.getStage())
          {
          case osgManipulator::MotionCommand::START:
              {
                  // Save the current matrix                  
                  osg::Vec3d startLocation = (*_feature->getGeometry())[_point];
                  double x, y, z;
                  _map->getProfile()->getSRS()->getEllipsoid()->convertLatLongHeightToXYZ(osg::DegreesToRadians(startLocation.y()), osg::DegreesToRadians(startLocation.x()), 0, x, y, z);
                  _startMotionMatrix = osg::Matrixd::translate(x, y, z);

                  // Get the LocalToWorld and WorldToLocal matrix for this node.
                  osg::NodePath nodePathToRoot;
                  _localToWorld = osg::Matrixd::identity();
                  _worldToLocal = osg::Matrixd::identity();

                  return true;
              }
          case osgManipulator::MotionCommand::MOVE:
              {
                  // Transform the command's motion matrix into local motion matrix.
                  osg::Matrix localMotionMatrix = _localToWorld * command.getWorldToLocal()
                      * command.getMotionMatrix()
                      * command.getLocalToWorld() * _worldToLocal;

                  osg::Matrixd newMatrix = localMotionMatrix * _startMotionMatrix;
                  osg::Vec2d location = getLocation( newMatrix );
                  (*_feature->getGeometry())[_point] = osg::Vec3d(location.x(), location.y(), 0);
                  _source->dirty();

                  return true;
              }
          case osgManipulator::MotionCommand::FINISH:
              {
                  return true;
              }
          case osgManipulator::MotionCommand::NONE:
          default:
              return false;
          }
      }


      osg::ref_ptr<const Map>            _map;      
      osg::ref_ptr< Feature > _feature;
      osg::ref_ptr< FeatureSource > _source;

      osg::Matrix _startMotionMatrix;
      int _point;

      osg::Matrix _localToWorld;
      osg::Matrix _worldToLocal;
};


void addFeatureEditor(Feature* feature, FeatureSource* source, MapNode* mapNode, osg::Group* root)
{    
    osg::Group* draggers = new osg::Group;
    //Create a dragger for each point
    for (int i = 0; i < feature->getGeometry()->size(); i++)
    {
        osg::Matrixd matrix;
        double lat = (*feature->getGeometry())[i].y();
        double lon = (*feature->getGeometry())[i].x();
        mapNode->getMap()->getProfile()->getSRS()->getEllipsoid()->computeLocalToWorldTransformFromLatLongHeight(osg::DegreesToRadians(lat), osg::DegreesToRadians(lon), 0, matrix);    

        IntersectingDragger* dragger = new IntersectingDragger;
        dragger->setNode( mapNode->getTerrainEngine() );
        dragger->setupDefaultGeometry();
        dragger->setMatrix(matrix);
        dragger->setHandleEvents( true );        
        dragger->addDraggerCallback(new MoveFeatureDraggerCallback(feature, source, mapNode->getMap(), i) );

        draggers->addChild(dragger);        
    }
    root->addChild( draggers );
}



struct AddPointHandler : public osgGA::GUIEventHandler 
{
    AddPointHandler(FeatureListSource* source, const osgEarth::SpatialReference* mapSRS):
_source( source ),
_mapSRS( mapSRS )
{
}

    void addPoint( float x, float y, osgViewer::View* view )
    {
        osgUtil::LineSegmentIntersector::Intersections results;
        if ( view->computeIntersections( x, y, results, 0x01 ) )
        {
            // find the first hit under the mouse:
            osgUtil::LineSegmentIntersector::Intersection first = *(results.begin());
            osg::Vec3d point = first.getWorldIntersectPoint();
            
            // transform it to map coordinates:
            double lat_rad, lon_rad, dummy;
            _mapSRS->getEllipsoid()->convertXYZToLatLongHeight( point.x(), point.y(), point.z(), lat_rad, lon_rad, dummy );

            double lat_deg = osg::RadiansToDegrees( lat_rad );
            double lon_deg = osg::RadiansToDegrees( lon_rad );

            //Feature* feature = _source->getFeatures().front();
            Feature* feature = _source->getFeature( 0 );
            if (feature)
            {
                feature->getGeometry()->push_back( osg::Vec3d(lon_deg, lat_deg, 0) );
                _source->dirty();
                OE_NOTICE << "Added feature point at " << lon_deg << ", " << lat_deg << std::endl;                    
            }
        }
    }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        if ( ea.getEventType() == osgGA::GUIEventAdapter::PUSH )
        {
            osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());
            addPoint( ea.getX(), ea.getY(), view );
        }
        else if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
        {
            if (ea.getKey() == 'd')
            {
                _source->deleteFeature( 0 );
            }
        }

        return false;
    }

    osg::ref_ptr< FeatureListSource > _source;
    osg::ref_ptr<const SpatialReference> _mapSRS;
};




//
// NOTE: run this sample from the repo/tests directory.
//
int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    bool useRaster  = arguments.read("--rasterize");
    bool useOverlay = arguments.read("--overlay");
    bool useStencil = arguments.read("--stencil");
    bool useMem     = arguments.read("--mem");
    bool useLabels  = arguments.read("--labels");

    osgViewer::Viewer viewer(arguments);

    // Start by creating the map:
    Map* map = new Map();

    // Start with a basemap imagery layer; we'll be using the GDAL driver
    // to load a local GeoTIFF file:
    GDALOptions basemapOpt;
    basemapOpt.url() = "c:/dev/osgEarth/osgearth-master/data/world.tif";
    map->addImageLayer( new ImageLayer( ImageLayerOptions("basemap", basemapOpt) ) );

        
    // Define a style for the feature data. Since we are going to render the
    // vectors as lines, configure the line symbolizer:
    Style style;

    LineSymbol* ls = style.getOrCreateSymbol<LineSymbol>();
    ls->stroke()->color() = osg::Vec4f( 1,1,0,1 ); // yellow
    ls->stroke()->width() = 2.0f;

    osg::ref_ptr< FeatureListSource > featureSource = new FeatureListSource();
    //Ring* line = new Ring();
    LineString* line = new LineString();
    line->push_back( osg::Vec3d(-60, 20, 0) );
    line->push_back( osg::Vec3d(-120, 20, 0) );
    line->push_back( osg::Vec3d(-120, 60, 0) );
    line->push_back( osg::Vec3d(-60, 60, 0) );
    Feature *feature = new Feature(s_fid++);
    feature->setGeometry( line );

    featureSource->insertFeature( feature );



    // That's it, the map is ready; now create a MapNode to render the Map:
    MapNodeOptions mapNodeOptions;
    mapNodeOptions.enableLighting() = false;

    MapNode* mapNode = new MapNode( map, mapNodeOptions );
    

    FeatureGeomModelOptions worldOpt;
    worldOpt.featureSource() = featureSource;
    worldOpt.geometryTypeOverride() = Geometry::TYPE_LINESTRING;
    worldOpt.styles()->addStyle( style );
    worldOpt.enableLighting() = false;
    worldOpt.depthTestEnabled() = false;

    ModelLayerOptions options( "my features", worldOpt );
    options.overlay() = useOverlay;
    ModelLayer* layer = new ModelLayer(options);
    map->addModelLayer( layer );


    osg::Group* root = new osg::Group;
    root->addChild( mapNode );
       
    
    viewer.setSceneData( root );
    viewer.setCameraManipulator( new EarthManipulator() );

    if ( !useStencil && !useOverlay )
        viewer.addEventHandler( new osgEarth::Util::AutoClipPlaneHandler );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    //viewer.addEventHandler( new AddPointHandler(featureSource, map->getProfile()->getSRS()));
    addFeatureEditor( feature, featureSource, mapNode, root );

    return viewer.run();
}
