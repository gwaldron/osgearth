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
#include <osgEarth/Utils>

#include <osgEarthSymbology/Style>
#include <osgEarthFeatures/FeatureModelGraph>
#include <osgEarthFeatures/FeatureListSource>
#include <osgEarthFeatures/GeometryCompiler>

#include <osgEarthDrivers/gdal/GDALOptions>
#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>
#include <osgEarthDrivers/agglite/AGGLiteOptions>
#include <osgEarthDrivers/model_feature_geom/FeatureGeomModelOptions>
#include <osgEarthDrivers/model_feature_stencil/FeatureStencilModelOptions>

#include <osgEarthUtil/Controls>
#include <osgEarthUtil/Draggers>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers;
using namespace osgEarth::Symbology;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;

osg::Vec4
randomColor()
{
    float r = (float)rand() / (float)RAND_MAX;
    float g = (float)rand() / (float)RAND_MAX;
    float b = (float)rand() / (float)RAND_MAX;
    return osg::Vec4(r,g,b,1.0f);
}


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


osg::Node* createFeatureEditor(Feature* feature, FeatureSource* source, MapNode* mapNode)
{    
    osg::Group* editor = new osg::Group; 
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

        editor->addChild(dragger);        
    }
    return editor;   
}



struct AddPointHandler : public osgGA::GUIEventHandler 
{
    AddPointHandler(Feature* feature, FeatureListSource* source, const osgEarth::SpatialReference* mapSRS):
_feature(feature),
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

            if (_feature.valid())            
            {
                _feature->getGeometry()->push_back( osg::Vec3d(lon_deg, lat_deg, 0) );
                _source->dirty();
                //OE_NOTICE << "Added feature point at " << lon_deg << ", " << lat_deg << std::endl;                    
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
        return false;
    }

    osg::ref_ptr< FeatureListSource > _source;
    osg::ref_ptr< Feature > _feature;
    osg::ref_ptr<const SpatialReference> _mapSRS;
};



static osg::ref_ptr< AddPointHandler > s_addPointHandler;
static osg::ref_ptr< osg::Node > s_editor;
static osg::ref_ptr< Feature > s_activeFeature;
static osgViewer::Viewer* s_viewer;
static osg::ref_ptr< osg::Group > s_root;
static osg::ref_ptr< FeatureListSource > s_source;
static osg::ref_ptr< MapNode > s_mapNode;

Grid* createToolBar()
{    
    Grid* toolbar = new Grid();
    toolbar->setBackColor(0,0,0,0.5);
    toolbar->setMargin( 10 );
    toolbar->setPadding( 10 );
    toolbar->setChildSpacing( 10 );
    toolbar->setChildVertAlign( Control::ALIGN_CENTER );
    toolbar->setAbsorbEvents( true );
    toolbar->setVertAlign( Control::ALIGN_TOP );    
    return toolbar;    
}

struct AddVertsModeHandler : public ControlEventHandler
{
    AddVertsModeHandler( FeatureModelGraph* featureGraph):
_featureGraph( featureGraph )
    {
    }

    void onClick( Control* control, int mouseButtonMask ) {

        //remove the editor if it's valid
        if (s_editor.valid())
        {
            s_root->removeChild( s_editor );
            s_editor = NULL;

            Style outStyle;
            if (_featureGraph->getStyles().getDefaultStyle( outStyle))
            {            
                outStyle.getSymbol<LineSymbol>()->stroke()->stipple().unset();
                _featureGraph->setStyle( _featureGraph->getStyles() );
            }
        }

        //Add the new add point handler
        if (!s_addPointHandler.valid() && s_activeFeature.valid())
        {
            s_addPointHandler = new AddPointHandler(s_activeFeature, s_source.get(), s_mapNode->getMap()->getProfile()->getSRS());
            s_viewer->addEventHandler( s_addPointHandler.get() );
        }        
    }

    osg::ref_ptr< FeatureModelGraph > _featureGraph;
};

struct EditModeHandler : public ControlEventHandler
{
    EditModeHandler( FeatureModelGraph* featureGraph):
_featureGraph( featureGraph )
    { 
    }

    void onClick( Control* control, int mouseButtonMask ) {
        
        //Remove the add point handler if it's valid
        if (s_addPointHandler.valid())
        {            
            osgEarth::removeEventHandler( s_viewer, s_addPointHandler.get() );
            s_addPointHandler = NULL;
        }        

        if (!s_editor.valid() && s_activeFeature.valid())
        {
            Style outStyle;
            if (_featureGraph->getStyles().getDefaultStyle( outStyle))
            {            
                outStyle.getSymbol<LineSymbol>()->stroke()->stipple() =  0x00FF ;
                _featureGraph->setStyle( _featureGraph->getStyles() );
            }
            s_editor = createFeatureEditor(s_activeFeature, s_source, s_mapNode);
            s_root->addChild( s_editor );
        }
    }

    osg::ref_ptr< FeatureModelGraph > _featureGraph;
};

struct ChangeStyleHandler : public ControlEventHandler
{
    ChangeStyleHandler( FeatureModelGraph * features, const StyleSheet& styleSheet):
_features( features),
_styleSheet(styleSheet)
    { 
    }

    void onClick( Control* control, int mouseButtonMask ) {
        _features->setStyle( _styleSheet );
    }

    osg::ref_ptr< FeatureModelGraph > _features;
    StyleSheet _styleSheet;
};

StyleSheet buildStyleSheet( const osg::Vec4 &color, float width )
{
    // Define a style for the feature data. Since we are going to render the
    // vectors as lines, configure the line symbolizer:
    Style style;

    LineSymbol* ls = style.getOrCreateSymbol<LineSymbol>();
    ls->stroke()->color() = color;
    ls->stroke()->width() = width;    
    style.addSymbol( ls );

    StyleSheet styleSheet;
    styleSheet.addStyle( style );
    return styleSheet;
}


//
// NOTE: run this sample from the repo/tests directory.
//
int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    bool useOverlay = arguments.read("--overlay");

    osgViewer::Viewer viewer(arguments);

    s_viewer = &viewer;

    // Start by creating the map:
    Map* map = new Map();

    // Start with a basemap imagery layer; we'll be using the GDAL driver
    // to load a local GeoTIFF file:
    GDALOptions basemapOpt;
    basemapOpt.url() = "../data/world.tif";
    map->addImageLayer( new ImageLayer( ImageLayerOptions("basemap", basemapOpt) ) );

        
    // Define a style for the feature data. Since we are going to render the
    // vectors as lines, configure the line symbolizer:
    Style style;

    LineSymbol* ls = style.getOrCreateSymbol<LineSymbol>();
    ls->stroke()->color() = osg::Vec4f( 1,1,0,1 ); // yellow
    ls->stroke()->width() = 2.0f;

    s_source = new FeatureListSource();
    //Ring* line = new Ring();
    LineString* line = new LineString();
    line->push_back( osg::Vec3d(-60, 20, 0) );
    line->push_back( osg::Vec3d(-120, 20, 0) );
    line->push_back( osg::Vec3d(-120, 60, 0) );
    line->push_back( osg::Vec3d(-60, 60, 0) );
    Feature *feature = new Feature(s_fid++);
    feature->setGeometry( line );
    s_source->insertFeature( feature );
    s_activeFeature = feature;



    // That's it, the map is ready; now create a MapNode to render the Map:
    MapNodeOptions mapNodeOptions;
    mapNodeOptions.enableLighting() = false;

    s_mapNode = new MapNode( map, mapNodeOptions );
    s_mapNode->setNodeMask( 0x01 );
  

    s_root = new osg::Group;
    s_root->addChild( s_mapNode );

    StyleSheet styleSheet;
    styleSheet.addStyle( style );

    FeatureModelGraph* graph = new FeatureModelGraph( 
        s_source.get(), 
        FeatureModelSourceOptions(), 
        new GeomFeatureNodeFactory(),
        styleSheet,
        new Session( map ) );

    graph->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    graph->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);


    s_root->addChild( graph );

    //Setup the controls
    ControlCanvas* canvas = ControlCanvas::get( &viewer );
    s_root->addChild( canvas );
    Grid *toolbar = createToolBar( );
    canvas->addControl( toolbar );
    canvas->setNodeMask( 0x1 << 1 );



    int col = 0;
    LabelControl* addVerts = new LabelControl("Add Verts");
    toolbar->setControl(col++, 0, addVerts );    
    addVerts->addEventHandler( new AddVertsModeHandler( graph ));
    
    LabelControl* edit = new LabelControl("Edit");
    toolbar->setControl(col++, 0, edit );    
    edit->addEventHandler(new EditModeHandler( graph ));

    unsigned int row = 0;
    Grid *styleBar = createToolBar( );
    styleBar->setPosition(0, 50);
    canvas->addControl( styleBar );
    
    //Make a list of styles
    styleBar->setControl(0, row++, new LabelControl("Styles") );    

    int numStyles = 8;
    for (unsigned int i = 0; i < numStyles; ++i)
    {
        float w = 50;
        osg::Vec4 color = randomColor();

        float widths[3] = {2, 4, 8};

        unsigned int r = row++;
        for (unsigned int j = 0; j < 3; j++) 
        {
            Control* l = new Control();            
            l->setBackColor( color );
            l->addEventHandler(new ChangeStyleHandler(graph, buildStyleSheet( color, widths[j] ) ));
            l->setSize(w,5 * widths[j]);
            styleBar->setControl(j, r, l);
        }
    }
   
    
    viewer.setSceneData( s_root );
    viewer.setCameraManipulator( new EarthManipulator() );

    if ( !useOverlay )
        viewer.addEventHandler( new osgEarth::Util::AutoClipPlaneHandler );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    //viewer.addEventHandler( new AddPointHandler(feature, featureSource, map->getProfile()->getSRS()));
    //addFeatureEditor( feature, featureSource, mapNode, root );

    return viewer.run();
}
