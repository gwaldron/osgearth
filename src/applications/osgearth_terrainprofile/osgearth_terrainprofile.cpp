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
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/MapNode>
#include <osgEarth/XmlUtils>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/TerrainProfile>
#include <osgEarth/GeoMath>
#include <osgEarthFeatures/Feature>
#include <osgEarthAnnotation/FeatureNode>
#include <osg/io_utils>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Symbology;
using namespace osgEarth::Features;
using namespace osgEarth::Annotation;



osg::Node* createBackground(double width, double height, const osg::Vec4f& backgroundColor)
{
    //Add a background quad
    osg::Geometry* geometry = new osg::Geometry();
    osg::Vec3Array* verts = new osg::Vec3Array();
    verts->reserve( 4 );
    verts->push_back( osg::Vec3(0,0,0));
    verts->push_back( osg::Vec3(width,0,0));
    verts->push_back( osg::Vec3(width,height,0));
    verts->push_back( osg::Vec3(0,height,0));
    geometry->setVertexArray( verts );

    osg::Vec4Array* colors = new osg::Vec4Array();
    colors->push_back( backgroundColor );
    geometry->setColorArray( colors );
    geometry->setColorBinding( osg::Geometry::BIND_OVERALL );

    geometry->addPrimitiveSet( new osg::DrawArrays( GL_QUADS, 0, 4 ) );

    osg::Geode* geode = new osg::Geode;
    geode->addDrawable( geometry );

    return geode;

}

osg::Camera* createHud(double width, double height)
{
    osg::Camera* hud = new osg::Camera;
    hud->setProjectionMatrix(osg::Matrix::ortho2D(0,width,0,height));    
    hud->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    hud->setViewMatrix(osg::Matrix::identity());    
    hud->setClearMask(GL_DEPTH_BUFFER_BIT);
    hud->setRenderOrder(osg::Camera::POST_RENDER);    
    hud->setAllowEventFocus(false);
    hud->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
    hud->getOrCreateStateSet()->setMode( GL_DEPTH_TEST, osg::StateAttribute::OFF);
    hud->getOrCreateStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON);

    return hud;
}

class TerrainProfileGraph : public osg::Group
{
public:

    struct GraphChangedCallback : public TerrainProfileCalculator::ChangedCallback
    {
        GraphChangedCallback( TerrainProfileGraph* graph):
    _graph( graph )
    {
    }

    virtual void onChanged(const TerrainProfileCalculator* sender )
    {
        _graph->setTerrainProfile( sender->getProfile() );
    }

    TerrainProfileGraph* _graph;
    };

    TerrainProfileGraph( TerrainProfileCalculator* profileCalculator, double graphWidth = 200, double graphHeight = 200 ):
      _profileCalculator( profileCalculator ),
      _graphWidth( graphWidth ),
      _graphHeight( graphHeight ),
      _color( 1.0f, 1.0f, 0.0f, 1.0f)
    {
        _graphChangedCallback = new GraphChangedCallback( this );
        _profileCalculator->addChangedCallback( _graphChangedCallback.get() );
    }

      ~TerrainProfileGraph()
      {
           _profileCalculator->removeChangedCallback( _graphChangedCallback.get() );
      }

      void setTerrainProfile( const TerrainProfile& profile)
      {
          _profile = profile;
          redraw();
      }

      void redraw()
      {
          removeChildren( 0, getNumChildren() );

          osg::Geometry* geom = new osg::Geometry;
          osg::Vec3Array* verts = new osg::Vec3Array();
          verts->reserve( _profile.getNumElevations() );
          geom->setVertexArray( verts );

          osg::Vec4Array* colors = new osg::Vec4Array();
          colors->push_back( _color );
          geom->setColorArray( colors );
          geom->setColorBinding( osg::Geometry::BIND_OVERALL );

          double minElevation, maxElevation;
          _profile.getElevationRanges( minElevation, maxElevation );
          double elevationRange = maxElevation - minElevation;

          double totalDistance = _profile.getTotalDistance();


          for (unsigned int i = 0; i < _profile.getNumElevations(); i++)
          {
              double distance = _profile.getDistance( i );
              double elevation = _profile.getElevation( i );

              double x = (distance / totalDistance) * _graphWidth;
              double y = ( (elevation - minElevation) / elevationRange) * _graphHeight;
              verts->push_back( osg::Vec3(x, y, 0 ) );
          }

          geom->addPrimitiveSet( new osg::DrawArrays( GL_LINE_STRIP, 0, verts->size()) );
          osg::Geode* geode = new osg::Geode;
          geode->addDrawable( geom );
          addChild( geode );
      }

      osg::Vec4f _color;
    TerrainProfile _profile;
    osg::ref_ptr< TerrainProfileCalculator > _profileCalculator;
    double _graphWidth, _graphHeight;
    osg::ref_ptr< GraphChangedCallback > _graphChangedCallback;
};

class DrawProfileEventHandler : public osgGA::GUIEventHandler
{
public:
    DrawProfileEventHandler(osgEarth::MapNode* mapNode, osg::Group* root, TerrainProfileCalculator* profileCalculator):
      _mapNode( mapNode ),
          _root( root ),
          _startValid( false ),
          _profileCalculator( profileCalculator )
      {
          _start = profileCalculator->getStart().vec3d();
          _end = profileCalculator->getEnd().vec3d();
          compute();
      }

      bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
      {
          if (ea.getEventType() == ea.PUSH && ea.getButton() == osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON)
          {
              osg::Vec3d world;
              if ( _mapNode->getTerrain()->getWorldCoordsUnderMouse( aa.asView(), ea.getX(), ea.getY(), world ))
              {
                  GeoPoint mapPoint;
                  _mapNode->getMap()->worldPointToMapPoint( world, mapPoint );

                  if (!_startValid)
                  {
                      _startValid = true;
                      _start = mapPoint.vec3d();
                      if (_featureNode.valid())
                      {
                          _root->removeChild( _featureNode.get() );
                          _featureNode = 0;
                      }
                  }
                  else
                  {
                      _end = mapPoint.vec3d();
                      compute();
                      _startValid = false;                    
                  }
              }        
          }
          return false;
      }

      void compute()
      {
          //Tell the calculator about the new start/end points
          _profileCalculator->setStartEnd( GeoPoint(_mapNode->getMapSRS(), _start.x(), _start.y(), 0),
                                           GeoPoint(_mapNode->getMapSRS(), _end.x(), _end.y(), 0));

          if (_featureNode.valid())
          {
              _root->removeChild( _featureNode.get() );
              _featureNode = 0;
          }

          LineString* line = new LineString();
          line->push_back( _start );
          line->push_back( _end );
          Feature* feature = new Feature(line, _mapNode->getMapSRS());
          feature->geoInterp() = GEOINTERP_GREAT_CIRCLE;    

          //Define a style for the line
          Style style;
          LineSymbol* ls = style.getOrCreateSymbol<LineSymbol>();
          ls->stroke()->color() = Color::Yellow;
          ls->stroke()->width() = 2.0f;
          ls->tessellation() = 20;

          style.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;

          feature->style() = style;

          _featureNode = new FeatureNode( _mapNode, feature );
          //Disable lighting
          _featureNode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
          _root->addChild( _featureNode.get() );

      }




      osgEarth::MapNode* _mapNode;      
      osg::Group* _root;
      TerrainProfileCalculator* _profileCalculator;
      osg::ref_ptr< FeatureNode > _featureNode;
      bool _startValid;
      osg::Vec3d _start;
      osg::Vec3d _end;  
};




int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osgViewer::Viewer viewer(arguments);

    // load the .earth file from the command line.
    osg::Node* earthNode = osgDB::readNodeFiles( arguments );
    if (!earthNode)
    {
        OE_NOTICE << "Unable to load earth model" << std::endl;
        return 1;
    }

    osg::Group* root = new osg::Group();

    osgEarth::MapNode * mapNode = osgEarth::MapNode::findMapNode( earthNode );
    if (!mapNode)
    {
        OE_NOTICE << "Could not find MapNode " << std::endl;
        return 1;
    }

    osgEarth::Util::EarthManipulator* manip = new EarthManipulator();    
    viewer.setCameraManipulator( manip );

    root->addChild( earthNode );    

    double backgroundWidth = 500;
    double backgroundHeight = 500;

    double graphWidth = 200;
    double graphHeight = 100;

    //Add the hud
    osg::Camera* hud = createHud( backgroundWidth, backgroundHeight );
    root->addChild( hud );

    osg::ref_ptr< TerrainProfileCalculator > calculator = new TerrainProfileCalculator(mapNode, 
                                                                                       GeoPoint(mapNode->getMapSRS(), -124.0, 40.0, 0),
                                                                                       GeoPoint(mapNode->getMapSRS(), -75.1, 39.2, 0)
                                                                                       );
    
    hud->addChild( createBackground( graphWidth, graphHeight, osg::Vec4(0,0,0,0.5)) );

    osg::Group* profileNode = new TerrainProfileGraph( calculator.get(), graphWidth, graphHeight );
    hud->addChild( profileNode );


    viewer.getCamera()->addCullCallback( new AutoClipPlaneCullCallback(mapNode->getMap()) );

    viewer.addEventHandler( new DrawProfileEventHandler( mapNode, root, calculator ) );

    // osgEarth benefits from pre-compilation of GL objects in the pager. In newer versions of
    // OSG, this activates OSG's IncrementalCompileOpeartion in order to avoid frame breaks.
    viewer.getDatabasePager()->setDoPreCompile( true );

    viewer.setSceneData( root );    

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgViewer::ThreadingHandler());
    viewer.addEventHandler(new osgViewer::LODScaleHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
