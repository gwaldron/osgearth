/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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

#include <osgEarthAnnotation/AnnotationEditing>
#include <osgEarth/GeoMath>
#include <osg/io_utils>

using namespace osgEarth::Annotation;
using namespace osgEarth::Symbology;

/**********************************************************************/
class DraggerCallback : public Dragger::PositionChangedCallback
{
public:
    DraggerCallback(LocalizedNode* node, LocalizedNodeEditor* editor):
      _node(node),
      _editor( editor )
      {          
      }

      virtual void onPositionChanged(const Dragger* sender, const osgEarth::GeoPoint& position)
      {
          _node->setPosition( position);                     
          _editor->updateDraggers();
      }

      LocalizedNode* _node;
      LocalizedNodeEditor* _editor;
};

/**********************************************************************/
LocalizedNodeEditor::LocalizedNodeEditor(LocalizedNode* node):
_node( node )
{
    _dragger  = new SphereDragger( _node->getMapNode());  
    _dragger->addPositionChangedCallback(new DraggerCallback(_node, this) );        
    addChild(_dragger);
    updateDraggers();
}

LocalizedNodeEditor::~LocalizedNodeEditor()
{    
}

void
LocalizedNodeEditor::updateDraggers()
{
    GeoPoint pos = _node->getPosition();    
    _dragger->setPosition( pos, false );
}

void
LocalizedNodeEditor::setPosition(const GeoPoint& pos)
{
    _node->setPosition( pos );
    updateDraggers();
}


/**********************************************************************/

class SetRadiusCallback : public Dragger::PositionChangedCallback
{
public:
    SetRadiusCallback(CircleNode* node, CircleNodeEditor* editor):
      _node(node),
      _editor( editor )
      {
      }

      virtual void onPositionChanged(const Dragger* sender, const osgEarth::GeoPoint& position)
      {
          const osg::EllipsoidModel* em = _node->getMapNode()->getMapSRS()->getEllipsoid();

          GeoPoint radiusLocation(position);
          radiusLocation.makeGeographic();

          //Figure out the distance between the center of the circle and this new location
          GeoPoint center = _node->getPosition();
          center.makeGeographic();

          double distance = GeoMath::distance(osg::DegreesToRadians( center.y() ), osg::DegreesToRadians( center.x() ), 
                                              osg::DegreesToRadians( radiusLocation.y() ), osg::DegreesToRadians( radiusLocation.x() ),
                                              em->getRadiusEquator());
          _node->setRadius( Linear(distance, Units::METERS ) );
          //The position of the radius dragger has changed, so recompute the bearing
          _editor->computeBearing();
      }

      CircleNode* _node;
      CircleNodeEditor* _editor;
};



CircleNodeEditor::CircleNodeEditor( CircleNode* node ):
LocalizedNodeEditor( node ),
_radiusDragger( 0 ),
_bearing( osg::DegreesToRadians( 90.0 ) )
{
    _radiusDragger  = new SphereDragger(_node->getMapNode());
    _radiusDragger->addPositionChangedCallback(new SetRadiusCallback( node,this ) );        
    static_cast<SphereDragger*>(_radiusDragger)->setColor(osg::Vec4(0,0,1,0));
    addChild(_radiusDragger);
    updateDraggers();
}

CircleNodeEditor::~CircleNodeEditor()
{
    //nop
}

void
CircleNodeEditor::setBearing( const Angle& bearing )
{
    _bearing = bearing.as(Units::RADIANS);
    updateDraggers();
}

void
CircleNodeEditor::computeBearing()
{
    _bearing = osg::DegreesToRadians( 90.0 );
    //Get the radius dragger's position
    if (!_radiusDragger->getMatrix().isIdentity())
    {
        // Get the current location of the center of the circle (in lat/long)
        GeoPoint location = _node->getPosition();
        location.makeGeographic();

        // location of the radius dragger (in lat/long)
        GeoPoint radiusLocation;
        radiusLocation.fromWorld( location.getSRS(), _radiusDragger->getMatrix().getTrans() );

        // calculate the bearing b/w the 
        _bearing = GeoMath::bearing(
            osg::DegreesToRadians(location.y()), osg::DegreesToRadians(location.x()),
            osg::DegreesToRadians(radiusLocation.y()), osg::DegreesToRadians(radiusLocation.x()));
    }
}

void
CircleNodeEditor::updateDraggers()
{
    LocalizedNodeEditor::updateDraggers();
    if (_radiusDragger)
    {
        const osg::EllipsoidModel* em = _node->getMapNode()->getMapSRS()->getEllipsoid();
        
        // Get the current location of the center of the circle (in lat/long, absolute Z)
        GeoPoint location = _node->getPosition();   
        location.makeGeographic();
        //location.makeAbsolute( _node->getMapNode()->getTerrain() );
        
        //Get the radius of the circle in meters
        double r = static_cast<CircleNode*>(_node.get())->getRadius().as(Units::METERS);

        double lat, lon;
        GeoMath::destination(
            osg::DegreesToRadians( location.y() ), osg::DegreesToRadians( location.x() ), 
            _bearing, r, lat, lon, em->getRadiusEquator() );

        GeoPoint draggerLocation( 
            location.getSRS(),
            osg::RadiansToDegrees(lon),
            osg::RadiansToDegrees(lat));

        draggerLocation.z() = 0;

        _radiusDragger->setPosition( draggerLocation, false );
    }
}



/***************************************************************************************************/


class SetEllipseRadiusCallback : public Dragger::PositionChangedCallback
{
public:
    SetEllipseRadiusCallback(EllipseNode* node, EllipseNodeEditor* editor, bool major):
      _node(node),
      _editor( editor ),
      _major( major )
      {
      }

      virtual void onPositionChanged(const Dragger* sender, const osgEarth::GeoPoint& position)
      {
          const osg::EllipsoidModel* em = _node->getMapNode()->getMapSRS()->getEllipsoid();

          //Figure out the distance between the center of the circle and this new location
          GeoPoint center = _node->getPosition();
          double distance = GeoMath::distance(osg::DegreesToRadians( center.y() ), osg::DegreesToRadians( center.x() ), 
                                              osg::DegreesToRadians( position.y() ), osg::DegreesToRadians( position.x() ),
                                              em->getRadiusEquator());

          double bearing = GeoMath::bearing(osg::DegreesToRadians( center.y() ), osg::DegreesToRadians( center.x() ), 
                                            osg::DegreesToRadians( position.y() ), osg::DegreesToRadians( position.x() ));
        

          //Compute the new angular rotation based on how they moved the point
          if (_major)
          {
              _node->setRotationAngle( Angular( -bearing, Units::RADIANS ) );
              _node->setRadiusMajor( Linear(distance, Units::METERS ) );
          }
          else
          {
              _node->setRotationAngle( Angular( osg::PI_2 - bearing, Units::RADIANS ) );
              _node->setRadiusMinor( Linear(distance, Units::METERS ) );
          }
          _editor->updateDraggers();
     }



      EllipseNode* _node;
      EllipseNodeEditor* _editor;
      bool _major;
};





EllipseNodeEditor::EllipseNodeEditor( EllipseNode* node ):
LocalizedNodeEditor( node ),
_minorDragger( 0 ),
_majorDragger( 0 )
{
    _minorDragger  = new SphereDragger( _node->getMapNode());
    _minorDragger->addPositionChangedCallback(new SetEllipseRadiusCallback( node, this, false ) );        
    static_cast<SphereDragger*>(_minorDragger)->setColor(osg::Vec4(0,0,1,0));
    addChild(_minorDragger);

    _majorDragger  = new SphereDragger( _node->getMapNode());
    _majorDragger->addPositionChangedCallback(new SetEllipseRadiusCallback( node, this, true ) );        
    static_cast<SphereDragger*>(_majorDragger)->setColor(osg::Vec4(1,0,0,1));
    addChild(_majorDragger);

    updateDraggers();
}

EllipseNodeEditor::~EllipseNodeEditor()
{
}

void
EllipseNodeEditor::updateDraggers()
{
    LocalizedNodeEditor::updateDraggers();
    if (_majorDragger && _minorDragger)
    {
        const osg::EllipsoidModel* em = _node->getMapNode()->getMap()->getProfile()->getSRS()->getEllipsoid();
        
        //Get the current location of the center of the circle
        GeoPoint location = _node->getPosition();    
        
        //Get the raddi of the ellipse in meters
        EllipseNode* ellipse = static_cast<EllipseNode*>(_node.get());
        double majorR = ellipse->getRadiusMajor().as(Units::METERS);
        double minorR = ellipse->getRadiusMinor().as(Units::METERS);

        double rotation = ellipse->getRotationAngle().as( Units::RADIANS );

        double lat, lon;
        GeoMath::destination(osg::DegreesToRadians( location.y() ), osg::DegreesToRadians( location.x() ), osg::PI_2 - rotation, minorR, lat, lon, em->getRadiusEquator());        

        GeoPoint minorLocation(location.getSRS(), osg::RadiansToDegrees( lon ), osg::RadiansToDegrees( lat ));
        minorLocation.z() = 0;       
        _minorDragger->setPosition( minorLocation, false);

        GeoMath::destination(osg::DegreesToRadians( location.y() ), osg::DegreesToRadians( location.x() ), -rotation, majorR, lat, lon, em->getRadiusEquator());                
        GeoPoint majorLocation(location.getSRS(), osg::RadiansToDegrees( lon ), osg::RadiansToDegrees( lat ));
        majorLocation.z() = 0;
        _majorDragger->setPosition( majorLocation, false);
    }
}


/***************************************************************************************************/
class SetCornerDragger : public Dragger::PositionChangedCallback
{
public:
    SetCornerDragger(RectangleNode* node, RectangleNodeEditor* editor, RectangleNode::Corner corner):
      _node(node),
      _editor( editor ),
      _corner( corner )
      {
      }

      virtual void onPositionChanged(const Dragger* sender, const osgEarth::GeoPoint& position)
      {
          _node->setCorner( _corner, position );                     
          _editor->updateDraggers();
      }

      RectangleNode* _node;
      RectangleNodeEditor* _editor;
      RectangleNode::Corner _corner;
};





RectangleNodeEditor::RectangleNodeEditor( RectangleNode* node ):
LocalizedNodeEditor( node ),
_llDragger( 0 ),
_lrDragger( 0 ),
_urDragger( 0 ),
_ulDragger( 0 )
{
    //Lower left
    _llDragger  = new SphereDragger(_node->getMapNode());
    _llDragger->addPositionChangedCallback(new SetCornerDragger( node, this, RectangleNode::CORNER_LOWER_LEFT ) );        
    static_cast<SphereDragger*>(_llDragger)->setColor(osg::Vec4(0,0,1,0));
    addChild(_llDragger);    

    //Lower right
    _lrDragger  = new SphereDragger(_node->getMapNode());
    _lrDragger->addPositionChangedCallback(new SetCornerDragger( node, this, RectangleNode::CORNER_LOWER_RIGHT ) );        
    static_cast<SphereDragger*>(_lrDragger)->setColor(osg::Vec4(0,0,1,0));
    addChild(_lrDragger);    

    //Upper right
    _urDragger  = new SphereDragger(_node->getMapNode());
    _urDragger->addPositionChangedCallback(new SetCornerDragger( node, this, RectangleNode::CORNER_UPPER_RIGHT ) );        
    static_cast<SphereDragger*>(_urDragger)->setColor(osg::Vec4(0,0,1,0));
    addChild(_urDragger);    

    //Upper left
    _ulDragger  = new SphereDragger(_node->getMapNode());
    _ulDragger->addPositionChangedCallback(new SetCornerDragger( node, this, RectangleNode::CORNER_UPPER_LEFT ) );        
    static_cast<SphereDragger*>(_ulDragger)->setColor(osg::Vec4(0,0,1,0));
    addChild(_ulDragger);    

    updateDraggers();
}

RectangleNodeEditor::~RectangleNodeEditor()
{
}

void
RectangleNodeEditor::updateDraggers()
{
    LocalizedNodeEditor::updateDraggers();    

    RectangleNode* rect = static_cast<RectangleNode*>(_node.get());
    
    osg::Matrixd matrix;

    _ulDragger->setPosition( rect->getUpperLeft(), false);
    _llDragger->setPosition( rect->getLowerLeft(), false);
    _urDragger->setPosition( rect->getUpperRight(), false);    
    _lrDragger->setPosition( rect->getLowerRight(), false);
}
