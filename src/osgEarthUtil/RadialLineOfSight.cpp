/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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
#include <osgEarthUtil/RadialLineOfSight>
#include <osgEarth/TerrainEngineNode>
#include <osgSim/LineOfSight>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>

using namespace osgEarth;
using namespace osgEarth::Util;

namespace
{
    bool getRelativeWorld(double x, double y, double relativeHeight, MapNode* mapNode, osg::Vec3d& world )
    {
        GeoPoint mapPoint(mapNode->getMapSRS(), x, y);
        osg::Vec3d pos;
        mapPoint.toWorld( pos, mapNode->getTerrain() );
        //mapNode->getMap()->toWorldPoint(mapPoint, pos);

        osg::Vec3d up(0,0,1);
        const osg::EllipsoidModel* em = mapNode->getMap()->getProfile()->getSRS()->getEllipsoid();
        if (em)
        {
            up = em->computeLocalUpVector( world.x(), world.y(), world.z());
        }    
        up.normalize();

        double segOffset = 50000;

        osg::Vec3d start = pos + (up * segOffset);
        osg::Vec3d end = pos - (up * segOffset);
        
        osgUtil::LineSegmentIntersector* i = new osgUtil::LineSegmentIntersector( start, end );
        
        osgUtil::IntersectionVisitor iv;    
        iv.setIntersector( i );
        mapNode->getTerrainEngine()->accept( iv );

        osgUtil::LineSegmentIntersector::Intersections& results = i->getIntersections();
        if ( !results.empty() )
        {
            const osgUtil::LineSegmentIntersector::Intersection& result = *results.begin();
            world = result.getWorldIntersectPoint();
            world += up * relativeHeight;
            return true;
        }
        return false;    
    }


    osg::Vec3d getNodeCenter(osg::Node* node)
    {
        osg::NodePathList nodePaths = node->getParentalNodePaths();
        if ( nodePaths.empty() )
            return node->getBound().center();

        osg::NodePath path = nodePaths[0];

        osg::Matrixd localToWorld = osg::computeLocalToWorld( path );
        osg::Vec3d center = osg::Vec3d(0,0,0) * localToWorld;

        // if the tether node is a MT, we are set. If it's not, we need to get the
        // local bound and add its translation to the localToWorld. We cannot just use
        // the bounds directly because they are single precision (unless you built OSG
        // with double-precision bounding spheres, which you probably did not :)
        if ( !dynamic_cast<osg::MatrixTransform*>( node ) )
        {
            const osg::BoundingSphere& bs = node->getBound();
            center += bs.center();
        }   
        return center;
    }
}

//------------------------------------------------------------------------

namespace
{
    class RadialLineOfSightNodeTerrainChangedCallback : public osgEarth::TerrainCallback
    {
    public:
        RadialLineOfSightNodeTerrainChangedCallback( RadialLineOfSightNode* los ):
          _los(los)
        {
        }

        virtual void onTileAdded(const osgEarth::TileKey& tileKey, osg::Node* terrain, TerrainCallbackContext& )
        {
            _los->terrainChanged( tileKey, terrain );
        }

    private:
        RadialLineOfSightNode* _los;
    };
}


RadialLineOfSightNode::RadialLineOfSightNode( MapNode* mapNode):
_mapNode( mapNode ),
_numSpokes(20),
_radius(500),
_center(0,0,0),
_goodColor(0.0f, 1.0f, 0.0f, 1.0f),
_badColor(1.0f, 0.0f, 0.0f, 1.0f),
_outlineColor( 1.0f, 1.0f, 1.0f, 1.0f),
_displayMode( LineOfSight::MODE_SPLIT ),
_altitudeMode( ALTMODE_ABSOLUTE ),
_fill(false),
_terrainOnly( false )
{
    compute(getNode());
    _terrainChangedCallback = new RadialLineOfSightNodeTerrainChangedCallback( this );
    _mapNode->getTerrain()->addTerrainCallback( _terrainChangedCallback.get() );        
    setNumChildrenRequiringUpdateTraversal( 1 );
}

RadialLineOfSightNode::~RadialLineOfSightNode()
{    
    _mapNode->getTerrain()->removeTerrainCallback( _terrainChangedCallback.get() );
}

bool
RadialLineOfSightNode::getFill() const
{
    return _fill;
}

void
RadialLineOfSightNode::setFill( bool fill)
{
    if (_fill != fill)
    {
        _fill = fill;
        compute(getNode() );
    }
}

double
RadialLineOfSightNode::getRadius() const
{
    return _radius;
}

void
RadialLineOfSightNode::setRadius(double radius)
{
    if (_radius != radius)
    {
        _radius = osg::clampAbove(radius, 1.0);
        compute(getNode());
    }
}

int
RadialLineOfSightNode::getNumSpokes() const
{
    return _numSpokes;
}

void RadialLineOfSightNode::setNumSpokes(int numSpokes)
{
    if (numSpokes != _numSpokes)
    {
        _numSpokes = osg::clampAbove(numSpokes, 1);
        compute(getNode());
    }
}

const osg::Vec3d&
RadialLineOfSightNode::getCenterWorld() const
{
    return _centerWorld;
}



const osg::Vec3d&
RadialLineOfSightNode::getCenter() const
{
    return _center;
}

void
RadialLineOfSightNode::setCenter(const osg::Vec3d& center)
{
    if (_center != center)
    {
        _center = center;
        compute(getNode());
    }
}

AltitudeMode
RadialLineOfSightNode::getAltitudeMode() const
{
    return _altitudeMode;
}

void
RadialLineOfSightNode::setAltitudeMode( AltitudeMode mode )
{
    if (_altitudeMode != mode)
    {
        _altitudeMode = mode;
        compute(getNode());
    }
}

bool
RadialLineOfSightNode::getTerrainOnly() const
{
    return _terrainOnly;
}

void RadialLineOfSightNode::setTerrainOnly( bool terrainOnly )
{
    if (_terrainOnly != terrainOnly)
    {
        _terrainOnly = terrainOnly;
        compute(getNode());
    }
}

osg::Node*
RadialLineOfSightNode::getNode()
{
    if (_terrainOnly) return _mapNode->getTerrainEngine();
    return _mapNode.get();
}


void
RadialLineOfSightNode::terrainChanged( const osgEarth::TileKey& tileKey, osg::Node* terrain )
{
    OE_DEBUG << "RadialLineOfSightNode::terrainChanged" << std::endl;
    //Make a temporary group that contains both the old MapNode as well as the new incoming terrain.
    //Because this function is called from the database pager thread we need to include both b/c 
    //the new terrain isn't yet merged with the new terrain.
    osg::ref_ptr < osg::Group > group = new osg::Group;
    group->addChild( terrain );
    group->addChild( getNode() );
    compute( group, true );
}

void
RadialLineOfSightNode::compute(osg::Node* node, bool backgroundThread)
{
    if (_fill)
    {
        compute_fill( node, backgroundThread );
    }
    else
    {
        compute_line( node, backgroundThread );
    }
}

void
RadialLineOfSightNode::compute_line(osg::Node* node, bool backgroundThread)
{    
    GeoPoint( _mapNode->getMapSRS(), _center, _altitudeMode ).toWorld( _centerWorld, _mapNode->getTerrain() );

#if 0
    //Get the center point in geocentric    
    if (_altitudeMode == ALTMODE_ABSOLUTE)
    {
        GeoPoint center(_mapNode->getMapSRS(),_center,ALTMODE_ABSOLUTE);
        center.toWorld( _centerWorld, _mapNode->getTerrain() );
        //_mapNode->getMap()->toWorldPoint( center, _centerWorld );
    }
    else
    {
        getRelativeWorld(_center.x(), _center.y(), _center.z(), _mapNode.get(), _centerWorld );
    }
#endif

    osg::Vec3d up = osg::Vec3d(_centerWorld);
    up.normalize();

    //Get the "side" vector
    osg::Vec3d side = up ^ osg::Vec3d(0,0,1);

    //Get the number of spokes
    double delta = osg::PI * 2.0 / (double)_numSpokes;
    
    osg::Geometry* geometry = new osg::Geometry;
    geometry->setUseVertexBufferObjects(true);

    osg::Vec3Array* verts = new osg::Vec3Array();
    verts->reserve(_numSpokes * 5);
    geometry->setVertexArray( verts );

    osg::Vec4Array* colors = new osg::Vec4Array();
    colors->reserve( _numSpokes * 5 );

    geometry->setColorArray( colors );
    geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    osg::Vec3d previousEnd;
    osg::Vec3d firstEnd;

    osgSim::LineOfSight los;
    los.setDatabaseCacheReadCallback(0);

    for (unsigned int i = 0; i < (unsigned int)_numSpokes; i++)
    {
        double angle = delta * (double)i;
        osg::Quat quat(angle, up );
        osg::Vec3d spoke = quat * (side * _radius);
        osg::Vec3d end = _centerWorld + spoke;        
        los.addLOS( _centerWorld, end);      
    }

    los.computeIntersections(node);

    for (unsigned int i = 0; i < (unsigned int)_numSpokes; i++)
    {
        osg::Vec3d start = los.getStartPoint(i);
        osg::Vec3d end = los.getEndPoint(i);

        osgSim::LineOfSight::Intersections hits = los.getIntersections(i);
        osg::Vec3d hit;
        bool hasLOS = hits.empty();
        if (!hasLOS)
        {
            hit = *hits.begin();
        }

        if (hasLOS)
        {
            verts->push_back( start - _centerWorld );
            verts->push_back( end - _centerWorld );
            colors->push_back( _goodColor );
            colors->push_back( _goodColor );
        }
        else
        {
            if (_displayMode == LineOfSight::MODE_SPLIT)
            {
                verts->push_back( start - _centerWorld );
                verts->push_back( hit - _centerWorld  );
                colors->push_back( _goodColor );
                colors->push_back( _goodColor );

                verts->push_back( hit - _centerWorld );
                verts->push_back( end - _centerWorld );
                colors->push_back( _badColor );
                colors->push_back( _badColor );
            }
            else if (_displayMode == LineOfSight::MODE_SINGLE)
            {
                verts->push_back( start - _centerWorld );
                verts->push_back( end - _centerWorld );
                colors->push_back( _badColor );                                
                colors->push_back( _badColor );                
            }
        }


        if (i > 0)
        {
            verts->push_back( end - _centerWorld );
            verts->push_back( previousEnd - _centerWorld );
            colors->push_back( _outlineColor );
            colors->push_back( _outlineColor );
        }
        else
        {
            firstEnd = end;
        }

        previousEnd = end;
    }


    //Add the last outside of circle
    verts->push_back( firstEnd - _centerWorld );
    verts->push_back( previousEnd - _centerWorld );
    colors->push_back( osg::Vec4(1,1,1,1));
    colors->push_back( osg::Vec4(1,1,1,1));

    geometry->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, verts->size()));

    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( geometry );

    getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    osg::MatrixTransform* mt = new osg::MatrixTransform;
    mt->setMatrix(osg::Matrixd::translate(_centerWorld));
    mt->addChild(geode);
    
    if (!backgroundThread)
    {
        //Remove all the children
        removeChildren(0, getNumChildren());
        addChild( mt );  
    }
    else
    {
        _pendingNode = mt;
    }

    for( LOSChangedCallbackList::iterator i = _changedCallbacks.begin(); i != _changedCallbacks.end(); i++ )
    {
        i->get()->onChanged();
    }	
}

void
RadialLineOfSightNode::compute_fill(osg::Node* node, bool backgroundThread)
{    
    //Get the center point in geocentric    
    if (_altitudeMode == ALTMODE_ABSOLUTE)
    {
        GeoPoint centerMap(_mapNode->getMapSRS(), _center, ALTMODE_ABSOLUTE);
        centerMap.toWorld( _centerWorld, _mapNode->getTerrain() );
        //_mapNode->getMap()->toWorldPoint( centerMap, _centerWorld );
    }
    else
    {
        getRelativeWorld(_center.x(), _center.y(), _center.z(), _mapNode.get(), _centerWorld );
    }

    osg::Vec3d up = osg::Vec3d(_centerWorld);
    up.normalize();

    //Get the "side" vector
    osg::Vec3d side = up ^ osg::Vec3d(0,0,1);

    //Get the number of spokes
    double delta = osg::PI * 2.0 / (double)_numSpokes;
    
    osg::Geometry* geometry = new osg::Geometry;
    geometry->setUseVertexBufferObjects(true);

    osg::Vec3Array* verts = new osg::Vec3Array();
    verts->reserve(_numSpokes * 2);
    geometry->setVertexArray( verts );

    osg::Vec4Array* colors = new osg::Vec4Array();
    colors->reserve( _numSpokes * 2 );

    geometry->setColorArray( colors );
    geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    osgSim::LineOfSight los;
    los.setDatabaseCacheReadCallback(0);

    for (unsigned int i = 0; i < (unsigned int)_numSpokes; i++)
    {
        double angle = delta * (double)i;
        osg::Quat quat(angle, up );
        osg::Vec3d spoke = quat * (side * _radius);
        osg::Vec3d end = _centerWorld + spoke;        
        los.addLOS( _centerWorld, end);      
    }

    los.computeIntersections(node);

    for (unsigned int i = 0; i < (unsigned int)_numSpokes; i++)
    {
        //Get the current hit
        osg::Vec3d currEnd = los.getEndPoint(i);
        bool currHasLOS = los.getIntersections(i).empty();
        osg::Vec3d currHit = currHasLOS ? osg::Vec3d() : *los.getIntersections(i).begin();

        unsigned int nextIndex = i + 1;
        if (nextIndex == _numSpokes) nextIndex = 0;
        //Get the current hit
        osg::Vec3d nextEnd = los.getEndPoint(nextIndex);
        bool nextHasLOS = los.getIntersections(nextIndex).empty();
        osg::Vec3d nextHit = nextHasLOS ? osg::Vec3d() : *los.getIntersections(nextIndex).begin();
        
        if (currHasLOS && nextHasLOS)
        {
            //Both rays have LOS            
            verts->push_back( _centerWorld - _centerWorld );
            colors->push_back( _goodColor );
            
            verts->push_back( nextEnd - _centerWorld );
            colors->push_back( _goodColor );
            
            verts->push_back( currEnd - _centerWorld );                       
            colors->push_back( _goodColor );
        }        
        else if (!currHasLOS && !nextHasLOS)
        {
            //Both rays do NOT have LOS

            //Draw the "good triangle"            
            verts->push_back( _centerWorld - _centerWorld );
            colors->push_back( _goodColor );
            
            verts->push_back( nextHit - _centerWorld );
            colors->push_back( _goodColor );
            
            verts->push_back( currHit - _centerWorld );                       
            colors->push_back( _goodColor );

            //Draw the two bad triangles
            verts->push_back( currHit - _centerWorld );
            colors->push_back( _badColor );
            
            verts->push_back( nextHit - _centerWorld );
            colors->push_back( _badColor );
            
            verts->push_back( nextEnd - _centerWorld );                       
            colors->push_back( _badColor );

            verts->push_back( currHit - _centerWorld );
            colors->push_back( _badColor );
            
            verts->push_back( nextEnd - _centerWorld );
            colors->push_back( _badColor );
            
            verts->push_back( currEnd - _centerWorld );                       
            colors->push_back( _badColor );
        }
        else if (!currHasLOS && nextHasLOS)
        {
            //Current does not have LOS but next does

            //Draw the good portion
            verts->push_back( _centerWorld - _centerWorld );
            colors->push_back( _goodColor );
            
            verts->push_back( nextEnd - _centerWorld );
            colors->push_back( _goodColor );
            
            verts->push_back( currHit - _centerWorld );                       
            colors->push_back( _goodColor );

            //Draw the bad portion
            verts->push_back( currHit - _centerWorld );
            colors->push_back( _badColor );
            
            verts->push_back( nextEnd - _centerWorld );
            colors->push_back( _badColor );
            
            verts->push_back( currEnd - _centerWorld );                       
            colors->push_back( _badColor );
        }
        else if (currHasLOS && !nextHasLOS)
        {
            //Current does not have LOS but next does
            //Draw the good portion
            verts->push_back( _centerWorld - _centerWorld );
            colors->push_back( _goodColor );
            
            verts->push_back( nextHit - _centerWorld );
            colors->push_back( _goodColor );
            
            verts->push_back( currEnd - _centerWorld );                       
            colors->push_back( _goodColor );

            //Draw the bad portion
            verts->push_back( nextHit - _centerWorld );
            colors->push_back( _badColor );
            
            verts->push_back( nextEnd - _centerWorld );
            colors->push_back( _badColor );
            
            verts->push_back( currEnd - _centerWorld );                       
            colors->push_back( _badColor );
        }               
    }
    

    geometry->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, 0, verts->size()));

    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( geometry );

    getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);

    osg::MatrixTransform* mt = new osg::MatrixTransform;
    mt->setMatrix(osg::Matrixd::translate(_centerWorld));
    mt->addChild(geode);
    
    if (!backgroundThread)
    {
        //Remove all the children
        removeChildren(0, getNumChildren());
        addChild( mt );  
    }
    else
    {
        _pendingNode = mt;
    }

    for( LOSChangedCallbackList::iterator i = _changedCallbacks.begin(); i != _changedCallbacks.end(); i++ )
    {
        i->get()->onChanged();
    }	
}

void
RadialLineOfSightNode::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR)
    {
        if (_pendingNode.valid())
        {
            removeChildren(0, getNumChildren());
            addChild( _pendingNode.get());
            _pendingNode = 0;            
        }
    }
    osg::Group::traverse(nv);
}


void
RadialLineOfSightNode::setGoodColor( const osg::Vec4f &color )
{
    if (_goodColor != color)
    {
        _goodColor = color;
        compute(getNode());
    }
}

const osg::Vec4f&
RadialLineOfSightNode::getGoodColor() const
{
    return _goodColor;
}

void
RadialLineOfSightNode::setBadColor( const osg::Vec4f &color )
{
    if (_badColor != color)
    {
        _badColor = color;
        compute(getNode());
    }
}

const osg::Vec4f&
RadialLineOfSightNode::getBadColor() const
{
    return _badColor;
}

void
RadialLineOfSightNode::setOutlineColor( const osg::Vec4f &color )
{
    if (_outlineColor != color)
    {
        _outlineColor = color;
        compute(getNode());
    }
}

const osg::Vec4f&
RadialLineOfSightNode::getOutlineColor() const
{
    return _outlineColor;
}

LineOfSight::DisplayMode
RadialLineOfSightNode::getDisplayMode() const
{
    return _displayMode;
}

void
RadialLineOfSightNode::setDisplayMode( LineOfSight::DisplayMode displayMode )
{
    if (_displayMode != displayMode)
    {
        _displayMode = displayMode;
        compute(getNode());
    }
}

void
RadialLineOfSightNode::addChangedCallback( LOSChangedCallback* callback )
{
    _changedCallbacks.push_back( callback );
}

void
RadialLineOfSightNode::removeChangedCallback( LOSChangedCallback* callback )
{
    LOSChangedCallbackList::iterator i = std::find( _changedCallbacks.begin(), _changedCallbacks.end(), callback);
    if (i != _changedCallbacks.end())
    {
        _changedCallbacks.erase( i );
    }    
}



/**********************************************************************/
RadialLineOfSightTether::RadialLineOfSightTether(osg::Node* node):
_node(node)
{
}

void 
RadialLineOfSightTether::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    if (nv->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR)
    {
        RadialLineOfSightNode* los = static_cast<RadialLineOfSightNode*>(node);

        osg::Vec3d worldCenter = getNodeCenter( _node );

        //Convert center to mappoint since that is what LOS expects
        GeoPoint mapCenter;
        mapCenter.fromWorld( los->getMapNode()->getMapSRS(), worldCenter );
        //los->getMapNode()->getMap()->worldPointToMapPoint( worldCenter, mapCenter );

        los->setCenter( mapCenter.vec3d() );      
    }
    traverse(node, nv);
}


/**********************************************************************/


namespace
{
    struct RadialUpdateDraggersCallback : public LOSChangedCallback
    {
    public:
        RadialUpdateDraggersCallback( RadialLineOfSightEditor * editor ):
          _editor( editor )
        {

        }
        virtual void onChanged()
        {
            _editor->updateDraggers();
        }

        RadialLineOfSightEditor *_editor;
    };

        
    class RadialLOSDraggerCallback : public Dragger::PositionChangedCallback
    {
    public:
        RadialLOSDraggerCallback(RadialLineOfSightNode* los):
          _los(los)
          {
          }

          virtual void onPositionChanged(const Dragger* sender, const osgEarth::GeoPoint& position)
          {
              GeoPoint location(position);
              if (_los->getAltitudeMode() == ALTMODE_RELATIVE)
              {
                  double z = _los->getCenter().z();
                  location.z() = z;
              }                  
              _los->setCenter( location.vec3d() );
          }

          RadialLineOfSightNode* _los;
          bool _start;
    };
}



/**********************************************************************/

RadialLineOfSightEditor::RadialLineOfSightEditor(RadialLineOfSightNode* los):
_los(los)
{

    _dragger  = new SphereDragger(_los->getMapNode());
    _dragger->addPositionChangedCallback(new RadialLOSDraggerCallback(_los ) );    
    static_cast<SphereDragger*>(_dragger)->setColor(osg::Vec4(0,0,1,0));
    addChild(_dragger);    

    _callback = new RadialUpdateDraggersCallback( this );
    _los->addChangedCallback( _callback.get() );

    updateDraggers();
}

RadialLineOfSightEditor::~RadialLineOfSightEditor()
{
    _los->removeChangedCallback( _callback.get() );
}



void
RadialLineOfSightEditor::updateDraggers()
{
    osg::Vec3d center = _los->getCenterWorld();             
    GeoPoint centerMap;
    centerMap.fromWorld(_los->getMapNode()->getMapSRS(), center);    
    _dragger->setPosition(centerMap, false);        
}
