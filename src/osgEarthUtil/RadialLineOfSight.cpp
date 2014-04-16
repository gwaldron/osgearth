/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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
#include <osgEarth/DPLineSegmentIntersector>
#include <osgSim/LineOfSight>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>

using namespace osgEarth;
using namespace osgEarth::Util;

namespace
{
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
//_center(0,0,0),
_goodColor(0.0f, 1.0f, 0.0f, 1.0f),
_badColor(1.0f, 0.0f, 0.0f, 1.0f),
_outlineColor( 1.0f, 1.0f, 1.0f, 1.0f),
_displayMode( LineOfSight::MODE_SPLIT ),
//_altitudeMode( ALTMODE_ABSOLUTE ),
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
    setMapNode( 0L );
}

void
RadialLineOfSightNode::setMapNode( MapNode* mapNode )
{
    MapNode* oldMapNode = getMapNode();

    if ( oldMapNode != mapNode )
    {
        if ( oldMapNode )
        {
            if ( _terrainChangedCallback.valid() )
            {
                oldMapNode->getTerrain()->removeTerrainCallback( _terrainChangedCallback.get() );
            }
        }

        _mapNode = mapNode;

        if ( _mapNode.valid() && _terrainChangedCallback.valid() )
        {
            _mapNode->getTerrain()->addTerrainCallback( _terrainChangedCallback.get() );
        }

        compute( getNode() );
    }
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



const GeoPoint&
RadialLineOfSightNode::getCenter() const
{
    return _center;
}

void
RadialLineOfSightNode::setCenter(const GeoPoint& center)
{
    if (_center != center)
    {
        _center = center;
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
    if (_terrainOnly && getMapNode()) return getMapNode()->getTerrainEngine();
    return _mapNode.get();
}


void
RadialLineOfSightNode::terrainChanged( const osgEarth::TileKey& tileKey, osg::Node* terrain )
{
    OE_DEBUG << "RadialLineOfSightNode::terrainChanged" << std::endl;
    compute( getNode() );    
}

void
RadialLineOfSightNode::compute(osg::Node* node )
{
    if (_fill)
    {
        compute_fill( node );
    }
    else
    {
        compute_line( node );
    }
}

void
RadialLineOfSightNode::compute_line(osg::Node* node)
{    
    if ( !getMapNode() )
        return;

    GeoPoint centerMap;
    _center.transform( getMapNode()->getMapSRS(), centerMap );
    centerMap.toWorld( _centerWorld, getMapNode()->getTerrain() );

    bool isProjected = getMapNode()->getMapSRS()->isProjected();
    osg::Vec3d up = isProjected ? osg::Vec3d(0,0,1) : osg::Vec3d(_centerWorld);
    up.normalize();

    //Get the "side" vector
    osg::Vec3d side = isProjected ? osg::Vec3d(1,0,0) : up ^ osg::Vec3d(0,0,1);

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

    osg::ref_ptr<osgUtil::IntersectorGroup> ivGroup = new osgUtil::IntersectorGroup();

    for (unsigned int i = 0; i < (unsigned int)_numSpokes; i++)
    {
        double angle = delta * (double)i;
        osg::Quat quat(angle, up );
        osg::Vec3d spoke = quat * (side * _radius);
        osg::Vec3d end = _centerWorld + spoke;
        osg::ref_ptr<DPLineSegmentIntersector> dplsi = new DPLineSegmentIntersector( _centerWorld, end );
        ivGroup->addIntersector( dplsi.get() );
    }

    osgUtil::IntersectionVisitor iv;
    iv.setIntersector( ivGroup.get() );

    node->accept( iv );

    for (unsigned int i = 0; i < (unsigned int)_numSpokes; i++)
    {
        DPLineSegmentIntersector* los = dynamic_cast<DPLineSegmentIntersector*>(ivGroup->getIntersectors()[i].get());
        DPLineSegmentIntersector::Intersections& hits = los->getIntersections();

        osg::Vec3d start = los->getStart();
        osg::Vec3d end = los->getEnd();

        osg::Vec3d hit;
        bool hasLOS = hits.empty();
        if (!hasLOS)
        {
            hit = hits.begin()->getWorldIntersectPoint();
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
    
    //Remove all the children
    removeChildren(0, getNumChildren());
    addChild( mt );  

    for( LOSChangedCallbackList::iterator i = _changedCallbacks.begin(); i != _changedCallbacks.end(); i++ )
    {
        i->get()->onChanged();
    }	
}

void
RadialLineOfSightNode::compute_fill(osg::Node* node)
{
    if ( !getMapNode() )
        return;

    GeoPoint centerMap;
    _center.transform( getMapNode()->getMapSRS(), centerMap );
    centerMap.toWorld( _centerWorld, getMapNode()->getTerrain() );

    bool isProjected = getMapNode()->getMapSRS()->isProjected();

    osg::Vec3d up = isProjected ? osg::Vec3d(0,0,1) : osg::Vec3d(_centerWorld);
    up.normalize();

    //Get the "side" vector
    osg::Vec3d side = isProjected ? osg::Vec3d(1,0,0) : up ^ osg::Vec3d(0,0,1);

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

    osg::ref_ptr<osgUtil::IntersectorGroup> ivGroup = new osgUtil::IntersectorGroup();

    for (unsigned int i = 0; i < (unsigned int)_numSpokes; i++)
    {
        double angle = delta * (double)i;
        osg::Quat quat(angle, up );
        osg::Vec3d spoke = quat * (side * _radius);
        osg::Vec3d end = _centerWorld + spoke;        
        osg::ref_ptr<DPLineSegmentIntersector> dplsi = new DPLineSegmentIntersector( _centerWorld, end );
        ivGroup->addIntersector( dplsi.get() );
    }

    osgUtil::IntersectionVisitor iv;
    iv.setIntersector( ivGroup.get() );

    node->accept( iv );

    for (unsigned int i = 0; i < (unsigned int)_numSpokes; i++)
    {
        //Get the current hit
        DPLineSegmentIntersector* los = dynamic_cast<DPLineSegmentIntersector*>(ivGroup->getIntersectors()[i].get());
        DPLineSegmentIntersector::Intersections& hits = los->getIntersections();

        osg::Vec3d currEnd = los->getEnd();
        bool currHasLOS = hits.empty();
        osg::Vec3d currHit = currHasLOS ? osg::Vec3d() : hits.begin()->getWorldIntersectPoint();

        //Get the next hit
        unsigned int nextIndex = i + 1;
        if (nextIndex == _numSpokes) nextIndex = 0;
        DPLineSegmentIntersector* losNext = static_cast<DPLineSegmentIntersector*>(ivGroup->getIntersectors()[nextIndex].get());
        DPLineSegmentIntersector::Intersections& hitsNext = losNext->getIntersections();

        osg::Vec3d nextEnd = losNext->getEnd();
        bool nextHasLOS = hitsNext.empty();
        osg::Vec3d nextHit = nextHasLOS ? osg::Vec3d() : hitsNext.begin()->getWorldIntersectPoint();
        
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
        
    //Remove all the children
    removeChildren(0, getNumChildren());
    addChild( mt );  

    for( LOSChangedCallbackList::iterator i = _changedCallbacks.begin(); i != _changedCallbacks.end(); i++ )
    {
        i->get()->onChanged();
    }	
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

        if ( los->getMapNode() )
        {
            osg::Vec3d worldCenter = getNodeCenter( _node );

            //Convert center to mappoint since that is what LOS expects
            GeoPoint mapCenter;
            mapCenter.fromWorld( los->getMapNode()->getMapSRS(), worldCenter );

            los->setCenter( mapCenter ); //mapCenter.vec3d() );
        }
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
              _los->setCenter( position );

              //GeoPoint location(position);
              //if (_los->getAltitudeMode() == ALTMODE_RELATIVE)
              //{
              //    double z = _los->getCenter().z();
              //    location.z() = z;
              //}                  
              //_los->setCenter( location.vec3d() );
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
    if ( _los->getMapNode() )
    {
        osg::Vec3d center = _los->getCenterWorld();             
        GeoPoint centerMap;
        centerMap.fromWorld(_los->getMapNode()->getMapSRS(), center);    
        _dragger->setPosition(centerMap, false);        
    }
}
