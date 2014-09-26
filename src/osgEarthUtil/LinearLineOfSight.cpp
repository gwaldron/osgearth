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
#include <osgEarthUtil/LinearLineOfSight>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/DPLineSegmentIntersector>
#include <osgSim/LineOfSight>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>

using namespace osgEarth;
using namespace osgEarth::Util;

namespace
{
    class TerrainChangedCallback : public osgEarth::TerrainCallback
    {
    public:
        TerrainChangedCallback( LinearLineOfSightNode* los ):
          _los(los)
        {
        }

        virtual void onTileAdded(const osgEarth::TileKey& tileKey, osg::Node* terrain, TerrainCallbackContext&)
        {
            _los->terrainChanged( tileKey, terrain );
        }

    private:
        LinearLineOfSightNode* _los;
    }; 


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


LinearLineOfSightNode::LinearLineOfSightNode(osgEarth::MapNode *mapNode):
LineOfSightNode(),
_mapNode(mapNode),
_hasLOS( true ),
_goodColor(0.0f, 1.0f, 0.0f, 1.0f),
_badColor(1.0f, 0.0f, 0.0f, 1.0f),
_displayMode( LineOfSight::MODE_SPLIT ),
_terrainOnly( false )
{
    compute(getNode());
    subscribeToTerrain();    
}


LinearLineOfSightNode::LinearLineOfSightNode(osgEarth::MapNode* mapNode, 
                                             const GeoPoint&    start,
                                             const GeoPoint&    end ) :
LineOfSightNode(),
_mapNode(mapNode),
_start(start),
_end(end),
_hasLOS( true ),
_goodColor(0.0f, 1.0f, 0.0f, 1.0f),
_badColor(1.0f, 0.0f, 0.0f, 1.0f),
_displayMode( LineOfSight::MODE_SPLIT ),
_terrainOnly( false )
{
    compute(getNode());    
    subscribeToTerrain();    
}


void
LinearLineOfSightNode::subscribeToTerrain()
{
    _terrainChangedCallback = new TerrainChangedCallback( this );
    _mapNode->getTerrain()->addTerrainCallback( _terrainChangedCallback.get() );        
}

LinearLineOfSightNode::~LinearLineOfSightNode()
{
    //Unsubscribe to the terrain callback
    setMapNode( 0L );
}

void
LinearLineOfSightNode::setMapNode( MapNode* mapNode )
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

void
LinearLineOfSightNode::terrainChanged( const osgEarth::TileKey& tileKey, osg::Node* terrain )
{
    compute( getNode() );
}

const GeoPoint&
LinearLineOfSightNode::getStart() const
{
    return _start;
}

void
LinearLineOfSightNode::setStart(const GeoPoint& start)
{
    if (_start != start)
    {
        _start = start;
        compute(getNode());
    }
}

const GeoPoint&
LinearLineOfSightNode::getEnd() const
{
    return _end;
}

void
LinearLineOfSightNode::setEnd(const GeoPoint& end)
{
    if (_end != end)
    {
        _end = end;
        compute(getNode());
    }
}

const osg::Vec3d&
LinearLineOfSightNode::getStartWorld() const
{
    return _startWorld;
}

const osg::Vec3d&
LinearLineOfSightNode::getEndWorld() const
{
    return _endWorld;
}

const osg::Vec3d&
LinearLineOfSightNode::getHitWorld() const
{
    return _hitWorld;
}

const GeoPoint&
LinearLineOfSightNode::getHit() const
{
    return _hit;
}

bool
LinearLineOfSightNode::getHasLOS() const
{
    return _hasLOS;
}

void
LinearLineOfSightNode::addChangedCallback( LOSChangedCallback* callback )
{
    _changedCallbacks.push_back( callback );
}

void
LinearLineOfSightNode::removeChangedCallback( LOSChangedCallback* callback )
{
    LOSChangedCallbackList::iterator i = std::find( _changedCallbacks.begin(), _changedCallbacks.end(), callback);
    if (i != _changedCallbacks.end())
    {
        _changedCallbacks.erase( i );
    }    
}

void
LinearLineOfSightNode::compute(osg::Node* node, bool backgroundThread)
{    
    if ( !getMapNode() )
        return;

    if (!_start.isValid() || !_end.isValid() )
    {
        return;          
    }

    if (_start != _end)
    {
      const SpatialReference* mapSRS = getMapNode()->getMapSRS();
      const Terrain* terrain = getMapNode()->getTerrain();

      //Computes the LOS and redraws the scene      
      if (!_start.transform(mapSRS).toWorld( _startWorld, terrain ) || !_end.transform(mapSRS).toWorld( _endWorld, terrain ))
      {
          return;
      }


      DPLineSegmentIntersector* lsi = new DPLineSegmentIntersector(_startWorld, _endWorld);
      osgUtil::IntersectionVisitor iv( lsi );

      node->accept( iv );

      DPLineSegmentIntersector::Intersections& hits = lsi->getIntersections();
      if ( hits.size() > 0 )
      {
          _hasLOS = false;
          _hitWorld = hits.begin()->getWorldIntersectPoint();
          _hit.fromWorld( mapSRS, _hitWorld );
      }
      else
      {
          _hasLOS = true;
      }
    }

    draw(backgroundThread);

    for( LOSChangedCallbackList::iterator i = _changedCallbacks.begin(); i != _changedCallbacks.end(); i++ )
    {
        i->get()->onChanged();
    }	
}

void
LinearLineOfSightNode::draw(bool backgroundThread)
{
    osg::MatrixTransform* mt = 0L;

    if (_start != _end)
    {
        osg::Geometry* geometry = new osg::Geometry;
        geometry->setUseVertexBufferObjects(true);

        osg::Vec3Array* verts = new osg::Vec3Array();
        verts->reserve(4);
        geometry->setVertexArray( verts );

        osg::Vec4Array* colors = new osg::Vec4Array();
        colors->reserve( 4 );

        geometry->setColorArray( colors );
        geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

        if (_hasLOS)
        {
            verts->push_back( _startWorld - _startWorld );
            verts->push_back( _endWorld   - _startWorld );
            colors->push_back( _goodColor );
            colors->push_back( _goodColor );
        }
        else
        {
            if (_displayMode == LineOfSight::MODE_SINGLE)
            {
                verts->push_back( _startWorld - _startWorld );
                verts->push_back( _endWorld - _startWorld );
                colors->push_back( _badColor );
                colors->push_back( _badColor );
            }
            else if (_displayMode == LineOfSight::MODE_SPLIT)
            {
                verts->push_back( _startWorld - _startWorld );
                colors->push_back( _goodColor );
                verts->push_back( _hitWorld   - _startWorld );
                colors->push_back( _goodColor );

                verts->push_back( _hitWorld   - _startWorld );
                colors->push_back( _badColor );
                verts->push_back( _endWorld   - _startWorld );
                colors->push_back( _badColor );
            }
        }

        geometry->addPrimitiveSet( new osg::DrawArrays( GL_LINES, 0, verts->size()) );        

        osg::Geode* geode = new osg::Geode;
        geode->addDrawable( geometry );

        mt = new osg::MatrixTransform;
        mt->setMatrix(osg::Matrixd::translate(_startWorld));
        mt->addChild(geode);  

        getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    }


        //Remove all children from this group
        removeChildren(0, getNumChildren());

        if (mt)
          addChild( mt );
}

void
LinearLineOfSightNode::setGoodColor( const osg::Vec4f &color )
{
    if (_goodColor != color)
    {
        _goodColor = color;
        draw();
    }
}

const osg::Vec4f&
LinearLineOfSightNode::getGoodColor() const
{
    return _goodColor;
}

void
LinearLineOfSightNode::setBadColor( const osg::Vec4f &color )
{
    if (_badColor != color)
    {
        _badColor = color;
        draw();
    }
}

const osg::Vec4f&
LinearLineOfSightNode::getBadColor() const
{
    return _badColor;
}

LineOfSight::DisplayMode
LinearLineOfSightNode::getDisplayMode() const
{
    return _displayMode;
}

void
LinearLineOfSightNode::setDisplayMode( LineOfSight::DisplayMode displayMode )
{
    if (_displayMode != displayMode)
    {
        _displayMode = displayMode;
        draw();
    }
}

bool
LinearLineOfSightNode::getTerrainOnly() const
{
    return _terrainOnly;
}

void
LinearLineOfSightNode::setTerrainOnly( bool terrainOnly )
{
    if (_terrainOnly != terrainOnly)
    {
        _terrainOnly = terrainOnly;
        compute(getNode());
    }
}

osg::Node*
LinearLineOfSightNode::getNode()
{
    if (_terrainOnly && _mapNode.valid() )
    {
        return _mapNode->getTerrainEngine();
    }
    return _mapNode.get();
}

/**********************************************************************/
LineOfSightTether::LineOfSightTether(osg::Node* startNode, osg::Node* endNode):
_startNode(startNode),
_endNode(endNode)
{
}

void 
LineOfSightTether::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    if (nv->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR)
    {
        LinearLineOfSightNode* los = static_cast<LinearLineOfSightNode*>(node);

        if ( los->getMapNode() )
        {
            if (_startNode.valid())
            {
                osg::Vec3d worldStart = getNodeCenter(_startNode);

                //Convert to mappoint since that is what LOS expects
                GeoPoint mapStart;
                mapStart.fromWorld( los->getMapNode()->getMapSRS(), worldStart );
                los->setStart( mapStart ); //.vec3d() );
            }

            if (_endNode.valid())
            {
                osg::Vec3d worldEnd = getNodeCenter( _endNode );

                //Convert to mappoint since that is what LOS expects
                GeoPoint mapEnd;
                mapEnd.fromWorld( los->getMapNode()->getMapSRS(), worldEnd );
                los->setEnd( mapEnd ); //.vec3d() );
            }
        }
    }
    traverse(node, nv);
}


/**********************************************************************/

namespace 
{
    class LOSDraggerCallback : public Dragger::PositionChangedCallback
    {
    public:
        LOSDraggerCallback(LinearLineOfSightNode* los, bool start):
          _los(los),
          _start(start)
          {      
          }

          virtual void onPositionChanged(const Dragger* sender, const osgEarth::GeoPoint& position)
          {   
              if ( _start )
                  _los->setStart( position );
              else
                  _los->setEnd( position );          
          }

          
          LinearLineOfSightNode* _los;
          bool _start;
    };
}

/**********************************************************************/

namespace
{
    struct LOSUpdateDraggersCallback : public LOSChangedCallback
    {
    public:
        LOSUpdateDraggersCallback( LinearLineOfSightEditor * editor ):
          _editor( editor )
        {

        }
        virtual void onChanged()
        {
            _editor->updateDraggers();
        }

        LinearLineOfSightEditor *_editor;
    };
}

LinearLineOfSightEditor::LinearLineOfSightEditor(LinearLineOfSightNode* los):
_los(los)
{
    _startDragger  = new SphereDragger( _los->getMapNode());
    _startDragger->addPositionChangedCallback(new LOSDraggerCallback(_los, true ) );    
    static_cast<SphereDragger*>(_startDragger)->setColor(osg::Vec4(0,0,1,0));
    addChild(_startDragger);

    _endDragger = new SphereDragger( _los->getMapNode());
    static_cast<SphereDragger*>(_endDragger)->setColor(osg::Vec4(0,0,1,0));
    _endDragger->addPositionChangedCallback(new LOSDraggerCallback(_los, false ) );

    addChild(_endDragger);

    _callback = new LOSUpdateDraggersCallback( this );
    _los->addChangedCallback( _callback.get() );

    updateDraggers();
}

LinearLineOfSightEditor::~LinearLineOfSightEditor()
{
    _los->removeChangedCallback( _callback.get() );
}

void
LinearLineOfSightEditor::updateDraggers()
{
    if ( _los->getMapNode() )
    {
        osg::Vec3d start = _los->getStartWorld();           
        GeoPoint startMap;
        startMap.fromWorld(_los->getMapNode()->getMapSRS(), start);
        _startDragger->setPosition( startMap, false );

        osg::Vec3d end = _los->getEndWorld();           
        GeoPoint endMap;
        endMap.fromWorld(_los->getMapNode()->getMapSRS(), end);    
        _endDragger->setPosition( endMap, false );
    }
}
