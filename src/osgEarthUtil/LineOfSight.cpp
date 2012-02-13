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
#include <osgEarthUtil/LineOfSight>
#include <osgSim/LineOfSight>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Annotation;

class LineOfSightNodeTerrainChangedCallback : public osgEarth::TerrainCallback
{
public:
    LineOfSightNodeTerrainChangedCallback( LineOfSightNode* los ):
      _los(los)
    {
    }

    virtual void onTileAdded(const osgEarth::TileKey& tileKey, osg::Node* terrain, TerrainCallbackContext&)
    {
        _los->terrainChanged( tileKey, terrain );
    }

private:
    LineOfSightNode* _los;
}; 

static bool getRelativeWorld(double x, double y, double relativeHeight, MapNode* mapNode, osg::Vec3d& world )
{
    GeoPoint mapPoint(mapNode->getMapSRS(), x, y);
    osg::Vec3d pos;
    mapNode->getMap()->toWorldPoint(mapPoint, pos);

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
    mapNode->accept( iv );

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


LineOfSightNode::LineOfSightNode(osgEarth::MapNode *mapNode):
_mapNode(mapNode),
_start(0,0,0),
_end(0,0,0),
_hit(0,0,0),
_hasLOS( true ),
_goodColor(0.0f, 1.0f, 0.0f, 1.0f),
_badColor(1.0f, 0.0f, 0.0f, 1.0f),
_displayMode( MODE_SPLIT ),
_altitudeMode( AltitudeMode::ABSOLUTE )
{
    compute(_mapNode.get());
    subscribeToTerrain();
    setNumChildrenRequiringUpdateTraversal( 1 );
}


LineOfSightNode::LineOfSightNode(osgEarth::MapNode *mapNode, const osg::Vec3d& start, const osg::Vec3d& end):
_mapNode(mapNode),
_start(start),
_end(end),
_hit(0,0,0),
_hasLOS( true ),
_goodColor(0.0f, 1.0f, 0.0f, 1.0f),
_badColor(1.0f, 0.0f, 0.0f, 1.0f),
_displayMode( MODE_SPLIT ),
_altitudeMode( AltitudeMode::ABSOLUTE )
{
    compute(_mapNode.get());    
    subscribeToTerrain();
    setNumChildrenRequiringUpdateTraversal( 1 );
}


void
LineOfSightNode::subscribeToTerrain()
{
    _terrainChangedCallback = new LineOfSightNodeTerrainChangedCallback( this );
    _mapNode->getTerrain()->addTerrainCallback( _terrainChangedCallback.get() );        
}

LineOfSightNode::~LineOfSightNode()
{
    //Unsubscribe to the terrain callback
    _mapNode->getTerrain()->removeTerrainCallback( _terrainChangedCallback.get() );
}

void
LineOfSightNode::terrainChanged( const osgEarth::TileKey& tileKey, osg::Node* terrain )
{
    OE_DEBUG << "LineOfSightNode::terrainChanged" << std::endl;
    //Make a temporary group that contains both the old MapNode as well as the new incoming terrain.
    //Because this function is called from the database pager thread we need to include both b/c 
    //the new terrain isn't yet merged with the new terrain.
    osg::ref_ptr < osg::Group > group = new osg::Group;
    group->addChild( terrain );
    group->addChild( _mapNode.get() );
    compute( group, true );
}

const osg::Vec3d&
LineOfSightNode::getStart() const
{
    return _start;
}

void
LineOfSightNode::setStart(const osg::Vec3d& start)
{
    if (_start != start)
    {
        _start = start;
        compute(_mapNode.get());
    }
}

const osg::Vec3d&
LineOfSightNode::getEnd() const
{
    return _end;
}

void
LineOfSightNode::setEnd(const osg::Vec3d& end)
{
    if (_end != end)
    {
        _end = end;
        compute(_mapNode.get());
    }
}

const osg::Vec3d&
LineOfSightNode::getStartWorld() const
{
    return _startWorld;
}

const osg::Vec3d&
LineOfSightNode::getEndWorld() const
{
    return _endWorld;
}

const osg::Vec3d&
LineOfSightNode::getHitWorld() const
{
    return _hitWorld;
}

const osg::Vec3d&
LineOfSightNode::getHit() const
{
    return _hit;
}

bool
LineOfSightNode::getHasLOS() const
{
    return _hasLOS;
}

AltitudeModeEnum
LineOfSightNode::getAltitudeMode() const
{
    return _altitudeMode;
}

void
LineOfSightNode::setAltitudeMode( AltitudeModeEnum mode )
{
    if (_altitudeMode != mode)
    {
        _altitudeMode = mode;
        compute(_mapNode.get());
    }
}

void
LineOfSightNode::addChangedCallback( ChangedCallback* callback )
{
    _changedCallbacks.push_back( callback );
}

void
LineOfSightNode::removeChangedCallback( ChangedCallback* callback )
{
    ChangedCallbackList::iterator i = std::find( _changedCallbacks.begin(), _changedCallbacks.end(), callback);
    if (i != _changedCallbacks.end())
    {
        _changedCallbacks.erase( i );
    }    
}


bool
LineOfSightNode::computeLOS( osgEarth::MapNode* mapNode, const osg::Vec3d& start, const osg::Vec3d& end, AltitudeModeEnum altitudeMode, osg::Vec3d& hit )
{
    const SpatialReference* mapSRS = mapNode->getMapSRS();

    osg::Vec3d startWorld, endWorld;
    if (altitudeMode == AltitudeMode::ABSOLUTE)
    {
        mapNode->getMap()->toWorldPoint( GeoPoint(mapSRS, start), startWorld );
        mapNode->getMap()->toWorldPoint( GeoPoint(mapSRS, end), endWorld );
    }
    else
    {
        getRelativeWorld(start.x(), start.y(), start.z(), mapNode, startWorld);
        getRelativeWorld(end.x(), end.y(), end.z(), mapNode, endWorld);
    }
    
    osgSim::LineOfSight los;
    los.setDatabaseCacheReadCallback(0);
    unsigned int index = los.addLOS(startWorld, endWorld);
    los.computeIntersections(mapNode);
    osgSim::LineOfSight::Intersections hits = los.getIntersections(0);    
    if (hits.size() > 0)
    {
        osg::Vec3d hitWorld = *hits.begin();
        GeoPoint mapHit;
        mapNode->getMap()->worldPointToMapPoint(hitWorld, mapHit);
        hit = mapHit.vec3d();
        return false;
    }
    return true;
}



void
LineOfSightNode::compute(osg::Node* node, bool backgroundThread)
{
    const SpatialReference* mapSRS = _mapNode->getMapSRS();

    //Computes the LOS and redraws the scene
    if (_altitudeMode == AltitudeMode::ABSOLUTE)
    {
        _mapNode->getMap()->toWorldPoint( GeoPoint(mapSRS,_start), _startWorld );
        _mapNode->getMap()->toWorldPoint( GeoPoint(mapSRS,_end), _endWorld );
    }
    else
    {
        getRelativeWorld(_start.x(), _start.y(), _start.z(), _mapNode.get(), _startWorld);
        getRelativeWorld(_end.x(), _end.y(), _end.z(), _mapNode.get(), _endWorld);
    }
    
    osgSim::LineOfSight los;
    los.setDatabaseCacheReadCallback(0);
    unsigned int index = los.addLOS(_startWorld, _endWorld);
    los.computeIntersections(node);
    osgSim::LineOfSight::Intersections hits = los.getIntersections(0);    
    if (hits.size() > 0)
    {
        _hasLOS = false;
        _hitWorld = *hits.begin();
        GeoPoint mapHit;
        _mapNode->getMap()->worldPointToMapPoint( _hitWorld, mapHit);
        _hit = mapHit.vec3d();
    }
    else
    {
        _hasLOS = true;
    }
    draw(backgroundThread);

    for( ChangedCallbackList::iterator i = _changedCallbacks.begin(); i != _changedCallbacks.end(); i++ )
    {
        i->get()->onChanged();
    }	
}

void
LineOfSightNode::draw(bool backgroundThread)
{    
    osg::Geometry* geometry = new osg::Geometry;
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
        if (_displayMode == MODE_SINGLE)
        {
            verts->push_back( _startWorld - _startWorld );
            verts->push_back( _endWorld - _startWorld );
            colors->push_back( _badColor );
            colors->push_back( _badColor );
        }
        else if (_displayMode == MODE_SPLIT)
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

    osg::MatrixTransform* mt = new osg::MatrixTransform;
    mt->setMatrix(osg::Matrixd::translate(_startWorld));
    mt->addChild(geode);  

    getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);


    if (!backgroundThread)
    {
        //Remove all children from this group
        removeChildren(0, getNumChildren());
        addChild( mt );
    }
    else
    {
        _pendingNode = mt;
    }
}

void
LineOfSightNode::setGoodColor( const osg::Vec4f &color )
{
    if (_goodColor != color)
    {
        _goodColor = color;
        draw();
    }
}

const osg::Vec4f&
LineOfSightNode::getGoodColor() const
{
    return _goodColor;
}

void
LineOfSightNode::setBadColor( const osg::Vec4f &color )
{
    if (_badColor != color)
    {
        _badColor = color;
        draw();
    }
}

const osg::Vec4f&
LineOfSightNode::getBadColor() const
{
    return _badColor;
}

LOSDisplayMode
LineOfSightNode::getDisplayMode() const
{
    return _displayMode;
}

void
LineOfSightNode::setDisplayMode( LOSDisplayMode displayMode )
{
    if (_displayMode != displayMode)
    {
        _displayMode = displayMode;
        draw();
    }
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

void
LineOfSightNode::traverse(osg::NodeVisitor& nv)
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
        LineOfSightNode* los = static_cast<LineOfSightNode*>(node);

        if (_startNode.valid())
        {
            osg::Vec3d worldStart = getNodeCenter(_startNode);

            //Convert to mappoint since that is what LOS expects
            GeoPoint mapStart;
            los->getMapNode()->getMap()->worldPointToMapPoint( worldStart, mapStart );
            los->setStart( mapStart.vec3d() );
        }

        if (_endNode.valid())
        {
            osg::Vec3d worldEnd = getNodeCenter( _endNode );

            //Convert to mappoint since that is what LOS expects
            GeoPoint mapEnd;
            los->getMapNode()->getMap()->worldPointToMapPoint( worldEnd, mapEnd );
            los->setEnd( mapEnd.vec3d() );
        }
    }
    traverse(node, nv);
}

/**********************************************************************/

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


RadialLineOfSightNode::RadialLineOfSightNode( MapNode* mapNode):
_mapNode( mapNode ),
_numSpokes(20),
_radius(500),
_center(0,0,0),
_goodColor(0.0f, 1.0f, 0.0f, 1.0f),
_badColor(1.0f, 0.0f, 0.0f, 1.0f),
_outlineColor( 1.0f, 1.0f, 1.0f, 1.0f),
_displayMode( MODE_SPLIT ),
_altitudeMode( AltitudeMode::ABSOLUTE ),
_fill(false)
{
    compute(_mapNode.get());
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
        compute(_mapNode.get() );
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
        compute(_mapNode.get());
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
        compute(_mapNode.get());
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
        compute(_mapNode.get());
    }
}

AltitudeModeEnum
RadialLineOfSightNode::getAltitudeMode() const
{
    return _altitudeMode;
}

void
RadialLineOfSightNode::setAltitudeMode( AltitudeModeEnum mode )
{
    if (_altitudeMode != mode)
    {
        _altitudeMode = mode;
        compute(_mapNode.get());
    }
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
    group->addChild( _mapNode.get() );
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
    //Get the center point in geocentric    
    if (_altitudeMode == AltitudeMode::ABSOLUTE)
    {
        GeoPoint center(_mapNode->getMapSRS(),_center);
        _mapNode->getMap()->toWorldPoint( center, _centerWorld );
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

    for (unsigned int i = 0; i < _numSpokes; i++)
    {
        double angle = delta * (double)i;
        osg::Quat quat(angle, up );
        osg::Vec3d spoke = quat * (side * _radius);
        osg::Vec3d end = _centerWorld + spoke;        
        los.addLOS( _centerWorld, end);      
    }

    los.computeIntersections(node);

    for (unsigned int i = 0; i < _numSpokes; i++)
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
            if (_displayMode == MODE_SPLIT)
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
            else if (_displayMode == MODE_SINGLE)
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

    for( ChangedCallbackList::iterator i = _changedCallbacks.begin(); i != _changedCallbacks.end(); i++ )
    {
        i->get()->onChanged();
    }	
}

void
RadialLineOfSightNode::compute_fill(osg::Node* node, bool backgroundThread)
{    
    //Get the center point in geocentric    
    if (_altitudeMode == AltitudeMode::ABSOLUTE)
    {
        GeoPoint centerMap(_mapNode->getMapSRS(), _center);
        _mapNode->getMap()->toWorldPoint( centerMap, _centerWorld );
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
    osg::Vec3Array* verts = new osg::Vec3Array();
    verts->reserve(_numSpokes * 2);
    geometry->setVertexArray( verts );

    osg::Vec4Array* colors = new osg::Vec4Array();
    colors->reserve( _numSpokes * 2 );

    geometry->setColorArray( colors );
    geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    osgSim::LineOfSight los;
    los.setDatabaseCacheReadCallback(0);

    for (unsigned int i = 0; i < _numSpokes; i++)
    {
        double angle = delta * (double)i;
        osg::Quat quat(angle, up );
        osg::Vec3d spoke = quat * (side * _radius);
        osg::Vec3d end = _centerWorld + spoke;        
        los.addLOS( _centerWorld, end);      
    }

    los.computeIntersections(node);

    for (unsigned int i = 0; i < _numSpokes; i++)
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

    for( ChangedCallbackList::iterator i = _changedCallbacks.begin(); i != _changedCallbacks.end(); i++ )
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
        compute(_mapNode.get());
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
        compute(_mapNode.get());
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
        compute(_mapNode.get());
    }
}

const osg::Vec4f&
RadialLineOfSightNode::getOutlineColor() const
{
    return _outlineColor;
}

LOSDisplayMode
RadialLineOfSightNode::getDisplayMode() const
{
    return _displayMode;
}

void
RadialLineOfSightNode::setDisplayMode( LOSDisplayMode displayMode )
{
    if (_displayMode != displayMode)
    {
        _displayMode = displayMode;
        compute(_mapNode.get());
    }
}

void
RadialLineOfSightNode::addChangedCallback( ChangedCallback* callback )
{
    _changedCallbacks.push_back( callback );
}

void
RadialLineOfSightNode::removeChangedCallback( ChangedCallback* callback )
{
    ChangedCallbackList::iterator i = std::find( _changedCallbacks.begin(), _changedCallbacks.end(), callback);
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
        los->getMapNode()->getMap()->worldPointToMapPoint( worldCenter, mapCenter );

        los->setCenter( mapCenter.vec3d() );      
    }
    traverse(node, nv);
}

/**********************************************************************/
class LOSDraggerCallback : public osgManipulator::DraggerCallback
{
public:
    LOSDraggerCallback(LineOfSightNode* los, bool start):
      _los(los),
      _start(start)
      {
          _ellipsoid = _los->getMapNode()->getMap()->getProfile()->getSRS()->getEllipsoid();
      }

      osg::Vec3d getLocation(const osg::Matrixd& matrix)
      {
          osg::Vec3d trans = matrix.getTrans();
          double lat, lon, height;
          _ellipsoid->convertXYZToLatLongHeight(trans.x(), trans.y(), trans.z(), lat, lon, height);
          return osg::Vec3d(osg::RadiansToDegrees(lon), osg::RadiansToDegrees(lat), height);
      }


      virtual bool receive(const osgManipulator::MotionCommand& command)
      {
          switch (command.getStage())
          {
          case osgManipulator::MotionCommand::START:
              {
                  // Save the current matrix
                  osg::Vec3d location = _start ? _los->getStart() : _los->getEnd();                  
                  double x, y, z;
                  _ellipsoid->convertLatLongHeightToXYZ(osg::DegreesToRadians(location.y()), osg::DegreesToRadians(location.x()), location.z(), x, y, z);
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
                  osg::Vec3d location = getLocation( newMatrix );
                  if (_los->getAltitudeMode() == AltitudeMode::RELATIVE_TO_TERRAIN)
                  {
                      double z = _start ? _los->getStart().z() : _los->getEnd().z();
                      location = osg::Vec3d(location.x(), location.y(), z);
                  }
                  if (_start)
                  {
                      _los->setStart( location );
                  }
                  else
                  {
                      _los->setEnd( location );
                  }

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


      osg::ref_ptr<const osg::EllipsoidModel>            _ellipsoid;
      LineOfSightNode* _los;
      bool _start;

      osg::Matrix _startMotionMatrix;

      osg::Matrix _localToWorld;
      osg::Matrix _worldToLocal;
};

/**********************************************************************/

struct LOSUpdateDraggersCallback : public ChangedCallback
{
public:
    LOSUpdateDraggersCallback( LineOfSightEditor * editor ):
      _editor( editor )
    {

    }
    virtual void onChanged()
    {
        _editor->updateDraggers();
    }

    LineOfSightEditor *_editor;
};

LineOfSightEditor::LineOfSightEditor(LineOfSightNode* los):
_los(los)
{

    _startDragger  = new IntersectingDragger;
    _startDragger->setNode( _los->getMapNode() );    
    _startDragger->setHandleEvents( true );
    _startDragger->addDraggerCallback(new LOSDraggerCallback(_los, true ) );    
    _startDragger->setColor(osg::Vec4(0,0,1,0));
    _startDragger->setupDefaultGeometry();    
    addChild(_startDragger);

    _endDragger = new IntersectingDragger;
    _endDragger->setNode( _los->getMapNode() );    
    _endDragger->setHandleEvents( true );
    _endDragger->setColor(osg::Vec4(0,0,1,0));
    _endDragger->setupDefaultGeometry();
    _endDragger->addDraggerCallback(new LOSDraggerCallback(_los, false ) );

    addChild(_endDragger);

    _callback = new LOSUpdateDraggersCallback( this );
    _los->addChangedCallback( _callback.get() );

    updateDraggers();
}

LineOfSightEditor::~LineOfSightEditor()
{
    _los->removeChangedCallback( _callback.get() );
}

void
LineOfSightEditor::updateDraggers()
{
    const osg::EllipsoidModel* em = _los->getMapNode()->getMap()->getProfile()->getSRS()->getEllipsoid();

    osg::Matrixd startMatrix;
    osg::Vec3d start = _los->getStartWorld();        
    em->computeLocalToWorldTransformFromXYZ(start.x(), start.y(), start.z(), startMatrix);
    _startDragger->setMatrix(startMatrix);        

    osg::Matrixd endMatrix;
    osg::Vec3d end = _los->getEndWorld();        
    em->computeLocalToWorldTransformFromXYZ(end.x(), end.y(), end.z(), endMatrix);
    _endDragger->setMatrix(endMatrix);       
}



/*****************************************************************************/

struct RadialUpdateDraggersCallback : public ChangedCallback
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


//TODO:  Need to consolidate this and the regular LOS callback.  
class RadialLOSDraggerCallback : public osgManipulator::DraggerCallback
{
public:
    RadialLOSDraggerCallback(RadialLineOfSightNode* los):
      _los(los)
      {
          _ellipsoid = _los->getMapNode()->getMap()->getProfile()->getSRS()->getEllipsoid();
      }

      osg::Vec3d getLocation(const osg::Matrixd& matrix)
      {
          osg::Vec3d trans = matrix.getTrans();
          double lat, lon, height;
          _ellipsoid->convertXYZToLatLongHeight(trans.x(), trans.y(), trans.z(), lat, lon, height);
          return osg::Vec3d(osg::RadiansToDegrees(lon), osg::RadiansToDegrees(lat), height);
      }


      virtual bool receive(const osgManipulator::MotionCommand& command)
      {
          switch (command.getStage())
          {
          case osgManipulator::MotionCommand::START:
              {
                  // Save the current matrix
                  osg::Vec3d location = _los->getCenter();
                  double x, y, z;
                  _ellipsoid->convertLatLongHeightToXYZ(osg::DegreesToRadians(location.y()), osg::DegreesToRadians(location.x()), location.z(), x, y, z);
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

                  osg::Vec3d location = getLocation( newMatrix );
                  if (_los->getAltitudeMode() == AltitudeMode::RELATIVE_TO_TERRAIN)
                  {
                      double z = _los->getCenter().z();
                      location = osg::Vec3d(location.x(), location.y(), z);
                  }                  
                  _los->setCenter( location );
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


      osg::ref_ptr<const osg::EllipsoidModel>            _ellipsoid;
      RadialLineOfSightNode* _los;
      bool _start;

      osg::Matrix _startMotionMatrix;

      osg::Matrix _localToWorld;
      osg::Matrix _worldToLocal;
};

/**********************************************************************/

RadialLineOfSightEditor::RadialLineOfSightEditor(RadialLineOfSightNode* los):
_los(los)
{

    _dragger  = new IntersectingDragger;
    _dragger->setNode( _los->getMapNode() );    
    _dragger->setHandleEvents( true );
    _dragger->addDraggerCallback(new RadialLOSDraggerCallback(_los ) );    
    _dragger->setColor(osg::Vec4(0,0,1,0));
    _dragger->setupDefaultGeometry();    
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
    const osg::EllipsoidModel* em = _los->getMapNode()->getMap()->getProfile()->getSRS()->getEllipsoid();

    osg::Matrixd matrix;
    osg::Vec3d center = _los->getCenterWorld();             
    em->computeLocalToWorldTransformFromXYZ(center.x(), center.y(), center.z(), matrix);
    _dragger->setMatrix(matrix);        
}