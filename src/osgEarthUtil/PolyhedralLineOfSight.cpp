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
#include <osgEarthUtil/PolyhedralLineOfSight>


#include <osgUtil/LineSegmentIntersector>
#include <osgUtil/IntersectionVisitor>
#include <osg/CullFace>

#define LC "[PolyhedralLineOfSight] "

using namespace osgEarth;
using namespace osgEarth::Util;

//------------------------------------------------------------------------

namespace
{
    struct TerrainChangedCallback : public osgEarth::TerrainCallback
    {
        TerrainChangedCallback( PolyhedralLineOfSightNode* los ) : _los(los) { }
        void onTileAdded(const osgEarth::TileKey& tileKey, osg::Node* terrain, TerrainCallbackContext& ) {
            _los->terrainChanged( tileKey, terrain );
        }
        PolyhedralLineOfSightNode* _los;
    };
}

//------------------------------------------------------------------------

PolyhedralLineOfSightNode::PolyhedralLineOfSightNode( MapNode* mapNode ) :
GeoPositionNode( mapNode ),
_startAzim   ( Angle(-45.0, Units::DEGREES) ),
_endAzim     ( Angle( 45.0, Units::DEGREES) ),
_startElev   ( Angle(  0.0, Units::DEGREES) ),
_endElev     ( Angle( 45.0, Units::DEGREES) ),
_spacing     ( Angle(  5.0, Units::DEGREES) ),
_distance    ( Distance(50000.0, Units::METERS) )
{
    OE_WARN << LC << "This class is under development; use at your own risk" << std::endl;

    _xform = new osg::MatrixTransform();
    this->addChild( _xform.get() );

    _geode = new osg::Geode();
    rebuildGeometry();
    recalculateExtent();

    _xform->addChild( _geode );

    _terrainCallback = new TerrainChangedCallback(this);
    
    if ( mapNode )
        mapNode->getTerrain()->addTerrainCallback( _terrainCallback );

    osg::StateSet* stateSet = this->getOrCreateStateSet();
    stateSet->setMode( GL_BLEND, 1 );
    stateSet->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );
    stateSet->setAttributeAndModes( new osg::CullFace(osg::CullFace::BACK), 1 );
}


PolyhedralLineOfSightNode::~PolyhedralLineOfSightNode()
{
    if (_terrainCallback && getMapNode() )
    {
        getMapNode()->getTerrain()->removeTerrainCallback( _terrainCallback.get() );
    }
}

void
PolyhedralLineOfSightNode::setMapNode( MapNode* mapNode )
{
    osg::ref_ptr<MapNode> oldMapNode = getMapNode();
    if ( oldMapNode.valid() )
    {
        if ( _terrainCallback.valid() )
        {
            oldMapNode->getTerrain()->removeTerrainCallback( _terrainCallback.get() );
        }
        if ( mapNode )
        {
            mapNode->getTerrain()->addTerrainCallback( _terrainCallback.get() );
        }
    }

    GeoPositionNode::setMapNode( mapNode );
}

void
PolyhedralLineOfSightNode::setDistance( const Distance& value )
{
    _distance = value;
    recalculateExtent();
    updateSamples();
}


void
PolyhedralLineOfSightNode::setAzimuthalRange(const Angle& start,
                                             const Angle& end)
{
    _startAzim = start;
    _endAzim   = end;
    rebuildGeometry();
    updateSamples();
}


void
PolyhedralLineOfSightNode::setElevationRange(const Angle& start,
                                             const Angle& end)
{
    _startElev = start;
    _endElev   = end;
    rebuildGeometry();
    updateSamples();
}


void
PolyhedralLineOfSightNode::setSampleSpacing(const Angle& value)
{
    _spacing = value;
    rebuildGeometry();
    updateSamples();
}


void 
PolyhedralLineOfSightNode::terrainChanged(const osgEarth::TileKey& tileKey,
                                          osg::Node*               patch)
{
    if ( tileKey.getExtent().intersects(_extent) )
    {
        updateSamples();
    }
}


void 
PolyhedralLineOfSightNode::traverse(osg::NodeVisitor& nv)
{
    osg::Group::traverse( nv );
}


void
PolyhedralLineOfSightNode::setPosition(const GeoPoint& pos)
{
    GeoPositionNode::setPosition( pos );
    recalculateExtent();
    updateSamples();
}


void
PolyhedralLineOfSightNode::dirty()
{
    //todo
}


void
PolyhedralLineOfSightNode::recalculateExtent()
{
    // get a local2world matrix for the map position:
    GeoPoint absMapPos = getPosition();
    absMapPos.makeAbsolute( getMapNode()->getTerrain() );
    osg::Matrix local2world;
    absMapPos.createLocalToWorld( local2world );

    // local offsets (east and north)
    osg::Vec3d x( _distance.as(Units::METERS), 0.0, 0.0 );
    osg::Vec3d y( 0.0, _distance.as(Units::METERS), 0.0 );

    // convert these to map coords:
    GeoPoint easting, northing;
    easting.fromWorld( getMapNode()->getMapSRS(), x * local2world );
    northing.fromWorld( getMapNode()->getMapSRS(), y * local2world );

    // calculate the offsets:
    double d_easting = easting.x() - absMapPos.x();
    double d_northing = northing.y() - absMapPos.y();

    // update the extent.
    _extent = GeoExtent(
        getMapNode()->getMapSRS(),
        absMapPos.x() - d_easting, absMapPos.y() - d_northing,
        absMapPos.x() + d_easting, absMapPos.y() + d_northing );

    OE_INFO << LC << "Cached extent = " << _extent.toString() << std::endl;
}


// Builds the initial basic geometry set. This needs to happen (a) on startup, and 
// (b) if something changes that would alter the vertex count or unit vector
void
PolyhedralLineOfSightNode::rebuildGeometry()
{
    // clear out the node first.
    _geode->removeDrawables( 0, _geode->getNumDrawables() );

    osg::Geometry* geom = new osg::Geometry();
    geom->setUseVertexBufferObjects( true );

    osg::Vec3Array* verts = new osg::Vec3Array();
    geom->setVertexArray( verts );
    verts->getVertexBufferObject()->setUsage(GL_DYNAMIC_DRAW_ARB);

    osg::Vec4Array* colors = new osg::Vec4Array();
    geom->setColorArray( colors );
    geom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );

    osg::Vec3Array* normals = new osg::Vec3Array();
    geom->setNormalArray( normals );
    geom->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );

    double azim0   = _startAzim.as(Units::RADIANS);
    double azim1   = _endAzim.as(Units::RADIANS);
    double elev0   = _startElev.as(Units::RADIANS);
    double elev1   = _endElev.as(Units::RADIANS);
    double spacing = _spacing.as(Units::RADIANS);
    if ( spacing <= 0.0 )
        spacing = 0.1;

    // the origin point
    verts->push_back  ( osg::Vec3(0.0f, 0.0f, 0.0f) );
    normals->push_back( osg::Vec3(0.0f, 0.0f, 1.0f) );
    colors->push_back ( osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f) );

    // Build the unit-vector vertex list. Later this will be updated
    // in updateGeometry.
    _numRows = 0;
    bool lastElev = false;
    for( double elev = elev0; !lastElev; elev += spacing, _numRows++ )
    {
        if ( elev >= elev1 ) 
        {
            elev = elev1;
            lastElev = true;
        }

        double cos_elev = cos(elev), sin_elev = sin(elev);

        _numCols = 0;
        bool lastAzim = false;
        for( double azim = azim0; !lastAzim; azim += spacing, _numCols++ )
        {
            if ( azim >= azim1 )
            {
                azim = azim1;
                lastAzim = true;
            }

            double cos_azim = cos(azim), sin_azim = sin(azim);

            double x = cos_elev * sin_azim;
            double y = cos_elev * cos_azim;
            double z = sin_elev;

            verts->push_back  ( osg::Vec3(x, y, z) );
            normals->push_back( osg::Vec3(x, y, z) );
            colors->push_back ( osg::Vec4(0,1,0,0.5f) );
        }
    }

    // Build the primitive sets to render the polyhedron.
    // first the walls.
    osg::DrawElements* de = 
        verts->size() > 0xFFFF ? (osg::DrawElements*)new osg::DrawElementsUShort  ( GL_TRIANGLES ) :
        verts->size() > 0xFF   ? (osg::DrawElements*)new osg::DrawElementsUShort( GL_TRIANGLES ) :
                                 (osg::DrawElements*)new osg::DrawElementsUByte ( GL_TRIANGLES );

    unsigned rowOffset = 1; // to account for the origin point

    OE_NOTICE << LC << "numRows = " << _numRows << ", numCols = " << _numCols << std::endl;

    // outer surface
    for( unsigned r=0; r<_numRows-1; ++r )
    {
        for( unsigned c=0; c<_numCols-1; ++c )
        {
            unsigned i  = rowOffset + c;
            unsigned i2 = rowOffset + c + _numCols;

            de->addElement( i );
            de->addElement( i+1 );
            de->addElement( i2 );
            de->addElement( i2 );
            de->addElement( i+1 );
            de->addElement( i2+1 );
        }
        rowOffset += _numCols;
    }

#if 1
    // top cap
    for( unsigned c=0; c<_numCols-1; ++c )
    {
        de->addElement( 0 );
        de->addElement( rowOffset + c + 1 );
        de->addElement( rowOffset + c );
    }
#endif

    geom->addPrimitiveSet( de );

    // finally, make the geode and attach it.
    _geode->addDrawable( geom );
}


void
PolyhedralLineOfSightNode::updateSamples()
{
    if ( _geode->getNumDrawables() == 0 )
        rebuildGeometry();

    osg::Geometry* geom  = _geode->getDrawable(0)->asGeometry();
    osg::Vec3Array* verts = dynamic_cast<osg::Vec3Array*>( geom->getVertexArray() );
    if (!verts)
        return;

    double distance = _distance.as(Units::METERS);

    // get the world coords and a l2w transform for the origin:
    osg::Vec3d  originWorld;
    osg::Matrix local2world, world2local;

    Terrain* t = getMapNode()->getTerrain();
    GeoPoint origin = getPosition();
    origin.makeAbsolute( t );
    origin.toWorld( originWorld, t );
    origin.createLocalToWorld( local2world );
    world2local.invert( local2world );

    // set up an intersector:
    osgUtil::LineSegmentIntersector* lsi = new osgUtil::LineSegmentIntersector( originWorld, originWorld );
    osgUtil::IntersectionVisitor iv( lsi );

    // intersect the verts (skip the origin point) with the map node.
    for( osg::Vec3Array::iterator v = verts->begin()+1; v != verts->end(); ++v )
    {
        osg::Vec3d unit = *v;
        unit.normalize();
        unit *= distance;

        osg::Vec3d world = unit * local2world;

        if ( osg::equivalent(unit.length(), 0.0) )
        {
            OE_WARN << "problem." << std::endl;
        }

        lsi->reset();
        lsi->setStart( originWorld );
        lsi->setEnd( world );

        OE_DEBUG << LC << "Ray: " <<
            originWorld.x() << "," << originWorld.y() << "," << originWorld.z() << " => "
            << world.x() << "," << world.y() << "," << world.z()
            << std::endl;


        getMapNode()->getTerrain()->accept( iv );

        osgUtil::LineSegmentIntersector::Intersections& hits = lsi->getIntersections();
        if ( !hits.empty() )
        {
            const osgUtil::LineSegmentIntersector::Intersection& hit = *hits.begin();
            osg::Vec3d newV = hit.getWorldIntersectPoint() * world2local;
            if ( newV.length() > 1.0 )
                *v = newV;
            else
                *v = unit;
        }
        else
        {
            *v = unit;
        }

        //OE_NOTICE << "Radial point = " << v->x() << ", " << v->y() << ", " << v->z() << std::endl;
    }

    verts->dirty();
    geom->dirtyBound();
}
