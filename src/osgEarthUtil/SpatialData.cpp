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
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include <osgEarthUtil/SpatialData>
#include <osgEarth/Registry>
#include <osgEarth/CullingUtils>
#include <osg/PolygonOffset>
#include <osg/Polytope>
#include <osg/Geometry>
#include <osg/Depth>
#include <osgText/Text>
#include <sstream>

#define LC "[GeoGraph] "

using namespace osgEarth;
using namespace osgEarth::Util;

//------------------------------------------------------------------------

namespace
{
    unsigned getIndex( const GeoExtent& cellExtent, const osg::Vec3d& point, unsigned xdim, unsigned ydim )
    {
        double cw = cellExtent.width() / (double)xdim;
        double ch = cellExtent.height() / (double)ydim;

        unsigned col = osg::clampBelow( (unsigned)((point.x()-cellExtent.xMin())/cw), xdim-1 );
        unsigned row = osg::clampBelow( (unsigned)((point.y()-cellExtent.yMin())/ch), ydim-1 );

        return row*xdim + col;
    }
    

    osg::Geode* makeClusterGeode( const GeoExtent& cellExtent, unsigned num )
    {
        osgText::Text* t = new osgText::Text();

        double clat, clon;
        cellExtent.getCentroid( clon, clat );
        osg::Vec3d xyz;        
        cellExtent.getSRS()->getEllipsoid()->convertLatLongHeightToXYZ(
            osg::DegreesToRadians( clat ), osg::DegreesToRadians( clon ), 0, xyz.x(), xyz.y(), xyz.z() );
        t->setPosition( xyz );
        
        std::stringstream buf;
        buf << num;
        std::string str;
        str = buf.str();
        t->setText( str );
        t->setCharacterSizeMode( osgText::TextBase::SCREEN_COORDS );
        t->setCharacterSize( 22.0f );
        t->setAutoRotateToScreen( true );
        
        t->setFont( osgEarth::Registry::instance()->getDefaultFont() );

        t->setBackdropType( osgText::Text::OUTLINE );
        t->setColor( osg::Vec4(1,1,1,1) );
        t->setBackdropColor( osg::Vec4(0,0,0,1) );

        osg::Geode* geode = new osg::Geode();
        geode->addDrawable( t );

        osg::StateSet* s = geode->getOrCreateStateSet();
        s->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS) );

        t->setDataVariance( osg::Object::DYNAMIC );

        return geode;
    }

    GeoObjectCollection::iterator
    findObject( GeoObjectCollection& objects, GeoObject* object )
    {
        float key = object->getPriority();
        GeoObjectCollection::iterator first = objects.find(key);
        if ( first == objects.end() )
            return objects.end();

        GeoObjectCollection::iterator last = objects.upper_bound(key);
        for( ; first != last; ++first )
            if ( first->second.get() == object )
                return first;

        return objects.end();
    }
}

//------------------------------------------------------------------------

GeoObject::GeoObject() : _priority(0.0f)
{
    //NOP
}

//------------------------------------------------------------------------

GeoGraph::GeoGraph(const GeoExtent& extent, float maxRange, unsigned maxObjects,
                   unsigned splitDim, float splitRangeFactor,
                   unsigned rootWidth, unsigned rootHeight ) :
GeoCell( extent, maxRange, maxObjects, splitDim, splitRangeFactor, 0 )
{
    _rootWidth = osg::maximum( rootWidth, (unsigned)2 );
    _rootHeight = osg::maximum( rootHeight, (unsigned)2 );

    if ( _depth == 0 )
    {
        double xinterval = extent.width() / (double)_rootWidth;
        double yinterval = extent.height() / (double)_rootHeight;

        for( unsigned y=0; y<_rootHeight; ++y )
        {
            for( unsigned x=0; x<_rootWidth; ++x )
            {
                GeoExtent cellExtent(
                    _extent.getSRS(),
                    _extent.xMin() + xinterval*(double)x,
                    _extent.yMin() + yinterval*(double)y,
                    _extent.xMin() + xinterval*(double)(x+1),
                    _extent.yMin() + yinterval*(double)(y+1) );

                GeoCell * child = new GeoCell(
                    cellExtent,
                    _maxRange,
                    _maxObjects,
                    _splitDim,
                    _splitRangeFactor,
                    1 );

                this->addChild( child, 0, maxRange ); //FLT_MAX );
            }
        }                    
    }
}

bool
GeoGraph::insertObject( GeoObject* object )
{
    osg::Vec3d loc;
    if ( object->getLocation(loc) )
    {
        unsigned index = getIndex( _extent, loc, _rootWidth, _rootHeight );
        return static_cast<GeoCell*>(getChild(index))->insertObject( object );
    }
    else
    {
        return false;
    }
}

//------------------------------------------------------------------------

GeoCell::GeoCell(const GeoExtent& extent, float maxRange, unsigned maxObjects,
                 unsigned splitDim, float splitRangeFactor, unsigned depth ) :
_extent( extent ),
_maxRange( maxRange ),
_maxObjects( maxObjects ),
_splitDim( splitDim ),
_splitRangeFactor( splitRangeFactor ),
_depth( depth ),
_minObjects( (maxObjects/10)*8 ), // 80%
_count( 0 ),
_boundaryPoints( 10 ),
_frameStamp( 0 ),
_boundaryColor(0L)
{
    generateBoundaries();
    //generateBoundaryGeometry();

    // Disable OSG's culling so we can do our own custom culling.
    this->setCullingActive( false );
}

void
GeoCell::generateBoundaries()
{
    const osg::EllipsoidModel* em = _extent.getSRS()->getEllipsoid();
    static const double hae =  1e6;
    static const double hbe = -1e5;

    // find the geodetic center:
    osg::Vec3d gcenter;
    _extent.getCentroid( gcenter.x(), gcenter.y() );

    // convert to a geocentric vector:
    osg::Vec3d center;
    em->convertLatLongHeightToXYZ(
        osg::DegreesToRadians(gcenter.y()), osg::DegreesToRadians(gcenter.x()), 0.0, center.x(), center.y(), center.z());

    osg::Vec3d centerVec = center;
    centerVec.normalize();

    // find the 4 geodetic corners:
    osg::Vec3d gcorner[4];
    gcorner[0].set( _extent.xMin(), _extent.yMin(), 0.0 );
    gcorner[1].set( _extent.xMin(), _extent.yMax(), 0.0 );
    gcorner[2].set( _extent.xMax(), _extent.yMax(), 0.0 );
    gcorner[3].set( _extent.xMax(), _extent.yMin(), 0.0 );

    // and convert those to geocentric vectors:
    osg::Vec3d corner[4];
    osg::Vec3d cornerVec[4];
    for(unsigned i=0; i<4; ++i )
    {
        em->convertLatLongHeightToXYZ(
            osg::DegreesToRadians(gcorner[i].y()), osg::DegreesToRadians(gcorner[i].x()), 0.0,
            corner[i].x(), corner[i].y(), corner[i].z() );
        cornerVec[i] = corner[i];
        cornerVec[i].normalize();
    }   
    
    // now extrude the center and corners up and down to get the boundary points:
    unsigned p = 0;
    _boundaryPoints[p++] = center + centerVec*hae;
    _boundaryPoints[p++] = center +centerVec*hbe;
    for( unsigned i=0; i<4; ++i )
    {
        _boundaryPoints[p++] = corner[i] + cornerVec[i]*hae;
        _boundaryPoints[p++] = corner[i] + cornerVec[i]*hbe;
    }
}

osg::BoundingSphere
GeoCell::computeBound() const
{
    osg::BoundingSphere bs;
    for( unsigned i=0; i<10; ++i )
        bs.expandBy( _boundaryPoints[i] );
    return bs;
}

void
GeoCell::generateBoundaryGeometry()
{
    osg::Geometry* g = new osg::Geometry();
    g->setUseVertexBufferObjects(true);

    osg::Vec3Array* v = new osg::Vec3Array(10);
    for( unsigned i=0; i<10; ++i )
        (*v)[i] = _boundaryPoints[i];
    g->setVertexArray( v );

    osg::DrawElementsUByte* el = new osg::DrawElementsUByte( GL_QUADS );
    el->push_back( 7 ); el->push_back( 5 ); el->push_back( 4 ); el->push_back( 6 );
    el->push_back( 9 ); el->push_back( 7 ); el->push_back( 6 ); el->push_back( 8 );
    el->push_back( 3 ); el->push_back( 9 ); el->push_back( 8 ); el->push_back( 2 );
    el->push_back( 5 ); el->push_back( 3 ); el->push_back( 2 ); el->push_back( 4 );
    //el->push_back( 2 ); el->push_back( 8 ); el->push_back( 6 ); el->push_back( 4 ); //top
    //el->push_back( 9 ); el->push_back( 3 ); el->push_back( 5 ); el->push_back( 7 ); // bottom
    g->addPrimitiveSet( el );

    osg::Vec4Array* c = new osg::Vec4Array(1);
    (*c)[0].set( 1, 1, 1, 0.25 );
    g->setColorArray( c );
    g->setColorBinding( osg::Geometry::BIND_OVERALL );

    _boundaryColor = c;

    g->setDataVariance( osg::Object::DYNAMIC );
    g->setUseDisplayList( false );
    g->setUseVertexBufferObjects( true );

    osg::StateSet* set = g->getOrCreateStateSet();
    set->setMode( GL_BLEND, 1 );
    set->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );
    set->setAttribute( new osg::PolygonOffset(1,1), 1 );

    _boundaryGeode = new osg::Geode();
    _boundaryGeode->addDrawable( g );
}

bool
GeoCell::intersects( const osg::Polytope& tope ) const
{
    const osg::Polytope::PlaneList& planes = tope.getPlaneList();
    for( osg::Polytope::PlaneList::const_iterator i = planes.begin(); i != planes.end(); ++i )
    {
        if ( i->intersect( _boundaryPoints ) < 0 )
            return false;
    }
    return true;
}

void
GeoCell::traverse( osg::NodeVisitor& nv )
{
    bool isCull = nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR;

    if ( _depth > 0 )
    {
        if ( isCull )
        {
            // process boundary geometry, if present.
            if ( _boundaryGeode.valid() ) 
            {
                if ( _count > 0 )
                    (*_boundaryColor)[0].set(1,0,0,0.35);
                else
                    (*_boundaryColor)[0].set(1,1,1,0.25);
                _boundaryColor->dirty();

                _boundaryGeode->accept( nv );
            }

            // custom BSP culling function. this checks that the set of boundary points
            // for this cell intersects the viewing frustum.
            osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);
            if ( cv )
            {
                if (!intersects( cv->getCurrentCullingSet().getFrustum() ) )
                {
                    return;
                }

                // passed cull, so record the framestamp.
                _frameStamp = cv->getFrameStamp()->getFrameNumber();
            }
        }

        if ( _objects.size() > 0 )
        {
            for( GeoObjectCollection::iterator i = _objects.begin(); i != _objects.end(); ++i )
            {
                osg::Node* node = i->second->getNode();
                if ( node )
                    node->accept( nv );
            }
        }

        if ( _clusterGeode.valid() )
            _clusterGeode->accept( nv );
    }

    else
    {
        if ( isCull )
            _frameStamp = nv.getFrameStamp()->getFrameNumber();
    }

    osg::LOD::traverse( nv );
}

void
GeoCell::adjustCount( int delta )
{
    _count += delta;

    if ( _depth > 0 && getNumParents() > 0 )
    {
        static_cast<GeoCell*>(getParent(0))->adjustCount( delta );
    }
}

bool
GeoCell::insertObject( GeoObject* object )
{
    osg::Vec3d location;
    if ( object->getLocation(location) && _extent.contains(location.x(), location.y()) )
    {
        object->_cell = this;
        _objects.insert( std::make_pair(object->getPriority(), object) );

        if ( _objects.size() > _maxObjects )
        {
            GeoObjectCollection::iterator low = _objects.begin();
            GeoObject* lowPriObject = low->second.get();
            
            if ( getNumChildren() == 0 )
                split();

            lowPriObject->getLocation(location);
            unsigned index = getIndex(_extent, location, _splitDim, _splitDim);
            bool insertedOK = static_cast<GeoCell*>(getChild(index))->insertObject( lowPriObject );
            if ( insertedOK )
            {
                // remove it from this cell.
                _objects.erase( low );
            }
            else
            {
                // should never ever happen..
                OE_WARN << LC << "Object insertion failed" << std::endl;
                return false;
            }
        }
        return true;
    }
    else
    {
        return false;
    }
}

void
GeoCell::split()
{
    // the max LOD range for children of this cell:
    float newRange = _maxRange * _splitRangeFactor;

    // first create all the child cells:
    double xInterval = _extent.width() / (double)_splitDim;
    double yInterval = _extent.height() / (double)_splitDim;

    for( unsigned row=0; row<_splitDim; ++row )
    {
        for( unsigned col=0; col<_splitDim; ++col )
        {
            GeoExtent cellExtent(
                _extent.getSRS(),
                _extent.xMin() + xInterval*(double)col,
                _extent.yMin() + yInterval*(double)row,
                _extent.xMin() + xInterval*(double)(col+1),
                _extent.yMin() + yInterval*(double)(row+1) );

            this->addChild(
                new GeoCell(cellExtent, newRange, _maxObjects, _splitDim, _splitRangeFactor, _depth+1),
                0.0f,
                newRange );
        }
    }
}

bool
GeoCell::removeObject( GeoObject* object )
{
    if ( object->_cell.get() == this )
    {
        object->_cell = 0L;
        _objects.erase( findObject(_objects, object) );
        adjustCount( -1 );

        // if we just fell beneath the threshold, pull one up from below.
        if ( _objects.size() == _minObjects-1 )
        {
            //TODO.
        }

        // TODO: rebalance, merge the tree, etc.
        return true;
    }
    else
    {
        for( unsigned i=0; i<getNumChildren(); ++i )
        {
            if ( static_cast<GeoCell*>(getChild(i))->removeObject( object ) )
                return true;
        }
    }
    return false;
}

void
GeoCell::merge()
{
    //NYI //TODO
}

bool
GeoCell::reindexObject( GeoObject* object )
{
    GeoCell* owner = object->getGeoCell();
    if ( owner )
    {
        osg::Vec3d location;
        if ( object->getLocation(location) && !owner->_extent.contains(location.x(), location.y()) )
        {
            // first remove from its current cell
            owner->removeObject( object );

            GeoCell* cell = dynamic_cast<GeoCell*>( owner->getParent(0) );
            while( cell )
            {
                if ( cell->getExtent().contains(location.x(), location.y()) )
                {
                    if ( cell->insertObject( object ) )
                        return true;
                }
                cell = dynamic_cast<GeoCell*>( cell->getParent(0) );
            }
        }

        // no change
        return true;
    }
    else
    {
        return insertObject( object );
    }
}
