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
#include <osgEarthUtil/SpatialData>
#include <osgUtil/CullVisitor>
#include <osg/PolygonOffset>
#include <osg/Polytope>
#include <osg/Geometry>

#define LC "[GeoGraph] "

using namespace osgEarth;
using namespace osgEarth::Util;

//------------------------------------------------------------------------

namespace
{
    unsigned getIndex( const GeoExtent& cellExtent, const osg::Vec3d& point, unsigned xdim, unsigned ydim )
    {
        unsigned col = (unsigned)((double)xdim * ((point.x() - cellExtent.xMin()) / cellExtent.width()));
        unsigned row = (unsigned)((double)ydim * ((point.y() - cellExtent.yMin()) / cellExtent.height()));
        return row*xdim + col;
    }
}

//------------------------------------------------------------------------

GeoObject::GeoObject()
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

                this->addChild( child, 0, FLT_MAX );
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
_splitDim( splitDim ),
_maxObjects( maxObjects ),
_splitRangeFactor( splitRangeFactor ),
_count( 0 ),
_depth( depth ),
_boundaryPoints( 10 )
{
    generateBoundaries();
    generateBoundaryGeometry();

    // Disable culling so we can do our own custom culling.
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

    osg::Vec3dArray* v = new osg::Vec3dArray(10);
    for( unsigned i=0; i<10; ++i )
        (*v)[i] = _boundaryPoints[i];
    g->setVertexArray( v );

    osg::DrawElementsUByte* el = new osg::DrawElementsUByte( GL_QUADS );
    el->push_back( 7 ); el->push_back( 5 ); el->push_back( 4 ); el->push_back( 6 );
    el->push_back( 9 ); el->push_back( 7 ); el->push_back( 6 ); el->push_back( 8 );
    el->push_back( 3 ); el->push_back( 9 ); el->push_back( 8 ); el->push_back( 2 );
    el->push_back( 5 ); el->push_back( 3 ); el->push_back( 2 ); el->push_back( 4 );
    el->push_back( 2 ); el->push_back( 8 ); el->push_back( 6 ); el->push_back( 4 );
    el->push_back( 9 ); el->push_back( 3 ); el->push_back( 5 ); el->push_back( 7 );
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
    set->setAttribute( new osg::PolygonOffset(-1,1), 1 );

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
    if ( _depth > 0 )
    {
        if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
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

            // custom BSP culling function.
            osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>( &nv );
            if ( cv && !intersects( cv->getCurrentCullingSet().getFrustum() ) )
                return;           
        }

        if ( _objects.size() > 0 )
        {
            for( std::vector<osg::ref_ptr<GeoObject> >::iterator i = _objects.begin(); i != _objects.end(); ++i )
            {
                osg::Node* node = i->get()->getNode();
                if ( node )
                    node->accept( nv );
            }
        }
        else
        {
            osg::LOD::traverse( nv );
        }
    }
    else
    {
        osg::LOD::traverse( nv );
    }
}

void
GeoCell::adjustCount( int delta )
{
    _count += delta;

    if ( _depth > 0 )
        static_cast<GeoCell*>(getParent(0))->adjustCount( delta );            
}

bool
GeoCell::insertObject( GeoObject* object )
{
    osg::Vec3d location;
    if ( object->getLocation(location) && _extent.contains(location.x(), location.y()) )
    {
        // first see if it will fit in this cell as-is:
        if ( _count < _maxObjects )
        {
            //if ( _objects.size() == 0 )
            //    _objects.reserve( _maxObjects );

            object->_cell = this;
            _objects.push_back( object );
            object->getNode()->dirtyBound();
            adjustCount( +1 );

            OE_INFO << LC << "Inserted object " << object->getNode()->getName() << " at " << location << std::endl;

            return true;
        }

        // next see if this cell is already split:
        else if ( getNumChildren() > 0 )
        {
            unsigned index = getIndex( _extent, location, _splitDim, _splitDim );
            return static_cast<GeoCell*>(getChild(index))->insertObject(object);
        }

        // otherwise, split the cell and try again
        else
        {
            split();
            return insertObject( object );
        }
    }

    return false;
}

void
GeoCell::split()
{
    std::vector<GeoCell*> newCells;
    newCells.reserve( _splitDim * _splitDim );

    // the max LOD range for children of this cell:
    float _newRange = _maxRange * _splitRangeFactor;

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
                _extent.yMin() + xInterval*(double)(col+1),
                _extent.yMin() + yInterval*(double)(row+1) );

            newCells.push_back( new GeoCell(cellExtent, _newRange, _maxObjects, _splitDim, _splitRangeFactor, _depth+1) );
        }
    }

    // now copy the objects over:
    for( std::vector<osg::ref_ptr<GeoObject> >::iterator i = _objects.begin(); i != _objects.end(); ++i )
    {
        // select the correct child cell:
        GeoObject* object = i->get();
        osg::Vec3d location;
        if ( object->getLocation(location) )
        {
            unsigned index = getIndex(_extent, location, _splitDim, _splitDim);
            newCells[index]->insertObject( object );
            object->_cell = newCells[index];
        }
        else
        {
            object->_cell = 0L;
            // quietly disappears from the graph ... 
        }
    }

    // now add the new cells:
    for( unsigned i=0; i<newCells.size(); ++i )
    {
        this->addChild( newCells[i], 0.0f, _newRange );
    }

    // remove all the objects from this cell.
    _objects.clear();
}

bool
GeoCell::removeObject( GeoObject* object )
{
    if ( object->_cell.get() == this )
    {
        object->_cell = 0L;
        _objects.erase( std::find( _objects.begin(), _objects.end(), object ) );
        adjustCount( -1 );
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
    osg::ref_ptr<GeoCell> safeCell = object->getGeoCell();
    if ( safeCell.valid() )
    {
        return safeCell->reindex( object );
    }
    else
    {
        return insertObject( object );
    }
}

bool
GeoCell::reindex( GeoObject* object )
{
    osg::Vec3d location;
    if ( object->getLocation(location) && !_extent.contains(location.x(), location.y()) )
    {
        // first remove from its current cell
        osg::ref_ptr<GeoCell> safeCell = object->_cell.get();
        if ( safeCell.valid() )
            safeCell->removeObject( object );

        GeoCell* cell = dynamic_cast<GeoCell*>( this->getParent(0) );
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

    return true;
}
