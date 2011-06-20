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
#include "Diamond"
#include "MeshManager"
#include <osgEarth/Cube>
#include <iterator>

#ifdef USE_DEBUG_TEXTURES

static osg::Image* createDebugImage()
{
    osg::Image* image = new osg::Image();
    image->allocateImage( 4, 4, 1, GL_RGBA, GL_UNSIGNED_BYTE );
    *(unsigned int*)(image->data( 1, 0 )) = 0xff0000ff;
    *(unsigned int*)(image->data( 2, 0 )) = 0xff0000ff;
    return image;
}

#endif // USE_DEBUG_TEXTURES

// --------------------------------------------------------------------------

static int s_numDiamonds = 0;

Diamond::Diamond( MeshManager* mesh, const osgEarth::TileKey& key, Level level, const std::string& name ) :
osg::Referenced(true),
_mesh( mesh ),
_key( key ),
_level( level ),
_name( name ),
_status( ACTIVE ),
_childValence( 4 ),
//_color(1,1,1,1),
_lastCullFrame( 0 ),
_queuedForSplit( false ),
_queuedForMerge( false ),
_queuedForImage( false ),
_drawableDirty( false ),
_bsComputed( false ),
_isSplit( false ),
_hasGeometry( level % 2 == 1 ),
_currentStateSetOwner( 0L ),
_targetStateSetOwner( 0L ),
_targetStateSetRevision( -1 ),
_hasFinalImage( false )
{
    this->setThreadSafeRefUnref(true);

    // only ODD-numbered levels have actual geometry.
    if ( _hasGeometry )
    {
        _stateSet = new RevisionedStateSet();
        _amrDrawable = new AMRDrawable();
    }

    s_numDiamonds++;
    //OE_NOTICE << s_numDiamonds << " ... " << std::endl;

    //if ( _key.valid() )
    //    OE_NOTICE << "New Diamond: " << _key->str() << std::endl;

    //if ( _key.valid() )
    //    OE_NOTICE << _name << ": " << _key->str() << " : " << _key->getGeoExtent().toString() << std::endl;

}

Diamond::~Diamond()
{
    s_numDiamonds--;
    //OE_NOTICE << s_numDiamonds << " ... " << std::endl;
}

const MeshNode&
Diamond::node() const
{
    return _mesh->node( _vi );
}

void
Diamond::activate()
{
    // this can only be called once all ancestors are established

    _status = ACTIVE;
    computeBound();

    if ( _hasGeometry )
    {
        // assign this diamond the stateset of the appropriate quadtree ancestor:
        int stateSetLevel = (int)_level;
        if ( stateSetLevel < (int)_mesh->_minActiveLevel )
            stateSetLevel = _mesh->_minActiveLevel;

        // obtain a pointer to the Diamond that owns the StateSet we wish to use:
        _targetStateSetOwner = this;
        while( _targetStateSetOwner->_level > stateSetLevel )
            _targetStateSetOwner = _targetStateSetOwner->_a[QUADTREE].get();

        // since that StateSet might not be populated yet, backtrack from there to find the
        // first available populated StateSet. This will serve as a temporary "placeholder"
        // until our target stateset is ready (i.e. the textures etc are loaded).
        _currentStateSetOwner = _targetStateSetOwner;
        while( !_currentStateSetOwner->_hasFinalImage && _currentStateSetOwner->_level > _mesh->_minActiveLevel )
        {
            _currentStateSetOwner = _currentStateSetOwner->_a[QUADTREE].get();
        }

        // assign the stateset to this Diamond's geometry.
        _amrDrawable->_stateSet = _currentStateSetOwner->_stateSet.get();

        // synchronize with the target state set. By doing this, we will detect when the target stateset
        // does finally get populated, and at that point we can replace the placeholder stateset with
        // the final stateset. (This check occurs in Diamond::cull.)
        _targetStateSetOwner->_stateSet->sync( _targetStateSetRevision );

#ifdef USE_TEXTURES

        // finally, queue up a request to populate the stateset if necessary.
        if ( !_targetStateSetOwner->_hasFinalImage )
        {
            _mesh->queueForImage( _targetStateSetOwner, 1.0f );
        }    

#endif
    }
}


//const osg::BoundingSphere&
//Diamond::getBound()
//{
//    if ( !_bsComputed )
//    {
//        computeBound();
//    }
//    return _bs;
//}

void
Diamond::setCoord( const osg::Vec3d& coord )
{
    _vi = _mesh->addNode( coord );
}

void
Diamond::seed( Level maxLevel )
{
    if ( maxLevel > _level )
    {
        for( ChildIndex c = 0; c < _childValence; ++c )
        {
            getOrCreateChild( c )->seed( maxLevel );
        }
    }
}

void
Diamond::split()
{
    // mark as split, and mark the primitive set as needing a refesh:
    _isSplit = true;

    if ( _hasGeometry )
    {
        // for geometry diamonds, a split means we must regenerate this diamond
        // AND its quadtree ancestor.
        this->dirty();
        _a[QUADTREE]->dirty();
    }
    else
    {
        // for intermediate diamonds, splitting means we must rebuild each immediate parent.
        _a[PARENT_L]->dirty();
        _a[PARENT_R]->dirty();
    }

    _queuedForSplit = false;

    // check to see whether any of our neighbors are split. If a neighbor is also
    // split, spawn a common child.
    for( ChildIndex c = 0; c < _childValence; ++c )
    {
        // debugging assertion:
        if ( _c[c].valid() )
        {
            OE_WARN << "ILLEGAL STATE: diamond just split but has kids!" << std::endl;
        }

        Diamond* d0 = getNeighbor( c );
        if ( d0 && d0->_isSplit )
        {
            getOrCreateChild( c );
        }
    }
}

void
Diamond::merge()
{
    // have to remove/merge all the children before we can merge.
    for( ChildIndex c = 0; c < _childValence; ++c )
    {
        if ( _c[c].valid() )
            removeChild( c );
    }
    _isSplit = false;

    if ( _hasGeometry )
    {
        this->dirty();
        _a[QUADTREE]->dirty();
    }
    else
    {
        // for intermediate diamonds, merging means we must rebuild each immediate parent.
        _a[PARENT_L]->dirty();
        _a[PARENT_R]->dirty();
    }

    _queuedForMerge = false;
}

#define DEVIATION 0

unsigned int
Diamond::cull( osgUtil::CullVisitor* cv )
{
    // NOTE:
    // the problem here is that if a diamond gets culled, none of its decendents are 
    // considered for merging. Perhaps a CULL-OUT means immediate merge-queueing? Or,
    // maybe we should check the traversal-number-delta. And if it's more than X,
    // queueForMerge.

    // if this diamond if maked INACTIVE, we have a problem
    if ( _status == INACTIVE )
    {
        OE_WARN << "ILLEGAL STATE: " << _name << " is marked INACTIVE but is being tested for cull!" << std::endl;
        return 0;
    }

    // trivial rejection: already traversed this diamond (multiple parenting)
    if ( cv->getTraversalNumber() <= _lastCullFrame )
        return 0;

    // record the framestamp so we don't cull more than once
    _lastCullFrame = cv->getTraversalNumber();
    
    const osg::Vec3d& eye = cv->getEyePoint();
    float range = (eye - visibleBound().center()).length();

    // NOTE: we never cull intermediate (non-geometry) diamonds.

    // if this geometry diamond is outside the extended bounds, it is eligible for merge AND
    // the diamond is not visible.
    if ( _hasGeometry && cv->isCulled( extendedBound() ) )
    {
        // start merging the children if this diamond fails the visibility tests.
        if (_level >= _mesh->_minActiveLevel && 
            _level <  _mesh->_maxActiveLevel && 
            _isSplit && 
            !_queuedForMerge && 
            !_queuedForSplit )
        {
            _mesh->queueForMerge( this, range );
            //OE_NOTICE << std::fixed << "isCUlled=false, range=" << range << ", radius=" << _bs.radius() << std::endl;
        }
        return 0;
    }

    // Back-face culling:
    // If the dot product of the eyepoint with each of the four ancestors is negative, then the
    // entire diamond is facing away from the camera and can be culled.
    if ( _hasGeometry )
    {
        int i;
        for( i=0; i<4; ++i )
        {
            osg::Vec3d eye_vec = eye - _a[i]->node()._vertex;
            double len = eye_vec.length();
            if ( len <= visibleBound().radius() ) break; // if we're inside the radius, bail
            double dev = ( eye_vec * _a[i]->node()._normal ) / len;
            if ( dev >= DEVIATION ) break;
        }
        if ( i == 4 )
            return 0;
    }

    // this will determine whether we actually add this diamond to the draw list later. we still
    // have to traverse children that are in the extended bounds (even if they're not in the view
    // frustum) in order to satifsy split requirements.
    bool inVisibleFrustum =
        _hasGeometry && !cv->isCulled( visibleBound() );

    // at this point, culling is now complete for this diamond.

#ifdef USE_TEXTURES

    // check to see whether the target stateset is "dirty".
    if ( _hasGeometry && _targetStateSetOwner->_stateSet->outOfSyncWith( _targetStateSetRevision ) )
    {
        // flags the primitive set for regeneration
        this->dirty();
    }

#endif

    // traverse the diamond's children.
    unsigned short numChildren = 0;
    for( ChildIndex c = 0; c < _childValence; ++c )
    {
        if ( _c[c].valid() )
        {
            ++numChildren;
            _c[c]->cull( cv );
        }
    }

    if ( _hasGeometry && _amrDrawable->_triangles.size() == 0 && numChildren < _childValence )
        OE_WARN << "BOOGER" << std::endl;

    //if ( _hasGeometry && _amrDrawable->_triangles.size() > 0 )
    if ( inVisibleFrustum )
    {
        _mesh->_amrDrawList.push_back( _amrDrawable.get() );
    }
    
    // culling is complete. next we will check to see if we need to split this diamond
    // based on LOD range.
    // FUTURE: splitting will depend on a couple factors: max split level (resolution),
    // available data, frame rate, etc.
    if ( _level >= _mesh->_minActiveLevel && _level < _mesh->_maxActiveLevel )
    {
        if (!_isSplit && 
            range < extendedBound().radius() * CULL_RANGE_FACTOR &&
            !_queuedForSplit &&
            !_queuedForMerge &&
            numChildren < _childValence )
        {
            _mesh->queueForSplit( this, -range );
            _queuedForMerge = false;
        }

        else if (
            _isSplit &&
            range > extendedBound().radius() * CULL_RANGE_FACTOR &&
            !_queuedForSplit &&
            !_queuedForMerge &&
            numChildren > 0 )
        {
            _mesh->queueForMerge( this, range );
            _queuedForSplit = false;
        }
    }

    return 0;
    //return _numGeometriesAdded;
}

void
Diamond::computeBound()
{
    // in vert space.
    _visibleBound = osg::BoundingSphere( node()._vertex, 1.0 );
    for(int i=0; i<4; i++) 
        _visibleBound.expandRadiusBy( _a[i]->node()._vertex );

    _extendedBound = _visibleBound;
    _extendedBound.radius() = _extendedBound.radius() * sqrt(2.0) * 2.0;

    _bsComputed = true;
}

const osg::BoundingSphere&
Diamond::visibleBound() const
{
    if ( !_bsComputed )
        const_cast<Diamond*>(this)->computeBound();
    return _visibleBound;
}

const osg::BoundingSphere&
Diamond::extendedBound() const
{
    if ( !_bsComputed )
        const_cast<Diamond*>(this)->computeBound();
    return _extendedBound;
}

void
Diamond::dump()
{
    OE_NOTICE
        << "Diamond " << _name << "\n" //" (" << _v.x() << "," << _v.y() << "," << _v.z() << ") :\n"
        << "  level    = " << _level << "\n"
        << "  parent_R = " << _a[PARENT_R]->_name << "\n"
        << "  parent_L = " << _a[PARENT_L]->_name << "\n"
        << "  quadtree = " << _a[QUADTREE]->_name << "\n"
        << "  gdparent = " << _a[GDPARENT]->_name << "\n"
        << "  children = ";
    
    for( int i=0; i<_childValence; ++i )
        OE_NOTICE << i << ":<" << ( _c[i].valid()? _c[i]->_name : "") << ">  ";

    OE_NOTICE << std::endl << std::endl;
}

void
Diamond::setChild( ChildIndex index, Diamond* child )
{
    _c[index] = child;

    if ( child->_hasGeometry )
    {
        // if this is a geometry diamond, invalidate the quadtree ancestor so it can include it.
        child->_a[QUADTREE]->dirty();
        child->dirty();
    }
    else
    {
        this->_a[PARENT_L]->dirty();
        this->_a[PARENT_R]->dirty();
    }
}

void
Diamond::dirty()
{
    // queue this diamond for a primitive set refresh .. but only if it actually has geometry.
    if ( _hasGeometry && _level >= _mesh->_minGeomLevel )
    {
        if ( !_drawableDirty )
        {
            _drawableDirty = true;
#ifdef USE_DIRTY_QUEUE
            _mesh->queueForRefresh( this );
#else
            refreshDrawable();
#endif
        }
    }
    else
    {
        //OE_WARN << "ILLEGAL: dirty(): _hasGeometry = " << _hasGeometry << ", _level = " << _level << std::endl;
    }
}

// texture coordinate for each orientation.
#define T_QUADTREE 0
#define T_CHILD_0  1
#define T_PARENT_R 2
#define T_CHILD_1  3
#define T_GDPARENT 4
#define T_CHILD_2  5
#define T_PARENT_L 6
#define T_CHILD_3  7

static osg::Vec2f otex[8] =
{
    osg::Vec2f(0.0,0.0), // T_QUADTREE
    osg::Vec2f(0.5,0.0), // T_CHILD_0
    osg::Vec2f(1.0,0.0), // T_PARENT_R
    osg::Vec2f(1.0,0.5), // T_CHILD_1
    osg::Vec2f(1.0,1.0), // T_GDPARENT
    osg::Vec2f(0.5,1.0), // T_CHILD_2
    osg::Vec2f(0.0,1.0), // T_PARENT_L
    osg::Vec2f(0.0,0.5)  // T_CHILD_3
};

#define OT(I,R) otex[ ( I + R ) % 8 ]

#define ADD_TRI( COORDS, TEX, C1, T1, C2, T2, C3, T3, OFFSET, SPAN ) { \
    COORDS->push_back(C1); \
    COORDS->push_back(C2); \
    COORDS->push_back(C3); \
    TEX->push_back( OFFSET + (T1*SPAN) ); \
    TEX->push_back( OFFSET + (T2*SPAN) ); \
    TEX->push_back( OFFSET + (T3*SPAN) );

void
Diamond::refreshDrawable()
{
    // the primitive set may have already been refreshed (due to double-parenting)
    if ( !_drawableDirty )
        return;

    // this level does not generate primitive sets
    if ( _level < _mesh->_minGeomLevel )
        return;

    if ( !_hasGeometry )
    {
        OE_WARN << "ILLEGAL: refreshPrimitiveSet called on Diamond at no-geom level " << _level << std::endl;
        return;
    }

    // figure out the subrange transformation.
    osg::Vec2f offset( 0.0, 0.0 );
    double span = 1.0;

    const TileKey& ssaKey = _currentStateSetOwner->_key;
    if ( _level > _currentStateSetOwner->_level ) //_currentStateSetOwner != _targetStateSetOwner )
    {
        span = 1.0/(double)(1 << ((_level-_currentStateSetOwner->_level)/2));
        offset.x() = (_key.getExtent().xMin()-ssaKey.getExtent().xMin())/ssaKey.getExtent().width();
        offset.y() = (_key.getExtent().yMin()-ssaKey.getExtent().yMin())/ssaKey.getExtent().height();
    }

    int o = _orientation;
    
    // Start by clearing out the old primitive set:
    _amrDrawable->_triangles.clear();

    //if ( false ) //!_isSplit ) // took this out to preserve the diamond center point.
    //{
    //    // if the diamond is not split, simply draw the two triangles.
    //    _amrDrawable->add( new AMRTriangle(
    //        _a[GDPARENT]->coord(), _a[GDPARENT]->vert(), offset + OT(T_GDPARENT,o) * span,
    //        _a[QUADTREE]->coord(), _a[QUADTREE]->vert(), offset + OT(T_QUADTREE,o) * span,
    //        _a[PARENT_R]->coord(), _a[PARENT_R]->vert(), offset + OT(T_PARENT_R,o) * span ) );

    //    _amrDrawable->add( new AMRTriangle(
    //        _a[GDPARENT]->coord(), _a[GDPARENT]->vert(), offset + OT(T_GDPARENT,o) * span,
    //        _a[PARENT_L]->coord(), _a[PARENT_L]->vert(), offset + OT(T_PARENT_L,o) * span,
    //        _a[QUADTREE]->coord(), _a[QUADTREE]->vert(), offset + OT(T_QUADTREE,o) * span ) );
    //}
    //else
    {
        // find this diamond's four quadtree descendants:
        Diamond* q0 = _c[0].valid() ? _c[0]->_c[1].get() : 0L;
        Diamond* q1 = _c[1].valid() ? _c[1]->_c[3].get() : 0L;
        Diamond* q2 = _c[2].valid() ? _c[2]->_c[1].get() : 0L;
        Diamond* q3 = _c[3].valid() ? _c[3]->_c[3].get() : 0L;

        osg::Vec2f center = osg::Vec2f(.5,.5);

        if ( !_c[0].valid() || !_c[0]->_isSplit )
        {
            _amrDrawable->add( new AMRTriangle(
                node(),               offset + center * span,
                _a[QUADTREE]->node(), offset + OT(T_QUADTREE,o) * span,
                _a[PARENT_R]->node(), offset + OT(T_PARENT_R,o) * span ) );
        }
        else
        {
            if ( !q0 )
            {
                _amrDrawable->add( new AMRTriangle(
                    node(),               offset + center * span,
                    _a[QUADTREE]->node(), offset + OT(T_QUADTREE,o) * span,
                    _c[0]->node(),        offset + OT(T_CHILD_0,o) * span ) );
            }
            if ( !q1 )
            {
                _amrDrawable->add( new AMRTriangle(
                    node(),               offset + center * span,
                    _c[0]->node(),        offset + OT(T_CHILD_0,o) * span,
                    _a[PARENT_R]->node(), offset + OT(T_PARENT_R,o) * span ) );
            }
        }

        if ( !_c[1].valid() || !_c[1]->_isSplit )
        {
            _amrDrawable->add( new AMRTriangle(
                node(),               offset + center * span,
                _a[PARENT_R]->node(), offset + OT(T_PARENT_R,o) * span,
                _a[GDPARENT]->node(), offset + OT(T_GDPARENT,o) * span ) );
        }
        else
        {
            if ( !q1 )
            {
                _amrDrawable->add( new AMRTriangle(
                    node(),               offset + center * span,
                    _a[PARENT_R]->node(), offset + OT(T_PARENT_R,o) * span,
                    _c[1]->node(),        offset + OT(T_CHILD_1,o) * span ) );
            }
            if ( !q2 )
            {
                _amrDrawable->add( new AMRTriangle(
                    node(),               offset + center * span,
                    _c[1]->node(),        offset + OT(T_CHILD_1,o) * span,
                    _a[GDPARENT]->node(), offset + OT(T_GDPARENT,o) * span ) );
            }
        }

        if ( !_c[2].valid() || !_c[2]->_isSplit )
        {
            _amrDrawable->add( new AMRTriangle(
                node(),               offset + center * span,
                _a[GDPARENT]->node(), offset + OT(T_GDPARENT,o) * span,
                _a[PARENT_L]->node(), offset + OT(T_PARENT_L,o) * span ) );
        }
        else
        {
            if ( !q2 )
            {
                _amrDrawable->add( new AMRTriangle(
                    node(),               offset + center * span,
                    _a[GDPARENT]->node(), offset + OT(T_GDPARENT,o) * span,
                    _c[2]->node(),        offset + OT(T_CHILD_2,o) * span ) );
            }
            if ( !q3 )
            {
                _amrDrawable->add( new AMRTriangle(
                    node(),               offset + center * span,
                    _c[2]->node(),        offset + OT(T_CHILD_2,o) * span,
                    _a[PARENT_L]->node(), offset + OT(T_PARENT_L,o) * span ) );
            }
        }

        if ( !_c[3].valid() || !_c[3]->_isSplit )
        {
            _amrDrawable->add( new AMRTriangle(
                node(),               offset + center * span,
                _a[PARENT_L]->node(), offset + OT(T_PARENT_L,o) * span,
                _a[QUADTREE]->node(), offset + OT(T_QUADTREE,o) * span ) );
        }
        else
        {
            if ( !q3 )
            {
                _amrDrawable->add( new AMRTriangle(
                    node(),               offset + center * span,
                    _a[PARENT_L]->node(), offset + OT(T_PARENT_L,o) * span,
                    _c[3]->node(),        offset + OT(T_CHILD_3,o) * span ) );
            }
            if ( !q0 )
            {
                _amrDrawable->add( new AMRTriangle(
                    node(),               offset + center * span,
                    _c[3]->node(),        offset + OT(T_CHILD_3,o) * span,
                    _a[QUADTREE]->node(), offset + OT(T_QUADTREE,o) * span ) );
            }
        }        
    }

    // dirty the underlying element buffer object
    _drawableDirty = false;
}

bool
Diamond::hasChildren() const
{
    for( unsigned short c = 0; c < _childValence; ++c )
    {
        if ( _c[c].valid() )
            return true;
    }
    return false;
}

bool
Diamond::hasAllChildren() const
{
    for( unsigned short c = 0; c < _childValence; ++c )
    {
        if ( !_c[c].valid() )
            return false;
    }
    return true;
}

ChildIndex
Diamond::getIndexOfChild( Diamond* child )
{
    for( ChildIndex c = 0; c < _childValence; ++c )
    {
        if ( _c[c].get() == child )
            return c;
    }
    return -1;
}

Diamond*
Diamond::getNeighbor( ChildIndex c )
{   
    unsigned short parent = c <= 1 ? PARENT_R : PARENT_L;
    ChildIndex myIndexInParent = _a[parent]->getIndexOfChild( this );
    ChildIndex i = ( myIndexInParent + (c==0 || c==2? 1 : 3) ) % _a[parent]->_childValence;
    return _a[parent]->_c[i].get();
}

Diamond*
Diamond::getOrCreateNeighbor( ChildIndex c )
{   
    //OE_NOTICE << "Looking up neighbor (" << c << ") of " << _name << std::endl;
    unsigned short parent = c <= 1 ? PARENT_R : PARENT_L;
    ChildIndex myIndexInParent = _a[parent]->getIndexOfChild( this );
    ChildIndex i = ( myIndexInParent + (c==0 || c==2? 1 : 3) ) % _a[parent]->_childValence;
    //if ( !_a[parent]->_c[i].valid() )
    //    OE_NOTICE << "...doesn't exist, creating:" << std::endl;
    return _a[parent]->getOrCreateChild( i );
}

ChildIndex 
Diamond::getIndexOfChildEdgeStartingAt( NodeIndex vi )
{
    if ( vi == _a[QUADTREE]->_vi ) return 0;
    else if ( vi == _a[PARENT_R]->_vi ) return 1;
    else if ( vi == _a[GDPARENT]->_vi ) return 2;
    else return 3;
}

Diamond*
Diamond::getOrCreateChild( ChildIndex c )
{
    if ( _c[c].valid() )
        return _c[c].get();

    // must force a split before creating children. dont' need to call dirty() because
    // that gets called later by setChild()
    _isSplit = true;

    //OE_NOTICE << "Creating child of " << _name << " at " << c << std::endl;
    //dump();

    Diamond* d  = this;
    osg::ref_ptr<Diamond> d0 = getOrCreateNeighbor( c );

    unsigned short whichParent = c <= 1 ? PARENT_R : PARENT_L;
    osg::ref_ptr<Diamond> qa = d->_a[whichParent].get(); // the common parent

    Diamond* child = new Diamond( _mesh, TileKey(), d->_level+1 );
    //Diamond* child = new Diamond( _mesh, 0L, d->_level+1,  " ("+_name+" + "+d0->_name+") " );

    //OE_NOTICE << "...neighbor: " << d0->_name << std::endl;   
    //OE_NOTICE << "...common parent: " << commonParent->_name << std::endl;
    
    // assign child's quadtree ansector to d and d0's common parent.
    child->_a[QUADTREE] = qa.get();

    // assign child's grandparent ansector pointer:
    child->_a[GDPARENT] = c == 0 || c == 3 ? d->_a[QUADTREE].get() : d->_a[GDPARENT].get();

    // determine the child index of child in d0:
    unsigned short p = c==0 || c==2 ? QUADTREE : GDPARENT;
    ChildIndex c_d0 = d0->getIndexOfChildEdgeStartingAt( child->_a[p]->_vi );
    if ( c_d0 < 0 )
    {
        OE_WARN << "Diamond: getOrCreateChild: illegal state, cannot find neighbor's child edge...crash immiment\n";
    }

    // assign the child's parent pointers
    child->_a[PARENT_R] = (c == 0 || c == 2) ? d : d0.get();
    child->_a[PARENT_L] = (c == 0 || c == 2) ? d0.get() : d;

    // how that we know the QUADTREE & GDPARENT ancestors, create the diamond vertex.
    osg::Vec3d newCoord = _mesh->_manifold->midpoint(
        child->_a[QUADTREE]->node()._manifoldCoord,
        child->_a[GDPARENT]->node()._manifoldCoord );

    child->setCoord( newCoord );

    // assign the new child in both d and d0:
    d->setChild( c, child );
    d0->setChild( c_d0, child );

    if ( child->_hasGeometry )
    {
        // if this is a intermediate diamond, make a new key for the parent's quadtree descendant.
        // if this is a geometry diamond, the new intermediate child gets no key.
        int quadrant = -1;

        if ( qa->_orientation == 0 )
        {
            quadrant = 
                qa->q(0) == child ? 2 :
                qa->q(1) == child ? 3 :
                qa->q(2) == child ? 1 : 0;
        }
        else if ( qa->_orientation == 2 )
        {
            quadrant =
                qa->q(0) == child ? 3 :
                qa->q(1) == child ? 1 :
                qa->q(2) == child ? 0 : 2;
        }
        else if ( qa->_orientation == 4 )
        {
            quadrant =
                qa->q(0) == child ? 1 :
                qa->q(1) == child ? 0 :
                qa->q(2) == child ? 2 : 3;
        }
        else // if ( qa->_orientation == 6 )
        {
            quadrant =
                qa->q(0) == child ? 0 :
                qa->q(1) == child ? 2 :
                qa->q(2) == child ? 3 : 1;
        }

        child->_key = qa->_key.createChildKey( quadrant );

        child->_orientation = 
            quadrant == 1 ? 0 :
            quadrant == 0 ? 2 :
            quadrant == 2 ? 4 : 6;
    }

    child->activate();

    return child;
}

void
Diamond::removeChild( ChildIndex c )
{
    osg::ref_ptr<Diamond> child = _c[c].get();
    if ( !child.valid() ) return;

    // first we must merge the child (i.e. remove all of ITS children recursively).
    // TODO: this probably needs to queue up another merge operation with a higher priority.
    // that should probably happen in the mesh manager.
    child->merge();

    // deactivate the child. it's possible that the child is in a queue somewhere. this
    // will prevent it from being processed after it's been removed.
    child->_status = INACTIVE;

    // remove it from its other parent as well.
    osg::ref_ptr<Diamond> d0 = getNeighbor( c );
    if ( d0.valid() )
    {
        ChildIndex c_d0 = d0->getIndexOfChild( child.get() );
        if ( c_d0 >= 0 )
        {
            d0->_c[c_d0] = 0L;
        }
    }

    // now clear out the child slot and invalidate the primitive set.
    this->_c[c] = 0L;

    // notify the common ancestry of the change
    if ( child->_hasGeometry )
    {
        child->_a[QUADTREE]->dirty();
        this->dirty();
    }
    else
    {
        // probably not strictly necessary since a split() will take care of this.. gw
        child->_a[PARENT_L]->dirty();
        child->_a[PARENT_R]->dirty();
    }

    // zero out the child's ancestor pointers.
    for( AncestorIndex i = 0; i < 4; ++i )
        child->_a[i] = 0L;
    for( ChildIndex i = 0; i < 4; ++i )
        child->_c[i] = 0L;

    // remove the child's vertex from the VBO.
    _mesh->removeNode( child->_vi );
}


