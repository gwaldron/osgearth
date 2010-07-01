/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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

Diamond::Diamond( MeshManager* mesh, osgEarth::TileKey* key, Level level, const std::string& name ) :
osg::Referenced(true),
//osgEarth::Revisioned<osg::Referenced>(),
_mesh( mesh ),
_key( key ),
_level( level ),
_name( name ),
_status( ACTIVE ),
_childValence( 4 ),
_color(1,1,1,1),
_lastCullFrame( 0 ),
_queuedForSplit( false ),
_queuedForMerge( false ),
_queuedForImage( false ),
_primSetDirty( false ),
_bsComputed( false ),
_isSplit( false ),
_hasGeometry( level % 2 == 1 ),
_stateSet( 0L ),
//_stateSetLevel( 0 ),
_currentStateSetOwner( 0L ),
_targetStateSetOwner( 0L ),
_targetStateSetRevision( -1 ),
_hasFinalImage( false )
{
    this->setThreadSafeRefUnref(true);
    //osg::setNotifyLevel( osg::INFO );

    // only ODD-numbered levels have actual geometry.
    if ( _hasGeometry )
    {
        // Create the geometry for rendering th "quadtree decendants" of this diamond.
        _geom = new osg::Geometry();
        _geom->setUseDisplayList( false );
        _geom->setUseVertexBufferObjects( true );

        // add the texture coordinates. note, we must apply this BEFORE setting the shared
        // vertex/normal/color arrays. otherwise the vertex processing gets corrupted. don't
        // ask me why. (to see the effect, just move these 2 lines after the assignments of
        // the shared vbos.)
#ifdef USE_TEXTURES
        _texCoords = new osg::Vec2Array();
        _geom->setTexCoordArray( 0, _texCoords );
#endif
        
        // but of course we need our own primitive set(s).
        _primSet = new osg::DrawElementsUInt( GL_TRIANGLES );
        _geom->addPrimitiveSet( _primSet );
        
        // copy the shared data from the mesh prototype:
        _geom->setVertexArray( _mesh->_geomPrototype->getVertexArray() );
        _geom->setNormalArray( _mesh->_geomPrototype->getNormalArray() );
        _geom->setNormalBinding( _mesh->_geomPrototype->getNormalBinding() );
        _geom->setColorArray( _mesh->_geomPrototype->getColorArray() );
        _geom->setColorBinding( _mesh->_geomPrototype->getColorBinding() );

        // this will prevent the DRAW of this geometry from overlapping the next UPDATE, making
        // it safe to change the geometry in the UPDATE traversal:
        _geom->setDataVariance( osg::Object::DYNAMIC );

        // create a stateset that will hold the textures for the quadtree starting at this diamond.
        // this is revisioned so that in activate() we can pre-populate the stateset with "placeholder"
        // lower-res textures if they are available (while loading the correct-res textures).
        _stateSet = new RevisionedStateSet();
        //_stateSetLevel = _level;
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
    releaseGLObjects();
}

void
Diamond::activate()
{
    // this can only be called once all ancestors are established

    _status = ACTIVE;
    computeBound();

    if ( _hasGeometry )
    {
        // this settings directs a Diamond to use a stateset from N levels back.
        int tsl = TEX_SUBRANGE_LEVELS;

        // for some unknown reason the polar faces come in much lower res. hack:
        //if ( osgEarth::UnifiedCubeProfile::getFace( _key.get() ) >= 4 )
        //    tsl--;

        // assign this diamond the stateset of the appropriate quadtree ancestor:
        int stateSetLevel = (int)_level - (tsl*2);
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
        _geom->setStateSet( _currentStateSetOwner->_stateSet.get() );

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


const osg::BoundingSphere&
Diamond::getBound()
{
    if ( !_bsComputed )
    {
        computeBound();
    }
    return _bs;
}

void
Diamond::setCoord( double x, double y, double z )
{
    setCoord( osg::Vec3d(x, y, z) );
}

void
Diamond::setCoord( const osg::Vec3d& coord )
{
    // store the parametric map coordinate in the diamond
    _coord = coord;

    // store the normal.
    //_normal = _mesh->_manifold->normal( _coord );

    // add the projected vert to the mesh, and store the index.
    Level a = _level % 3;
    osg::Vec4f color = a==0 ? RED : a==1 ? GREEN : YELLOW; //_level % 2 ? GREEN : YELLOW;
    osg::Vec3f vert = _mesh->_manifold->project( coord );
    _normal = _mesh->_manifold->normal( vert );
    _vi = _mesh->addVert( vert, _normal, color );
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
    float range = (eye - getBound().center()).length();
    bool isVisible = true;

    // trivial rejection: the bound does not intersect the view frustum
    // (note: we do NOT frustum-cull intermediate diamonds)
    if ( _hasGeometry && cv->isCulled( getBound() ) )
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
    //TODO: this does not work that great for some reason. Things disappear when the eyepoint
    // gets too low to the ground. Probably the higher-level disamond are getting culled
    // too soon....
    if ( _hasGeometry )
    {
        int i;
        for( i=0; i<4; ++i )
        {
            osg::Vec3d eye_vec = eye - _mesh->v( _a[i]->_vi );
            double len = eye_vec.length();
            if ( len <= getBound().radius() ) break; // if we're inside the radius, bail
            double dev = ( eye_vec * _a[i]->_normal ) / len;
            if ( dev >= DEVIATION ) break;
        }
        if ( i == 4 )
            return 0;
    }


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

    if ( _hasGeometry && _primSet->size() == 0 && numChildren < _childValence )
        OE_WARN << "BOOGER" << std::endl;
    
    // if the child traversal resulted in a full load of geometries, we do NOT need to
    // add the current geometry to the set.
    if ( _hasGeometry && _primSet->size() > 0 )
    {
#ifdef USE_AMR
        std::copy( _amrDrawList.begin(), _amrDrawList.end(), std::back_inserter(_mesh->_amrDrawList) );
#endif
        _mesh->_activeDrawables.push_back( _geom.get() );
    }
    
    // culling is complete. next we will check to see if we need to split this diamond
    // based on LOD range.
    // FUTURE: splitting will depend on a couple factors: max split level (resolution),
    // available data, frame rate, etc.
    if ( _level >= _mesh->_minActiveLevel && _level < _mesh->_maxActiveLevel )
    {
        if (!_isSplit && 
            range < _bs.radius() * CULL_RANGE_FACTOR &&
            !_queuedForSplit &&
            !_queuedForMerge &&
            numChildren < _childValence )
        {
            _mesh->queueForSplit( this, -range );
            _queuedForMerge = false;
        }

        else if (
            _isSplit &&
            range > _bs.radius() * CULL_RANGE_FACTOR &&
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
    _bs = osg::BoundingSphere( _mesh->v(_vi), 1.0 );
    for(int i=0; i<4; i++) 
        _bs.expandRadiusBy( _mesh->v( _a[i]->_vi ) );

    _bs.radius() = _bs.radius() * sqrt(2.0) * 2.0;

    _bsComputed = true;
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
        if ( !_primSetDirty )
        {
            _primSetDirty = true;
#ifdef USE_DIRTY_QUEUE
            _mesh->queueForRefresh( this );
#else
            refreshPrimitiveSet();
#endif
        }
    }
    else
    {
        //OE_WARN << "ILLEGAL: dirty(): _hasGeometry = " << _hasGeometry << ", _level = " << _level << std::endl;
    }
}

//#define ADD_TRI( P, T, A, C, D, G, H, J ) { \
//    (P)->push_back(A); (P)->push_back(D); (P)->push_back(H); \
//    (*T)[A] = C; (*T)[D] = G; (*T)[H] = J; }

#ifdef USE_TEXTURES

#   define ADD_TRI( P, T, A, C, D, G, H, J, O, S ) { \
        (P)->push_back(A); (P)->push_back(D); (P)->push_back(H); \
        (*T)[A] = O+(C*S); (*T)[D] = O+(G*S); (*T)[H] = O+(J*S); }

#else

#   define ADD_TRI( P, T, A, C, D, G, H, J, O, S ) { \
        (P)->push_back(A); (P)->push_back(D); (P)->push_back(H); }

#endif

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

#define T_CENTER osg::Vec2f(.5,.5)


void
Diamond::refreshPrimitiveSet()
{
    // the primitive set may have already been refreshed (due to double-parenting)
    if ( !_primSetDirty )
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

    const TileKey* ssaKey = _currentStateSetOwner->_key.get();
    if ( _level > _currentStateSetOwner->_level ) //_currentStateSetOwner != _targetStateSetOwner )
    {
        span = 1.0/(double)(1 << ((_level-_currentStateSetOwner->_level)/2));
        offset.x() = (_key->getGeoExtent().xMin()-ssaKey->getGeoExtent().xMin())/ssaKey->getGeoExtent().width();
        offset.y() = (_key->getGeoExtent().yMin()-ssaKey->getGeoExtent().yMin())/ssaKey->getGeoExtent().height();
    }

    // clear it out so we can build new triangles.
    osg::DrawElementsUInt* p = _primSet;
    _primSet->clear();

#ifdef USE_TEXTURES
    if ( _texCoords->size() < _mesh->_verts->size() )
        _texCoords->resize( _mesh->_verts->size() );
    osg::Vec2Array* t = _texCoords;
#endif

    int o = _orientation;

    if ( !_isSplit )
    {
        // if the diamond is not split, simply draw the two triangles.
        ADD_TRI( p, t, _a[GDPARENT]->_vi, OT(T_GDPARENT,o), _a[QUADTREE]->_vi, OT(T_QUADTREE,o), _a[PARENT_R]->_vi, OT(T_PARENT_R,o), offset, span );
        ADD_TRI( p, t, _a[GDPARENT]->_vi, OT(T_GDPARENT,o), _a[PARENT_L]->_vi, OT(T_PARENT_L,o), _a[QUADTREE]->_vi, OT(T_QUADTREE,o), offset, span );
    }
    else
    {
        // find this diamond's four quadtree descendants:
        Diamond* q0 = _c[0].valid() ? _c[0]->_c[1].get() : 0L;
        Diamond* q1 = _c[1].valid() ? _c[1]->_c[3].get() : 0L;
        Diamond* q2 = _c[2].valid() ? _c[2]->_c[1].get() : 0L;
        Diamond* q3 = _c[3].valid() ? _c[3]->_c[3].get() : 0L;

        if ( !_c[0].valid() || !_c[0]->_isSplit ) {
            ADD_TRI( p, t, _vi, T_CENTER, _a[QUADTREE]->_vi, OT(T_QUADTREE,o), _a[PARENT_R]->_vi, OT(T_PARENT_R,o), offset, span );
        }
        else {
            if ( !q0 ) {
                ADD_TRI( p, t, _vi, T_CENTER, _a[QUADTREE]->_vi, OT(T_QUADTREE,o), _c[0]->_vi, OT(T_CHILD_0,o), offset, span );
            }
            if ( !q1 ) {
                ADD_TRI( p, t, _vi, T_CENTER, _c[0]->_vi, OT(T_CHILD_0,o), _a[PARENT_R]->_vi, OT(T_PARENT_R,o), offset, span );
            }
        }

        if ( !_c[1].valid() || !_c[1]->_isSplit ) {
            ADD_TRI( p, t, _vi, T_CENTER, _a[PARENT_R]->_vi, OT(T_PARENT_R,o), _a[GDPARENT]->_vi, OT(T_GDPARENT,o), offset, span );
        }
        else {
            if ( !q1 ) {
                ADD_TRI( p, t, _vi, T_CENTER, _a[PARENT_R]->_vi, OT(T_PARENT_R,o), _c[1]->_vi, OT(T_CHILD_1,o), offset, span );
            }
            if ( !q2 ) {
                ADD_TRI( p, t, _vi, T_CENTER, _c[1]->_vi, OT(T_CHILD_1,o), _a[GDPARENT]->_vi, OT(T_GDPARENT,o), offset, span );
            }
        }

        if ( !_c[2].valid() || !_c[2]->_isSplit ) {
            ADD_TRI( p, t, _vi, T_CENTER, _a[GDPARENT]->_vi, OT(T_GDPARENT,o), _a[PARENT_L]->_vi, OT(T_PARENT_L,o), offset, span );
        }
        else {
            if ( !q2 ) {
                ADD_TRI( p, t, _vi, T_CENTER, _a[GDPARENT]->_vi, OT(T_GDPARENT,o), _c[2]->_vi, OT(T_CHILD_2,o), offset, span );
            }
            if ( !q3 ) {
                ADD_TRI( p, t, _vi, T_CENTER, _c[2]->_vi, OT(T_CHILD_2,o), _a[PARENT_L]->_vi, OT(T_PARENT_L,o), offset, span );
            }
        }

        if ( !_c[3].valid() || !_c[3]->_isSplit ) {
            ADD_TRI( p, t, _vi, T_CENTER, _a[PARENT_L]->_vi, OT(T_PARENT_L,o), _a[QUADTREE]->_vi, OT(T_QUADTREE,o), offset, span );
        }
        else {
            if ( !q3 ) {
                ADD_TRI( p, t, _vi, T_CENTER, _a[PARENT_L]->_vi, OT(T_PARENT_L,o), _c[3]->_vi, OT(T_CHILD_3,o), offset, span );
            }
            if ( !q0 ) {
                ADD_TRI( p, t, _vi, T_CENTER, _c[3]->_vi, OT(T_CHILD_3,o), _a[QUADTREE]->_vi, OT(T_QUADTREE,o), offset, span );
            }
        }        
    }

    // dirty the underlying element buffer object
    _primSetDirty = false;
    _primSet->dirty();

#ifdef USE_TEXTURES
    _texCoords->dirty();
#endif

#ifdef USE_AMR
    // temp: copy the tris over to the AMR draw list.
    _amrDrawList.clear();
    for(int i=0; i<_primSet->size(); i+=3)
    {
        osg::Vec3 p1 = _mesh->v( (*_primSet)[i] );
        osg::Vec3 p2 = _mesh->v( (*_primSet)[i+1] );
        osg::Vec3 p3 = _mesh->v( (*_primSet)[i+2] );
        osg::Vec3 n1 = p1; n1.normalize();
        osg::Vec3 n2 = p2; n2.normalize();
        osg::Vec3 n3 = p3; n3.normalize();
        AMRTriangle* tri = new AMRTriangle( p1, p2, p3, n1, n2, n3 );
        _amrDrawList.push_back( tri );
    }
#endif
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
Diamond::getIndexOfChildEdgeStartingAt( VertexIndex vi )
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

    Diamond* child = new Diamond( _mesh, 0L, d->_level+1 );
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
    osg::Vec3d newCoord = _mesh->_manifold->midpoint( child->_a[QUADTREE]->_coord, child->_a[GDPARENT]->_coord );
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

        child->_key = qa->_key->createSubkey( quadrant );

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
    //TODO:
    //disconnent all the pointers, and mark this diamond as "inactive" or "deleted". b/c it might be
    //    in a queue somewhere and we don't want to be messing with it once it's
    //    been removed!

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

        // no longer need this:
        //d0->dirty();
    }

    // now clear out the child slot and invalidate the primitive set.
    this->_c[c] = 0L;
    //this->dirty();

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
    _mesh->removeVert( child->_vi );
}

void
Diamond::releaseGLObjects()
{
    if ( _hasGeometry )
    {
        //TBD
        _stateSet->releaseGLObjects();
        _primSet->releaseGLObjects();
        _geom->releaseGLObjects();
    }
}

