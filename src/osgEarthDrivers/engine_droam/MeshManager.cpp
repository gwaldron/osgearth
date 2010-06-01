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
#include "MeshManager"
#include <osg/CullFace>
#include <osg/Texture2D>

// --------------------------------------------------------------------------

struct ImageRequest : public osgEarth::TaskRequest
{
    ImageRequest( MapLayer* layer, const TileKey* key ) : _layer(layer), _key(key) { }

    void operator()( ProgressCallback* progress )
    {
        _result = _layer->createImage( _key.get() );
    }

    osg::ref_ptr<MapLayer> _layer;
    osg::ref_ptr<const TileKey> _key;
};

// --------------------------------------------------------------------------

MeshManager::MeshManager( Manifold* manifold, Map* map ) :
_manifold( manifold ),
_map( map ),
_minGeomLevel( 1 ),
_minActiveLevel( 0 ),
_maxActiveLevel( MAX_ACTIVE_LEVEL ),
_maxJobsPerFrame( MAX_JOBS_PER_FRAME )
{
    // fire up a task service to load textures.
    _imageService = new TaskService( "Image Service", 16 );

    // this geometry is used solely to create VBOs and to act as a cloning
    // prototype for diamonds.
    _geomPrototype = new osg::Geometry();
    _geomPrototype->setUseDisplayList( false );
    _geomPrototype->setUseVertexBufferObjects( true );
    _geomPrototype->setDataVariance( osg::Object::DYNAMIC );

    // this vertex array will be shared amongst all diamonds in the manifold.
    _verts = new osg::Vec3Array();
    _geomPrototype->setVertexArray( _verts.get() );
    _verts->reserve( RESERVED_VERTICES ); 
    _verts->getVertexBufferObject()->setUsage(GL_DYNAMIC_DRAW_ARB);

    // one-for-one normal array to match the verts.
    _normals = new osg::Vec3Array();
    _geomPrototype->setNormalArray( _normals.get() );
    _geomPrototype->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );
    _normals->reserve( RESERVED_VERTICES );
    _normals->getVertexBufferObject()->setUsage(GL_DYNAMIC_DRAW_ARB);

#ifdef USE_VERTEX_COLORS
    // one-for-one color array.
    _colors = new osg::Vec4Array();
    _geomPrototype->setColorArray( _colors.get() );
    _geomPrototype->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
    _colors->reserve( RESERVED_VERTICES );
    _colors->getVertexBufferObject()->setUsage(GL_DYNAMIC_DRAW_ARB);
#endif

    // the master geode containing the entire visible terrain.
    _geode = new osg::Geode();
    _geode->setDataVariance( osg::Object::DYNAMIC );
    _geode->getOrCreateStateSet()->setAttributeAndModes( new osg::CullFace( osg::CullFace::BACK ), 1 );

    // set up the manifold framework.
    manifold->initialize( this );
}

VertexIndex
MeshManager::addVert(const osg::Vec3f& vert, const osg::Vec3f& normal, const osg::Vec4f& color)
{
    VertexIndex result;

    if ( _freeList.empty() )
    {
        _verts->push_back( vert );
        _normals->push_back( normal );
#ifdef USE_VERTEX_COLORS
        _colors->push_back( color );
#endif
        result = _verts->size()-1;
    }
    else
    {
        VertexIndex vi = _freeList.front();
        _freeList.pop();
        (*_verts)[vi] = vert;
        (*_normals)[vi] = normal;
#ifdef USE_VERTEX_COLORS
        (*_colors)[vi] = color;
#endif
        result = vi;
    }

    _verts->dirty();
    _normals->dirty();

#ifdef USE_VERTEX_COLORS
    _colors->dirty();
#endif

    //OE_NOTICE << "Verts=" << _verts->size() << ", used=" << _verts->size()-_freeList.size() << ", free=" << _freeList.size() << std::endl;

    return result;
}

void
MeshManager::removeVert( VertexIndex vi )
{
    // DO NOT remove a vert that is already on the free list
    // we don't check for that.
    _freeList.push( vi );
}

void
MeshManager::queueForRefresh( Diamond* d )
{
    //OE_NOTICE << d->_name << ": queued for refresh." << std::endl;
    _dirtyQueue.push( d );
}

void
MeshManager::queueForSplit( Diamond* d, float priority )
{    
    if ( !d->_queuedForSplit )
    {
        //OE_NOTICE << "q split: " << d->_name << std::endl;        
        _splitQueue.push( DiamondJob( d, priority ) );
        d->_queuedForSplit = true;
    }
}

void
MeshManager::queueForMerge( Diamond* d, float priority )
{
    if ( !d->_queuedForMerge )
    {
        //OE_NOTICE << "q merge: " << d->_name << std::endl;
        _mergeQueue.push( DiamondJob( d, priority ) );
        d->_queuedForMerge = true;
    }
}

void
MeshManager::queueForImage( Diamond* d, float priority )
{
    if ( !d->_queuedForImage && !d->_imageRequest.valid() )
    {
        //OE_NOTICE << "REQ: " << d->_key->str() << " queueing request..." << std::endl;
        d->_imageRequest = new ImageRequest( _map->getImageMapLayers()[0].get(), d->_key.get() );
        _imageService->add( d->_imageRequest.get() );
        _imageQueue.push_back( DiamondJob( d, priority ) ); //.insert( DiamondJob( d, priority ) );
        d->_queuedForImage = true;
    }
}

void
MeshManager::update()
{
    int j;

    // process the split queue. these are diamonds that have requested to be split into
    // all four children.
    for( j=0; j<_maxJobsPerFrame && !_splitQueue.empty(); ++j )
    //if( !_splitQueue.empty() )
    {
        Diamond* d = _splitQueue.top()._d.get();
        if ( d->_status == ACTIVE && d->referenceCount() > 1 )
        {
            if ( d->_queuedForSplit )
            {
                //OE_NOTICE << "split: " << d->_name << "\n";
                d->split();
                d->_queuedForSplit = false;
                d->_queuedForMerge = false;
            }
            else
            {
                //OE_WARN << d->_name << " was in the split Q, but NOT marked for splitting!" << std::endl;
            }
        }
        else
        {
            // the diamond was removed while in the queue. ignore it.
        }
        _splitQueue.pop();
    }

    // process the merge queue. these are diamonds that have requested that all their
    // children be removed.
    // FUTURE: process jobs until we reach some sort of time quota?

    for( j=0; j<_maxJobsPerFrame && !_mergeQueue.empty(); ++j )
    {
        Diamond* d = _mergeQueue.top()._d.get();
        if ( d->_status == ACTIVE && d->referenceCount() > 1 )
        {
            if ( d->_queuedForMerge )
            {
                //OE_NOTICE << "merge: " << d->_name << "\n";

                //TODO: when you merge, children are recursively merged..thus the merge
                //may take some time. rather it might be better to traverse and schedule
                //child merges first.?
                d->merge();
                d->_queuedForMerge = false;
                d->_queuedForSplit = false;
            }
            else
            {
                //this means that the diamond was once queued for a merge, but then
                // passed cull again.
                //OE_WARN << d->_name << " was in the merge Q, but NOT marked for merging!" << std::endl;
            }
        }
        _mergeQueue.pop();
    }

    // process the texture image request queue.
    j=0;
    for( DiamondJobList::iterator i = _imageQueue.begin(); i != _imageQueue.end() && j < _maxJobsPerFrame; ++j )
    //if( _imageQueue.size() > 0 )
    {
        bool increment = true;
        bool remove = true;

        Diamond* d = i->_d.get();

        if ( d->_status == ACTIVE && d->referenceCount() > 1 && d->_imageRequest.valid() )
        {
            if ( d->_imageRequest->isCompleted() )
            {
                //OE_NOTICE << "REQ: " << d->_key->str() << " completed" << std::endl;
                osg::Texture2D* tex = 0L;
                
    #ifdef USE_DEBUG_TEXTURES

                tex = new osg::Texture2D();
                tex->setImage( createDebugImage() );

    #else

                GeoImage* geoImage = dynamic_cast<GeoImage*>( d->_imageRequest->getResult() );
                if ( geoImage )
                {
                    tex = new osg::Texture2D();
                    tex->setImage( geoImage->getImage() );
                }

    #endif // USE_DEBUG_TEXTURES

                if ( tex )
                {
                    tex->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );
                    tex->setWrap( osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE );
                    tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
                    tex->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
                    d->_stateSet->setTextureAttributeAndModes( 0, tex, osg::StateAttribute::ON );
                    d->_stateSet->dirty(); // bump revision number so that users of this stateset can detect the change
                    d->_hasFinalImage = true;
                }

                remove = true;
            }

            else
            {
                remove = false;
            }
        }


        if ( remove )
        {
            d->_imageRequest = 0L;
            d->_queuedForImage = false;
            i = _imageQueue.erase( i );
        }
        else
        {
            ++i;
        }
    }

#ifdef USE_DIRTY_QUEUE

    // process the dirty diamond queue. these are diamonds that have been changed and
    // need a new primitive set.
    // NOTE: we need to process the entire dirty queue each frame.
    while( _dirtyQueue.size() > 0 )
    {
        Diamond* d = _dirtyQueue.front().get();

        if ( d->_status == ACTIVE && d->referenceCount() > 1 )
        {
            // first, check to see whether the diamond's target stateset is ready. if so,
            // install it and mark it up to date.
            if ( d->_targetStateSetOwner->_stateSet->outOfSyncWith( d->_targetStateSetRevision ) )
            {            
                d->_geom->setStateSet( d->_targetStateSetOwner->_stateSet.get() );
                d->_currentStateSetOwner = d->_targetStateSetOwner;
                d->_targetStateSetOwner->_stateSet->sync( d->_targetStateSetRevision );
            }

            // rebuild the primitives now.
            d->refreshPrimitiveSet();
        }
        _dirtyQueue.pop();
    }

#endif

    //OE_NOTICE << "dq size = " << _dirtyQueue.size() << "; splits = " << _splitQueue.size() << "; merges = " << _mergeQueue.size() << std::endl;
}
