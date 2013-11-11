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
#include <osgEarth/StateSetCache>
#include <osg/NodeVisitor>
#include <osg/Geode>
#include <osg/BufferIndexBinding>

#define LC "[StateSetCache] "

#define PRUNE_ACCESS_COUNT 40

using namespace osgEarth;

//---------------------------------------------------------------------------

namespace
{
    /**
     * Visitor that calls StateSetCache::share on all attributes found
     * in a scene graph.
     */
    struct ShareStateAttributes : public osg::NodeVisitor
    {
        StateSetCache* _cache;

        ShareStateAttributes(StateSetCache* cache) :
            osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ),
            _cache          ( cache ) { }

        void apply(osg::Node& node)
        {
            if ( node.getStateSet() && node.getStateSet()->getDataVariance() != osg::Object::DYNAMIC )
            {
                applyStateSet( node.getStateSet() );
            }
            traverse(node);
        }

        void apply(osg::Geode& geode)
        {
            unsigned numDrawables = geode.getNumDrawables();
            for( unsigned i=0; i<numDrawables; ++i )
            {
                osg::Drawable* d = geode.getDrawable(i);
                if ( d && d->getStateSet() && d->getStateSet()->getDataVariance() != osg::Object::DYNAMIC )
                {
                    applyStateSet( d->getStateSet() );
                }
            }
            apply((osg::Node&)geode);
        }

        void applyStateSet(osg::StateSet* stateSet)
        {
            osg::StateSet::AttributeList& attrs = stateSet->getAttributeList();
            for( osg::StateSet::AttributeList::iterator i = attrs.begin(); i != attrs.end(); ++i )
            {
                osg::ref_ptr<osg::StateAttribute> in, shared;
                in = i->second.first.get();
                if ( in.valid() && _cache->share(in, shared) )
                {
                    i->second.first = shared.get();
                }
            }

            osg::StateSet::TextureAttributeList& texAttrs = stateSet->getTextureAttributeList();
            for( osg::StateSet::TextureAttributeList::iterator j = texAttrs.begin(); j != texAttrs.end(); ++j )
            {
                osg::StateSet::AttributeList& attrs = *j;
                for( osg::StateSet::AttributeList::iterator i = attrs.begin(); i != attrs.end(); ++i )
                {
                    osg::StateAttribute* sa = i->second.first.get();
                    osg::ref_ptr<osg::StateAttribute> in, shared;
                    in = i->second.first.get();
                    if ( in.valid() && _cache->share(in, shared) )
                    {
                        i->second.first = shared.get();
                    }
                }
            }
        }
    };


    /**
     * Visitor that calls StateSetCache::share on all statesets found
     * in a scene graph.
     */
    struct ShareStateSets : public osg::NodeVisitor
    {
        StateSetCache* _cache;
        unsigned       _stateSets;
        unsigned       _shares;
        //std::vector<osg::StateSet*> _misses; // for debugging

        ShareStateSets(StateSetCache* cache) :
            osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ),
            _cache    ( cache ),
            _stateSets( 0 ),
            _shares   ( 0 ) { }

        void apply(osg::Node& node)
        {
            if ( node.getStateSet() && node.getStateSet()->getDataVariance() != osg::Object::DYNAMIC )
            {
                _stateSets++;
                osg::ref_ptr<osg::StateSet> in, shared;
                in = node.getStateSet();
                if ( in.valid() && _cache->share(in, shared) )
                {
                    node.setStateSet( shared.get() );
                    _shares++;
                }
                //else _misses.push_back(in.get());
            }
            traverse(node);
        }

        void apply(osg::Geode& geode)
        {
            unsigned numDrawables = geode.getNumDrawables();
            for( unsigned i=0; i<numDrawables; ++i )
            {
                osg::Drawable* d = geode.getDrawable(i);
                if ( d && d->getStateSet() && d->getStateSet()->getDataVariance() != osg::Object::DYNAMIC )
                {
                    _stateSets++;
                    osg::ref_ptr<osg::StateSet> in, shared;
                    in = d->getStateSet();
                    if ( in.valid() && _cache->share(in, shared) )
                    {
                        d->setStateSet( shared.get() );
                        _shares++;
                    }
                    //else _misses.push_back(in.get());
                }
            }
            apply((osg::Node&)geode);
        }
    };
}

//------------------------------------------------------------------------

StateSetCache::StateSetCache() :
_pruneCount( 0 )
{
    //nop
}

StateSetCache::~StateSetCache()
{
    Threading::ScopedMutexLock lock( _mutex );
    prune();
}

void
StateSetCache::optimize(osg::Node* node)
{
    if ( node )
    {
        // replace all equivalent attributes with a single instance
        ShareStateAttributes v1( this );
        node->accept( v1 );

        
#if OSG_MIN_VERSION_REQUIRED(3,1,4)
        // replace all equivalent static statesets with a single instance
        // only supported in OSG 3.1.4+ because of the Uniform mutex 
        // protection.
        ShareStateSets v2( this );
        node->accept( v2 );
#endif
    }
}


bool
StateSetCache::eligible(osg::StateSet* stateSet) const
{
#if OSG_MIN_VERSION_REQUIRED(3,1,4)
    if ( !stateSet )
        return false;

    // DYNAMIC means the user intends to change it later. So it needs to
    // stay independent.
    if ( stateSet->getDataVariance() == osg::Object::DYNAMIC )
        return false;

    const osg::StateSet::AttributeList& attrs = stateSet->getAttributeList();
    for( osg::StateSet::AttributeList::const_iterator i = attrs.begin(); i != attrs.end(); ++i )
    {
        osg::StateAttribute* a = i->second.first.get();
        if ( !eligible(a) )
            return false;
    }

    return true;
#else
    return false;
#endif
}


bool
StateSetCache::eligible(osg::StateAttribute* attr) const
{
    if ( !attr )
        return false;

    // DYNAMIC means the user intends to change it later. So it needs to
    // stay independent.
    if ( attr->getDataVariance() == osg::Object::DYNAMIC )
        return false;

    // cannot share BIB's. They don't clone well since they have underlying buffer objects
    // that may be in use. It results in OpenGL invalid enumerant errors and errors such as
    // "uniform block xxx has no binding"
    if (dynamic_cast<osg::BufferIndexBinding*>(attr) != 0L)
        return false;

    return true;
}


bool
StateSetCache::share(osg::ref_ptr<osg::StateSet>& input,
                     osg::ref_ptr<osg::StateSet>& output,
                     bool                         checkEligible)
{
    bool shared     = false;
    bool shareattrs = true;

    if ( !checkEligible || eligible(input.get()) )
    {
        Threading::ScopedMutexLock lock( _mutex );

        pruneIfNecessary();

        shareattrs = false;

        std::pair<StateSetSet::iterator,bool> result = _stateSetCache.insert( input );
        if ( result.second )
        {
            // first use
            output = input.get();
            shareattrs = true;
            shared = false;
        }
        else
        {
            // found a share!
            output = result.first->get();
            shared = true;
        }
    }
    else
    {
        output = input.get();
        shared = false;
    }

    if ( shareattrs )
    {
        ShareStateAttributes sa(this);
        sa.applyStateSet( input.get() );
    }

    return shared;
}



bool
StateSetCache::share(osg::ref_ptr<osg::StateAttribute>& input,
                     osg::ref_ptr<osg::StateAttribute>& output,
                     bool                               checkEligible)
{
    if ( !checkEligible || eligible(input.get()) )
    {
        Threading::ScopedMutexLock lock( _mutex );

        pruneIfNecessary();

        std::pair<StateAttributeSet::iterator,bool> result = _stateAttributeCache.insert( input );
        if ( result.second )
        {
            // first use
            output = input.get();
            return false;
        }
        else
        {
            // found a share!
            output = result.first->get();
            return true;
        }
    }
    else
    {
        output = input.get();
        return false;
    }
}

void
StateSetCache::pruneIfNecessary()
{
    // assume an exclusve mutex is taken
    if ( _pruneCount++ == PRUNE_ACCESS_COUNT )
    {
        prune();
        _pruneCount = 0;
    }
}

void
StateSetCache::prune()
{
    // assume an exclusive mutex is taken.

    unsigned ss_count = 0, sa_count = 0;

    for( StateSetSet::iterator i = _stateSetCache.begin(); i != _stateSetCache.end(); )
    {
        if ( i->get()->referenceCount() == 1 )
        {
            // do not call releaseGLObjects since the attrs themselves might still be shared
            _stateSetCache.erase( i++ );
            ss_count++;
        }
        else
        {
            ++i;
        }
    }

    for( StateAttributeSet::iterator i = _stateAttributeCache.begin(); i != _stateAttributeCache.end(); )
    {
        if ( i->get()->referenceCount() == 1 )
        {
            i->get()->releaseGLObjects( 0L );
            _stateAttributeCache.erase( i++ );
            sa_count++;
        }
        else
        {
            ++i;
        }
    }

    OE_DEBUG << LC << "Pruned " << sa_count << " attributes, " << ss_count << " statesets" << std::endl;
}

void
StateSetCache::clear()
{
    Threading::ScopedMutexLock lock( _mutex );

    _stateAttributeCache.clear();
    _stateSetCache.clear();
}
