/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/
#include <osgEarth/StateSetCache>
#include <osg/NodeVisitor>
#include <osg/BufferIndexBinding>
#include <osg/ProxyNode>

#define LC "[StateSetCache] "

#define DEFAULT_PRUNE_ACCESS_COUNT 40

#define STATESET_SHARING_SUPPORTED 1

using namespace osgEarth;

//---------------------------------------------------------------------------

namespace
{
    bool isEligible(osg::StateAttribute* attr)
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

    bool isEligible(osg::StateSet* stateSet)
    {
#ifdef STATESET_SHARING_SUPPORTED
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
            if ( !isEligible(a) )
                return false;
        }

        const osg::StateSet::TextureAttributeList& texattrs = stateSet->getTextureAttributeList();
        for( osg::StateSet::TextureAttributeList::const_iterator i = texattrs.begin(); i != texattrs.end(); ++i )
        {
            const osg::StateSet::AttributeList& texattrlist = *i;
            for( osg::StateSet::AttributeList::const_iterator i = texattrlist.begin(); i != texattrlist.end(); ++i )
            {
                osg::StateAttribute* a = i->second.first.get();
                if ( !isEligible(a) )
                    return false;
            }
        }

        return true;
#else
        return false;
#endif
    }

    /**
    * Visitor that calls StateSetCache::share on all attributes found
    * in a scene graph.
    */
    struct ShareStateAttributes : public osg::NodeVisitor
    {
        StateSetCache* _cache;
        bool           _traverseProxies;

        ShareStateAttributes(StateSetCache* cache, bool traverseProxies = true)
            : _cache(cache),
            _traverseProxies(traverseProxies)
        {
            setTraversalMode( TRAVERSE_ALL_CHILDREN );
            setNodeMaskOverride( ~0 );
        }

        void apply(osg::Node& node)
        {
            if (node.getStateSet())
            {
                // ref for thread-safety; another thread might replace this stateset
                // during optimization while we're looking at it.
                osg::ref_ptr<osg::StateSet> stateset = node.getStateSet();
                if (stateset.valid() && 
                    stateset->getDataVariance() != osg::Object::DYNAMIC )            
                {
                    applyStateSet( stateset.get() );
                }
            }
            traverse(node);
        }

        void apply(osg::ProxyNode& proxyNode)
        {
            if (_traverseProxies)
            {
                if (proxyNode.getStateSet())
                {
                    // ref for thread-safety; another thread might replace this stateset
                    // during optimization while we're looking at it.
                    osg::ref_ptr<osg::StateSet> stateset = proxyNode.getStateSet();
                    if (stateset.valid() &&
                        stateset->getDataVariance() != osg::Object::DYNAMIC)
                    {
                        applyStateSet(stateset.get());
                    }
                }
                traverse(proxyNode);
            }
        }

        // assume: stateSet is safely referenced by caller
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
        bool           _traverseProxies;
        //std::vector<osg::StateSet*> _misses; // for debugging

        ShareStateSets(StateSetCache* cache, bool traverseProxies)
            : _cache(cache),
            _stateSets( 0 ),
            _shares   ( 0 ),
            _traverseProxies(traverseProxies)

        {
            setTraversalMode( TRAVERSE_ALL_CHILDREN );
            setNodeMaskOverride( ~0 );
        }

        void apply(osg::Node& node)
        {
            if (node.getStateSet())
            {
                osg::ref_ptr<osg::StateSet> stateset = node.getStateSet();

                if ( isEligible(stateset.get()) )
                {
                    _stateSets++;
                    osg::ref_ptr<osg::StateSet> shared;
                    if ( _cache->share(stateset, shared) )
                    {
                        node.setStateSet( shared.get() );
                        _shares++;
                    }
                    //else _misses.push_back(in.get());
                }
            }
            traverse(node);
        }

        void apply(osg::ProxyNode& proxyNode)
        {
            if (_traverseProxies)
            {
                if (proxyNode.getStateSet())
                {
                    osg::ref_ptr<osg::StateSet> stateset = proxyNode.getStateSet();

                    if (isEligible(stateset.get()))
                    {
                        _stateSets++;
                        osg::ref_ptr<osg::StateSet> shared;
                        if (_cache->share(stateset, shared))
                        {
                            proxyNode.setStateSet(shared.get());
                            _shares++;
                        }
                        //else _misses.push_back(in.get());
                    }
                }
                traverse(proxyNode);
            }
        }
    };
}

//------------------------------------------------------------------------

StateSetCache::StateSetCache() :
    _pruneCount(0),
    _maxSize(DEFAULT_PRUNE_ACCESS_COUNT),
    _attrShareAttempts(0),
    _attrsIneligible(0),
    _attrShareHits(0),
    _attrShareMisses(0)
{
    //nop
}

StateSetCache::~StateSetCache()
{
    std::lock_guard<std::mutex> lock( _mutex );
    prune();
}

void
StateSetCache::releaseGLObjects(osg::State* state) const
{
    std::lock_guard<std::mutex> lock( _mutex );
    for(StateSetSet::const_iterator i = _stateSetCache.begin(); i != _stateSetCache.end(); ++i)
    {
        i->get()->releaseGLObjects(state);
    }

}

void
StateSetCache::setMaxSize(unsigned value)
{
    _maxSize = value;
    {
        std::lock_guard<std::mutex> lock( _mutex );
        pruneIfNecessary();
    }
}

void
StateSetCache::consolidateStateAttributes(osg::Node* node, bool traverseProxies)
{
    if ( !node )
        return;

    ShareStateAttributes v(this, traverseProxies);
    node->accept( v );
}

void
StateSetCache::consolidateStateSets(osg::Node* node, bool traverseProxies)
{
    if ( !node )
        return;

#ifdef STATESET_SHARING_SUPPORTED
    ShareStateSets v(this, traverseProxies);
    node->accept( v );
#endif
}

void
StateSetCache::optimize(osg::Node* node, bool traverseProxies)
{
    if ( node )
    {
        consolidateStateAttributes(node, traverseProxies);
        consolidateStateSets(node, traverseProxies);
    }
}


bool
StateSetCache::eligible(osg::StateSet* stateSet) const
{
    return isEligible(stateSet);
}


bool
StateSetCache::eligible(osg::StateAttribute* attr) const
{
    return isEligible(attr);
}


bool
StateSetCache::share(osg::ref_ptr<osg::StateSet>& input,
    osg::ref_ptr<osg::StateSet>& output,
    bool                         checkEligible)
{
    bool shared     = false;
    //bool shareattrs = true;

    if ( !checkEligible || eligible(input.get()) )
    {
        std::lock_guard<std::mutex> lock( _mutex );

        pruneIfNecessary();

        //shareattrs = false;

        std::pair<StateSetSet::iterator,bool> result = _stateSetCache.insert( input );
        if ( result.second )
        {
            // first use
            output = input.get();
            //shareattrs = true;
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

    // OBE.
    //if ( shareattrs )
    //{
    //    ShareStateAttributes sa(this);
    //    sa.applyStateSet( input.get() );
    //}

    return shared;
}



bool
StateSetCache::share(osg::ref_ptr<osg::StateAttribute>& input,
    osg::ref_ptr<osg::StateAttribute>& output,
    bool                               checkEligible)
{
    _attrShareAttempts++;

    if ( !checkEligible || eligible(input.get()) )
    {
        std::lock_guard<std::mutex> lock( _mutex );

        pruneIfNecessary();

        std::pair<StateAttributeSet::iterator,bool> result = _stateAttributeCache.insert( input );
        if ( result.second )
        {
            // first use
            output = input.get();
            _attrShareMisses++;
            return false;
        }
        else
        {
            // found a share!
            output = result.first->get();
            _attrShareHits++;
            return true;
        }
    }
    else
    {
        _attrsIneligible++;
        output = input.get();
        return false;
    }
}

void
StateSetCache::pruneIfNecessary()
{
    // assume an exclusve mutex is taken
    if ( _pruneCount++ >= _maxSize )
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
        if ( i->get()->referenceCount() <= 1 )
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
        if ( i->get()->referenceCount() <= 1 )
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

    OE_NULL << LC << "Pruned " << sa_count << " attributes, " << ss_count << " statesets" << std::endl;
}

void
StateSetCache::clear()
{
    std::lock_guard<std::mutex> lock( _mutex );

    prune();
    _stateAttributeCache.clear();
    _stateSetCache.clear();
}

void
StateSetCache::protect()
{
    std::lock_guard<std::mutex> lock( _mutex );
    for(auto i : _stateSetCache)
    {
        i->setDataVariance(osg::Object::DYNAMIC);
    }
}


void
StateSetCache::dumpStats()
{
    std::lock_guard<std::mutex> lock( _mutex );

    OE_NOTICE << LC << "StateSetCache Dump:" << std::endl
        << "    attr attempts     = " << _attrShareAttempts << std::endl
        << "    ineligibles attrs = " << _attrsIneligible << std::endl
        << "    attr share hits   = " << _attrShareHits << std::endl
        << "    attr share misses = " << _attrShareMisses << std::endl;
}
