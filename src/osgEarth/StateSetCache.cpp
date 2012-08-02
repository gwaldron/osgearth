/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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

#define LC "[StateSetCache] "

using namespace osgEarth;

//---------------------------------------------------------------------------

namespace
{
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
            if ( node.getStateSet() )
            {
                _stateSets++;
                osg::ref_ptr<osg::StateSet> in, out;
                in = node.getStateSet();
                if ( _cache->share(in, out) )
                {
                    node.setStateSet( out.get() );
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
                if ( d && d->getStateSet() )
                {
                    _stateSets++;
                    osg::ref_ptr<osg::StateSet> in, out;
                    in = d->getStateSet();
                    if ( _cache->share(in, out) )
                    {
                        d->setStateSet( out.get() );
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

void
StateSetCache::optimize(osg::Node* node)
{
    if ( node )
    {
        ShareStateSets visitor( this );
        node->accept( visitor );
    }
}

bool
StateSetCache::share(osg::ref_ptr<osg::StateSet>& input, 
                     osg::ref_ptr<osg::StateSet>& output )
{
    Threading::ScopedMutexLock lock( _mutex );

    std::pair<StateSetSet::iterator,bool> result = _stateSetCache.insert( input );
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
