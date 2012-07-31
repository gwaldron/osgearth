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

#include <osgEarthFeatures/Session>
#include <osgEarthFeatures/Script>
#include <osgEarthFeatures/ScriptEngine>
#include <osgEarthFeatures/FeatureSource>
#include <osgEarth/FileUtils>
#include <osgEarth/HTTPClient>
#include <osgEarth/StringUtils>
#include <osg/AutoTransform>
#include <osg/Depth>
#include <osg/TextureRectangle>

#define LC "[Session] "

using namespace osgEarth;
using namespace osgEarth::Features;

//---------------------------------------------------------------------------

namespace
{
    /**
     * Visitor that calls Session::getSharedStateSet on all statesets found
     * in a scene graph.
     */
    struct ShareStateSets : public osg::NodeVisitor
    {
        Session* _session;
        unsigned _stateSets;
        unsigned _shares;
        //std::vector<osg::StateSet*> _misses; // for debugging

        ShareStateSets(Session* session) :
            osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ),
            _session  ( session ),
            _stateSets( 0 ),
            _shares   ( 0 ) { }

        void apply(osg::Node& node)
        {
            if ( node.getStateSet() )
            {
                _stateSets++;
                osg::ref_ptr<osg::StateSet> in, out;
                in = node.getStateSet();
                if ( _session->getSharedStateSet(in, out) )
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
                    if ( _session->getSharedStateSet(in, out) )
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

//---------------------------------------------------------------------------

Session::Session( const Map* map, StyleSheet* styles, FeatureSource* source, const osgDB::Options* dbOptions ) :
osg::Referenced( true ),
_map           ( map ),
_mapInfo       ( map ),
_featureSource ( source ),
_dbOptions     ( dbOptions )
{
    if ( styles )
        setStyles( styles );
    else
        _styles = new StyleSheet();

    // if the caller did not provide a dbOptions, take it from the map.
    if ( map && !dbOptions )
        _dbOptions = map->getDBOptions();
}

Session::~Session()
{
}

const osgDB::Options*
Session::getDBOptions() const
{
    return _dbOptions.get();
}

MapFrame
Session::createMapFrame( Map::ModelParts parts ) const
{
    return MapFrame( _map.get(), parts );
}

void
Session::removeObject( const std::string& key )
{
    Threading::ScopedMutexLock lock( _objMapMutex );
    //Threading::ScopedWriteLock lock( _objMapMutex );
    _objMap.erase( key );
}

void
Session::setStyles( StyleSheet* value )
{
    _styles = value ? value : new StyleSheet();

    // Go ahead and create the script engine for the StyleSheet
    if (_styles && _styles->script())
      _styleScriptEngine = ScriptEngineFactory::create(Script(_styles->script()->code, _styles->script()->language, _styles->script()->name));
    else
      _styleScriptEngine = 0L;
}

ScriptEngine*
Session::getScriptEngine() const
{
  return _styleScriptEngine.get();
}

FeatureSource*
Session::getFeatureSource() const 
{ 
	return _featureSource.get(); 
}


bool
Session::getSharedStateSet( osg::ref_ptr<osg::StateSet>& input, osg::ref_ptr<osg::StateSet>& output )
{
    Threading::ScopedMutexLock lock( _objMapMutex );

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


void
Session::shareStateSets( osg::Node* graph )
{
    if ( graph )
    {
        ShareStateSets visitor( this );
        graph->accept( visitor );
        //OE_WARN << LC << "Shares = " << visitor._shares << "/" << visitor._stateSets << ", cache = " << _stateSetCache.size() << std::endl;
    }
}
