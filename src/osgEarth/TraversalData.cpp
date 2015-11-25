/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2015 Pelican Mapping
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
#include <osgEarth/TraversalData>
#include <osgEarth/Registry>
#include <osg/ValueObject>

using namespace osgEarth;

void
TransientUserDataStore::store(osg::Referenced* owner, const std::string& key, osg::Referenced* data)
{
    if ( !owner ) return;

    _mutex.lock();

    // clear out orphaned data:
    for(Table::iterator i = _table.begin(); i != _table.end(); )
    {
        if ( !i->second._owner.valid() )
            _table.erase(i++);
        else
            ++i;
    }

    // insert new data:
    DataPair& p = _table[owner];
    p._owner = owner;
    p._data[key] = data;

    _mutex.unlock();
}

osg::Referenced*
TransientUserDataStore::fetch(osg::Referenced* owner, const std::string& key) const
{
    if ( !owner )
        return 0L;

    Threading::ScopedMutexLock lock(_mutex);
    Table::const_iterator i = _table.find(owner);
    if ( i == _table.end() || i->second._owner.valid() == false )
        return 0L;

    StringTable::const_iterator j = i->second._data.find(key);
    if ( j == i->second._data.end() )
        return 0L;

    return j->second.get();
}

bool
TransientUserDataStore::exists(osg::Referenced* owner, const std::string& key) const
{
    if ( !owner )
        return false;

    Threading::ScopedMutexLock lock(_mutex);
    Table::const_iterator i = _table.find(owner);
    if ( i == _table.end() || i->second._owner.valid() == false )
        return false;

    StringTable::const_iterator j = i->second._data.find(key);
    return ( j != i->second._data.end() );
}

bool
VisitorData::store(osg::NodeVisitor& nv, const std::string& key, osg::Referenced* data)
{
    Registry::instance()->dataStore().store( &nv, key, data );
    return true;
}

osg::Referenced*
VisitorData::_fetch(osg::NodeVisitor& nv, const std::string& key)
{
    return Registry::instance()->dataStore().fetch( &nv, key );
}

bool
VisitorData::isSet(osg::NodeVisitor& nv, const std::string& key)
{
    return Registry::instance()->dataStore().exists( &nv, key );
}
