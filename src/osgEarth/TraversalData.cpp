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
#include <osgEarth/TraversalData>
#include <osgEarth/Registry>
#include <osg/ValueObject>

using namespace osgEarth;

bool
TransientUserDataStore::isObserverInvalid(const Table::entry_t& p)
{
    return p.second._owner.valid() == false;
}

int
TransientUserDataStore::size() const
{
    return _table.size();
}

void
TransientUserDataStore::store(osg::Referenced* owner, const std::string& key, osg::Referenced* data)
{
    if ( !owner ) return;

    _mutex.lock();

    // clear out orphaned data:
    _table.erase( std::remove_if(_table.begin(), _table.end(), isObserverInvalid), _table.end() );

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
TransientUserDataStore::unitTest()
{
  osgEarth::TransientUserDataStore tuds;
  osg::ref_ptr<osg::Referenced> owner1 = new osg::Referenced;
  osg::ref_ptr<osg::Referenced> owner2 = new osg::Referenced;
  osg::ref_ptr<osg::Referenced> owner3 = new osg::Referenced;
  osg::ref_ptr<osg::Referenced> data1 = new osg::Referenced;
  osg::ref_ptr<osg::Referenced> data2 = new osg::Referenced;
  osg::ref_ptr<osg::Referenced> data3 = new osg::Referenced;
  tuds.store(owner1, "foo", data1);
  tuds.store(owner1, "foo", data2);
  tuds.store(owner1, "foo", data1);
  data1 = NULL;
  tuds.store(owner1, "foo", data2);
  data2 = new osg::Referenced;
  tuds.store(owner1, "foo", data3);
  tuds.store(owner1, "foo", data2);
  data1 = new osg::Referenced;
  owner1 = owner2;
  tuds.store(owner1, "foo", data3);
  tuds.store(owner1, "foo", data1);
  tuds.store(owner3, "foo", data1);
  owner3 = NULL;
  tuds.store(owner1, "foo", data2);
  tuds.store(owner2, "foo", data2);
  owner1 = owner2 = owner3 = NULL;
  owner1 = new osg::Referenced;
  tuds.store(owner1, "foo", data3);
  return true;
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

