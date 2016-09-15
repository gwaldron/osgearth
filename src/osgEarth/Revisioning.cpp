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
#include <osgEarth/Revisioning>
#include <algorithm>

#define LC "[Revisioning] "

using namespace osgEarth;


//------------------------------------------------------------------------


DirtyNotifier::DirtyNotifier()
{
    _counter = new DirtyCounter(this);
}


void
DirtyNotifier::addParent( DirtyNotifier* parent )
{
    if ( parent )
    {
        _parents.push_back( parent->_counter.get() );
        if ( isDirty() )
            parent->setDirty();
    }
}


void
DirtyNotifier::removeParent( DirtyNotifier* parent )
{
    for( std::vector< osg::observer_ptr<DirtyCounter> >::iterator i = _parents.begin(); i != _parents.end(); )
    {
        if ( i->valid() )
        {
            if ( i->get()->_owner == parent )
            {
                i = _parents.erase( i );
            }
            else
            {
                ++i;
            }
        }
        else
        {
            i = _parents.erase( i );
        }
    }
}


void
DirtyNotifier::setDirty()
{
    _counter->_count++;

    if ( _parents.size() > 0 )
    {
        for( std::vector< osg::observer_ptr<DirtyCounter> >::iterator i = _parents.begin(); i != _parents.end(); )
        {
            osg::ref_ptr<DirtyCounter> parent;
            if ( i->lock(parent) )
            {
                parent->_owner->setDirty();
                ++i;
            }
            else
            {
                i = _parents.erase( i );
            }
        }
    }
}
