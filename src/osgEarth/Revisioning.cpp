/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/Revisioning>

#define LC "[Revisioning] "

using namespace osgEarth::Util;


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
