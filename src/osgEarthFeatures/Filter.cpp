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
#include <osgEarthFeatures/Filter>

using namespace osgEarth;
using namespace osgEarthFeatures;

FilterContext::FilterContext()
{
    //NOP
}

FilterContext::FilterContext(const FilterContext& rhs) :
_extent( rhs._extent ),
_profile( rhs._profile )
{
    //NOP
}

//template<typename T>
//FilterChain<T>::FilterChain( const FilterContext& context ) :
//_prototypeContext( context )
//{
//    //NOP
//}
//template<typename T>
//FilterChain<T>::FilterChain( const FilterChain& rhs ) :
//_prototypeContext( rhs._prototypeContext )
//{
//    //NOP
//}
//
//template<typename T>
//bool
//FilterChain<T>::run(FeatureCursor* inputCursor,
//                    std::list< osg::ref_ptr<T> >& output )
//{
//    while( inputCursor->hasMore() )
//    {
//        osg::Referenced* obj = inputCursor->nextFeature();
//        if ( obj )
//        {
//            for( std::list< osg::ref_ptr<Filter> >::iterator k = begin(); k != end(); k++ )
//            {
//                FilterContext context( _prototypeContext );
//                obj = k->get()->process( obj, context );
//                if ( !obj.valid() )
//                    break;
//            }
//
//            if ( obj.valid() )
//                output.push_back( obj.get() );
//        }
//    }
//}
//
//
//template<typename T>
//bool
//FilterChain<T>::run(std::list< osg::ref_ptr<osg::Referenced> >& input,
//                    std::list< osg::ref_ptr<T> >& output)
//{
//    for( std::list< osg::ref_ptr<T> >::iterator i = begin(); i != end(); i++ )
//    {
//        osg::ref_ptr< osg::Referenced > input = i->get();
//
//        for( std::list< osg::ref_ptr<Filter> >::iterator k = begin(); k != end(); k++ )
//        {
//            FilterContext context( _prototypeContext );
//            input = k->get()->process( input.get(), context );
//            if ( !input.valid() ) break;
//        }
//
//        if ( input.valid() )
//            output.push_back( input.get() );
//    }
//}
//
