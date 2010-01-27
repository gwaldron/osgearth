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
#include <osgEarthFeatures/ResampleFilter>
#include <list>
#include <deque>
#include <cstdlib>

using namespace osgEarth;
using namespace osgEarth::Features;

bool
ResampleFilter::isSupported()
{
    return true;
}

ResampleFilter::ResampleFilter() :
_minLen( 0 ),
_maxLen( DBL_MAX ),
_perturbThresh( 0 )
{
    //NOP
}

ResampleFilter::ResampleFilter( double minLen, double maxLen ) :
_minLen( minLen ),
_maxLen( maxLen ),
_perturbThresh( 0 )
{
    // NOP
}

bool
ResampleFilter::push( Feature* input, const FilterContext& context )
{
    if ( !input || !input->getGeometry() )
        return true;

    bool success = true;

    GeometryIterator i( input->getGeometry() );
    while( i.hasMore() )
    {
        Geometry* part = i.next();

        if ( part->size() < 3 ) continue;
        int partSize0 = part->size();

        // copy the original part to a linked list. use a std::list since insert/erase
        // will not invalidate iterators.
        std::list<osg::Vec3d> plist;
        plist.insert( plist.begin(), part->begin(), part->end() );

        std::list<osg::Vec3d>::iterator v1 = plist.begin(); ++v1;
        std::list<osg::Vec3d>::iterator v0 = plist.begin();
        std::list<osg::Vec3d>::iterator last = plist.end(); --last;

        while( v0 != last )
        {
            bool increment = true;

            osg::Vec3d& p0 = *v0;
            osg::Vec3d& p1 = *v1;
            bool lastSeg = v1 == last;
            osg::Vec3d seg = p1 - p0;
            double segLen = seg.length();

            if ( segLen < _minLen && !lastSeg && plist.size() > 3 )
            {
                v1 = plist.erase( v1 );
                increment = false;
            }
            else if ( segLen > _maxLen )
            {
                int numDivs = (1 + (int)(segLen/_maxLen));
                double newSegLen = segLen/(double)numDivs;
                seg.normalize();
                osg::Vec3d newPt = p0 + seg * newSegLen;
                if ( _perturbThresh > 0.0 && _perturbThresh < newSegLen )
                {
                    float r = 0.5 - (float)::rand()/(float)RAND_MAX;
                    newPt.x() += r;
                    newPt.y() += r;
                }
                v1 = plist.insert( v1, p0 + seg * newSegLen );
            }

            if ( increment ) { ++v0; ++v1; }
        }

        part->clear();
        part->reserve( plist.size() );
        part->insert( part->begin(), plist.begin(), plist.end() );

        //if ( partSize0 != part->size() )
        //{
        //    osg::notify(osg::NOTICE) << "Resampled part from " << partSize0 << " to " << part->size() << " points" << std::endl;
        //}
    }
    return success;
}


FilterContext
ResampleFilter::push( FeatureList& input, const FilterContext& context )
{
    if ( !isSupported() )
    {
        osg::notify(osg::NOTICE) << "[osgEarth] ResampleFilter support not enabled" << std::endl;
        return context;
    }

    bool ok = true;
    for( FeatureList::iterator i = input.begin(); i != input.end(); ++i )
        if ( !push( i->get(), context ) )
            ok = false;

    return context;
}
