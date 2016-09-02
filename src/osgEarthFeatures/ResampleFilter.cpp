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
#include <osgEarthFeatures/ResampleFilter>
#include <osgEarth/GeoMath>
#include <osg/io_utils>
#include <list>
#include <deque>
#include <cstdlib>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

OSGEARTH_REGISTER_SIMPLE_FEATUREFILTER(resample, ResampleFilter );

bool
ResampleFilter::isSupported()
{
    return true;
}

ResampleFilter::ResampleFilter() :
ResampleFilterOptions()
{
    //NOP
}

ResampleFilter::ResampleFilter( double minLen, double maxLen ) :
ResampleFilterOptions()
{
    _minLen = minLen;
    _maxLen = maxLen;
}

ResampleFilter::ResampleFilter( const Config& conf ):
ResampleFilterOptions( conf )
{
    //nop
}


bool
ResampleFilter::push( Feature* input, FilterContext& context )
{
    if ( !input || !input->getGeometry() )
        return true;

    bool success = true;

    GeometryIterator i( input->getGeometry() );
    while( i.hasMore() )
    {        
        Geometry* part = i.next();

        if ( part->size() < 2 ) continue;

        unsigned int origSize = part->size();

        // whether to resample the virtual segment that closes an open ring:
        bool loop =
            part->getComponentType() == Geometry::TYPE_RING &&
            static_cast<Ring*>(part)->isOpen();

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

            osg::Vec3d p0Rad, p1Rad;

            if (resampleMode().value() == RESAMPLE_GREATCIRCLE || resampleMode().value() == RESAMPLE_RHUMB)
            {
                p0Rad = osg::Vec3d(osg::DegreesToRadians(p0.x()), osg::DegreesToRadians(p0.y()), p0.z());
                p1Rad = osg::Vec3d(osg::DegreesToRadians(p1.x()), osg::DegreesToRadians(p1.y()), p1.z());
            }
                       
            //Compute the length of the segment
            double segLen = 0.0;
            switch (resampleMode().value())
            {
            case RESAMPLE_LINEAR:
                segLen = seg.length();
                break;
            case RESAMPLE_GREATCIRCLE:
                segLen = GeoMath::distance(p0Rad.y(), p0Rad.x(), p1Rad.y(), p1Rad.x());
                break;
            case RESAMPLE_RHUMB:
                segLen = GeoMath::rhumbDistance(p0Rad.y(), p0Rad.x(), p1Rad.y(), p1Rad.x());
                break;
            }

            if ( segLen < _minLen.value() && !lastSeg && plist.size() > 2 )
            {
                v1 = plist.erase( v1 );
                increment = false;
            }
            else if ( segLen > _maxLen.value() )
            {
                //Compute the number of divisions to make
                int numDivs = (1 + (int)(segLen/_maxLen.value()));
                double newSegLen = segLen/(double)numDivs;
                seg.normalize();
                osg::Vec3d newPt;
                double newHeight;
                switch (resampleMode().value())
                {
                case RESAMPLE_LINEAR:
                    {
                        newPt = p0 + seg * newSegLen;
                    }
                    break;
                case RESAMPLE_GREATCIRCLE:
                    {
                        double bearing = GeoMath::bearing(p0Rad.y(), p0Rad.x(), p1Rad.y(), p1Rad.x());
                        double lat,lon;
                        GeoMath::destination(p0Rad.y(), p0Rad.x(), bearing, newSegLen, lat, lon);
                        newHeight = p0Rad.z() + ( p1Rad.z() - p0Rad.z() ) / (double)numDivs;
                        newPt = osg::Vec3d(osg::RadiansToDegrees(lon), osg::RadiansToDegrees(lat), newHeight);
                    }
                    break;
                case RESAMPLE_RHUMB:
                    {
                        double bearing = GeoMath::rhumbBearing(p0Rad.y(), p0Rad.x(), p1Rad.y(), p1Rad.x());
                        double lat,lon;
                        GeoMath::rhumbDestination(p0Rad.y(), p0Rad.x(), bearing, newSegLen, lat, lon);
                        newHeight = p0Rad.z() + ( p1Rad.z() - p0Rad.z() ) / (double)numDivs;
                        newPt = osg::Vec3d(osg::RadiansToDegrees(lon), osg::RadiansToDegrees(lat), newHeight);
                    }
                    break;
                }
                
                if ( _perturbThresh.value() > 0.0 && _perturbThresh.value() < newSegLen )
                {
                    float r = 0.5 - (float)::rand()/(float)RAND_MAX;
                    newPt.x() += r;
                    newPt.y() += r;
                }
                v1 = plist.insert( v1, newPt );
            }

            if ( increment ) { ++v0; ++v1; }
        }

        part->clear();
        part->reserve( plist.size() );
        part->insert( part->begin(), plist.begin(), plist.end() );
    }
    return success;
}


FilterContext
ResampleFilter::push( FeatureList& input, FilterContext& context )
{
    if ( !isSupported() )
    {
        OE_WARN << "ResampleFilter support not enabled" << std::endl;
        return context;
    }

    bool ok = true;
    for( FeatureList::iterator i = input.begin(); i != input.end(); ++i )
        if ( !push( i->get(), context ) )
            ok = false;

    return context;
}
