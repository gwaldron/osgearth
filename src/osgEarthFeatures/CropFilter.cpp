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
#include <osgEarthFeatures/CropFilter>
#include <osg/Notify>
#include <stack>

using namespace osgEarth;
using namespace osgEarthFeatures;

#define UINT int
#define UINTList std::vector<int>

static bool
findIsectSegmentAndPoint(const osg::Vec3d& p1,
                         const osg::Vec3d& p2,
                         int               dim,
                         const osg::Vec3Array* input,
                         const UINT  start_i,
                         int&        out_seg_i,
                         osg::Vec3d& out_isect_p )
{
    for( UINT i=0; i<input->size()-1; i++ )
    {
        // ignore the segments preceding and following the start index
        if ( (input->size()+start_i-1)%input->size() <= i && i <= (input->size()+start_i+1)%input->size() )
            continue;

        const osg::Vec3d& p3 = (*input)[i];
        const osg::Vec3d& p4 = (*input)[i+1];

        double denom = (p4.y()-p3.y())*(p2.x()-p1.x()) - (p4.x()-p3.x())*(p2.y()-p1.y());
        if ( denom != 0.0 )
        {
            double ua_num = (p4.x()-p3.x())*(p1.y()-p3.y()) - (p4.y()-p3.y())*(p1.x()-p3.x());
            double ub_num = (p2.x()-p1.x())*(p1.y()-p3.y()) - (p2.y()-p1.y())*(p1.x()-p3.x());

            double ua = ua_num/denom;
            double ub = ub_num/denom;

            if ( ua <= 1.0 ) {
                //osgGIS::notify( osg::WARN ) 
                //    << " [" 
                //    << i << "] "
                //    << "isect point was found at on source segment (ua=" << ua << ")" 
                //    << std::endl
                //    << "  source segment = (" << p1.toString() << " => " << p2.toString() << "), "
                //    << "  target segment = (" << p3.toString() << " => " << p4.toString() << ")"
                //    << std::endl;
            }

            else if ( ub < 0.0 || ub > 1.0 ) 
            {
                //osgGIS::notify( osg::WARN ) 
                //    << " [" 
                //    << i << "] "
                //    << "isect point was not found on target segment (ub=" << ub << ")" 
                //    << std::endl
                //    << "  source segment = (" << p1.toString() << " => " << p2.toString() << "), "
                //    << "  target segment = (" << p3.toString() << " => " << p4.toString() << ")"
                //    << std::endl;
            }
            else
            {
                double isect_x = p1.x() + ua*(p2.x()-p1.x());
                double isect_y = p1.y() + ua*(p2.y()-p1.y());
                out_seg_i = i;
                out_isect_p.set( isect_x, isect_y, p4.z() );
                return true;
            }
        }
    }
    return false;
}
static bool
pointInsideOrOnLine( const osg::Vec3d& p, const osg::Vec3d& s1, const osg::Vec3d& s2 )
{
    osg::Vec3d cross = (s2-s1) ^ (p-s1);
    return cross.z() >= 0.0;
}

static bool
extentInsideOrOnLine( const GeoExtent& p, const osg::Vec3d& s1, const osg::Vec3d& s2 )
{
    return 
        pointInsideOrOnLine( osg::Vec3d( p.xMin(), p.yMin(), 0 ), s1, s2 ) &&
        pointInsideOrOnLine( osg::Vec3d( p.xMax(), p.yMax(), 0 ), s1, s2 );
}

static bool
getIsectPoint(const osg::Vec3d& p1, const osg::Vec3d& p2, 
              const osg::Vec3d& p3, const osg::Vec3d& p4,
              osg::Vec3d& out,
              int dim =2 )
{
    double denom = (p4.y()-p3.y())*(p2.x()-p1.x()) - (p4.x()-p3.x())*(p2.y()-p1.y());
    if ( denom == 0.0 )
    {
        //out = GeoPoint::invalid(); // parallel lines
        return false;
    }

    double ua_num = (p4.x()-p3.x())*(p1.y()-p3.y()) - (p4.y()-p3.y())*(p1.x()-p3.x());
    double ub_num = (p2.x()-p1.x())*(p1.y()-p3.y()) - (p2.y()-p1.y())*(p1.x()-p3.x());

    double ua = ua_num/denom;
    double ub = ub_num/denom;

    if ( ua < 0.0 || ua > 1.0 ) // || ub < 0.0 || ub > 1.0 )
    {
        //out = GeoPoint::invalid(); // isect point is on line, but not on line segment
        return false;
    }

    double x = p1.x() + ua*(p2.x()-p1.x());
    double y = p1.y() + ua*(p2.y()-p1.y());
    double z = dim > 2? p1.z() + ua*(p2.z()-p1.z()) : 0.0; //right?
    out.set( x, y, z );
    return true;
}

#define STAGE_SOUTH 0
#define STAGE_EAST  1
#define STAGE_NORTH 2
#define STAGE_WEST  3

static void
spatialInsert( UINTList& list, UINT value, UINT stage, const osg::Vec3dArray* points )
{
    UINTList::iterator i;
    double x, y;
    
    switch( stage )
    {
    case STAGE_SOUTH:
        x = (*points)[value].x();
        for( i = list.begin(); i != list.end() && x < (*points)[*i].x(); i++ );
        list.insert( i, value );
        break;

    case STAGE_EAST:
        y = (*points)[value].y();
        for( i = list.begin(); i != list.end() && y < (*points)[*i].y(); i++ );
        list.insert( i, value );
        break;

    case STAGE_NORTH:
        x = (*points)[value].x();
        for( i = list.begin(); i != list.end() && x > (*points)[*i].x(); i++ );
        list.insert( i, value );
        break;

    case STAGE_WEST:
        y = (*points)[value].y();
        for( i = list.begin(); i != list.end() && y > (*points)[*i].y(); i++ );
        list.insert( i, value );
        break;
    }
}

static UINT
findIndexOf( const UINTList& list, UINT value )
{
    for( UINT i=0; i<list.size(); i++ )
        if ( list[i] == value )
            return i;
    return 0;
}

class DeferredPart {
public:
    DeferredPart( osg::Vec3dArray* a, UINT b, UINT c )
        : part( a ), 
          part_start_ptr( b ),
          waiting_on_ptr( c ) { }
    osg::ref_ptr<osg::Vec3dArray> part;
    UINT part_start_ptr;
    UINT waiting_on_ptr;
};

static void
scrubPart( osg::Vec3dArray* part )
{
    while( part->size() >= 0 && part->front() == part->back() )
        part->erase( part->end()-1 );
}

static bool
extentContainsAllPoints( const GeoExtent& extent, osg::Vec3dArray* points )
{
    for( osg::Vec3dArray::iterator i = points->begin(); i != points->end(); i++ )
    {
        if ( !extent.contains( (*i).x(), (*i).y() ) )
            return false;
    }
    return true;
}

// poly clipping algorithm Aug 2007
static bool
cropPolygonPart( osg::Vec3dArray* initial_input, const GeoExtent& window, FeatureGeometry& final_outputs )
{
    // trivial rejection for a non-polygon:
    if ( initial_input->size() < 3 )
    {
        return false;
    }

    // check for trivial acceptance.
    if ( extentContainsAllPoints( window, initial_input ) )
    {
        final_outputs.push_back( initial_input );
        return true;
    }

    // prepare the list of input parts to process.
    FeatureGeometry inputs;
    inputs.push_back( initial_input );

    // run the algorithm against all four sides of the window in succession.
    for( UINT stage = 0; stage < 4; stage++ )
    {
        osg::Vec3d s1, s2;
        switch( stage )
        {
            case STAGE_SOUTH:
                s1.set( window.xMin(), window.yMin(), 0 );
                s2.set( window.xMax(), window.yMin(), 0 );
                break;                
            case STAGE_EAST:
                s1.set( window.xMax(), window.yMin(), 0 );
                s2.set( window.xMax(), window.yMax(), 0 );
                break;
            case STAGE_NORTH: 
                s1.set( window.xMax(), window.yMax(), 0 );
                s2.set( window.xMin(), window.yMax(), 0 );
                break;
            case STAGE_WEST:
                s1.set( window.xMin(), window.yMax(), 0 );
                s2.set( window.xMin(), window.yMin(), 0 );
                break;
        }

        // output parts to send to the next stage (or to return).
        FeatureGeometry outputs;

        // run against each input part.
        for( FeatureGeometry::iterator i = inputs.begin(); i != inputs.end(); i++ )
        {
            //GeoPointList& input = *i;
            osg::Vec3dArray* input = i->get(); //*i;

            scrubPart( input );

            // trivially reject a degenerate part (should never happen ;)
            if ( input->size() < 3 )
            {
                continue;
            }
            
            // trivially accept if the window contains the entire extent of the part:
            GeoExtent input_extent;
            for( osg::Vec3dArray::iterator k = input->begin(); k != input->end(); k++ )
            {
                input_extent.expandToInclude( k->x(), k->y() );
            }

            // trivially accept when the part lies entirely within the line:
            if ( extentInsideOrOnLine( input_extent, s1, s2 ) )
            {
                outputs.push_back( input );
                continue;
            }

            // trivially reject when there's no overlap:
            if ( !window.intersects( input_extent ) || extentInsideOrOnLine( input_extent, s2, s1 ) )
            {
                continue;
            }                

            // close the part in preparation for cropping. The cropping process with undo
            // this automatically.
            input->push_back( input->front() );

            // 1a. Traverse the part and find all intersections. Insert them into the input shape.
            // 1b. Create a traversal-order list, ordering the isect points in the order in which they are encountered.
            // 1c. Create a spatial-order list, ordering the isect points along the boundary segment in the direction of the segment.
            osg::ref_ptr<osg::Vec3dArray> working;
            UINTList traversal_order;
            UINTList spatial_order;
            osg::Vec3d prev_p;
            bool was_inside = true;

            for( UINT input_i = 0; input_i < input->size(); input_i++ )
            {
                const osg::Vec3d& p = (*input)[ input_i ];
                bool is_inside = pointInsideOrOnLine( p, s1, s2 );

                if ( input_i > 0 )
                {
                    if ( was_inside != is_inside ) // entering or exiting
                    {
                        osg::Vec3d isect_p;
                        if ( getIsectPoint( prev_p, p, s1, s2, /*out*/ isect_p ) )
                        {
                            working->push_back( isect_p );
                            traversal_order.push_back( working->size()-1 );
                            spatialInsert( spatial_order, working->size()-1, stage, working );
                        }
                        else
                        {
                            osg::notify( osg::WARN ) << "getIsectPoint failed" << std::endl;
                        }
                    }
                }

                working->push_back( p );
                prev_p = p;
                was_inside = is_inside;
            }

            if ( spatial_order.size() == 0 )
            {
                outputs.push_back( input );
                continue;
            }

            // 2. Start at the point preceding the first isect point (in spatial order). This will be the first
            //    outside point (since the first isect point is always an ENTRY.
            UINT overall_start_ptr = spatial_order[0];
            UINT shape_ptr = overall_start_ptr;

            // initialize the isect traversal pointer to start at the spatial order's first isect point:
            UINT trav_ptr = findIndexOf( traversal_order, spatial_order[0] );
            UINT traversals = 0;

            std::stack<DeferredPart> part_stack;
            osg::ref_ptr<osg::Vec3dArray> current_part;

            // Process until we make it all the way around.
            while( traversals < traversal_order.size() )
            {
                // 4. We are outside. Find the next ENTRY point and either start a NEW part, or RESUME
                //    a previously deferred part that is on top of the stack.
                shape_ptr = traversal_order[trav_ptr]; // next ENTRY
                trav_ptr = (trav_ptr + 1) % traversal_order.size();
                traversals++;

                UINT part_start_ptr = shape_ptr;

                if ( part_stack.size() > 0 && part_stack.top().waiting_on_ptr == part_start_ptr )
                {
                    // time to resume a part that we deferred earlier.
                    DeferredPart& top = part_stack.top();
                    current_part = top.part.get();
                    part_start_ptr = top.part_start_ptr;
                    part_stack.pop();
                }
                else
                {
                    // start a new part
                    current_part = new osg::Vec3dArray(); //GeoPointList();
                }

                // 5. Traverse to the next EXIT, adding all points along the way. 
                //    Then check the spatial order of the EXIT against the part's starting point. If the former
                //    is ONE MORE than the latter, close out the part.
                for( bool part_done = false; !part_done && traversals < traversal_order.size(); )
                {
                    UINT next_exit_ptr = traversal_order[trav_ptr];
                    trav_ptr = (trav_ptr + 1) % traversal_order.size();
                    traversals++;
                
                    for( ; shape_ptr != next_exit_ptr; shape_ptr = (shape_ptr+1) % working->size() )
                    {
                        current_part->push_back( (*working)[shape_ptr] );
                    }

                    // record the exit point:
                    current_part->push_back( (*working)[next_exit_ptr] );

                    UINT part_start_order = findIndexOf( spatial_order, part_start_ptr );
                    UINT next_exit_order = findIndexOf( spatial_order, next_exit_ptr );
                    if ( next_exit_order - part_start_order == 1 )
                    {
                        outputs.push_back( current_part.get() );
                        part_done = true;
                        continue;
                    }

                    // 6. Find the next ENTRY. If the spatial order of the ENTRY is one less than the 
                    //    spatial ordering of the preceding EXIT, continue on with the part.
                    UINT next_entry_ptr = traversal_order[trav_ptr];
                    trav_ptr = (trav_ptr + 1) % traversal_order.size();
                    traversals++;

                    // check whether we are back at the beginning:
                    if ( traversals >= traversal_order.size() )
                    {
                        current_part->push_back( (*working)[next_entry_ptr] );
                        continue;
                    }

                    // check whether we are continuing the current part:
                    UINT next_entry_order = findIndexOf( spatial_order, next_entry_ptr );
                    if ( next_exit_order - next_entry_order == 1 )
                    {
                        shape_ptr = next_entry_ptr; // skip ahead to the entry point
                        continue;
                    }

                    // 7. We encountered an out-of-order traversal, so need to push the current part onto
                    //    the deferral stack until later, and start a new part.
                    part_stack.push( DeferredPart(
                        current_part.get(),
                        part_start_ptr,
                        spatial_order[next_exit_order-1] ) );

                    current_part = new osg::Vec3dArray();
                    //current_part.push_back( working[next_entry_ptr] );
                    part_start_ptr = next_entry_ptr;
                    shape_ptr = next_entry_ptr;
                }
            }

            // pop any parts left on the stack (they're complete now)
            while( part_stack.size() > 0 )
            {
                osg::Vec3dArray* part = part_stack.top().part.get();
                //GeoPointList& part = part_stack.top().part;
                part->push_back( (*working)[part_stack.top().waiting_on_ptr] );
                outputs.push_back( part );
                part_stack.pop();
            }
        }

        // set up for next iteration
        outputs.swap( inputs );
    }

    // go through and make sure no polys are "closed" (probably unnecessary).
    //for( GeoPartList::iterator k = inputs.begin(); k != inputs.end(); k++ )
    //{
    //    while ( k->size() > 3 && k->front() == k->back() )
    //        k->erase( k->end()-1 );
    //}

    final_outputs.swap( inputs );
    return true;
}

static bool
cropLinePart( osg::Vec3dArray* part, const GeoExtent& extent, FeatureGeometry& output )
{
    FeatureGeometry working_parts;
    working_parts.push_back( part );

    for( UINT stage=0; stage<4; stage++ )
    {
        FeatureGeometry new_parts;

        osg::Vec3d s1, s2;
        switch( stage )
        {
            case STAGE_SOUTH:
                s1.set( extent.xMin(), extent.yMin(), 0 );
                s2.set( extent.xMax(), extent.yMin(), 0 );
                break;                
            case STAGE_EAST:
                s1.set( extent.xMax(), extent.yMin(), 0 );
                s2.set( extent.xMax(), extent.yMax(), 0 );
                break;
            case STAGE_NORTH: 
                s1.set( extent.xMax(), extent.yMax(), 0 );
                s2.set( extent.xMin(), extent.yMax(), 0 );
                break;
            case STAGE_WEST:
                s1.set( extent.xMin(), extent.yMax(), 0 );
                s2.set( extent.xMin(), extent.yMin(), 0 );
                break;
            //case 0: s1 = extent.getSouthwest(), s2 = extent.getSoutheast(); break;
            //case 1: s1 = extent.getSoutheast(), s2 = extent.getNortheast(); break;
            //case 2: s1 = extent.getNortheast(), s2 = extent.getNorthwest(); break;
            //case 3: s1 = extent.getNorthwest(), s2 = extent.getSouthwest(); break;
        }

        for( UINT part_i = 0; part_i < working_parts.size(); part_i++ )
        {
            bool inside = true;
            osg::Vec3d prev_p;
            bool first = true;
            osg::ref_ptr<osg::Vec3dArray> current_part;

            osg::ref_ptr<osg::Vec3dArray> working_part = working_parts[ part_i ];
            for( UINT point_i = 0; point_i < working_part->size(); point_i++ )
            {
                const osg::Vec3d& p = (*working_part)[ point_i ];

                if ( pointInsideOrOnLine( p, s1, s2 ) )
                {
                    if ( inside )
                    {
                        current_part->push_back( p );
                    }
                    else if ( !first )
                    {
                        osg::Vec3d isect_p;
                        if ( getIsectPoint( prev_p, p, s1, s2, /*out*/ isect_p ) )
                        {
                            current_part->push_back( isect_p );
                            current_part->push_back( p );
                        }
                    }
                    inside = true;
                }
                else // outside.
                {
                    if ( inside && point_i > 0 ) // inside heading out
                    {
                        osg::Vec3d isect_p;
                        if ( getIsectPoint( prev_p, p, s1, s2, /*out*/ isect_p ) )
                        {
                            current_part->push_back( isect_p );
                            if ( current_part->size() > 1 )
                                new_parts.push_back( current_part.get() );
                            current_part = new osg::Vec3dArray();
                        }
                    }
                    inside = false;
                }

                prev_p = p;
                first = false;
            }

            if ( current_part->size() > 1 )
                new_parts.push_back( current_part.get() );
        }

        working_parts.swap( new_parts );
    }

    output.swap( working_parts );
    return true;
}

static bool
cropPointPart( const osg::Vec3dArray* part, const GeoExtent& extent, FeatureGeometry& output )
{
    osg::Vec3dArray* new_part = new osg::Vec3dArray();
    for( UINT i=0; i<part->size(); i++ )
    {
        if ( extent.contains( (*part)[i].x(), (*part)[i].y() ) )
            new_part->push_back( (*part)[i] );
    }
    output.push_back( new_part );
    return true;
}

static bool
cropPart(osg::Vec3dArray* part, const GeoExtent& extent,
         const FeatureProfile::GeometryType& type, FeatureGeometry& cropped_parts )
{
    FeatureGeometry outputs;

    if ( type == FeatureProfile::GEOM_POLYGON )
    {
        cropPolygonPart( part, extent, outputs );
    }
    else if ( type == FeatureProfile::GEOM_LINE )
    {
        cropLinePart( part, extent, outputs );
    }
    else if ( type == FeatureProfile::GEOM_POINT )
    {
        cropPointPart( part, extent, outputs );
    }
    else
    {
        // unsupported type..?
        return false; 
    }
    
    cropped_parts.insert( cropped_parts.end(), outputs.begin(), outputs.end() );
    return true;
}

/****************************************************************************/

CropFilter::CropFilter(const GeoExtent& cropExtent) :
_cropExtent( cropExtent )
{
    //NOP
}

bool
CropFilter::push( Feature* input, FilterContext& context )
{
    FeatureGeometry newParts;

    for( int p=0; p<input->getNumParts(); p++ )
    {
        osg::Vec3dArray* part = input->getPart( p );
        cropPart( part, _cropExtent, context._profile->getGeometryType(), newParts );
    }

    input->setGeometry( newParts );

    return true;
}

bool
CropFilter::push( FeatureList& input, FilterContext& context )
{
    bool ok = true;
    for( FeatureList::iterator i = input.begin(); i != input.end(); i++ )
        if ( !push( i->get(), context ) )
            ok = false;
    return true;
}
