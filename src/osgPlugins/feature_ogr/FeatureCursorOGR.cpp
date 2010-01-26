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
#include "FeatureCursorOGR"
#include <osgEarthFeatures/Feature>
#include <osgEarth/Registry>
#include <algorithm>

#define OGR_SCOPED_LOCK GDAL_SCOPED_LOCK

using namespace osgEarth;
using namespace osgEarth::Features;


FeatureCursorOGR::FeatureCursorOGR(OGRDataSourceH dsHandle,
                                   OGRLayerH layerHandle,
                                   const FeatureProfile* profile,
                                   const Query& query,
                                   FeatureFilterList& filters ) :
_dsHandle( dsHandle ),
_layerHandle( layerHandle ),
_resultSetHandle( 0L ),
_profile( profile ),
_query( query ),
_filters( filters ),
_chunkSize( 50 ),
_nextHandleToQueue( 0L ),
_spatialFilter( 0L )
{
    _resultSetHandle = _layerHandle;
    {
        OGR_SCOPED_LOCK;

        std::string expr;
        if ( query.expression().isSet() )
        {
            // build the SQL: allow the Query to include either a full SQL statement or
            // just the WHERE clause.
            expr = query.expression().value();

            // if the expression is just a where clause, expand it into a complete SQL expression.
            std::string temp = expr;
            std::transform( temp.begin(), temp.end(), temp.begin(), ::tolower );
            bool complete = temp.find( "select" ) == 0;
            if ( temp.find( "select" ) != 0 )
            {
                OGRFeatureDefnH layerDef = OGR_L_GetLayerDefn( _layerHandle ); // just a ref.
                std::stringstream buf;
                buf << "SELECT * FROM " << OGR_FD_GetName( layerDef ) << " WHERE " << expr;
				std::string bufStr;
				bufStr = buf.str();
                expr = bufStr;
            }
        }

        // if there's a spatial extent in the query, build the spatial filter:
        if ( query.bounds().isSet() )
        {
            OGRGeometryH ring = OGR_G_CreateGeometry( wkbLinearRing );
            OGR_G_AddPoint(ring, query.bounds()->xMin(), query.bounds()->yMin(), 0 );
            OGR_G_AddPoint(ring, query.bounds()->xMax(), query.bounds()->yMin(), 0 );
            OGR_G_AddPoint(ring, query.bounds()->xMax(), query.bounds()->yMax(), 0 );
            OGR_G_AddPoint(ring, query.bounds()->xMin(), query.bounds()->yMax(), 0 );

            _spatialFilter = OGR_G_CreateGeometry( wkbPolygon );
            OGR_G_AddGeometryDirectly( _spatialFilter, ring ); 
            // note: "Directly" above means _spatialFilter takes ownership if ring handle
        }

        if ( !expr.empty() )
        {
            // an SQL expression, with or without a spatial filter:
            _resultSetHandle = OGR_DS_ExecuteSQL( _dsHandle, expr.c_str(), _spatialFilter, 0L );
        }
        else if ( _spatialFilter ) 
        {
            // just a spatial filter. If it not clear from the docs whether this will take
            // advantage of a source-supplied spatial index; the docs say this operates at
            // the OGR_L_GetNextFeature level.
            OGR_L_SetSpatialFilter( _resultSetHandle, _spatialFilter );
        }

        if ( _resultSetHandle )
        {
            OGR_L_ResetReading( _resultSetHandle );
        }
    }

    readChunk();
}

FeatureCursorOGR::~FeatureCursorOGR()
{
    OGR_SCOPED_LOCK;

    if ( _nextHandleToQueue )
        OGR_F_Destroy( _nextHandleToQueue );

    if ( _resultSetHandle != _layerHandle )
        OGR_DS_ReleaseResultSet( _dsHandle, _resultSetHandle );

    if ( _spatialFilter )
        OGR_G_DestroyGeometry( _spatialFilter );
}

bool
FeatureCursorOGR::hasMore() const
{
    return _resultSetHandle && ( _queue.size() > 0 || _nextHandleToQueue != 0L );
}

Feature*
FeatureCursorOGR::nextFeature()
{
    if ( !hasMore() )
        return 0L;

    if ( _queue.size() == 0 && _nextHandleToQueue )
        readChunk();

    // do this in order to hold a reference to the feature we return, so the caller
    // doesn't have to. This lets us avoid requiring the caller to use a ref_ptr when 
    // simply iterating over the cursor, making the cursor move conventient to use.
    _lastFeatureReturned = _queue.front();
    _queue.pop();

    return _lastFeatureReturned.get();
}

// reads a chunk of features into a memory cache; do this for performance
// and to avoid needing the OGR Mutex every time
void
FeatureCursorOGR::readChunk()
{
    if ( !_resultSetHandle )
        return;
    
    FeatureList preProcessList;
    
    OGR_SCOPED_LOCK;

    if ( _nextHandleToQueue )
    {
        Feature* f = createFeature( _nextHandleToQueue );
        if ( f ) 
        {
            _queue.push( f );
            
            if ( _filters.size() > 0 )
                preProcessList.push_back( f );
        }
        OGR_F_Destroy( _nextHandleToQueue );
    }

    int handlesToQueue = _chunkSize - _queue.size();

    for( int i=0; i<handlesToQueue; i++ )
    {
        OGRFeatureH handle = OGR_L_GetNextFeature( _layerHandle );
        if ( handle )
        {
            Feature* f = createFeature( handle );
            if ( f ) 
            {
                _queue.push( f );

                if ( _filters.size() > 0 )
                    preProcessList.push_back( f );
            }
            OGR_F_Destroy( handle );
        }
        else
            break;
    }

    // preprocess the features using the filter list:
    if ( preProcessList.size() > 0 )
    {
        FilterContext cx;
        cx.profile() = _profile.get();

        for( FeatureFilterList::iterator i = _filters.begin(); i != _filters.end(); ++i )
        {
            FeatureFilter* filter = i->get();
            cx = filter->push( preProcessList, cx );
        }
    }

    // read one more for "more" detection:
    _nextHandleToQueue = OGR_L_GetNextFeature( _layerHandle );

    //osg::notify(osg::NOTICE) << "read " << _queue.size() << " features ... " << std::endl;
}

static void
populate( OGRGeometryH geomHandle, Geometry* target, int numPoints )
{
    for( int v = numPoints-1; v >= 0; v-- ) // reverse winding.. we like ccw
    {
        double x=0, y=0, z=0;
        OGR_G_GetPoint( geomHandle, v, &x, &y, &z );
        osg::Vec3d p( x, y, z );
        if ( target->size() == 0 || p != target->back() ) // remove dupes
            target->push_back( p );
    }
}

static Polygon*
createPolygon( OGRGeometryH geomHandle )
{
    Polygon* output = 0L;

    int numParts = OGR_G_GetGeometryCount( geomHandle );
    if ( numParts == 0 )
    {
        int numPoints = OGR_G_GetPointCount( geomHandle );
        output = new Polygon( numPoints );
        populate( geomHandle, output, numPoints );
        output->open();
    }
    else if ( numParts > 0 )
    {
        Polygon* poly = 0L;
        for( int p = 0; p < numParts; p++ )
        {
            OGRGeometryH partRef = OGR_G_GetGeometryRef( geomHandle, p );
            int numPoints = OGR_G_GetPointCount( partRef );
            if ( p == 0 )
            {
                output = new Polygon( numPoints );
                populate( partRef, output, numPoints );
                output->open();
            }
            else
            {
                Ring* hole = new Ring( numPoints );
                populate( partRef, hole, numPoints );
                hole->open();
                output->getHoles().push_back( hole );
            }
        }
    }
    return output;
}

static Geometry*
createGeometry( OGRGeometryH geomHandle )
{
    Geometry* output = 0L;

    OGRwkbGeometryType wkbType = OGR_G_GetGeometryType( geomHandle );        
    
    if (
        wkbType == wkbPolygon ||
        wkbType == wkbPolygon25D )
    {
        output = createPolygon( geomHandle );
    }
    else if (
        wkbType == wkbLineString ||
        wkbType == wkbLineString25D )
    {
        int numPoints = OGR_G_GetPointCount( geomHandle );
        output = new LineString( numPoints );
        populate( geomHandle, output, numPoints );
    }
    else if (
        wkbType == wkbLinearRing )
    {
        int numPoints = OGR_G_GetPointCount( geomHandle );
        output = new Ring( numPoints );
        populate( geomHandle, output, numPoints );
    }
    else if ( 
        wkbType == wkbPoint ||
        wkbType == wkbPoint25D )
    {
        //todo
    }
    else if (
        wkbType == wkbGeometryCollection ||
        wkbType == wkbGeometryCollection25D ||
        wkbType == wkbMultiPoint ||
        wkbType == wkbMultiPoint25D ||
        wkbType == wkbMultiLineString ||
        wkbType == wkbMultiLineString25D ||
        wkbType == wkbMultiPolygon ||
        wkbType == wkbMultiPolygon25D )
    {
        MultiGeometry* multi = new MultiGeometry();

        int numGeoms = OGR_G_GetGeometryCount( geomHandle );
        for( int n=0; n<numGeoms; n++ )
        {
            OGRGeometryH subGeomRef = OGR_G_GetGeometryRef( geomHandle, n );
            if ( subGeomRef )
            {
                Geometry* geom = createGeometry( subGeomRef );
                if ( geom ) multi->getComponents().push_back( geom );
            }
        } 

        output = multi;
    }

    return output;
}

// NOTE: ASSUMES that OGR_SCOPED_LOCK is already in effect upon entry!
Feature*
FeatureCursorOGR::createFeature( OGRFeatureH handle )
{
    long fid = OGR_F_GetFID( handle );

    Feature* feature = new Feature( fid );

    OGRGeometryH geomRef = OGR_F_GetGeometryRef( handle );	
	if ( geomRef )
	{
        Geometry* geom = createGeometry( geomRef );
        feature->setGeometry( geom );
	}

    //if ( _profile->getGeometryType() == FeatureProfile::GEOM_POLYGON )
    //{
    //    feature->getGeometry().normalizePolygon();
    //}

    //loadAttributes();

    return feature;
}
