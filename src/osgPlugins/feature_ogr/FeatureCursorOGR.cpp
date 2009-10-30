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

#define OGR_SCOPED_LOCK GDAL_SCOPED_LOCK

using namespace osgEarth;
using namespace osgEarthFeatures;


FeatureCursorOGR::FeatureCursorOGR(OGRLayerH layerHandle,
                                   const FeatureProfile* profile,
                                   const FeatureQuery& query ) :
_layerHandle( layerHandle ),
_query( query ),
_profile( profile ),
_chunkSize( 50 ),
_nextHandleToQueue( 0L )
{
    {
        OGR_SCOPED_LOCK;
        OGR_L_ResetReading( _layerHandle );
    }
    readChunk();
}

FeatureCursorOGR::~FeatureCursorOGR()
{
    OGR_SCOPED_LOCK;

    if ( _nextHandleToQueue )
        OGR_F_Destroy( _nextHandleToQueue );
}

bool
FeatureCursorOGR::hasMore() const
{
    return _queue.size() > 0 || _nextHandleToQueue != 0L;
}

Feature*
FeatureCursorOGR::nextFeature()
{
    if ( !hasMore() )
        return 0L;

    if ( _queue.size() == 0 && _nextHandleToQueue )
        readChunk();

    // do this in order to hold a reference to the feature we return, so the caller
    // doesn't have to. This will avoid requiring the caller to use a ref_ptr when 
    // simply iterating over the cursor.
    _lastFeatureReturned = _queue.front();
    _queue.pop();

    return _lastFeatureReturned.get();
}

void
FeatureCursorOGR::readChunk()
{
    if ( _nextHandleToQueue )
    {
        Feature* f = createFeature( _nextHandleToQueue );
        if ( f ) _queue.push( f );
    }

    int handlesToQueue = _chunkSize - _queue.size();
        
    OGR_SCOPED_LOCK;

    for( int i=0; i<handlesToQueue; i++ )
    {
        OGRFeatureH handle = OGR_L_GetNextFeature( _layerHandle );
        if ( handle )
        {
            Feature* f = createFeature( handle );
            if ( f ) _queue.push( f );
        }
        else
            break;
    }

    // read one more for "more" detection:
    _nextHandleToQueue = OGR_L_GetNextFeature( _layerHandle );
}

static void
insertPart( OGRGeometryH geomHandle, const FeatureProfile* profile, Feature* feature )
{    
    int numPoints = OGR_G_GetPointCount( geomHandle );
    if ( numPoints > 0 )
    {
        osg::Vec3dArray* points = new osg::Vec3dArray( numPoints );

        for( int v = numPoints-1, j=0; v >= 0; v--, j++ ) // reserve winding
        {
            double x=0, y=0, z=0;
            OGR_G_GetPoint( geomHandle, v, &x, &y, &z );
            (*points)[j].set( x, y, z );
        }
        
        feature->addPart( points );
    }
}

static void
insertGeometry( OGRGeometryH geomHandle, const FeatureProfile* profile, Feature* feature )
{
    int numParts = OGR_G_GetGeometryCount( geomHandle );

    if ( numParts == 0 )
    {
        insertPart( geomHandle, profile, feature );
    }
    else
    {
        for( int p = 0; p < numParts; p++ )
        {
            OGRGeometryH partRef = OGR_G_GetGeometryRef( geomHandle, p );
            if ( partRef )
                insertPart( partRef, profile, feature );
        }
    }
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
        if ( _profile->isMultiGeometry() )
        {
            int numGeoms = OGR_G_GetGeometryCount( geomRef );
            for( int n=0; n<numGeoms; n++ )
            {
                OGRGeometryH subGeomRef = OGR_G_GetGeometryRef( geomRef, n );
                if ( subGeomRef )
                    insertGeometry( subGeomRef, _profile.get(), feature );               
                //if ( shape.getParts().size() )
                //{
                //    shapes.push_back( shape );
                //    extent.expandToInclude( shape.getExtent() );
                //}
            }
        }
        else // single-geometry
        {
            insertGeometry( geomRef, _profile.get(), feature );
            //if ( shape.getParts().size() > 0 )
            //{
            //    shapes.push_back( shape );
            //    extent.expandToInclude( shape.getExtent() );
            //}
        }
	}

    //loadAttributes();

    return feature;
}

