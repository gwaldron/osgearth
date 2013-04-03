/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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
#include <osgEarthFeatures/OgrUtils>

#define LC "[FeatureSource] "

using namespace osgEarth::Features;


void
    OgrUtils::populate( OGRGeometryH geomHandle, Symbology::Geometry* target, int numPoints )
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

Symbology::Polygon*
    OgrUtils::createPolygon( OGRGeometryH geomHandle )
{
    Symbology::Polygon* output = 0L;

    int numParts = OGR_G_GetGeometryCount( geomHandle );
    if ( numParts == 0 )
    {
        int numPoints = OGR_G_GetPointCount( geomHandle );
        output = new Symbology::Polygon( numPoints );
        populate( geomHandle, output, numPoints );
        output->open();
    }
    else if ( numParts > 0 )
    {
        for( int p = 0; p < numParts; p++ )
        {
            OGRGeometryH partRef = OGR_G_GetGeometryRef( geomHandle, p );
            int numPoints = OGR_G_GetPointCount( partRef );
            if ( p == 0 )
            {
                output = new Symbology::Polygon( numPoints );
                populate( partRef, output, numPoints );
                //output->open();
                output->rewind( Symbology::Ring::ORIENTATION_CCW );
            }
            else
            {
                Symbology::Ring* hole = new Symbology::Ring( numPoints );
                populate( partRef, hole, numPoints );
                //hole->open();
                hole->rewind( Symbology::Ring::ORIENTATION_CW );
                output->getHoles().push_back( hole );
            }
        }
    }
    return output;
}

Symbology::Geometry*
    OgrUtils::createGeometry( OGRGeometryH geomHandle )
{
    Symbology::Geometry* output = 0L;

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
        output = new Symbology::LineString( numPoints );
        populate( geomHandle, output, numPoints );
    }
    else if (
        wkbType == wkbLinearRing )
    {
        int numPoints = OGR_G_GetPointCount( geomHandle );
        output = new Symbology::Ring( numPoints );
        populate( geomHandle, output, numPoints );
    }
    else if ( 
        wkbType == wkbPoint ||
        wkbType == wkbPoint25D )
    {
        int numPoints = OGR_G_GetPointCount( geomHandle );
        output = new Symbology::PointSet( numPoints );
        populate( geomHandle, output, numPoints );
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
        Symbology::MultiGeometry* multi = new Symbology::MultiGeometry();

        int numGeoms = OGR_G_GetGeometryCount( geomHandle );
        for( int n=0; n<numGeoms; n++ )
        {
            OGRGeometryH subGeomRef = OGR_G_GetGeometryRef( geomHandle, n );
            if ( subGeomRef )
            {
                Symbology::Geometry* geom = createGeometry( subGeomRef );
                if ( geom ) multi->getComponents().push_back( geom );
            }
        } 

        output = multi;
    }

    return output;
}

OGRGeometryH
    OgrUtils::encodePart( Geometry* geometry, OGRwkbGeometryType part_type )
{
    OGRGeometryH part_handle = OGR_G_CreateGeometry( part_type );

    for( int v = geometry->size()-1; v >= 0; v-- )
    {
        osg::Vec3d p = (*geometry)[v];
        OGR_G_AddPoint( part_handle, p.x(), p.y(), p.z() );
    }

    return part_handle;
}


OGRGeometryH
    OgrUtils::encodeShape( Geometry* geometry, OGRwkbGeometryType shape_type, OGRwkbGeometryType part_type )
{
    OGRGeometryH shape_handle = OGR_G_CreateGeometry( shape_type );
    if ( shape_handle )
    {
        GeometryIterator itr(geometry, true);
        while (itr.hasMore())
        {
            Geometry* geom = itr.next();
            OGRGeometryH part_handle = encodePart( geom, part_type );
            if ( part_handle )
            {
                OGR_G_AddGeometryDirectly( shape_handle, part_handle );
            }
        }
    }
    return shape_handle;
}

OGRGeometryH
    OgrUtils::createOgrGeometry(osgEarth::Symbology::Geometry* geometry, OGRwkbGeometryType requestedType)
{
    if (!geometry) return NULL;

    if (requestedType == wkbUnknown)
    {
        osgEarth::Symbology::Geometry::Type geomType = geometry->getType();
        switch( geomType)
        {
        case osgEarth::Symbology::Geometry::TYPE_POLYGON:  
            requestedType = wkbPolygon;
            break;
        case osgEarth::Symbology::Geometry::TYPE_POINTSET:  
            requestedType = wkbPoint;
            break;
        case osgEarth::Symbology::Geometry::TYPE_LINESTRING:
            requestedType = wkbLineString;
            break;
        case osgEarth::Symbology::Geometry::TYPE_RING:
            requestedType = wkbLinearRing;
            break;            
        case Geometry::TYPE_UNKNOWN: break;
        case Geometry::TYPE_MULTI: 
            {
                osgEarth::Symbology::MultiGeometry* multi = dynamic_cast<MultiGeometry*>(geometry);
                osgEarth::Symbology::Geometry::Type componentType = multi->getComponentType();
                requestedType = componentType == Geometry::TYPE_POLYGON ? wkbMultiPolygon : 
                    componentType == Geometry::TYPE_POINTSET ? wkbMultiPoint :
                    componentType == Geometry::TYPE_LINESTRING ? wkbMultiLineString :
                    wkbNone;                    
            }
            break;
        }
    }

    OGRwkbGeometryType shape_type =
        requestedType == wkbPolygon || requestedType == wkbMultiPolygon ? wkbPolygon :
        requestedType == wkbPolygon25D || requestedType == wkbMultiPolygon25D? wkbPolygon25D :
        requestedType == wkbLineString || requestedType == wkbMultiLineString? wkbMultiLineString :
        requestedType == wkbLineString25D || requestedType == wkbMultiLineString25D? wkbMultiLineString25D :
        requestedType == wkbPoint || requestedType == wkbMultiPoint? wkbMultiPoint :
        requestedType == wkbPoint25D || requestedType == wkbMultiPoint25D? wkbMultiPoint25D :
        wkbNone;

    OGRwkbGeometryType part_type =
        shape_type == wkbPolygon || shape_type == wkbPolygon25D? wkbLinearRing :
        shape_type == wkbMultiLineString? wkbLineString :
        shape_type == wkbMultiLineString25D? wkbLineString25D :
        shape_type == wkbMultiPoint? wkbPoint :
        shape_type == wkbMultiPoint25D? wkbPoint25D :
        wkbNone;

    //OE_NOTICE << "shape_type = " << shape_type << " part_type=" << part_type << std::endl;


    osgEarth::Symbology::MultiGeometry* multi = dynamic_cast<MultiGeometry*>(geometry);

    if ( multi )
    {
        OGRGeometryH group_handle = OGR_G_CreateGeometry( wkbGeometryCollection );

        for (GeometryCollection::iterator itr = multi->getComponents().begin(); itr != multi->getComponents().end(); ++itr)
        {
            OGRGeometryH shape_handle = encodeShape( itr->get(), shape_type, part_type );
            if ( shape_handle )
            {
                OGRErr error = OGR_G_AddGeometryDirectly( group_handle, shape_handle );
                if ( error != OGRERR_NONE )
                {
                    OE_WARN << "OGR_G_AddGeometryDirectly failed! " << error << std::endl;
                    OE_WARN << "shape_type = " << shape_type << " part_type=" << part_type << std::endl;
                }                    
            }
        }

        return group_handle;
    }
    else
    {
        OGRGeometryH shape_handle = encodeShape( geometry, shape_type, part_type );
        return shape_handle;
    }
}

Feature*
    OgrUtils::createFeature( OGRFeatureH handle, const SpatialReference* srs )
{
    long fid = OGR_F_GetFID( handle );

    OGRGeometryH geomRef = OGR_F_GetGeometryRef( handle );	

    Symbology::Geometry* geom = 0;

    if ( geomRef )
    {
        geom = OgrUtils::createGeometry( geomRef );
    }

    Feature* feature = new Feature( geom, srs, Style(), fid );

    int numAttrs = OGR_F_GetFieldCount(handle); 
    for (int i = 0; i < numAttrs; ++i) 
    { 
        OGRFieldDefnH field_handle_ref = OGR_F_GetFieldDefnRef( handle, i ); 

        // get the field name and convert to lower case:
        const char* field_name = OGR_Fld_GetNameRef( field_handle_ref ); 
        std::string name = std::string( field_name ); 
        std::transform( name.begin(), name.end(), name.begin(), ::tolower ); 

        // get the field type and set the value appropriately
        OGRFieldType field_type = OGR_Fld_GetType( field_handle_ref );        
        switch( field_type )
        {
        case OFTInteger:
            {     
                if (OGR_F_IsFieldSet( handle, i ))
                {
                    int value = OGR_F_GetFieldAsInteger( handle, i );
                    feature->set( name, value );                    
                }
                else
                {
                    feature->setNull( name, ATTRTYPE_INT );
                }
            }
            break;
        case OFTReal:
            {
                if (OGR_F_IsFieldSet( handle, i ))
                {
                    double value = OGR_F_GetFieldAsDouble( handle, i );
                    feature->set( name, value );
                }
                else
                {
                    feature->setNull( name, ATTRTYPE_DOUBLE );
                }
            }
            break;
        default:
            {
                if (OGR_F_IsFieldSet( handle, i ))
                {
                    const char* value = OGR_F_GetFieldAsString(handle, i);
                    feature->set( name, std::string(value) );
                }
                else
                {
                    feature->setNull( name, ATTRTYPE_STRING );
                }
            }
        }
    } 

    return feature;
}

AttributeType OgrUtils::getAttributeType( OGRFieldType type )
{
    switch (type)
    {
    case OFTString: return ATTRTYPE_STRING;
    case OFTReal: return ATTRTYPE_DOUBLE;
    case OFTInteger: return ATTRTYPE_INT;
    default: return ATTRTYPE_UNSPECIFIED;
    };        
}


