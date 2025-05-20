/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/
#include <osgEarth/OgrUtils>

#define LC "[FeatureSource] "

using namespace osgEarth;

#ifndef GDAL_VERSION_AT_LEAST
#define GDAL_VERSION_AT_LEAST(MAJOR, MINOR, REV) ((GDAL_VERSION_MAJOR>MAJOR) || (GDAL_VERSION_MAJOR==MAJOR && (GDAL_VERSION_MINOR>MINOR || (GDAL_VERSION_MINOR==MINOR && GDAL_VERSION_REV>=REV))))
#endif

#if GDAL_VERSION_AT_LEAST(2,1,0)
#  define GDAL_HAS_M_TYPES
#endif

namespace
{
    int IsFieldSet(OGRFeatureH handle, int i)
    {
        // https://github.com/Toblerity/Fiona/issues/460
        // GDAL 2.2 changed the behavior of OGR_F_IsFieldSet so that null fields will still be considered set.
        // We consider unset or null fields to be the same, so we use OGR_F_IsFieldSetAndNotNull
    #if GDAL_VERSION_AT_LEAST(2,2,0)
        return OGR_F_IsFieldSetAndNotNull(handle, i);
    #else
        return OGR_F_IsFieldSet(handle, i);
    #endif
    }
}

void
OgrUtils::populate( OGRGeometryH geomHandle, Geometry* target, int numPoints )
{
    for (int v = 0; v < numPoints; ++v)
    {
        double x=0, y=0, z=0;
        OGR_G_GetPoint( geomHandle, v, &x, &y, &z );
        osg::Vec3d p( x, y, z );
        if ( target->size() == 0 || p != target->back() ) // remove dupes
            target->push_back( p );
    }
}

MultiGeometry*
OgrUtils::createTIN(OGRGeometryH geomHandle)
{
    MultiGeometry *multi = new MultiGeometry;

    int numParts = OGR_G_GetGeometryCount(geomHandle);
    if (numParts > 0)
    {
        for (int p = 0; p < numParts; p++)
        {
            OGRGeometryH partRef = OGR_G_GetGeometryRef(geomHandle, p);

            OGRwkbGeometryType partWkbType = OGR_G_GetGeometryType(partRef);

            unsigned int numSubParts = OGR_G_GetGeometryCount(partRef);
            OGRGeometryH subPartRef = OGR_G_GetGeometryRef(partRef, 0);
            unsigned int numSubPoints = OGR_G_GetPointCount(subPartRef);
            Polygon *output = new Polygon(numSubPoints);
            populate(subPartRef, output, numSubPoints);
            output->open();

            // Rewind the triangle so it's oriented correctly
            std::reverse(output->begin(), output->end());

            multi->add(output);
        }
    }

    return multi;
}

Polygon*
OgrUtils::createPolygon( OGRGeometryH geomHandle, bool rewindPolygons)
{
    Polygon* output = 0L;

    int numParts = OGR_G_GetGeometryCount( geomHandle );
    if ( numParts == 0 )
    {
        int numPoints = OGR_G_GetPointCount( geomHandle );
        output = new Polygon( numPoints );
        populate( geomHandle, output, numPoints );

        if (rewindPolygons)
        {
            output->open();
            output->rewind(Ring::ORIENTATION_CCW);
        }
    }
    else if ( numParts > 0 )
    {
        for( int p = 0; p < numParts; p++ )
        {
            OGRGeometryH partRef = OGR_G_GetGeometryRef( geomHandle, p );
            int numPoints = OGR_G_GetPointCount( partRef );

            if ( p == 0 )
            {
                output = new Polygon( numPoints );
                populate( partRef, output, numPoints );
                if (rewindPolygons)
                {
                    output->open();
                    output->rewind(Ring::ORIENTATION_CCW);
                }
            }
            else
            {
                Ring* hole = new Ring( numPoints );
                populate( partRef, hole, numPoints );
                if (rewindPolygons)
                {
                    hole->open();
                    hole->rewind(Ring::ORIENTATION_CW );
                }
                output->getHoles().push_back( hole );
            }
        }
    }
    return output;
}

Geometry*
OgrUtils::createGeometry( OGRGeometryH geomHandle, bool rewindPolygons)
{
    Geometry* output = 0L;

    OGRwkbGeometryType wkbType = OGR_G_GetGeometryType( geomHandle );

    int numPoints, numGeoms;

    switch (wkbType)
    {
    case wkbPolygon:
    case wkbPolygon25D:
#ifdef GDAL_HAS_M_TYPES
    case wkbPolygonM:
    case wkbPolygonZM:
#endif
        output = createPolygon(geomHandle, rewindPolygons);
        break;

    case wkbLineString:
    case wkbLineString25D:
#ifdef GDAL_HAS_M_TYPES
    case wkbLineStringM:
    case wkbLineStringZM:
#endif
        numPoints = OGR_G_GetPointCount( geomHandle );
        output = new LineString( numPoints );
        populate( geomHandle, output, numPoints );
        break;

    case wkbLinearRing:
        numPoints = OGR_G_GetPointCount( geomHandle );
        output = new Ring( numPoints );
        populate( geomHandle, output, numPoints );
        break;

    case wkbPoint:
    case wkbPoint25D:
#ifdef GDAL_HAS_M_TYPES
    case wkbPointM:
    case wkbPointZM:
#endif
        numPoints = OGR_G_GetPointCount( geomHandle );
        output = new Point( numPoints );
        populate( geomHandle, output, numPoints );
        break;

    case wkbMultiPoint:
    case wkbMultiPoint25D:
#ifdef GDAL_HAS_M_TYPES
    case wkbMultiPointM:
    case wkbMultiPointZM:
#endif
        numGeoms = OGR_G_GetGeometryCount(geomHandle);
        output = new PointSet();
        for (int n = 0; n < numGeoms; n++)
        {
            OGRGeometryH subGeomRef = OGR_G_GetGeometryRef(geomHandle, n);
            if (subGeomRef)
            {
                numPoints = OGR_G_GetPointCount(subGeomRef);
                populate(subGeomRef, output, numPoints);
            }
        }
        break;

#ifdef GDAL_HAS_M_TYPES
    case wkbTINZ:
    case wkbTIN:
    case wkbTINM:
    case wkbTINZM:
        output = createTIN(geomHandle);
        break;
#endif

    case wkbGeometryCollection:
    case wkbGeometryCollection25D:
    case wkbMultiLineString:
    case wkbMultiLineString25D:
    case wkbMultiPolygon:
    case wkbMultiPolygon25D:
#ifdef GDAL_HAS_M_TYPES
    case wkbGeometryCollectionM:
    case wkbGeometryCollectionZM:
    case wkbMultiLineStringM:
    case wkbMultiLineStringZM:
    case wkbMultiPolygonM:
    case wkbMultiPolygonZM:
#endif
        MultiGeometry* multi = new MultiGeometry();
        numGeoms = OGR_G_GetGeometryCount( geomHandle );
        for( int n=0; n<numGeoms; n++ )
        {
            OGRGeometryH subGeomRef = OGR_G_GetGeometryRef( geomHandle, n );
            if ( subGeomRef )
            {
                Geometry* geom = createGeometry( subGeomRef, rewindPolygons);
                if ( geom ) multi->getComponents().push_back( geom );
            }
        }
        output = multi;
        break;
    }

    return output;
}

OGRGeometryH
OgrUtils::encodePart( const Geometry* geometry, OGRwkbGeometryType part_type )
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
OgrUtils::encodeShape( const Geometry* geometry, OGRwkbGeometryType shape_type, OGRwkbGeometryType part_type )
{
    OGRGeometryH shape_handle = OGR_G_CreateGeometry(shape_type);
    if (shape_handle)
    {
        if (part_type == wkbNone)
        {
            for (int v = 0; v < geometry->size(); v++)
            {
                osg::Vec3d p = (*geometry)[v];
                OGR_G_AddPoint(shape_handle, p.x(), p.y(), p.z());
            }
        }
        else if (part_type == wkbPoint)
        {
            for (int v = 0; v < geometry->size(); v++)
            {
                osg::Vec3d p = (*geometry)[v];
                OGRGeometryH part_handle = OGR_G_CreateGeometry(part_type);
                OGR_G_AddPoint(part_handle, p.x(), p.y(), p.z());
                OGR_G_AddGeometryDirectly(shape_handle, part_handle);
            }
        }
        else
        {
            ConstGeometryIterator itr(geometry, true);
            while (itr.hasMore())
            {
                const Geometry* geom = itr.next();
                OGRGeometryH part_handle = encodePart(geom, part_type);
                if (part_handle)
                {
                    OGR_G_AddGeometryDirectly(shape_handle, part_handle);
                }
            }
        }
    }
    return shape_handle;

#if 0
    // emit single-point pointset as wkbPoint
    if (geometry->getType() == Geometry::TYPE_POINT)
    {
        shape_handle = OGR_G_CreateGeometry(part_type);
        if (shape_handle)
        {
            osg::Vec3d p = (*geometry)[0];
            OGR_G_AddPoint(shape_handle, p.x(), p.y(), p.z());
        }
    }

    else
    {
        shape_handle = OGR_G_CreateGeometry( shape_type );
        if ( shape_handle )
        {
            // POINTSET requires special handling
            if (shape_type == wkbMultiPoint &&
                part_type == wkbPoint &&
                geometry->getType() == Geometry::TYPE_POINTSET)
            {
                for (int v = geometry->size() - 1; v >= 0; v--)
                {
                    osg::Vec3d p = (*geometry)[v];
                    OGRGeometryH part_handle = OGR_G_CreateGeometry(part_type);
                    OGR_G_AddPoint(part_handle, p.x(), p.y(), p.z());
                    OGR_G_AddGeometryDirectly(shape_handle, part_handle);
                }
            }
            else
            {
                ConstGeometryIterator itr(geometry, true);
                while (itr.hasMore())
                {
                    const Geometry* geom = itr.next();
                    OGRGeometryH part_handle = encodePart( geom, part_type );
                    if ( part_handle )
                    {
                        OGR_G_AddGeometryDirectly( shape_handle, part_handle );
                    }
                }
            }
        }
    }

    return shape_handle;
#endif
}

OGRwkbGeometryType
OgrUtils::getOGRGeometryType(const osgEarth::Geometry::Type& geomType)
{
    OGRwkbGeometryType requestedType = wkbUnknown;

    switch (geomType)
    {
    case osgEarth::Geometry::TYPE_POLYGON:
        requestedType = wkbPolygon;
        break;
    case osgEarth::Geometry::TYPE_POINT:
        requestedType = wkbPoint;
        break;
    case osgEarth::Geometry::TYPE_POINTSET:
        requestedType = wkbMultiPoint;
        break;
    case osgEarth::Geometry::TYPE_LINESTRING:
        requestedType = wkbLineString;
        break;
    case osgEarth::Geometry::TYPE_RING:
        requestedType = wkbLinearRing;
        break;
    default:
    case Geometry::TYPE_UNKNOWN:
        break;
    }

    return requestedType;
}

OGRwkbGeometryType
OgrUtils::getOGRGeometryType(const osgEarth::Geometry* geometry)
{
    OGRwkbGeometryType requestedType = wkbUnknown;

    switch (geometry->getType())
    {
    case osgEarth::Geometry::TYPE_POLYGON:
        requestedType = wkbPolygon;
        break;
    case osgEarth::Geometry::TYPE_POINT:
        requestedType = wkbPoint;
        break;
    case osgEarth::Geometry::TYPE_POINTSET:
        requestedType = wkbMultiPoint;
        break;
    case osgEarth::Geometry::TYPE_LINESTRING:
        requestedType = wkbLineString;
        break;
    case osgEarth::Geometry::TYPE_RING:
        requestedType = wkbLinearRing;
        break;
    case Geometry::TYPE_UNKNOWN:
        break;
    case Geometry::TYPE_MULTI:
    {
        const osgEarth::MultiGeometry* multi = dynamic_cast<const MultiGeometry*>(geometry);
        if (multi)
        {
            osgEarth::Geometry::Type componentType = multi->getComponentType();
            requestedType = componentType == Geometry::TYPE_POLYGON ? wkbMultiPolygon :
                componentType == Geometry::TYPE_POINT? wkbMultiPoint :
                componentType == Geometry::TYPE_POINTSET ? wkbMultiPoint :
                componentType == Geometry::TYPE_LINESTRING ? wkbMultiLineString :
                wkbNone;
        }
    }
    break;
    }

    return requestedType;
}

OGRGeometryH
OgrUtils::createOgrGeometry(const osgEarth::Geometry* geometry, OGRwkbGeometryType requestedType)
{
    if (!geometry) return NULL;

    if (requestedType == wkbUnknown)
    {
        requestedType = getOGRGeometryType(geometry);
    }

    OGRwkbGeometryType shape_type, part_type;

    switch(requestedType)
    {
    case wkbPolygon:
    case wkbPolygon25D:
        shape_type = wkbPolygon, part_type = wkbLinearRing; break;
    case wkbLineString:
    case wkbLineString25D:
        shape_type = wkbLineString, part_type = wkbNone; break;
    case wkbPoint:
    case wkbPoint25D:
        shape_type = wkbPoint, part_type = wkbNone; break;
    case wkbMultiPoint:
    case wkbMultiPoint25D:
        shape_type = wkbMultiPoint, part_type = wkbPoint; break;
    case wkbMultiPolygon:
    case wkbMultiPolygon25D:
        shape_type = wkbPolygon, part_type = wkbLinearRing; break;
    case wkbMultiLineString:
    case wkbMultiLineString25D:
        shape_type = wkbMultiLineString, part_type = wkbLineString; break;

    default:
        shape_type = wkbMultiPoint, part_type = wkbPoint; break;
    }


    const osgEarth::MultiGeometry* multi = dynamic_cast<const MultiGeometry*>(geometry);
    if ( multi )
    {
        OGRGeometryH group_handle = OGR_G_CreateGeometry( wkbGeometryCollection );

        for (GeometryCollection::const_iterator itr = multi->getComponents().begin(); itr != multi->getComponents().end(); ++itr)
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
OgrUtils::OGRFeatureFactory::createFeature(OGRFeatureH handle) const
{
    FeatureID fid = OGR_F_GetFID( handle );

    OGRGeometryH geomRef = OGR_F_GetGeometryRef( handle );

    Geometry* geom = nullptr;

    if ( geomRef )
    {
        geom = OgrUtils::createGeometry( geomRef, rewindPolygons);
    }

    Feature* feature = new Feature(geom, srs, Style(), fid);

    if (srs && interp.isSet())
        feature->geoInterp() = interp.value();

    int numAttrs = OGR_F_GetFieldCount(handle);
    for (int i = 0; i < numAttrs; ++i)
    {
        OGRFieldDefnH field_handle_ref = OGR_F_GetFieldDefnRef( handle, i );

        // get the field name and convert to lower case:
        const char* field_name = OGR_Fld_GetNameRef( field_handle_ref );
        std::string name = osgEarth::toLower( std::string(field_name) );

        // get the field type and set the value appropriately
        OGRFieldType field_type = OGR_Fld_GetType( field_handle_ref );
        switch( field_type )
        {
        case OFTInteger:
            {
                if (IsFieldSet( handle, i ))
                {
                    long long value = OGR_F_GetFieldAsInteger(handle, i);
                    feature->set( name, value );
                }
                else if (keepNullValues)
                {
                    feature->setNull( name, ATTRTYPE_INT );
                }
            }
            break;

        case OFTInteger64:
            {
                if (IsFieldSet(handle, i))
                {
                    long long value = OGR_F_GetFieldAsInteger64(handle, i);
                    feature->set(name, value);
                }
                else if (keepNullValues)
                {
                    feature->setNull(name, ATTRTYPE_INT);
                }
            }
            break;

        case OFTReal:
            {
                if (IsFieldSet( handle, i ))
                {
                    double value = OGR_F_GetFieldAsDouble( handle, i );
                    feature->set( name, value );
                }
                else if (keepNullValues)
                {
                    feature->setNull( name, ATTRTYPE_DOUBLE );
                }
            }
            break;
        default:
            {
                if (IsFieldSet( handle, i ))
                {
                    const char* value = OGR_F_GetFieldAsString(handle, i);
                    feature->set( name, std::string(value) );
                }
                else if (keepNullValues)
                {
                    feature->setNull( name, ATTRTYPE_STRING );
                }
            }
        }
    }

    return feature;
}

AttributeType
OgrUtils::getAttributeType( OGRFieldType type )
{
    switch (type)
    {
    case OFTString: return ATTRTYPE_STRING;
    case OFTReal: return ATTRTYPE_DOUBLE;
    case OFTInteger: return ATTRTYPE_INT;
    default: return ATTRTYPE_UNSPECIFIED;
    };
}


