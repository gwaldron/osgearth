/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2018 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
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
OgrUtils::createPolygon( OGRGeometryH geomHandle )
{
    Polygon* output = 0L;

#if GDAL_VERSION_AT_LEAST(2,0,0)
    int is3D = OGR_G_Is3D(geomHandle);
#else
    int is3D = OGR_G_GetCoordinateDimension(geomHandle) == 3;
#endif

    int numParts = OGR_G_GetGeometryCount( geomHandle );
    if ( numParts == 0 )
    {
        int numPoints = OGR_G_GetPointCount( geomHandle );
        output = new Polygon( numPoints );
        populate( geomHandle, output, numPoints );

        if (!is3D)
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
                if (!is3D)
                {
                    output->open();
                    output->rewind(Ring::ORIENTATION_CCW);
                }
            }
            else
            {
                Ring* hole = new Ring( numPoints );
                populate( partRef, hole, numPoints );
                if (!is3D)
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
OgrUtils::createGeometry( OGRGeometryH geomHandle )
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
        output = createPolygon(geomHandle);
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
        output = new PointSet( numPoints );
        populate( geomHandle, output, numPoints );
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
    case wkbMultiPoint:
    case wkbMultiPoint25D:
    case wkbMultiLineString:
    case wkbMultiLineString25D:
    case wkbMultiPolygon:
    case wkbMultiPolygon25D:
#ifdef GDAL_HAS_M_TYPES
    case wkbGeometryCollectionM:
    case wkbGeometryCollectionZM:
    case wkbMultiPointM:
    case wkbMultiPointZM:
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
                Geometry* geom = createGeometry( subGeomRef );
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
    OGRGeometryH shape_handle = OGR_G_CreateGeometry( shape_type );
    if ( shape_handle )
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
    return shape_handle;
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
    case osgEarth::Geometry::TYPE_POINTSET:
        requestedType = wkbPoint;
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
    case osgEarth::Geometry::TYPE_POINTSET:
        requestedType = wkbPoint;
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
OgrUtils::createFeature(OGRFeatureH handle, const FeatureProfile* profile)
{
    Feature* f = 0L;
    if ( profile )
    {
        f = createFeature( handle, profile->getSRS() );
        if ( f && profile->geoInterp().isSet() )
            f->geoInterp() = profile->geoInterp().get();
    }
    else
    {
        f = createFeature( handle, (const SpatialReference*)0L );
    }
    return f;
}            

Feature*
OgrUtils::createFeature( OGRFeatureH handle, const SpatialReference* srs )
{
    long fid = OGR_F_GetFID( handle );

    OGRGeometryH geomRef = OGR_F_GetGeometryRef( handle );	

    Geometry* geom = 0;

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
        std::string name = osgEarth::toLower( std::string(field_name) );

        // get the field type and set the value appropriately
        OGRFieldType field_type = OGR_Fld_GetType( field_handle_ref );        
        switch( field_type )
        {
        case OFTInteger:
            {     
                if (IsFieldSet( handle, i ))
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
                if (IsFieldSet( handle, i ))
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
                if (IsFieldSet( handle, i ))
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


