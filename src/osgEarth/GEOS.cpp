/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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
#include <osgEarth/GEOS>

#ifdef OSGEARTH_HAVE_GEOS

#include <osg/Notify>

using namespace osgEarth;

#define LC "[GEOS] "

#define GEOS_VERSION_AT_LEAST(MAJOR, MINOR) \
    ((GEOS_VERSION_MAJOR>MAJOR) || (GEOS_VERSION_MAJOR==MAJOR && GEOS_VERSION_MINOR>=MINOR))

namespace
{
    GEOSCoordSequence*
        vec3dArray2CoordSeq(GEOSContextHandle_t handle, const Geometry* input, bool close)
    {
        bool needToClose = close && input->size() > 2 && input->front() != input->back();

        unsigned int size = input->size() + (needToClose ? 1 : 0);

        GEOSCoordSequence* coords = GEOSCoordSeq_create_r(handle, size, 3);

        unsigned int idx = 0;
        for (osg::Vec3dArray::const_iterator i = input->begin(); i != input->end(); ++i)
        {
            GEOSCoordSeq_setX_r(handle, coords, idx, i->x());
            GEOSCoordSeq_setY_r(handle, coords, idx, i->y());
            GEOSCoordSeq_setZ_r(handle, coords, idx, i->z());
            ++idx;
        }
        if (needToClose)
        {
            GEOSCoordSeq_setX_r(handle, coords, idx, input->begin()->x());
            GEOSCoordSeq_setY_r(handle, coords, idx, input->begin()->y());
            GEOSCoordSeq_setZ_r(handle, coords, idx, input->begin()->z());
        }
        return coords;
    }

    GEOSGeometry*
        import(GEOSContextHandle_t handle, const Geometry* input)
    {
        GEOSGeometry* output = 0L;

        if (input->getType() == Geometry::TYPE_UNKNOWN)
        {
            output = 0L;
        }
        else if (input->getType() == Geometry::TYPE_MULTI)
        {
            const MultiGeometry* multi = static_cast<const MultiGeometry*>(input);

            Geometry::Type compType = multi->getComponentType();

            std::vector<GEOSGeom> children;
            for (GeometryCollection::const_iterator i = multi->getComponents().begin(); i != multi->getComponents().end(); ++i)
            {
                GEOSGeometry* child = import(handle, i->get());
                if (child)
                    children.push_back(child);
            }

            if (children.size() > 0)
            {
                if (compType == Geometry::TYPE_POLYGON)
                    output = GEOSGeom_createCollection_r(handle, GEOSGeomTypes::GEOS_MULTIPOLYGON, children.data(), children.size());
                else if (compType == Geometry::TYPE_LINESTRING)
                    output = GEOSGeom_createCollection_r(handle, GEOSGeomTypes::GEOS_MULTILINESTRING, children.data(), children.size());
                else if (compType == Geometry::TYPE_POINT || compType == Geometry::TYPE_POINTSET)
                    output = GEOSGeom_createCollection_r(handle, GEOSGeomTypes::GEOS_MULTIPOINT, children.data(), children.size());
                else
                    output = GEOSGeom_createCollection_r(handle, GEOSGeomTypes::GEOS_GEOMETRYCOLLECTION, children.data(), children.size());
            }
        }
        else
        {
            // any other type will at least contain points:
            GEOSCoordSequence* seq = 0;
            GEOSGeom* geomsList = 0;
            switch (input->getType())
            {
            case Geometry::TYPE_UNKNOWN:
                break;
				
            case Geometry::TYPE_MULTI: 
				break;

            case Geometry::TYPE_POINT:
                seq = vec3dArray2CoordSeq(handle, input, false);
                if (seq) output = GEOSGeom_createPoint_r(handle, seq);
                break;

            case Geometry::TYPE_POINTSET:
                geomsList=new GEOSGeom[input->size()];
                for (int i = 0; i < input->size(); i++)
                {
                    seq = GEOSCoordSeq_create_r(handle, 1, 3);
                    {
                        GEOSCoordSeq_setX_r(handle, seq, 0, input->at(i).x());
                        GEOSCoordSeq_setY_r(handle, seq, 0, input->at(i).y());
                        GEOSCoordSeq_setZ_r(handle, seq, 0, input->at(i).z());
                    }
                    geomsList[i] = GEOSGeom_createPoint_r(handle, seq);
                }
                output = GEOSGeom_createCollection_r(handle, GEOS_MULTIPOINT, geomsList, input->size());
                delete geomsList;
                break;

            case Geometry::TYPE_LINESTRING:
                seq = vec3dArray2CoordSeq(handle, input, false);
                if (seq) output = GEOSGeom_createLineString_r(handle, seq);
                break;

            case Geometry::TYPE_RING:
                seq = vec3dArray2CoordSeq(handle, input, true);
                if (seq) output = GEOSGeom_createLinearRing_r(handle, seq);
                break;

            case Geometry::TYPE_POLYGON:
                seq = vec3dArray2CoordSeq(handle, input, true);
                GEOSGeometry* shell = 0L;
                if (seq)
                    shell = GEOSGeom_createLinearRing_r(handle, seq);

                if (shell)
                {
                    const Polygon* poly = static_cast<const Polygon*>(input);
                    std::vector<GEOSGeom> holes;

                    if (poly->getHoles().size() > 0)
                    {
                        for (RingCollection::const_iterator r = poly->getHoles().begin(); r != poly->getHoles().end(); ++r)
                        {
                            GEOSGeometry* hole = import(handle, r->get());
                            if (hole)
                            {
                                if (GEOSGeomTypeId_r(handle, hole) == GEOSGeomTypes::GEOS_LINEARRING)
                                {
                                    holes.push_back(hole);
                                }
                                else
                                {
                                    GEOSGeom_destroy_r(handle, hole);
                                }
                            }
                        }
                    }
                    output = GEOSGeom_createPolygon_r(handle, shell, holes.data(), holes.size());
                }

                break;
            }
        }

        return output;
    }


    Geometry*
        exportPolygon_c(GEOSContextHandle_t handle, const GEOSGeometry* input)
    {
        Polygon* output = 0L;

        const GEOSGeometry* outerRing = GEOSGetExteriorRing_r(handle, input);

        if (outerRing)
        {
            const GEOSCoordSequence* s = GEOSGeom_getCoordSeq_r(handle, outerRing);

            unsigned int outerSize;
            GEOSCoordSeq_getSize_r(handle, s, &outerSize);
            output = new Polygon(outerSize);

            for (unsigned int j = 0; j < outerSize; j++)
            {
                double x, y, z;
                GEOSCoordSeq_getX_r(handle, s, j, &x);
                GEOSCoordSeq_getY_r(handle, s, j, &y);
                GEOSCoordSeq_getZ_r(handle, s, j, &z);
                output->push_back(osg::Vec3d(x, y, !osg::isNaN(z) ? z : 0.0));
                //OE_NOTICE << "c.z = " << c.z << "\n";
            }
            output->rewind(Ring::ORIENTATION_CCW);

            unsigned int numInteriorRings = GEOSGetNumInteriorRings_r(handle, input);
            for (unsigned k = 0; k < numInteriorRings; k++)
            {
                const GEOSGeometry* inner = GEOSGetInteriorRingN_r(handle, input, k);
                const GEOSCoordSequence* s = GEOSGeom_getCoordSeq_r(handle, inner);
                unsigned int innerSize;
                GEOSCoordSeq_getSize_r(handle, s, &innerSize);
                Ring* hole = new Ring(innerSize);
                for (unsigned int m = 0; m < innerSize; m++)
                {
                    double x, y, z;
                    GEOSCoordSeq_getX_r(handle, s, m, &x);
                    GEOSCoordSeq_getY_r(handle, s, m, &y);
                    GEOSCoordSeq_getZ_r(handle, s, m, &z);
                    hole->push_back(osg::Vec3d(x, y, !osg::isNaN(z) ? z : 0.0));
                }
                hole->rewind(Ring::ORIENTATION_CW);
                output->getHoles().push_back(hole);
            }
        }
        return output;
    }
}

GEOSGeometry* GEOS::importGeometry(GEOSContextHandle_t handle, const Geometry* input)
{
    return import(handle, input);
}

Geometry* GEOS::exportGeometry(GEOSContextHandle_t handle, const GEOSGeometry* input)
{
    GeometryCollection parts;

    int typeId = GEOSGeomTypeId_r(handle, input);
    if (typeId == GEOSGeomTypes::GEOS_POINT)
    {
        const GEOSCoordSequence* s = GEOSGeom_getCoordSeq_r(handle, input);
        Point* part = new Point();
        double x, y, z;
        GEOSCoordSeq_getX_r(handle, s, 0, &x);
        GEOSCoordSeq_getY_r(handle, s, 0, &y);
        GEOSCoordSeq_getZ_r(handle, s, 0, &z);
        part->set(osg::Vec3d(x, y, z));
        return part;
    }
    else if (typeId == GEOSGeomTypes::GEOS_MULTIPOINT)
    {
        unsigned int numGeometries = GEOSGetNumGeometries_r(handle, input);
        PointSet* part = new PointSet(numGeometries);
        for (unsigned int i = 0; i < numGeometries; i++)
        {
            const GEOSGeometry* g = GEOSGetGeometryN_r(handle, input, i);
            if (g)
            {
                const GEOSCoordSequence* s = GEOSGeom_getCoordSeq_r(handle, g);
                double x, y, z;
                GEOSCoordSeq_getX_r(handle, s, 0, &x);
                GEOSCoordSeq_getY_r(handle, s, 0, &y);
                GEOSCoordSeq_getZ_r(handle, s, 0, &z);
                part->push_back(osg::Vec3d(x, y, z));
            }
        }
        parts.push_back(part);
    }
    else if (typeId == GEOSGeomTypes::GEOS_LINESTRING)
    {
        const GEOSCoordSequence* s = GEOSGeom_getCoordSeq_r(handle, input);
        unsigned int size;
        GEOSCoordSeq_getSize_r(handle, s, &size);
        LineString* part = new LineString(size);
        for (unsigned int i = 0; i < size; i++)
        {
            double x, y, z;
            GEOSCoordSeq_getX_r(handle, s, i, &x);
            GEOSCoordSeq_getY_r(handle, s, i, &y);
            GEOSCoordSeq_getZ_r(handle, s, i, &z);
            part->push_back(osg::Vec3d(x, y, z));
        }
        parts.push_back(part);
    }
    else if (typeId == GEOSGeomTypes::GEOS_MULTILINESTRING)
    {
        unsigned int numGeometries = GEOSGetNumGeometries_r(handle, input);
        for (unsigned int i = 0; i < numGeometries; i++)
        {
            Geometry* part = GEOS::exportGeometry(handle, GEOSGetGeometryN_r(handle, input, i));
            if (part) parts.push_back(part);
        }
    }
    else if (typeId == GEOSGeomTypes::GEOS_POLYGON)
    {
        Geometry* part = exportPolygon_c(handle, input);
        if (part) parts.push_back(part);
    }
    else if (typeId == GEOSGeomTypes::GEOS_MULTIPOLYGON)
    {
        unsigned int numGeometries = GEOSGetNumGeometries_r(handle, input);
        for (unsigned int i = 0; i < numGeometries; i++)
        {
            Geometry* part = exportPolygon_c(handle, GEOSGetGeometryN_r(handle, input, i));
            if (part) parts.push_back(part);
        }
    }

    if (parts.size() == 1)
    {
        osg::ref_ptr<Geometry> part = parts.front().get();
        parts.clear();
        return part.release();
    }
    else if (parts.size() > 1)
    {
        return new MultiGeometry(parts);
    }
    else
    {
        return 0L;
    }
}

#endif // OSGEARTH_HAVE_GEOS

