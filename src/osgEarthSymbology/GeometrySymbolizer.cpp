/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osgEarthSymbology/GeometrySymbolizer>
#include <osgEarthSymbology/PointSymbol>
#include <osgEarthSymbology/LineSymbol>
#include <osgEarthSymbology/PolygonSymbol>
#include <osgUtil/Tessellator>
#include <osg/Geometry>
#include <osg/Point>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/Geode>

using namespace osgEarth::Symbology;


osg::Node* GeometrySymbolizer::GeometrySymbolizerOperator::operator()(const GeometryList& geometryList,
                                                                      const Style* style)
{
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    for (GeometryList::const_iterator it = geometryList.begin(); it != geometryList.end(); ++it)
    {
        Geometry* geometry = *it;
        if (!geometry)
            continue;

        GeometryIterator geomIterator( geometry );
        geomIterator.traverseMultiGeometry() = true;
        geomIterator.traversePolygonHoles() = true;

        while( geomIterator.hasMore() )
        {
            Geometry* part = geomIterator.next();
            if (!part)
                continue;

            osg::ref_ptr<osg::Geometry> osgGeom = new osg::Geometry;
            osg::PrimitiveSet::Mode primMode = osg::PrimitiveSet::POINTS;

            osg::Vec4 color = osg::Vec4(1.0, 0.0, 1.0, 1.);

            switch( part->getType())
            {
            case Geometry::TYPE_POINTSET:
            {
                primMode = osg::PrimitiveSet::POINTS;
                const PointSymbol* point = style->getSymbol<PointSymbol>();
                if (point)
                {
                    color = point->fill()->color();

                    float size = point->size().value();
                    osgGeom->getOrCreateStateSet()->setAttributeAndModes( new osg::Point(size) );
                }
            }
            break;

            case Geometry::TYPE_LINESTRING:
            {
                primMode = osg::PrimitiveSet::LINE_STRIP;
                const LineSymbol* line = style->getSymbol<LineSymbol>();
                if (line) 
                {
                    color = line->stroke()->color();
                    float size = line->stroke()->width().value();
                    osgGeom->getOrCreateStateSet()->setAttributeAndModes( new osg::LineWidth(size));
                }
            }
            break;

            case Geometry::TYPE_RING:
            {
                primMode = osg::PrimitiveSet::LINE_LOOP;
                const LineSymbol* line = style->getSymbol<LineSymbol>();
                if (line) 
                {
                    color = line->stroke()->color();
                    float size = line->stroke()->width().value();
                    osgGeom->getOrCreateStateSet()->setAttributeAndModes( new osg::LineWidth(size));
                }
            }
            break;

            case Geometry::TYPE_POLYGON:
            {
                primMode = osg::PrimitiveSet::LINE_LOOP; // loop will tessellate into polys
                const PolygonSymbol* poly = style->getSymbol<PolygonSymbol>();
                if (poly)
                {
                    color = poly->fill()->color();
                }
            }
            break;
            
            default:
            break;
            }

            osg::Material* material = new osg::Material;
            material->setDiffuse(osg::Material::FRONT_AND_BACK, color);

            if ( part->getType() == Geometry::TYPE_POLYGON && static_cast<Polygon*>(part)->getHoles().size() > 0 )
            {
                Polygon* poly = static_cast<Polygon*>(geometry);
                int totalPoints = poly->getTotalPointCount();
                osg::Vec3Array* allPoints = new osg::Vec3Array( totalPoints );
                int offset = 0;
                for( RingCollection::const_iterator h = poly->getHoles().begin(); h != poly->getHoles().end(); ++h )
                {
                    Geometry* hole = h->get();
                    std::copy( hole->begin(), hole->end(), allPoints->begin() + offset );
                    osgGeom->addPrimitiveSet( new osg::DrawArrays( primMode, offset, hole->size() ) );
                    offset += hole->size();
                }
                osgGeom->setVertexArray( allPoints );
            }
            else
            {
                osgGeom->setVertexArray( part->toVec3Array() );
                osgGeom->addPrimitiveSet( new osg::DrawArrays( primMode, 0, part->size() ) );
            }

            // tessellate all polygon geometries. Tessellating each geometry separately
            // with TESS_TYPE_GEOMETRY is much faster than doing the whole bunch together
            // using TESS_TYPE_DRAWABLE.

            if ( part->getType() == Geometry::TYPE_POLYGON)
            {
                osgUtil::Tessellator tess;
                tess.setTessellationType( osgUtil::Tessellator::TESS_TYPE_GEOMETRY );
                tess.setWindingType( osgUtil::Tessellator::TESS_WINDING_POSITIVE );
                tess.retessellatePolygons( *osgGeom );
            }
            osgGeom->getOrCreateStateSet()->setAttributeAndModes(material);
            geode->addDrawable(osgGeom);
        }
    }

    if (geode->getNumDrawables())
        return geode.release();
    return 0;
}


GeometrySymbolizer::GeometrySymbolizer()
{
    //nop
}

bool
GeometrySymbolizer::compile(GeometrySymbolizerState* state,
                            osg::Group* attachPoint)
{
    if ( !state || !state->getContent() || !attachPoint || !state->getStyle() )
        return false;

    //const GeometryContent* geometryInput = dynamic_cast<const GeometryContent*>(dataSet);
    //if (!geometryInput)
    //    return false;

    const GeometryList& geometryList = state->getContent()->getGeometryList();

    GeometrySymbolizerOperator functor;
    osg::Node* node = (functor)(geometryList, state->getStyle());
    if (node)
    {
        attachPoint->removeChildren(0, attachPoint->getNumChildren());
        attachPoint->addChild(node);
        return true;
    }

    return false;
}
