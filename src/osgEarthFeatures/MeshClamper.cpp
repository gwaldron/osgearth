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
#include <osgEarthFeatures/MeshClamper>

#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>

#include <osg/TemplatePrimitiveFunctor>
#include <osg/Geode>
#include <osg/Geometry>

#define LC "[MeshClamper] "

using namespace osgEarth;
using namespace osgEarth::Features;

//-----------------------------------------------------------------------

MeshClamper::MeshClamper(osg::Node*              terrainPatch,
                         const SpatialReference* terrainSRS,
                         bool                    geocentric,
                         double                  scale,
                         double                  offset) :
osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ),
_terrainPatch( terrainPatch ),
_terrainSRS  ( terrainSRS ),
_geocentric  ( geocentric ),
_scale       ( scale ),
_offset      ( offset )
{
    //nop
}

void
MeshClamper::apply( osg::Transform& xform )
{
    osg::Matrixd matrix;
    if ( !_matrixStack.empty() ) matrix = _matrixStack.back();
    xform.computeLocalToWorldMatrix( matrix, this );
    _matrixStack.push_back( matrix );
    traverse(xform);
    _matrixStack.pop_back();
}

void
MeshClamper::apply( osg::Geode& geode )
{
    const osg::Matrixd& local2world = _matrixStack.back();
    osg::Matrix world2local;
    world2local.invert( local2world );

    const osg::EllipsoidModel* em = _terrainSRS->getEllipsoid();
    osg::Vec3d up(0,0,1), start, end, msl;

    osgUtil::LineSegmentIntersector* lsi = new osgUtil::LineSegmentIntersector(start, end);
    osgUtil::IntersectionVisitor iv( lsi );

    for( unsigned i=0; i<geode.getNumDrawables(); ++i )
    {
        bool geomDirty = false;
        osg::Geometry* geom = geode.getDrawable(i)->asGeometry();
        if ( geom )
        {
            osg::Vec3Array* verts = static_cast<osg::Vec3Array*>(geom->getVertexArray());

            for( osg::Vec3Array::iterator k = verts->begin(); k != verts->end(); ++k )
            {
                osg::Vec3d vw = *k;
                vw = vw * local2world;

                if ( _geocentric )
                {
                    up = em->computeLocalUpVector(vw.x(),vw.y(),vw.z());

                    // if we're scaling, we need to know the MSL coord
                    if ( _scale != 1.0 )
                    {
                        double lat,lon,height;
                        em->convertXYZToLatLongHeight(vw.x(), vw.y(), vw.z(), lat, lon, height);
                        em->convertLatLongHeightToXYZ(lat,lon,0.0,msl.x(),msl.y(),msl.z());
                    }
                }

                lsi->reset();
                lsi->setStart( vw + up*50000.0*_scale );
                lsi->setEnd( vw - up*50000.0*_scale );

                _terrainPatch->accept( iv );

                if ( lsi->containsIntersections() )
                {
                    osg::Vec3d fw = lsi->getFirstIntersection().getWorldIntersectPoint();

                    if ( _scale != 1.0 )
                    {
                        osg::Vec3d delta = fw - msl;
                        fw += delta*_scale;
                    }
                    if ( _offset != 0.0 )
                    {
                        fw += up*_offset;
                    }

                    *k = (fw * world2local);
                    geomDirty = true;
                }
            }

            if ( geomDirty )
            {
                geom->dirtyBound();
                if ( geom->getUseVertexBufferObjects() )
                    verts->dirty();
                else
                    geom->dirtyDisplayList();
            }
        }
    }
}
