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
#include <osgEarthFeatures/MeshClamper>

#include <osgEarth/DPLineSegmentIntersector>

#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>

#include <osg/TemplatePrimitiveFunctor>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/UserDataContainer>

#define LC "[MeshClamper] "

using namespace osgEarth;
using namespace osgEarth::Features;

#define ZOFFSETS_NAME "MeshClamper::zOffsets"

//-----------------------------------------------------------------------

MeshClamper::MeshClamper(osg::Node*              terrainPatch,
                         const SpatialReference* terrainSRS,
                         bool                    geocentric,
                         bool                    preserveZ,
                         double                  scale,
                         double                  offset) :

osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ),
_terrainPatch   ( terrainPatch ),
_terrainSRS     ( terrainSRS ),
_geocentric     ( geocentric ),
_preserveZ      ( preserveZ ),
_scale          ( scale ),
_offset         ( offset )
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
    osg::Vec3d n_vector(0,0,1), start, end, msl;

    // use a double-precision intersector b/c our intersection segment will be really long :)
    DPLineSegmentIntersector* lsi = new DPLineSegmentIntersector(start, end);
    osgUtil::IntersectionVisitor iv( lsi );

    double r = std::min( em->getRadiusEquator(), em->getRadiusPolar() );
    //double r = 50000;

    unsigned count = 0;

    for( unsigned i=0; i<geode.getNumDrawables(); ++i )
    {
        bool geomDirty = false;
        osg::Geometry* geom = geode.getDrawable(i)->asGeometry();
        if ( geom )
        {
            osg::Vec3Array*  verts = static_cast<osg::Vec3Array*>(geom->getVertexArray());
            osg::FloatArray* zOffsets = 0L;

            // if preserve-Z is on, check for our elevations array. Create it if is doesn't
            // already exist.
            bool buildZOffsets = false;
            if ( _preserveZ )
            {
                osg::UserDataContainer* udc = geom->getOrCreateUserDataContainer();
                unsigned n = udc->getUserObjectIndex( ZOFFSETS_NAME );
                if ( n < udc->getNumUserObjects() )
                {
                    zOffsets = dynamic_cast<osg::FloatArray*>(udc->getUserObject(n));
                }

                else
                {
                    zOffsets = new osg::FloatArray();
                    zOffsets->setName( ZOFFSETS_NAME );
                    zOffsets->reserve( verts->size() );
                    udc->addUserObject( zOffsets );
                    buildZOffsets = true;
                }
            }

            for( unsigned k=0; k<verts->size(); ++k )
            {
                osg::Vec3d vw = (*verts)[k];
                vw = vw * local2world;

                if ( _geocentric )
                {
                    // normal to the ellipsoid:
                    n_vector = em->computeLocalUpVector(vw.x(),vw.y(),vw.z());

                    // if we need to build to z-offsets array, calculate the z offset now:
                    if ( buildZOffsets || _scale != 1.0 )
                    {
                        double lat,lon,hae;
                        em->convertXYZToLatLongHeight(vw.x(), vw.y(), vw.z(), lat, lon, hae);

                        if ( buildZOffsets )
                        {
                            zOffsets->push_back( float(hae) );
                        }

                        if ( _scale != 1.0 )
                        {
                            msl = vw - n_vector*hae;
                        }
                    }
                }

                else if ( buildZOffsets ) // flat map
                {
                    zOffsets->push_back( float(vw.z()) );
                }

#if 0
                    // if we're scaling, we need to know the MSL coord
                    if ( _scale != 1.0 )
                    {
                        double lat,lon,height;
                        em->convertXYZToLatLongHeight(vw.x(), vw.y(), vw.z(), lat, lon, height);
                        msl = vw - n_vector*height;
                    }
                }
#endif

                lsi->reset();
                lsi->setStart( vw + n_vector*r*_scale );
                lsi->setEnd( vw - n_vector*r );

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
                        fw += n_vector*_offset;
                    }
                    if ( _preserveZ )
                    {
                        fw += n_vector * (*zOffsets)[k];
                    }

                    (*verts)[k] = (fw * world2local);
                    geomDirty = true;
                    ++count;
                }
            }

            if ( geomDirty )
            {
                geom->dirtyBound();
                if ( geom->getUseVertexBufferObjects() )
                {
                    verts->getVertexBufferObject()->setUsage( GL_DYNAMIC_DRAW_ARB );
                    verts->dirty();
                }
                else
                    geom->dirtyDisplayList();
            }
        }

        //OE_NOTICE << LC << "clamped " << count << " verts." << std::endl;
    }
}
