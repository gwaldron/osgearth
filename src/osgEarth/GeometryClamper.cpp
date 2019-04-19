/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#include <osgEarth/GeometryClamper>
#include <osg/Geometry>

#define LC "[GeometryClamper] "

using namespace osgEarth;

#define ZOFFSETS_NAME "GeometryClamper::zOffsets"

//-----------------------------------------------------------------------

GeometryClamper::GeometryClamper(GeometryClamper::LocalData& localData) :
osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ),
_localData(localData),
_useVertexZ(true),
_revert(false),
_scale( 1.0f ),
_offset( 0.0f )
{
    this->setNodeMaskOverride( ~0 );
    _lsi = new osgUtil::LineSegmentIntersector(osg::Vec3d(0,0,0), osg::Vec3d(0,0,0));
}

void
GeometryClamper::apply(osg::Transform& xform)
{
    osg::Matrixd matrix;
    if ( !_matrixStack.empty() ) matrix = _matrixStack.back();
    xform.computeLocalToWorldMatrix( matrix, this );
    _matrixStack.push_back( matrix );
    traverse(xform);
    _matrixStack.pop_back();
}

void
GeometryClamper::apply(osg::Drawable& drawable)
{
    osg::Geometry* geom = drawable.asGeometry();
    if ( !geom )
        return;

    osg::Vec3Array* verts = static_cast<osg::Vec3Array*>(geom->getVertexArray());

    if (_revert)
    {
        GeometryData& data = _localData[verts];
        if (data._verts.valid() && verts->size() == data._verts->size())
        {
            std::copy(data._verts->begin(), data._verts->end(), verts->begin());
            verts->dirty();
        }
        return;
    }
    
    if ( !_terrainSRS.valid() )
        return;

    const osg::Matrixd& local2world = _matrixStack.back();
    osg::Matrix world2local;
    world2local.invert( local2world );

    const osg::EllipsoidModel* em = _terrainSRS->getEllipsoid();
    osg::Vec3d n_vector(0,0,1), start, end, msl;

    bool isGeocentric = _terrainSRS->isGeographic();

    osgUtil::IntersectionVisitor iv( _lsi.get() );

    double r = osg::minimum( em->getRadiusEquator(), em->getRadiusPolar() );

    unsigned count = 0;

    bool geomDirty = false;

    GeometryData& data = _localData[verts];
    
    bool storeAltitudes = false;

    if (!data._verts.valid() || data._verts->size() != verts->size())
    {
        data._verts = osg::clone(verts, osg::CopyOp::DEEP_COPY_ALL);
        data._altitudes = new osg::FloatArray();
        data._altitudes->reserve(verts->size());
        storeAltitudes = true;
    }

    for( unsigned k=0; k<verts->size(); ++k )
    {
        osg::Vec3d vw = (*verts)[k];
        vw = vw * local2world;

        if ( isGeocentric )
        {
            // normal to the ellipsoid:
            n_vector = em->computeLocalUpVector(vw.x(),vw.y(),vw.z());

            // if we need to store the original altitudes:
            if (storeAltitudes)
            {
                // should really be the alt along the n_vector but leave for now
                // since most scene-clamped geometry will be in relative to a
                // local tangent plane anyway -gw
                data._altitudes->push_back( (*verts)[k].z() );
            }
        }

        else
        {
            if (storeAltitudes)
            {
                data._altitudes->push_back( float(vw.z()) - _offset);
            }
        }

        _lsi->reset();
        _lsi->setStart( vw + n_vector*r*_scale );
        _lsi->setEnd( vw - n_vector*r );
        _lsi->setIntersectionLimit( _lsi->LIMIT_NEAREST );

        _terrainPatch->accept( iv );

        if ( _lsi->containsIntersections() )
        {
            osg::Vec3d fw = _lsi->getFirstIntersection().getWorldIntersectPoint();
            //if ( _scale != 1.0 )
            //{
            //    osg::Vec3d delta = fw - msl;
            //    fw += delta*_scale;
            //}

            if ( _offset != 0.0 )
            {
                fw += n_vector*_offset;
            }

            if (_useVertexZ)
            {
                fw += n_vector * (*data._altitudes)[k];
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
        {
#if OSG_VERSION_LESS_THAN(3,6,0)
            geom->dirtyDisplayList();
#else
            geom->dirtyGLObjects();
#endif
        }

        OE_DEBUG << LC << "clamped " << count << " verts." << std::endl;
    }
}


void
GeometryClamperCallback::onTileAdded(const TileKey&          key, 
                                     osg::Node*              tile, 
                                     TerrainCallbackContext& context)
{
    tile->accept( _clamper );
}
