/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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

#include <osgUtil/IntersectionVisitor>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/UserDataContainer>

#define LC "[GeometryClamper] "

using namespace osgEarth;

#define ZOFFSETS_NAME "GeometryClamper::zOffsets"

//-----------------------------------------------------------------------

GeometryClamper::GeometryClamper() :
osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ),
_preserveZ      ( false ),
_scale          ( 1.0f ),
_offset         ( 0.0f )
{
    this->setNodeMaskOverride( ~0 );
    _lsi = new osgEarth::DPLineSegmentIntersector(osg::Vec3d(0,0,0), osg::Vec3d(0,0,0));
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
GeometryClamper::apply(osg::Geode& geode)
{
    if ( !_terrainSRS.valid() )
        return;

    const osg::Matrixd& local2world = _matrixStack.back();
    osg::Matrix world2local;
    world2local.invert( local2world );

    const osg::EllipsoidModel* em = _terrainSRS->getEllipsoid();
    osg::Vec3d n_vector(0,0,1), start, end, msl;

    bool isGeocentric = _terrainSRS->isGeographic();

    osgUtil::IntersectionVisitor iv( _lsi.get() );

    double r = std::min( em->getRadiusEquator(), em->getRadiusPolar() );

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

                if ( isGeocentric )
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
                            zOffsets->push_back( (*verts)[k].z() );
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

                _lsi->reset();
                _lsi->setStart( vw + n_vector*r*_scale );
                _lsi->setEnd( vw - n_vector*r );
                _lsi->setIntersectionLimit( _lsi->LIMIT_NEAREST );

                _terrainPatch->accept( iv );

                if ( _lsi->containsIntersections() )
                {
                    osg::Vec3d fw = _lsi->getFirstIntersection().getWorldIntersectPoint();
                    if ( _scale != 1.0 )
                    {
                        osg::Vec3d delta = fw - msl;
                        fw += delta*_scale;
                    }
                    if ( _offset != 0.0 )
                    {
                        fw += n_vector*_offset;
                    }
                    if ( _preserveZ && (zOffsets != 0L) )
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
                {
                    geom->dirtyDisplayList();
                }
            }
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
