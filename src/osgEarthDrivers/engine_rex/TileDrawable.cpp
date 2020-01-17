/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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
#include "TileDrawable"
#include "EngineContext"

#include <osg/Version>
#include <iterator>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/ImageUtils>

using namespace osg;
using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;

#define LC "[TileDrawable] "

//........................................................................

ModifyBoundingBoxCallback::ModifyBoundingBoxCallback(EngineContext* engine) : 
_engine(engine)
{ 
    //nop
}

void
ModifyBoundingBoxCallback::operator()(const TileKey& key, osg::BoundingBox& bbox)
{
    _engine->getEngine()->fireModifyTileBoundingBoxCallbacks(key, bbox);

    osg::ref_ptr<const Map> map = _engine->getMap();
    if (map.valid())
    {
        LayerVector layers;
        map->getLayers(layers);

        for (LayerVector::const_iterator layer = layers.begin(); layer != layers.end(); ++layer)
        {
            if (layer->valid())
            {
                layer->get()->modifyTileBoundingBox(key, bbox);
            }
        }
    }
}
//........................................................................

TileDrawable::TileDrawable(const TileKey& key,
                           SharedGeometry* geometry,
                           int            tileSize) :
osg::Drawable( ),
_key         ( key ),
_geom        ( geometry ),
_tileSize    ( tileSize ),
_bboxRadius  ( 1.0 )
{   
    // a mesh to materialize the heightfield for functors
    _mesh = new osg::Vec3f[ tileSize*tileSize ];
    
    // allocate and prepopulate mesh index array. 
    // TODO: This is the same for all tiles (of the same tilesize)
    // so perhaps in the future we can just share it.
    _meshIndices = new GLuint[ (tileSize-1)*(tileSize-1)*6 ];
    
    GLuint* k = &_meshIndices[0];
    for(int t=0; t<_tileSize-1; ++t)
    {
        for(int s=0; s<_tileSize-1; ++s)
        {
            int i00 = t*_tileSize + s;
            int i10 = i00 + 1;
            int i01 = i00 + _tileSize;
            int i11 = i01 + 1;

            *k++ = i00; *k++ = i10; *k++ = i01;
            *k++ = i01; *k++ = i10; *k++ = i11;
        }
    }
    
    // builds the initial mesh.
    setElevationRaster(0L, osg::Matrixf::identity());
}

TileDrawable::~TileDrawable()
{
    delete [] _meshIndices;
    delete [] _mesh;
}

void
TileDrawable::setElevationRaster(const osg::Image*   image,
                                 const osg::Matrixf& scaleBias)
{
    _elevationRaster = image;
    _elevationScaleBias = scaleBias;

    if (osg::equivalent(0.0f, _elevationScaleBias(0,0)) ||
        osg::equivalent(0.0f, _elevationScaleBias(1,1)))
    {
        OE_WARN << "("<<_key.str()<<") precision error\n";
    }
    
    const osg::Vec3Array& verts = *static_cast<osg::Vec3Array*>(_geom->getVertexArray());

    if ( _elevationRaster.valid() )
    {
        const osg::Vec3Array& normals = *static_cast<osg::Vec3Array*>(_geom->getNormalArray());

        //OE_INFO << LC << _key.str() << " - rebuilding height cache" << std::endl;

        ImageUtils::PixelReader elevation(_elevationRaster.get());
        elevation.setBilinear(true);

        float
            scaleU = _elevationScaleBias(0,0),
            scaleV = _elevationScaleBias(1,1),
            biasU  = _elevationScaleBias(3,0),
            biasV  = _elevationScaleBias(3,1);

        if ( osg::equivalent(scaleU, 0.0f) || osg::equivalent(scaleV, 0.0f) )
        {
            OE_WARN << LC << "Precision loss in tile " << _key.str() << "\n";
        }
    
        for(int t=0; t<_tileSize; ++t)
        {
            float v = (float)t / (float)(_tileSize-1);
            v = v*scaleV + biasV;

            for(int s=0; s<_tileSize; ++s)
            {
                float u = (float)s / (float)(_tileSize-1);
                u = u*scaleU + biasU;

                unsigned index = t*_tileSize+s;
                _mesh[index] = verts[index] + normals[index] * elevation(u, v).r();
            }
        }
    }

    else
    {
        for (int i = 0; i < _tileSize*_tileSize; ++i)
        {
            _mesh[i] = verts[i];
        }
    }

    dirtyBound();    
}

// Functor supplies triangles to things like IntersectionVisitor, ComputeBoundsVisitor, etc.
void
TileDrawable::accept(osg::PrimitiveFunctor& f) const
{
    f.setVertexArray(_tileSize*_tileSize, _mesh);
    f.drawElements(GL_TRIANGLES, (_tileSize - 1)*(_tileSize-1)*6, _meshIndices);
}

osg::BoundingSphere
TileDrawable::computeBound() const
{
    return osg::BoundingSphere(getBoundingBox());
}

osg::BoundingBox
TileDrawable::computeBoundingBox() const
{
    osg::BoundingBox box;

    // core bbox created from the mesh:
    for(unsigned i=0; i<_tileSize*_tileSize; ++i)
    {
        box.expandBy(_mesh[i]);
    }

    // finally see if any of the layers request a bbox change:
    if (_bboxCB)
    {
        (*_bboxCB)(_key, box);
    }

    _bboxRadius = box.radius();

    return box;
}
