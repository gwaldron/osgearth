/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
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

#include <osg/Version>
#include <iterator>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/ImageUtils>

using namespace osg;
using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;

#define LC "[TileDrawable] "


TileDrawable::TileDrawable(const TileKey& key,
                           osg::Geometry* geometry,
                           int            tileSize) :
osg::Drawable( ),
_key         ( key ),
_geom        ( geometry ),
_tileSize    ( tileSize )
{   
    // a mesh to materialize the heightfield for functors
    _mesh = new osg::Vec3f[ tileSize*tileSize ];
    
    // builds the initial mesh.
    setElevationRaster(0L, osg::Matrixf::identity());
}

TileDrawable::~TileDrawable()
{
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
#if 1 // triangles (OSG-stats-friendly)
    //TODO: improve by caching the entire Vec3f, not just the height.
    
    f.begin(GL_TRIANGLES);
    for(int t=0; t<_tileSize-1; ++t)
    {
        for(int s=0; s<_tileSize-1; ++s)
        {
            int i00 = t*_tileSize + s;
            int i10 = i00 + 1;
            int i01 = i00 + _tileSize;
            int i11 = i01 + 1;

            f.vertex(_mesh[i00]);  f.vertex(_mesh[i01]);  f.vertex(_mesh[i10]);
            f.vertex(_mesh[i10]);  f.vertex(_mesh[i01]);  f.vertex(_mesh[i11]);
        }
    }

    f.end();

#else
    // triangle-strips (faster? but not stats-friendly; will cause the OSG stats
    // to report _tileSize-1 primitive sets per TileDrawable even though there
    // is only one.

    for(int t=0; t<_tileSize-1; ++t)
    {
        f.begin( GL_TRIANGLE_STRIP );

        for(int s=0; s<_tileSize; ++s)
        {
            int i = t*_tileSize + s;
            f.vertex( _mesh[i] );

            i += _tileSize;
            f.vertex( _mesh[i] );
        }

        f.end();
    }

#endif
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

    for(unsigned i=0; i<_tileSize*_tileSize; ++i)
    {
        box.expandBy(_mesh[i]);
    }

    if (_bboxCB)
    {
        (*_bboxCB)(_key, box);
    }

    return box;
}
