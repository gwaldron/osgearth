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
#include <osg/KdTree>
#include <iterator>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/ImageUtils>


using namespace osg;
using namespace osgEarth::REX;
using namespace osgEarth;

#define LC "[TileDrawable] "

//........................................................................

ModifyBoundingBoxCallback::ModifyBoundingBoxCallback(EngineContext* context) :
_context(context)
{
    //nop
}

void
ModifyBoundingBoxCallback::operator()(const TileKey& key, osg::BoundingBox& bbox)
{
    osg::ref_ptr<TerrainEngineNode> engine = _context->getEngine();
    if (engine.valid())
    {
        engine->fireModifyTileBoundingBoxCallbacks(key, bbox);

        osg::ref_ptr<const Map> map = _context->getMap();
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
}
//........................................................................

TileDrawable::TileDrawable(const TileKey& key,
                           SharedGeometry* geometry,
                           int tileSize) :
osg::Drawable( ),
_key         ( key ),
_geom        ( geometry ),
_tileSize    ( tileSize ),
_bboxRadius  ( 1.0 ),
_bboxCB      ( NULL )
{
    // builds the initial mesh.
    setElevationRaster(0L, osg::Matrixf::identity());
}

TileDrawable::~TileDrawable()
{
    //nop
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
    const osg::DrawElementsUShort* de = dynamic_cast<osg::DrawElementsUShort*>(_geom->getDrawElements());

    OE_SOFT_ASSERT_AND_RETURN(de != nullptr, void());

    if (_mesh.size() < verts.size())
    {
        _mesh.resize(verts.size());
    }

    if ( _elevationRaster.valid() )
    {
        const osg::Vec3Array& normals = *static_cast<osg::Vec3Array*>(_geom->getNormalArray());
        const osg::Vec3Array& units = *static_cast<osg::Vec3Array*>(_geom->getTexCoordArray());

        //OE_INFO << LC << _key.str() << " - rebuilding height cache" << std::endl;

        ImageUtils::PixelReader readElevation(_elevationRaster.get());
        readElevation.setBilinear(true);
        osg::Vec4f sample;

        float
            scaleU = _elevationScaleBias(0,0),
            scaleV = _elevationScaleBias(1,1),
            biasU  = _elevationScaleBias(3,0),
            biasV  = _elevationScaleBias(3,1);

        if ( osg::equivalent(scaleU, 0.0f) || osg::equivalent(scaleV, 0.0f) )
        {
            OE_WARN << LC << "Precision loss in tile " << _key.str() << "\n";
        }

        for (int i = 0; i < verts.size(); ++i)
        {
            if ( ((int)units[i].z() & VERTEX_HAS_ELEVATION) == 0)
            {
                readElevation(
                    sample,
                    clamp(units[i].x()*scaleU + biasU, 0.0f, 1.0f),
                    clamp(units[i].y()*scaleV + biasV, 0.0f, 1.0f));

                _mesh[i] = verts[i] + normals[i] * sample.r();
            }
            else
            {
                _mesh[i] = verts[i];
            }
        }
    }

    else
    {
        std::copy(verts.begin(), verts.end(), _mesh.begin());
    }


    // Make a temporary geometry to build kdtrees on and copy the shape over
    if (_geom->getDrawElements()->getMode() != GL_PATCHES)
    {
        osg::ref_ptr< osg::Geometry > tempGeom = new osg::Geometry;
        osg::Vec3Array* tempVerts = new osg::Vec3Array;
        tempVerts->reserve(_mesh.size());
        for (unsigned int i = 0; i < _mesh.size(); i++)
        {
            tempVerts->push_back(_mesh[i]);
        }
        tempGeom->setVertexArray(tempVerts);
        tempGeom->addPrimitiveSet(_geom->getDrawElements());

        osg::ref_ptr< osg::KdTreeBuilder > kdTreeBuilder = new osg::KdTreeBuilder();
        tempGeom->accept(*kdTreeBuilder.get());
        if (tempGeom->getShape())
        {
            setShape(tempGeom->getShape());
        }
    }

    dirtyBound();
}

// Functor supplies triangles to things like IntersectionVisitor, ComputeBoundsVisitor, etc.
void
TileDrawable::accept(osg::PrimitiveFunctor& f) const
{
    f.setVertexArray(_mesh.size(), _mesh.data());

    f.drawElements(
        GL_TRIANGLES,
        _geom->getDrawElements()->getNumIndices(),
        static_cast<const GLushort*>(_geom->getDrawElements()->getDataPointer()));
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
    for(auto& vert : _mesh)
    {
        box.expandBy(vert);
    }

    // finally see if any of the layers request a bbox change:
    if (_bboxCB)
    {
        (*_bboxCB)(_key, box);
    }

    _bboxRadius = box.radius();

    return box;
}
