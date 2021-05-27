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
#include <osgEarth/ElevationRanges>
#include <osg/io_utils>


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

    OE_SOFT_ASSERT_AND_RETURN(de != nullptr, __func__, );

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
            // TODO:  This is just for when you have a constraint...
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
    // TODO:  This might be able to take into account the min/max height.
    osg::BoundingBox box;    

    // TODO:  And also check to see if it has elevation data at all?
    //if (!_elevationRaster.valid()) 
    //if (!_elevationScaleBias.isIdentity())
    if (false)
    {
        // Lookup the min/max height and use that for the bounds.....
        // Could take the mesh and just use verts at the min and verts at the max to compute a tight bounds.
        // In order to actually take advantage of this there has to be an API outside of rex to grab the AABB and be able to do logic against it on the node level.
        // It is used by the TerrainCuller right now on line 250ish (            if (!_cv->isCulled(surface->getAlignedBoundingBox()))).

        // Get the approximate elevation range if we have elevation data in the map
        unsigned int lod = osg::clampBetween(_key.getLevelOfDetail(), 0u, ElevationRanges::getMaxLevel());

        GeoPoint center = _key.getExtent().getCentroid();

        TileKey rangeKey = ElevationRanges::getProfile()->createTileKey(center.x(), center.y(), lod);

        short min, max;
        ElevationRanges::getElevationRange(rangeKey.getLevelOfDetail(), rangeKey.getTileX(), rangeKey.getTileY(), min, max);
        //std::cout << "Got min/max " << min << "/" << max << " for " << _key.str() << std::endl;
        // Clamp the min value to avoid extreme underwater values.
        //short minElevation = osg::maximum(min, (short)-500);
        // Add a little bit extra of extra height to account for feature data.
        //short maxElevation = max + 100.0f;

        const osg::Vec3Array& verts = *static_cast<osg::Vec3Array*>(_geom->getVertexArray());
        const osg::Vec3Array& normals = *static_cast<osg::Vec3Array*>(_geom->getNormalArray());

        for (int i = 0; i < verts.size(); ++i)
        {
            osg::Vec3 high = verts[i] + normals[i] * (float)max;
            osg::Vec3 low = verts[i] + normals[i] * (float)min;
            box.expandBy(high);
            box.expandBy(low);
        }
    }
    else
    {
        // core bbox created from the mesh:
        for (auto& vert : _mesh)
        {
            box.expandBy(vert);
        }
    }

    // finally see if any of the layers request a bbox change:
    if (_bboxCB)
    {
        (*_bboxCB)(_key, box);
    }

    _bboxRadius = box.radius();

    return box;
}
