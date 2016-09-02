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
#include <osgEarth/GeoTransform>
#include <osgEarth/Terrain>

#define LC "[GeoTransform] "

#define OE_TEST OE_DEBUG

using namespace osgEarth;

GeoTransform::GeoTransform() :
_autoRecompute     ( false ),
_autoRecomputeReady( false )
{
   //nop
}

GeoTransform::GeoTransform(const GeoTransform& rhs,
                           const osg::CopyOp&  op) :
osg::MatrixTransform(rhs, op)
{
    _position           = rhs._position;
    _terrain            = rhs._terrain.get();
    _autoRecompute      = rhs._autoRecompute;
    _autoRecomputeReady = false;
}

void
GeoTransform::setTerrain(Terrain* terrain)
{
    _terrain = terrain;

    // Change in the terrain means we need to recompute the position
    // if one is set.
    if ( _position.isValid() )
        setPosition( _position );
}

void
GeoTransform::setAutoRecomputeHeights(bool value)
{
    if (value != _autoRecompute)
    {
        _autoRecompute = value;
    }
}

const GeoPoint&
GeoTransform::getPosition() const
{
    return _position;
}

bool
GeoTransform::setPosition(const GeoPoint& position)
{
    if ( !position.isValid() )
        return false;

    _position = position;

    // relative Z or reprojection require a terrain:
    osg::ref_ptr<Terrain> terrain;
    _terrain.lock(terrain);

    // relative Z requires a terrain:
    if (position.altitudeMode() == ALTMODE_RELATIVE && !terrain.valid())
    {
        OE_TEST << LC << "setPosition failed condition 1\n";
        return false;
    }

    GeoPoint p;

    // transform into terrain SRS if neccesary:
    if (terrain.valid() && !terrain->getSRS()->isEquivalentTo(position.getSRS()))
        p = position.transform(terrain->getSRS());
    else
        p = position;

    // bail if the transformation failed:
    if ( !p.isValid() )
    {
        OE_TEST << LC << "setPosition failed condition 2\n";
        return false;
    }

    // convert to absolute height:
    if ( !p.makeAbsolute(_terrain.get()) )
    {
        OE_TEST << LC << "setPosition failed condition 3\n";
        return false;
    }

    // assemble the matrix:
    osg::Matrixd local2world;
    p.createLocalToWorld( local2world );
    this->setMatrix( local2world );

    // install auto-recompute?
    if (_autoRecompute &&
        _position.altitudeMode() == ALTMODE_RELATIVE &&
        !_autoRecomputeReady)
    {
        // by using the adapter, there's no need to remove
        // the callback then this object destructs.
        terrain->addTerrainCallback(
           new TerrainCallbackAdapter<GeoTransform>(this) );

        _autoRecomputeReady = true;
    }

    return true;
}

void
GeoTransform::onTileAdded(const TileKey&          key,
                          osg::Node*              node,
                          TerrainCallbackContext& context)
{
   if (!_position.isValid() || _position.altitudeMode() != ALTMODE_RELATIVE)
   {
       OE_TEST << LC << "onTileAdded fail condition 1\n";
       return;
   }

   if (!key.getExtent().contains(_position))
   {
       OE_DEBUG << LC << "onTileAdded fail condition 2\n";
       return;
   }

   setPosition(_position);
}

void
GeoTransform::setComputeMatrixCallback(GeoTransform::ComputeMatrixCallback* cb)
{
    _computeMatrixCallback = cb;
}

bool
GeoTransform::ComputeMatrixCallback::computeLocalToWorldMatrix(const GeoTransform* xform, osg::Matrix& m, osg::NodeVisitor* nv) const
{
    if (xform->getReferenceFrame() == xform->RELATIVE_RF)
        m.preMult(xform->getMatrix());
    else
        m = xform->getMatrix();
    return true;
}

bool
GeoTransform::ComputeMatrixCallback::computeWorldToLocalMatrix(const GeoTransform* xform, osg::Matrix& m, osg::NodeVisitor* nv) const
{
    if (xform->getReferenceFrame() == xform->RELATIVE_RF)
        m.postMult(xform->getInverseMatrix());
    else
        m = xform->getInverseMatrix();
    return true;
}
