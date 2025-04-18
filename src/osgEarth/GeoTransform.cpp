/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/GeoTransform>
#include <osgEarth/MapNode>
#include <osgEarth/NodeUtils>

#define LC "[GeoTransform] "

#define OE_TEST OE_NULL

using namespace osgEarth;

GeoTransform::GeoTransform() :
    _findTerrainInUpdateTraversal(false),
    _terrainCallbackInstalled(false),
    _autoRecomputeHeights(true),
    _clampInUpdateTraversal(false)
{
    //nop
}

GeoTransform::GeoTransform(const GeoTransform& rhs, const osg::CopyOp& op) :
    osg::MatrixTransform(rhs, op)
{
    _position = rhs._position;
    _terrain = rhs._terrain.get();
    _autoRecomputeHeights = rhs._autoRecomputeHeights;
    _terrainCallbackInstalled = false;
    _findTerrainInUpdateTraversal = false;
    _clampInUpdateTraversal = false;
}

GeoTransform::~GeoTransform()
{
    if (_terrain.valid())
        _terrain->removeObserver(this);
}

void
GeoTransform::setTerrain(Terrain* terrain)
{
    if (terrain)
    {
        if (_terrain.valid())
        {
            if (_terrainCallback.valid())
            {
                _terrain->removeTerrainCallback(_terrainCallback.get());
                _terrainCallbackInstalled = false;
            }
            _terrain->removeObserver(this);
        }

        _terrain = terrain;
        _terrain->addObserver(this);

        if (_terrainCallback.valid())
        {
            _terrain->addTerrainCallback(_terrainCallback);
            _terrainCallbackInstalled = true;
        }

        setPosition(_position);
    }
}

void
GeoTransform::objectDeleted(void* ptr)
{
    // called when the observed Terrain is deleted.  Since _terrain is
    // in an observer_ptr, we cannot trust its value because it might
    // be cleared out before we got here.  We also cannot set its value
    // because that will change the observer list on the item being
    // deleted, leading to invalidated iterators in OSG.  Because it's
    // in an observer_ptr, _terrain will automatically set to NULL.
    if (!_findTerrainInUpdateTraversal)
    {
        _findTerrainInUpdateTraversal = true;
        ADJUST_UPDATE_TRAV_COUNT(this, +1);
    }
}

void
GeoTransform::setAutoRecomputeHeights(bool value)
{
    if (value != _autoRecomputeHeights)
    {
        _autoRecomputeHeights = value;
        setPosition(_position);
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

    bool result = true;

    _position = position;

    // relative Z or reprojection require a terrain:
    osg::ref_ptr<Terrain> terrain;
    _terrain.lock(terrain);

    // If we don't have a pointer to a terrain, schedule an attempt
    // to find one on the next update traversal.
    if (!terrain.valid() && !_findTerrainInUpdateTraversal)
    {
        _findTerrainInUpdateTraversal = true;
        ADJUST_UPDATE_TRAV_COUNT(this, +1);
    }

    GeoPoint p;

    // transform into terrain SRS if neccesary:
    if (terrain.valid() && !terrain->getSRS()->isEquivalentTo(position.getSRS()))
    {
        p = position.transform(terrain->getSRS());
    }
    else
    {
        p = position;
    }

    // bail if the transformation failed:
    if ( !p.isValid() )
    {
        OE_TEST << LC << "setPosition failed condition 2\n";
        return false;
    }

    // Convert the point to an absolute Z if necessry. If we don't have
    // a terrain, skip and hope for the best.
    if (terrain.valid())
    {
        result = p.makeAbsolute(terrain.get()) && result;
    }

    // Is this is a relative-Z position, we need to install a terrain callback
    // so we can recompute the altitude when new terrain tiles become available.
    if (_position.altitudeMode() == ALTMODE_RELATIVE &&
        _autoRecomputeHeights &&
        !_terrainCallbackInstalled &&
        terrain.valid())
    {
        // The Adapter template auto-destructs, so we never need to remove it manually.
        if (!_terrainCallback.valid())
            _terrainCallback = new TerrainCallbackAdapter<GeoTransform>(this);
        terrain->addTerrainCallback(_terrainCallback);
        _terrainCallbackInstalled = true;
    }

    // Finally, assemble the matrix from our position point.
    osg::Matrixd local2world;
    p.createLocalToWorld( local2world );
    this->setMatrix( local2world );

    return true;
}

void
GeoTransform::onTileUpdate(const TileKey&          key,
                          osg::Node*              node,
                          TerrainCallbackContext& context)
{
    if (!_clampInUpdateTraversal)
    {
       if (!_position.isValid() || _position.altitudeMode() != ALTMODE_RELATIVE || !_autoRecomputeHeights)
       {
           OE_TEST << LC << "onTileUpdate fail condition 1\n";
           return;
       }

       if (key.valid() && !key.getExtent().contains(_position))
       {
           OE_TEST << LC << "onTileUpdate fail condition 2\n";
           return;
       }

       _clampInUpdateTraversal = true;
       ADJUST_UPDATE_TRAV_COUNT(this, +1);
    }
}

void
GeoTransform::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.UPDATE_VISITOR)
    {
        if (_findTerrainInUpdateTraversal)
        {
            osg::ref_ptr<MapNode> mapNode;
            if (ObjectStorage::get(&nv, mapNode) && mapNode.valid())
            {
                _findTerrainInUpdateTraversal = false;
                ADJUST_UPDATE_TRAV_COUNT(this, -1);
                setTerrain(mapNode->getTerrain());
            }
        }

        if (_clampInUpdateTraversal)
        {
            setPosition(_position);
            _clampInUpdateTraversal = false;
            ADJUST_UPDATE_TRAV_COUNT(this, -1);
        }
    }

    osg::MatrixTransform::traverse(nv);
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
