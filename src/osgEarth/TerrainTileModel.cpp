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
#include <osgEarth/TerrainTileModel>
#include <osgUtil/IncrementalCompileOperation>

using namespace osgEarth;

#undef  LC
#define LC "[TerrainTileModel] "

//...................................................................

TerrainTileLayerModel::TerrainTileLayerModel() :
    _revision(-1)
{
}

//...................................................................

TerrainTileElevationModel::TerrainTileElevationModel() :
_minHeight( FLT_MAX ),
_maxHeight(-FLT_MAX )
{
    //NOP
}

//...................................................................

TerrainTileModel::TerrainTileModel(const TileKey&  key,
                                   const Revision& revision) :
_key                   ( key ),
_revision              ( revision ),
_requiresUpdateTraverse( false )
{
    //NOP
}

osg::Texture* 
TerrainTileModel::getNormalTexture() const
{
    return _normalLayer.valid() ? _normalLayer->getTexture() : 0L;
}

osg::RefMatrixf* 
TerrainTileModel::getNormalTextureMatrix() const
{
    return _normalLayer.valid() ? _normalLayer->getMatrix() : 0L;
}

osg::Texture* 
TerrainTileModel::getElevationTexture() const
{
    return _elevationLayer.valid() ? _elevationLayer->getTexture() : 0L;
}

osg::RefMatrixf* 
TerrainTileModel::getElevationTextureMatrix() const
{
    return _elevationLayer.valid() ? _elevationLayer->getMatrix() : 0L;
}

osg::Texture* 
TerrainTileModel::getLandCoverTexture() const
{
    return _landCoverLayer.valid() ? _landCoverLayer->getTexture() : 0L;
}

osg::RefMatrixf* 
TerrainTileModel::getLandCoverTextureMatrix() const
{
    return _landCoverLayer.valid() ? _landCoverLayer->getMatrix() : 0L;
}

void
TerrainTileModel::compileGLObjects(osg::State& state) const
{
    for (TerrainTileColorLayerModelVector::const_iterator i = _colorLayers.begin();
        i != _colorLayers.end();
        ++i)
    {
        if (i->get()->getTexture())
            i->get()->getTexture()->compileGLObjects(state);
    }

    // since non-core shared layers are ALSO in the colorLayers vector,
    // there is no need to iterator over them.

    if (getNormalTexture())
        getNormalTexture()->compileGLObjects(state);

    if (getElevationTexture())
        getElevationTexture()->compileGLObjects(state);

    if (getLandCoverTexture())
        getLandCoverTexture()->compileGLObjects(state);
}

void
TerrainTileModel::getStateToCompile(osgUtil::StateToCompile& out) const
{
    for(auto& colorLayer : _colorLayers)
        if (colorLayer->getTexture())
            out._textures.insert(colorLayer->getTexture());

    if (getNormalTexture())
        out._textures.insert(getNormalTexture());

    if (getElevationTexture())
        out._textures.insert(getElevationTexture());

    if (getLandCoverTexture())
        out._textures.insert(getLandCoverTexture());
}

osg::Texture*
TerrainTileModel::getTexture(UID layerUID) const
{
    for(unsigned i=0; i<colorLayers().size(); ++i)
        if (colorLayers()[i]->getLayer()->getUID() == layerUID)
            return colorLayers()[i]->getTexture();
    return NULL;
}

osg::RefMatrixf* 
TerrainTileModel::getMatrix(UID layerUID) const
{
    for (unsigned i = 0; i < colorLayers().size(); ++i)
        if (colorLayers()[i]->getLayer()->getUID() == layerUID)
            return colorLayers()[i]->getMatrix();
    return NULL;
}
