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

namespace
{
    // adapter that lets us compile a Texture::Ptr using the ICO
    struct TextureAdapter : public osg::Texture2D
    {
        osgEarth::Texture::Ptr _tex;
        TextureAdapter(osgEarth::Texture::Ptr value) : _tex(value) { }

        // apply is called by the ICO for textures (not compileGLObjects)
        void apply(osg::State& state) const override {
            if (_tex)
                _tex->compileGLObjects(state);
        }
    };
}

//...................................................................

TerrainTileModel::Elevation::Elevation() :
    _minHeight(FLT_MAX),
    _maxHeight(-FLT_MAX)
{
    //nop
}

//...................................................................

TerrainTileModel::TerrainTileModel(
    const TileKey&  key,
    const Revision& revision) :
    _key(key),
    _revision(revision),
    _requiresUpdateTraversal(false)
{
    // nop
}

void
TerrainTileModel::getStateToCompile(
    osgUtil::StateToCompile& out,
    bool bindless) const
{
    for (auto& colorLayer : colorLayers())
    {
        if (colorLayer.texture())
        {
            out._textures.insert(bindless ?
                new TextureAdapter(colorLayer.texture()) :
                colorLayer.texture()->osgTexture().get());
        }
    }

    if (normalMap().texture())
    {
        out._textures.insert(bindless ?
            new TextureAdapter(normalMap().texture()) :
            normalMap().texture()->osgTexture().get());
    }

    if (elevation().texture())
    {
        out._textures.insert(bindless ?
            new TextureAdapter(elevation().texture()) :
            elevation().texture()->osgTexture().get());
    }

    if (landCover().texture())
    {
        out._textures.insert(bindless ?
            new TextureAdapter(landCover().texture()) :
            landCover().texture()->osgTexture().get());
    }
}

Texture::Ptr
TerrainTileModel::getTexture(UID layerUID) const
{
    for (auto& colorLayer : colorLayers())
        if (colorLayer.layer() && colorLayer.layer()->getUID() == layerUID)
            return colorLayer.texture();

    return nullptr;
}

const osg::Matrixf&
TerrainTileModel::getMatrix(UID layerUID) const
{
    static osg::Matrixf s_identity;

    for (auto& colorLayer : colorLayers())
        if (colorLayer.layer() && colorLayer.layer()->getUID() == layerUID)
            return colorLayer.matrix();

    return s_identity;
}
