/* osgEarth
* Copyright 2008-2014 Pelican Mapping
* MIT License
*/
#include <osgEarth/TerrainTileModel>
#include <osgUtil/IncrementalCompileOperation>

using namespace osgEarth;

#undef  LC
#define LC "[TerrainTileModel] "

namespace
{
    // adapter that lets us compile a Texture::Ptr using the ICO
    struct TextureICOAdapter : public osg::Texture2D
    {
        osgEarth::Texture::WeakPtr _tex_weak;
        osg::observer_ptr<osg::Object> _token;
        bool _hasToken = false;

        TextureICOAdapter(osgEarth::Texture::Ptr value, osg::Object* cancelation_token) :
            _tex_weak(value),
            _token(cancelation_token),
            _hasToken(cancelation_token != nullptr) { }

        // apply is called by the ICO for textures (not compileGLObjects)
        void apply(osg::State& state) const override
        {
            // cancelation check:
            if (_hasToken && !_token.valid())
            {
                //OE_WARN << "Canceled ICO for " << _tex->name() << std::endl;
                return;
            }

            osgEarth::Texture::Ptr tex = _tex_weak.lock();
            if (tex)
            {
                if (tex->compileGLObjects(state))
                {
                    //OE_WARN << "Compiled ICO = " << _tex->name() << (std::uintptr_t)_tex.get() << std::endl;
                }
            }
        }
    };
}

//...................................................................

void
TerrainTileModel::getStateToCompile(osgUtil::StateToCompile& out, bool bindless, osg::Object* token) const
{
    for (auto& colorLayer : colorLayers)
    {
        if (colorLayer.texture)
        {
            out._textures.insert(bindless ?
                new TextureICOAdapter(colorLayer.texture, token) :
                colorLayer.texture->osgTexture().get());
        }
    }

    if (normalMap.texture)
    {
        out._textures.insert(bindless ?
            new TextureICOAdapter(normalMap.texture, token) :
            normalMap.texture->osgTexture().get());
    }

    if (elevation.texture)
    {
        out._textures.insert(bindless ?
            new TextureICOAdapter(elevation.texture, token) :
            elevation.texture->osgTexture().get());
    }

    if (landCover.texture)
    {
        out._textures.insert(bindless ?
            new TextureICOAdapter(landCover.texture, token) :
            landCover.texture->osgTexture().get());
    }
}

Texture::Ptr
TerrainTileModel::getTexture(UID layerUID) const
{
    for (auto& colorLayer : colorLayers)
        if (colorLayer.layer && colorLayer.layer->getUID() == layerUID)
            return colorLayer.texture;

    return nullptr;
}

const osg::Matrixf&
TerrainTileModel::getMatrix(UID layerUID) const
{
    static osg::Matrixf s_identity;

    for (auto& colorLayer : colorLayers)
        if (colorLayer.layer && colorLayer.layer->getUID() == layerUID)
            return colorLayer.matrix;

    return s_identity;
}
