/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2020 Pelican Mapping
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
#include "TextureArena"
#include <osgEarth/ImageUtils>
#include <osgEarth/Math>
#include <osgEarth/Metrics>
#include <osgViewer/View>
#include <osg/State>

// osg 3.6:
#ifndef GL_TEXTURE_2D_ARRAY
#define GL_TEXTURE_2D_ARRAY 0x8C1A
#endif

using namespace osgEarth;

#undef LC
#define LC "[Texture] "

#define OE_DEVEL OE_DEBUG

//#define USE_ICO 1

Texture::Ptr
Texture::create(GLTexture::Ptr gltexture, osg::State& state)
{
    Texture::Ptr object(new Texture());
    GCState& gs = object->_gs[state.getContextID()];
    return object;
}

Texture::GCState&
Texture::get(const osg::State& state) const
{
    return _gs[state.getContextID()];
}

bool
Texture::isCompiled(const osg::State& state) const
{
    return _gs[state.getContextID()]._gltexture != nullptr;
}

void
Texture::compileGLObjects(osg::State& state) const
{
    OE_PROFILING_ZONE;

    OE_HARD_ASSERT(_image.valid());

    osg::GLExtensions* ext = state.get<osg::GLExtensions>();
    Texture::GCState& gc = get(state);

    // If you change this you must change the typecast in the fragment shader too
    //GLenum target = GL_TEXTURE_2D_ARRAY;
    GLenum target = GL_TEXTURE_2D;

    // mipmaps already created and in the image:
    unsigned numMipLevelsInMemory = _image->getNumMipmapLevels();

    // how much space we need to allocate on the GPU:
    unsigned numMipLevelsToAllocate = numMipLevelsInMemory;

    if (numMipLevelsInMemory <= 1 && _mipmap == true)
    {
        numMipLevelsToAllocate = osg::Image::computeNumberOfMipmapLevels(
            _image->s(), _image->t(), _image->t());
    }
    
    GLenum pixelFormat = _image->getPixelFormat();

    GLenum internalFormat =
        _image->isCompressed() ? _image->getInternalTextureFormat() :
        _internalFormat.isSet() ? _internalFormat.get() :
        pixelFormat == GL_RED ? GL_R32F :
        pixelFormat == GL_RG ? GL_RG8 :
        pixelFormat == GL_RGB ? GL_RGB8 :
        GL_RGBA8;

    if (_compress)
    {
        if (pixelFormat == GL_RGB)
            internalFormat = GL_COMPRESSED_RGB_S3TC_DXT1_EXT;
        else if (pixelFormat == GL_RGBA)
            internalFormat = GL_COMPRESSED_RGBA_S3TC_DXT5_EXT;
    }

    // Calculate the size beforehand so we can make the texture recyclable
    GLTexture::Profile profileHint(
        target,
        numMipLevelsToAllocate, 
        internalFormat,
        _image->s(), _image->t(), _image->r(),
        0, // border
        _mipmap ? GL_LINEAR_MIPMAP_LINEAR : GL_NEAREST,
        GL_LINEAR,
        _clamp ? GL_CLAMP_TO_EDGE : GL_REPEAT,
        _clamp ? GL_CLAMP_TO_EDGE : GL_REPEAT,
        _clamp ? GL_CLAMP_TO_EDGE : GL_REPEAT,
        _maxAnisotropy.getOrUse(4.0f));

    gc._gltexture = GLTexture::create(
        target,
        state,
        profileHint,
        _label.empty() ? _uri->base() : _label);

    // debugging
    gc._gltexture->id() = _uri->base();

    // Blit our image to the GPU
    gc._gltexture->bind(state);

    if (target == GL_TEXTURE_2D_ARRAY)
    {
        gc._gltexture->storage3D(profileHint);
    }
    else
    {
        gc._gltexture->storage2D(profileHint);
    }

#if 0
    GLint min_filter = _mipmap ? GL_LINEAR_MIPMAP_LINEAR : GL_NEAREST;
    glTexParameteri(target, GL_TEXTURE_MIN_FILTER, min_filter);
    glTexParameteri(target, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    GLint wrap_value = _clamp ? GL_CLAMP_TO_EDGE : GL_REPEAT;
    glTexParameteri(target, GL_TEXTURE_WRAP_S, wrap_value);
    glTexParameteri(target, GL_TEXTURE_WRAP_T, wrap_value);

    float ma_value = _maxAnisotropy.getOrUse(4.0f);
    glTexParameterf(target, GL_TEXTURE_MAX_ANISOTROPY_EXT, ma_value);
#endif

    // Force creation of the bindless handle - once you do this, you can
    // no longer change the texture parameters.
    gc._gltexture->handle(state);

    // debugging
    OE_DEVEL << LC 
        << "Texture::compileGLObjects '" << gc._gltexture->id() 
        << "' name=" << gc._gltexture->name() 
        << " handle=" << gc._gltexture->handle(state) << std::endl;

    // TODO: At this point, if/when we go with SPARSE textures,
    // don't actually copy the image down until activation.

    bool compressed = _image->isCompressed();

    // Iterate over the in-memory mipmap levels in this layer
    // and download each one
    for (unsigned mipLevel = 0; mipLevel < numMipLevelsInMemory; ++mipLevel)
    {
        // Note: getImageSizeInBytes() will return the actual data size 
        // even if the data is compressed.
        int mipmapBytes = _image->getImageSizeInBytes() >> (2*mipLevel);

        // Iterate over image slices:
        for (int r = 0; r < _image->r(); ++r)
        {
            if (target == GL_TEXTURE_2D_ARRAY)
            {
                unsigned char* dataptr =
                    _image->getMipmapData(mipLevel) +
                    mipmapBytes * r;

                if (compressed)
                {
                    gc._gltexture->compressedSubImage3D(
                        mipLevel,
                        0, 0, // xoffset, yoffset
                        r, // zoffset (array layer)
                        _image->s() >> mipLevel, // width at mipmap level i
                        _image->t() >> mipLevel, // height at mipmap level i
                        1, // z size always = 1
                        _image->getInternalTextureFormat(),
                        mipmapBytes,
                        dataptr);
                }
                else
                {
                    gc._gltexture->subImage3D(
                        mipLevel, // mip level
                        0, 0, // xoffset, yoffset
                        r, // zoffset (array layer)
                        _image->s() >> mipLevel, // width at mipmap level i
                        _image->t() >> mipLevel, // height at mipmap level i
                        1, // z size always = 1
                        _image->getPixelFormat(),
                        _image->getDataType(),
                        dataptr );
                }
            }
            else
            {
                unsigned char* dataptr =
                    _image->getMipmapData(mipLevel);

                if (compressed)
                {
                    gc._gltexture->compressedSubImage2D(
                        mipLevel, // mip level
                        0, 0, // xoffset, yoffset
                        _image->s() >> mipLevel, // width at mipmap level i
                        _image->t() >> mipLevel, // height at mipmap level i
                        _image->getInternalTextureFormat(),
                        mipmapBytes,
                        dataptr );
                }
                else
                {
                    gc._gltexture->subImage2D(
                        mipLevel, // mip level
                        0, 0, // xoffset, yoffset
                        _image->s() >> mipLevel, // width at mipmap level i
                        _image->t() >> mipLevel, // height at mipmap level i
                        _image->getPixelFormat(),
                        _image->getDataType(),
                        dataptr );
                }
            }
        }
    }

    if (numMipLevelsInMemory < numMipLevelsToAllocate)
    {
        OE_PROFILING_ZONE_NAMED("glGenerateMipmap");
        ext->glGenerateMipmap(target);
    }
}

void
Texture::makeResident(const osg::State& state, bool toggle) const
{
    GCState& gc = get(state);

    if (gc._gltexture != nullptr)
    {
        gc._gltexture->makeResident(toggle);

        OE_DEVEL << LC
            << "Texture::makeResident '" << gc._gltexture->id()
            << "' name=" << gc._gltexture->name() << std::endl;
    }
}

bool
Texture::isResident(const osg::State& state) const
{
    GCState& gc = get(state);
    return (gc._gltexture != nullptr && gc._gltexture->isResident());
}

void
Texture::resizeGLObjectBuffers(unsigned maxSize)
{
    if (_gs.size() < maxSize)
        _gs.resize(maxSize);
}

void
Texture::releaseGLObjects(osg::State* state) const
{
    if (state)
    {
        if (_gs[state->getContextID()]._gltexture != nullptr)
        {
            GCState& gs = get(*state);

            // debugging
            OE_DEVEL << LC 
                << "Texture::releaseGLObjects '" << gs._gltexture->id() 
                << "' name=" << gs._gltexture->name() 
                << " handle=" << gs._gltexture->handle(*state) << std::endl;

            //gc._gltexture->release();
            gs._gltexture->makeResident(false);

            // will activate the releaser
            gs._gltexture = nullptr;
        }
    }
    else
    {
        // rely on the Releaser to get around to it
        for(unsigned i=0; i< _gs.size(); ++i)
        {
            // will activate the releaser(s)
            _gs[i]._gltexture = nullptr;
        }
    }
}


//...................................................................

#undef LC
#define LC "[TextureArena] "


TextureArena::TextureArena() :
    _autoRelease(false),
    _bindingPoint(5u),
    _useUBO(false)
{
    // Keep this synchronous w.r.t. the render thread since we are
    // giong to be changing things on the fly
    setDataVariance(DYNAMIC);
}

TextureArena::~TextureArena()
{
    releaseGLObjects(nullptr);
}

void
TextureArena::setAutoRelease(bool value)
{
    _autoRelease = value;
}

void
TextureArena::setBindingPoint(unsigned value)
{
    _bindingPoint = value;
}

int
TextureArena::find(Texture::Ptr tex) const
{
    for (int i = 0; i < _textures.size(); ++i)
        if (_textures[i] == tex)
            return i;
    return -1;
}

int
TextureArena::add(Texture::Ptr tex)
{
    ScopedMutexLock lock(_m);

    if (tex == nullptr)
    {
        OE_SOFT_ASSERT_AND_RETURN(tex != nullptr, -1);
    }

    // if it's already there, we good
    int index = find(tex);
    if (index >= 0)
        return index;

    // find an open slot if one is available:
    if (_autoRelease == true)
    {
        for (int i = 0; i < _textures.size(); ++i)
        {
            if (_textures[i].use_count() == 1)
            {
                index = i;
                break;
            }
        }
    }

    if (index < 0)
    {
        index = _textures.size();
    }

    // load the image if necessary (TODO: background?)
    if (tex->_image.valid() == false &&
        tex->_uri.isSet())
    {
        // TODO support read options for caching
        tex->_image = tex->_uri->getImage(nullptr);
    }

    if (tex->_image.valid())
    {
        tex->_image->setWriteHint(osg::Image::STORE_INLINE);

        if (!tex->_image->isCompressed())
        {
            if (tex->_image->getPixelFormat() == tex->_image->getInternalTextureFormat())
            {
                // normalize the internal texture format
                GLenum internalFormat =
                    tex->_internalFormat.isSet() ? tex->_internalFormat.get() :
                    tex->_image->getPixelFormat() == GL_RED ? GL_R32F :
                    tex->_image->getPixelFormat() == GL_RG ? GL_RG8 :
                    tex->_image->getPixelFormat() == GL_RGB ? GL_RGB8 :
                    tex->_image->getPixelFormat() == GL_RGBA ? GL_RGBA8 :
                    GL_RGBA8;

                tex->_image->setInternalTextureFormat(internalFormat);
            }

            if (tex->_compress)
            {
                ImageUtils::compressImageInPlace(tex->_image.get());
            }
        }

        // add to all GCs.
        for(unsigned i=0; i<_gc.size(); ++i)
        {
            if (_gc[i]._inUse)
                _gc[i]._toAdd.push_back(tex);
        }

        if (index < _textures.size())
            _textures[index] = tex;
        else
            _textures.push_back(tex);

        return index;
    }

    else
    {
        // might be a pre-existing GLTexture:
        bool added = false;

        for (unsigned i = 0; i < _gc.size(); ++i)
        {
            if (_gc[i]._inUse && 
                tex->_gs.size() > i &&
                tex->_gs[i]._gltexture != nullptr)
            {
                _gc[i]._toAdd.push_back(tex);
                added = true;
            }
        }

        if (added)
        {
            if (index < _textures.size())
                _textures[index] = tex;
            else
                _textures.push_back(tex);

            return index;
        }
    }

    // nope...fail
    return -1;
}

void
TextureArena::activate(Texture::Ptr tex)
{
    ScopedMutexLock lock(_m);

    if (!tex) return;

    // add to all GCs.
    for(unsigned i=0; i<_gc.size(); ++i)
    {
        _gc[i]._toActivate.push_back(tex);
    }

    //TODO: consider issues like multiple GCs and "unref after apply"
}

void
TextureArena::deactivate(Texture::Ptr tex)
{
    ScopedMutexLock lock(_m);

    if (!tex) return;

    // add to all GCs.
    for(unsigned i=0; i<_gc.size(); ++i)
    {
        _gc[i]._toDeactivate.push_back(tex);
    }
}

namespace
{
    struct TextureCompileOp : public osgUtil::IncrementalCompileOperation::CompileOp
    {
        Texture::Ptr _tex;

        TextureCompileOp(Texture::Ptr tex) : _tex(tex) { }

        // How many seconds we expect the operation to take. Educated guess.
        double estimatedTimeForCompile(osgUtil::IncrementalCompileOperation::CompileInfo& compileInfo) const {
            return 0.1;
        }
        bool compile(osgUtil::IncrementalCompileOperation::CompileInfo& compileInfo) {
            OE_PROFILING_ZONE_NAMED("TextureCompileOp::compile");
            _tex->compileGLObjects(*compileInfo.getState());
            return true;
        }
    };
}

void
TextureArena::apply(osg::State& state) const
{
    if (_textures.empty())
        return;

    ScopedMutexLock lock(_m);

    OE_PROFILING_ZONE;

    GCState& gc = _gc[state.getContextID()];

    // first time seeing this GC? Prime it by adding all textures!
    if (gc._inUse == false)
    {
        gc._inUse = true;
        gc._toAdd.resize(_textures.size());
        std::copy(_textures.begin(), _textures.end(), gc._toAdd.begin());
    }

#ifdef USE_ICO
    osgUtil::IncrementalCompileOperation* ico = nullptr;

    if (!gc._toAdd.empty())
    {
        const osg::Camera* camera = state.getGraphicsContext()->getCameras().front();
        const osgViewer::View* osgView = dynamic_cast<const osgViewer::View*>(camera->getView());
        if (osgView)
            ico = const_cast<osgViewer::View*>(osgView)->getDatabasePager()->getIncrementalCompileOperation();
    }
#endif

    // allocate textures and resident handles

    TextureVector waitingToCompile;

    const unsigned max_to_compile_per_apply = ~0;
    unsigned num_compiled_this_apply = 0;

    for(auto& tex : gc._toAdd)
    {
        if (!tex->isCompiled(state))
        {
#ifdef USE_ICO
            if (ico)
            {
                Texture::GCState& tex_gc = tex->get(state);

                if (!tex_gc._compileSet.valid())
                {
                    tex_gc._compileSet = new osgUtil::IncrementalCompileOperation::CompileSet();
                    tex_gc._compileSet->_compileMap[state.getGraphicsContext()].add(new TextureCompileOp(tex));
                    ico->add(tex_gc._compileSet.get());
                }

                waitingToCompile.push_back(tex);
            }
            else
            {
                tex->compileGLObjects(state);
                gc._toActivate.push_back(tex);
            }
#else
            if (num_compiled_this_apply < max_to_compile_per_apply)
            {
                tex->compileGLObjects(state);
                ++num_compiled_this_apply;
                gc._toActivate.push_back(tex);
            }
            else
            {
                waitingToCompile.push_back(tex);
            }
#endif
        }
        else
        {
            gc._toActivate.push_back(tex);
        }
    }
    gc._toAdd.swap(waitingToCompile);

    // remove pending objects by swapping them out of memory
    for(auto& tex : gc._toDeactivate)
    {
        tex->makeResident(state, false);
    }
    gc._toDeactivate.clear();

    // add pending textures by swapping them in to memory
    if (!gc._toActivate.empty())
    {
        for(auto& tex : gc._toActivate)
        {
            tex->makeResident(state, true);
        }
        gc._toActivate.clear();
        gc._dirty = true;
    }

    if (_autoRelease == true)
    {
        for (auto& tex : _textures)
        {
            if (tex.use_count() == 1)
            {
                // TODO: remove it from the area altogether
                tex->makeResident(state, false);
                tex->releaseGLObjects(&state);
            }
        }
    }

    if (gc._handleBuffer == nullptr)
    {
        std::string bufferName = "TextureArena " + getName();

        if (_useUBO)
            gc._handleBuffer = GLBuffer::create(GL_UNIFORM_BUFFER, state, bufferName);
        else
            gc._handleBuffer = GLBuffer::create(GL_SHADER_STORAGE_BUFFER, state, bufferName);

        gc._dirty = true;
    }

    // refresh the handles buffer if necessary:
    if (_textures.size() > gc._handles.size())
    {
        gc._handles.resize(_textures.size());
        gc._dirty = true;
    }

    unsigned ptr = 0;
    for (auto& tex : _textures)
    {
        GLTexture* gltex = tex->_gs[state.getContextID()]._gltexture.get();
        GLuint64 handle = gltex ? gltex->handle(state) : 0ULL;
        if (gc._handles[ptr] != handle)
        {
            gc._handles[ptr] = handle;
            gc._dirty = true;
        }       
        ++ptr;

        if (_useUBO)
            ++ptr; // hack for std140 vec4 alignment
    }

    // upload to GPU if it changed:
    if (gc._dirty)
    {
        gc._handleBuffer->uploadData(
            gc._handles.size() * sizeof(GLuint64),
            gc._handles.data());

        gc._dirty = false;
    }

    gc._handleBuffer->bindBufferBase(_bindingPoint);
}

void
TextureArena::compileGLObjects(osg::State& state) const
{
    OE_DEBUG << LC << "Compiling GL objects for arena " << getName() << std::endl;
    apply(state);
}

void
TextureArena::resizeGLObjectBuffers(unsigned maxSize)
{
    ScopedMutexLock lock(_m);

    if (_gc.size() < maxSize)
    {
        _gc.resize(maxSize);
    }

    for(auto& tex : _textures)
    {
        tex->resizeGLObjectBuffers(maxSize);
    }
}

void
TextureArena::releaseGLObjects(osg::State* state) const
{
    ScopedMutexLock lock(_m);

    for(auto& tex : _textures)
    {
        tex->releaseGLObjects(state);
    }

    if (state)
    {
        //_gc[state->getContextID()]._handleLUT.release();
        _gc[state->getContextID()]._handleBuffer = nullptr;
    }
    else
    {
        for (unsigned i = 0; i < _gc.size(); ++i)
        {
            //_gc[i]._handleLUT.release();
            _gc[i]._handleBuffer = nullptr;
        }
    }
}
