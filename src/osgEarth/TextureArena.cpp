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
    //OE_GL_ZONE_NAMED("oe tex compile");

    OE_SOFT_ASSERT_AND_RETURN(_image.valid(), void(), "Tried to compile a null texture, stop that");

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
        _mipmap ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR,
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

    // Force creation of the bindless handle - once you do this, you can
    // no longer change the texture parameters.
    gc._gltexture->handle(state);

    // debugging
    OE_DEVEL << LC 
        << "Texture::compileGLObjects '" << gc._gltexture->id() 
        << "' name=" << gc._gltexture->name() 
        << " handle=" << gc._gltexture->handle(state) << std::endl;

    bool compressed = _image->isCompressed();

    glPixelStorei(GL_UNPACK_ALIGNMENT, _image->getPacking());
    glPixelStorei(GL_UNPACK_ROW_LENGTH, _image->getRowLength());

    GLsizei width = _image->s();
    GLsizei height = _image->t();

    // Iterate over the in-memory mipmap levels in this layer
    // and download each one
    for (unsigned mipLevel = 0; mipLevel < numMipLevelsInMemory; ++mipLevel)
    {
        // Note: getImageSizeInBytes() will return the actual data size 
        // even if the data is compressed.
        GLsizei mipmapBytes = _image->getImageSizeInBytes() >> (2*mipLevel);

        if (compressed)
        {
            GLsizei blockSize; // unused

            osg::Texture::getCompressedSize(
                _image->getInternalTextureFormat(),
                width, height, 1,
                blockSize, mipmapBytes);
        }
        else
        {
            mipmapBytes = _image->getImageSizeInBytes() >> (2 * mipLevel);
        }


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
                        width, height,
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
                        width, height,
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
                        width, height,
                        _image->getInternalTextureFormat(),
                        mipmapBytes,
                        dataptr );
                }
                else
                {
                    gc._gltexture->subImage2D(
                        mipLevel, // mip level
                        0, 0, // xoffset, yoffset
                        width, height,
                        _image->getPixelFormat(),
                        _image->getDataType(),
                        dataptr );
                }
            }
        }

        width >>= 1;
        if (width < 1) width = 1;
        height >>= 1;
        if (height < 1) height = 1;
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
TextureArena::find_no_lock(Texture::Ptr tex) const
{
    if (tex == nullptr)
        return -1;

    for (int i = 0; i < _textures.size(); ++i)
    {
        if (_textures[i] == tex)
        {
            return i;
        }
    }
    return -1;
}

int
TextureArena::find(Texture::Ptr tex) const
{
    ScopedMutexLock lock(_m);
    return find_no_lock(tex);
}

Texture::Ptr
TextureArena::find(unsigned index) const
{
    ScopedMutexLock lock(_m);
    if (index >= _textures.size())
        return nullptr;

    return _textures[index];
}

int
TextureArena::add(Texture::Ptr tex)
{
    if (tex == nullptr)
    {
        OE_SOFT_ASSERT_AND_RETURN(tex != nullptr, -1);
    }

    // First check whether it's already there; if so, return the index.
    int existingIndex = find(tex);
    if (existingIndex >= 0)
        return existingIndex;

    // not there - load and prepare the texture while keeping 
    // the arena UNLOCKED
    OE_PROFILING_ZONE;

    // load the image if necessary
    if (tex->_image.valid() == false &&
        tex->_uri.isSet())
    {
        // TODO support read options for caching
        tex->_image = tex->_uri->getImage(nullptr);
    }

    if (tex->_image.valid())
    {
        // in case we want to cache it later:
        tex->_image->setWriteHint(osg::Image::STORE_INLINE);

        // compress and mipmap:
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
    }
    else
    {
        // is it a pre-existing gltexture? If not, fail.
        if (tex->_gs.size() == 0)
            return -1;
    }

    // Now, lock the repo and find a place for it.
    ScopedMutexLock lock(_m);

    int index = -1;

    // find an open slot if one is available:
    if (_autoRelease == true)
    {
        for (int i = 0; i < _textures.size(); ++i)
        {
            if (_textures[i] == nullptr || _textures[i].use_count() == 1)
            {
                index = i;
                break;
            }
        }
    }

    // no slot, stick it at the end.
    if (index < 0)
    {
        index = _textures.size();
    }

    // add to all existing GCs:
    for(unsigned i=0; i<_gc.size(); ++i)
    {
        if (_gc[i]._inUse)
            _gc[i]._toCompile.push(index);
    }

    if (index < _textures.size())
        _textures[index] = tex;
    else
        _textures.push_back(tex);

    return index;
//    }

#if 0
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
                _gc[i]._toCompile.push(index);
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
#endif
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
        while (!gc._toCompile.empty())
            gc._toCompile.pop();
        //gc._toCompile.clear();
        for (unsigned i = 0; i < _textures.size(); ++i) {
            if (_textures[i])
                gc._toCompile.push(i);
        }
    }

    // allocate textures and resident handles
    while(!gc._toCompile.empty())
    {
        int index = gc._toCompile.front();
        gc._toCompile.pop();
        auto tex = _textures[index];
        if (tex)
        {
            if (!tex->isCompiled(state))
            {
                tex->compileGLObjects(state);
                gc._toActivate.push_back(tex);
            }
        }
    }
    //gc._toCompile.clear();

    // remove pending objects by swapping them out of memory
    for(auto& tex : gc._toDeactivate)
    {
        if (tex)
            tex->makeResident(state, false);
    }
    gc._toDeactivate.clear();

    // add pending textures by swapping them in to memory
    if (!gc._toActivate.empty())
    {
        for(auto& tex : gc._toActivate)
        {
            if (tex)
                tex->makeResident(state, true);
        }
        gc._toActivate.clear();
        gc._dirty = true;
    }

    if (_autoRelease == true)
    {
        for (Texture::Ptr& tex : _textures)
        {
            if (tex && tex.use_count() == 1)
            {
                // TODO: remove it from the arena altogether
                tex->makeResident(state, false);
                tex->releaseGLObjects(&state);
                tex = nullptr;
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
        //gc._handles.resize(_textures.size());
        gc._handles.assign(_textures.size(), 0);
        gc._dirty = true;
    }

    unsigned ptr = 0;
    for (auto& tex : _textures)
    {
        GLTexture* gltex = tex ? tex->_gs[state.getContextID()]._gltexture.get() : nullptr;
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
        gc._handleBuffer->uploadData(gc._handles);
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
        if (tex)
            tex->resizeGLObjectBuffers(maxSize);
    }
}

void
TextureArena::releaseGLObjects(osg::State* state) const
{
    ScopedMutexLock lock(_m);

    if (state)
    {
        GCState& gs = _gc[state->getContextID()];
        gs._handleBuffer = nullptr;
        while (!gs._toCompile.empty())
            gs._toCompile.pop();
        //gs._toCompile.clear();
        for (unsigned i = 0; i < _textures.size(); ++i)
        {
            if (_textures[i])
            {
                _textures[i]->releaseGLObjects(state);
                gs._toCompile.push(i);
            }
        }
    }
    else
    {
        for (auto& tex : _textures)
        {
            if (tex)
                tex->releaseGLObjects(state);
        }

        for (unsigned i = 0; i < _gc.size(); ++i)
        {
            GCState& gs = _gc[i];
            if(gs._inUse)
            {
                gs._handleBuffer = nullptr;
                while (!gs._toCompile.empty())
                    gs._toCompile.pop();
                for (unsigned i = 0; i < _textures.size(); ++i)
                {
                    if (_textures[i])
                        gs._toCompile.push(i);
                }
            }
        }
    }
}
