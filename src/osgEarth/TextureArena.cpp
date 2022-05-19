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
#include "Registry"
#include "ImageUtils"
#include "Math"
#include "Metrics"

#include <osg/State>
#include <osg/Texture1D>
#include <osg/Texture2D>
#include <osg/Texture3D>
#include <osg/Texture2DArray>
#include <osgUtil/IncrementalCompileOperation>

// osg 3.6:
#ifndef GL_TEXTURE_2D_ARRAY
#define GL_TEXTURE_2D_ARRAY 0x8C1A
#endif

using namespace osgEarth;

#undef LC
#define LC "[Texture] "

#define OE_DEVEL OE_DEBUG

Texture::Ptr
Texture::create(osg::Image* image, GLenum target)
{
    Texture::Ptr object(new Texture(target));
    if (image)
        object->osgTexture()->setImage(0, image);
    return object;
}

Texture::Ptr
Texture::create(osg::Texture* input)
{
    OE_HARD_ASSERT(input);
    return Texture::Ptr(new Texture(input));
}

Texture::Texture(GLenum target_) :
    _globjects(16),
    _compress(true),
    _mipmap(true),
    _clamp(false),
    _keepImage(true),
    _target(target_)
{
    // nop
    if (target() == GL_TEXTURE_1D)
        osgTexture() = new osg::Texture1D();
    else if (target() == GL_TEXTURE_2D)
        osgTexture() = new osg::Texture2D();
    else if (target() == GL_TEXTURE_3D)
        osgTexture() = new osg::Texture3D();
    else if (target() == GL_TEXTURE_2D_ARRAY)
        osgTexture() = new osg::Texture2DArray();
    else {
        OE_HARD_ASSERT(false, "Invalid texture target");
    }
    if (osgTexture().valid())
        osgTexture()->setUnRefImageDataAfterApply(
            Registry::instance()->unRefImageDataAfterApply().get());

    keepImage() = !Registry::instance()->unRefImageDataAfterApply().get();
}

Texture::Texture(osg::Texture* input) :
    _globjects(16),
    _compress(false),
    _osgTexture(input)
{
    target() = input->getTextureTarget();

    mipmap() =
        input->getFilter(osg::Texture::MIN_FILTER) == osg::Texture::LINEAR_MIPMAP_LINEAR ||
        input->getFilter(osg::Texture::MIN_FILTER) == osg::Texture::LINEAR_MIPMAP_NEAREST ||
        input->getFilter(osg::Texture::MIN_FILTER) == osg::Texture::NEAREST_MIPMAP_LINEAR ||
        input->getFilter(osg::Texture::MIN_FILTER) == osg::Texture::NEAREST_MIPMAP_NEAREST;

    clamp() =
        input->getWrap(osg::Texture::WRAP_S) == osg::Texture::CLAMP ||
        input->getWrap(osg::Texture::WRAP_S) == osg::Texture::CLAMP_TO_EDGE;

    maxAnisotropy() =
        input->getMaxAnisotropy();

    if (input->getInternalFormatMode() != osg::Texture::USE_IMAGE_DATA_FORMAT)
        internalFormat() = input->getInternalFormat();

    // pick a name.
    name() = input->getName();
    if (name().empty() && dataLoaded())
        name() = input->getImage(0)->getFileName();

    if (dataLoaded())
    {
        uri() = URI(input->getImage(0)->getFileName());
    }

    keepImage() = !Registry::instance()->unRefImageDataAfterApply().get();
}

Texture::~Texture()
{
    //nop
}

bool
Texture::isCompiled(const osg::State& state) const
{
    return GLObjects::get(_globjects, state)._gltexture != nullptr;
}

bool
Texture::needsCompile(const osg::State& state) const
{
    auto& gc = GLObjects::get(_globjects, state);

    bool hasData = dataLoaded();

    if (gc._gltexture == nullptr && hasData == true)
        return true;

    if (hasData == false)
        return false;

    return (osgTexture()->getImage(0)->getModifiedCount() != gc._imageModCount);
}

bool
Texture::needsUpdates() const
{
    return
        dataLoaded() &&
        osgTexture()->getImage(0)->requiresUpdateCall();
}

void
Texture::update(osg::NodeVisitor& nv)
{
    osgTexture()->getImage(0)->update(&nv);
}

bool
Texture::dataLoaded() const
{
    return
        osgTexture().valid() &&
        osgTexture()->getNumImages() > 0 &&
        osgTexture()->getImage(0) != nullptr;
}

void
Texture::compileGLObjects(osg::State& state) const
{
    if (!needsCompile(state))
        return;

    OE_PROFILING_ZONE;
    OE_HARD_ASSERT(dataLoaded() == true);

    osg::GLExtensions* ext = state.get<osg::GLExtensions>();
    auto& gc = GLObjects::get(_globjects, state);

    auto image = osgTexture()->getImage(0);

    // make sure we need to compile this
    if (gc._gltexture != nullptr)
    {
        // hmm, it's already compiled. Does it need a recompile 
        // because of a modified image?

        if (gc._imageModCount == image->getModifiedCount())
            return; // nope
    }

    if (target() == GL_TEXTURE_2D)
    {
        // mipmaps already created and in the image:
        unsigned numMipLevelsInMemory = image->getNumMipmapLevels();

        // how much space we need to allocate on the GPU:
        unsigned numMipLevelsToAllocate = numMipLevelsInMemory;

        if (numMipLevelsInMemory <= 1 && mipmap() == true)
        {
            numMipLevelsToAllocate = osg::Image::computeNumberOfMipmapLevels(
                image->s(), image->t(), image->t());
        }

        GLenum pixelFormat = image->getPixelFormat();

        GLenum gpuInternalFormat =
            image->isCompressed() ? image->getInternalTextureFormat() :
            internalFormat().isSet() ? internalFormat().get() :
            pixelFormat == GL_RED ? GL_R32F :
            pixelFormat == GL_RG ? GL_RG8 :
            pixelFormat == GL_RGB ? GL_RGB8 :
            GL_RGBA8;

        if (compress())
        {
            if (pixelFormat == GL_RGB)
                gpuInternalFormat = GL_COMPRESSED_RGB_S3TC_DXT1_EXT;
            else if (pixelFormat == GL_RGBA)
                gpuInternalFormat = GL_COMPRESSED_RGBA_S3TC_DXT5_EXT;
        }

        // Calculate the size beforehand so we can make the texture recyclable
        GLTexture::Profile profileHint(
            target(),
            numMipLevelsToAllocate,
            gpuInternalFormat,
            image->s(), image->t(), image->r(),
            0, // border
            mipmap() ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR,
            GL_LINEAR,
            clamp() ? GL_CLAMP_TO_EDGE : GL_REPEAT,
            clamp() ? GL_CLAMP_TO_EDGE : GL_REPEAT,
            clamp() ? GL_CLAMP_TO_EDGE : GL_REPEAT,
            maxAnisotropy().getOrUse(4.0f));

        gc._gltexture = GLTexture::create(
            target(),
            state,
            profileHint);

        OE_SOFT_ASSERT(gc._gltexture->name() != 0, "Oh no, GLTexture name == 0");

        // Blit our image to the GPU
        gc._gltexture->bind(state);

        gc._gltexture->debugLabel(label(), name());

        if (target() == GL_TEXTURE_2D)
        {
            gc._gltexture->storage2D(profileHint);
        }
        else if (target() == GL_TEXTURE_3D || target() == GL_TEXTURE_2D_ARRAY)
        {
            gc._gltexture->storage3D(profileHint);
        }

        // Force creation of the bindless handle - once you do this, you can
        // no longer change the texture parameters.
        gc._gltexture->handle(state);

        // debugging
        OE_DEVEL << LC
            << "Texture::compileGLObjects '" << gc._gltexture->id()
            << "' name=" << gc._gltexture->name()
            << " handle=" << gc._gltexture->handle(state) << std::endl;

        bool compressed = image->isCompressed();

        glPixelStorei(GL_UNPACK_ALIGNMENT, image->getPacking());
        glPixelStorei(GL_UNPACK_ROW_LENGTH, image->getRowLength());

        GLsizei width = image->s();
        GLsizei height = image->t();

        // Iterate over the in-memory mipmap levels in this layer
        // and download each one
        for (unsigned mipLevel = 0; mipLevel < numMipLevelsInMemory; ++mipLevel)
        {
            // Note: getImageSizeInBytes() will return the actual data size 
            // even if the data is compressed.
            GLsizei mipmapBytes = image->getImageSizeInBytes() >> (2 * mipLevel);

            if (compressed)
            {
                GLsizei blockSize; // unused

                osg::Texture::getCompressedSize(
                    image->getInternalTextureFormat(),
                    width, height, 1,
                    blockSize, mipmapBytes);
            }
            else
            {
                mipmapBytes = image->getImageSizeInBytes() >> (2 * mipLevel);
            }

            // Iterate over image slices:
            for (int r = 0; r < image->r(); ++r)
            {
                if (target() == GL_TEXTURE_2D)
                {
                    unsigned char* dataptr =
                        image->getMipmapData(mipLevel);

                    if (compressed)
                    {
                        gc._gltexture->compressedSubImage2D(
                            mipLevel, // mip level
                            0, 0, // xoffset, yoffset
                            width, height,
                            image->getInternalTextureFormat(),
                            mipmapBytes,
                            dataptr);
                    }
                    else
                    {
                        gc._gltexture->subImage2D(
                            mipLevel, // mip level
                            0, 0, // xoffset, yoffset
                            width, height,
                            image->getPixelFormat(),
                            image->getDataType(),
                            dataptr);
                    }
                }
                else if (target() == GL_TEXTURE_2D_ARRAY || target() == GL_TEXTURE_3D)
                {
                    unsigned char* dataptr =
                        image->getMipmapData(mipLevel) +
                        mipmapBytes * r;

                    if (compressed)
                    {
                        gc._gltexture->compressedSubImage3D(
                            mipLevel,
                            0, 0, // xoffset, yoffset
                            r, // zoffset (array layer)
                            width, height,
                            1, // z size always = 1
                            image->getInternalTextureFormat(),
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
                            image->getPixelFormat(),
                            image->getDataType(),
                            dataptr);
                    }
                }
            }

            width >>= 1;
            if (width < 1) width = 1;
            height >>= 1;
            if (height < 1) height = 1;
        }

        // TODO:
        // Detect this situation, and find another place to generate the
        // mipmaps offline. This should never happen here.
        if (numMipLevelsInMemory < numMipLevelsToAllocate)
        {
            OE_PROFILING_ZONE_NAMED("glGenerateMipmap");
            ext->glGenerateMipmap(target());
        }

        if (keepImage() == false)
        {
            const_cast<Texture*>(this)->osgTexture_mutable() = nullptr;
        }

        // finally, make it resident.
        gc._gltexture->makeResident(state, true);
    }

    // sync the mod counts.
    gc._imageModCount = image->getModifiedCount();
}

void
Texture::makeResident(const osg::State& state, bool toggle) const
{
    auto& gc = GLObjects::get(_globjects, state);

    if (gc._gltexture != nullptr)
    {
        gc._gltexture->makeResident(state, toggle);

        OE_DEVEL << LC
            << "Texture::makeResident '" << gc._gltexture->id()
            << "' name=" << gc._gltexture->name() << std::endl;
    }
}

bool
Texture::isResident(const osg::State& state) const
{
    auto& gc = GLObjects::get(_globjects, state);
    return (gc._gltexture != nullptr && gc._gltexture->isResident(state));
}

void
Texture::resizeGLObjectBuffers(unsigned maxSize)
{
    if (_globjects.size() < maxSize)
        _globjects.resize(maxSize);

    if (osgTexture().valid())
        osgTexture()->resizeGLObjectBuffers(maxSize);
}

void
Texture::releaseGLObjects(osg::State* state) const
{
    if (state)
    {
        auto& gc = GLObjects::get(_globjects, *state);
        if (gc._gltexture != nullptr)
        {
            // debugging
            OE_DEVEL << LC 
                << "Texture::releaseGLObjects '" << gc._gltexture->id()
                << "' name=" << gc._gltexture->name()
                << " handle=" << gc._gltexture->handle(*state) << std::endl;

            // will activate the releaser
            gc._gltexture = nullptr;
        }
    }
    else
    {
        // rely on the Releaser to get around to it
        for(unsigned i=0; i< _globjects.size(); ++i)
        {
            // will activate the releaser(s)
            _globjects[i]._gltexture = nullptr;
        }
    }

    if (osgTexture().valid())
        osgTexture()->releaseGLObjects(state);
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
    // going to be changing things on the fly
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

    auto& osgTex = tex->osgTexture();

    // load the image if necessary
    if (!tex->dataLoaded() && tex->uri().isSet())
    {
        // TODO support read options for caching
        osg::ref_ptr<osg::Image> image = tex->_uri->getImage(nullptr);
        if (image.valid())
        {
            osgTex->setImage(0, image);
        }
        else
        {
            OE_WARN << LC << "Failed to load \"" << tex->_uri->full() << "\"" << std::endl;
        }
    }

    if (tex->dataLoaded())
    {
        auto image = osgTex->getImage(0);

        // in case we want to cache it later:
        image->setWriteHint(osg::Image::STORE_INLINE);

        // compress and mipmap:
        if (!image->isCompressed())
        {
            if (image->getPixelFormat() == image->getInternalTextureFormat())
            {
                // normalize the internal texture format
                GLenum internalFormat =
                    tex->internalFormat().isSet() ? tex->internalFormat().get() :
                    image->getPixelFormat() == GL_RED ? GL_R32F :
                    image->getPixelFormat() == GL_RG ? GL_RG8 :
                    image->getPixelFormat() == GL_RGB ? GL_RGB8 :
                    image->getPixelFormat() == GL_RGBA ? GL_RGBA8 :
                    GL_RGBA8;

                image->setInternalTextureFormat(internalFormat);
            }

            if (tex->_compress)
            {
                ImageUtils::compressImageInPlace(image);
            }
        }
    }
    else
    {
        // is it a pre-existing gltexture? If not, fail.
        if (tex->_globjects.size() == 0)
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
    for(unsigned i=0; i< _globjects.size(); ++i)
    {
        if (_globjects[i]._inUse)
            _globjects[i]._toCompile.push(index);
    }

    if (index < _textures.size())
        _textures[index] = tex;
    else
        _textures.push_back(tex);

    return index;
}

void
TextureArena::apply(osg::State& state) const
{
    if (_textures.empty())
        return;

    ScopedMutexLock lock(_m);

    OE_PROFILING_ZONE;

    GLObjects& gc = GLObjects::get(_globjects, state);

    // first time seeing this GC? Prime it by adding all textures!
    if (gc._inUse == false)
    {
        gc._inUse = true;

        while (!gc._toCompile.empty())
            gc._toCompile.pop();

        for (unsigned i = 0; i < _textures.size(); ++i) {
            if (_textures[i])
                gc._toCompile.push(i);
        }
    }

    const osg::StateAttribute* lastTex = nullptr;
    if (!gc._toCompile.empty())
    {
        // need to save any bound texture so we can reinstate it:
        lastTex = state.getLastAppliedTextureAttribute(
            state.getActiveTextureUnit(), osg::StateAttribute::TEXTURE);
    }

    // allocate textures and resident handles
    while(!gc._toCompile.empty())
    {
        int index = gc._toCompile.front();
        gc._toCompile.pop();
        auto tex = _textures[index];
        if (tex)
        {
            tex->compileGLObjects(state);
        }

        // mark the GC to re-upload its LUT
        gc._dirty = true;
    }

    if (_autoRelease == true)
    {
        for (Texture::Ptr& tex : _textures)
        {
            if (tex && tex.use_count() == 1)
            {
                // TODO: remove it from the arena altogether
                tex->releaseGLObjects(&state);
                tex = nullptr;
            }
        }
    }

    if (gc._handleBuffer == nullptr || !gc._handleBuffer->valid())
    {
        if (_useUBO)
            gc._handleBuffer = GLBuffer::create(GL_UNIFORM_BUFFER, state);
        else
            gc._handleBuffer = GLBuffer::create(GL_SHADER_STORAGE_BUFFER, state);

        gc._handleBuffer->bind();
        gc._handleBuffer->debugLabel("TextureArena", getName());
        gc._handleBuffer->unbind();

        gc._dirty = true;
    }

    // refresh the handles buffer if necessary:
    if (_textures.size() > gc._handles.size())
    {
        size_t aligned_size = gc._handleBuffer->align(_textures.size() * sizeof(gc._handles[0]))
            / sizeof(gc._handles[0]);
        gc._handles.assign(aligned_size, 0);
        gc._dirty = true;
    }

    unsigned ptr = 0;
    for (auto& tex : _textures)
    {
        GLTexture* gltex = nullptr;
        if (tex)
            gltex = Texture::GLObjects::get(tex->_globjects, state)._gltexture.get();

        GLuint64 handle = gltex ? gltex->handle(state) : 0ULL;
        unsigned index = _useUBO ? ptr * 2 : ptr; // hack for std140 vec4 alignment
        if (gc._handles[index] != handle)
        {
            gc._handles[index] = handle;
            gc._dirty = true;
        }

        // check whether a dynamic texture needs recompile:
        if (tex && tex->needsCompile(state))
        {
            gc._toCompile.push(ptr);
        }

        ++ptr;
    }

    // upload to GPU if it changed:
    if (gc._dirty)
    {
        gc._handleBuffer->uploadData(gc._handles);
        gc._dirty = false;

        // reinstate the old bound texture
        if (lastTex)
            state.applyTextureAttribute(state.getActiveTextureUnit(), lastTex);
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

    if (_globjects.size() < maxSize)
    {
        _globjects.resize(maxSize);
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
        auto& gc = GLObjects::get(_globjects, *state);

        gc._handleBuffer = nullptr;
        while (!gc._toCompile.empty())
            gc._toCompile.pop();

        for (unsigned i = 0; i < _textures.size(); ++i)
        {
            if (_textures[i])
            {
                _textures[i]->releaseGLObjects(state);
                gc._toCompile.push(i);
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

        for (unsigned i = 0; i < _globjects.size(); ++i)
        {
            GLObjects& gc = _globjects[i];
            if(gc._inUse)
            {
                gc._handleBuffer = nullptr;
                while (!gc._toCompile.empty())
                    gc._toCompile.pop();

                for (unsigned i = 0; i < _textures.size(); ++i)
                {
                    if (_textures[i])
                        gc._toCompile.push(i);
                }
            }
        }
    }
}
