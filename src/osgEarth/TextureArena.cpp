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

#ifndef GL_TEXTURE_SPARSE_ARB
    #define GL_TEXTURE_SPARSE_ARB 0x91A6
#endif

// osg 3.6:
#ifndef GL_TEXTURE_2D_ARRAY
#define GL_TEXTURE_2D_ARRAY 0x8C1A
#endif

#ifndef GL_VIRTUAL_PAGE_SIZE_INDEX_ARB
    #define GL_VIRTUAL_PAGE_SIZE_INDEX_ARB 0x91A7
    #define GL_NUM_VIRTUAL_PAGE_SIZES_ARB 0x91A8
    #define GL_VIRTUAL_PAGE_SIZE_X_ARB 0x9195
    #define GL_VIRTUAL_PAGE_SIZE_Y_ARB 0x9196 
    #define GL_VIRTUAL_PAGE_SIZE_Z_ARB 0x9197
#endif

using namespace osgEarth;

#undef LC
#define LC "[Texture] "

#define OE_DEVEL OE_DEBUG

//#define USE_ICO 1

Texture::GCState&
Texture::get(const osg::State& state) const
{
    return _gc[state.getContextID()];
}

bool
Texture::isCompiled(const osg::State& state) const
{
    return _gc[state.getContextID()]._gltexture != nullptr;
}

void
Texture::compileGLObjects(osg::State& state) const
{
    osg::GLExtensions* ext = state.get<osg::GLExtensions>();
    Texture::GCState& gc = get(state);

    // If you change this you must change the typecast in the fragment shader too
    //GLenum target = GL_TEXTURE_2D_ARRAY;
    GLenum target = GL_TEXTURE_2D;

    gc._gltexture = GLTexture::create(target, state, _uri->base());

    // debugging
    gc._gltexture->id() = _uri->base();

    // mipmaps already created and in the image:
    unsigned numMipLevelsInMemory = _image->getNumMipmapLevels();

    // how much space we need to allocate on the GPU:
    unsigned numMipLevelsToAllocate = numMipLevelsInMemory;

    if (numMipLevelsInMemory <= 1)
    {
        numMipLevelsToAllocate = osg::Image::computeNumberOfMipmapLevels(
            _image->s(), _image->t(), _image->t());
    }
    
    GLenum pixelFormat = _image->getPixelFormat();

    // trigger GPU compression if not already compressed
    GLenum internalFormat =
        _image->isCompressed() ? _image->getInternalTextureFormat() :
        pixelFormat == GL_RG ? GL_RG8 :
        pixelFormat == GL_RGB ? GL_RGB8 :
        GL_RGBA8;

    if (_compress)
    {
        if (pixelFormat == GL_RGB)
            internalFormat = GL_COMPRESSED_RGB_S3TC_DXT1_EXT;
        else if (pixelFormat == GL_RGBA)
            GL_COMPRESSED_RGBA_S3TC_DXT5_EXT;
    }

    // Blit our image to the GPU
    gc._gltexture->bind(state);

    if (target == GL_TEXTURE_2D_ARRAY)
    {
        GLFunctions::get(state).
            glTexStorage3D(
                target,
                numMipLevelsToAllocate,
                internalFormat,
                _image->s(),
                _image->t(),
                _image->r());
    }
    else
    {
        ext->glTexStorage2D(
            target,
            numMipLevelsToAllocate,
            internalFormat,
            _image->s(),
            _image->t() );
    }

    glTexParameteri(target, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(target, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glTexParameteri(target, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(target, GL_TEXTURE_WRAP_T, GL_REPEAT);

    glTexParameterf(target, GL_TEXTURE_MAX_ANISOTROPY_EXT, 4.0f);

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
                    ext->glCompressedTexSubImage3D(
                        target, // GL_TEXTURE_2D_ARRAY
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
                    ext->glTexSubImage3D(
                        target, // GL_TEXTURE_2D_ARRAY
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
                    ext->glCompressedTexSubImage2D(
                        target, // GL_TEXTURE_2D
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
                    glTexSubImage2D(
                        target, // GL_TEXTURE_2D
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


void
Texture::resizeGLObjectBuffers(unsigned maxSize)
{
    if (_gc.size() < maxSize)
        _gc.resize(maxSize);
}

void
Texture::releaseGLObjects(osg::State* state) const
{
    if (state)
    {
        if (_gc[state->getContextID()]._gltexture != nullptr)
        {
            GCState& gc = get(*state);

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
        for(unsigned i=0; i<_gc.size(); ++i)
        {
            // will activate the releaser(s)
            _gc[i]._gltexture = nullptr;
        }
    }
}


//...................................................................

#undef LC
#define LC "[TextureArena] "


TextureArena::TextureArena()
{
    // Keep this synchronous w.r.t. the render thread since we are
    // giong to be changing things on the fly
    setDataVariance(DYNAMIC);
}

TextureArena::~TextureArena()
{
    releaseGLObjects(nullptr);
}

bool
TextureArena::add(Texture::Ptr tex)
{
    OE_SOFT_ASSERT_AND_RETURN(tex != nullptr, false);

    if (tex->_image.valid() == false)
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
                    tex->_image->getPixelFormat() == GL_RED ? GL_R16F :
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

        _textures.push_back(tex);

        return true;
    }
    else
    {
        return false;
    }

    //TODO: consider issues like multiple GCs and "unref after apply"
}

void
TextureArena::activate(Texture::Ptr tex)
{
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

    GCState& gc = _gc[state.getContextID()];

    // first time seeing this GC? Prime it by adding all textures!
    if (gc._inUse == false)
    {
        gc._inUse = true;
        gc._toAdd.resize(_textures.size());
        std::copy(_textures.begin(), _textures.end(), gc._toAdd.begin());
    }

    osgUtil::IncrementalCompileOperation* ico = nullptr;

#ifdef USE_ICO
    if (!gc._toAdd.empty())
    {
        const osg::Camera* camera = state.getGraphicsContext()->getCameras().front();
        const osgViewer::View* osgView = dynamic_cast<const osgViewer::View*>(camera->getView());
        if (osgView)
            ico = const_cast<osgViewer::View*>(osgView)->getDatabasePager()->getIncrementalCompileOperation();
    }
#endif

    // allocate textures and resident handles

    TextureVector stillCompiling;

    for(auto& tex : gc._toAdd)
    {
        if (!tex->isCompiled(state))
        {
            if (ico)
            {
                Texture::GCState& tex_gc = tex->get(state);

                if (!tex_gc._compileSet.valid())
                {
                    tex_gc._compileSet = new osgUtil::IncrementalCompileOperation::CompileSet();
                    tex_gc._compileSet->_compileMap[state.getGraphicsContext()].add(new TextureCompileOp(tex));
                    ico->add(tex_gc._compileSet.get());
                }

                stillCompiling.push_back(tex);
            }
            else
            {
                tex->compileGLObjects(state);
                gc._toActivate.push_back(tex);
            }
        }
        else
        {
            gc._toActivate.push_back(tex);
        }
    }
    gc._toAdd.swap(stillCompiling);

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
        gc._handleLUT._dirty = true;
    }

#if 0
    // Works, but re-visit so we don't need to call every frame
    // and re-visit loading/unloading of textures from main memory..
    for (auto& tex : _textures)
    {
        if (tex.use_count() == 1)
        {
            tex->makeResident(state, false);
        }
        else
        {
            tex->makeResident(state, true);
        }
    }
#endif

    // update the LUT if it needs more space:
    gc._handleLUT.sync(_textures, state);

    // bind to the layout index in the shader
    gc._handleLUT.bindLayout();
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
    for(auto& tex : _textures)
    {
        tex->releaseGLObjects(state);
    }

    if (state)
    {
        _gc[state->getContextID()]._handleLUT.release();
    }
    else
    {
        for (unsigned i = 0; i < _gc.size(); ++i)
            _gc[i]._handleLUT.release();
    }
}

void
TextureArena::HandleLUT::sync(const TextureVector& textures, osg::State& state)
{
    _requiredSize = textures.size() * sizeof(GLuint64);

    if (_requiredSize > _allocatedSize)
    {
        static Threading::Mutex s_mutex;
        Threading::ScopedMutexLock lock(s_mutex);

        if (!_alignment.isSet())
        {
            glGetIntegerv(GL_SHADER_STORAGE_BUFFER_OFFSET_ALIGNMENT, &_alignment.mutable_value());
            OE_DEBUG << "SSBO Alignment = " << _alignment.get() << std::endl;
        }

        // TODO: fetch the binding
        // for now, hard-code it
        //ext->glGetProgramResourceIndex(...
        if (_bindingIndex < 0)
        {
            _bindingIndex = 5; // TODO: query...?
        }

        release();

        _numTextures = textures.size();
        _allocatedSize = _requiredSize;

        _buf = new GLuint64[textures.size()];

        // set all data:
        OE_DEBUG << LC << "Uploading " << _numTextures << " texture handles" << std::endl;

        // copy handles to buffer:
        refresh(textures, state);

        _buffer = GLBuffer::create(GL_SHADER_STORAGE_BUFFER, state, "OE TextureArena");

        _buffer->bind();

        GLFunctions::get(state).
            glBufferStorage(GL_SHADER_STORAGE_BUFFER, _allocatedSize, _buf, GL_DYNAMIC_STORAGE_BIT);
    }

    else if (_dirty)
    {
        refresh(textures, state);
        update();
    }
}

void
TextureArena::HandleLUT::refresh(const TextureVector& textures, osg::State& state)
{
    unsigned i=0;
    for(const auto& tex : textures)
    {
        GLTexture* glTex = tex->_gc[state.getContextID()]._gltexture.get();
        GLuint64 handle = 0ULL;
        if (glTex)
            handle = glTex->handle(state);

        if (_buf[i] != handle)
        {
            _buf[i] = handle;
            _dirty = true;
        }
        i++;
    }
}

void
TextureArena::HandleLUT::release() const
{
    SSBO::release();
    if (_buf)
    {
        delete[] _buf;
        _buf = nullptr;
    }
}

bool
TextureArena::HandleLUT::update()
{
    bool updateMe = _dirty;
    if (updateMe)
    {
        _buffer->bind();
        _buffer->ext()->glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, _allocatedSize, _buf);
        _dirty = false;
    }
    return updateMe;
}
