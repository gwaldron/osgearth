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
#include <osg/State>

#ifndef GL_TEXTURE_SPARSE_ARB
    #define GL_TEXTURE_SPARSE_ARB 0x91A6
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

Texture::GCState&
Texture::get(const osg::State& state) const
{
    return _gc[state.getContextID()];
}

bool
Texture::isCompiled(const osg::State& state) const
{
    return _gc[state.getContextID()]._gltexture.valid();
}

void
Texture::compileGLObjects(osg::State& state) const
{
    osg::GLExtensions* ext = state.get<osg::GLExtensions>();
    Texture::GCState& gc = get(state);

    // If you change this you must change the typecast in the fragment shader too
    GLenum target = GL_TEXTURE_2D_ARRAY;
    //GLenum target = GL_TEXTURE_2D;

    gc._gltexture = new GLTexture(target, state, _uri->base());

    // TODO: make sure we can actually generate mipmaps for a texture array ...
    bool useGPUmipmaps = false;

    unsigned numMipLevels = _image->getNumMipmapLevels();

    useGPUmipmaps = (numMipLevels <= 1);

    //TODO: compute the best internal format
    GLenum pixelFormat = _image->getPixelFormat();

    GLenum internalFormat = 
        pixelFormat == GL_RGB ? GL_COMPRESSED_RGBA_S3TC_DXT1_EXT :
        pixelFormat == GL_RGBA ? GL_COMPRESSED_RGBA_S3TC_DXT5_EXT :
        GL_RGBA8;

    // Blit our image to the GPU
    gc._gltexture->bind();

    if (target == GL_TEXTURE_2D_ARRAY)
    {
        ext->glTexStorage3D(
            target,
            numMipLevels,
            internalFormat,
            _image->s(),
            _image->t(),
            _image->r() );
    }
    else
    {
        ext->glTexStorage2D(
            target,
            numMipLevels,
            internalFormat,
            _image->s(),
            _image->t() );
    }

    glTexParameteri(target, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(target, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glTexParameteri(target, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(target, GL_TEXTURE_WRAP_T, GL_REPEAT);

    // Foce creation of the bindless handle - once you do this, you can
    // no longer change the texture parameters.
    gc._gltexture->handle();

    // TODO: At this point, if/when we go with SPARSE textures,
    // don't actually copy the image down until activation.

    // Iterate over mipmap levels in this layer:
    for (unsigned mipLevel = 0; mipLevel < numMipLevels; ++mipLevel)
    {
        int sizeOfMipImage = _image->getImageSizeInBytes() >> (2*mipLevel);

        // Iterate over image slices:
        for (int r = 0; r < _image->r(); ++r)
        {
            if (target == GL_TEXTURE_2D_ARRAY)
            {
                unsigned char* dataptr =
                    _image->getMipmapData(mipLevel) +
                    sizeOfMipImage * r;

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
                    _image->getMipmapData(mipLevel) );
            }
        }
    }

    if (useGPUmipmaps)
    {
        ext->glGenerateMipmap(target);
    }
}

void
Texture::makeResident(const osg::State& state, bool toggle) const
{
    GCState& gc = get(state);

    if (gc._gltexture.valid())
    {
        gc._gltexture->makeResident(toggle);
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
        if (_gc[state->getContextID()]._gltexture.valid())
        {
            GCState& gc = get(*state);

            // will activate the releaser
            gc._gltexture = NULL;
        }
    }
    else
    {
        // rely on the Releaser to get around to it
        for(int i=0; i<_gc.size(); ++i)
        {
            // will activate the releaser(s)
            _gc[i]._gltexture = NULL;
        }
    }
}


//...................................................................

#undef LC
#define LC "[TextureArena] "


TextureArena::TextureArena() :
    _compiled(false)
{
    // Keep this synchronous w.r.t. the render thread since we are
    // giong to be changing things on the fly
    setDataVariance(DYNAMIC);
}

TextureArena::~TextureArena()
{
    releaseGLObjects(NULL);
}

void
TextureArena::add(Texture* tex)
{
    if (!tex) return;

    if (tex->_image.valid() == false)
    {
        //TODO support read options for caching
        tex->_image = tex->_uri->getImage(NULL);
    }

    if (tex->_image.valid())
    {
        // Try to generate mipmaps on the CPU.
        if ( tex->_image->getNumMipmapLevels() <= 1 )
        {
            ImageUtils::generateMipmaps(tex->_image.get());
        }

        // add to all GCs.
        for(unsigned i=0; i<_gc.size(); ++i)
        {
            if (_gc[i]._inUse)
                _gc[i]._toAdd.push_back(tex);
        }

        _textures.push_back(tex);
    }

    //TODO: consider issues like multiple GCs and "unref after apply"
}

void
TextureArena::activate(Texture* tex)
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
TextureArena::deactivate(Texture* tex)
{
    if (!tex) return;

    // add to all GCs.
    for(unsigned i=0; i<_gc.size(); ++i)
    {
        _gc[i]._toDeactivate.push_back(tex);
    }

    //TODO: consider issues like multiple GCs and "unref after apply"
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

    // allocate textures and resident handles
    for(auto& tex : gc._toAdd)
    {
        if (!tex->isCompiled(state))
        {
            tex->compileGLObjects(state);

            //TODO: take this out -- temp for TESTING.
            tex->makeResident(state, true);
        }

        //TODO: Consider making the texture SPARSE as well so we can
        // control the actual texture residency along with the handle residency.
        // i.e. call glTexPageCommitment(...,GL_TRUE) here
    }
    gc._toAdd.clear();

    // remove pending objects by swapping them out of memory
    for(auto& tex : gc._toDeactivate)
    {
        tex->makeResident(state, false);
    }
    gc._toDeactivate.clear();

    // add pending textures by swapping them in to memory
    for(auto& tex : gc._toActivate)
    {
        tex->makeResident(state, true);
    }
    gc._toActivate.clear();

    // update the LUT if it's dirty
    gc._handleLUT.allocate(_textures, state);

    osg::GLExtensions* ext = state.get<osg::GLExtensions>();

    if (gc._handleLUT._dirty)
    {
        gc._handleLUT.update();
    }

    // bind to the layout index in the shader
    gc._handleLUT.bindLayout();
}

void
TextureArena::compileGLObjects(osg::State& state) const
{
    apply(state);
    _compiled = true;
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
}

void
TextureArena::HandleLUT::allocate(const TextureVector& textures, osg::State& state)
{
    _requiredSize = textures.size() * sizeof(GLuint64);

    if (_requiredSize > _allocatedSize)
    {
        static Threading::Mutex s_mutex;
        Threading::ScopedMutexLock lock(s_mutex);

        initGL();

        if (!_alignment.isSet())
        {
            glGetIntegerv(GL_SHADER_STORAGE_BUFFER_OFFSET_ALIGNMENT, &_alignment.mutable_value());
            OE_DEBUG << "SSBO Alignment = " << _alignment << std::endl;
        }

        // TODO: fetch the binding
        // for now, hard-code it
        //ext->glGetProgramResourceIndex(...
        if (_bindingIndex < 0)
        {
            _bindingIndex = 5; // TODO: query!
        }

        release();

        _numTextures = textures.size();
        _allocatedSize = _requiredSize;

        _buf = new GLuint64[textures.size()];

        // set all data:
        OE_DEBUG << LC << "Uploading " << _numTextures << " texture handles" << std::endl;

        osg::GLExtensions* ext = state.get<osg::GLExtensions>();

        unsigned i=0;
        for(const auto& tex : textures)
        {
            _buf[i++] = tex->_gc[state.getContextID()]._gltexture->handle();
        }

        _buffer = new GLBuffer(GL_SHADER_STORAGE_BUFFER, state);

        _buffer->bind();
        _glBufferStorage(GL_SHADER_STORAGE_BUFFER, _allocatedSize, _buf, GL_DYNAMIC_STORAGE_BIT);

        _dirty = true;
    }
}

void
TextureArena::HandleLUT::release() const
{
    SSBO::release();
    if (_buf)
    {
        delete[] _buf;
        _buf = NULL;
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
