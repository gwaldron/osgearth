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


TextureArena::TextureArena() :
    _compiled(false)
{
    // Keep this synchronous w.r.t. the render thread since we are
    // giong to be changing things on the fly
    setDataVariance(DYNAMIC);
}

TextureArena::~TextureArena()
{
    //todo
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
        _toAdd.push_back(tex);
    }

    //TODO: consider issues like multiple GCs and "unref after apply"
}

void
TextureArena::activate(Texture* tex)
{
    if (!tex) return;
    _toActivate.push_back(tex);

    //TODO: consider issues like multiple GCs and "unref after apply"
}

void
TextureArena::deactivate(Texture* tex)
{
    if (!tex) return;
    _toDeactivate.push_back(tex);

    //TODO: consider issues like multiple GCs and "unref after apply"
}

void
TextureArena::allocate(Texture* tex, osg::State& state) const
{
    osg::GLExtensions* ext = state.get<osg::GLExtensions>();

    //TODO: move all this to GLTexture
    glGenTextures(1, &tex->_gltexture.mutable_value());

    //TODO: deal with potentially large numbers of these things
    //state.getGraphicsContext()->add(new GLTextureReleaser(tex->_object.get()));

    // TODO: make sure we can actually generate mipmaps for a texture array ...
    bool useGPUmipmaps = false;

    if ( !useGPUmipmaps && tex->_image->getNumMipmapLevels() <= 1 )
    {
        if (ImageUtils::generateMipmaps(tex->_image.get()) == false)
        {
            useGPUmipmaps = true;
        }
    }
    unsigned numMipLevels = tex->_image->getNumMipmapLevels();

    //TODO: compute the best internal format
    GLenum pixelFormat = tex->_image->getPixelFormat();

    GLenum internalFormat = 
        pixelFormat == GL_RGB ? GL_COMPRESSED_RGBA_S3TC_DXT1_EXT :
        pixelFormat == GL_RGBA ? GL_COMPRESSED_RGBA_S3TC_DXT5_EXT :
        GL_RGBA8;

    // If you change this you must change the typecast in the fragment shader too
    GLenum target = GL_TEXTURE_2D_ARRAY;
    //GLenum target = GL_TEXTURE_2D;

    // Blit our image to the GPU
    glBindTexture(
        target,
        tex->_gltexture.get());

    if (target == GL_TEXTURE_2D_ARRAY)
    {
        ext->glTexStorage3D(
            target,
            numMipLevels,
            internalFormat,
            tex->_image->s(),
            tex->_image->t(),
            tex->_image->r() );
    }
    else
    {
        ext->glTexStorage2D(
            target,
            numMipLevels,
            internalFormat,
            tex->_image->s(),
            tex->_image->t() );
    }

    glTexParameteri(target, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(target, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glTexParameteri(target, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(target, GL_TEXTURE_WRAP_T, GL_REPEAT);

    // Create the bindless handle
    tex->_glhandle = ext->glGetTextureHandle(tex->_gltexture.get());

    // At this point, if/when we go with SPARSE textures, don't actually
    // copy the image down until activation.

    // Iterate over mipmap levels in this layer:
    for (unsigned mipLevel = 0; mipLevel < numMipLevels; ++mipLevel)
    {
        int sizeOfMipImage = tex->_image->getImageSizeInBytes() >> (2*mipLevel);

        // Iterate over image slices:
        for (int r = 0; r < tex->_image->r(); ++r)
        {
            if (target == GL_TEXTURE_2D_ARRAY)
            {
                unsigned char* dataptr =
                    tex->_image->getMipmapData(mipLevel) +
                    sizeOfMipImage * r;

                ext->glTexSubImage3D(
                    target, // GL_TEXTURE_2D_ARRAY
                    mipLevel, // mip level
                    0, 0, // xoffset, yoffset
                    r, // zoffset (array layer)
                    tex->_image->s() >> mipLevel, // width at mipmap level i
                    tex->_image->t() >> mipLevel, // height at mipmap level i
                    1, // z size always = 1
                    tex->_image->getPixelFormat(),
                    tex->_image->getDataType(),
                    dataptr );
            }
            else
            {
                glTexSubImage2D(
                    target, // GL_TEXTURE_2D
                    mipLevel, // mip level
                    0, 0, // xoffset, yoffset
                    tex->_image->s() >> mipLevel, // width at mipmap level i
                    tex->_image->t() >> mipLevel, // height at mipmap level i
                    tex->_image->getPixelFormat(),
                    tex->_image->getDataType(),
                    tex->_image->getMipmapData(mipLevel) );
            }
        }
    }

    if (useGPUmipmaps)
    {
        ext->glGenerateMipmap(target);
    }
}

void
TextureArena::apply(osg::State& state) const
{
    //TODO: support multiple contexts
    osg::GLExtensions* ext = state.get<osg::GLExtensions>();

    // remove pending objects by swapping them out of memory
    for(auto i = _toDeactivate.begin(); i != _toDeactivate.end(); ++i)
    {
        Texture* tex = i->get();
        if (tex->_glhandle.isSet())
        {
            ext->glMakeTextureHandleNonResident(tex->_glhandle.get());
            tex->_resident = false;

            //TODO: remove a sparse tex from memory forcably by calling
            // i.e. call glTexPageCommitment(...,GL_FALSE) here...? Consider it
        }
    }
    _toDeactivate.clear();

    // add pending textures by swapping them in to memory
    for(auto i = _toActivate.begin(); i != _toActivate.end(); ++i)
    {
        Texture* tex = i->get();
        if (tex->_glhandle.isSet())
        {
            ext->glMakeTextureHandleResident(tex->_glhandle.get());
            tex->_resident = true;
        }

        //TODO: Consider making the texture SPARSE as well so we can
        // control the actual texture residency along with the handle residency.
        // i.e. call glTexPageCommitment(...,GL_TRUE) here
    }
    _toActivate.clear();

    // TODO: update the SSBO (?) with handle information...or don't, maybe
    // all the handles are always just there whether they are in use or not.
}

void
TextureArena::compileGLObjects(osg::State& state) const
{
    // allocate textures and resident handles
    for(auto i = _toAdd.begin(); i != _toAdd.end(); ++i)
    {
        Texture* tex = i->get();
        if (tex->_gltexture.isSet() == false)
        {
            allocate(tex, state);

            //TODO: take this out -- temp for TESTING.
            osg::GLExtensions* ext = state.get<osg::GLExtensions>();
            ext->glMakeTextureHandleResident(tex->_glhandle.get());
            tex->_resident = true;
        }

        //TODO: Consider making the texture SPARSE as well so we can
        // control the actual texture residency along with the handle residency.
        // i.e. call glTexPageCommitment(...,GL_TRUE) here
    }
    _toAdd.clear();
    _compiled = true;

    apply(state);
}

void
TextureArena::resizeGLObjectBuffers(unsigned maxSize)
{
    // TODO
}

void
TextureArena::releaseGLObjects(osg::State* state) const
{
    // TODO
}
