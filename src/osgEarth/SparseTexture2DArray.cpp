/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
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
#include <osgEarth/SparseTexture2DArray>

// this class is only supported in newer OSG versions.
#if OSG_VERSION_GREATER_OR_EQUAL( 2, 9, 8 )

using namespace osgEarth;

int
SparseTexture2DArray::firstValidImageIndex() const 
{
    for( int i=0; i<(int)_images.size(); ++i )
        if ( _images[i].valid() )
            return i;
    return -1;
}

osg::Image* 
SparseTexture2DArray::firstValidImage() const
{
    int i = firstValidImageIndex();
    return i >= 0 ? _images[i].get() : 0L;
}

void 
SparseTexture2DArray::computeInternalFormat() const
{
    osg::Image* image = firstValidImage();
    if ( image )
        computeInternalFormatWithImage( *image );
    else
        computeInternalFormatType();
}

void 
SparseTexture2DArray::apply( osg::State& state ) const
{
    // get the contextID (user defined ID of 0 upwards) for the 
    // current OpenGL context.
    const unsigned int contextID = state.getContextID();

    Texture::TextureObjectManager* tom = Texture::getTextureObjectManager(contextID).get();
    //ElapsedTime elapsedTime(&(tom->getApplyTime()));
    tom->getNumberApplied()++;

    const Extensions* extensions = getExtensions(contextID,true);

    // if not supported, then return
    if (!extensions->isTexture2DArraySupported() || !extensions->isTexture3DSupported())
    {
        OSG_WARN<<"Warning: Texture2DArray::apply(..) failed, 2D texture arrays are not support by OpenGL driver."<<std::endl;
        return;
    }

    // get the texture object for the current contextID.
    TextureObject* textureObject = getTextureObject(contextID);

    if (textureObject && _textureDepth>0)
    {
        const osg::Image* image = firstValidImage();
        if (image && getModifiedCount(0, contextID) != image->getModifiedCount())
        {
            // compute the internal texture format, this set the _internalFormat to an appropriate value.
            computeInternalFormat();

            GLsizei new_width, new_height, new_numMipmapLevels;

            // compute the dimensions of the texture.
            computeRequiredTextureDimensions(state, *image, new_width, new_height, new_numMipmapLevels);

            if (!textureObject->match(GL_TEXTURE_2D_ARRAY_EXT, new_numMipmapLevels, _internalFormat, new_width, new_height, 1, _borderWidth))
            {
                Texture::releaseTextureObject(contextID, _textureObjectBuffer[contextID].get());
                _textureObjectBuffer[contextID] = 0;
                textureObject = 0;
            }
        }
    }

    // if we already have an texture object, then 
    if (textureObject)
    {
        // bind texture object
        textureObject->bind();

        // if texture parameters changed, then reset them
        if (getTextureParameterDirty(state.getContextID())) applyTexParameters(GL_TEXTURE_2D_ARRAY_EXT,state);

        // if subload is specified, then use it to subload the images to GPU memory
        //if (_subloadCallback.valid())
        //{
        //    _subloadCallback->subload(*this,state);
        //}
        //else
        {
            // for each image of the texture array do
            for (GLsizei n=0; n < _textureDepth; n++)
            {
                osg::Image* image = _images[n].get();

                // if image content is modified, then upload it to the GPU memory
                // GW: this means we have to "dirty" an image before setting it!
                if (image && getModifiedCount(n,contextID) != image->getModifiedCount())
                {
                    applyTexImage2DArray_subload(state, image, _textureWidth, _textureHeight, n, _internalFormat, _numMipmapLevels);
                    getModifiedCount(n,contextID) = image->getModifiedCount();
                }
            }
        }
    }

    // nothing before, but we have valid images, so do manual upload and create texture object manually
    else if ( firstValidImage() != 0L ) // if (imagesValid())
    {
        // compute the internal texture format, this set the _internalFormat to an appropriate value.
        computeInternalFormat();

        // compute the dimensions of the texture.
        osg::Image* firstImage = firstValidImage();
        computeRequiredTextureDimensions(state, *firstImage, _textureWidth, _textureHeight, _numMipmapLevels);

        // create texture object
        textureObject = generateTextureObject(
            this, contextID,GL_TEXTURE_2D_ARRAY_EXT,_numMipmapLevels,_internalFormat,_textureWidth,_textureHeight,_textureDepth,0);

        // bind texture
        textureObject->bind();
        applyTexParameters(GL_TEXTURE_2D_ARRAY_EXT, state);

        _textureObjectBuffer[contextID] = textureObject;

        // First we need to allocate the texture memory
        int sourceFormat = _sourceFormat ? _sourceFormat : _internalFormat;

        if( isCompressedInternalFormat( sourceFormat ) && 
            sourceFormat == _internalFormat &&
            extensions->isCompressedTexImage3DSupported() )
        {
            extensions->glCompressedTexImage3D( GL_TEXTURE_2D_ARRAY_EXT, 0, _internalFormat,
                _textureWidth, _textureHeight, _textureDepth, _borderWidth,
                firstImage->getImageSizeInBytes() * _textureDepth,
                0);
        }
        else
        {   
            // Override compressed source format with safe GL_RGBA value which not generate error
            // We can safely do this as source format is not important when source data is NULL
            if( isCompressedInternalFormat( sourceFormat ) )
                sourceFormat = GL_RGBA;

            extensions->glTexImage3D( GL_TEXTURE_2D_ARRAY_EXT, 0, _internalFormat,
                _textureWidth, _textureHeight, _textureDepth, _borderWidth,
                sourceFormat, _sourceType ? _sourceType : GL_UNSIGNED_BYTE,
                0); 
        }

        // For certain we have to manually allocate memory for mipmaps if images are compressed
        // if not allocated OpenGL will produce errors on mipmap upload.
        // I have not tested if this is neccessary for plain texture formats but 
        // common sense suggests its required as well.
        if( _min_filter != LINEAR && _min_filter != NEAREST && firstImage->isMipmap() )
            allocateMipmap( state );

        // now for each layer we upload it into the memory
        for (GLsizei n=0; n<_textureDepth; n++)
        {
            // if image is valid then upload it to the texture memory
            osg::Image* image = _images[n].get();
            if (image)
            {
                // now load the image data into the memory, this will also check if image do have valid properties
                applyTexImage2DArray_subload(state, image, _textureWidth, _textureHeight, n, _internalFormat, _numMipmapLevels);
                getModifiedCount(n,contextID) = image->getModifiedCount();
            }
        }

        const Texture::Extensions* texExtensions = Texture::getExtensions(contextID,true);
        // source images have no mipmamps but we could generate them...  
        if( _min_filter != LINEAR && _min_filter != NEAREST && !firstImage->isMipmap() &&  
            _useHardwareMipMapGeneration && texExtensions->isGenerateMipMapSupported() )
        {
            _numMipmapLevels = osg::Image::computeNumberOfMipmapLevels( _textureWidth, _textureHeight );
            generateMipmap( state );
        }

        textureObject->setAllocated(_numMipmapLevels,_internalFormat,_textureWidth,_textureHeight,_textureDepth,0);

        // unref image data?
        if (isSafeToUnrefImageData(state))
        {
            SparseTexture2DArray* non_const_this = const_cast<SparseTexture2DArray*>(this);
            for (int n=0; n<_textureDepth; n++)
            {                
                if (_images[n].valid() && _images[n]->getDataVariance()==STATIC)
                {
                    non_const_this->_images[n] = NULL;
                }
            }
        }

    }

    // No images present, but dimensions are set. So create empty texture
    else if ( (_textureWidth > 0) && (_textureHeight > 0) && (_textureDepth > 0) && (_internalFormat!=0) )
    {
        // generate texture 
        _textureObjectBuffer[contextID] = textureObject = generateTextureObject(
            this, contextID, GL_TEXTURE_2D_ARRAY_EXT,_numMipmapLevels,_internalFormat,_textureWidth,_textureHeight,_textureDepth,0);

        textureObject->bind();
        applyTexParameters(GL_TEXTURE_2D_ARRAY_EXT,state);

        extensions->glTexImage3D( GL_TEXTURE_2D_ARRAY_EXT, 0, _internalFormat,
            _textureWidth, _textureHeight, _textureDepth,
            _borderWidth,
            _sourceFormat ? _sourceFormat : _internalFormat,
            _sourceType ? _sourceType : GL_UNSIGNED_BYTE,
            0); 

    }

    // nothing before, so just unbind the texture target
    else
    {
        glBindTexture( GL_TEXTURE_2D_ARRAY_EXT, 0 );
    }

    // if texture object is now valid and we have to allocate mipmap levels, then
    if (textureObject != 0 && _texMipmapGenerationDirtyList[contextID])
    {
        generateMipmap(state);
    }
}

// replaces the same func in the superclass
void
SparseTexture2DArray::applyTexImage2DArray_subload(osg::State& state, osg::Image* image,
                                                   GLsizei inwidth, GLsizei inheight, GLsizei indepth, 
                                                   GLint inInternalFormat, GLsizei& numMipmapLevels) const
{
    //// if we don't have a valid image we can't create a texture!
    //if (!imagesValid())
    //    return;

    // get the contextID (user defined ID of 0 upwards) for the 
    // current OpenGL context.
    const unsigned int contextID = state.getContextID();
    const Extensions* extensions = getExtensions(contextID,true);    
    const Texture::Extensions* texExtensions = Texture::getExtensions(contextID,true);
    GLenum target = GL_TEXTURE_2D_ARRAY_EXT;

    // compute the internal texture format, this set the _internalFormat to an appropriate value.
    computeInternalFormat();

    // select the internalFormat required for the texture.
    // bool compressed = isCompressedInternalFormat(_internalFormat);
    bool compressed_image = isCompressedInternalFormat((GLenum)image->getPixelFormat());

    // if the required layer is exceeds the maximum allowed layer sizes
    if (indepth > extensions->maxLayerCount())
    {
        // we give a warning and do nothing
        OSG_WARN<<"Warning: Texture2DArray::applyTexImage2DArray_subload(..) the given layer number exceeds the maximum number of supported layers."<<std::endl;
        return;        
    }

    //Rescale if resize hint is set or NPOT not supported or dimensions exceed max size
    if( _resizeNonPowerOfTwoHint || !texExtensions->isNonPowerOfTwoTextureSupported(_min_filter)
        || inwidth > extensions->max2DSize()
        || inheight > extensions->max2DSize())
        image->ensureValidSizeForTexturing(extensions->max2DSize());

    // image size or format has changed, this is not allowed, hence return
    if (image->s()!=inwidth || 
        image->t()!=inheight || 
        image->getInternalTextureFormat()!=inInternalFormat ) 
    {
        OSG_WARN<<"Warning: Texture2DArray::applyTexImage2DArray_subload(..) given image do have wrong dimension or internal format."<<std::endl;
        return;        
    }    

    glPixelStorei(GL_UNPACK_ALIGNMENT,image->getPacking());

    bool useHardwareMipmapGeneration = 
        !image->isMipmap() && _useHardwareMipMapGeneration && texExtensions->isGenerateMipMapSupported();

    // if no special mipmapping is required, then
    if( _min_filter == LINEAR || _min_filter == NEAREST || useHardwareMipmapGeneration )
    {
        if( _min_filter == LINEAR || _min_filter == NEAREST )
            numMipmapLevels = 1;
        else //Hardware Mipmap Generation
            numMipmapLevels = image->getNumMipmapLevels();

        // upload non-compressed image
        if ( !compressed_image )
        {
            extensions->glTexSubImage3D( target, 0,
                0, 0, indepth,
                inwidth, inheight, 1,
                (GLenum)image->getPixelFormat(),
                (GLenum)image->getDataType(),
                image->data() );
        }

        // if we support compression and image is compressed, then
        else if (extensions->isCompressedTexImage3DSupported())
        {
            // OSG_WARN<<"glCompressedTexImage3D "<<inwidth<<", "<<inheight<<", "<<indepth<<std::endl;

            GLint blockSize, size;
            getCompressedSize(_internalFormat, inwidth, inheight, 1, blockSize,size);

            extensions->glCompressedTexSubImage3D(target, 0,
                0, 0, indepth,  
                inwidth, inheight, 1, 
                (GLenum)image->getPixelFormat(),
                size, 
                image->data());
        }

        // we want to use mipmapping, so enable it
    }else
    {
        // image does not provide mipmaps, so we have to create them
        if( !image->isMipmap() )
        {
            numMipmapLevels = 1;
            OSG_WARN<<"Warning: Texture2DArray::applyTexImage2DArray_subload(..) mipmap layer not passed, and auto mipmap generation turned off or not available. Check texture's min/mag filters & hardware mipmap generation."<<std::endl;

            // the image object does provide mipmaps, so upload the in the certain levels of a layer
        }else
        {
            numMipmapLevels = image->getNumMipmapLevels();

            int width  = image->s();
            int height = image->t();

            if( !compressed_image )
            {

                for( GLsizei k = 0 ; k < numMipmapLevels  && (width || height ) ;k++)
                {
                    if (width == 0)
                        width = 1;
                    if (height == 0)
                        height = 1;

                    extensions->glTexSubImage3D( target, k, 0, 0, indepth,
                        width, height, 1, 
                        (GLenum)image->getPixelFormat(),
                        (GLenum)image->getDataType(),
                        image->getMipmapData(k));

                    width >>= 1;
                    height >>= 1;
                }
            }
            else if (extensions->isCompressedTexImage3DSupported())
            {
                GLint blockSize,size;
                for( GLsizei k = 0 ; k < numMipmapLevels  && (width || height) ;k++)
                {
                    if (width == 0)
                        width = 1;
                    if (height == 0)
                        height = 1;

                    getCompressedSize(image->getInternalTextureFormat(), width, height, 1, blockSize,size);

                    //                    state.checkGLErrors("before extensions->glCompressedTexSubImage3D(");

                    extensions->glCompressedTexSubImage3D(target, k, 0, 0, indepth,
                        width, height, 1,
                        (GLenum)image->getPixelFormat(),
                        size,
                        image->getMipmapData(k));

                    //                    state.checkGLErrors("after extensions->glCompressedTexSubImage3D(");

                    width >>= 1;
                    height >>= 1;
                }
            }
        }
    }
}

#endif // OSG_VERSION_GREATER_OR_EQUAL( 2, 9, 8 )
