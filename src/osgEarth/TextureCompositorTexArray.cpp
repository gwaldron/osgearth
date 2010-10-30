/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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

#include <osgEarth/TextureCompositorTexArray>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/ShaderComposition>
#include <osg/Texture2DArray>
#include <vector>

using namespace osgEarth;

#define LC "[TextureCompositorTexArray] "

//------------------------------------------------------------------------

// specialized version of osg::Texture2DArray that allows NULL image layers.
namespace
{
    class SparseTexture2DArray : public osg::Texture2DArray
    {
    public:
        SparseTexture2DArray() : osg::Texture2DArray() { }

        SparseTexture2DArray( const SparseTexture2DArray& rhs, const osg::CopyOp& copyop =osg::CopyOp::DEEP_COPY_ALL )
            : osg::Texture2DArray( rhs, copyop ) { }

        META_StateAttribute( osgEarth, SparseTexture2DArray, TEXTURE );

    public:
        int firstValidImageIndex() const 
        {
            for( int i=0; i<_images.size(); ++i )
                if ( _images[i].valid() )
                    return i;
            return -1;
        }

        osg::Image* firstValidImage() const
        {
            int i = firstValidImageIndex();
            return i >= 0 ? _images[i].get() : 0L;
        }

        virtual void computeInternalFormat() const
        {
            osg::Image* image = firstValidImage();
            if ( image )
                computeInternalFormatWithImage( *image );
            else
                computeInternalFormatType();
        }

        virtual void apply( osg::State& state ) const
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
        void applyTexImage2DArray_subload( osg::State& state, osg::Image* image, GLsizei inwidth, GLsizei inheight, GLsizei indepth, GLint inInternalFormat, GLsizei& numMipmapLevels) const
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

    };
}

//------------------------------------------------------------------------

#if 0
static osg::Shader*
s_createTextureFragShaderFunction( int numImageLayers )
{
    std::stringstream buf;

    buf << "#version 130 \n"
        << "#extension GL_EXT_gpu_shader4 : enable \n"

        << "uniform sampler2DArray tex0; \n"
        << "uniform float[] region; \n"
        << "uniform float[] osgearth_imagelayer_opacity; \n"
        << "uniform bool[]  osgearth_imagelayer_enabled; \n"
        << "uniform float[] osgearth_imagelayer_range; \n"
        << "uniform float   osgearth_imagelayer_attenuation; \n"
        << "varying float osgearth_range; \n"

        << "vec4 osgearth_frag_texture(void) \n"
        << "{ \n"
        << "    vec3 color = vec3(1,1,1); \n"
        << "    float u, v, dmin, dmax, atten_min, atten_max; \n"
        << "    vec4 texel; \n";

    for(int i=0; i<numImageLayers; ++i)
    {
        int j = i*4;
        int k = i*2;
        buf << "    if (osgearth_imagelayer_enabled["<< i << "]) { \n"
            << "        u = region["<< j <<"] + (region["<< j+2 <<"] * gl_TexCoord[0].s); \n"
            << "        v = region["<< j+1 <<"] + (region["<< j+3 <<"] * gl_TexCoord[0].t); \n"
            << "        dmin = osgearth_range - osgearth_imagelayer_range["<< k << "]; \n"
            << "        dmax = osgearth_range - osgearth_imagelayer_range["<< k+1 <<"]; \n"
            << "        if (dmin >= 0 && dmax <= 0.0) { \n"
            << "            atten_max = -clamp( dmax, -osgearth_imagelayer_attenuation, 0 ) / osgearth_imagelayer_attenuation; \n"
            << "            atten_min =  clamp( dmin, 0, osgearth_imagelayer_attenuation ) / osgearth_imagelayer_attenuation; \n"
            << "            texel = texture2DArray( tex0, vec3(u,v,"<< i <<") ); \n"
            << "            color = mix(color, texel.rgb, texel.a * osgearth_imagelayer_opacity["<< i <<"] * atten_max * atten_min); \n"
            << "        } \n"
            << "    } \n";
    }

    buf << "    return vec4(color,1); \n"
        << "} \n";

    std::string str = buf.str();
    //OE_INFO << std::endl << str;
    return new osg::Shader( osg::Shader::FRAGMENT, str );
}
#endif

static osg::Shader*
s_createTextureFragShaderFunction( const TextureLayout& layout )
{
    std::stringstream buf;

    buf << "#version 130 \n"
        << "#extension GL_EXT_gpu_shader4 : enable \n"

        << "uniform sampler2DArray tex0; \n"
        << "uniform float[] region; \n"
        << "uniform float[] osgearth_imagelayer_opacity; \n"
        << "uniform bool[]  osgearth_imagelayer_enabled; \n"
        << "uniform float[] osgearth_imagelayer_range; \n"
        << "uniform float   osgearth_imagelayer_attenuation; \n"
        << "varying float osgearth_range; \n"

        << "vec4 osgearth_frag_texture(void) \n"
        << "{ \n"
        << "    vec3 color = vec3(1,1,1); \n"
        << "    float u, v, dmin, dmax, atten_min, atten_max; \n"
        << "    vec4 texel; \n";

    const RenderOrderVector& order = layout.getRenderOrder();

    for( int i = 0; i < order.size(); ++i )
    {
        int slot = order[i];
        int q = 2 * i;
        int r = 4 * slot;

        buf << "    if (osgearth_imagelayer_enabled["<< i << "]) { \n"
            << "        u = region["<< r <<"] + (region["<< r+2 <<"] * gl_TexCoord[0].s); \n"
            << "        v = region["<< r+1 <<"] + (region["<< r+3 <<"] * gl_TexCoord[0].t); \n"
            << "        dmin = osgearth_range - osgearth_imagelayer_range["<< q << "]; \n"
            << "        dmax = osgearth_range - osgearth_imagelayer_range["<< q+1 <<"]; \n"
            << "        if (dmin >= 0 && dmax <= 0.0) { \n"
            << "            atten_max = -clamp( dmax, -osgearth_imagelayer_attenuation, 0 ) / osgearth_imagelayer_attenuation; \n"
            << "            atten_min =  clamp( dmin, 0, osgearth_imagelayer_attenuation ) / osgearth_imagelayer_attenuation; \n"
            << "            texel = texture2DArray( tex0, vec3(u,v,"<< slot <<") ); \n"
            << "            color = mix(color, texel.rgb, texel.a * osgearth_imagelayer_opacity["<< i <<"] * atten_max * atten_min); \n"
            << "        } \n"
            << "    } \n"
            ;
    }

    buf << "    return vec4(color,1); \n"
        << "} \n";

    std::string str = buf.str();
    //OE_INFO << std::endl << str;
    return new osg::Shader( osg::Shader::FRAGMENT, str );
}

//------------------------------------------------------------------------

namespace
{
    static osg::Texture2DArray*
    s_getTexture( osg::StateSet* stateSet, const TextureLayout& layout )
    {
        osg::Texture2DArray* tex = static_cast<osg::Texture2DArray*>(
            stateSet->getTextureAttribute( 0, osg::StateAttribute::TEXTURE ) );

        // if the texture array doesn't exist, create it anew.
        if ( !tex )
        {
            tex = new SparseTexture2DArray();
            tex->setInternalFormat( GL_RGBA );
            tex->setTextureWidth( 256 );
            tex->setTextureHeight( 256 );
            
            // configure the mipmapping 
            tex->setMaxAnisotropy(16.0f);
            tex->setResizeNonPowerOfTwoHint(false);
            tex->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
            tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );

            // configure the wrapping
            tex->setWrap(osg::Texture::WRAP_S,osg::Texture::CLAMP_TO_EDGE);
            tex->setWrap(osg::Texture::WRAP_T,osg::Texture::CLAMP_TO_EDGE);

            stateSet->setTextureAttribute( 0, tex, osg::StateAttribute::ON );
            stateSet->addUniform( new osg::Uniform("tex0", 0) );
        }

        // grow the texture array if necessary.
        int requiredDepth = layout.getMaxUsedSlot() + 1;
        if ( tex->getTextureDepth() < requiredDepth )
            tex->setTextureDepth( requiredDepth );

        const TextureSlotVector& slots = layout.getTextureSlots();

        // null out any empty slots (to save memory, i guess)
        for( int i=0; i<tex->getTextureDepth(); ++i )
        {
            if ( i < slots.size() && slots[i] < 0 )
                tex->setImage( i, 0L );
        }

        return tex;
    }

    static osg::Uniform*
    s_getRegionUniform( osg::StateSet* stateSet, const TextureLayout& layout )
    {
        osg::Uniform* region = stateSet->getUniform( "region" );

        // if the region-uniform doesn't exist, create it now
        if ( !region )
        {
            region = new osg::Uniform( osg::Uniform::FLOAT, "region", layout.getTextureSlots().size() * 4 );
            stateSet->addUniform( region );
        }

        // if the region exists but is too small, re-allocate it (cannot grow it) and copy over the old values
        else if ( region->getNumElements() < layout.getTextureSlots().size() * 4 )
        {            
            osg::Uniform* newRegion = new osg::Uniform( osg::Uniform::FLOAT, "region", layout.getTextureSlots().size() * 4 );
            for( int i=0; i<region->getNumElements(); ++i )
            {
                float value;
                region->getElement( i, value );
                newRegion->setElement( i, value );
            }

            stateSet->removeUniform( region );
            stateSet->addUniform( newRegion );
            region = newRegion;
        }

        return region;
    }
};

//------------------------------------------------------------------------

GeoImage
TextureCompositorTexArray::prepareImage( const GeoImage& layerImage, const GeoExtent& tileExtent ) const
{
    osg::ref_ptr<osg::Image> image = layerImage.getImage();

    // Because all tex2darray layers must be identical in format, let's use RGBA.
    if ( image->getPixelFormat() != GL_RGBA )
        image = ImageUtils::convertToRGBA( image.get() );

    // TODO: revisit. For now let's just settle on 256 (again, all layers must be the same size)
    if ( image->s() != 256 || image->t() != 256 )
        image = ImageUtils::resizeImage( image.get(), 256, 256 );

    //Make sure that the internal texture format is always set to GL_RGBA
    image->setInternalTextureFormat( GL_RGBA );
    
    // Failure to do this with a Texture2DArray will result in texture corruption if we are 
    // updating layers (like in sequential mode).
    image->setDataVariance( osg::Object::DYNAMIC );

    return GeoImage( image.get(), layerImage.getExtent() );
}

void
TextureCompositorTexArray::applyLayerUpdate(osg::StateSet* stateSet,
                                            UID layerUID,
                                            const GeoImage& preparedImage,
                                            const GeoExtent& tileExtent,
                                            const TextureLayout& layout ) const
{
    int slot = layout.getSlot( layerUID );
    if ( slot < 0 )
        return; // means the layer no longer exists

    // access the texture array, creating or growing it if necessary:
    osg::Texture2DArray* texture = s_getTexture( stateSet, layout );

    // assign the new image at the proper position in the texture array. We have to 
    // dirty() the image because otherwise the texture2d array implementation will not
    // recognize it as new data.
    osg::Image* image = preparedImage.getImage();
    image->dirty();
    texture->setImage( slot, image );
    
    // update the region uniform to reflect the geo extent of the image:
    const GeoExtent& imageExtent = preparedImage.getExtent();
    float xoffset = (tileExtent.xMin() - imageExtent.xMin()) / imageExtent.width();
    float yoffset = (tileExtent.yMin() - imageExtent.yMin()) / imageExtent.height();
    float xscale  = tileExtent.width() / imageExtent.width();
    float yscale  = tileExtent.height() / imageExtent.height();

    // access the region uniform, creating or growing it if necessary:
    osg::Uniform* region = s_getRegionUniform( stateSet, layout );
    if ( region )
    {
        int layerOffset = slot * 4;
        region->setElement( layerOffset,     xoffset );
        region->setElement( layerOffset + 1, yoffset );
        region->setElement( layerOffset + 2, xscale );
        region->setElement( layerOffset + 3, yscale );
        region->dirty();
    }
}

#if 0
osg::StateSet*
TextureCompositorTexArray::createStateSet( const GeoImageVector& layerImages, const GeoExtent& tileExtent ) const
{
    osg::StateSet* stateSet = new osg::StateSet();
    if ( layerImages.size() < 1 )
        return stateSet;

    osg::Texture2DArray* texture = new osg::Texture2DArray();

    int texWidth = 0, texHeight = 0;

    // find each image layer and create a region entry for it
    texture->setTextureDepth( layerImages.size() );
    texture->setInternalFormat( GL_RGBA );

    // this uniform will properly position the image within the tile (kind of like
    // texture coordinate generation parameters).
    osg::Uniform* texInfoArray = new osg::Uniform( osg::Uniform::FLOAT, "region", layerImages.size() * 4 );

    int layerNum = 0, u = 0;
    for( GeoImageVector::const_iterator i = layerImages.begin(); i != layerImages.end(); ++i, ++layerNum )
    {
        const GeoImage& geoImage = *i;
        GeoImage preparedImage = prepareLayerUpdate( geoImage, tileExtent );
        osg::Image* image = preparedImage.getImage();

        // add the layer image to the composite.
        texture->setImage( layerNum, image );

        // track the maximum texture size
        // TODO: currently pointless since we are resizing all layers to 256 anyway...
        if ( image->s() > texWidth )
            texWidth = image->s();

        if ( image->t() > texHeight )
            texHeight = image->t();
            
        // record the proper texture offset/scale for this layer. this accounts for subregions that
        // are used when referencing lower LODs.
        const GeoExtent& layerExtent = preparedImage.getExtent();

        float xoffset = (tileExtent.xMin() - layerExtent.xMin()) / layerExtent.width();
        float yoffset = (tileExtent.yMin() - layerExtent.yMin()) / layerExtent.height();
        float xscale  = tileExtent.width() / layerExtent.width();
        float yscale  = tileExtent.height() / layerExtent.height();

        texInfoArray->setElement( u++, xoffset );
        texInfoArray->setElement( u++, yoffset );
        texInfoArray->setElement( u++, xscale );
        texInfoArray->setElement( u++, yscale );
    }

    texture->setTextureSize( texWidth, texHeight, layerImages.size() );

    // configure the mipmapping 
    texture->setMaxAnisotropy(16.0f);
    texture->setResizeNonPowerOfTwoHint(false);
    texture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
    bool powerOfTwo = texWidth > 0 && (texWidth & (texWidth - 1)) && texHeight > 0 && (texHeight & (texHeight - 1));
    if ( powerOfTwo )
        texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
    else
        texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );

    // configure the wrapping
    texture->setWrap(osg::Texture::WRAP_S,osg::Texture::CLAMP_TO_EDGE);
    texture->setWrap(osg::Texture::WRAP_T,osg::Texture::CLAMP_TO_EDGE);
    //texture->setWrap(osg::Texture::WRAP_R,osg::Texture::REPEAT);

    // (note: do NOT call setTextureAttributeAndModes -- opengl error since TEXTURE_2D_ARRAY is not a valid mode)
    //TODO: un-hard-code the texture unit, perhaps
    stateSet->setTextureAttribute( 0, texture, osg::StateAttribute::ON ); 
    stateSet->addUniform( texInfoArray );
    stateSet->addUniform( new osg::Uniform("tex0", 0) );

    return stateSet;
}
#endif

void
TextureCompositorTexArray::updateMasterStateSet( osg::StateSet* stateSet, const TextureLayout& layout ) const
{
    VirtualProgram* vp = static_cast<VirtualProgram*>( stateSet->getAttribute(osg::StateAttribute::PROGRAM) );
    vp->setShader( "osgearth_frag_texture", s_createTextureFragShaderFunction(layout) );
}
