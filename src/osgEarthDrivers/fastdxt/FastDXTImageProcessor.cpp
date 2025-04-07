/* osgEarth
* Copyright 2008-2013 Pelican Mapping
* MIT License
*/

#include <osg/Texture>
#include <osgDB/Registry>
#include <osgEarth/Notify>
#include <osg/GLU>
#include <osgEarth/ImageUtils>
#include <stdlib.h>
#include "libdxt.h"
#include <string.h>


using namespace osgEarth;
using namespace osgEarth::Util;

class FastDXTProcessor : public osgDB::ImageProcessor
{
public:
    virtual void compress(
        osg::Image& input, 
        osg::Texture::InternalFormatMode compressedFormat, 
        bool generateMipMap,
        bool resizeToPowerOfTwo, 
        CompressionMethod method, 
        CompressionQuality quality)
    {
        // Gotta be at least 16x16
        if (input.s() < 16 || input.t() < 16)
            return;

        // Not already compressed?
        if (input.isCompressed())
            return;

        //Resize the image to the nearest power of two
        if (!ImageUtils::isPowerOfTwo( &input ))
        {
            unsigned int s = osg::Image::computeNearestPowerOfTwo( input.s() );
            unsigned int t = osg::Image::computeNearestPowerOfTwo( input.t() );
            input.scaleImage(s, t, input.r());
        }

        osg::Image* sourceImage = &input;

        //FastDXT only works on RGBA imagery so we must convert it
        osg::ref_ptr< osg::Image > rgba;
        if (input.getPixelFormat() != GL_RGBA)
        {
            rgba = ImageUtils::convertToRGBA8( &input );
            sourceImage = rgba.get();
        }

        OE_SOFT_ASSERT_AND_RETURN(sourceImage != nullptr, void());

        int format;
        GLenum compressedPixelFormat;
        int minLevelSize;

        switch (compressedFormat)
        {
        case osg::Texture::USE_S3TC_DXT1_COMPRESSION:
            format = FORMAT_DXT1;
            compressedPixelFormat = GL_COMPRESSED_RGBA_S3TC_DXT1_EXT;
            minLevelSize = 8;
            break;
        case osg::Texture::USE_S3TC_DXT5_COMPRESSION:
            format = FORMAT_DXT5;
            compressedPixelFormat = GL_COMPRESSED_RGBA_S3TC_DXT5_EXT;
            minLevelSize = 16;
            break;
        default:
            OSG_WARN << "Unhandled compressed format" << compressedFormat << std::endl;
            return;
            break;
        }

        if (generateMipMap)
        {
            // size in bytes of the top-level image set (sum of [0..r-1])
            int levelZeroSizeBytes = sourceImage->getTotalSizeInBytes();

            // Mipmap the image
            osg::ref_ptr< const osg::Image > mipmapped = ImageUtils::mipmapImage(sourceImage, minLevelSize);

            // data ptr of each mipmap level [0..r-1]
            std::vector<unsigned char*> mipLevels;

            // length in bytes of each mipmap level [0..r-1]
            std::vector<unsigned> mipLevelBytes;

            // offset vector does not include level 0 (the full-resolution level)
            osg::Image::MipmapDataType mipOffsets;
            mipOffsets.reserve(mipmapped->getNumMipmapLevels());

            // now, populate the image levels.
            osg::PixelStorageModes psm;
            psm.pack_alignment = sourceImage->getPacking();
            psm.pack_row_length = sourceImage->getRowLength();
            psm.unpack_alignment = sourceImage->getPacking();

            const unsigned char* in = nullptr;

            unsigned totalCompressedBytes = 0u;

            // interate over mipmap levels:
            unsigned int numLevels = mipmapped->getNumMipmapLevels();
            for (unsigned level = 0; level < numLevels; ++level)
            {
                int level_s = sourceImage->s() >> level;
                int level_t = sourceImage->t() >> level;

                std::size_t levelAllocatedBytes = sourceImage->r() * (levelZeroSizeBytes); // max possible size
                unsigned char* compressedLevelDataPtr = (unsigned char*)memalign(16, levelAllocatedBytes);
                ::memset(compressedLevelDataPtr, 0, levelAllocatedBytes);
                mipLevels.push_back(compressedLevelDataPtr);

                unsigned levelCompressedBytes = 0u;

                in = mipmapped->getMipmapData(level);

                if (level != 0)
                {
                    // when we start a new mip level that's > 0, record its offset.
                    mipOffsets.push_back(totalCompressedBytes);
                }

                int outputBytes = CompressDXT(
                    in,
                    compressedLevelDataPtr,
                    level_s,
                    level_t,
                    format);

                // advance our counters:
                levelCompressedBytes += outputBytes;
                totalCompressedBytes += outputBytes;

                // advance output pointer:
                compressedLevelDataPtr += outputBytes;

                // record the data offset of this mip level
                mipLevelBytes.push_back(levelCompressedBytes);
            }

            // now combine into a new mipmapped compressed image
            // and delete any workspaces along the way.
            unsigned char* data = new unsigned char[totalCompressedBytes];
            unsigned char* ptr = data;
            for (unsigned i = 0; i < mipLevels.size(); ++i)
            {
                ::memcpy(ptr, mipLevels[i], mipLevelBytes[i]);
                ptr += mipLevelBytes[i];
                memfree(mipLevels[i]);
            }

            input.setImage(
                sourceImage->s(),
                sourceImage->t(),
                sourceImage->r(),
                compressedPixelFormat, // ?
                compressedPixelFormat,
                GL_UNSIGNED_BYTE,
                data,
                osg::Image::USE_NEW_DELETE);

            input.setMipmapLevels(mipOffsets);
        }

        else // no mipmaps, just one level
        {
            //TODO: support r > 1
            //Copy over the source data to an array
            unsigned char *in = 0;
            in = (unsigned char*)memalign(16, sourceImage->getTotalSizeInBytes());
            memcpy(in, sourceImage->data(0,0), sourceImage->getTotalSizeInBytes());

            //Allocate memory for the output
            unsigned char* out = (unsigned char*)memalign(16, input.s()*input.t()*4);
            memset(out, 0, input.s()*input.t()*4);

            int outputBytes = CompressDXT(in, out, sourceImage->s(), sourceImage->t(), format);

            //Allocate and copy over the output data to the correct size array.
            unsigned char* data = (unsigned char*)malloc(outputBytes);
            memcpy(data, out, outputBytes);
            memfree(out);
            memfree(in);
            input.setImage(input.s(), input.t(), input.r(), compressedPixelFormat, compressedPixelFormat, GL_UNSIGNED_BYTE, data, osg::Image::USE_MALLOC_FREE);
        }
    }

    virtual void generateMipMap(osg::Image& image, bool resizeToPowerOfTwo, CompressionMethod method)
    {
        OSG_WARN << "FastDXT: generateMipMap not implemented" << std::endl;
    }
};

REGISTER_OSGIMAGEPROCESSOR(fastdxt, FastDXTProcessor)
