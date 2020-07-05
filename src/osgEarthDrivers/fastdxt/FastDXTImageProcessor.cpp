/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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

#include <osg/Texture>
#include <osgDB/Registry>
#include <osg/Notify>
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
    virtual void compress(osg::Image& input, osg::Texture::InternalFormatMode compressedFormat, bool generateMipMap, bool resizeToPowerOfTwo, CompressionMethod method, CompressionQuality quality)
    {
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

        int format;
        GLint pixelFormat;
        switch (compressedFormat)
        {
        case osg::Texture::USE_S3TC_DXT1_COMPRESSION:
            format = FORMAT_DXT1;
            pixelFormat = GL_COMPRESSED_RGB_S3TC_DXT1_EXT;
            OE_DEBUG << "FastDXT using dxt1 format" << std::endl;
            break;
        case osg::Texture::USE_S3TC_DXT5_COMPRESSION:
            format = FORMAT_DXT5;
            pixelFormat = GL_COMPRESSED_RGBA_S3TC_DXT5_EXT;
            OE_DEBUG << "FastDXT dxt5 format" << std::endl;
            break;
        default:
            OSG_WARN << "Unhandled compressed format" << compressedFormat << std::endl;
            return;
            break;
        }

        if (generateMipMap)
        {
            std::vector<unsigned char*> inputLevels;
            std::vector<unsigned char*> mipLevels;
            std::vector<unsigned> lengths;

            // first, build the image that will hold all the mipmap levels.
            int numLevels = osg::Image::computeNumberOfMipmapLevels(sourceImage->s(), sourceImage->t(), sourceImage->r());
            int imageSizeBytes = sourceImage->getTotalSizeInBytes();

            // offset vector does not include level 0 (the full-resolution level)
            osg::Image::MipmapDataType mipOffsets;
            mipOffsets.reserve(numLevels-1);

            // now, populate the image levels.
            osg::PixelStorageModes psm;
            psm.pack_alignment = sourceImage->getPacking();
            psm.pack_row_length = sourceImage->getRowLength();
            psm.unpack_alignment = sourceImage->getPacking();

            unsigned char* workspace = (unsigned char*)memalign(16, imageSizeBytes);
            unsigned totalBytes = 0;
            unsigned char* in;

            for(int level=0; level<numLevels; ++level)
            {
                int level_s = sourceImage->s() >> level;
                int level_t = sourceImage->t() >> level;

                if (level == 0)
                {
                    inputLevels.push_back(sourceImage->data());
                    in = sourceImage->data();
                }
                else
                {
                    // store offset of next mip level
                    mipOffsets.push_back(totalBytes);

                    // OSG-custom gluScaleImage that does not require a graphics context
                    GLint status = gluScaleImage(
                        &psm,
                        sourceImage->getPixelFormat(),
                        sourceImage->s(),
                        sourceImage->t(),
                        sourceImage->getDataType(),
                        sourceImage->data(),
                        level_s,
                        level_t,
                        sourceImage->getDataType(),
                        workspace);

                    in = workspace;
                }

                unsigned char* out = (unsigned char*)memalign(16, imageSizeBytes);
                memset(out, 0, imageSizeBytes);
                mipLevels.push_back(out);

                int outputBytes = CompressDXT(
                    in,
                    out, 
                    level_s, 
                    level_t, 
                    format);

                lengths.push_back(outputBytes);
                totalBytes += outputBytes;
            }

            memfree(workspace);

            // now combine into a new mipmapped compressed image.
            unsigned char* data = new unsigned char[totalBytes];
            unsigned char* ptr = data;
            for(unsigned i=0; i<mipLevels.size(); ++i)
            {
                ::memcpy(ptr, mipLevels[i], lengths[i]);
                ptr += lengths[i];
                memfree(mipLevels[i]);
            }

            input.setImage(
                sourceImage->s(),
                sourceImage->t(),
                sourceImage->r(),
                pixelFormat, 
                pixelFormat, 
                GL_UNSIGNED_BYTE, 
                data, 
                osg::Image::USE_NEW_DELETE);

            input.setMipmapLevels(mipOffsets);
        }

        else // no mipmaps, just one level
        {
            //Copy over the source data to an array
            unsigned char *in = 0;
            in = (unsigned char*)memalign(16, sourceImage->getTotalSizeInBytes());
            memcpy(in, sourceImage->data(0,0), sourceImage->getTotalSizeInBytes());

            //Allocate memory for the output
            unsigned char* out = (unsigned char*)memalign(16, input.s()*input.t()*4);
            memset(out, 0, input.s()*input.t()*4);

            osg::Timer_t start = osg::Timer::instance()->tick();
            int outputBytes = CompressDXT(in, out, sourceImage->s(), sourceImage->t(), format);
            osg::Timer_t end = osg::Timer::instance()->tick();
            OE_DEBUG << "compression took" << osg::Timer::instance()->delta_m(start, end) << std::endl;

            //Allocate and copy over the output data to the correct size array.
            unsigned char* data = (unsigned char*)malloc(outputBytes);
            memcpy(data, out, outputBytes);
            memfree(out);
            memfree(in);
            input.setImage(input.s(), input.t(), input.r(), pixelFormat, pixelFormat, GL_UNSIGNED_BYTE, data, osg::Image::USE_MALLOC_FREE);
        }
    }

    virtual void generateMipMap(osg::Image& image, bool resizeToPowerOfTwo, CompressionMethod method)
    {
        OSG_WARN << "FastDXT: generateMipMap not implemented" << std::endl;
    }
};

REGISTER_OSGIMAGEPROCESSOR(fastdxt, FastDXTProcessor)
