
#include <osg/Texture>
#include <osgDB/Registry>
#include <osgDB/WriteFile>

#include <osgEarth/ImageUtils>

#include "libdxt.h"
#include <string.h>

class FastDXTImageProcessor : public osgDB::ImageProcessor
{
public:
    virtual void compress(osg::Image& image, osg::Texture::InternalFormatMode compressedFormat, bool generateMipMap, bool resizeToPowerOfTwo, CompressionMethod method, CompressionQuality quality);
    virtual void generateMipMap(osg::Image& image, bool resizeToPowerOfTwo, CompressionMethod method);

protected:



};


void FastDXTImageProcessor::compress(osg::Image& image, osg::Texture::InternalFormatMode compressedFormat, bool generateMipMap, bool resizeToPowerOfTwo, CompressionMethod method, CompressionQuality quality)
{
    //Resize the image to the nearest power of two 
    if (!osgEarth::ImageUtils::isPowerOfTwo( &image ))
    {            
        OE_NOTICE << "Resizing" << std::endl;
        unsigned int s = osg::Image::computeNearestPowerOfTwo( image.s() );
        unsigned int t = osg::Image::computeNearestPowerOfTwo( image.t() );
        image.scaleImage(s, t, image.r());
    }

    //Allocate memory for the output
    unsigned char* out = (unsigned char*)memalign(16, image.s()*image.t()*4);
    memset(out, 0, image.s()*image.t()*4);

    osg::Image* sourceImage = &image;

    //FastDXT only works on RGBA imagery so we must convert it
    osg::ref_ptr< osg::Image > rgba;
    if (image.getPixelFormat() != GL_RGBA)
    {
        OSG_INFO << "Converting to RGBA" << std::endl;
        osg::Timer_t start = osg::Timer::instance()->tick();
        rgba = osgEarth::ImageUtils::convertToRGBA8( &image );
        osg::Timer_t end = osg::Timer::instance()->tick();
        OSG_INFO << "conversion to rgba took" << osg::Timer::instance()->delta_m(start, end) << std::endl;
        sourceImage = rgba.get();
    }

    //Copy over the source data to an array
    unsigned char *in = 0;
    in = (unsigned char*)memalign(16, sourceImage->getTotalSizeInBytes());
    memcpy(in, sourceImage->data(0,0), sourceImage->getTotalSizeInBytes());

    int format;
    GLint pixelFormat;
    switch (compressedFormat)
    {
    case osg::Texture::USE_S3TC_DXT1_COMPRESSION:
        format = FORMAT_DXT1;
        pixelFormat = GL_COMPRESSED_RGB_S3TC_DXT1_EXT;
        break;
    case osg::Texture::USE_S3TC_DXT5_COMPRESSION:
        format = FORMAT_DXT5;
        pixelFormat = GL_COMPRESSED_RGBA_S3TC_DXT5_EXT;
        break;
    default:
        OSG_WARN << "Unhandled compressed format" << compressedFormat << std::endl;
        return;
        break;
    }    

    unsigned int numThreads = 1;
    int outputBytes = CompressDXT(in, out, sourceImage->s(), sourceImage->t(), format, numThreads);

    //Allocate and copy over the output data to the correct size array.
    unsigned char* data = (unsigned char*)malloc(outputBytes);
    memcpy(data, out, outputBytes);
    free(out);
    free(in);

    image.setImage(image.s(), image.t(), image.r(), pixelFormat, pixelFormat, GL_UNSIGNED_BYTE, data, osg::Image::USE_MALLOC_FREE);
}

void FastDXTImageProcessor::generateMipMap(osg::Image& image, bool resizeToPowerOfTwo, CompressionMethod method)
{    
    OSG_WARN << "FastDXT: generateMipMap not implemented" << std::endl;
}

REGISTER_OSGIMAGEPROCESSOR(nvtt, FastDXTImageProcessor)