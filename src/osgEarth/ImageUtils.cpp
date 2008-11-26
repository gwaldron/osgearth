#include <osgEarth/ImageUtils>

#include <osg/Notify>
#include <osg/Texture>
#include <osg/Timer>

#include <squish.h>


using namespace osgEarth;

using namespace squish;


bool ImageUtils::canCompress(const osg::Image* image)
{
    return (image &&
            (image->getPixelFormat() == GL_RGB || image->getPixelFormat() == GL_RGBA) &&
            ( ( ( image->s() % 4 ) == 0) && ( ( image->t() % 4 ) == 0 ) )
            );
}

osg::Image* 
ImageUtils::compress(const osg::Image *image, Quality quality)
{
    if (!canCompress(image)) return NULL;

    //osg::Timer_t start_tick = osg::Timer::instance()->tick();

    //osg::notify(osg::NOTICE) << "Compressing to DDS" << std::endl;
	// get the image info
	int width = image->s();
	int height = image->t();
	int stride = image->getPixelSizeInBits() / 8;

    GLenum output_format;
    int method;

    if (image->getPixelFormat() == GL_RGB)
    {
        output_format = GL_COMPRESSED_RGBA_S3TC_DXT1_EXT;
        method = kDxt1;
    }
    else if (image->getPixelFormat() == GL_RGBA)
    {
        method = kDxt5;
        output_format = GL_COMPRESSED_RGBA_S3TC_DXT5_EXT;
    }
    
	int metric = kColourMetricPerceptual;
	int fit = kColourRangeFit;
    switch (quality)
    {
    case LOW: fit = kColourRangeFit; break;
    case MED: fit = kColourClusterFit; break;
    case HIGH: fit = kColourIterativeClusterFit; break;
    }

    int flags = method | metric | fit;

	// create the target data
	int bytesPerBlock = ( ( flags & kDxt1 ) != 0 ) ? 8 : 16;
	int targetDataSize = bytesPerBlock*width*height/16;

    u8 *targetBlock = new u8[targetDataSize];

    osg::ref_ptr<osg::Image> output_image = new osg::Image;
    output_image->setImage(width, height, 1, output_format, output_format, GL_UNSIGNED_BYTE, targetBlock, osg::Image::USE_NEW_DELETE);

    bool alpha = (image->getPixelFormat() == GL_RGBA);

    for( int y = 0; y < height; y += 4 )
	{
		// process a row of blocks
		for( int x = 0; x < width; x += 4 )
		{
			// get the block data
			u8 sourceRgba[16*4];
			for( int py = 0, i = 0; py < 4; ++py )
			{
				//u8 const* row = sourceImage.GetRow( y + py ) + x*stride;
                u8 const* row = image->data(x, y + py);
				for( int px = 0; px < 4; ++px, ++i )
				{
					// get the pixel colour 
                    for( int j = 0; j < 3; ++j )
                        sourceRgba[4*i + j] = *row++;

					// skip alpha for now
					if( alpha )
						sourceRgba[4*i + 3] = *row++;
					else
						sourceRgba[4*i + 3] = 255;
				}
			}
			
			// compress this block
			Compress( sourceRgba, targetBlock, flags );
			
			// advance
			targetBlock += bytesPerBlock;			
		}
	}
    //delete[] targetBlock;

    //osg::Timer_t end_tick = osg::Timer::instance()->tick();
    //osg::notify(osg::NOTICE)<<"Compressed to DDS "<<osg::Timer::instance()->delta_m(start_tick,end_tick)<<"ms"<<std::endl;

    return output_image.release();
}

bool
ImageUtils::isCompressed(const osg::Image* image)
{
    return ((image->getInternalTextureFormat() == GL_COMPRESSED_RGBA_S3TC_DXT5_EXT) ||
            (image->getInternalTextureFormat() == GL_COMPRESSED_RGBA_S3TC_DXT3_EXT) ||
            (image->getInternalTextureFormat() == GL_COMPRESSED_RGB_S3TC_DXT1_EXT) ||
            (image->getInternalTextureFormat() == GL_COMPRESSED_RGBA_S3TC_DXT1_EXT));
}

osg::Image* ImageUtils::decompress(const osg::Image* image, GLint destFormat)
{
    //Do nothing if the image is not compressed
    if (!isCompressed(image)) return 0;

    //osg::Timer_t start_tick = osg::Timer::instance()->tick();

    //osg::notify(osg::NOTICE) << "Decompressing DDS" << std::endl;
	// get the image info
	int width = image->s();
	int height = image->t();
    int bytesPerBlock = 16;
    int flags = kDxt5;
    
    if (image->getInternalTextureFormat() == GL_COMPRESSED_RGB_S3TC_DXT1_EXT ||
        image->getInternalTextureFormat() == GL_COMPRESSED_RGBA_S3TC_DXT1_EXT)
    {
        bytesPerBlock = 8;
        flags = kDxt1;
    }
    else if (image->getInternalTextureFormat() == GL_COMPRESSED_RGBA_S3TC_DXT5_EXT)
    {
        bytesPerBlock = 16;
        flags = kDxt5;
    }
    else if (image->getInternalTextureFormat() == GL_COMPRESSED_RGBA_S3TC_DXT3_EXT)
    {
        bytesPerBlock = 16;
        flags = kDxt3;
    }
    
	int targetDataSize = width*height*4;

    u8 *targetBlock = new u8[targetDataSize];


    int numComponents = 3;
    if (destFormat == GL_RGB)
    {
        numComponents = 3;
    }
    else if (destFormat = GL_RGBA)
    {
        numComponents = 4;
    }
    else
    {
        osg::notify(osg::NOTICE) << "Dest format " <<  destFormat << " unhandled " << std::endl;
        return 0;
    }

    osg::ref_ptr<osg::Image> output_image = new osg::Image;
    output_image->setImage(width, height, 1, destFormat, destFormat, GL_UNSIGNED_BYTE, targetBlock, osg::Image::USE_NEW_DELETE);

    u8 const* sourceBlock = image->data();

    for( int y = 0; y < height; y += 4 )
	{
		// process a row of blocks
		for( int x = 0; x < width; x += 4 )
		{
			// get the block data
			u8 targetRgba[16*4];
            Decompress( targetRgba, sourceBlock, flags );

            for( int py = 0, i = 0; py < 4; ++py )
			{
                u8* row = output_image->data(x, y + py);
				for( int px = 0; px < 4; ++px, ++i )
				{
					// get the pixel colour 
                    for( int j = 0; j < numComponents; ++j )
                        *row++ = targetRgba[4*i + j];
				}
			}

			// advance
			sourceBlock += bytesPerBlock;			
		}
	}

    //osg::Timer_t end_tick = osg::Timer::instance()->tick();
    //osg::notify(osg::NOTICE)<<"Decompressed DDS "<<osg::Timer::instance()->delta_m(start_tick,end_tick)<<"ms"<<std::endl;

    return output_image.release();
}