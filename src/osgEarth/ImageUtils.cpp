#include <osgEarth/ImageUtils>

#include <osg/Notify>
#include <osg/Texture>
#include <osg/Timer>

#include <squish.h>


using namespace osgEarth;

using namespace squish;


bool ImageUtils::canDDSCompress(const osg::Image* image)
{
    return (image &&
            (image->getPixelFormat() == GL_RGB || image->getPixelFormat() == GL_RGBA) &&
            ( ( ( image->s() % 4 ) == 0) && ( ( image->t() % 4 ) == 0 ) )
            );
}

osg::Image* 
ImageUtils::convertRGBAtoDDS(const osg::Image *image)
{
    if (!canDDSCompress(image)) return NULL;

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
        output_format = GL_COMPRESSED_RGB_S3TC_DXT1_EXT;
        method = kDxt1;
    }
    else if (image->getPixelFormat() == GL_RGBA)
    {
        method = kDxt5;
        output_format = GL_COMPRESSED_RGBA_S3TC_DXT5_EXT;
    }
    
	int metric = kColourMetricPerceptual;
	int fit = kColourRangeFit;

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