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

#include <osgEarth/ImageUtils>
#include <osg/Notify>
#include <string.h>
#include <memory.h>

#define LC "[ImageUtils] "

using namespace osgEarth;

namespace osgEarth
{
    static const float r10= 1.0f/1023.0f;
    static const float r8 = 1.0f/255.0f;
    static const float r6 = 1.0f/63.0f;
    static const float r5 = 1.0f/31.0f;
    static const float r4 = 1.0f/15.0f;
    static const float r3 = 1.0f/7.0f;
    static const float r2 = 1.0f/3.0f;
}


// todo: add support for other data types as needed.
osg::Vec4
ImageUtils::getColor(const osg::Image* image, int s, int t, int r)
{
    switch( image->getDataType() )
    {
    case( GL_UNSIGNED_SHORT_5_5_5_1 ):
        {
            unsigned short p = *(unsigned short*)image->data( s, t, r );
            //internal format GL_RGB5_A1 is implied
            return osg::Vec4( r5*(float)(p>>11), r5*(float)((p&0x7c0)>>6), r5*((p&0x3e)>>1), (float)(p&0x1) );
        }         
    case( GL_UNSIGNED_BYTE_3_3_2 ):
        {
            unsigned char p = *(unsigned char*)image->data( s, t, r );
            // internal format GL_R3_G3_B2 is implied
            return osg::Vec4( r3*(float)(p>>5), r3*(float)((p&0x28)>>2), r2*(float)(p&0x3), 1.0f );
        }
    }

    // default: let osg::Image handle the usual types
    return image->getColor( s, t, r );
}

bool
ImageUtils::setColor(osg::Image* image, int s, int t, int r, const osg::Vec4& color)
{
    if ( image->getDataType() == GL_UNSIGNED_BYTE )
    {
        unsigned char* p = image->data(s, t, r);
        *p++ = (char)(color.r() * 255.0f);
        *p++ = (char)(color.g() * 255.0f);
        *p++ = (char)(color.b() * 255.0f);
        if ( image->getPixelFormat() == GL_RGBA )
            *p++ = (char)(color.a() * 255.0f);
        return true;
    }
    else 
    if ( image->getDataType() == GL_UNSIGNED_SHORT_5_5_5_1 )
    {
        unsigned short
            r = (unsigned short)(color.r()*255),
            g = (unsigned short)(color.g()*255),
            b = (unsigned short)(color.b()*255),
            a = color.a() < 0.15 ? 0 : 1;

        unsigned short* p = (unsigned short*)image->data(s, t, r);
        *p = (((r) & (0xf8)) << 8) | (((g) & (0xf8)) << 3) | (((b) & (0xF8)) >> 2) | a;
        return true;
    }
    else
    if ( image->getDataType() == GL_UNSIGNED_BYTE_3_3_2 )
    {
        //TODO
        OE_WARN << LC << "setColor(GL_UNSIGNED_BYTE_3_3_2) not yet implemented!" << std::endl;
    }

    return false;
}

bool
ImageUtils::copyAsSubImage(const osg::Image* src, osg::Image* dst, int dst_start_col, int dst_start_row, int dst_img )
{
    if (!src || !dst ||
        dst_start_col + src->s() > dst->s() ||
        dst_start_row + src->t() > dst->t() )
    {
        return false;
    }

    // check for fast bytewise copy:
    if (src->getPacking() == dst->getPacking() &&
        src->getDataType() == dst->getDataType() &&
        src->getPixelFormat() == dst->getPixelFormat() )
    {
        for( int src_row=0, dst_row=dst_start_row; src_row < src->t(); src_row++, dst_row++ )
        {
            const void* src_data = src->data( 0, src_row, 0 );
            void* dst_data = dst->data( dst_start_col, dst_row, dst_img );
            memcpy( dst_data, src_data, src->getRowSizeInBytes() );
        }
    }

    // otherwise loop through an convert pixel-by-pixel.
    else
    {
        for( int src_t=0, dst_t=dst_start_row; src_t < src->t(); src_t++, dst_t++ )
        {
            for( int src_s=0, dst_s=dst_start_col; src_s < src->s(); src_s++, dst_s++ )
            {
                //TODO: port to PixelReader/Writer
                setColor( dst, dst_s, dst_t, dst_img, getColor(src, src_s, src_t) );                
            }
        }
    }

    return true;
}

#if 0
bool
ImageUtils::copyAsSubImage( const osg::Image* src, osg::Image* dst, int dst_start_col, int dst_start_row )
{
    if (!src || !dst || 
        src->getPacking() != dst->getPacking() || 
        src->getDataType() != dst->getDataType() || 
        src->getPixelFormat() != dst->getPixelFormat() ||
        dst_start_col + src->s() > dst->s() ||
        dst_start_row + src->t() > dst->t() )
    {
        return false;
    }

    for( int src_row=0, dst_row=dst_start_row; src_row < src->t(); src_row++, dst_row++ )
    {
        const void* src_data = src->data( 0, src_row, 0 );
        void* dst_data = dst->data( dst_start_col, dst_row, 0 );
        memcpy( dst_data, src_data, src->getRowSizeInBytes() );
    }

    return true;
}
#endif

bool
ImageUtils::resizeImage(const osg::Image* input, 
                        unsigned int out_s, unsigned int out_t, 
                        osg::ref_ptr<osg::Image>& output,
                        unsigned int mipmapLevel )
{
    if ( !input && out_s == 0 && out_t == 0 )
        return false;

    GLenum pf = input->getPixelFormat();

    //if ( pf != GL_RGBA && pf != GL_RGB && pf != GL_LUMINANCE && pf != GL_RED && pf != GL_LUMINANCE_ALPHA )
    //{
    //    OE_WARN << LC << "resizeImage: unsupported pixel format " << std::hex << pf << std::endl;
    //    return 0L;
    //}

    unsigned int in_s = input->s();
    unsigned int in_t = input->t();

    if ( !output.valid() )
    {
        output = new osg::Image();
        output->allocateImage( out_s, out_t, 1, pf, input->getDataType(), input->getPacking() );
    }
    output->setInternalTextureFormat( input->getInternalTextureFormat() );

    if ( in_s == out_s && in_t == out_t && mipmapLevel == 0 )
    {
        memcpy( output->getMipmapData(mipmapLevel), input->data(), input->getTotalSizeInBytes() );
    }
    else
    {       
        float s_ratio = (float)in_s/(float)out_s;
        float t_ratio = (float)in_t/(float)out_t;
        unsigned int pixel_size_bytes = input->getRowSizeInBytes() / in_s;

        unsigned char* dataOffset = output->getMipmapData(mipmapLevel);
        unsigned int   dataRowSizeBytes = output->getRowSizeInBytes() >> mipmapLevel;

        for( unsigned int output_row=0; output_row < out_t; output_row++ )
        {
            // get an appropriate input row
            float output_row_ratio = (float)output_row/(float)out_t;
            int input_row = (unsigned int)( output_row_ratio * (float)in_t );
            if ( input_row >= input->t() ) input_row = in_t-1;
            else if ( input_row < 0 ) input_row = 0;

            for( unsigned int output_col = 0; output_col < out_s; output_col++ )
            {
                float output_col_ratio = (float)output_col/(float)out_s;
                int input_col = (unsigned int)( output_col_ratio * (float)in_s );
                if ( input_col >= in_s ) input_col = in_s-1;
                else if ( input_row < 0 ) input_row = 0;

                unsigned char* outaddr =
                    dataOffset + 
                    (output_col*output->getPixelSizeInBits())/8+output_row*dataRowSizeBytes;

                memcpy(
                    outaddr,
                    input->data( input_col, input_row ),
                    pixel_size_bytes );
            }
        }
    }

    return true;
}

osg::Image*
ImageUtils::createMipmapBlendedImage( const osg::Image* primary, const osg::Image* secondary )
{
    // ASSUMPTION: primary and secondary are the same size, same format.

    // first, build the image that will hold all the mipmap levels.
    int numMipmapLevels = osg::Image::computeNumberOfMipmapLevels( primary->s(), primary->t() );
    int pixelSizeBytes  = osg::Image::computeRowWidthInBytes( primary->s(), primary->getPixelFormat(), primary->getDataType(), primary->getPacking() ) / primary->s();
    int totalSizeBytes  = 0;
    std::vector< unsigned int > mipmapDataOffsets;

    mipmapDataOffsets.reserve( numMipmapLevels-1 );

    for( int i=0; i<numMipmapLevels; ++i )
    {
        if ( i > 0 )
            mipmapDataOffsets.push_back( totalSizeBytes );

        int level_s = primary->s() >> i;
        int level_t = primary->t() >> i;
        int levelSizeBytes = level_s * level_t * pixelSizeBytes;

        totalSizeBytes += levelSizeBytes;
    }

    unsigned char* data = new unsigned char[totalSizeBytes];

    osg::ref_ptr<osg::Image> result = new osg::Image();
    result->setImage(
        primary->s(), primary->t(), 1,
        primary->getInternalTextureFormat(), 
        primary->getPixelFormat(), 
        primary->getDataType(), 
        data, osg::Image::USE_NEW_DELETE );

    result->setMipmapLevels( mipmapDataOffsets );

    // now, populate the image levels.
    int level_s = primary->s();
    int level_t = primary->t();

    for( int level=0; level<numMipmapLevels; ++level )
    {
        if ( secondary && level > 0 )
            ImageUtils::resizeImage( secondary, level_s, level_t, result, level );
        else
            ImageUtils::resizeImage( primary, level_s, level_t, result, level );

        level_s >>= 1;
        level_t >>= 1;
    }

    return result.release();
}

bool
ImageUtils::mix(osg::Image* dest, const osg::Image* src, float a)
{
    if (!dest || !src || dest->s() != src->s() || dest->t() != src->t() )
        return false;

    a = osg::clampBetween( a, 0.0f, 1.0f );

    PixelReader src_reader( src );
    PixelReader dest_reader( dest );
    PixelWriter dest_writer( dest );

    for( int r=0; r<dest->r(); ++r )
    {
        for( int s=0; s<dest->s(); ++s )
        {
            for( int t=0; t<dest->t(); ++t )
            {
                osg::Vec4f srcColor  = src_reader( s, t, r );
                osg::Vec4f destColor = dest_reader( s, t, r );

                float sa = src->getPixelSizeInBits() == 32 ? a * srcColor.a() : a;
                float da = dest->getPixelSizeInBits() == 32 ? destColor.a() : 1.0f;

                dest_writer( s, t, r, osg::Vec4(
                    destColor.r()*(1.0f-sa) + srcColor.r()*sa,
                    destColor.g()*(1.0f-sa) + srcColor.g()*sa,
                    destColor.b()*(1.0f-sa) + srcColor.b()*sa,
                    destColor.a() ) );
            }
        }
    }

    return true;
}

osg::Image*
ImageUtils::cropImage(const osg::Image* image,
                      double src_minx, double src_miny, double src_maxx, double src_maxy,
                      double &dst_minx, double &dst_miny, double &dst_maxx, double &dst_maxy)
{
    //Compute the desired cropping rectangle
    int windowX        = osg::clampBetween( (int)floor( (dst_minx - src_minx) / (src_maxx - src_minx) * (double)image->s()), 0, image->s()-1);
    int windowY        = osg::clampBetween( (int)floor( (dst_miny - src_miny) / (src_maxy - src_miny) * (double)image->t()), 0, image->t()-1);
    int windowWidth    = osg::clampBetween( (int)ceil(  (dst_maxx - src_minx) / (src_maxx - src_minx) * (double)image->s()) - windowX, 0, image->s());
    int windowHeight   = osg::clampBetween( (int)ceil(  (dst_maxy - src_miny) / (src_maxy - src_miny) * (double)image->t()) - windowY, 0, image->t());    

    if (windowX + windowWidth > image->s())
    {
        windowWidth = image->s() - windowX;        
    }

    if (windowY + windowHeight > image->t())
    {
        windowHeight = image->t() - windowY;
    }
    
    if ((windowWidth * windowHeight) == 0)
    {
        return NULL;
    }

    //Compute the actual bounds of the area we are computing
    double res_s = (src_maxx - src_minx) / (double)image->s();
    double res_t = (src_maxy - src_miny) / (double)image->t();

    dst_minx = src_minx + (double)windowX * res_s;
    dst_miny = src_miny + (double)windowY * res_t;
    dst_maxx = dst_minx + (double)windowWidth * res_s;
    dst_maxy = dst_miny + (double)windowHeight * res_t;

    //OE_NOTICE << "Copying from " << windowX << ", " << windowY << ", " << windowWidth << ", " << windowHeight << std::endl;

    //Allocate the croppped image
    osg::Image* cropped = new osg::Image;
    cropped->allocateImage(windowWidth, windowHeight, 1, image->getPixelFormat(), image->getDataType());
    cropped->setInternalTextureFormat( image->getInternalTextureFormat() );
    
    
    for (int src_row = windowY, dst_row=0; dst_row < windowHeight; src_row++, dst_row++)
    {
        if (src_row > image->t()-1) OE_NOTICE << "HeightBroke" << std::endl;
        const void* src_data = image->data(windowX, src_row, 0);
        void* dst_data = cropped->data(0, dst_row, 0);
        memcpy( dst_data, src_data, cropped->getRowSizeInBytes());
    }

    return cropped;
}

bool
ImageUtils::isPowerOfTwo(const osg::Image* image)
{
    return (((image->s() & (image->s()-1))==0) &&
            ((image->t() & (image->t()-1))==0));
}


osg::Image*
ImageUtils::sharpenImage( const osg::Image* input )
{
    int filter[9] = { 0, -1, 0, -1, 5, -1, 0, -1, 0 };
    osg::Image* output = new osg::Image( *input );
    for( int t=1; t<input->t()-1; t++ )
    {
        for( int s=1; s<input->s()-1; s++ )
        {
            int pixels[9] = {
                *(int*)input->data(s-1,t-1), *(int*)input->data(s,t-1), *(int*)input->data(s+1,t-1),
                *(int*)input->data(s-1,t  ), *(int*)input->data(s,t  ), *(int*)input->data(s+1,t  ),
                *(int*)input->data(s-1,t+1), *(int*)input->data(s,t+1), *(int*)input->data(s+1,t+1) };

            int shifts[4] = { 0, 8, 16, 32 };

            for( int c=0; c<4; c++ ) // components
            {
                int mask = 0xff << shifts[c];
                int sum = 0;
                for( int i=0; i<9; i++ )
                {
                    sum += ((pixels[i] & mask) >> shifts[c]) * filter[i];
                }
                sum = sum > 255? 255 : sum < 0? 0 : sum;
                output->data(s,t)[c] = sum;
            }
        }
    }
    return output;
}


osg::Image*
ImageUtils::createEmptyImage()
{
    osg::Image* image = new osg::Image;
    image->allocateImage(1,1,1, GL_RGBA, GL_UNSIGNED_BYTE);
    unsigned char *data = image->data(0,0);
    memset(data, 0, 4);
    return image;
}

osg::Image*
ImageUtils::convertToRGB8(const osg::Image *image)
{
	if (image)
	{
        if ( image->getInternalTextureFormat() == GL_RGB8 )
        {
            return new osg::Image( *image );
        }
		////If the image is already RGB, clone it and return
		//if (image->getPixelFormat() == GL_RGB)
  //      {
  //          return new osg::Image(*image);
  //      }

		else if (image->getPixelFormat() == GL_RGBA)
		{
			osg::Image* result = new osg::Image();
			result->allocateImage(image->s(), image->t(), image->r(), GL_RGB, GL_UNSIGNED_BYTE);
            result->setInternalTextureFormat( GL_RGB );

			for (int s = 0; s < image->s(); ++s)
			{
				for (int t = 0; t < image->t(); ++t)
				{
					result->data(s,t)[0] = image->data(s, t)[0];
					result->data(s,t)[1] = image->data(s, t)[1];
					result->data(s,t)[2] = image->data(s, t)[2];
				}
			}

			return result;
		}
		else
		{
			//TODO:  Handle other cases
            OE_WARN << LC << "convertToRGB: pixelFormat " << std::hex << image->getPixelFormat() << " not yet supported " << std::endl;
		}
	}

	return NULL;
}

osg::Image*
ImageUtils::convertToRGBA8(const osg::Image* image)
{
    if ( image )
    {
        if ( image->getInternalTextureFormat() == GL_RGBA8 )
        {
            return new osg::Image( *image );
        }
        //if ( image->getPixelFormat() == GL_RGBA )
        //{
        //    return new osg::Image( *image );
        //}

        else
        {
            osg::Image* result = new osg::Image();
            result->allocateImage( image->s(), image->t(), image->r(), GL_RGBA, GL_UNSIGNED_BYTE );
            result->setInternalTextureFormat( GL_RGBA8 );

            struct CopyImage : public PixelVisitor {
                bool operator()(osg::Vec4f& pixel) { return true; }
            };

            CopyImage().accept( image, result );

#if 0
            PixelReader reader(image);
            //PixelReader ra(result);
            PixelWriter writer(result);
            for( int r=0; r<image->r(); ++r )
            {
                for( int s=0; s<image->s(); ++s )
                {
                    for( int t=0; t<image->t(); ++t )
                    {
                        writer( s, t, r, reader(s, t, r) );
                        //osg::Vec4f color = reader( s, t, r );
                        //writer( s, t, r, color );

                        //GLubyte* data
                        //    = const_cast<GLubyte*>(ra.data( s, t, r ));
                        //*data++ = (unsigned char)(color.r()*255.0f);
                        //*data++ = (unsigned char)(color.g()*255.0f);
                        //*data++ = (unsigned char)(color.b()*255.0f);
                        //*data++ = (unsigned char)(color.a()*255.0f);
                    }
                }
            }
#endif
             
            return result;
        }

    }
    return 0L;
}

bool 
ImageUtils::areEquivalent(const osg::Image *lhs, const osg::Image *rhs)
{
	if (lhs == rhs) return true;

	if ((lhs->s() == rhs->s()) &&
		(lhs->t() == rhs->t()) &&
		(lhs->getInternalTextureFormat() == rhs->getInternalTextureFormat()) &&
		(lhs->getPixelFormat() == rhs->getPixelFormat()) &&
		(lhs->getDataType() == rhs->getDataType()) &&
		(lhs->getPacking() == rhs->getPacking()) &&
		(lhs->getImageSizeInBytes() == rhs->getImageSizeInBytes()))
	{
		unsigned int size = lhs->getImageSizeInBytes();
        const unsigned char* ptr1 = lhs->data();
        const unsigned char* ptr2 = rhs->data();
		for (unsigned int i = 0; i < size; ++i)
		{
            if ( *ptr1++ != *ptr2++ )
                return false;
			//if (lhs->data()[i] != rhs->data()[i])
			//{
			//	return false;
			//}
		}
	}
	return true;
}

bool
ImageUtils::hasAlphaChannel(const osg::Image* image) 
{
    return image && (
        image->getPixelFormat() == GL_RGBA ||
        image->getPixelFormat() == GL_BGRA ||
        image->getPixelFormat() == GL_LUMINANCE_ALPHA );
}

namespace
{
    // The scale factors to convert from an image data type to a
    // float. This is copied from OSG; I think the factors for the signed
    // types are wrong, but need to investigate further.

    template<typename T> struct GLTypeTraits;

    template<> struct GLTypeTraits<GLbyte>
    {
        static float scale() { return 1.0f/128.0f; } // XXX
    };

    template<> struct GLTypeTraits<GLubyte>
    {
        static float scale() { return 1.0f/255.0f; }
    };

    template<> struct GLTypeTraits<GLshort>
    {
        static float scale() { return 1.0f/32768.0f; } // XXX
    };

    template<> struct GLTypeTraits<GLushort>
    {
        static float scale() { return 1.0f/65535.0f; }
    };

    template<> struct GLTypeTraits<GLint>
    {
        static float scale() { return 1.0f/2147483648.0f; } // XXX
    };

    template<> struct GLTypeTraits<GLuint>
    {
        static float scale() { return 1.0f/4294967295.0f; }
    };

    template<> struct GLTypeTraits<GLfloat>
    {
        static float scale() { return 1.0f; }
    };

    // The Reader function that performs the read.
    template<int Format, typename T> struct ColorReader;
    template<int Format, typename T> struct ColorWriter;

    template<typename T>
    struct ColorReader<GL_DEPTH_COMPONENT, T>
    {
        static osg::Vec4 read(const ImageUtils::PixelReader* ia, int s, int t, int r)
        {
            const T* ptr = (const T*)ia->data(s, t, r);
            float l = float(*ptr) * GLTypeTraits<T>::scale();
            return osg::Vec4(l, l, l, 1.0f);
        }
    };

    template<typename T>
    struct ColorWriter<GL_DEPTH_COMPONENT, T>
    {
        static void write(const ImageUtils::PixelWriter* iw, int s, int t, int r, const osg::Vec4f& c )
        {
            T* ptr = (T*)iw->data(s, t, r);
            (*ptr) = (GLubyte)(c.r() / GLTypeTraits<T>::scale());
        }
    };

    template<typename T>
    struct ColorReader<GL_LUMINANCE, T>
    {
        static osg::Vec4 read(const ImageUtils::PixelReader* ia, int s, int t, int r)
        {
            const T* ptr = (const T*)ia->data(s, t, r);
            float l = float(*ptr) * GLTypeTraits<T>::scale();
            return osg::Vec4(l, l, l, 1.0f);
        }
    };

    template<typename T>
    struct ColorWriter<GL_LUMINANCE, T>
    {
        static void write(const ImageUtils::PixelWriter* iw, int s, int t, int r, const osg::Vec4f& c )
        {
            T* ptr = (T*)iw->data(s, t, r);
            (*ptr) = (GLubyte)(c.r() / GLTypeTraits<T>::scale());
        }
    };

    template<typename T>
    struct ColorReader<GL_ALPHA, T>
    {
        static osg::Vec4 read(const ImageUtils::PixelReader* ia, int s, int t, int r)
        {
            const T* ptr = (const T*)ia->data(s, t, r);
            float a = float(*ptr) * GLTypeTraits<T>::scale();
            return osg::Vec4(1.0f, 1.0f, 1.0f, a);
        }
    };

    template<typename T>
    struct ColorWriter<GL_ALPHA, T>
    {
        static void write(const ImageUtils::PixelWriter* iw, int s, int t, int r, const osg::Vec4f& c )
        {
            T* ptr = (T*)iw->data(s, t, r);
            (*ptr) = (GLubyte)(c.a() / GLTypeTraits<T>::scale());
        }
    };

    template<typename T>
    struct ColorReader<GL_LUMINANCE_ALPHA, T>
    {
        static osg::Vec4 read(const ImageUtils::PixelReader* ia, int s, int t, int r)
        {
            const T* ptr = (const T*)ia->data(s, t, r);
            float l = float(*ptr++) * GLTypeTraits<T>::scale();
            float a = float(*ptr) * GLTypeTraits<T>::scale();
            return osg::Vec4(l, l, l, a);
        }
    };

    template<typename T>
    struct ColorWriter<GL_LUMINANCE_ALPHA, T>
    {
        static void write(const ImageUtils::PixelWriter* iw, int s, int t, int r, const osg::Vec4f& c )
        {
            T* ptr = (T*)iw->data(s, t, r);
            *ptr++ = (GLubyte)( c.r() / GLTypeTraits<T>::scale() );
            *ptr   = (GLubyte)( c.a() / GLTypeTraits<T>::scale() );
        }
    };

    template<typename T>
    struct ColorReader<GL_RGB, T>
    {
        static osg::Vec4 read(const ImageUtils::PixelReader* ia, int s, int t, int r)
        {
            const T* ptr = (const T*)ia->data(s, t, r);
            float d = float(*ptr++) * GLTypeTraits<T>::scale();
            float g = float(*ptr++) * GLTypeTraits<T>::scale();
            float b = float(*ptr) * GLTypeTraits<T>::scale();
            return osg::Vec4(d, g, b, 1.0f);
        }
    };

    template<typename T>
    struct ColorWriter<GL_RGB, T>
    {
        static void write(const ImageUtils::PixelWriter* iw, int s, int t, int r, const osg::Vec4f& c )
        {
            T* ptr = (T*)iw->data(s, t, r);
            *ptr++ = (GLubyte)( c.r() / GLTypeTraits<T>::scale() );
            *ptr++ = (GLubyte)( c.g() / GLTypeTraits<T>::scale() );
            *ptr++ = (GLubyte)( c.b() / GLTypeTraits<T>::scale() );
        }
    };

    template<typename T>
    struct ColorReader<GL_RGBA, T>
    {
        static osg::Vec4 read(const ImageUtils::PixelReader* ia, int s, int t, int r)
        {
            const T* ptr = (const T*)ia->data(s, t, r);
            float d = float(*ptr++) * GLTypeTraits<T>::scale();
            float g = float(*ptr++) * GLTypeTraits<T>::scale();
            float b = float(*ptr++) * GLTypeTraits<T>::scale();
            float a = float(*ptr) * GLTypeTraits<T>::scale();
            return osg::Vec4(d, g, b, a);
        }
    };

    template<typename T>
    struct ColorWriter<GL_RGBA, T>
    {
        static void write(const ImageUtils::PixelWriter* iw, int s, int t, int r, const osg::Vec4f& c )
        {
            T* ptr = (T*)iw->data(s, t, r);
            *ptr++ = (GLubyte)( c.r() / GLTypeTraits<T>::scale() );
            *ptr++ = (GLubyte)( c.g() / GLTypeTraits<T>::scale() );
            *ptr++ = (GLubyte)( c.b() / GLTypeTraits<T>::scale() );
            *ptr++ = (GLubyte)( c.a() / GLTypeTraits<T>::scale() );
        }
    };

    template<typename T>
    struct ColorReader<GL_BGR, T>
    {
        static osg::Vec4 read(const ImageUtils::PixelReader* ia, int s, int t, int r)
        {
            const T* ptr = (const T*)ia->data(s, t, r);
            float b = float(*ptr) * GLTypeTraits<T>::scale();
            float g = float(*ptr++) * GLTypeTraits<T>::scale();
            float d = float(*ptr++) * GLTypeTraits<T>::scale();
            return osg::Vec4(d, g, b, 1.0f);
        }
    };

    template<typename T>
    struct ColorWriter<GL_BGR, T>
    {
        static void write(const ImageUtils::PixelWriter* iw, int s, int t, int r, const osg::Vec4f& c )
        {
            T* ptr = (T*)iw->data(s, t, r);
            *ptr++ = (GLubyte)( c.b() / GLTypeTraits<T>::scale() );
            *ptr++ = (GLubyte)( c.g() / GLTypeTraits<T>::scale() );
            *ptr++ = (GLubyte)( c.r() / GLTypeTraits<T>::scale() );
        }
    };

    template<typename T>
    struct ColorReader<GL_BGRA, T>
    {
        static osg::Vec4 read(const ImageUtils::PixelReader* ia, int s, int t, int r)
        {
            const T* ptr = (const T*)ia->data(s, t, r);
            float b = float(*ptr++) * GLTypeTraits<T>::scale();
            float g = float(*ptr++) * GLTypeTraits<T>::scale();
            float d = float(*ptr++) * GLTypeTraits<T>::scale();
            float a = float(*ptr) * GLTypeTraits<T>::scale();
            return osg::Vec4(d, g, b, a);
        }
    };

    template<typename T>
    struct ColorWriter<GL_BGRA, T>
    {
        static void write(const ImageUtils::PixelWriter* iw, int s, int t, int r, const osg::Vec4f& c )
        {
            T* ptr = (T*)iw->data(s, t, r);
            *ptr++ = (GLubyte)( c.b() / GLTypeTraits<T>::scale() );
            *ptr++ = (GLubyte)( c.g() / GLTypeTraits<T>::scale() );
            *ptr++ = (GLubyte)( c.r() / GLTypeTraits<T>::scale() );
            *ptr++ = (GLubyte)( c.a() / GLTypeTraits<T>::scale() );
        }
    };

    template<typename T>
    struct ColorReader<0, T>
    {
        static osg::Vec4 read(const ImageUtils::PixelReader* ia, int s, int t, int r)
        {
            return osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f);
        }
    };

    template<typename T>
    struct ColorWriter<0, T>
    {
        static void write(const ImageUtils::PixelWriter* iw, int s, int t, int r, const osg::Vec4f& c )
        {
            //nop
        }
    };

    template<>
    struct ColorReader<GL_UNSIGNED_SHORT_5_5_5_1, GLushort>
    {
        static osg::Vec4 read(const ImageUtils::PixelReader* ia, int s, int t, int r)
        {
            GLushort p = *(const GLushort*)ia->data( s, t, r );
            //internal format GL_RGB5_A1 is implied
            return osg::Vec4( r5*(float)(p>>11), r5*(float)((p&0x7c0)>>6), r5*((p&0x3e)>>1), (float)(p&0x1));
        }
    };

    template<>
    struct ColorWriter<GL_UNSIGNED_SHORT_5_5_5_1, GLushort>
    {
        static void write(const ImageUtils::PixelWriter* iw, int s, int t, int r, const osg::Vec4f& c )
        {
            GLushort
                red = (unsigned short)(c.r()*255),
                g = (unsigned short)(c.g()*255),
                b = (unsigned short)(c.b()*255),
                a = c.a() < 0.15 ? 0 : 1;

            GLushort* ptr = (GLushort*)iw->data(s, t, r);
            *ptr = (((red) & (0xf8)) << 8) | (((g) & (0xf8)) << 3) | (((b) & (0xF8)) >> 2) | a;
        }
    };

    template<>
    struct ColorReader<GL_UNSIGNED_BYTE_3_3_2, GLubyte>
    {
        static osg::Vec4 read(const ImageUtils::PixelReader* ia, int s, int t, int r)
        {
              GLubyte p = *(const GLubyte*)ia->data( s, t, r );
            // internal format GL_R3_G3_B2 is implied
            return osg::Vec4( r3*(float)(p>>5), r3*(float)((p&0x28)>>2), r2*(float)(p&0x3), 1.0f );
        }
    };

    template<>
    struct ColorWriter<GL_UNSIGNED_BYTE_3_3_2, GLubyte>
    {
        static void write(const ImageUtils::PixelWriter* iw, int s, int t, int r, const osg::Vec4f& c )
        {
            GLubyte* ptr = (GLubyte*)iw->data(s, t, r);
            OE_WARN << LC << "Target GL_UNSIGNED_BYTE_3_3_2 not yet implemented" << std::endl;
        }
    };
}

template<int GLFormat>
inline ImageUtils::PixelReader::ReaderFunc chooseReader(GLenum dataType)
{
    switch (dataType)
    {
    case GL_BYTE:
        return &ColorReader<GLFormat, GLbyte>::read;
    case GL_UNSIGNED_BYTE:
        return &ColorReader<GLFormat, GLubyte>::read;
    case GL_SHORT:
        return &ColorReader<GLFormat, GLshort>::read;
    case GL_UNSIGNED_SHORT:
        return &ColorReader<GLFormat, GLushort>::read;
    case GL_INT:
        return &ColorReader<GLFormat, GLint>::read;
    case GL_UNSIGNED_INT:
        return &ColorReader<GLFormat, GLuint>::read;
    case GL_FLOAT:
        return &ColorReader<GLFormat, GLfloat>::read;       
    case GL_UNSIGNED_SHORT_5_5_5_1:
        return &ColorReader<GL_UNSIGNED_SHORT_5_5_5_1, GLushort>::read;
    case GL_UNSIGNED_BYTE_3_3_2:
        return &ColorReader<GL_UNSIGNED_BYTE_3_3_2, GLubyte>::read;
    default:
        return &ColorReader<0, GLbyte>::read;
    }
}
    
ImageUtils::PixelReader::PixelReader(const osg::Image* image) :
_image(image)
{
    _colMult = _image->getPixelSizeInBits() / 8;
    _rowMult = _image->getRowSizeInBytes();
    _imageSize = _image->getImageSizeInBytes();
    GLenum dataType = _image->getDataType();
    switch(image->getPixelFormat())
    {
    case GL_DEPTH_COMPONENT:
        _reader = chooseReader<GL_DEPTH_COMPONENT>(dataType);
        break;
    case GL_LUMINANCE:
        _reader = chooseReader<GL_LUMINANCE>(dataType);
        break;        
    case GL_ALPHA:
        _reader = chooseReader<GL_ALPHA>(dataType);
        break;        
    case GL_LUMINANCE_ALPHA:
        _reader = chooseReader<GL_LUMINANCE_ALPHA>(dataType);
        break;        
    case GL_RGB:
        _reader = chooseReader<GL_RGB>(dataType);
        break;        
    case GL_RGBA:
        _reader = chooseReader<GL_RGBA>(dataType);
        break;        
    case GL_BGR:
        _reader = chooseReader<GL_BGR>(dataType);
        break;        
    case GL_BGRA:
        _reader = chooseReader<GL_BGRA>(dataType);
        break; 
    default:
        _reader = &ColorReader<0, GLbyte>::read;
        break;
    }
}


template<int GLFormat>
inline ImageUtils::PixelWriter::WriterFunc chooseWriter(GLenum dataType)
{
    switch (dataType)
    {
    case GL_BYTE:
        return &ColorWriter<GLFormat, GLbyte>::write;
    case GL_UNSIGNED_BYTE:
        return &ColorWriter<GLFormat, GLubyte>::write;
    case GL_SHORT:
        return &ColorWriter<GLFormat, GLshort>::write;
    case GL_UNSIGNED_SHORT:
        return &ColorWriter<GLFormat, GLushort>::write;
    case GL_INT:
        return &ColorWriter<GLFormat, GLint>::write;
    case GL_UNSIGNED_INT:
        return &ColorWriter<GLFormat, GLuint>::write;
    case GL_FLOAT:
        return &ColorWriter<GLFormat, GLfloat>::write;       
    case GL_UNSIGNED_SHORT_5_5_5_1:
        return &ColorWriter<GL_UNSIGNED_SHORT_5_5_5_1, GLushort>::write;
    case GL_UNSIGNED_BYTE_3_3_2:
        return &ColorWriter<GL_UNSIGNED_BYTE_3_3_2, GLubyte>::write;
    default:
        return &ColorWriter<0, GLbyte>::write;
    }
}
    
ImageUtils::PixelWriter::PixelWriter(osg::Image* image) :
_image(image)
{
    _colMult = _image->getPixelSizeInBits() / 8;
    _rowMult = _image->getRowSizeInBytes();
    _imageSize = _image->getImageSizeInBytes();
    GLenum dataType = _image->getDataType();

    switch(image->getPixelFormat())
    {
    case GL_DEPTH_COMPONENT:
        _writer = chooseWriter<GL_DEPTH_COMPONENT>(dataType);
        break;
    case GL_LUMINANCE:
        _writer = chooseWriter<GL_LUMINANCE>(dataType);
        break;        
    case GL_ALPHA:
        _writer = chooseWriter<GL_ALPHA>(dataType);
        break;        
    case GL_LUMINANCE_ALPHA:
        _writer = chooseWriter<GL_LUMINANCE_ALPHA>(dataType);
        break;        
    case GL_RGB:
        _writer = chooseWriter<GL_RGB>(dataType);
        break;        
    case GL_RGBA:
        _writer = chooseWriter<GL_RGBA>(dataType);
        break;        
    case GL_BGR:
        _writer = chooseWriter<GL_BGR>(dataType);
        break;        
    case GL_BGRA:
        _writer = chooseWriter<GL_BGRA>(dataType);
        break; 
    default:
        _writer = chooseWriter<0>(dataType);
        break;
    }
}
