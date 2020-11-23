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

#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/Metrics>

#include <osg/GLU>
#include <osgDB/Registry>

#include <osg/ValueObject>

#define LC "[ImageUtils] "


#if defined(OSG_GLES1_AVAILABLE) || defined(OSG_GLES2_AVAILABLE) || defined(OSG_GLES3_AVAILABLE)
#    define GL_RGB8_INTERNAL  GL_RGB8_OES
#    define GL_RGB8A_INTERNAL GL_RGBA8_OES
#else
#    define GL_RGB8_INTERNAL  GL_RGB8
#    define GL_RGB8A_INTERNAL GL_RGBA8
#endif


using namespace osgEarth;
using namespace osgEarth::Util;


osg::Image*
ImageUtils::cloneImage( const osg::Image* input )
{
    // Why not just call image->clone()? Because, the osg::Image copy constructor does not
    // clear out the underlying BufferData/BufferObject's GL handles. This can cause
    // exepected results if you are cloning an image that has already been used in GL.
    // Calling clone->dirty() might work, but we are not sure.

    if ( !input ) return 0L;

    osg::Image* clone = osg::clone( input, osg::CopyOp::DEEP_COPY_ALL );
    clone->dirty();
    return clone;
}

bool
ImageUtils::copyAsSubImage(const osg::Image* src, osg::Image* dst, int dst_start_col, int dst_start_row)
{
    if (!src || !dst ||
        dst_start_col + src->s() > dst->s() ||
        dst_start_row + src->t() > dst->t() ||
        src->r() != dst->r())
    {
        return false;
    }

    // check for fast bytewise copy:
    if (src->getPacking() == dst->getPacking() &&
        src->getDataType() == dst->getDataType() &&
        src->getPixelFormat() == dst->getPixelFormat() )
    {
        for(int r=0; r<src->r(); ++r) // each layer
        {
            for( int src_row=0, dst_row=dst_start_row; src_row < src->t(); src_row++, dst_row++ )
            {
                const void* src_data = src->data( 0, src_row, r );
                void* dst_data = dst->data( dst_start_col, dst_row, r );
                memcpy( dst_data, src_data, src->getRowSizeInBytes() );
            }
        }
    }

    // otherwise loop through an convert pixel-by-pixel.
    else
    {
        if ( !PixelReader::supports(src) || !PixelWriter::supports(dst) )
            return false;

        PixelReader read(src);
        PixelWriter write(dst);

        for( int r=0; r<src->r(); ++r)
        {
            for( int src_t=0, dst_t=dst_start_row; src_t < src->t(); src_t++, dst_t++ )
            {
                for( int src_s=0, dst_s=dst_start_col; src_s < src->s(); src_s++, dst_s++ )
                {
                    write(read(src_s, src_t, r), dst_s, dst_t, r);
                }
            }
        }
    }

    return true;
}

osg::Image*
ImageUtils::createBumpMap(const osg::Image* input)
{
    if ( !PixelReader::supports(input) || !PixelWriter::supports(input) )
        return 0L;

    osg::Image* output = osg::clone(input, osg::CopyOp::DEEP_COPY_ALL);

    static const float kernel[] = {
        -1.0, -1.0, 0.0,
        -1.0,  0.0, 1.0,
         0.0,  1.0, 1.0
    };

    PixelReader read(input);
    PixelWriter write(output);

    osg::Vec4f mid(0.5f,0.5f,0.5f,0.5f);

    for( int t=0; t<input->t(); ++t )
    {
        for( int s=0; s<input->s(); ++s )
        {
            if ( t == 0 || t == input->t()-1 || s == 0 || s == input->s()-1 )
            {
                write( mid, s, t );
            }
            else
            {
                osg::Vec4f sum;

                // run the emboss kernel:
                for( int tt=0; tt<=2; ++tt )
                    for( int ss=0; ss<=2; ++ss )
                        sum += read(s+ss-1,t+tt-1) * kernel[tt*3+ss];
                sum /= 9.0f;

                // bias for bumpmapping:
                sum += osg::Vec4f(0.5f,0.5f,0.5f,0.5f);

                // convert to greyscale:
                sum.r() *= 0.2989f;
                sum.g() *= 0.5870f;
                sum.b() *= 0.1140f;

                sum.a() = read(s,t).a();
                write( sum, s, t );
            }
        }
    }
    return output;
}

bool
ImageUtils::resizeImage(const osg::Image* input,
                        unsigned int out_s, unsigned int out_t,
                        osg::ref_ptr<osg::Image>& output,
                        unsigned int mipmapLevel,
                        bool bilinear)
{
    if ( !input && out_s == 0 && out_t == 0 )
        return false;

    if ( !PixelReader::supports(input) )
    {
        OE_WARN << LC << "resizeImage: unsupported format" << std::endl;
        return false;
    }

    if ( output.valid() && !PixelWriter::supports(output.get()) )
    {
        OE_WARN << LC << "resizeImage: pre-allocated output image is in an unsupported format" << std::endl;
        return false;
    }

    unsigned int in_s = input->s();
    unsigned int in_t = input->t();

    if ( !output.valid() )
    {
        output = new osg::Image();

        if ( PixelWriter::supports(input) )
        {
            output->allocateImage( out_s, out_t, input->r(), input->getPixelFormat(), input->getDataType(), input->getPacking() );
            output->setInternalTextureFormat( input->getInternalTextureFormat() );
        }
        else
        {
            // for unsupported write formats, convert to normalized RGBA8 automatically.
            output->allocateImage( out_s, out_t, input->r(), GL_RGBA, GL_UNSIGNED_BYTE );
            output->setInternalTextureFormat( GL_RGB8A_INTERNAL );
        }
    }
    else
    {
        // make sure they match up
        output->setInternalTextureFormat( input->getInternalTextureFormat() );
    }

    if ( in_s == out_s && in_t == out_t && mipmapLevel == 0 && input->getInternalTextureFormat() == output->getInternalTextureFormat() )
    {
        memcpy( output->data(), input->data(), input->getTotalSizeInBytes() );
    }
    else
    {
        PixelReader read( input );
        PixelWriter write( output.get() );

        osg::Vec4 color;

        for( unsigned int output_row=0; output_row < out_t; output_row++ )
        {
            // get an appropriate input row
            float output_row_ratio = (float)output_row/(float)out_t;
            float input_row = output_row_ratio * (float)in_t;
            if ( input_row >= input->t() ) input_row = in_t-1;
            else if ( input_row < 0 ) input_row = 0;

            for( unsigned int output_col = 0; output_col < out_s; output_col++ )
            {
                float output_col_ratio = (float)output_col/(float)out_s;
                float input_col =  output_col_ratio * (float)in_s;
                if ( input_col >= (int)in_s ) input_col = in_s-1;
                else if ( input_col < 0 ) input_col = 0.0f;

                for(int layer=0; layer<input->r(); ++layer)
                {
                    if (bilinear)
                    {
                        // Do a bilinear interpolation for the image
                        int rowMin = osg::maximum((int)floor(input_row), 0);
                        int rowMax = osg::maximum(osg::minimum((int)ceil(input_row), (int)(input->t()-1)), 0);
                        int colMin = osg::maximum((int)floor(input_col), 0);
                        int colMax = osg::maximum(osg::minimum((int)ceil(input_col), (int)(input->s()-1)), 0);

                        if (rowMin > rowMax) rowMin = rowMax;
                        if (colMin > colMax) colMin = colMax;

                        osg::Vec4 urColor = read(colMax, rowMax, layer);
                        osg::Vec4 llColor = read(colMin, rowMin, layer);
                        osg::Vec4 ulColor = read(colMin, rowMax, layer);
                        osg::Vec4 lrColor = read(colMax, rowMin, layer);

                        if ((colMax == colMin) && (rowMax == rowMin))
                        {
                            // Exact value
                            color = urColor;
                        }
                        else if (colMax == colMin)
                        {
                            // Linear interpolate vertically
                            color = llColor * ((double)rowMax - input_row) + ulColor * (input_row - (double)rowMin);
                        }
                        else if (rowMax == rowMin)
                        {
                            // Linear interpolate horizontally
                            color = llColor * ((double)colMax - input_col) + lrColor * (input_col - (double)colMin);
                        }
                        else
                        {
                            // Bilinear interpolate
                            osg::Vec4 r1 = llColor * ((double)colMax - input_col) + lrColor * (input_col - (double)colMin);
                            osg::Vec4 r2 = ulColor * ((double)colMax - input_col) + urColor * (input_col - (double)colMin);
                            color = r1 * ((double)rowMax - input_row) + r2 * (input_row - (double)rowMin);
                        }
                    }
                    else
                    {
                        // nearest neighbor:
                        int col = (input_col-(int)input_col) <= (ceil(input_col)-input_col) ?
                            (int)input_col :
                            osg::minimum( 1+(int)input_col, (int)in_s-1 );

                        int row = (input_row-(int)input_row) <= (ceil(input_row)-input_row) ?
                            (int)input_row :
                            osg::minimum( 1+(int)input_row, (int)in_t-1 );

                        read(color, col, row, layer); // read pixel from mip level 0.

                        // old code
                        //color = read( (int)input_col, (int)input_row, layer ); // read pixel from mip level 0
                    }

                    write( color, output_col, output_row, layer, mipmapLevel ); // write to target mip level
                }
            }
        }
    }

    return true;
}

bool
ImageUtils::flattenImage(const osg::Image* input,
                         std::vector<osg::ref_ptr<osg::Image> >& output)
{
    if (input == 0L || input->r() < 2)
        return false;

    for(int r=0; r<input->r(); ++r)
    {
        osg::Image* layer = new osg::Image();
        layer->allocateImage(input->s(), input->t(), 1, input->getPixelFormat(), input->getDataType(), input->getPacking());
        layer->setPixelAspectRatio(input->getPixelAspectRatio());

        layer->setRowLength(input->getRowLength());
        layer->setOrigin(input->getOrigin());
        layer->setFileName(input->getFileName());
        layer->setWriteHint(input->getWriteHint());
        layer->setInternalTextureFormat(input->getInternalTextureFormat());
        ::memcpy(layer->data(), input->data(0,0,r), layer->getTotalSizeInBytes());
        output.push_back(layer);
    }

    return true;
}

bool
ImageUtils::bicubicUpsample(const osg::Image* source,
                            osg::Image* target,
                            unsigned quadrant,
                            unsigned stride)
{
    const int border = 1; // don't change this.

    int width = ((source->s() - 2*border)/2)+1 + 2*border;
    int height = ((source->t() - 2*border)/2)+1 + 2*border;

    int s_off = quadrant == 0 || quadrant == 2 ? 0 : source->s()-width;
    int t_off = quadrant == 2 || quadrant == 3 ? 0 : source->t()-height;

    ImageUtils::PixelReader readSource(source);
    ImageUtils::PixelWriter writeTarget(target);
    ImageUtils::PixelReader readTarget(target);

    // copy the main box, which is all odd-numbered cells when there is a border size = 1.
    for (int t = 1; t<height-1; ++t)
    {
        for (int s = 1; s<width-1; ++s)
        {
            osg::Vec4 value = readSource(s_off+s, t_off+t);
            writeTarget(value, (s-1)*2+1, (t-1)*2+1);
        }
    }

    // copy the corner border cells.
    writeTarget(readSource(s_off, t_off), 0, 0); // upper left.
    writeTarget(readSource(s_off + width - 1, t_off), target->s()-1, 0);
    writeTarget(readSource(s_off, t_off + height - 1), 0, target->t()-1);
    writeTarget(readSource(s_off + width - 1, t_off + height - 1), target->s() - 1, target->t() - 1);

    // copy the border intermediate cells.
    for (int s=1; s<width-1; ++s) // top/bottom:
    {
        writeTarget(readSource(s_off+s, t_off), (s-1)*2+1, 0);
        writeTarget(readSource(s_off+s, t_off + height - 1), (s-1)*2+1, target->t()-1);
    }
    for (int t = 1; t < height-1; ++t) // left/right:
    {
        writeTarget(readSource(s_off, t_off+t), 0, (t-1)*2+1);
        writeTarget(readSource(s_off + width - 1, t_off + t), target->s()-1, (t-1)*2+1);
    }

    // now interpolate the missing columns, including the border cells.
    for (int s = 2; s<target->s()-2; s += 2)
    {
        for (int t = 0; t < target->t(); )
        {
            int offset = (s-1) % stride; // the minus1 accounts for the border
            int s0 = osg::maximum(s - offset, 0);
            int s1 = osg::minimum(s0 + (int)stride, target->s()-1);
            double mu = (double)offset / (double)(s1-s0);
            osg::Vec4 p1 = readTarget(s0, t);
            osg::Vec4 p2 = readTarget(s1, t);
            double mu2 = (1.0 - cos(mu*osg::PI))*0.5;
            osg::Vec4 v = (p1*(1.0-mu2)) + (p2*mu2);
            writeTarget(v, s, t);

            if (t == 0 || t == target->t()-2) t+=1; else t+=2;
        }
    }

    // next interpolate the odd numbered rows
    for (int s = 0; s < target->s();)
    {
        for (int t = 2; t<target->t()-2; t += 2)
        {
            int offset = (t-1) % stride; // the minus1 accounts for the border
            int t0 = osg::maximum(t - offset, 0);
            int t1 = osg::minimum(t0 + (int)stride, target->t()-1);
            double mu = (double)offset / double(t1-t0);

            osg::Vec4 p1 = readTarget(s, t0);
            osg::Vec4 p2 = readTarget(s, t1);
            double mu2 = (1.0 - cos(mu*osg::PI))*0.5;
            osg::Vec4 v = (p1*(1.0-mu2)) + (p2*mu2);
            writeTarget(v, s, t);
        }

        if (s == 0 || s == target->s()-2) s+=1; else s+=2;
    }

    // then interpolate the centers
    for (int s = 2; s<target->s()-2; s += 2)
    {
        for (int t = 2; t<target->t()-2; t += 2)
        {
            int s_offset = (s-1) % stride;
            int s0 = osg::maximum(s - s_offset, 0);
            int s1 = osg::minimum(s0 + (int)stride, target->s()-1);

            int t_offset = (t-1) % stride;
            int t0 = osg::maximum(t - t_offset, 0);
            int t1 = osg::minimum(t0 + (int)stride, target->t()-1);

            double mu, mu2;

            osg::Vec4 p1 = readTarget(s0, t);
            osg::Vec4 p2 = readTarget(s1, t);
            mu = (double)s_offset / (double)(s1-s0);
            mu2 = (1.0 - cos(mu*osg::PI))*0.5;
            osg::Vec4 v1 = (p1*(1.0-mu2)) + (p2*mu2);

            osg::Vec4 p3 = readTarget(s, t0);
            osg::Vec4 p4 = readTarget(s, t1);
            mu = (double)t_offset / (double)(t1-t0);
            mu2 = (1.0 - cos(mu*osg::PI))*0.5;
            osg::Vec4 v2 = (p3*(1.0-mu2)) + (p4*mu2);

            osg::Vec4 v = (v1+v2)*0.5;

            writeTarget(v, s, t);
        }
    }

    return true;
}

const osg::Image*
ImageUtils::mipmapImage(const osg::Image* input)
{
    OE_PROFILING_ZONE;

    if (!input)
    {
        OE_WARN << LC << "createMipmappedImage() called with NULL input" << std::endl;
        return input;
    }

    if (input->r() > 1)
    {
        OE_WARN << LC << "createMipmappedImage() not implemented for 3D image" << std::endl;
        return input;
    }

    // already has mipmaps?
    if (input->getNumMipmapLevels() > 1)
    {
        return input;
    }

    // compressed? this algorithm won't work
    if (input->isCompressed())
    {
        return input;
    }

    // first, build the image that will hold all the mipmap levels.
    int numLevels = osg::Image::computeNumberOfMipmapLevels(input->s(), input->t(), input->r());
    int imageSizeBytes = input->getTotalSizeInBytes();

    // offset vector does not include level 0 (the full-resolution level)
    osg::Image::MipmapDataType mipOffsets;
    mipOffsets.reserve(numLevels-1);

    // calculate memory requirements:
    int totalSizeBytes = imageSizeBytes;
    for( int i=1; i<numLevels; ++i )
    {
        mipOffsets.push_back(totalSizeBytes);
        totalSizeBytes += (imageSizeBytes >> i);
    }

    osg::Image* output = new osg::Image();
    output->setName(input->getName());

    // allocate space for the new data and copy over level 0 of the old data
    unsigned char* newData = new unsigned char[totalSizeBytes];
    ::memcpy(newData, input->data(), imageSizeBytes);

    output->setImage(
        input->s(), input->t(), input->r(),
        input->getInternalTextureFormat(),
        input->getPixelFormat(),
        input->getDataType(),
        newData,
        osg::Image::USE_NEW_DELETE,
        input->getPacking(),
        input->getRowLength());

    output->setMipmapLevels(mipOffsets);

    // now, populate the image levels.
    osg::PixelStorageModes psm;
    psm.pack_alignment = input->getPacking();
    psm.pack_row_length = input->getRowLength();
    psm.unpack_alignment = input->getPacking();

    for(int level=1; level<numLevels; ++level)
    {
        // OSG-custom gluScaleImage that does not require a graphics context
        GLint status = gluScaleImage(
            &psm,
            output->getPixelFormat(),
            output->s(),
            output->t(),
            output->getDataType(),
            output->data(),
            output->s() >> level,
            output->t() >> level,
            output->getDataType(),
            output->getMipmapData(level));
    }

    return output;
}

void
ImageUtils::mipmapImageInPlace(osg::Image* input)
{
    OE_PROFILING_ZONE;

    if (!input)
    {
        OE_WARN << LC << "createMipmappedImage() called with NULL input" << std::endl;
        return;
    }

    if (input->r() > 1)
    {
        OE_WARN << LC << "createMipmappedImage() not implemented for 3D image" << std::endl;
        return;
    }

    // already has mipmaps?
    if (input->getNumMipmapLevels() > 1)
    {
        return;
    }

    // compressed? this algorithm won't work
    if (input->isCompressed())
    {
        return;
    }

    // first, build the image that will hold all the mipmap levels.
    int numLevels = osg::Image::computeNumberOfMipmapLevels(input->s(), input->t(), input->r());
    int imageSizeBytes = input->getTotalSizeInBytes();

    // offset vector does not include level 0 (the full-resolution level)
    osg::Image::MipmapDataType mipOffsets;
    mipOffsets.reserve(numLevels-1);

    // calculate memory requirements:
    int totalSizeBytes = imageSizeBytes;
    for( int i=1; i<numLevels; ++i )
    {
        mipOffsets.push_back(totalSizeBytes);
        totalSizeBytes += (imageSizeBytes >> i);
    }

    // allocate space for the new data and copy over level 0 of the old data
    unsigned char* newData = new unsigned char[totalSizeBytes];
    ::memcpy(newData, input->data(), input->getTotalSizeInBytes());

    input->setImage(
        input->s(), input->t(), input->r(),
        input->getInternalTextureFormat(),
        input->getPixelFormat(),
        input->getDataType(),
        newData,
        osg::Image::USE_NEW_DELETE,
        input->getPacking(),
        input->getRowLength());

    input->setMipmapLevels(mipOffsets);

    // now, populate the image levels.
    osg::PixelStorageModes psm;
    psm.pack_alignment = input->getPacking();
    psm.pack_row_length = input->getRowLength();
    psm.unpack_alignment = input->getPacking();

    for(int level=1; level<numLevels; ++level)
    {
        // OSG-custom gluScaleImage that does not require a graphics context
        GLint status = gluScaleImage(
            &psm,
            input->getPixelFormat(),
            input->s(),
            input->t(),
            input->getDataType(),
            input->data(),
            input->s() >> level,
            input->t() >> level,
            input->getDataType(),
            input->getMipmapData(level));
    }
}

const osg::Image*
ImageUtils::compressImage(
    const osg::Image* input,
    const std::string& method)
{
    OE_PROFILING_ZONE;

    if (!input)
        return input;

    if (input->isCompressed())
        return input;

    if (method.empty() || method == "none")
        return input;

    if (method == "gpu")
        return input;

    // return the input if nothing works
    osg::Image* output = const_cast<osg::Image*>(input);

    osgDB::ImageProcessor* ip = nullptr;

    std::string driver(method);

    if (driver == "cpu" ||
        driver == "auto" ||
        (driver.length() >=3 && driver.substr(0,3)=="dxt"))
    {
        driver = "fastdxt";
    }

    ip = osgDB::Registry::instance()->getImageProcessorForExtension(driver);

    if (ip)
    {
        output = osg::clone(input, osg::CopyOp::DEEP_COPY_ALL);

        // RGB uses DXT1
        osg::Texture::InternalFormatMode mode;
        if (hasAlphaChannel(input))
            mode = osg::Texture::USE_S3TC_DXT5_COMPRESSION;
        else
            mode = osg::Texture::USE_S3TC_DXT1_COMPRESSION;

        ip->compress(
            *output,        // image to compress
            mode,           // compression mode
            true,           // generate mipmaps if possible
            true,           // resize to power of 2
            ip->USE_CPU,    // technique (always use CPU here)
            ip->FASTEST);   // quality
    }

    return output;
}

void
ImageUtils::compressImageInPlace(
    osg::Image* input,
    const std::string& method)
{
    OE_PROFILING_ZONE;

    if (!input)
        return;

    if (input->isCompressed())
        return;

    if (method.empty() || method == "none")
        return;

    // RGB uses DXT1
    osg::Texture::InternalFormatMode mode;

    if (hasAlphaChannel(input))
    {
        mode = osg::Texture::USE_S3TC_DXT5_COMPRESSION;
    }
    else
    {
        mode = osg::Texture::USE_S3TC_DXT1_COMPRESSION;
    }

    if (method == "gpu")
    {
        // cheat! don't tell anyone
        input->setInternalTextureFormat(mode);
        return;
    }
    else
    {
        // return the input if nothing works
        osgDB::ImageProcessor* ip = nullptr;

        std::string driver(method);

        if (driver == "cpu" ||
            driver == "auto" ||
            (driver.length() >=3 && driver.substr(0,3)=="dxt"))
        {
            driver = "fastdxt";
        }

        ip = osgDB::Registry::instance()->getImageProcessorForExtension(driver);

        if (ip)
        {
            ip->compress(
                *input,         // image to compress
                mode,           // compression mode
                true,           // generate mipmaps if possible
                true,           // resize to power of 2
                ip->USE_CPU,    // technique (always use CPU here)
                ip->FASTEST);   // quality
        }

        else
        {
            // CPU didn't work so just use GPU. (cheating)
            input->setInternalTextureFormat(mode);
        }
    }
}

namespace
{
    struct CompressAndMipmapTextures : public TextureAndImageVisitor
    {
        void apply(osg::Texture& texture)
        {
            for (unsigned i = 0; i < texture.getNumImages(); ++i)
            {
                // Only process textures with valid images.
                osg::ref_ptr< osg::Image > image = texture.getImage(i);
                if (image.valid())
                {
                    ImageUtils::compressImageInPlace(image.get());
                    ImageUtils::mipmapImageInPlace(image.get());
                }
                else
                {
                    OE_WARN << "Skipping null image in CompressAndMipmapTextures" << std::endl;
                }
            }
        }
    };
}

void
ImageUtils::compressAndMipmapTextures(osg::Node* node)
{
    if (node)
    {
        CompressAndMipmapTextures visitor;
        node->accept(visitor);
    }
}

osgDB::ReaderWriter*
ImageUtils::getReaderWriterForStream(std::istream& stream) {
    // Modified from https://oroboro.com/image-format-magic-bytes/

    // Get the length of the stream
    stream.seekg(0, std::ios::end);
    unsigned int len = stream.tellg();
    stream.seekg(0, std::ios::beg);

    if (len < 16) return 0;

    //const char* data = input.c_str();
    // Read a 16 byte header
    char data[16];
    stream.read(data, 16);
    // Reset reading
    stream.seekg(0, std::ios::beg);

    // .jpg:  FF D8 FF
    // .png:  89 50 4E 47 0D 0A 1A 0A
    // .gif:  GIF87a
    //        GIF89a
    // .tiff: 49 49 2A 00
    //        4D 4D 00 2A
    // .bmp:  BM
    // .webp: RIFF ???? WEBP
    // .ico   00 00 01 00
    //        00 00 02 00 ( cursor files )
    switch (data[0])
    {
    case '\xFF':
        return (!strncmp((const char*)data, "\xFF\xD8\xFF", 3)) ?
            osgDB::Registry::instance()->getReaderWriterForExtension("jpg") : 0;

    case '\x89':
        return (!strncmp((const char*)data,
            "\x89\x50\x4E\x47\x0D\x0A\x1A\x0A", 8)) ?
            osgDB::Registry::instance()->getReaderWriterForExtension("png") : 0;

    case 'G':
        return (!strncmp((const char*)data, "GIF87a", 6) ||
            !strncmp((const char*)data, "GIF89a", 6)) ?
            osgDB::Registry::instance()->getReaderWriterForExtension("gif") : 0;

    case 'I':
        return (!strncmp((const char*)data, "\x49\x49\x2A\x00", 4)) ?
            osgDB::Registry::instance()->getReaderWriterForExtension("tif") : 0;

    case 'M':
        return (!strncmp((const char*)data, "\x4D\x4D\x00\x2A", 4)) ?
            osgDB::Registry::instance()->getReaderWriterForExtension("tif") : 0;

    case 'B':
        return ((data[1] == 'M')) ?
            osgDB::Registry::instance()->getReaderWriterForExtension("bmp") : 0;

    case 'R':
        return (!strncmp((const char*)data, "RIFF", 4)) ?
            osgDB::Registry::instance()->getReaderWriterForExtension("webp") : 0;


    default:
        return 0;
    }
}

osg::Image*
ImageUtils::readStream(std::istream& stream, const osgDB::Options* options) {

    osgDB::ReaderWriter* rw = getReaderWriterForStream(stream);
    if (!rw) {
        return 0;
    }

    osgDB::ReaderWriter::ReadResult rr = rw->readImage(stream, options);
    if (rr.validImage()) {
        return rr.takeImage();
    }
    return 0;
}

osg::Texture2DArray*
ImageUtils::makeTexture2DArray(osg::Image* image)
{
    std::vector< osg::ref_ptr<osg::Image> > images;
    if (image->r() > 1)
    {
        ImageUtils::flattenImage(image, images);
    }
    else
    {
        images.push_back(image);
    }
    osg::Texture2DArray* tex2dArray = new osg::Texture2DArray();

    tex2dArray->setTextureDepth(images.size());
    tex2dArray->setInternalFormat(images[0]->getInternalTextureFormat());
    tex2dArray->setSourceFormat(images[0]->getPixelFormat());
    for (int i = 0; i < (int)images.size(); ++i)
    {
        tex2dArray->setImage(i, const_cast<osg::Image*>(images[i].get()));
    }
    return tex2dArray;
}

namespace
{
    struct MixImage
    {
        float _a;
        bool _srcHasAlpha, _destHasAlpha;

        bool operator()( const osg::Vec4f& src, osg::Vec4f& dest )
        {
            float sa = _srcHasAlpha ? _a * src.a() : _a;
            float da = _destHasAlpha ? dest.a() : 1.0f;
            dest.set(
                dest.r()*(1.0f-sa) + src.r()*sa,
                dest.g()*(1.0f-sa) + src.g()*sa,
                dest.b()*(1.0f-sa) + src.b()*sa,
                osg::maximum(sa, da) );
            return true;
        }
    };
}

bool
ImageUtils::mix(osg::Image* dest, const osg::Image* src, float a)
{
    if (!dest || !src || dest->s() != src->s() || dest->t() != src->t() || src->r() != dest->r() ||
        !PixelReader::supports(src) ||
        !PixelWriter::supports(dest) )
    {
        return false;
    }

    PixelVisitor<MixImage> mixer;
    mixer._a = osg::clampBetween( a, 0.0f, 1.0f );
    mixer._srcHasAlpha = hasAlphaChannel(src); //src->getPixelSizeInBits() == 32;
    mixer._destHasAlpha = hasAlphaChannel(dest); //dest->getPixelSizeInBits() == 32;

    mixer.accept( src, dest );

    return true;
}

osg::Image*
ImageUtils::cropImage(const osg::Image* image,
                      double src_minx, double src_miny, double src_maxx, double src_maxy,
                      double &dst_minx, double &dst_miny, double &dst_maxx, double &dst_maxy)
{
    if ( image == 0L )
        return 0L;

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
    cropped->allocateImage(windowWidth, windowHeight, image->r(), image->getPixelFormat(), image->getDataType());
    cropped->setInternalTextureFormat( image->getInternalTextureFormat() );

    for (int layer=0; layer<image->r(); ++layer)
    {
        for (int src_row = windowY, dst_row=0; dst_row < windowHeight; src_row++, dst_row++)
        {
            if (src_row > image->t()-1) OE_NOTICE << "HeightBroke" << std::endl;
            const void* src_data = image->data(windowX, src_row, layer);
            void* dst_data = cropped->data(0, dst_row, layer);
            memcpy( dst_data, src_data, cropped->getRowSizeInBytes());
        }
    }
    return cropped;
}

osg::Image*
ImageUtils::cropImage(osg::Image* image, unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
    osg::Image* cropped = new osg::Image;
    cropped->allocateImage(width, height, image->r(), image->getPixelFormat(), image->getDataType());
    cropped->setInternalTextureFormat(image->getInternalTextureFormat());

    for (int layer = 0; layer < image->r(); ++layer)
    {
        for (int src_row = y, dst_row = 0; dst_row < height; src_row++, dst_row++)
        {
            const void* src_data = image->data(x, src_row, layer);
            void* dst_data = cropped->data(0, dst_row, layer);
            memcpy(dst_data, src_data, cropped->getRowSizeInBytes());
        }
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
ImageUtils::createSharpenedImage( const osg::Image* input )
{
    int filter[9] = { 0, -1, 0, -1, 5, -1, 0, -1, 0 };
    osg::Image* output = ImageUtils::cloneImage(input);
    for( int r=0; r<input->r(); ++r)
    {
        for( int t=1; t<input->t()-1; t++ )
        {
            for( int s=1; s<input->s()-1; s++ )
            {
                int pixels[9] = {
                    *(int*)input->data(s-1,t-1,r), *(int*)input->data(s,t-1,r), *(int*)input->data(s+1,t-1,r),
                    *(int*)input->data(s-1,t  ,r), *(int*)input->data(s,t  ,r), *(int*)input->data(s+1,t  ,r),
                    *(int*)input->data(s-1,t+1,r), *(int*)input->data(s,t+1,r), *(int*)input->data(s+1,t+1,r) };

                int shifts[4] = { 0, 8, 16, 24 };

                for( int c=0; c<4; c++ ) // components
                {
                    int mask = 0xff << shifts[c];
                    int sum = 0;
                    for( int i=0; i<9; i++ )
                    {
                        sum += ((pixels[i] & mask) >> shifts[c]) * filter[i];
                    }
                    sum = sum > 255? 255 : sum < 0? 0 : sum;
                    output->data(s,t,r)[c] = sum;
                }
            }
        }
    }
    return output;
}

namespace
{
    static Threading::Mutex         s_emptyImageMutex(OE_MUTEX_NAME);
    static osg::ref_ptr<osg::Image> s_emptyImage;
}

osg::Image*
ImageUtils::createEmptyImage()
{
    if (!s_emptyImage.valid())
    {
        Threading::ScopedMutexLock exclusive( s_emptyImageMutex );
        if (!s_emptyImage.valid())
        {
            s_emptyImage = createEmptyImage( 1, 1 );
        }
    }
    return s_emptyImage.get();
}

osg::Image*
ImageUtils::createEmptyImage(unsigned int s, unsigned int t, unsigned int r)
{
    osg::Image* empty = new osg::Image;
    empty->allocateImage(s,t, r, GL_RGBA, GL_UNSIGNED_BYTE);
    empty->setInternalTextureFormat( GL_RGB8A_INTERNAL );
    unsigned char *data = empty->data(0,0);
    memset(data, 0, 4 * s * t * r);
    return empty;
}

bool
ImageUtils::isEmptyImage(const osg::Image* image, float alphaThreshold)
{
    if ( !hasAlphaChannel(image) || !PixelReader::supports(image) )
        return false;

    PixelReader read(image);
    for(unsigned r=0; r<(unsigned)image->r(); ++r)
    {
        for(unsigned t=0; t<(unsigned)image->t(); ++t)
        {
            for(unsigned s=0; s<(unsigned)image->s(); ++s)
            {
                osg::Vec4 color = read(s, t, r);
                if ( color.a() > alphaThreshold )
                    return false;
            }
        }
    }
    return true;
}

osg::Image*
ImageUtils::createOnePixelImage(const osg::Vec4& color)
{
    osg::Image* image = new osg::Image;
    image->allocateImage(1,1,1, GL_RGBA, GL_UNSIGNED_BYTE);
    image->setInternalTextureFormat( GL_RGB8A_INTERNAL );
    PixelWriter write(image);
    write(color, 0, 0);
    return image;
}

bool
ImageUtils::isSingleColorImage(const osg::Image* image, float threshold)
{
    if ( !PixelReader::supports(image) )
        return false;

    PixelReader read(image);

    osg::Vec4 referenceColor = read(0, 0, 0);
    float refR = referenceColor.r();
    float refG = referenceColor.g();
    float refB = referenceColor.b();
    float refA = referenceColor.a();

    for(unsigned r=0; r<(unsigned)image->r(); ++r)
    {
        for(unsigned t=0; t<(unsigned)image->t(); ++t)
        {
            for(unsigned s=0; s<(unsigned)image->s(); ++s)
            {
                osg::Vec4 color = read(s, t, r);
                if (   (fabs(color.r()-refR) > threshold)
                    || (fabs(color.g()-refG) > threshold)
                    || (fabs(color.b()-refB) > threshold)
                    || (fabs(color.a()-refA) > threshold) )
                {
                    return false;
                }
            }
        }
    }
    return true;
}

bool
ImageUtils::computeTextureCompressionMode(const osg::Image*                 image,
                                          osg::Texture::InternalFormatMode& out_mode)
{
    if (!image)
        return false;

    const Capabilities& caps = Registry::capabilities();

#if !defined(OSG_GLES2_AVAILABLE) && !defined(OSG_GLES3_AVAILABLE)

    if (image->getPixelFormat() == GL_RGBA && image->getPixelSizeInBits() == 32)
    {
        if (caps.supportsTextureCompression(osg::Texture::USE_S3TC_DXT5_COMPRESSION))
        {
            out_mode = osg::Texture::USE_S3TC_DXT5_COMPRESSION;
            return true;
        }
        //todo: add ETC2
        else if (caps.supportsTextureCompression(osg::Texture::USE_ARB_COMPRESSION))
        {
            out_mode = osg::Texture::USE_ARB_COMPRESSION;
            return true;
        }
    }
    else if (image->getPixelFormat() == GL_RGB && image->getPixelSizeInBits() == 24)
    {
        if (caps.supportsTextureCompression(osg::Texture::USE_S3TC_DXT1_COMPRESSION))
        {
            out_mode = osg::Texture::USE_S3TC_DXT1_COMPRESSION;
            return true;
        }
        else if (caps.supportsTextureCompression(osg::Texture::USE_ETC_COMPRESSION))
        {
            // ETC1 is RGB only
            out_mode = osg::Texture::USE_ETC_COMPRESSION;
            return true;
        }
        else if (caps.supportsTextureCompression(osg::Texture::USE_ARB_COMPRESSION))
        {
            out_mode = osg::Texture::USE_ARB_COMPRESSION;
            return true;
        }
    }

#else // OSG_GLES2_AVAILABLE

    if (caps.supportsTextureCompression(osg::Texture::USE_PVRTC_4BPP_COMPRESSION))
    {
        out_mode = osg::Texture::USE_PVRTC_4BPP_COMPRESSION;
        return true;
    }
    else if (caps.supportsTextureCompression(osg::Texture::USE_PVRTC_2BPP_COMPRESSION))
    {
        out_mode = osg::Texture::USE_PVRTC_2BPP_COMPRESSION;
        return true;
    }
    else if (caps.supportsTextureCompression(osg::Texture::USE_ETC_COMPRESSION))
    {
        out_mode = osg::Texture::USE_ETC_COMPRESSION;
        return true;
    }

#endif

    return false;
}

bool
ImageUtils::replaceNoDataValues(osg::Image*       target,
                                const Bounds&     targetBounds,
                                const osg::Image* reference,
                                const Bounds&     referenceBounds)
{
    if (target == 0L ||
        reference == 0L ||
        !targetBounds.intersects(referenceBounds) )
    {
        return false;
    }

    float
        xscale = targetBounds.width()/referenceBounds.width(),
        yscale = targetBounds.height()/referenceBounds.height();

    float
        xbias = targetBounds.xMin() - referenceBounds.xMin(),
        ybias = targetBounds.yMin() - referenceBounds.yMin();

    PixelReader readTarget(target);
    PixelWriter writeTarget(target);
    PixelReader readReference(reference);

    for(int s=0; s<target->s(); ++s)
    {
        for(int t=0; t<target->t(); ++t)
        {
            osg::Vec4f pixel = readTarget(s, t);
            if ( pixel.r() == NO_DATA_VALUE )
            {
                float nx = (float)s / (float)(target->s()-1);
                float ny = (float)t / (float)(target->t()-1);
                osg::Vec4f refValue = readReference( xscale*nx+xbias, yscale*ny+ybias );
                writeTarget(refValue, s, t);
            }
        }
    }

    return true;
}

bool
ImageUtils::canConvert( const osg::Image* image, GLenum pixelFormat, GLenum dataType )
{
    if ( !image ) return false;
    return PixelReader::supports( image ) && PixelWriter::supports(pixelFormat, dataType);
}

osg::Image*
ImageUtils::convert(const osg::Image* image, GLenum pixelFormat, GLenum dataType)
{
    if ( !image )
        return 0L;

    // Very fast conversion if possible : clone image
    if ( image->getPixelFormat() == pixelFormat && image->getDataType() == dataType)
    {
        GLenum texFormat = image->getInternalTextureFormat();
        if (dataType != GL_UNSIGNED_BYTE
            || (pixelFormat == GL_RGB  && texFormat == GL_RGB8_INTERNAL)
            || (pixelFormat == GL_RGBA && texFormat == GL_RGB8A_INTERNAL))
        return cloneImage(image);
    }

    // Fast conversion if possible : RGB8 to RGBA8
    if ( dataType == GL_UNSIGNED_BYTE && pixelFormat == GL_RGBA && image->getDataType() == GL_UNSIGNED_BYTE && image->getPixelFormat() == GL_RGB)
    {
        // Do fast conversion
        osg::Image* result = new osg::Image();
        result->allocateImage(image->s(), image->t(), image->r(), GL_RGBA, GL_UNSIGNED_BYTE);
        result->setInternalTextureFormat(GL_RGBA8);

        const unsigned char* pSrcData = image->data();
        unsigned char* pDstData = result->data();
        int srcIndex = 0;
        int dstIndex = 0;

        // Convert all pixels except last one by reading 32bits chunks
        for (int i=0; i<image->t()*image->s()*image->r()-1; i++)
        {
            unsigned int srcValue = *((const unsigned int*) (pSrcData + srcIndex)) | 0xFF000000;
            *((unsigned int*) (pDstData + dstIndex)) = srcValue;

            srcIndex += 3;
            dstIndex += 4;
        }

        // Convert last pixel
        pDstData[dstIndex + 0] = pSrcData[srcIndex + 0];
        pDstData[dstIndex + 1] = pSrcData[srcIndex + 1];
        pDstData[dstIndex + 2] = pSrcData[srcIndex + 2];
        pDstData[dstIndex + 3] = 0xFF;

        return result;
    }

    // Test if generic conversion is possible
    if ( !canConvert(image, pixelFormat, dataType) )
        return 0L;

    // Generic conversion : use PixelVisitor
    osg::Image* result = new osg::Image();
    result->allocateImage(image->s(), image->t(), image->r(), pixelFormat, dataType);
    memset(result->data(), 0, result->getTotalSizeInBytes());

    if ( pixelFormat == GL_RGB && dataType == GL_UNSIGNED_BYTE )
        result->setInternalTextureFormat( GL_RGB8_INTERNAL );
    else if ( pixelFormat == GL_RGBA && dataType == GL_UNSIGNED_BYTE )
        result->setInternalTextureFormat( GL_RGB8A_INTERNAL );
    else
        result->setInternalTextureFormat( pixelFormat );

    PixelVisitor<CopyImage>().accept( image, result );

    return result;
}

osg::Image*
ImageUtils::convertToRGB8(const osg::Image *image)
{
    return convert( image, GL_RGB, GL_UNSIGNED_BYTE );
}

osg::Image*
ImageUtils::convertToRGBA8(const osg::Image* image)
{
    return convert( image, GL_RGBA, GL_UNSIGNED_BYTE );
}

bool
ImageUtils::areEquivalent(const osg::Image *lhs, const osg::Image *rhs)
{
    if (lhs == rhs) return true;

    if ((lhs->s() == rhs->s()) &&
        (lhs->t() == rhs->t()) &&
        (lhs->r() == rhs->r()) &&
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
        }

        return true;
    }

    return false;
}

bool
ImageUtils::hasAlphaChannel(const osg::Image* image)
{
    return image && (
        image->getPixelFormat() == GL_RGBA ||
        image->getPixelFormat() == GL_BGRA ||
        image->getPixelFormat() == GL_LUMINANCE_ALPHA ||
        image->getPixelFormat() == GL_COMPRESSED_RGBA_S3TC_DXT1_EXT ||
        image->getPixelFormat() == GL_COMPRESSED_RGBA_S3TC_DXT3_EXT ||
        image->getPixelFormat() == GL_COMPRESSED_RGBA_S3TC_DXT5_EXT ||
        image->getPixelFormat() == GL_COMPRESSED_RGBA_PVRTC_4BPPV1_IMG ||
        image->getPixelFormat() == GL_COMPRESSED_RGBA_PVRTC_2BPPV1_IMG );
}


bool
ImageUtils::hasTransparency(const osg::Image* image, float threshold)
{
    if ( !image || !hasAlphaChannel(image) || !PixelReader::supports(image) )
        return false;

    PixelReader read(image);
    for( int r=0; r<image->r(); ++r)
        for( int t=0; t<image->t(); ++t )
            for( int s=0; s<image->s(); ++s )
                if ( read(s, t, r).a() < threshold )
                    return true;

    return false;
}

#if 0
bool
ImageUtils::generateMipmaps(osg::Texture* tex)
{
    OE_PROFILING_ZONE;

    // Verify that this texture requests mipmaps:
    osg::Texture::FilterMode minFilter = tex->getFilter(tex->MIN_FILTER);

    bool mipsRequested =
        minFilter == tex->LINEAR_MIPMAP_LINEAR ||
        minFilter == tex->LINEAR_MIPMAP_NEAREST ||
        minFilter == tex->NEAREST_MIPMAP_LINEAR ||
        minFilter == tex->NEAREST_MIPMAP_NEAREST;

    bool mipsAdded = false;

    if (mipsRequested && tex->getNumImages() > 0)
    {
        for (unsigned i = 0; i < tex->getNumImages(); ++i)
        {
           //Can't use || since it short circuits when optimizations are on.
            mipsAdded |= generateMipmaps(tex->getImage(i));
        }
    }

    if (mipsAdded)
    {
        tex->setUseHardwareMipMapGeneration(false);
    }

    return mipsAdded;
}
#endif

bool
ImageUtils::convertToPremultipliedAlpha(osg::Image* image)
{
    if ( !PixelReader::supports(image) || !PixelWriter::supports(image) )
        return false;

    PixelReader read(image);
    PixelWriter write(image);
    for(int r=0; r<image->r(); ++r) {
        for(int s=0; s<image->s(); ++s) {
            for( int t=0; t<image->t(); ++t ) {
                osg::Vec4f c = read(s, t, r);
                write( osg::Vec4f(c.r()*c.a(), c.g()*c.a(), c.b()*c.a(), c.a()), s, t, r);
            }
        }
    }
    return true;
}


bool
ImageUtils::isCompressed(const osg::Image *image)
{
    //Later versions of OSG have an Image::isCompressed function but earlier versions like 2.8.3 do not.  This is a workaround so that
    //we can tell if an image is compressed on all versions of OSG.
    switch(image->getPixelFormat())
    {
        case(GL_COMPRESSED_ALPHA_ARB):
        case(GL_COMPRESSED_INTENSITY_ARB):
        case(GL_COMPRESSED_LUMINANCE_ALPHA_ARB):
        case(GL_COMPRESSED_LUMINANCE_ARB):
        case(GL_COMPRESSED_RGBA_ARB):
        case(GL_COMPRESSED_RGB_ARB):
        case(GL_COMPRESSED_RGB_S3TC_DXT1_EXT):
        case(GL_COMPRESSED_RGBA_S3TC_DXT1_EXT):
        case(GL_COMPRESSED_RGBA_S3TC_DXT3_EXT):
        case(GL_COMPRESSED_RGBA_S3TC_DXT5_EXT):
        case(GL_COMPRESSED_SIGNED_RED_RGTC1_EXT):
        case(GL_COMPRESSED_RED_RGTC1_EXT):
        case(GL_COMPRESSED_SIGNED_RED_GREEN_RGTC2_EXT):
        case(GL_COMPRESSED_RED_GREEN_RGTC2_EXT):
        case(GL_COMPRESSED_RGB_PVRTC_4BPPV1_IMG):
        case(GL_COMPRESSED_RGB_PVRTC_2BPPV1_IMG):
        case(GL_COMPRESSED_RGBA_PVRTC_4BPPV1_IMG):
        case(GL_COMPRESSED_RGBA_PVRTC_2BPPV1_IMG):
            return true;
        default:
            return false;
    }
}


bool
ImageUtils::isFloatingPointInternalFormat(GLint i)
{
    return
        (i >= 0x8C10 && i <= 0x8C17) || // GL_TEXTURE_RED_TYPE_ARB, et al
        (i >= 0x8814 && i <= 0x881F);   // GL_RGBA32F_ARB, et al
}

bool
ImageUtils::sameFormat(const osg::Image* lhs, const osg::Image* rhs)
{
    return
        lhs != 0L &&
        rhs != 0L &&
        lhs->getPixelFormat() == rhs->getPixelFormat() &&
        lhs->getDataType()    == rhs->getDataType();
}

bool
ImageUtils::textureArrayCompatible(const osg::Image* lhs, const osg::Image* rhs)
{
    return
        sameFormat(lhs, rhs) &&
        lhs->s() == rhs->s() &&
        lhs->t() == rhs->t() &&
        lhs->r() == rhs->r();
}

//------------------------------------------------------------------------

namespace
{
    //static const double r10= 1.0/1023.0;
    //static const double r8 = 1.0/255.0;
    //static const double r6 = 1.0/63.0;
    static const double r5 = 1.0/31.0;
    //static const double r4 = 1.0/15.0;
    static const double r3 = 1.0/7.0;
    static const double r2 = 1.0/3.0;

    // The scale factors to convert from an image data type to a
    // float. This is copied from OSG; I think the factors for the signed
    // types are wrong, but need to investigate further.

    template<typename T> struct GLTypeTraits;

    template<> struct GLTypeTraits<GLbyte>
    {
        static double scale(bool norm) { return norm? 1.0/128.0 : 1.0; } // XXX
    };

    template<> struct GLTypeTraits<GLubyte>
    {
        static double scale(bool norm) { return norm? 1.0/255.0 : 1.0; }
    };

    template<> struct GLTypeTraits<GLshort>
    {
        static double scale(bool norm) { return norm? 1.0/32768.0 : 1.0; } // XXX
    };

    template<> struct GLTypeTraits<GLushort>
    {
        static double scale(bool norm) { return norm? 1.0/65535.0 : 1.0; }
    };

    template<> struct GLTypeTraits<GLint>
    {
        static double scale(bool norm) { return norm? 1.0/2147483648.0 : 1.0; } // XXX
    };

    template<> struct GLTypeTraits<GLuint>
    {
        static double scale(bool norm) { return norm? 1.0/4294967295.0 : 1.0; }
    };

    template<> struct GLTypeTraits<GLfloat>
    {
        static double scale(bool norm) { return 1.0; }
    };

    // The Reader function that performs the read.
    template<int Format, typename T> struct ColorReader;
    template<int Format, typename T> struct ColorWriter;

    template<typename T>
    struct ColorReader<GL_DEPTH_COMPONENT, T>
    {
        static void read(const ImageUtils::PixelReader* ia, osg::Vec4f& out, int s, int t, int r, int m)
        {
            const T* ptr = (const T*)ia->data(s, t, r, m);
            float d = float(*ptr) * GLTypeTraits<T>::scale(ia->_normalized);
            out.set(d, d, d, 1.0f);
        }
    };

    template<typename T>
    struct ColorWriter<GL_DEPTH_COMPONENT, T>
    {
        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f& c, int s, int t, int r, int m)
        {
            T* ptr = (T*)iw->data(s, t, r, m);
            (*ptr) = (T)(c.r() / GLTypeTraits<T>::scale(iw->_normalized));
        }
    };

    template<typename T>
    struct ColorReader<GL_LUMINANCE, T>
    {
        static void read(const ImageUtils::PixelReader* ia, osg::Vec4f& out, int s, int t, int r, int m)
        {
            const T* ptr = (const T*)ia->data(s, t, r, m);
            float red = float(*ptr) * GLTypeTraits<T>::scale(ia->_normalized);
            out.set(red, red, red, 1.0f);
        }
    };

    template<typename T>
    struct ColorWriter<GL_LUMINANCE, T>
    {
        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f& c, int s, int t, int r, int m)
        {
            T* ptr = (T*)iw->data(s, t, r, m);
            (*ptr) = (T)(c.r() / GLTypeTraits<T>::scale(iw->_normalized));
        }
    };

    template<typename T>
    struct ColorReader<GL_RED, T>
    {
        static void read(const ImageUtils::PixelReader* ia, osg::Vec4f& out, int s, int t, int r, int m)
        {
            const T* ptr = (const T*)ia->data(s, t, r, m);
            float red = float(*ptr) * GLTypeTraits<T>::scale(ia->_normalized);
            out.set(red, red, red, 1.0f);
        }
    };

    template<typename T>
    struct ColorWriter<GL_RED, T>
    {
        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f& c, int s, int t, int r, int m)
        {
            T* ptr = (T*)iw->data(s, t, r, m);
            (*ptr) = (T)(c.r() / GLTypeTraits<T>::scale(iw->_normalized));
        }
    };

    template<typename T>
    struct ColorReader<GL_ALPHA, T>
    {
        static void read(const ImageUtils::PixelReader* ia, osg::Vec4f& out, int s, int t, int r, int m)
        {
            const T* ptr = (const T*)ia->data(s, t, r, m);
            float a = float(*ptr) * GLTypeTraits<T>::scale(ia->_normalized);
            out.set(1.0f, 1.0f, 1.0f, a);
        }
    };

    template<typename T>
    struct ColorWriter<GL_ALPHA, T>
    {
        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f& c, int s, int t, int r, int m)
        {
            T* ptr = (T*)iw->data(s, t, r, m);
            (*ptr) = (T)(c.a() / GLTypeTraits<T>::scale(iw->_normalized));
        }
    };

    template<typename T>
    struct ColorReader<GL_LUMINANCE_ALPHA, T>
    {
        static void read(const ImageUtils::PixelReader* ia, osg::Vec4f& out, int s, int t, int r, int m)
        {
            const T* ptr = (const T*)ia->data(s, t, r, m);
            float l = float(*ptr++) * GLTypeTraits<T>::scale(ia->_normalized);
            float a = float(*ptr) * GLTypeTraits<T>::scale(ia->_normalized);
            out.set(l, l, l, a);
        }
    };

    template<typename T>
    struct ColorWriter<GL_LUMINANCE_ALPHA, T>
    {
        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f& c, int s, int t, int r, int m )
        {
            T* ptr = (T*)iw->data(s, t, r, m);
            *ptr++ = (T)( c.r() / GLTypeTraits<T>::scale(iw->_normalized) );
            *ptr   = (T)( c.a() / GLTypeTraits<T>::scale(iw->_normalized) );
        }
    };

    template<typename T>
    struct ColorReader<GL_RG, T>
    {
        static void read(const ImageUtils::PixelReader* ia, osg::Vec4f& out, int s, int t, int r, int m)
        {
            const T* ptr = (const T*)ia->data(s, t, r, m);
            float red = float(*ptr++) * GLTypeTraits<T>::scale(ia->_normalized);
            float g = float(*ptr++) * GLTypeTraits<T>::scale(ia->_normalized);
            out.set(red, g, 0.0f, 1.0f);
        }
    };

    template<typename T>
    struct ColorWriter<GL_RG, T>
    {
        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f& c, int s, int t, int r, int m )
        {
            T* ptr = (T*)iw->data(s, t, r, m);
            *ptr++ = (T)( c.r() / GLTypeTraits<T>::scale(iw->_normalized) );
            *ptr++ = (T)( c.g() / GLTypeTraits<T>::scale(iw->_normalized) );
        }
    };

    template<typename T>
    struct ColorReader<GL_RGB, T>
    {
        static void read(const ImageUtils::PixelReader* ia, osg::Vec4f& out, int s, int t, int r, int m)
        {
            const T* ptr = (const T*)ia->data(s, t, r, m);
            float red = float(*ptr++) * GLTypeTraits<T>::scale(ia->_normalized);
            float g = float(*ptr++) * GLTypeTraits<T>::scale(ia->_normalized);
            float b = float(*ptr) * GLTypeTraits<T>::scale(ia->_normalized);
            out.set(red, g, b, 1.0f);
        }
    };

    template<typename T>
    struct ColorWriter<GL_RGB, T>
    {
        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f& c, int s, int t, int r, int m )
        {
            T* ptr = (T*)iw->data(s, t, r, m);
            *ptr++ = (T)( c.r() / GLTypeTraits<T>::scale(iw->_normalized) );
            *ptr++ = (T)( c.g() / GLTypeTraits<T>::scale(iw->_normalized) );
            *ptr++ = (T)( c.b() / GLTypeTraits<T>::scale(iw->_normalized) );
        }
    };

    template<typename T>
    struct ColorReader<GL_RGBA, T>
    {
        static void read(const ImageUtils::PixelReader* ia, osg::Vec4f& out, int s, int t, int r, int m)
        {
            const T* ptr = (const T*)ia->data(s, t, r, m);
            float red = float(*ptr++) * GLTypeTraits<T>::scale(ia->_normalized);
            float g = float(*ptr++) * GLTypeTraits<T>::scale(ia->_normalized);
            float b = float(*ptr++) * GLTypeTraits<T>::scale(ia->_normalized);
            float a = float(*ptr) * GLTypeTraits<T>::scale(ia->_normalized);
            out.set(red, g, b, a);
        }
    };

    template<typename T>
    struct ColorWriter<GL_RGBA, T>
    {
        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f& c, int s, int t, int r, int m)
        {
            T* ptr = (T*)iw->data(s, t, r, m);
            *ptr++ = (T)( c.r() / GLTypeTraits<T>::scale(iw->_normalized) );
            *ptr++ = (T)( c.g() / GLTypeTraits<T>::scale(iw->_normalized) );
            *ptr++ = (T)( c.b() / GLTypeTraits<T>::scale(iw->_normalized) );
            *ptr++ = (T)( c.a() / GLTypeTraits<T>::scale(iw->_normalized) );
        }
    };

    template<typename T>
    struct ColorReader<GL_BGR, T>
    {
        static void read(const ImageUtils::PixelReader* ia, osg::Vec4f& out, int s, int t, int r, int m)
        {
            const T* ptr = (const T*)ia->data(s, t, r, m);
            float b = float(*ptr) * GLTypeTraits<T>::scale(ia->_normalized);
            float g = float(*ptr++) * GLTypeTraits<T>::scale(ia->_normalized);
            float red = float(*ptr++) * GLTypeTraits<T>::scale(ia->_normalized);
            out.set(red, g, b, 1.0f);
        }
    };

    template<typename T>
    struct ColorWriter<GL_BGR, T>
    {
        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f& c, int s, int t, int r, int m )
        {
            T* ptr = (T*)iw->data(s, t, r, m);
            *ptr++ = (T)( c.b() / GLTypeTraits<T>::scale(iw->_normalized) );
            *ptr++ = (T)( c.g() / GLTypeTraits<T>::scale(iw->_normalized) );
            *ptr++ = (T)( c.r() / GLTypeTraits<T>::scale(iw->_normalized) );
        }
    };

    template<typename T>
    struct ColorReader<GL_BGRA, T>
    {
        static void read(const ImageUtils::PixelReader* ia, osg::Vec4f& out, int s, int t, int r, int m)
        {
            const T* ptr = (const T*)ia->data(s, t, r, m);
            float b = float(*ptr++) * GLTypeTraits<T>::scale(ia->_normalized);
            float g = float(*ptr++) * GLTypeTraits<T>::scale(ia->_normalized);
            float red = float(*ptr++) * GLTypeTraits<T>::scale(ia->_normalized);
            float a = float(*ptr) * GLTypeTraits<T>::scale(ia->_normalized);
            out.set(red, g, b, a);
        }
    };

    template<typename T>
    struct ColorWriter<GL_BGRA, T>
    {
        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f& c, int s, int t, int r, int m )
        {
            T* ptr = (T*)iw->data(s, t, r, m);
            *ptr++ = (T)( c.b() / GLTypeTraits<T>::scale(iw->_normalized) );
            *ptr++ = (T)( c.g() / GLTypeTraits<T>::scale(iw->_normalized) );
            *ptr++ = (T)( c.r() / GLTypeTraits<T>::scale(iw->_normalized) );
            *ptr++ = (T)( c.a() / GLTypeTraits<T>::scale(iw->_normalized) );
        }
    };

    template<typename T>
    struct ColorReader<0, T>
    {
        static void read(const ImageUtils::PixelReader* ia, osg::Vec4f& out, int s, int t, int r, int m)
        {
            out.set(1.0f, 1.0f, 1.0f, 1.0f);
        }
    };

    template<typename T>
    struct ColorWriter<0, T>
    {
        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f& c, int s, int t, int r, int m )
        {
            //nop
        }
    };

    template<>
    struct ColorReader<GL_UNSIGNED_SHORT_5_5_5_1, GLushort>
    {
        static void read(const ImageUtils::PixelReader* ia, osg::Vec4f& out, int s, int t, int r, int m)
        {
            GLushort p = *(const GLushort*)ia->data(s, t, r, m);
            //internal format GL_RGB5_A1 is implied
            out.set(
                r5*(float)(p>>11),
                r5*(float)((p&0x7c0)>>6),
                r5*(float)((p&0x3e)>>1),
                (float)(p&0x1));
        }
    };

    template<>
    struct ColorWriter<GL_UNSIGNED_SHORT_5_5_5_1, GLushort>
    {
        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f& c, int s, int t, int r, int m )
        {
            GLushort
                red = (unsigned short)(c.r()*255),
                g = (unsigned short)(c.g()*255),
                b = (unsigned short)(c.b()*255),
                a = c.a() < 0.15 ? 0 : 1;

            GLushort* ptr = (GLushort*)iw->data(s, t, r, m);
            *ptr = (((red) & (0xf8)) << 8) | (((g) & (0xf8)) << 3) | (((b) & (0xF8)) >> 2) | a;
        }
    };

    template<>
    struct ColorReader<GL_UNSIGNED_BYTE_3_3_2, GLubyte>
    {
        static void read(const ImageUtils::PixelReader* ia, osg::Vec4f& out, int s, int t, int r, int m)
        {
            GLubyte p = *(const GLubyte*)ia->data(s,t,r,m);
            // internal format GL_R3_G3_B2 is implied
            out.set( r3*(float)(p>>5), r3*(float)((p&0x28)>>2), r2*(float)(p&0x3), 1.0f );
        }
    };

    template<>
    struct ColorWriter<GL_UNSIGNED_BYTE_3_3_2, GLubyte>
    {
        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f& c, int s, int t, int r, int m )
        {
            GLubyte* ptr = (GLubyte*)iw->data(s,t,r,m);
            OE_WARN << LC << "Target GL_UNSIGNED_BYTE_3_3_2 not yet implemented" << std::endl;
        }
    };

    template<>
    struct ColorReader<GL_COMPRESSED_RGB_S3TC_DXT1_EXT, GLubyte>
    {
        static void read(const ImageUtils::PixelReader* pr, osg::Vec4f& out, int s, int t, int r, int m)
        {
            static const int BLOCK_BYTES = 8;

            unsigned int blocksPerRow = pr->_image->s()/4;
            unsigned int bs = s/4, bt = t/4;
            unsigned int blockStart = (bt*blocksPerRow+bs) * BLOCK_BYTES;

            const GLushort* p = (const GLushort*)(pr->data() + blockStart);

            GLushort c0p = *p++;
            osg::Vec4f c0(
                (float)(c0p >> 11)/31.0f,
                (float)((c0p & 0x07E0) >> 5)/63.0f,
                (float)((c0p & 0x001F))/31.0f,
                1.0f );

            GLushort c1p = *p++;
            osg::Vec4f c1(
                (float)(c1p >> 11)/31.0f,
                (float)((c1p & 0x07E0) >> 5)/63.0f,
                (float)((c1p & 0x001F))/31.0f,
                1.0f );

            static const float one_third  = 1.0f/3.0f;
            static const float two_thirds = 2.0f/3.0f;

            osg::Vec4f c2, c3;
            if ( c0p > c1p )
            {
                c2 = c0*two_thirds + c1*one_third;
                c3 = c0*one_third  + c1*two_thirds;
            }
            else
            {
                c2 = c0*0.5 + c1*0.5;
                c3.set(0,0,0,1);
            }

            unsigned int table = *(unsigned int*)p;
            int ls = s-4*bs, lt = t-4*bt; //int ls = s % 4, lt = t % 4;
            int x = ls + (4 * lt);

            unsigned int index = (table >> (2*x)) & 0x00000003;

            out =
                index == 0? c0 :
                index == 1? c1 :
                index == 2? c2 : c3;
        }
    };

    template<int GLFormat>
    inline ImageUtils::PixelReader::ReaderFunc
    chooseReader(GLenum dataType)
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
        case GL_UNSIGNED_INT_8_8_8_8_REV:
            return &ColorReader<GLFormat, GLubyte>::read;
        default:
            return &ColorReader<0, GLbyte>::read;
        }
    }

    inline ImageUtils::PixelReader::ReaderFunc
    getReader( GLenum pixelFormat, GLenum dataType )
    {
        switch( pixelFormat )
        {
        case GL_DEPTH_COMPONENT:
            return chooseReader<GL_DEPTH_COMPONENT>(dataType);
            break;
        case GL_LUMINANCE:
            return chooseReader<GL_LUMINANCE>(dataType);
            break;
        case GL_RED:
            return chooseReader<GL_RED>(dataType);
            break;
        case GL_ALPHA:
            return chooseReader<GL_ALPHA>(dataType);
            break;
        case GL_LUMINANCE_ALPHA:
            return chooseReader<GL_LUMINANCE_ALPHA>(dataType);
            break;
        case GL_RG:
            return chooseReader<GL_RG>(dataType);
            break;
        case GL_RGB:
            return chooseReader<GL_RGB>(dataType);
            break;
        case GL_RGBA:
            return chooseReader<GL_RGBA>(dataType);
            break;
        case GL_BGR:
            return chooseReader<GL_BGR>(dataType);
            break;
        case GL_BGRA:
            return chooseReader<GL_BGRA>(dataType);
            break;
        case GL_COMPRESSED_RGB_S3TC_DXT1_EXT:
            return &ColorReader<GL_COMPRESSED_RGB_S3TC_DXT1_EXT, GLubyte>::read;
            break;
        default:
            return 0L;
            break;
        }
    }
}

ImageUtils::PixelReader::PixelReader() :
    _bilinear(false),
    _sampleAsTexture(false),
    _sampleAsRepeatingTexture(false)
{
    setImage(NULL);
}

ImageUtils::PixelReader::PixelReader(const osg::Image* image) :
    _bilinear(false),
    _sampleAsTexture(false),
    _sampleAsRepeatingTexture(false)
{
    setImage(image);
}

void
ImageUtils::PixelReader::setImage(const osg::Image* image)
{
    _image = image;
    if (image)
    {
        _normalized = image->getDataType() == GL_UNSIGNED_BYTE;
        _colBytes = _image->getPixelSizeInBits() / 8;
        _rowBytes = _image->getRowStepInBytes(); //getRowSizeInBytes();
        _imageBytes = _image->getImageSizeInBytes();
        GLenum dataType = _image->getDataType();
        _reader = getReader( _image->getPixelFormat(), dataType );
        if ( !_reader )
        {
            OE_WARN << "[PixelReader] No reader found for pixel format " << std::hex << _image->getPixelFormat() << std::endl;
            _reader = &ColorReader<0,GLbyte>::read;
        }
    }
}

void
ImageUtils::PixelReader::setTexture(const osg::Texture* tex)
{
    if (tex)
    {
        setImage(tex->getImage(0));
        setSampleAsTexture(true);
        setBilinear(tex->getFilter(tex->MAG_FILTER) != tex->NEAREST);
        setSampleAsRepeatingTexture(tex->getWrap(tex->WRAP_S) == tex->REPEAT);
    }
}

osg::Vec4
ImageUtils::PixelReader::operator()(float u, float v, int r, int m) const
{
    osg::Vec4f temp;
    this->operator()(temp, u, v, r, m);
    return temp;
}

namespace {
    double fract(double x) {
        return x >= 0.0 ? (x - floor(x)) : (x - ceil(x));
    }
    float fractf(float x) {
        return x >= 0.0 ? (x - floorf(x)) : (x - ceilf(x));
    }
    float clamp(double x, double a, double b) {
        return x<a ? a : x>b ? b : x;
    }
    float clampf(float x, float a, float b) {
        return x<a ? a : x>b ? b : x;
    }
    double quantizeTo9bits(double x) {
        double frac, tmp = x - (double)(int)(x);
        double frac256 = (double)(int)(tmp*256.0 + 0.5);
        frac = frac256 / 256.0;
        return clamp(frac, 0.0, 1.0);
    }
    float quantizeTo9bitsf(float x) {
        float frac, tmp = x - (float)(int)(x);
        float frac256 = (float)(int)(tmp*256.0f + 0.5f);
        frac = frac256 / 256.0f;
        return clamp(frac, 0.0f, 1.0f);
    }

    // port of sample_2d_nearest from mesa
    /**
     * Sometimes we treat GLfloats as GLints.  On x86 systems, moving a float
     * as an int (thereby using integer registers instead of FP registers) is
     * a performance win.  Typically, this can be done with ordinary casts.
     * But with gcc's -fstrict-aliasing flag (which defaults to on in gcc 3.0)
     * these casts generate warnings.
     * The following union typedef is used to solve that.
     */
    typedef union { GLfloat f; GLint i; GLuint u; } fi_type;

    /** Return (as an integer) floor of float */
    static inline int ifloorf(float f)
    {
#if defined(USE_X86_ASM) && defined(__GNUC__) && defined(__i386__)
        /*
         * IEEE floor for computers that round to nearest or even.
         * 'f' must be between -4194304 and 4194303.
         * This floor operation is done by "(iround(f + .5) + iround(f - .5)) >> 1",
         * but uses some IEEE specific tricks for better speed.
         * Contributed by Josh Vanderhoof
         */
        int ai, bi;
        double af, bf;
        af = (3 << 22) + 0.5 + (double)f;
        bf = (3 << 22) + 0.5 - (double)f;
        /* GCC generates an extra fstp/fld without this. */
        __asm__("fstps %0" : "=m" (ai) : "t" (af) : "st");
        __asm__("fstps %0" : "=m" (bi) : "t" (bf) : "st");
        return (ai - bi) >> 1;
#else
        int ai, bi;
        double af, bf;
        fi_type u;
        af = (3 << 22) + 0.5 + (double)f;
        bf = (3 << 22) + 0.5 - (double)f;
        u.f = (float)af;  ai = u.i;
        u.f = (float)bf;  bi = u.i;
        return (ai - bi) >> 1;
#endif
    }
}

void
ImageUtils::PixelReader::operator()(osg::Vec4f& out, float u, float v, int r, int m) const
{
    if (!_bilinear)
    {
        // NN sample with clamp-to-edge from mesa in s_texfilter.c
        const float umin = 1.0f / (2.0f * (float)_image->s());
        const float vmin = 1.0f / (2.0f * (float)_image->t());
        int s = u<umin? 0 : u>(1.0f-umin)? _image->s()-1 : (int)floorf(u*(float)_image->s());
        int t = v<vmin? 0 : v>(1.0f-vmin)? _image->t()-1 : (int)floorf(v*(float)_image->t());
        (*_reader)(this, out, s, t, r, m);
    }

    else if (_sampleAsTexture)
    {
        // port of Mesa sample_2d_linear() in s_texfilter.c

        float tex_size_x = (float)_image->s();
        float tex_size_y = (float)_image->t();

        float unnorm_tex_coord_x = (u * tex_size_x) - 0.5f;
        float unnorm_tex_coord_y = (v * tex_size_y) - 0.5f;

        float snap_tex_coord_x = (floorf(unnorm_tex_coord_x) + 0.5f) / tex_size_x;
        float snap_tex_coord_y = (floorf(unnorm_tex_coord_y) + 0.5f) / tex_size_y;

        // wut?
        // NVIDIA uses 9-bit fixed point format with 8-bit fractional value.
        // So we have to quantize our coordinates to match. If you don't
        // do this you will have a bad time and coords > 0.5 will return
        // different values than in GLSL's texture method.
        // https://docs.nvidia.com/cuda/cuda-c-programming-guide/index.html#linear-filtering
        snap_tex_coord_x = quantizeTo9bitsf(snap_tex_coord_x);
        snap_tex_coord_y = quantizeTo9bitsf(snap_tex_coord_y);

        float sf = floorf(snap_tex_coord_x * (tex_size_x - 1.0f));
        float tf = floorf(snap_tex_coord_y * (tex_size_y - 1.0f));

        int s, t;

        if (_sampleAsRepeatingTexture)
        {
            s = sf >= 0.0 ? (int)sf : (int)fmodf(sf, tex_size_x);
            t = tf >= 0.0 ? (int)tf : (int)fmodf(tf, tex_size_y);
        }
        else
        {
            s = (int)sf;
            t = (int)tf;
        }

        float fx = fractf(unnorm_tex_coord_x);
        float fy = fractf(unnorm_tex_coord_y);

        int splus1, tplus1;
        if (_sampleAsRepeatingTexture)
        {
            splus1 = (s + 1 < _image->s()) ? s + 1 : 0;
            tplus1 = (t + 1 < _image->t()) ? t + 1 : 0;
        }
        else
        {
            splus1 = (s + 1 < _image->s()) ? s + 1 : s;
            tplus1 = (t + 1 < _image->t()) ? t + 1 : t;
        }

        osg::Vec4f p1, p2, p3, p4;
        (*_reader)(this, p1, s, t, r, m);
        (*_reader)(this, p2, splus1, t, r, m);
        (*_reader)(this, p3, s, tplus1, r, m);
        (*_reader)(this, p4, splus1, tplus1, r, m);

        p1 = p1 * (1.0 - fx) + p2 * fx;
        p2 = p3 * (1.0 - fx) + p4 * fx;
        out = p1 * (1.0 - fy) + p2 * fy;
    }

    else // sample as image
    {
        float sizeS = (float)(_image->s() - 1);
        float sizeT = (float)(_image->t() - 1);

        // u, v => [0..1]
        float s = u * sizeS;
        float t = v * sizeT;

        float s0 = osg::maximum(floorf(s), 0.0f);
        float s1 = osg::minimum(s0 + 1.0f, sizeS);
        float smix = s0 < s1 ? (s - s0) / (s1 - s0) : 0.0f;

        float t0 = osg::maximum(floorf(t), 0.0f);
        float t1 = osg::minimum(t0 + 1.0f, sizeT);
        float tmix = t0 < t1 ? (t - t0) / (t1 - t0) : 0.0f;

        osg::Vec4f UL, UR, LL, LR;

        (*_reader)(this, UL, (int)s0, (int)t0, r, m); // upper left
        (*_reader)(this, UR, (int)s1, (int)t0, r, m); // upper right
        (*_reader)(this, LL, (int)s0, (int)t1, r, m); // lower left
        (*_reader)(this, LR, (int)s1, (int)t1, r, m); // lower right

        osg::Vec4f TOP = UL * (1.0f - smix) + UR * smix;
        osg::Vec4f BOT = LL * (1.0f - smix) + LR * smix;

        out = TOP * (1.0f - tmix) + BOT * tmix;
    }
}

void
ImageUtils::PixelReader::operator()(osg::Vec4f& out, double u, double v, int r, int m) const
{
    if (!_bilinear)
    {
        // NN sample with clamp-to-edge from mesa in s_texfilter.c
        const double umin = 1.0 / (2.0 * (double)_image->s());
        const double vmin = 1.0 / (2.0 * (double)_image->t());
        int s = u<umin ? 0 : u>(1.0 - umin) ? _image->s() - 1 : (int)floorf(u*(double)_image->s());
        int t = v<vmin ? 0 : v>(1.0 - vmin) ? _image->t() - 1 : (int)floorf(v*(double)_image->t());
        (*_reader)(this, out, s, t, r, m);
    }

    else if (_sampleAsTexture)
    {
        // port of Mesa sample_2d_linear() in s_texfilter.c

        double tex_size_x = (double)_image->s();
        double tex_size_y = (double)_image->t();

        double unnorm_tex_coord_x = (u * tex_size_x) - 0.5;
        double unnorm_tex_coord_y = (v * tex_size_y) - 0.5;

        double snap_tex_coord_x = (floorf(unnorm_tex_coord_x) + 0.5) / tex_size_x;
        double snap_tex_coord_y = (floorf(unnorm_tex_coord_y) + 0.5) / tex_size_y;

        // wut?
        // NVIDIA uses 9-bit fixed point format with 8-bit fractional value.
        // So we have to quantize our coordinates to match. If you don't
        // do this you will have a bad time and coords > 0.5 will return
        // different values than in GLSL's texture method.
        // https://docs.nvidia.com/cuda/cuda-c-programming-guide/index.html#linear-filtering
        snap_tex_coord_x = quantizeTo9bitsf(snap_tex_coord_x);
        snap_tex_coord_y = quantizeTo9bitsf(snap_tex_coord_y);

        double sf = floor(snap_tex_coord_x * (tex_size_x - 1.0));
        double tf = floor(snap_tex_coord_y * (tex_size_y - 1.0));

        int s, t;

        if (_sampleAsRepeatingTexture)
        {
            s = sf >= 0.0 ? (int)sf : (int)fmod(sf, tex_size_x);
            t = tf >= 0.0 ? (int)tf : (int)fmod(tf, tex_size_y);
        }
        else
        {
            s = (int)sf;
            t = (int)tf;
        }

        double fx = fract(unnorm_tex_coord_x);
        double fy = fract(unnorm_tex_coord_y);

        int splus1, tplus1;
        if (_sampleAsRepeatingTexture)
        {
            splus1 = (s + 1 < _image->s()) ? s + 1 : 0;
            tplus1 = (t + 1 < _image->t()) ? t + 1 : 0;
        }
        else
        {
            splus1 = (s + 1 < _image->s()) ? s + 1 : s;
            tplus1 = (t + 1 < _image->t()) ? t + 1 : t;
        }

        osg::Vec4f p1, p2, p3, p4;
        (*_reader)(this, p1, s, t, r, m);
        (*_reader)(this, p2, splus1, t, r, m);
        (*_reader)(this, p3, s, tplus1, r, m);
        (*_reader)(this, p4, splus1, tplus1, r, m);

        p1 = p1 * (1.0 - fx) + p2 * fx;
        p2 = p3 * (1.0 - fx) + p4 * fx;
        out = p1 * (1.0 - fy) + p2 * fy;
    }

    else // sample as image
    {
        double sizeS = (double)(_image->s() - 1);
        double sizeT = (double)(_image->t() - 1);

        // u, v => [0..1]
        double s = u * sizeS;
        double t = v * sizeT;

        double s0 = osg::maximum(floor(s), 0.0);
        double s1 = osg::minimum(s0 + 1.0, sizeS);
        double smix = s0 < s1 ? (s - s0) / (s1 - s0) : 0.0;

        double t0 = osg::maximum(floor(t), 0.0);
        double t1 = osg::minimum(t0 + 1.0, sizeT);
        double tmix = t0 < t1 ? (t - t0) / (t1 - t0) : 0.0;

        osg::Vec4f UL, UR, LL, LR;

        (*_reader)(this, UL, (int)s0, (int)t0, r, m); // upper left
        (*_reader)(this, UR, (int)s1, (int)t0, r, m); // upper right
        (*_reader)(this, LL, (int)s0, (int)t1, r, m); // lower left
        (*_reader)(this, LR, (int)s1, (int)t1, r, m); // lower right

        osg::Vec4f TOP = UL * (1.0f - smix) + UR * smix;
        osg::Vec4f BOT = LL * (1.0f - smix) + LR * smix;

        out = TOP * (1.0f - tmix) + BOT * tmix;
    }
}

osg::Vec4f
ImageUtils::PixelReader::operator()(double u, double v, int r, int m) const
{
    osg::Vec4f temp;
    this->operator()(temp, u, v, r, m);
    return temp;
}

bool
ImageUtils::PixelReader::supports( GLenum pixelFormat, GLenum dataType )
{
    return getReader(pixelFormat, dataType) != 0L;
}

//------------------------------------------------------------------------

namespace
{
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
            return 0L;
        }
    }

    inline ImageUtils::PixelWriter::WriterFunc getWriter(GLenum pixelFormat, GLenum dataType)
    {
        switch( pixelFormat )
        {
        case GL_DEPTH_COMPONENT:
            return chooseWriter<GL_DEPTH_COMPONENT>(dataType);
            break;
        case GL_LUMINANCE:
            return chooseWriter<GL_LUMINANCE>(dataType);
            break;
        case GL_RED:
            return chooseWriter<GL_RED>(dataType);
            break;
        case GL_ALPHA:
            return chooseWriter<GL_ALPHA>(dataType);
            break;
        case GL_LUMINANCE_ALPHA:
            return chooseWriter<GL_LUMINANCE_ALPHA>(dataType);
            break;
        case GL_RG:
            return chooseWriter<GL_RG>(dataType);
            break;
        case GL_RGB:
            return chooseWriter<GL_RGB>(dataType);
            break;
        case GL_RGBA:
            return chooseWriter<GL_RGBA>(dataType);
            break;
        case GL_BGR:
            return chooseWriter<GL_BGR>(dataType);
            break;
        case GL_BGRA:
            return chooseWriter<GL_BGRA>(dataType);
            break;
        default:
            return 0L;
            break;
        }
    }
}

ImageUtils::PixelWriter::PixelWriter(osg::Image* image) :
_image(image)
{
    if (image)
    {
        _normalized = image->getDataType() == GL_UNSIGNED_BYTE;
        _colBytes = _image->getPixelSizeInBits() / 8;
        _rowBytes = _image->getRowStepInBytes();
        _imageBytes = _image->getImageSizeInBytes();
        GLenum dataType = _image->getDataType();
        _writer = getWriter( _image->getPixelFormat(), dataType );
        if ( !_writer )
        {
            OE_WARN << "[PixelWriter] No writer found for pixel format " << std::hex << _image->getPixelFormat() << std::endl;
            _writer = &ColorWriter<0, GLbyte>::write;
        }
    }
}

bool
ImageUtils::PixelWriter::supports( GLenum pixelFormat, GLenum dataType )
{
    return getWriter(pixelFormat, dataType) != 0L;
}

void
ImageUtils::PixelWriter::assign(const osg::Vec4& c)
{
    if (_image->valid())
    {
        for(int r=0; r<_image->r(); ++r)
            for(int t=0; t<_image->t(); ++t)
                for(int s=0; s<_image->s(); ++s)
                    (*this)(c, s, t, r);
    }
}

void
ImageUtils::PixelWriter::assign(const osg::Vec4& c, int layer)
{
    if (_image->valid())
    {
        for(int t=0; t<_image->t(); ++t)
            for(int s=0; s<_image->s(); ++s)
                (*this)(c, s, t, layer);
    }
}

unsigned char*
ImageUtils::PixelWriter::data(int s, int t, int r, int m) const
{
    return m == 0 ?
        _image->data() + s*_colBytes + t*_rowBytes + r*_imageBytes :
        _image->getMipmapData(m) + (s)*_colBytes + (t)*(_rowBytes>>m) + r*(_imageBytes>>m);
//        _image->getMipmapData(m-1) + (s>>m)*_colBytes + (t>>m)*(_rowBytes>>m) + r*(_imageBytes>>m);
}

TextureAndImageVisitor::TextureAndImageVisitor() :
osg::NodeVisitor()
{
    setNodeMaskOverride( ~0L );
    setTraversalMode(TRAVERSE_ALL_CHILDREN);
}

void
TextureAndImageVisitor::apply(osg::Texture& texture)
{
    for (unsigned k = 0; k < texture.getNumImages(); ++k)
    {
        osg::Image* image = texture.getImage(k);
        if (image)
        {
            apply(*image);
        }
    }
}

void
TextureAndImageVisitor::apply(osg::Node& node)
{
    if (node.getStateSet())
        apply(*node.getStateSet());

    traverse(node);
}

void
TextureAndImageVisitor::apply(osg::StateSet& stateSet)
{
    osg::StateSet::TextureAttributeList& a = stateSet.getTextureAttributeList();
    for (osg::StateSet::TextureAttributeList::iterator i = a.begin(); i != a.end(); ++i)
    {
        osg::StateSet::AttributeList& b = *i;
        for (osg::StateSet::AttributeList::iterator j = b.begin(); j != b.end(); ++j)
        {
            osg::StateAttribute* sa = j->second.first.get();
            if (sa)
            {
                osg::Texture* tex = dynamic_cast<osg::Texture*>(sa);
                if (tex)
                {
                    apply(*tex);
                }
            }
        }
    }
}
