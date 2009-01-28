/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <string.h>

using namespace osgEarth;

bool ImageUtils::copyAsSubImage( osg::Image* src, osg::Image* dst, int dst_start_col, int dst_start_row )
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
        void* src_data = src->data( 0, src_row, 0 );
        void* dst_data = dst->data( dst_start_col, dst_row, 0 );
        memcpy( dst_data, src_data, src->getRowSizeInBytes() );
    }

    return true;
}

osg::Image* ImageUtils::resizeImage( osg::Image* input, unsigned int new_s, unsigned int new_t )
{
    osg::Image* output = NULL;

    GLenum pf = input->getPixelFormat();

    if ( input && new_s > 0 && new_t > 0 && 
        (pf == GL_RGBA || pf == GL_RGB || pf == GL_LUMINANCE || pf == GL_LUMINANCE_ALPHA) )
    {
        float s_ratio = (float)input->s()/(float)new_s;
        float t_ratio = (float)input->t()/(float)new_t;

        output = new osg::Image();
        output->allocateImage( new_s, new_t, 1, pf, input->getDataType(), input->getPacking() );

        unsigned int pixel_size_bytes = input->getRowSizeInBytes() / input->s();

        for( unsigned int output_row=0; output_row < output->t(); output_row++ )
        {
            // get an appropriate input row
            float output_row_ratio = (float)output_row/(float)output->t();
            unsigned int input_row = (unsigned int)( output_row_ratio * (float)input->t() );
            if ( input_row >= input->t() ) input_row = input->t()-1;
            else if ( input_row < 0 ) input_row = 0;

            for( unsigned int output_col = 0; output_col < output->s(); output_col++ )
            {
                float output_col_ratio = (float)output_col/(float)output->s();
                unsigned int input_col = (unsigned int)( output_col_ratio * (float)input->s() );
                if ( input_col >= input->s() ) input_col = input->s()-1;
                else if ( input_row < 0 ) input_row = 0;
                
                memcpy( output->data( output_col, output_row ), input->data( input_col, input_row ), pixel_size_bytes );
            }
        }
    }

    return output;
}

osg::Image* ImageUtils::cropImage(osg::Image* image, double src_minx, double src_miny, double src_maxx, double src_maxy,
                                                     double dst_minx, double dst_miny, double dst_maxx, double dst_maxy)
{
    int windowX       = osg::maximum( (int)floor( (dst_minx - src_minx) / (src_maxx - src_minx) * (double)image->s()), 0);
    int windowY       = osg::maximum( (int)floor( (dst_miny - src_miny) / (src_maxy - src_miny) * (double)image->t()), 0);
    int windowWidth   = osg::minimum( (int)ceil(  (dst_maxx - src_minx) / (src_maxx - src_minx) * (double)image->s()), image->s()) - windowX;
    int windowHeight  = osg::minimum( (int)ceil(  (dst_maxy - src_miny) / (src_maxy - src_miny) * (double)image->t()), image->t()) - windowY;

    //osg::notify(osg::NOTICE) << "Copying from " << windowX << ", " << windowY << ", " << windowWidth << ", " << windowHeight << std::endl;

    //Allocate the croppped image
    osg::Image* cropped = new osg::Image;
    cropped->allocateImage(windowWidth, windowHeight, 1, image->getPixelFormat(), image->getDataType());
    
    
    for (int src_row = windowY, dst_row=0; dst_row < windowHeight; src_row++, dst_row++)
    {
        void* src_data = image->data(windowX, src_row, 0);
        void* dst_data = cropped->data(0, dst_row, 0);
        memcpy( dst_data, src_data, cropped->getRowSizeInBytes());
    }

    return cropped;
}
