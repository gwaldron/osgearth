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

#include <osgEarth/ImageToHeightFieldConverter>
#include <osg/Notify>
#include <limits.h>

using namespace osgEarth;

static bool
isNoData( short s )
{
    return s == SHRT_MAX || s == SHRT_MIN;
}

static bool
isNoData( float f )
{
    return f == FLT_MAX || f == FLT_MIN;
}


ImageToHeightFieldConverter::ImageToHeightFieldConverter()
: _nodata_value( 0.0f ),
  _replace_nodata( false )
{
    //NOP
}

void
ImageToHeightFieldConverter::setRemoveNoDataValues( bool which, float f )
{
    _nodata_value = f;
    _replace_nodata = which;
}

osg::HeightField*
ImageToHeightFieldConverter::convert(osg::Image* image, float scaleFactor)
{
    //osg::notify(osg::NOTICE) << "Scale factor " << scaleFactor << std::endl;
	if (image)
	{
        osg::HeightField *hf = new osg::HeightField;
		//osg::notify(osg::NOTICE) << "Read heightfield image" << std::endl;
		hf->allocate(image->s(), image->t());
				
        // first create the new heightfield
		for( unsigned int row=0; row < image->t(); row++ )
        {
            for( unsigned int col=0; col < image->s(); col++ )
            {
                unsigned char* ptr = image->data( col, row );
                if ( image->getPixelSizeInBits() == 16 )
                {
                    short val = (short)*(short*)ptr;
                    if ( _replace_nodata && isNoData( val ) )
                        hf->setHeight( col, row, FLT_MAX );
                    else
                        hf->setHeight( col, row, (float)val );
                }
                else if ( image->getPixelSizeInBits() == 32 )
                {
                    float val = (float)*(float*)ptr;
                    if ( _replace_nodata && isNoData( val ) )
                        hf->setHeight( col, row, FLT_MAX );
                    else
                        hf->setHeight( col, row, val );
                }
            }
        }

        // scan for and replace NODATA values. This algorithm is terrible but good enough for now
        if ( _replace_nodata )
        {
            for( unsigned int row=0; row < hf->getNumRows(); row++ )
            {
                for( unsigned int col=0; col < hf->getNumColumns(); col++ )
                {
                    float val = hf->getHeight(col, row);
                    if ( isNoData( val ) )
                    {
                        if ( col > 0 )
                            val = hf->getHeight(col-1,row);
                        else if ( col <= hf->getNumColumns()-1 )
                            val = hf->getHeight(col+1,row);

                        if ( isNoData( val ) )
                        {
                            if ( row > 0 )
                                val = hf->getHeight(col, row-1);
                            else if ( row < hf->getNumRows()-1 )
                                val = hf->getHeight(col, row+1);
                        }

                        if ( isNoData( val ) )
                        {
                            val = _nodata_value;
                        }

                        hf->setHeight( col, row, val );
                    }
                }
            }
        }

        // finally, apply the scale factor.
        for( osg::FloatArray::iterator i = hf->getFloatArray()->begin(); i != hf->getFloatArray()->end(); i++ )
        {
            (*i) *= scaleFactor;
        }

        return hf;
    }
    return NULL;
}

osg::Image*
ImageToHeightFieldConverter::convert(osg::HeightField* hf, int pixelSize)
{
	if (hf)
	{
        int type;
        if (pixelSize == 16) type = GL_SHORT;
        else if (pixelSize == 32) type = GL_FLOAT;
        else type = GL_SHORT;

        osg::Image* image = new osg::Image;
        image->allocateImage(hf->getNumColumns(), hf->getNumRows(), 1, GL_LUMINANCE, type);

        for( unsigned int row=0; row < hf->getNumRows(); row++ )
        {
            for( unsigned int col=0; col < hf->getNumColumns(); col++ )
            {
                if (pixelSize == 16)
                {
                    short val = (short)hf->getHeight(col,row);
                    *((short*)image->data( col, row)) = val;
                }
                else if (pixelSize == 32)
                {
                    float val = (float)hf->getHeight(col,row);
                    *((float*)image->data( col, row)) = val;
                }
            }
        }
        return image;
    }
    return NULL;
}
