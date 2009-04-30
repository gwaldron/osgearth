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

#include <osgEarth/HeightFieldUtils>
#include <osgEarth/Mercator>
#include <osg/Notify>

using namespace osgEarth;

float
HeightFieldUtils::getHeightAtPixel(const osg::HeightField* hf, float c, float r, ElevationInterpolation interpolation)
{
    float result = 0.0;
    if (interpolation == NEAREST)
    {
        //Nearest interpolation
        result = hf->getHeight((unsigned int)c, (unsigned int)r);
    }
    else
    {
        //osg::notify(osg::INFO) << "getHeightAtPixel: (" << c << ", " << r << ")" << std::endl;
        int rowMin = osg::maximum((int)floor(r), 0);
        int rowMax = osg::maximum(osg::minimum((int)ceil(r), (int)(hf->getNumRows()-1)), 0);
        int colMin = osg::maximum((int)floor(c), 0);
        int colMax = osg::maximum(osg::minimum((int)ceil(c), (int)(hf->getNumColumns()-1)), 0);

        if (rowMin > rowMax) rowMin = rowMax;
        if (colMin > colMax) colMin = colMax;

        float urHeight = hf->getHeight(colMax, rowMax);
        float llHeight = hf->getHeight(colMin, rowMin);
        float ulHeight = hf->getHeight(colMin, rowMax);
        float lrHeight = hf->getHeight(colMax, rowMin);

        //Make sure not to use NoData in the interpolation
        if (urHeight == NO_DATA_VALUE || llHeight == NO_DATA_VALUE || ulHeight == NO_DATA_VALUE || lrHeight == NO_DATA_VALUE)
        {
            return NO_DATA_VALUE;
        }

        //osg::notify(osg::INFO) << "Heights (ll, lr, ul, ur) ( " << llHeight << ", " << urHeight << ", " << ulHeight << ", " << urHeight << std::endl;

        if (interpolation == BILINEAR)
        {
            //Check for exact value
            if ((colMax == colMin) && (rowMax == rowMin))
            {
                //osg::notify(osg::NOTICE) << "Exact value" << std::endl;
                result = hf->getHeight((int)c, (int)r);
            }
            else if (colMax == colMin)
            {
                //osg::notify(osg::NOTICE) << "Vertically" << std::endl;
                //Linear interpolate vertically
                result = ((float)rowMax - r) * llHeight + (r - (float)rowMin) * ulHeight;
            }
            else if (rowMax == rowMin)
            {
                //osg::notify(osg::NOTICE) << "Horizontally" << std::endl;
                //Linear interpolate horizontally
                result = ((float)colMax - c) * llHeight + (c - (float)colMin) * lrHeight;
            }
            else
            {
                //osg::notify(osg::NOTICE) << "Bilinear" << std::endl;
                //Bilinear interpolate
                float r1 = ((float)colMax - c) * llHeight + (c - (float)colMin) * lrHeight;
                float r2 = ((float)colMax - c) * ulHeight + (c - (float)colMin) * urHeight;

                //osg::notify(osg::INFO) << "r1, r2 = " << r1 << " , " << r2 << std::endl;

                result = ((float)rowMax - r) * r1 + (r - (float)rowMin) * r2;
            }
        }
        else if (interpolation == AVERAGE)
        {
            double x_rem = c - (int)c;
            double y_rem = r - (int)r;

            double w00 = (1.0 - y_rem) * (1.0 - x_rem) * (double)llHeight;
            double w01 = (1.0 - y_rem) * x_rem * (double)lrHeight;
            double w10 = y_rem * (1.0 - x_rem) * (double)ulHeight;
            double w11 = y_rem * x_rem * (double)urHeight;

            result = (float)(w00 + w01 + w10 + w11);
        }
    }

    return result;
}

float
HeightFieldUtils::getHeightAtLocation(const osg::HeightField* hf, float x, float y, ElevationInterpolation interpolation)
{
    //Determine the pixel to sample
    osg::Vec3 origin = hf->getOrigin();

    float px = (x - origin.x()) / hf->getXInterval();
    float py = (y - origin.y()) / hf->getYInterval();

    return getHeightAtPixel(hf, px, py, interpolation);
}


/******************************************************************************************/

ReplaceInvalidDataOperator::ReplaceInvalidDataOperator():
_replaceWith(0.0f)
{
}

void
ReplaceInvalidDataOperator::operator ()(osg::HeightField *heightField)
{
    if (heightField && _validDataOperator.valid())
    {
        for (unsigned int i = 0; i < heightField->getHeightList().size(); ++i)
        {
            float elevation = heightField->getHeightList()[i];
            if (!(*_validDataOperator)(elevation))
            {
                heightField->getHeightList()[i] = _replaceWith;
            }
        }
    }
}


/******************************************************************************************/
FillNoDataOperator::FillNoDataOperator():
_defaultValue(0.0f)
{
}

void
FillNoDataOperator::operator ()(osg::HeightField *heightField)
{
    if (heightField && _validDataOperator.valid())
    {
        for( unsigned int row=0; row < heightField->getNumRows(); row++ )
        {
            for( unsigned int col=0; col < heightField->getNumColumns(); col++ )
            {
                float val = heightField->getHeight(col, row);

                if (!(*_validDataOperator)(val))
                {
                    if ( col > 0 )
                        val = heightField->getHeight(col-1,row);
                    else if ( col <= heightField->getNumColumns()-1 )
                        val = heightField->getHeight(col+1,row);

                    if (!(*_validDataOperator)(val))
                    {
                        if ( row > 0 )
                            val = heightField->getHeight(col, row-1);
                        else if ( row < heightField->getNumRows()-1 )
                            val = heightField->getHeight(col, row+1);
                    }

                    if (!(*_validDataOperator)(val))
                    {
                        val = _defaultValue;
                    }

                    heightField->setHeight( col, row, val );
                }
            }
        }
    }
}