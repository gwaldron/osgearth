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
HeightFieldUtils::getHeightAtPixel(const osg::HeightField* hf, float c, float r)
{
    //osg::notify(osg::INFO) << "getInterpolateValue: (" << c << ", " << r << ")" << std::endl;
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

    //osg::notify(osg::INFO) << "Heights (ll, lr, ul, ur) ( " << llHeight << ", " << urHeight << ", " << ulHeight << ", " << urHeight << std::endl;

    float result = 0.0f;

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

    return result;
}

float
HeightFieldUtils::getHeightAtLocation(const osg::HeightField* hf, float x, float y)
{
    //Determine the pixel to sample
    osg::Vec3 origin = hf->getOrigin();

    float px = (x - origin.x()) / hf->getXInterval();
    float py = (y - origin.y()) / hf->getYInterval();

    //float px = ((x - origin.x()) / ((float)(hf->getNumColumns() -1) * hf->getXInterval())) * ((float)hf->getNumColumns()-1);
    //float py = ((y - origin.y()) / ((float)(hf->getNumRows() -1) * hf->getYInterval())) * ((float)hf->getNumRows()-1);

    return getHeightAtPixel(hf, px, py);
}

osg::HeightField* 
HeightFieldUtils::extractHeightField(const osg::HeightField* hf,
                                     double minx, double miny, double maxx, double maxy,
                                     int numCols, int numRows)
{
    if (numCols < 0) numCols = (int)((maxx - minx) / hf->getXInterval()) + 1;
    if (numRows < 0) numRows = (int)((maxy - miny) / hf->getYInterval()) + 1;

    osg::HeightField* result = new osg::HeightField;
    result->allocate(numCols, numRows);

    float dx = (maxx - minx) / (numCols-1);
    float dy = (maxy - miny) / (numRows-1);
    
    for (int c = 0; c < numCols; ++c)
    {
        float x = minx + (float)c * dx;
        for (int r = 0; r < numRows; ++r)
        {
            float y = miny + (float)r * dy;
            float height = getHeightAtLocation(hf, x, y);
            result->setHeight(c,r, height);
        }
    }

    return result;

}

osg::HeightField* HeightFieldUtils::convertMercatorToGeodetic(const TileKey* key, osg::HeightField* hf)
{
  if (!key->isMercator())
  {
    osg::notify(osg::NOTICE) << "HeightFieldUtils::convertMercatorToGeodetic can only operate on Mercator TileKeys" << std::endl;
    return 0;
  }

  //Get the geographic extents of the heightfield
  double min_lon, min_lat, max_lon, max_lat;
  key->getGeoExtent().getBounds(min_lon, min_lat, max_lon, max_lat);

  //Get the native, meter, extents of the heightfield
  double min_x, min_y, max_x, max_y;
  Mercator::latLongToMeters(min_lat, min_lon, min_x, min_y);
  Mercator::latLongToMeters(max_lat, max_lon, max_x, max_y);

  //Get the native width of the tile
  double width  = (max_x - min_x);
  double height = (max_y - min_y);

  double dx = width / (double)(hf->getNumColumns() -1);
  double dy = height / (double)(hf->getNumRows() -1);


  //Allocate the heighfield.  It will have the same dimensions as the incoming heightfield
  osg::HeightField *result = new osg::HeightField;
  result->allocate(hf->getNumColumns(), hf->getNumRows());

  double dlon = (max_lon - min_lon) / (double)(hf->getNumColumns() -1);
  double dlat = (max_lat - min_lat) / (double)(hf->getNumRows() - 1);



  //Sample at each lat/lon
  for (unsigned int c = 0; c < hf->getNumColumns(); ++c)
  {
    //Determine the longitude sampling point
    double lon = min_lon + (double)c * dlon;

    for (unsigned int r = 0; r < hf->getNumRows(); ++r)
    {
      //Determine the latitude sampling point
      double lat = min_lat + (double)r * dlat;

      //Convert the lat/lon to mercator
      double x, y;
      Mercator::latLongToMeters(lat, lon, x, y);

      //Determine the pixel location
      float px = (x - min_x) / dx;
      float py = (y - min_y) / dy;
      //osg::notify(osg::NOTICE) << "Sampling at (" << px << "," << py << ") for (" << c << "," << r << ")" << std::endl;
      float h = getHeightAtPixel(hf, px, py);
      result->setHeight(c, r, h);
    }
  }

  return result; 
}

bool HeightFieldUtils::contains(osg::HeightField* hf, double x, double y)
{
    if (!hf) return false;

    double minX = hf->getOrigin().x();
    double minY = hf->getOrigin().y();
    double maxX = minX + hf->getXInterval() * (double)(hf->getNumColumns()-1);
    double maxY = minY + hf->getYInterval() * (double)(hf->getNumRows()-1);

    return (x >= minX && x <= maxX && y >= minY && y <= maxY);
}

osg::HeightField* HeightFieldUtils::compositeHeightField(double minx, double miny, double maxx, double maxy,
                                       unsigned int numCols, unsigned int numRows,
                                       std::vector<osg::ref_ptr<osg::HeightField> > &heightFields)
{
    if (heightFields.size() == 0) return 0;

    osg::HeightField* hf = new osg::HeightField;
    hf->allocate(numCols, numRows);

    double dx = (maxx - minx) / (double)(numCols-1);
    double dy = (maxy - miny) / (double)(numRows-1);


    for (unsigned int c = 0; c < numCols; ++c)
    {
        double x = minx + (double)c * dx;
        for (unsigned int r = 0; r < numRows; ++r)
        {
            double y = miny + (double)r * dy;
            double h = 0;
            for (unsigned int i = 0; i < heightFields.size(); ++i)
            {
                osg::HeightField *hf2 = heightFields[i].get();
                if (contains(hf2, x, y))
                {
                    h = HeightFieldUtils::getHeightAtLocation(hf2, x, y);
                    break;
                }
            }
            hf->setHeight(c, r, h);
        }
    }
    return hf;
}
