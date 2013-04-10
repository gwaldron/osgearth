/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
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

#ifndef OSGEARTH_ARCGIS_EXTENT_H
#define OSGEARTH_ARCGIS_EXTENT_H 1

class Extent 
{
public:
    Extent() :
      is_valid(false) { }

      Extent( double _xmin, double _ymin, double _xmax, double _ymax, const std::string& _srs ) :
      xmin(_xmin), ymin(_ymin), xmax(_xmax), ymax(_ymax), srs_str(_srs), is_valid(true) { }

      Extent( const Extent& rhs ) :
      xmin(rhs.xmin), ymin(rhs.ymin), xmax(rhs.xmax), ymax(rhs.ymax), srs_str(rhs.srs_str), is_valid(rhs.is_valid) { }

      double xMin() const { return xmin; }
      double yMin() const { return ymin; }
      double xMax() const { return xmax; }
      double yMax() const { return ymax; }

      const std::string& srs() { return srs_str; }

      bool isValid() const { return is_valid; }

private:
    double xmin, ymin, xmax, ymax;
    std::string srs_str;
    bool is_valid;
};

#endif // OSGEARTH_ARCGIS_EXTENT_H
