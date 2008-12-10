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

#include <osgEarth/TileGridProfile>

using namespace osgEarth;

TileGridProfile::TileGridProfile( double _xmin, double _ymin, double _xmax, double _ymax, int _px_width )
{
    xmin = _xmin;
    ymin = _ymin;
    xmax = _xmax;
    ymax = _ymax;
    px_width = _px_width;
}

TileGridProfile::TileGridProfile( const TileGridProfile& rhs )
: xmin( rhs.xmin ),
  ymin( rhs.ymin ),
  xmax( rhs.xmax ),
  ymax( rhs.ymax ),
  px_width( rhs.px_width )
{
    //NOP
}

double
TileGridProfile::xMin() const {
    return xmin;
}

double
TileGridProfile::yMin() const {
    return ymin;
}

double
TileGridProfile::xMax() const {
    return xmax;
}

double
TileGridProfile::yMax() const {
    return ymax;
}

int 
TileGridProfile::pixelsPerTile() const {
    return px_width;
}