#include <osgEarth/TileGridProfile>

using namespace osgEarth;

TileGridProfile::TileGridProfile( double _xmin, double _ymin, double _xmax, double _ymax )
{
    xmin = _xmin;
    ymin = _ymin;
    xmax = _xmax;
    ymax = _ymax;
}

TileGridProfile::TileGridProfile( const TileGridProfile& rhs )
: xmin( rhs.xmin ),
  ymin( rhs.ymin ),
  xmax( rhs.xmax ),
  ymax( rhs.ymax )
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