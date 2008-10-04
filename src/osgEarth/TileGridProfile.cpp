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