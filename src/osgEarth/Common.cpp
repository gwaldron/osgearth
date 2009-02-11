#include <osgEarth/Common>

using namespace osgEarth;


std::string osgEarth::toString<bool>(const bool value)
{
  return value ? "true":"false";
};