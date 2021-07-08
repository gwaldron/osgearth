#include "eb_shaders.h"

Bruneton::Shaders::Shaders()
{
// library functions used by the compute and rendering shaders
#include "eb_function_shaders.cpp"

// compute shaders
#include "eb_compute_shaders.cpp"

// osgEarth shaders
#include "eb_osgearth_shaders.cpp"
}

