/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2019 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include <osg/Notify>
#include <osgGA/GUIEventHandler>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/MapNode>
#include <osgEarth/ImageLayer>
#include <osgEarth/SimplexNoise>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/Controls>
#include <osgEarthSymbology/Color>
#include <osgEarthDrivers/tms/TMSOptions>
#include <osgEarthDrivers/wms/WMSOptions>
#include <osgEarthDrivers/gdal/GDALOptions>
#include <osgDB/WriteFile>
#include <osg/Image>

using namespace osgEarth;
using namespace osgEarth::Drivers;
using namespace osgEarth::Util;

int usage()
{
    OE_WARN << "\n"
        "osgearth_noisegen --size n             ; image dimension\n"
        "                  --out string         ; output filename\n"
        "                  [--frequency n]      ; default = 16\n"
        "                  [--octaves n]        ; default = 12\n"
        << std::endl;
    return -1;
}
/**
 * How to create a simple osgEarth map and display it.
 */
int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    int dim;
    if (!arguments.read("--size", dim))
        return usage();

    std::string out;
    if (!arguments.read("--out", out))
        return usage();

    double freq = 16.0;
    arguments.read("--frequency", freq);

    int octaves = 12;
    arguments.read("--octaves", octaves);

    SimplexNoise noise;
    noise.setFrequency(freq);
    noise.setOctaves(octaves);
    noise.setNormalize(true);
    osg::Image* image = noise.createSeamlessImage(dim);
    osgDB::writeImageFile(*image, out);
    return 0;
}
