/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2020 Pelican Mapping
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
#include <osgEarth/Notify>
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/MapNode>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/Metrics>
#include <osgEarth/Lighting>
#include <osgEarth/NodeUtils>
#include <osgEarth/PhongLightingEffect>

#include <osgEarthProcedural/BiomeLayer>
#include <osgEarthProcedural/BiomeManager>
#include <osgEarthProcedural/LifeMapLayer>
#include <osgEarthProcedural/VegetationLayer>

#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/Uniform>
#include <iostream>

#define LC "[osgearth_biome] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Procedural;

int
usage(const char* name)
{
    OE_NOTICE
        << "\nUsage: " << name
        << "\n   --encode-texture [filename]"
        << std::endl;
    return 0;
}

int
encodeTexture(osg::ArgumentParser& args)
{
    std::string infile;
    if (args.read("--encode-texture", infile))
    {
        std::string libName = osgDB::Registry::instance()->createLibraryNameForNodeKit("osgEarthProcedural");
        osgDB::Registry::LoadStatus status = osgDB::Registry::instance()->loadLibrary(libName);

        osg::ref_ptr<osg::Image> image;

        image = osgDB::readRefImageFile(infile + ".oe_splat_rgbh");
        if (image.valid())
            osgDB::writeImageFile(*image.get(), infile + ".oe_splat_rgbh");

        image = osgDB::readRefImageFile(infile + ".oe_splat_nnra");
        if (image.valid())
            osgDB::writeImageFile(*image.get(), infile + ".oe_splat_nnra");
    }
    return 0;
}

int
main(int argc, char** argv)
{
    osgEarth::initialize();

    osg::ArgumentParser arguments(&argc, argv);

    if (arguments.find("--encode-texture") >= 0)
        return encodeTexture(arguments);
    
    return usage(argv[0]);
}
