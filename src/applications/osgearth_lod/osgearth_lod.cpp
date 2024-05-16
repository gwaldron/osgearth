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
#define LC "[osgearth_lod] "

#include <osgEarth/Notify>
#include <osgEarth/LODGenerator>

#include <osg/ArgumentParser>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

using namespace osgEarth;

int
main(int argc, char** argv)
{
    osg::ArgumentParser args(&argc, argv);

    // Read the ouptut filename
    std::string outFilename = "out.osgb";
    args.read("--out", outFilename);

    // Read the levels from the command line, formatted like --level threshold,range,aggressive
    std::vector< LODGenerator::LODOptions> options;
    float threshold = 0.0f;
    float range = 0.0f;
    bool aggressive = false;
    while (args.read("--level", threshold, range, aggressive))
    {
        OE_NOTICE << "Adding level " << threshold << ", " << range << ", " << aggressive << std::endl;
        options.push_back({ threshold, range, aggressive });
    }

    osg::ref_ptr<osg::Node> root = osgDB::readRefNodeFiles(args);

    LODGenerator generator;
    osg::ref_ptr< osg::Node> result = generator.generateLODs(root.get(), options);

    osgDB::writeNodeFile(*result, outFilename);
    return 0;
}
