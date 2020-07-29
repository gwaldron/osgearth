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
#include <osgEarth/Threading>
#include <osgEarth/Registry>
#include <osgEarth/TileKey>
#include <osgDB/WriteFile>
#include <osgEarth/FileUtils>
#include <iostream>
#include <vector>
#include <string>
#include <array>

#include "heatmap.h"

using namespace osgEarth;

int usage(const char* name)
{
    std::cout
        << "Generates a heatmap tiled dataset from a series of points.\n\n"
        << "osgearth_heatmap < points.txt  where points.txt contains a series of lat lon points separated by a space"
        << name
        << "\n    --max-level level            : The maximum level to generate to."
        << "\n    --max-heat maxHeat           : The maximum heat value to scale the color ramp to."
        << "\n    --format format              : The format to write image tiles to.  Default png."
        << std::endl;

    return 0;
}


typedef std::unordered_map<unsigned short, unsigned short> CellIndex;
typedef std::unordered_map<osgEarth::TileKey, CellIndex> TileKeyMap;

static int numRead = 0;

auto wgs84 = osgEarth::Registry::instance()->getGlobalGeodeticProfile();

static TileKeyMap s_keys;

static unsigned int maxHitCount = 0;

unsigned int maxLevel = 8;
float maxHeat = 100.0;
std::string format = "png";

inline void addPoint(double lon, double lat)
{
    for (unsigned int level = 0; level <= maxLevel; ++level)
    {
        TileKey key = wgs84->createTileKey(lon, lat, level);

        GeoExtent extent = key.getExtent();

        unsigned int x = osg::clampBetween((unsigned int)(256.0 * (lon - extent.xMin()) / extent.width()), 0u, 255u);
        unsigned int y = osg::clampBetween((unsigned int)(256.0 * (lat - extent.yMin()) / extent.height()), 0u, 255u);
        unsigned short index = (unsigned short)(y * 256 + x);
        s_keys[key][index]++;
    }
}

void ReadFile(std::istream& in)
{
    double lat, lon;
    while (in >> lat >> lon)
    {
        ++numRead;
        if (numRead % 50000 == 0) std::cout << "Read " << numRead << std::endl;

        addPoint(lon, lat);
    }
}

void WriteKeys()
{
    unsigned int numKeys = s_keys.size();
    unsigned int numProcessed = 0;
    for (const auto& key : s_keys)
    {
        unsigned x, y;
        key.first.getTileXY(x, y);

        std::stringstream filename;
        filename << key.first.getLevelOfDetail() << "/" << x << "/" << y << "." << format;

        // Create the heatmap object with the given dimensions (in pixel).
        heatmap_t* hm = heatmap_new(256, 256);

        for (auto& cell : key.second)
        {
            unsigned int index = cell.first;

            unsigned short r = index / 256;
            unsigned short c = index % 256;
            auto hitCount = cell.second;
            if (hitCount > maxHitCount) maxHitCount = hitCount;

            // Add the point but weighted to the number of hits in the cell to get a nice clustered view of the points of the lower levels.
            heatmap_add_weighted_point(hm, c, r, hitCount);
        }

        std::vector<unsigned char> imageData(256 * 256 * 4);
        heatmap_render_saturated_to(hm, heatmap_cs_default, maxHeat, &imageData[0]);
        heatmap_free(hm);

        osg::ref_ptr< osg::Image > image = new osg::Image;
        image->setImage(256, 256, 1, GL_RGB8, GL_RGBA, GL_UNSIGNED_BYTE, &imageData[0], osg::Image::NO_DELETE);

        osgEarth::makeDirectoryForFile(filename.str());
        osgDB::writeImageFile(*image.get(), filename.str());

        ++numProcessed;
        if (numProcessed % 100 == 0)
        {
            std::cout << "Processed " << numProcessed << " of " << numKeys << " keys" << std::endl;
        }
    }
}

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc, argv);

    // help?
    if (arguments.read("--help"))
        return usage(argv[0]);

    auto startTime = std::chrono::high_resolution_clock::now();

    arguments.read("--max-level", maxLevel);
    arguments.read("--max-heat", maxHeat);
    arguments.read("--format", format);

    std::cout << "Generating heatmap to max level of " << maxLevel << " with max heat " << maxHeat << std::endl;

    ReadFile(std::cin);

    std::cout << "Read " << numRead << " points" << std::endl;

    // Write out all the keys
    std::cout << "Writing " << s_keys.size() << " keys" << std::endl;
    WriteKeys();
    std::cout << "Max hitcount " << maxHitCount << std::endl;

    auto endTime = std::chrono::high_resolution_clock::now();
    long long ms = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
    std::cout << "Complete in " << ms << " ms " << std::endl;

    return 0;
}