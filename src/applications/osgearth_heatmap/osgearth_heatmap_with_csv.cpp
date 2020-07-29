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

int
usage(const char* name)
{
    OE_NOTICE
        << "\nUsage: " << name << std::endl;
    return 0;
}


typedef std::unordered_map<unsigned short, unsigned short> CellIndex;

typedef std::unordered_map<osgEarth::TileKey, CellIndex> TileKeyMap;


static int numRead = 0;

auto wgs84 = osgEarth::Registry::instance()->getGlobalGeodeticProfile();

static TileKeyMap s_keys;

static unsigned int maxHitCount = 0;

void LoadCSV(const std::string& filename)
{
    std::cout << "Loading " << filename << std::endl;
    std::ifstream in(filename);
    std::string throwaway;
    std::getline(in, throwaway);

    std::array<std::string, 16> cells;

    std::string line;

    while (true)
    {
        std::getline(in, line);

        if (!in) break;

        std::istringstream stream(line);
        for (auto& cell : cells) {
            std::getline(stream, cell, ',');
        }

        if (cells[2].size() > 0 && cells[3].size() > 0)
        {
            double lat = std::stod(cells[2]);
            double lon = std::stod(cells[3]);
            ++numRead;
            if (numRead % 50000 == 0) std::cout << "Read " << numRead << std::endl;

            unsigned int maxLevel = 8;
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
    }
}

void PrintCSV(const std::string& filename)
{
    std::cout << "Loading " << filename << std::endl;
    std::ifstream in(filename);
    std::string throwaway;
    std::getline(in, throwaway);

    std::array<std::string, 16> cells;

    std::string line;

    while (true)
    {
        std::getline(in, line);

        if (!in) break;

        std::istringstream stream(line);
        for (auto& cell : cells) {
            std::getline(stream, cell, ',');
        }

        if (cells[2].size() > 0 && cells[3].size() > 0)
        {
            double lat = std::stod(cells[2]);
            double lon = std::stod(cells[3]);
            std::cout << lat << " " << lon << std::endl;
        }
    }
}

void WriteKeysHeatMap()
{
    unsigned int numKeys = s_keys.size();
    unsigned int numProcessed = 0;
    for (const auto& key : s_keys)
    {
        unsigned x, y;
        key.first.getTileXY(x, y);

        unsigned cols = 0, rows = 0;
        key.first.getProfile()->getNumTiles(key.first.getLevelOfDetail(), cols, rows);
        y = rows - y - 1;

        std::stringstream filename;
        filename << key.first.getLevelOfDetail() << "/" << x << "/" << y << ".png";

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
        float maxHeat = 100.0;
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
    /*

    double lon, lat;
    while (std::cin >> lon >> lat)
    {
        std::cout << "Read " << lon << ", " << lat << std::endl;
    }
    */

    // help?
    if (arguments.read("--help"))
        return usage(argv[0]);

    std::vector<std::string> files = {
        "D:/geodata/simdis/Heatmap_CSV_Data/ADS-B/states_2020-06-15-00.csv",
        "D:/geodata/simdis/Heatmap_CSV_Data/ADS-B/states_2020-06-15-01.csv",
        "D:/geodata/simdis/Heatmap_CSV_Data/ADS-B/states_2020-06-15-02.csv",
        "D:/geodata/simdis/Heatmap_CSV_Data/ADS-B/states_2020-06-15-03.csv",
        "D:/geodata/simdis/Heatmap_CSV_Data/ADS-B/states_2020-06-15-04.csv",
        "D:/geodata/simdis/Heatmap_CSV_Data/ADS-B/states_2020-06-15-05.csv",
        "D:/geodata/simdis/Heatmap_CSV_Data/ADS-B/states_2020-06-15-06.csv",
        "D:/geodata/simdis/Heatmap_CSV_Data/ADS-B/states_2020-06-15-07.csv",
        "D:/geodata/simdis/Heatmap_CSV_Data/ADS-B/states_2020-06-15-08.csv",
        "D:/geodata/simdis/Heatmap_CSV_Data/ADS-B/states_2020-06-15-09.csv",
        "D:/geodata/simdis/Heatmap_CSV_Data/ADS-B/states_2020-06-15-10.csv",
        "D:/geodata/simdis/Heatmap_CSV_Data/ADS-B/states_2020-06-15-11.csv",
        "D:/geodata/simdis/Heatmap_CSV_Data/ADS-B/states_2020-06-15-12.csv",
        "D:/geodata/simdis/Heatmap_CSV_Data/ADS-B/states_2020-06-15-13.csv",
        "D:/geodata/simdis/Heatmap_CSV_Data/ADS-B/states_2020-06-15-14.csv",
        "D:/geodata/simdis/Heatmap_CSV_Data/ADS-B/states_2020-06-15-15.csv",
        "D:/geodata/simdis/Heatmap_CSV_Data/ADS-B/states_2020-06-15-16.csv",
        "D:/geodata/simdis/Heatmap_CSV_Data/ADS-B/states_2020-06-15-17.csv",
        "D:/geodata/simdis/Heatmap_CSV_Data/ADS-B/states_2020-06-15-18.csv",
        "D:/geodata/simdis/Heatmap_CSV_Data/ADS-B/states_2020-06-15-19.csv",
        "D:/geodata/simdis/Heatmap_CSV_Data/ADS-B/states_2020-06-15-20.csv",
        "D:/geodata/simdis/Heatmap_CSV_Data/ADS-B/states_2020-06-15-21.csv",
        "D:/geodata/simdis/Heatmap_CSV_Data/ADS-B/states_2020-06-15-22.csv",
        "D:/geodata/simdis/Heatmap_CSV_Data/ADS-B/states_2020-06-15-23.csv"
    };
    auto startTime = std::chrono::high_resolution_clock::now();
    for (auto &f : files)
    {
        //LoadCSV(f);
        PrintCSV(f);
    }
    std::cout << "Read " << numRead << " from " << files.size() << " files" << std::endl;
    std::cout << "Number of keys created " << s_keys.size() << std::endl;

    WriteKeysHeatMap();
    std::cout << "Max hitcount " << maxHitCount << std::endl;

    auto endTime = std::chrono::high_resolution_clock::now();
    long long ms = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
    std::cout << "Complete processing in " << ms << " ms " << std::endl;
    return 0;
}