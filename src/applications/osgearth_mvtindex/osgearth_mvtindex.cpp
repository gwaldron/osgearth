/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2020 Pelican Mapping
* MIT License
*/

#include <osgEarth/TileKey>
#include <osgEarth/FeatureSource>
#include <osgEarth/FeatureCursor>
#include <osgEarth/OGRFeatureSource>
#include <osgEarth/Utils>
#include <osgEarth/MVT>

#include <iostream>

using namespace osgEarth;
using namespace osgEarth::Util;

std::map< unsigned int, std::set< TileKey > > levelsToTiles;

int
usage(const std::string& message)
{
    OE_WARN
        << "\n\n" << message
        << "\n\nUsage: osgearth_mvtindex osm-qa.mbtiles"
        << "\n"
        << "\n     --zoom                                       : The zoom level to read from the mvt database.  (default=12)"
        << "\n     --bounds [minLon] [minLat] [maxLon] [maxLat] : Limit the bounds of the tiles to iterate through."
        << "\n     --level [level]                              : The quadtree level of detail to generate the output index.  Default is the read zoom level.  Will accept multiple --level arguments"
        << "\n     --index [index]                              : The base filename of the output index shapefile.  (default=index)"
        << "\n     --include [attribute]                        : The attribute to search the features.  Will accept multiple --attribute arguments"
        << "\n"
        << std::endl;

    return -1;
}

static int complete = 0;

std::vector< std::string > attributes;

void processTile(const TileKey& key, const FeatureList& features, void* context)
{
    for (auto& f : features)
    {
        for (auto& attribute : attributes)
        {
            if (f->hasAttr(attribute))
            {
                for (auto& k : levelsToTiles)
                {
                    unsigned int level = k.first;

                    TileKey levelKey = key;
                    if (level == key.getLevelOfDetail())
                    {
                        k.second.insert(levelKey);
                    }
                    else if (level < key.getLevelOfDetail())
                    {
                        while (levelKey.getLevelOfDetail() != level)
                        {
                            levelKey.makeParent();
                        }
                        k.second.insert(levelKey);
                    }
                }

                ++complete;
                if (complete % 100 == 0)
                {
                    std::cout << std::fixed << "\rIndexed " << complete << " tiles" << std::flush;
                }
                return;
            }
        }
    }
}

int
main(int argc, char** argv)
{
    osg::ArgumentParser args(&argc, argv);

    osg::Timer_t startTime = osg::Timer::instance()->tick();

    std::string database;

    //Get the first argument that is not an option
    for (int pos = 1; pos < args.argc(); ++pos)
    {
        if (!args.isOption(pos))
        {
            database = args[pos];
            break;
        }
    }

    if (database.empty())
    {
        std::cout << "Please specify a MVT database to read from" << std::endl;
        return -1;
    }

    std::cout << "Reading tiles from " << database << std::endl;

    MVTFeatureSource* input = new MVTFeatureSource();
    input->setURL(database);
    if (input->open().isError())
    {
        std::cout << "Failed to open database " << database << ": " <<  input->getStatus().toString() << std::endl;
        return -1;
    }

    unsigned int zoomLevel = 12;
    args.read("--zoom", zoomLevel);

    GeoExtent queryExtent;
    double xmin = DBL_MAX, ymin = DBL_MAX, xmax = DBL_MIN, ymax = DBL_MIN;
    while (args.read("--bounds", xmin, ymin, xmax, ymax))
    {
        queryExtent = GeoExtent(osgEarth::SpatialReference::create("epsg:4326"), xmin, ymin, xmax, ymax);
    }

    std::string attribute;
    while (args.read("--include", attribute))
    {
        attributes.push_back(attribute);
    }

    if (attributes.empty())
    {
        std::cout << "Please specify at least one attribute to search for using --include" << std::endl;
        return -1;
    }

    std::string indexName = "index";
    args.read("--index", indexName);

    std::set< unsigned int > levels;

    unsigned int level;
    while (args.read("--level", level))
    {
        if (level > zoomLevel)
        {
            std::cout << "Level " << level << " cannot be greater than zoom level of " << zoomLevel << std::endl;
            return -1;
        }
        levels.insert(level);
    }

    if (levels.empty())
    {
        levels.insert(zoomLevel);
    }

    // Initialize the levels
    for (auto l : levels)
    {
        levelsToTiles[l] = std::set< TileKey >();
    }

    input->iterateTiles(zoomLevel, 0, 0, queryExtent, processTile, nullptr);

    std::cout << std::endl << "Indexing complete, writing index shapefiles" << std::endl << std::endl;

    for (auto& k : levelsToTiles)
    {
        // create output shapefile
        FeatureSchema outSchema;
        auto output = new OGRFeatureSource();
        output->setOGRDriver("ESRI Shapefile");
        std::stringstream filename;
        filename << indexName << "_" << k.first << ".shp";
        output->setURL(filename.str());

        if (output->create(input->getFeatureProfile(), outSchema, osgEarth::Geometry::TYPE_POLYGON, NULL).isError())
        {
            std::cout << output->getStatus().toString() << std::endl;
            return -1;
        }

        std::cout << std::endl << "Writing index " << filename.str() << std::endl << std::endl;

        unsigned int totalTiles = k.second.size();
        unsigned int writtenTiles = 0;
        for (auto& tile : k.second)
        {
            GeoExtent extent = tile.getExtent();
            Geometry* poly = new osgEarth::Polygon();
            poly->push_back(extent.xMin(), extent.yMin());
            poly->push_back(extent.xMax(), extent.yMin());
            poly->push_back(extent.xMax(), extent.yMax());
            poly->push_back(extent.xMin(), extent.yMax());
            osg::ref_ptr< Feature > feature = new Feature(poly, extent.getSRS());
            output->insertFeature(feature.get());
            writtenTiles++;
            if (writtenTiles % 100 == 0)
            {
                std::cout << "\rWrote " << writtenTiles << " of " << totalTiles << std::flush;
            }
        }
    }

    osg::Timer_t endTime = osg::Timer::instance()->tick();
    double time = osg::Timer::instance()->delta_s(startTime, endTime);

    std::cout << std::endl << "Completed tiling in " << osgEarth::prettyPrintTime(time) << std::endl;
    return 0;
}