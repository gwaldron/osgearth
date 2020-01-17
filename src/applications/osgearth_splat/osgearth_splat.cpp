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

#include <osgViewer/Viewer>

#include <osgEarth/MapNode>
#include <osgEarth/Registry>
#include <osgEarth/LandCoverLayer>

#include <osgEarthSplat/SplatLayer>
#include <osgEarthSplat/GroundCoverLayer>

#include <osgEarthDrivers/gdal/GDALOptions>

#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/EarthManipulator>

#include <osgEarthSymbology/BillboardSymbol>


#define LC "[splat] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Splat;
using namespace osgEarth::Drivers;
using namespace osgEarth::Symbology;

int
failed(const std::string& s) {
    OE_WARN << "FAILED: " << s << "\n";
    return -1;
}

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    bool fromXML = arguments.find("--xml") >= 0;

    // Create a land cover dictionary.
    LandCoverDictionary* dictionary;

    if (fromXML)
    {
        LandCoverDictionaryOptions options;
        if (options.loadFromXML("../data/land_cover_dictionary.xml") == false)
            return failed("Cannot find XML land cover dictionary");
        
        dictionary = new LandCoverDictionary(options);
    }
    else
    {
        dictionary = new LandCoverDictionary();
        dictionary->setName("Land Cover Dictionary");
        dictionary->addClass("forest");
        dictionary->addClass("cropland");
        dictionary->addClass("grassland");
        dictionary->addClass("savanna");
        dictionary->addClass("swamp");
        dictionary->addClass("desert");
        dictionary->addClass("rock");
        dictionary->addClass("water");    
        dictionary->addClass("tundra");
        dictionary->addClass("urban");
    }

    // Create the data source for our land cover data and
    // map each value to a class in the dictionary.
    // This example uses the ESA GLOBCOVER data set from
    // http://due.esrin.esa.int/page_globcover.php
    GDALOptions coverageDriver;
    coverageDriver.url() = "H:/data/esa/GLOBCOVER_L4_200901_200912_V2.3_Ant_tiled.tif"; 
    coverageDriver.profile() = ProfileOptions("global-geodetic");

    LandCoverCoverageLayerOptions coverage;
    coverage.driver() = coverageDriver;
    coverage.warp() = 0.035;
    if (fromXML)
    {
        if (coverage.loadMappingsFromXML("../data/land_cover_ESA_GLOBCOVER.xml") == false)
            return failed("Cannot find coverage mappings XML\n");
    }
    else
    {
        coverage.map(11, "cropland");
        coverage.map(14, "cropland");
        coverage.map(20, "cropland");
        coverage.map(30, "cropland");
        coverage.map(40, "forest");
        coverage.map(50, "forest");
        coverage.map(60, "forest");
        coverage.map(70, "forest");
        coverage.map(80, "forest");
        coverage.map(90, "forest");
        coverage.map(100, "forest");
        coverage.map(110, "grassland");
        coverage.map(120, "grassland");
        coverage.map(130, "savanna");
        coverage.map(140, "savanna");
        coverage.map(150, "savanna");
        coverage.map(160, "swamp");
        coverage.map(170, "swamp");
        coverage.map(180, "swamp");
        coverage.map(190, "urban");
        coverage.map(200, "desert");
        coverage.map(210, "water");
        coverage.map(220, "tundra");
        coverage.map(230, "water");
    }

    // Create the land cover layer for the map:
    LandCoverLayer* landCover = new LandCoverLayer();
    landCover->setName("LandCover");
    landCover->options().cachePolicy() = CachePolicy::NO_CACHE;
    landCover->options().coverages().push_back(coverage);
    landCover->options().maxDataLevel() = 15u;

    // Next, load the definitions that map land cover classes to actual textures.
    Surface* surface = new Surface();
    SplatCatalog* catalog = SplatCatalog::read("../data/splat/splat_catalog.xml");
    if (catalog == 0L)
        return failed("Reading splat catalog");
    surface->setCatalog(catalog);

    // The zone designates the geographic area over which to apply the surface.
    // At least one zone is required and by default it covers the entire map.
    Zone* splatZone = new Zone();
    splatZone->setSurface(surface);
    
    // Create an imagery splatting layer that uses the configured land cover.
    SplatLayer* splatLayer = new SplatLayer();
    splatLayer->setName("Splat imagery");
    splatLayer->options().cachePolicy() = CachePolicy::NO_CACHE;
    splatLayer->setLandCoverDictionary(dictionary);
    splatLayer->setLandCoverLayer(landCover);
    splatLayer->zones().push_back(splatZone);


    // Now, the trees:

    // Load a tree image and make a billboard symbol from it:
    osg::ref_ptr<osg::Image> tree = URI("../data/splat/pine2.png").getImage();
    if (tree.valid() == false)
        return failed("Loading tree image");

    BillboardSymbol* treeSymbol = new BillboardSymbol();
    treeSymbol->setSideImage(tree.get());
    treeSymbol->width() = 12.0f;
    treeSymbol->height() = 16.0f;

    // Add this symbol to a "forest" biome.
    GroundCoverBiomeOptions forestBiome;
    forestBiome.biomeClasses() = "forest";
    forestBiome.symbols().push_back(treeSymbol);
    
    // Assemble the ground cover coniguration:
    GroundCoverOptions treeOptions;
    treeOptions.biomes().push_back(forestBiome);
    treeOptions.maxDistance() = 15000.0;
    treeOptions.density() = 4.0;
    treeOptions.fill() = 0.85;
    treeOptions.brightness() = 2.0;
    treeOptions.contrast() = 1.0;
    GroundCover* trees = new GroundCover(treeOptions);

    Zone* treeZone = new Zone();
    treeZone->setGroundCover(trees);

    // Now, create a ground cover layer for some trees.
    GroundCoverLayer* treeLayer = new GroundCoverLayer();
    treeLayer->setName("Ground cover");
    treeLayer->options().lod() = 13;
    treeLayer->setLandCoverDictionary(dictionary);
    treeLayer->setLandCoverLayer(landCover);
    treeLayer->zones().push_back(treeZone);


    // Assemble the Map.
    Map* map = new Map();
    map->addLayer(dictionary);
    map->addLayer(landCover);
    map->addLayer(splatLayer);
    map->addLayer(treeLayer);

    // Activate the REX terrain engine (required for splatting)
    osgEarth::Registry::instance()->overrideTerrainEngineDriverName() = "rex";

    // create a viewer:
    osgViewer::Viewer viewer(arguments);
    viewer.getDatabasePager()->setUnrefImageDataAfterApplyPolicy( false, false );
    viewer.setCameraManipulator( new EarthManipulator(arguments) );
    viewer.setSceneData(new MapNode(map));
    return viewer.run();
}
