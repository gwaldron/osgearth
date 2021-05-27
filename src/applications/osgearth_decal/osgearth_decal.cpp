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

#include <osgViewer/Viewer>
#include <osgEarth/Notify>
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/MapNode>
#include <osgEarth/Threading>
#include <osgEarth/DecalLayer>
#include <osgEarth/ImageUtils>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/Random>
#include <osgEarth/Elevation>
#include <osgDB/ReadFile>
#include <iostream>

#define LC "[decal] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Contrib;

int
usage(const char* name)
{
    OE_NOTICE
        << "\nUsage: " << name << " file.earth" << std::endl
        << MapNodeHelper().usage() << std::endl;

    return 0;
}

struct CraterRenderer
{
    static void render(
        const GeoPoint& center,
        const Distance& radius,
        GeoExtent& out_extent,
        osg::ref_ptr<osg::Image>& out_elevation,
        osg::ref_ptr<osg::Image>& out_lifemap)
    {
        out_extent = GeoExtent(center.getSRS());
        out_extent.expandToInclude(center.x(), center.y());
        out_extent.expand(radius*2.0, radius*2.0);

        out_elevation = new osg::Image();
        out_elevation->allocateImage(
            ELEVATION_TILE_SIZE,
            ELEVATION_TILE_SIZE,
            1,
            GL_RED,
            GL_UNSIGNED_BYTE);

        ImageUtils::PixelWriter writeElevation(out_elevation.get());
        ImageUtils::ImageIterator e_iter(writeElevation);
        osg::Vec4 value;
        e_iter.forEachPixel([&]()
            {
                float a = (e_iter.u() - 0.5f);
                float b = (e_iter.v() - 0.5f);
                float d = sqrt((a*a) + (b*b));
                value.r() = clamp(d, 0.0f, 0.5f);
                writeElevation(value, e_iter.s(), e_iter.t());
            }
        );

        out_lifemap = new osg::Image();
        out_lifemap->allocateImage(
            256,
            256,
            1,
            GL_RGBA,
            GL_UNSIGNED_BYTE);

        ImageUtils::PixelWriter writeLifeMap(out_lifemap.get());
        ImageUtils::ImageIterator lm_iter(writeLifeMap);
        lm_iter.forEachPixel([&]()
            {
                float a = 2.0f*(lm_iter.u() - 0.5f);
                float b = 2.0f*(lm_iter.v() - 0.5f);
                float d = clamp(sqrt((a*a) + (b*b)), 0.0f, 1.0f);
                value.set(0.0f, 0.0f, 0.85f, 1.0f - (d*d));
                writeLifeMap(value, lm_iter.s(), lm_iter.t());
            }
        );
    }
};

struct App
{
    unsigned _minLevel;
    float _size;

    osg::ref_ptr<MapNode> _mapNode;
    osg::ref_ptr<DecalImageLayer> _imageLayer;
    osg::ref_ptr<DecalElevationLayer> _elevLayer;
    osg::ref_ptr<DecalLandCoverLayer> _landCoverLayer;
    osg::ref_ptr<DecalImageLayer> _lifemapLayer;
    osg::ref_ptr<osg::Image> _image_for_elev;
	osg::ref_ptr<osg::Image> _image_for_rgb;
    osg::ref_ptr<osg::Image> _landCover;
    std::stack<std::string> _undoStack;
    unsigned _idGenerator;
    std::vector<const Layer*> _layersToRefresh;

    App()
    {
        _idGenerator = 0u;
        _size = 0.0f;

        // Needs to be less than or equal to the underlying terrain
        // data's max LOD, or else the normal maps won't update.
        // I do not yet know why that is -gw
        _minLevel = 10u;
    }

    void init(MapNode* mapNode)
    {
        _mapNode = mapNode;

#if 0
        _imageLayer = new DecalImageLayer();
        _imageLayer->setName("Image Decals");
        _imageLayer->setMinLevel(_minLevel);
        _imageLayer->setOpacity(0.95f);
        mapNode->getMap()->addLayer(_imageLayer.get());
        _layersToRefresh.push_back(_imageLayer.get());
#endif

        _elevLayer = new DecalElevationLayer();
        _elevLayer->setName("Elevation Decals");
        _elevLayer->setMinLevel(_minLevel);
        mapNode->getMap()->addLayer(_elevLayer.get());
        _layersToRefresh.push_back(_elevLayer.get());

        _lifemapLayer = new DecalImageLayer();
        _lifemapLayer->setName("LifeMap Decals");
        _lifemapLayer->setMinLevel(_minLevel);

        // If there is a layer called "Life Map" append to it as a Post,
        // otherwise standalone decal.
        ImageLayer* lm = mapNode->getMap()->getLayerByName<ImageLayer>("Life Map");
        if (lm)
        {
            lm->addPostLayer(_lifemapLayer.get());
            _layersToRefresh.push_back(lm);
        }
        else
        {
            mapNode->getMap()->addLayer(_lifemapLayer.get());
            _layersToRefresh.push_back(_lifemapLayer.get());
        }
    }

    void addCrater(const GeoPoint& center, const Distance& radius)
    {
        if (!center.isValid())
            return;

        GeoExtent extent;
        osg::ref_ptr<osg::Image> elevation;
        osg::ref_ptr<osg::Image> lifemap;

        CraterRenderer::render(
            center,
            radius,
            extent,
            elevation,
            lifemap);

        // ID for the new decal(s). ID's need to be unique in a single decal layer,
        // but the three different TYPES of layers can share the same ID
        std::string id = Stringify() << _idGenerator++;
        _undoStack.push(id);

        OE_NOTICE << "Adding crater # " << id << std::endl;

        if (_elevLayer.valid())
        {
            _elevLayer->addDecal(id, extent, elevation.get(), -25.0, 25.0, GL_RED);
        }

        if (_lifemapLayer.valid())
        {
            _lifemapLayer->addDecal(id, extent, lifemap.get());
        }

        // Tell the terrain engine to regenerate the effected area.
        _mapNode->getTerrainEngine()->invalidateRegion(
            _layersToRefresh, 
            extent, 
            _minLevel, 
            INT_MAX);
    }

    void undoLastAdd()
    {
        if (!_undoStack.empty())
        {
            std::string id = _undoStack.top();
            _undoStack.pop();

            OE_NOTICE << "Undo-ing bomb #" << id << std::endl;

            GeoExtent extent;

            if (_imageLayer.valid())
            {
                extent.expandToInclude(_imageLayer->getDecalExtent(id));
                _imageLayer->removeDecal(id);
            }

            if (_elevLayer.valid())
            {
                extent.expandToInclude(_elevLayer->getDecalExtent(id));
                _elevLayer->removeDecal(id);
            }

            if (_landCoverLayer.valid())
            {
                extent.expandToInclude(_landCoverLayer->getDecalExtent(id));
                _landCoverLayer->removeDecal(id);
            }

            if (_lifemapLayer.valid())
            {
                extent.expandToInclude(_lifemapLayer->getDecalExtent(id));
                _lifemapLayer->removeDecal(id);
            }

            _mapNode->getTerrainEngine()->invalidateRegion(
                _layersToRefresh, 
                extent, 
                _minLevel, 
                INT_MAX);
        }
    }

    void reset()
    {
        OE_NOTICE << "Starting over" << std::endl;

        if (_imageLayer.valid())
        {
            _imageLayer->clearDecals();
        }

        if (_elevLayer.valid())
        {
            _elevLayer->clearDecals();
        }

        if (_landCoverLayer.valid())
        {
            _landCoverLayer->clearDecals();
        }

        if (_lifemapLayer.valid())
        {
            _lifemapLayer->clearDecals();
        }

        _mapNode->getTerrainEngine()->invalidateRegion(
            _layersToRefresh,
            GeoExtent::INVALID,
            _minLevel,
            INT_MAX);
    }
};

int
main(int argc, char** argv)
{
	osgEarth::initialize();
    osg::ArgumentParser arguments(&argc, argv);

    // help?
    if (arguments.read("--help"))
        return usage(argv[0]);

    // create a viewer:
    osgViewer::Viewer viewer(arguments);
    viewer.setCameraManipulator(new EarthManipulator(arguments));

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if (node)
    {
        App app;

        app._size = 100.0f;
        arguments.read("--size", app._size);

        app.init(MapNode::get(node));
        viewer.setSceneData(node);

        EventRouter& ui = EventRouter::get(&viewer);

        // Press 'D' to drop a crater under the mouse
        ui.onKeyPress(ui.KEY_D, [&](osg::View* view, float x, float y)
            {
                app.addCrater(
                    app._mapNode->getGeoPointUnderMouse(view, x, y),
                    Distance(app._size, Units::METERS));
            });

        // Press 'U' to undo the last crater
        ui.onKeyPress(ui.KEY_U, [&]() { app.undoLastAdd(); });

        // Press 'C' to clear all craters
        ui.onKeyPress(ui.KEY_C, [&]() { app.reset(); });

        OE_NOTICE << LC << 
            "\n\n-- Zoom in close ..."
            "\n-- Press 'd' to drop bombs"
            "\n-- Press 'u' to undo last drop"
            "\n-- Press 'c' to start over"
            << std::endl;

        return viewer.run();
    }
    else
    {
        return usage(argv[0]);
    }

    return 0;
}