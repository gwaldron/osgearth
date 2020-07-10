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

struct App
{
    unsigned _minLevel;
    float _size;

    osg::ref_ptr<MapNode> _mapNode;
    osg::ref_ptr<DecalImageLayer> _imageLayer;
    osg::ref_ptr<DecalElevationLayer> _elevLayer;
    osg::ref_ptr<DecalLandCoverLayer> _landCoverLayer;
    osg::ref_ptr<osg::Image> _image_for_elev;
	osg::ref_ptr<osg::Image> _image_for_rgb;
    osg::ref_ptr<osg::Image> _landCover;
    std::stack<std::string> _undoStack;
    unsigned _idGenerator;
    unsigned _decalsPerClick;
    std::vector<const Layer*> _layersToRefresh;

    App()
    {
        _image_for_elev = osgDB::readRefImageFile("../data/burn.png");
        if (!_image_for_elev.valid())
        {
            OE_WARN << "Failed to load elev decal image!" << std::endl;
            return;
        }
		
		_image_for_rgb = osgDB::readRefImageFile("../data/crater.png");
		if (!_image_for_rgb.valid())
		{
			OE_WARN << "Failed to load rgb decal image!" << std::endl;
			return;
		}
        _idGenerator = 0u;
        _minLevel = 11u;
        _size = 0.0f;
        _decalsPerClick = 1u;
    }

    void init(MapNode* mapNode)
    {
        _mapNode = mapNode;

        // read from the visible image:
        ImageUtils::PixelReader read(_image_for_rgb.get());
        osg::Vec4 value;

        // Only activate the land cover decal if there's a dictionary in the map:
		LandCoverDictionary* dic = mapNode->getMap()->getLayer<LandCoverDictionary>();
        if (dic)
        {
            // Synthesize a land cover raster to use as a decal and for masking trees & grass
            const LandCoverClass* lc_class = dic->getClassByName("rock");
            if (lc_class)
            {
                // write to the landcover raster:
                _landCover = LandCover::createImage(_image_for_rgb->s(), _image_for_rgb->t());
                ImageUtils::PixelWriter write(_landCover.get());

                const float lc_code = (float)lc_class->getValue();

                for (int t = 0; t < read.t(); ++t)
                {
                    for (int s = 0; s < read.s(); ++s)
                    {
                        read(value, s, t);
						float c = value.a() >= 0.1 ? lc_code : NO_DATA_VALUE;
                        value.set(c, c, c, c);
                        write(value, s, t);
                    }
                }
            }
        }

        _imageLayer = new DecalImageLayer();
        _imageLayer->setName("Image Decals");
        _imageLayer->setMinLevel(_minLevel);
        _imageLayer->setOpacity(0.95f);
        mapNode->getMap()->addLayer(_imageLayer.get());
        _layersToRefresh.push_back(_imageLayer.get());

        _elevLayer = new DecalElevationLayer();
        _elevLayer->setName("Elevation Decals");
        _elevLayer->setMinLevel(_minLevel);
        mapNode->getMap()->addLayer(_elevLayer.get());
        _layersToRefresh.push_back(_elevLayer.get());


        if (_landCover.valid())
        {
            _landCoverLayer = new DecalLandCoverLayer();
            _landCoverLayer->setName("LandCover Decals");
            _landCoverLayer->setMinLevel(_minLevel);
            mapNode->getMap()->addLayer(_landCoverLayer.get());
            _layersToRefresh.push_back(_landCoverLayer.get());
        }

    }

    void addDecal(const GeoExtent& extent)
    {
        // ID for the new decal(s). ID's need to be unique in a single decal layer,
        // but the three different TYPES of layers can share the same ID
        std::string id = Stringify() << _idGenerator++;
        _undoStack.push(id);

        OE_NOTICE << "Dropping bomb #" << id << std::endl;

        if (_imageLayer.valid())
        {
            _imageLayer->addDecal(
                id, 
                extent, 
                _image_for_rgb.get());
        }

        if (_elevLayer.valid())
        {
            _elevLayer->addDecal(
                id, 
                extent, 
                _image_for_elev.get(), 
                _size / 15.0f, -_size / 15.0f,  // min value (0.0), max value(1.0)
                GL_ALPHA);
        }


		if (_landCoverLayer.valid())
        {
           _landCoverLayer->addDecal(
               id, 
               extent, 
               _landCover.get());
        }


        // Tell the terrain engine to regenerate the effected area.
        _mapNode->getTerrainEngine()->invalidateRegion(_layersToRefresh, extent, _minLevel, INT_MAX);
    }


    void undoLastAdd()
    {
        for(unsigned i=0; !_undoStack.empty() && i<_decalsPerClick; ++i)
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


            _mapNode->getTerrainEngine()->invalidateRegion(_layersToRefresh, extent, _minLevel, INT_MAX);
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

        _mapNode->getTerrainEngine()->invalidateRegion(_layersToRefresh, GeoExtent::INVALID, _minLevel, INT_MAX);
    }
};


struct ClickToDecal : public osgGA::GUIEventHandler
{
    App _app;
    Random rng;
    ClickToDecal(App& app) : _app(app) { }

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        if (ea.getEventType() == ea.KEYDOWN && ea.getKey() == 'd')
        {
            osg::Vec3d world;
            if (!_app._mapNode->getTerrain()->getWorldCoordsUnderMouse(aa.asView(), ea.getX(), ea.getY(), world))
            {
                OE_WARN << LC << "No intersection under mouse." << std::endl;
                return false;
            }

            GeoPoint mapPoint;
            mapPoint.fromWorld(_app._mapNode->getMapSRS(), world);

            double t = aa.asView()->getFrameStamp()->getReferenceTime();
            t = 500.0 + 250.0 * (t - (long)t);

            mapPoint.transformInPlace(SpatialReference::get("spherical-mercator"));

            for (unsigned i = 0; i < _app._decalsPerClick; ++i)
            {
                float d = _app._size * 0.5;

                float offsetx = 0.0f, offsety = 0.0f;
                if (i > 0)
                {
                    offsetx = 2.0*(rng.next()-0.5) * _app._size*2.0;
                    offsety = 2.0*(rng.next()-0.5) * _app._size*2.0;
                }

                GeoExtent extent(
                    mapPoint.getSRS(),
                    offsetx + mapPoint.x() - d,
                    offsety + mapPoint.y() - d,
                    offsetx + mapPoint.x() + d,
                    offsety + mapPoint.y() + d);

                _app.addDecal(extent);
            }

            return true;
        }

        else if (ea.getEventType() == ea.KEYDOWN && ea.getKey() == 'u')
        {
            _app.undoLastAdd();
            return true;
        }

        else if (ea.getEventType() == ea.KEYDOWN && ea.getKey() == 'c')
        {
            _app.reset();
            return true;
        }

        return false;
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

        app._size = 250.0f;
        arguments.read("--size", app._size);

        app._decalsPerClick = 1u;
        arguments.read("--count", app._decalsPerClick);

        app.init(MapNode::get(node));

        viewer.setSceneData(node);
        viewer.addEventHandler(new ClickToDecal(app));

        OE_WARN << LC << 
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