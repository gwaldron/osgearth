/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2018 Pelican Mapping
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
#include <osgEarth/ThreadingUtils>
#include <osgEarth/DecalLayer>
#include <osgEarth/ImageUtils>
#include <osgEarth/TerrainEngineNode>
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
    unsigned _maxDataLevel;
    float _size;

    osg::ref_ptr<MapNode> _mapNode;
    osg::ref_ptr<DecalImageLayer> _imageLayer;
    osg::ref_ptr<DecalElevationLayer> _elevLayer;
    osg::ref_ptr<DecalLandCoverLayer> _landCoverLayer;
    osg::ref_ptr<osg::Image> _image;
    osg::ref_ptr<osg::Image> _landCover;

    App()
    {
        _image = osgDB::readRefImageFile("../data/burn.png");
        if ( !_image.valid())
        {
            OE_WARN << "Failed to load decal image!" << std::endl;
            return;
        }

        _minLevel = 11u;
        _maxDataLevel = 18u; // must be >= other land cover layers!

        _landCover = new osg::Image();
        _landCover->allocateImage(_image->s(), _image->t(), 1, GL_RED, GL_FLOAT);
        _landCover->setInternalTextureFormat(GL_R16F);

        ImageUtils::PixelReader read(_image.get());

        ImageUtils::PixelWriter write(_landCover.get());
        write.setNormalize(false);

        osg::Vec4 value;

        for(unsigned t=0; t<_image->t(); ++t)
        {
            for(unsigned s=0; s<_image->s(); ++s)
            {
                read(value, s, t);
                float code = value.a() > 0.2? 7.0 : NO_DATA_VALUE;
                value.set(code,code,code,code);
                write(value, s, t);
            }
        }
        
    }

    void init(MapNode* mapNode)
    {        
        _mapNode = mapNode;

        // By default, decal layers will NOT cache. Here we are setting up a policy
        // to cache them with a 10-minute expiration.
        CachePolicy policy;
        //policy.usage() = CachePolicy::USAGE_READ_WRITE;
        //policy.maxAge() = 10.0;
        
        _imageLayer = new DecalImageLayer();
        _imageLayer->setMinLevel(_minLevel);
        _imageLayer->setMaxDataLevel(_maxDataLevel);
        _imageLayer->setCachePolicy(policy);
        mapNode->getMap()->addLayer(_imageLayer.get());

        _elevLayer = new DecalElevationLayer();
        _elevLayer->setMinLevel(_minLevel);
        _elevLayer->setMaxDataLevel(_maxDataLevel);
        _elevLayer->setCachePolicy(policy);
        mapNode->getMap()->addLayer(_elevLayer.get());

        _landCoverLayer = new DecalLandCoverLayer();
        _landCoverLayer->setMinLevel(_minLevel);
        _landCoverLayer->setMaxDataLevel(_maxDataLevel); // must be the same as other land cover layer(s)!
        _landCoverLayer->setCachePolicy(policy);
        mapNode->getMap()->addLayer(_landCoverLayer.get());
    }

    void addDecal(const GeoExtent& extent)
    {
        _imageLayer->addDecal(extent, _image.get());
        _elevLayer->addDecal(extent, _image.get(), -_size/20.0f); //-35.0f);
        _landCoverLayer->addDecal(extent, _landCover.get());
        _mapNode->getTerrainEngine()->invalidateRegion(extent, _minLevel, _maxDataLevel);
    }
};


struct ClickToDecal : public osgGA::GUIEventHandler
{
    App _app;
    ClickToDecal(App& app) : _app(app) { }

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        if (ea.getEventType() == ea.KEYDOWN && ea.getKey() == 'd')
        {
            osg::Vec3d world;
            _app._mapNode->getTerrain()->getWorldCoordsUnderMouse(aa.asView(), ea.getX(), ea.getY(), world);

            GeoPoint mapPoint;
            mapPoint.fromWorld(_app._mapNode->getMapSRS(), world);

            double t = aa.asView()->getFrameStamp()->getReferenceTime();
            t = 500.0 + 250.0 * (t - (long)t);
            
            mapPoint.transformInPlace(SpatialReference::get("spherical-mercator"));

            float d = _app._size * 0.5;

            GeoExtent extent(
                mapPoint.getSRS(),
                mapPoint.x()-d,
                mapPoint.y()-d,
                mapPoint.x()+d,
                mapPoint.y()+d);

            _app.addDecal(extent);

            return true;
        }
        return false;
    }
};

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    // help?
    if ( arguments.read("--help") )
        return usage(argv[0]);

    // create a viewer:
    osgViewer::Viewer viewer(arguments);
    viewer.getDatabasePager()->setUnrefImageDataAfterApplyPolicy( true, false );
    viewer.setCameraManipulator( new EarthManipulator(arguments) );

    App app;

    app._size = 250.0f;
    arguments.read("--size", app._size);

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if ( node )
    {
        viewer.setSceneData( node );

        app.init(MapNode::get(node));
        viewer.addEventHandler(new ClickToDecal(app));
        OE_NOTICE << LC << "Press 'd' to drop the bomb." << std::endl;

        return viewer.run();
    }
    else
    {
        return usage(argv[0]);
    }

    return 0;
}