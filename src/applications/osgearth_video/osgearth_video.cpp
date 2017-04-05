/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2016 Pelican Mapping
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
#include <osgEarth/Registry>
#include <osgEarth/ImageLayer>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/Controls>
#include <osgEarthSymbology/Color>
#include <osgEarthDrivers/tms/TMSOptions>
#include <osgEarthDrivers/wms/WMSOptions>
#include <osgEarthDrivers/gdal/GDALOptions>
#include <osg/ImageStream>

using namespace osgEarth;
using namespace osgEarth::Drivers;
using namespace osgEarth::Util;

class VideoLayer : public osgEarth::ImageLayer
{
public:

     VideoLayer(const std::string& filename)
     {         
         osg::Image* image = osgDB::readImageFile(filename);
         if (image)
         {             
             osg::ImageStream* is = dynamic_cast< osg::ImageStream*>( image );
             if (is)
             {
                 is->setLoopingMode(osg::ImageStream::LOOPING);
                 is->play();                 
             }

             _texture = new osg::Texture2D( image );
             _texture->setResizeNonPowerOfTwoHint( false );
             _texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture2D::LINEAR);
             _texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture2D::LINEAR);
             _texture->setWrap( osg::Texture::WRAP_S, osg::Texture::REPEAT );
             _texture->setWrap( osg::Texture::WRAP_T, osg::Texture::REPEAT );

         }

         setProfile(osgEarth::Registry::instance()->getGlobalGeodeticProfile());
     }

     virtual bool createTextureSupported() const { return true; }
     
     virtual osg::Texture* createTexture(const TileKey& key, ProgressCallback* progress, osg::Matrixf& textureMatrix)
     {
         if (key.getLOD() > 0) return 0;

         bool flip = _texture->getImage()->getOrigin()==osg::Image::TOP_LEFT;
         osg::Matrixf scale = osg::Matrixf::scale(0.5, flip? -1.0 : 1.0, 1.0);         

         if (key.getTileX() == 0)
         {
             textureMatrix = scale;
         }
         else if (key.getTileX() == 1)
         {
             textureMatrix =  scale * osg::Matrixf::translate(0.5, 0.0, 0.0);
         }

         return _texture.get();
     }

     osg::ref_ptr< osg::Texture2D > _texture;
};

/**
 * How to create a simple osgEarth map and display it.
 */
int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    // create the empty map.
    Map* map = new Map();

    // add a TMS imagery layer:
    TMSOptions imagery;
    imagery.url() = "http://readymap.org/readymap/tiles/1.0.0/22/";
    map->addLayer( new ImageLayer("ReadyMap Imagery", imagery) );

    // add a TMS elevation layer:
    TMSOptions elevation;
    elevation.url() = "http://readymap.org/readymap/tiles/1.0.0/116/";
    map->addLayer( new ElevationLayer("ReadyMap Elevation", elevation) );
   
    // Load command line arguments as videos.
    for(int pos=1;pos<arguments.argc();++pos)
    {
        if (!arguments.isOption(pos))
        {
            std::string filename = arguments[ pos ];
            OE_NOTICE << "Loading " << filename << std::endl;
            map->addLayer(new VideoLayer(filename));
        }
    }

    // make the map scene graph:
    MapNode* node = new MapNode( map );

    // initialize a viewer:
    osgViewer::Viewer viewer(arguments);
    viewer.setCameraManipulator( new EarthManipulator );
    viewer.setSceneData( node );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgViewer::ThreadingHandler());
    viewer.addEventHandler(new osgViewer::LODScaleHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    viewer.addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));

    return viewer.run();
}