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
#include <osg/Image>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarth/ImageLayer>
#include <osgEarth/Registry>
#include <osgEarthSymbology/Geometry>
#include <osgEarthSymbology/GeometryRasterizer>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Symbology;

/**
 * This sample demonstrates how to create a custom TileSource.
 */

static osg::Vec4 colors[4] = {
    osg::Vec4(1,0,0,1),
    osg::Vec4(0,1,0,1),
    osg::Vec4(0,0,1,1),
    osg::Vec4(0,0,0,1)
};

/**
 * Our homemade TileSource.
 */
class CustomTileSource : public TileSource
{
public:
    // Constructor that takes the user-provided options.
    CustomTileSource() : TileSource(TileSourceOptions())
    {
        // Create a shape that we will use to render tile images.
        _geom = new Ring();
        _geom->push_back( osg::Vec3(5, 5, 0) );
        _geom->push_back( osg::Vec3(250, 5, 0) );
        _geom->push_back( osg::Vec3(250, 250, 0) );
        _geom->push_back( osg::Vec3(5, 250, 0) );
    }

    // Called by the terrain engine when a layer using this driver is first added.
    Status initialize(const osgDB::Options* dbOptions)
    {
        if ( !getProfile() )
        {
            // Set the profile for this tile source. The profile defines the 
            // tiling scheme native to this tile source. The terrain engine will
            // call createImage or createHeightField with TileKeys according to
            // the profile you set here.
            setProfile( Registry::instance()->getGlobalGeodeticProfile() );

            // Create custom data extents. This is optional, but giving the terrain
            // engine information about the extents of your dataset will improve
            // performance in most cases. In this case, the data covers the
            // entire profile, but we want to tell the terrain engine that this
            // tile source only has data up to LOD 15:
            getDataExtents().push_back(DataExtent(getProfile()->getExtent(), 0u, 15u));
        }
        return STATUS_OK;
    }

    // Tells the layer not to cache data from this tile source.
    // Overriding this function is optional - by default it will inherit the 
    // caching policy from the Layer.
    CachePolicy getCachePolicyHint(const Profile* profile) const 
    {
        return CachePolicy::NO_CACHE;
    }

    // Define this method to return an image corresponding to the given TileKey.
    osg::Image* createImage( const TileKey& key, ProgressCallback* progress )
    {
        GeometryRasterizer rasterizer( 256, 256 );
        rasterizer.draw( _geom.get(), colors[key.getLevelOfDetail() % 4] );
        return rasterizer.finalize();
    }

    osg::ref_ptr<Ring> _geom;
};


int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osgViewer::Viewer viewer(arguments);

    // Start by creating an empty map:
    Map* map = new Map();

    // Create out image layer with a custom tile source.
    CustomTileSource* tileSource = new CustomTileSource();

    // Open the tile source. If you don't do this, the Map will automatically try to 
    // open it when you add the Layer later on. But doing so here allows us to check
    // for any errors beforehand.
    Status status = tileSource->open();
    if (status.isError())
    {
        OE_WARN << "Error opening the tile source; message = " << status.message() << std::endl;
        return -1;
    }

    // Add a new ImageLayer to the map with our custom tile source.
    ImageLayerOptions options( "My custom ImageLayer" );
    map->addLayer( new ImageLayer(options, tileSource) );

    // That's it, the map is ready; now create a MapNode to render the Map:
    MapNode* mapNode = new MapNode( map );

    viewer.setSceneData( mapNode );
    viewer.setCameraManipulator( new EarthManipulator() );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
