/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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
#include <osgEarthUtil/SkyView>

#define LC "[SkyView] "

using namespace osgEarth;
using namespace osgEarth::Util;

//........................................................................

Config
SkyViewImageLayer::Options::getMetadata()
{
    return Config::readJSON(OE_MULTILINE(
        { "name" : "SkyView",
          "properties" : [
            { "name": "image", "description" : "Image layer to render", "type" : "ImageLayer", "default" : "" },
          ]
        }
    ));
}

Config
SkyViewImageLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    if (_imageLayer.isSet())
        conf.add(_imageLayer->getConfig());
    return conf;
}

void
SkyViewImageLayer::Options::fromConfig(const Config& conf)
{
    // TODO: make a helper function for this..LayerClient<> maybe,
    // analagous to FeatureSourceClient?
    for (ConfigSet::const_iterator i = conf.children().begin();
        i != conf.children().end();
        ++i)
    {
        osg::ref_ptr<Layer> fs = Layer::create(*i);
        if (fs.valid() && dynamic_cast<ImageLayer*>(fs.get()))
        {
            imageLayer() = ImageLayer::Options(*i);
            break;
        }
    }
}

//........................................................................

REGISTER_OSGEARTH_LAYER(skyview, SkyViewImageLayer);

void
SkyViewImageLayer::setImageLayer(ImageLayer* layer)
{
    _imageLayer = layer;
}

ImageLayer*
SkyViewImageLayer::getImageLayer() const
{
    return _imageLayer.get();
}

void
SkyViewImageLayer::init()
{
    ImageLayer::init();
    setTileSourceExpected(false);
}

const Status&
SkyViewImageLayer::open()
{
    // create the subordinate layer ig required:
    if (_imageLayer.valid() == false && options().imageLayer().isSet())
    {
        _imageLayer = dynamic_cast<ImageLayer*>(Layer::create(options().imageLayer().get()));
    }

    if (_imageLayer.valid() == false)
    {
        return setStatus(Status::ConfigurationError, "Missing subordinate image layer");
    }

    // open the subordinate layer:
    _imageLayer->setReadOptions(getReadOptions());
    const Status& insideStatus = _imageLayer->open();
    if (insideStatus.isError())
    {
        return setStatus(insideStatus);
    }

    // copy the profile:
    setProfile(_imageLayer->getProfile());

    return ImageLayer::open();
}

GeoImage
SkyViewImageLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
    if (getStatus().isError())
        return GeoImage(getStatus());

    // We need to flip the image horizontally so that it's viewable from inside the globe.

    // Create a new key with the x coordinate flipped.
    unsigned int numRows, numCols;

    key.getProfile()->getNumTiles(key.getLevelOfDetail(), numCols, numRows);
    unsigned int tileX = numCols - key.getTileX() - 1;
    unsigned int tileY = key.getTileY();
    unsigned int level = key.getLevelOfDetail();

    TileKey flippedKey(level, tileX, tileY, key.getProfile());
    GeoImage image = _imageLayer->createImageImplementation(flippedKey, progress);
    if (image.valid())
    {
        // If an image was read successfully, we still need to flip it horizontally
        image.getImage()->flipHorizontal();
    }
    return GeoImage(image.takeImage(), key.getExtent());
}
