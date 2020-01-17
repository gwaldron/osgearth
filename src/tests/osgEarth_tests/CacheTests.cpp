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

#include <osgEarth/catch.hpp>
#include <osgEarth/GeoData>
#include <osgEarth/Registry>
#include <osgEarth/Cache>

using namespace osgEarth;

TEST_CASE( "Cache" ) {

    // Get the cache
    osg::ref_ptr<Cache> cache = Registry::instance()->getDefaultCache();
    REQUIRE(cache.valid());

    // open a bin:
    osg::ref_ptr< CacheBin > bin = cache->addBin("test_bin");
    REQUIRE(bin.valid());

    SECTION("String")
    {
        std::string key("string_key");
        std::string value("What is the sound of one hand clapping?");
        osg::ref_ptr<StringObject> s = new StringObject(value);

        // Write a string to the cache
        REQUIRE(bin->write(key, s.get(), 0L));

        // Read the string from the cache
        ReadResult r = bin->readString(key, 0L);
        REQUIRE(r.succeeded());

        // Make sure the values match
        REQUIRE(r.getString().compare(value) == 0);

        // Remove the string
        REQUIRE(bin->remove(key));

        // Try to read it again and make sure it's gone
        ReadResult r2 = bin->readString(key, 0L);
        REQUIRE(r2.failed());        
    }

    SECTION("Image")
    {
        std::string key("image_key");
        osg::ref_ptr<osg::Image> image = ImageUtils::createOnePixelImage(osg::Vec4(1, 0, 0, 1));

        // Write an image to the cache
        REQUIRE(bin->write(key, image.get(), 0L));

        // Read the image from the cache
        ReadResult r = bin->readImage(key, 0L);
        REQUIRE(r.succeeded());

        REQUIRE(ImageUtils::areEquivalent(r.getImage(), image.get()));

        // Remove the image
        REQUIRE(bin->remove(key));

        // Try to read it again and make sure it's gone
        ReadResult r2 = bin->readImage(key, 0L);
        REQUIRE(r2.failed());
    }  
}
