/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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
#include <osgEarth/Cache>
#include <osgEarth/Registry>
#include <osgEarth/StringUtils>
#include <osgEarth/ImageUtils>
#include <osg/ArgumentParser>

#define LC "[cache_test] "

using namespace osgEarth;


int
quit(const std::string& msg)
{
    OE_NOTICE << msg << std::endl;
    return -1;
}

int
main(int argc, char** argv)
{
    osg::ref_ptr<Cache> cache = Registry::instance()->getDefaultCache();
    if ( !cache.valid() )
    {
        return quit( "Please configure a cache path in your environment (OSGEARTH_CACHE_PATH)." );
    }

    // open a bin:
    CacheBin* bin = cache->addBin("test_bin");
    if (!bin)
        return quit( "Failed to open the cache bin!" );

    // STRING:
    {
        std::string value( "What is the sound of one hand clapping?" );
        osg::ref_ptr<StringObject> s = new StringObject( value );

        if ( !bin->write("string_key", s.get(), 0L) )
            return quit( "String write failed." );

        ReadResult r = bin->readString("string_key", 0L);
        if ( r.failed() )
            return quit( Stringify() << "String read failed - " << r.getResultCodeString() );

        if ( r.getString().compare(value) != 0 )
            return quit( "String read error - values do not match" );

        OE_NOTICE << "String test: PASS" << std::endl;
    }

    // IMAGE:
    {
        osg::ref_ptr<osg::Image> image = ImageUtils::createOnePixelImage(osg::Vec4(1,0,0,1));

        if ( !bin->write("image_key", image.get(), 0L) )
            return quit("Image write failed.");

        ReadResult r = bin->readImage("image_key", 0L);
        if ( r.failed() )
            return quit( Stringify() << "Image read failed - " << r.getResultCodeString() );

        if ( !ImageUtils::areEquivalent(r.getImage(), image.get()) )
            return quit( "Image read error - images do not match" );

        OE_NOTICE << "Image test: PASS" << std::endl;
    }

    // Need to properly shut down the cache here
    cache = 0L;

    OE_NOTICE << "All tests passed." << std::endl;
    return 0;
}
