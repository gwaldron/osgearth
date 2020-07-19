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
#include <osgEarth/Notify>
#include <osgEarth/Geocoder>

#define LC "[geocode] "

using namespace osgEarth;

int
usage(const char* name)
{
    OE_NOTICE << "\nUsage: " << name << " [--async] <place name>" << std::endl;
    return 0;
}


int
main(int argc, char** argv)
{
    osgEarth::initialize();

    osg::ArgumentParser args(&argc, argv);

    if (argc < 2)
        return usage(argv[0]);

    bool async = args.read("--async");

    osg::ref_ptr<osgDB::Options> options;
    osg::ref_ptr<ThreadPool> pool;
    if (async)
    {
        options = new osgDB::Options();
        pool = new ThreadPool("Geocoder");
        pool->put(options.get());
    }

    Geocoder geocoder;
    geocoder.setServiceOption("WRITE_CACHE", "FALSE");
    Geocoder::Results results = geocoder.search(argv[1], options.get());

    if (async)
    {
        // prove that it ran in the background :)
        int count = 0;
        while(!results.isReady()) ++count;
        OE_NOTICE << "Took " << count << " ticks to finish running asynchronously." << std::endl;
    }

    if (results.getStatus().isOK())
    {
        while(results.getFeatures()->hasMore())
        {
            OE_NOTICE << "Result -------------------------------------------" << std::endl;
            Feature* f = results.getFeatures()->nextFeature();
            const AttributeTable& attrs = f->getAttrs();
            for(AttributeTable::const_iterator i = attrs.begin(); i != attrs.end(); ++i)
            {
                OE_NOTICE << i->first << " = " << i->second.getString() << std::endl;
            }
        }
    }
    else
    {
        OE_WARN << results.getStatus().toString() << std::endl;
    }
}
