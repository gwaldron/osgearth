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

#include <osg/Notify>
#include <osgViewer/Viewer>
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/Filter>
#include <osgEarth/FilterContext>

#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;


/**
 * Simple Feature Filter that changes the value of a Feature data sources attribute
 */
class ChangeAttributeFilter : public FeatureFilter
{
public:
    ChangeAttributeFilter(const Config& conf)
    {
        if (conf.key() == "change_attribute")
        {
            conf.get("key", _key);
            conf.get("value", _value);
        }
    }

    virtual Config getConfig() const
    {
        Config config("change_attribute");
        config.set("key", _key);
        config.set("value", _value);
        return config;
    }


    virtual FilterContext push( FeatureList& input, FilterContext& context )
    {
        for (FeatureList::iterator itr = input.begin(); itr != input.end(); itr++)
        {
            //Change the value of the attribute
            if (_key.isSet() && _value.isSet())
            {
                itr->get()->set(*_key, std::string(*_value));
            }
        }
        return context;
    }

    optional< std::string > _key;
    optional< std::string > _value;
};

//Register our custom FeatureFilter with osgEarth
//The first
OSGEARTH_REGISTER_SIMPLE_FEATUREFILTER(change_attribute, ChangeAttributeFilter);

int
main(int argc, char** argv)
{
    osgEarth::initialize();

    //Run this example with the feature_custom_filters.earth file in the tests directory for a simple example
    osg::ArgumentParser arguments(&argc,argv);

    // create a viewer:
    osgViewer::Viewer viewer(arguments);

    //Tell the database pager to not modify the unref settings
    viewer.getDatabasePager()->setUnrefImageDataAfterApplyPolicy( false, false );

    // install our default manipulator (do this before calling load)
    viewer.setCameraManipulator( new EarthManipulator() );

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags
    osg::Node* node = MapNodeHelper().load( arguments, &viewer );
    if ( node )
    {
        viewer.setSceneData( node );

        viewer.run();
    }
    else
    {
        OE_NOTICE
            << "\nUsage: " << argv[0] << " file.earth" << std::endl
            << MapNodeHelper().usage() << std::endl;
    }
    return 0;
}
