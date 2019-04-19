/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
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

#include <osgEarthFeatures/Feature>
#include <osgEarthFeatures/GeometryUtils>

using namespace osgEarth;
using namespace osgEarth::Symbology;
using namespace osgEarth::Features;

TEST_CASE("Feature::splitAcrossDateLine doesn't modify features that don't cross the dateline") {
    osg::ref_ptr< Feature > feature = new Feature(GeometryUtils::geometryFromWKT("POLYGON((-81 26, -40.5 45, -40.5 75.5, -81 60))"), osgEarth::SpatialReference::create("wgs84"));
    FeatureList features;
    feature->splitAcrossDateLine(features);
    // We only have one feature in the list.
    REQUIRE( features.size() == 1 );
    // The feature is exactly the same feature that was passed in
    REQUIRE(features.front().get() == feature.get());
}

TEST_CASE("Feature::splitAcrossDateLine works") {
    osg::ref_ptr< Feature > feature = new Feature(GeometryUtils::geometryFromWKT("POLYGON((170 26, 190 26, 190 56, 170 56))"), osgEarth::SpatialReference::create("wgs84"));
    FeatureList features;
    feature->splitAcrossDateLine(features);
    // We have two features in the list
    REQUIRE(features.size() == 2);
    // The features don't cross the anti-meridian
    for (FeatureList::iterator itr = features.begin(); itr != features.end(); ++itr)
    {
        REQUIRE_FALSE(itr->get()->getExtent().crossesAntimeridian());
    }    
}

TEST_CASE("Feature handles attributes correctly.") {
    osg::ref_ptr< Feature > feature = new Feature(new Geometry(), osgEarth::SpatialReference::create("wgs84"));

    SECTION("Missing attributes get the correct default values.") {
        REQUIRE(feature->getDouble("foo") == 0.0);
        REQUIRE(feature->getBool("foo") == false);
        REQUIRE(feature->getString("foo") == "");
        REQUIRE(feature->getInt("foo") == 0);
    }

    SECTION("Setting attributes works") {
        feature->set("string", std::string("test"));
        REQUIRE(feature->getString("string") == "test");

        feature->set("double", 6.0);
        REQUIRE(feature->getDouble("double") == 6.0);

        feature->set("int", 8);
        REQUIRE(feature->getInt("int") == 8);

        feature->set("bool", true);
        REQUIRE(feature->getBool("bool") == true);        
    }

    SECTION("Null attributes get the correct default values") {
        // First we set the attribute to a value before we set it to null.

        feature->set("string", std::string("test"));
        REQUIRE(feature->isSet("string") == true);
        
        feature->setNull("string");
        REQUIRE(feature->isSet("string") == false);
        REQUIRE(feature->getString("string") == "");

        feature->set("double", 6.0);
        REQUIRE(feature->isSet("double") == true);
        feature->setNull("double");
        REQUIRE(feature->isSet("double") == false);
        REQUIRE(feature->getDouble("double") == 0.0);

        feature->set("int", 8);
        REQUIRE(feature->isSet("int") == true);
        feature->setNull("int");
        REQUIRE(feature->isSet("int") == false);
        REQUIRE(feature->getInt("int") == 0);

        feature->set("bool", true);
        REQUIRE(feature->isSet("bool") == true);
        feature->setNull("bool");
        REQUIRE(feature->isSet("bool") == false);
        REQUIRE(feature->getBool("bool") == false);
    }
}
