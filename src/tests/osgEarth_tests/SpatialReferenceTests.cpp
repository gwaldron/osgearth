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

#include <osgEarth/SpatialReference>

using namespace osgEarth;

TEST_CASE( "SpatialReferences are cached" ) {
    osg::ref_ptr< const SpatialReference > srs1 = SpatialReference::create("spherical-mercator");
    REQUIRE(srs1.valid());

    osg::ref_ptr< const SpatialReference > srs2 = SpatialReference::create("spherical-mercator");
    REQUIRE(srs2.valid());

    REQUIRE(srs1.get() == srs2.get());
}

TEST_CASE( "Spherical Mercator SpatialReferences can be created" ) {
    // Create a spherical mercator SRS.
    osg::ref_ptr< const SpatialReference > mercSRS = SpatialReference::create("spherical-mercator");

    REQUIRE( mercSRS.valid() );
    REQUIRE( mercSRS->isSphericalMercator() );    
    REQUIRE( mercSRS->isProjected() );    
    REQUIRE( !mercSRS->isGeodetic() );    
    REQUIRE( !mercSRS->isGeographic() );    
    
    SECTION("epsg:900913 is equivalent") {
        osg::ref_ptr< const SpatialReference > epsg900913 = SpatialReference::create("epsg:900913");
        REQUIRE(epsg900913.valid());
        REQUIRE(epsg900913->isEquivalentTo(mercSRS.get()));
    }

    SECTION("epsg:3785 is equivalent") {
        osg::ref_ptr< const SpatialReference > epsg3785 = SpatialReference::create("epsg:3785");
        REQUIRE(epsg3785.valid());
        REQUIRE(epsg3785->isEquivalentTo(mercSRS.get()));
    }

    SECTION("epsg:102113 is equivalent") {
        osg::ref_ptr< const SpatialReference > epsg102113 = SpatialReference::create("epsg:102113");
        REQUIRE(epsg102113.valid());
        REQUIRE(epsg102113->isEquivalentTo(mercSRS.get()));
    }    
}

TEST_CASE( "WGS84 SpatialReferences can be created" ) {
    // Create a wgs84 mercator SRS.
    osg::ref_ptr< const SpatialReference > wgs84 = SpatialReference::create("wgs84");     
    REQUIRE( wgs84.valid() );

    REQUIRE(wgs84->isGeographic());
    REQUIRE(wgs84->isGeodetic());
    REQUIRE(!wgs84->isMercator());
    REQUIRE(!wgs84->isProjected());
    
    SECTION("epsg:4326 is equivalent") {
        osg::ref_ptr< const SpatialReference > epsg4326 = SpatialReference::create("epsg:4326");
        REQUIRE(epsg4326.valid());
        REQUIRE(epsg4326->isEquivalentTo(wgs84.get()));
    }
}

TEST_CASE("Plate Carre SpatialReferences can be created") {
    osg::ref_ptr< const SpatialReference > plateCarre = SpatialReference::create("plate-carre");
    REQUIRE(plateCarre.valid());
    REQUIRE(!plateCarre->isGeographic());
    REQUIRE(!plateCarre->isMercator());
    REQUIRE(!plateCarre->isGeodetic());
    REQUIRE(plateCarre->isProjected());
}
