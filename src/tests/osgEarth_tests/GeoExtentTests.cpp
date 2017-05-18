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

#include <osgEarth/catch.hpp>

#include <osgEarth/GeoData>

using namespace osgEarth;

TEST_CASE( "GeoExtent transformMBR preserves the crossesAntimerdian properly" ) {

    // Create an extent that crosses the antimerdian.
    GeoExtent ext(SpatialReference::create("wgs84"), 178, 30, 183.4, 34.5);
    REQUIRE(ext.crossesAntimeridian());
    
    // Transform it and make sure it still crosses the antimerdian.
    GeoExtent transformed;
    ext.transform(SpatialReference::create("wgs84"), transformed);
    REQUIRE(transformed.crossesAntimeridian());
}

TEST_CASE( "GeoExtent contains work" ) {
    GeoExtent ext(SpatialReference::create("wgs84"), -10.0, -10.0, 10.0, 10.0);
    REQUIRE(ext.contains(5.0, 5.0));
}

TEST_CASE( "GeoExtent contains works when the extent cross the antimerdian" ) {

    // Create an extent that crosses the antimerdian.
    GeoExtent ext(SpatialReference::create("wgs84"), -180.001, -90.0, 179.995, 90.0);
    REQUIRE(ext.contains(5.0, 0.0));
}

TEST_CASE( "GeoExtent expandToInclude works") {
    
    GeoExtent ext(SpatialReference::create("wgs84"), -10.0, -10.0, 10.0, 10.0);          

    SECTION("expandToInclude is a noop when point is within the extent") {
        ext.expandToInclude(0.0, 0.0);
        REQUIRE(ext == GeoExtent(SpatialReference::create("wgs84"), -10.0, -10.0, 10.0, 10.0));
    }

    SECTION("expandToInclude expand to the east and north") {
        ext.expandToInclude(15.0, 15.0);
        REQUIRE(ext == GeoExtent(SpatialReference::create("wgs84"), -10.0, -10.0, 15.0, 15.0));
    }

    SECTION("expandToInclude expand to the west and south") {
        ext.expandToInclude(-15.0, -15.0);
        REQUIRE(ext == GeoExtent(SpatialReference::create("wgs84"), -15.0, -15.0, 10.0, 10.0));
    }

    SECTION("multiple expandToIncludes") {
        ext.expandToInclude(-15.0, -15.0);
        ext.expandToInclude(15.0, 15.0);
        REQUIRE(ext == GeoExtent(SpatialReference::create("wgs84"), -15.0, -15.0, 15.0, 15.0));
    }

    SECTION("calling expandToInclude on an invalid extent just takes the incoming value.") {
        GeoExtent invalid(SpatialReference::create("wgs84"));
        invalid.expandToInclude(-15.0, -15.0);        
        REQUIRE(invalid == GeoExtent(SpatialReference::create("wgs84"), -15.0, -15.0, -15.0, -15.0));
    }

    SECTION("expandToInclude expands to the full extent") {
        GeoExtent full(SpatialReference::create("wgs84"));
        full.expandToInclude(-180.0, -90);
        // First point should result in zero width
        REQUIRE(full.width() == 0.0);
        full.expandToInclude(180.0, 90);
        // Seond point should result in full width
        REQUIRE(full.width() == 360.0);
    }
}

TEST_CASE( "GeoExtent expandToInclude works with values that cross the antimerdian") {
    
    GeoExtent ext(SpatialReference::create("wgs84"), 175.0, -10.0, 185.0, 10.0);
    REQUIRE(ext.crossesAntimeridian());
    REQUIRE(ext.east() == -175.0);

    SECTION("expandToInclude is a noop when point is within the extent") {
        ext.expandToInclude(180.0, 0.0);
        REQUIRE(ext == GeoExtent(SpatialReference::create("wgs84"), 175.0, -10.0, 185.0, 10.0));
    }

    SECTION("expandToInclude expand to the west") {
        ext.expandToInclude(170.0, 0.0);        
        REQUIRE(ext == GeoExtent(SpatialReference::create("wgs84"), 170.0, -10.0, 185.0, 10.0));
        REQUIRE(ext.crossesAntimeridian());
    }

    SECTION("expandToInclude expand to the east") {
        ext.expandToInclude(186.0, 0.0);        
        REQUIRE(ext == GeoExtent(SpatialReference::create("wgs84"), 175.0, -10.0, 186.0, 10.0));
        REQUIRE(ext.crossesAntimeridian());
    }

    SECTION("expandToInclude expands to the closest side of the bounds") {
        // This seems like it would expand to the east, but b/c of wrapping the final point is actually closer to the 
        // west side, so it will expand westward.
        ext.expandToInclude(525.0, 0.0);        
        REQUIRE(ext == GeoExtent(SpatialReference::create("wgs84"), 165.0, -10.0, 185.0, 10.0));
        REQUIRE(ext.crossesAntimeridian());
    }
}




