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

using namespace osgEarth;

TEST_CASE( "GeoExtent" ) {
    
    const SpatialReference* WGS84 = SpatialReference::get("wgs84");
    const SpatialReference* CUBE = SpatialReference::get("unified-cube");
    
    SECTION("Create an extent that crosses the antimeridian") {
        GeoExtent ext(SpatialReference::create("wgs84"), 178, 30, 183.4, 34.5);
        REQUIRE(ext.crossesAntimeridian());
    
        // Transform it and make sure it still crosses the antimeridian.
        GeoExtent transformed;
        ext.transform(SpatialReference::create("wgs84"), transformed);
        REQUIRE(transformed.crossesAntimeridian());
    }

    SECTION("GeoExtent contains simple") {
        GeoExtent ext(WGS84, -10.0, -10.0, 10.0, 10.0);
        REQUIRE(ext.contains(5.0, 5.0));
    }

    SECTION( "GeoExtent contains works when the extent cross the antimeridian" ) {
        GeoExtent ext(WGS84, -180.001, -90.0, 179.995, 90.0);
        REQUIRE(ext.contains(5.0, 0.0));
    }

    SECTION("GeoExtent contains a point on the boundary") {
        GeoExtent ext(WGS84, -100, -80, 100, 80);
        REQUIRE(ext.contains(-100, -80));
    }

    SECTION("expandToInclude is a noop when point is within the extent") {
        GeoExtent ext(WGS84, -10.0, -10.0, 10.0, 10.0);          
        ext.expandToInclude(0.0, 0.0);
        REQUIRE(ext == GeoExtent(WGS84, -10.0, -10.0, 10.0, 10.0));
    }

    SECTION("expandToInclude expand to the east and north") {
        GeoExtent ext(WGS84, -10.0, -10.0, 10.0, 10.0);          
        ext.expandToInclude(15.0, 15.0);
        REQUIRE(ext == GeoExtent(WGS84, -10.0, -10.0, 15.0, 15.0));
    }

    SECTION("expandToInclude expand to the west and south") {
        GeoExtent ext(WGS84, -10.0, -10.0, 10.0, 10.0);          
        ext.expandToInclude(-15.0, -15.0);
        REQUIRE(ext == GeoExtent(WGS84, -15.0, -15.0, 10.0, 10.0));
    }

    SECTION("expandToInclude to include the antimeridian to the east") {
        GeoExtent ext(WGS84, 160.0, -10.0, 170.0, 10.0);
        ext.expandToInclude(-160.0, 0.0);
        REQUIRE(ext == GeoExtent(WGS84, 160.0, -10.0, -160.0, 10.0));
    }

    SECTION("expandToInclude to include the antimeridian to the west") {
        GeoExtent ext(WGS84, -170.0, -10.0, -160.0, 10.0);
        ext.expandToInclude(160.0, 0.0);
        REQUIRE(ext == GeoExtent(WGS84, 160.0, -10.0, -160.0, 10.0));
    }

    SECTION("expandToInclude across the antimeridian to the east") {
        GeoExtent ext(WGS84, 160.0, -10.0, -170.0, 10.0);
        ext.expandToInclude(-160.0, 0.0);
        REQUIRE(ext == GeoExtent(WGS84, 160.0, -10.0, -160.0, 10.0));
    }

    SECTION("expandToInclude across the antimeridian to the west") {
        GeoExtent ext(WGS84, 170.0, -10.0, -160.0, 10.0);
        ext.expandToInclude(160.0, 0.0);
        REQUIRE(ext == GeoExtent(WGS84, 160.0, -10.0, -160.0, 10.0));
    }

    SECTION("multiple expandToIncludes") {
        GeoExtent ext(WGS84, -10.0, -10.0, 10.0, 10.0);
        ext.expandToInclude(-15.0, -15.0);
        ext.expandToInclude(15.0, 15.0);
        REQUIRE(ext == GeoExtent(WGS84, -15.0, -15.0, 15.0, 15.0));
    }

    SECTION("calling expandToInclude on an invalid extent just takes the incoming value.") {
        GeoExtent invalid(WGS84);
        invalid.expandToInclude(-15.0, -15.0);        
        REQUIRE(invalid == GeoExtent(WGS84, -15.0, -15.0, -15.0, -15.0));
    }

    SECTION("expandToInclude another geoextent to the east") {
        GeoExtent ext(WGS84, -130, -10, -100, 10);
        ext.expandToInclude(GeoExtent(WGS84, 0, -10, 40, 10));
        REQUIRE(ext == GeoExtent(WGS84, -130, -10, 40, 10));
    }

    SECTION("expandToInclude another geoextent wrapping around") {
        GeoExtent ext(WGS84, -130, -10, -100, 10);
        ext.expandToInclude(GeoExtent(WGS84, 70, -10, 80, 10));
        REQUIRE(ext == GeoExtent(WGS84, 70, -10, -100, 10));
    }

    SECTION("expandToInclude LHS completely contains RHS") {
        GeoExtent ext(WGS84, -90, 0, -45, 45);
        ext.expandToInclude(GeoExtent(WGS84, -60, 10, -50, 20));
        REQUIRE(ext == GeoExtent(WGS84, -90, 0, -45, 45));
    }

    SECTION("expandToInclude LHS overlaps RHS") {
        GeoExtent ext(WGS84, -90, 0, -45, 45);
        ext.expandToInclude(GeoExtent(WGS84, -60, 20, -35, 50));
        REQUIRE(ext == GeoExtent(WGS84, -90, 0, -35, 50));
    }

    SECTION("expandToInclude LHS overlaps RHS and RHS wraps around") {
        GeoExtent ext(WGS84, 150, 0, 170, 10);
        ext.expandToInclude(GeoExtent(WGS84, 160, 5, -165, 25));
        REQUIRE(ext == GeoExtent(WGS84, 150, 0, -165, 25));
    }

    //SECTION("expandToInclude expands to the full extent") {
    //    GeoExtent full(WGS84);
    //    full.expandToInclude(-180.0, -90);
    //    // First point should result in zero width
    //    REQUIRE(full.width() == 0.0);
    //    full.expandToInclude(180.0, 90);
    //    // Seond point should result in full width
    //    REQUIRE(full.width() == 360.0);
    //}

    SECTION("Intersect 2 non-overlapping extents") {
        GeoExtent e1(WGS84, -10, -10, 10, 10);
        GeoExtent e2(WGS84, 20, 20, 30, 30);
        REQUIRE(e1.intersects(e2)==false);
        REQUIRE(e1.intersectionSameSRS(e2).isInvalid());
    }

    SECTION("Intersect 2 simple overlapping extents") {
        GeoExtent e1(WGS84, -10, -10, 10, 10);
        GeoExtent e2(WGS84, 5, 5, 20, 20);
        REQUIRE(e1.intersects(e2)==true);
        REQUIRE(e1.intersectionSameSRS(e2) == GeoExtent(WGS84, 5, 5, 10, 10));
    }

    SECTION("Intersect non-overlapping anti-meridian extent with a simple extent") {
        GeoExtent e1(WGS84, 170, -10, -170, 10);
        GeoExtent e2(WGS84, 20, 20, 30, 30);
        REQUIRE(e1.intersects(e2)==false);
        REQUIRE(e1.intersectionSameSRS(e2).isInvalid());
    }

    SECTION("Intersect overlapping anti-meridian extent with a simple extent") {
        GeoExtent e1(WGS84, 170, -10, -170, 10);
        GeoExtent e2(WGS84, -175, -60, -165, 60);
        REQUIRE(e1.intersects(e2)==true);
        REQUIRE(e1.intersectionSameSRS(e2) == GeoExtent(WGS84, -175, -10, -170, 10));
    }

    SECTION("Intersect 2 non-overlapping anti-meridian extents") {
        GeoExtent e1(WGS84, 170, -10, -170, 10);
        GeoExtent e2(WGS84, 130, -50, -120, -40);
        REQUIRE(e1.intersects(e2)==false);
        REQUIRE(e1.intersectionSameSRS(e2).isInvalid());
    }

    SECTION("Intersect 2 overlapping anti-meridian extents") {
        GeoExtent e1(WGS84, 170, -10, -170, 10);
        GeoExtent e2(WGS84, 130, -50, -120, 5);
        REQUIRE(e1.intersects(e2)==true);
        REQUIRE(e1.intersectionSameSRS(e2) == GeoExtent(WGS84, 170, -10, -170, 5));
    }

    SECTION("2 extents that abut do not intersect") {
        GeoExtent e1(WGS84, 0, 0, 10, 10);
        GeoExtent e2(WGS84, 10, 0, 20, 10);
        REQUIRE(e1.intersects(e2)==false);
        REQUIRE(e1.intersectionSameSRS(e2).isInvalid());
    }
    
    SECTION("Scaling") {
        GeoExtent e1(WGS84, -10, -10, 10, 10);
        e1.scale(2, 2);
        REQUIRE(e1 == GeoExtent(WGS84, -20, -20, 20, 20));
    }

    SECTION("ExpandBy") {
        GeoExtent e1(WGS84, -10, -10, 10, 10);
        e1.expand(5, 5);
        REQUIRE(e1 == GeoExtent(WGS84, -12.5, -12.5, 12.5, 12.5));
    }


    // Older ones
    SECTION("Normalization across the antimeridian") {
        GeoExtent ext(SpatialReference::create("wgs84"), 175.0, -10.0, 185.0, 10.0);
        REQUIRE(ext.crossesAntimeridian());
        REQUIRE(ext.east() == -175.0);
    }

    SECTION("expandToInclude is a noop when point is within the extent") {
        GeoExtent ext(SpatialReference::create("wgs84"), 175.0, -10.0, 185.0, 10.0);
        ext.expandToInclude(180.0, 0.0);
        REQUIRE(ext == GeoExtent(SpatialReference::create("wgs84"), 175.0, -10.0, 185.0, 10.0));
    }

    SECTION("expandToInclude expand to the west") {
        GeoExtent ext(SpatialReference::create("wgs84"), 175.0, -10.0, 185.0, 10.0);
        ext.expandToInclude(170.0, 0.0);        
        REQUIRE(ext == GeoExtent(SpatialReference::create("wgs84"), 170.0, -10.0, 185.0, 10.0));
        REQUIRE(ext.crossesAntimeridian());
    }

    SECTION("expandToInclude expand to the east") {
        GeoExtent ext(SpatialReference::create("wgs84"), 175.0, -10.0, 185.0, 10.0);
        ext.expandToInclude(186.0, 0.0);        
        REQUIRE(ext == GeoExtent(SpatialReference::create("wgs84"), 175.0, -10.0, 186.0, 10.0));
        REQUIRE(ext.crossesAntimeridian());
    }

    SECTION("expandToInclude expands to the closest side of the bounds") {
        // This seems like it would expand to the east, but b/c of wrapping the final point is actually closer to the 
        // west side, so it will expand westward.
        GeoExtent ext(SpatialReference::create("wgs84"), 175.0, -10.0, 185.0, 10.0);
        ext.expandToInclude(525.0, 0.0);        
        REQUIRE(ext == GeoExtent(SpatialReference::create("wgs84"), 165.0, -10.0, 185.0, 10.0));
        REQUIRE(ext.crossesAntimeridian());
    }

    SECTION("Validate input") {
        REQUIRE(GeoExtent(WGS84, DBL_MAX, DBL_MAX, -DBL_MAX, -DBL_MAX).isInvalid());
        REQUIRE(GeoExtent(WGS84, 0, 0, 1, DBL_MAX).isInvalid());
        REQUIRE(GeoExtent(WGS84, 0.0, 0.0, 10.0, -10.0).isInvalid());
        REQUIRE(GeoExtent(WGS84, sqrt(-1.0), 0.0, 10.0, 10.0).isInvalid());
    }

    SECTION("Cube 1") {
        GeoExtent e1(CUBE, 0.0, 0.0, 6.0, 1.0);
        GeoExtent e2(WGS84, -180.0, -90.0, 180.0, 90.0);
        REQUIRE(e1.isValid());
        REQUIRE(e1.intersects(e2));
        REQUIRE(e2.intersects(e1));
        REQUIRE(e1.contains(e2));
        REQUIRE(e2.contains(e1));
    }
}
