/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/

#include <osgEarth/catch.hpp>

#include <osgEarth/GeoData>

using namespace osgEarth;

TEST_CASE( "GeoExtent" ) {

    const SpatialReference* WGS84 = SpatialReference::get("wgs84");
    const SpatialReference* CUBE = SpatialReference::get("unified-cube");
    const SpatialReference* SPHMERC = SpatialReference::get("spherical-mercator");
    
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

    SECTION("Intersect overlapping anti-meridian extent with a simple extent (again)") {
        GeoExtent e1(WGS84, -10, -10, 10, 10);
        GeoExtent e2(WGS84, 170, -10, 0, 10);
        REQUIRE(e1.intersects(e2) == true);
        REQUIRE(e1.intersectionSameSRS(e2) == GeoExtent(WGS84, -10, -10, 0, 10));
    }

    SECTION("Intersect overlapping anti-meridian extent with a simple extent (and again)") {
        GeoExtent e1(WGS84, -10, -10, -170, 10);
        GeoExtent e2(WGS84, 0, -10, 10, 10);
        REQUIRE(e1.intersects(e2) == true);
        REQUIRE(e1.intersectionSameSRS(e2) == GeoExtent(WGS84, 0, -10, 10, 10));
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
        GeoExtent ext(WGS84, 175.0, -10.0, 185.0, 10.0);
        REQUIRE(ext.crossesAntimeridian());
        REQUIRE(ext.east() == -175.0);
    }

    SECTION("expandToInclude is a noop when point is within the extent") {
        GeoExtent ext(WGS84, 175.0, -10.0, 185.0, 10.0);
        ext.expandToInclude(180.0, 0.0);
        REQUIRE(ext == GeoExtent(WGS84, 175.0, -10.0, 185.0, 10.0));
    }

    SECTION("expandToInclude expand to the west") {
        GeoExtent ext(WGS84, 175.0, -10.0, 185.0, 10.0);
        ext.expandToInclude(170.0, 0.0);        
        REQUIRE(ext == GeoExtent(WGS84, 170.0, -10.0, 185.0, 10.0));
        REQUIRE(ext.crossesAntimeridian());
    }

    SECTION("expandToInclude expand to the east") {
        GeoExtent ext(WGS84, 175.0, -10.0, 185.0, 10.0);
        ext.expandToInclude(186.0, 0.0);        
        REQUIRE(ext == GeoExtent(WGS84, 175.0, -10.0, 186.0, 10.0));
        REQUIRE(ext.crossesAntimeridian());
    }

    SECTION("expandToInclude expands to the closest side of the bounds") {
        // This seems like it would expand to the east, but b/c of wrapping the final point is actually closer to the 
        // west side, so it will expand westward.
        GeoExtent ext(WGS84, 175.0, -10.0, 185.0, 10.0);
        ext.expandToInclude(525.0, 0.0);        
        REQUIRE(ext == GeoExtent(WGS84, 165.0, -10.0, 185.0, 10.0));
        REQUIRE(ext.crossesAntimeridian());
    }

    SECTION("Validate input") {
        REQUIRE(GeoExtent(WGS84, DBL_MAX, DBL_MAX, -DBL_MAX, -DBL_MAX).isInvalid());
        REQUIRE(GeoExtent(WGS84, 0, 0, 1, DBL_MAX).isInvalid());
        REQUIRE(GeoExtent(WGS84, 0.0, 0.0, 10.0, -10.0).isInvalid());
        REQUIRE(GeoExtent(WGS84, sqrt(-1.0), 0.0, 10.0, 10.0).isInvalid());
    }

    SECTION("Cube versus WGS84") {
        GeoExtent cube(CUBE, 0.0, 0.0, 6.0, 1.0);
        GeoExtent wgs84(WGS84, -180.0, -90.0, 180.0, 90.0);
        REQUIRE(cube.isValid());
        REQUIRE(wgs84.isValid());
        REQUIRE(cube.intersects(wgs84));
        REQUIRE(wgs84.intersects(cube));
        REQUIRE(cube.contains(wgs84));
        REQUIRE(wgs84.contains(cube));
    }

    SECTION("SphMerc versus WGS84") {
        GeoExtent sphm(SPHMERC, MERC_MINX, MERC_MINY, MERC_MAXX, MERC_MAXY);
        GeoExtent wgs84(WGS84, -180.0, -90.0, 180.0, 90.0);
        REQUIRE(sphm.isValid());
        REQUIRE(wgs84.isValid());
        REQUIRE(sphm.intersects(wgs84));
        REQUIRE(wgs84.intersects(sphm));
        REQUIRE(!sphm.contains(wgs84));
        REQUIRE(wgs84.contains(sphm));
    }
}
