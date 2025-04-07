/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/

#include <osgEarth/catch.hpp>

#include <iostream>

#include <osgEarth/FileUtils>

using namespace osgEarth;

TEST_CASE( "getFullPath works" ) {    
    // Basic relative paths should should.
    std::string fullPath = osgEarth::Util::getFullPath("C:/images/vacation.jpg", "../models/model.obj");
    REQUIRE(fullPath == "C:/models/model.obj");

    // Single and double dots should be handled.
    fullPath = osgEarth::Util::getFullPath("C:/images/vacation.jpg", "./../models/model.obj");
    REQUIRE(fullPath == "C:/models/model.obj");

    // If an absolute path is passed in along with a relativeTo the absolute paths should be returned.
    fullPath = osgEarth::Util::getFullPath("C:/images/vacation.jpg", "c:/models/model2.obj");
    REQUIRE(fullPath == "c:/models/model2.obj");

    // If no relativeTo is passed in the unmodified path should be returned.
    fullPath = osgEarth::Util::getFullPath("", "../../images/vacation.jpg");
    REQUIRE(fullPath == "../../images/vacation.jpg");

    // Absolute paths with relative paths should get resolved
    fullPath = osgEarth::Util::getFullPath("", "c:/images/../models/model.obj");
    REQUIRE(fullPath == "c:/models/model.obj");

    // If just the relativeTo is passed in then that is what should be returned.
    fullPath = osgEarth::Util::getFullPath("c:/images/vacation.jpg", "");
    REQUIRE(fullPath == "c:/images/vacation.jpg");
}

TEST_CASE("stripRelativePaths works") {
    // Basic relative paths should should.
    std::string fullPath = osgEarth::Util::stripRelativePaths("http://server.com/files/1/2/3/../../../image.png");
    REQUIRE(fullPath == "http://server.com/files/image.png");

    fullPath = osgEarth::Util::stripRelativePaths("C:/files/1/2/3/../../../image.png");
    REQUIRE(fullPath == "C:/files/image.png");

    // If you pass in a relative path it should be returned unmodified.
    fullPath = osgEarth::Util::stripRelativePaths("../data/world.tif");
    REQUIRE(fullPath == "../data/world.tif");
}

