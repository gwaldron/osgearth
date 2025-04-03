/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2018 Pelican Mapping
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

