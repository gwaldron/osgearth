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
#include <osgEarth/SpatialReference>
#include <osgEarth/StringUtils>
#include <iostream>

bool eq(const double& a, const double& b, double e =1e-6) {
    return fabs(a - b) <= e;
}

bool eq(const osg::Vec3d& a, const osg::Vec3d& b, double e =1e-6) {
    return fabs(a.x() - b.x()) <= e && fabs(a.y() - b.y()) <= e && fabs(a.z() - b.z()) <= e;
}

int fail(const std::string& test, const std::string& msg) {
    std::cout << test << ": " << msg << std::endl;
    return -1;
}

int
main(int argc, char** argv)
{
    using namespace osgEarth;

    std::string test;

    // Vertical datum tests.
    // Reference: http://earth-info.nga.mil/GandG/wgs84/gravitymod/egm96/intpt.html
    {
        test = "Convert geodetic Z to EGM96";
        const SpatialReference* wgs84 = SpatialReference::get("wgs84");
        const SpatialReference* wgs84_egm96 = SpatialReference::get("wgs84", "egm96");        
        osg::Vec3d input[4] = {
            osg::Vec3d( 0,  0,  17.16),
            osg::Vec3d( 90, 0, -63.24),
            osg::Vec3d(180, 0,  21.15),
            osg::Vec3d(-90, 0,  -4.29)
        };
        osg::Vec3d output;
        for (int i = 0; i<4; ++i) {
            osg::Vec3d output;
            if (!wgs84->transform(input[i], wgs84_egm96, output))
                std::cout << test << ": transform failed for texst #" << i << std::endl;
            else if (!eq(output.z(), 0.0, 0.01)) {
                std::cout << test << ": output doesn't match for test #" << i << "; expected Z=0, got Z=" << output.z() << std::endl;
            }
        }
    }

    return 0;
}
