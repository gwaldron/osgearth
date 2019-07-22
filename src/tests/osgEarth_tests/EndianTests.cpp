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
#include <cmath>
#include <osgEarth/catch.hpp>
#include <osgEarth/Endian>

namespace {

/** Returns true if two floats are equal within a given epsilon. */
bool floatAreEqual(float v1, float v2, double epsilon)
{
  return static_cast<double>(fabs(v2 - v1)) <= epsilon;
}

/** Returns true if two doubles are equal within a given epsilon. */
bool doubleAreEqual(double v1, double v2, double epsilon)
{
  return fabs(v2 - v1) <= epsilon;
}

/** Returns true if the system is detected as little endian */
bool systemIsLittleEndian()
{
  // Source: https://stackoverflow.com/questions/4181951
  int n = 1;
  return (*(char*)&n == 1);
}

}

TEST_CASE( "OE_ENCODE_FLOAT equality tests" ) {
    // Test for big endian systems has not been developed.
    if (!systemIsLittleEndian())
        return;
    REQUIRE(OE_ENCODE_FLOAT(0.f) == 0);
    REQUIRE(OE_ENCODE_FLOAT(1.f) == 32831);
    REQUIRE(OE_ENCODE_FLOAT(-1.f) == 32959);
    REQUIRE(OE_ENCODE_FLOAT(100.5f) == 51522);
    REQUIRE(OE_ENCODE_FLOAT(1.83e11f) == 2590911058);
}

TEST_CASE( "OE_ENCODE_DOUBLE equality tests" ) {
    // Test for big endian systems has not been developed.
    if (!systemIsLittleEndian())
        return;
    REQUIRE(OE_ENCODE_DOUBLE(0.) == 0);
    REQUIRE(OE_ENCODE_DOUBLE(1.) == 61503);
    REQUIRE(OE_ENCODE_DOUBLE(-1.) == 61631);
    REQUIRE(OE_ENCODE_DOUBLE(100.5) == 2120000);
    REQUIRE(OE_ENCODE_DOUBLE(1.83e11) == 222588388674);
}

TEST_CASE( "OE_DECODE_FLOAT equality tests" ) {
    // Test for big endian systems has not been developed.
    if (!systemIsLittleEndian())
        return;
    REQUIRE(floatAreEqual(OE_DECODE_FLOAT(0), 0.f, 1e-038));
    REQUIRE(floatAreEqual(OE_DECODE_FLOAT(1), 2.35099e-038f, 1e-042));
    REQUIRE(floatAreEqual(OE_DECODE_FLOAT(13784), -7.96046e+014f, 1e+10));
    REQUIRE(floatAreEqual(OE_DECODE_FLOAT(0x12345678), 1.73782e+034f, 1e+29));
    REQUIRE(floatAreEqual(OE_DECODE_FLOAT(0x78563412), 5.69046e-028f, 1e-32));
    REQUIRE(floatAreEqual(OE_DECODE_FLOAT(0xf8393841), 11.5142f, 1e-04));
}

TEST_CASE( "OE_DECODE_DOUBLE equality tests" ) {
    // Test for big endian systems has not been developed.
    if (!systemIsLittleEndian())
        return;
    REQUIRE(doubleAreEqual(OE_DECODE_DOUBLE(0), 0.f, 1e-308));
    REQUIRE(doubleAreEqual(OE_DECODE_DOUBLE(1), 7.29112e-304, 1e-308));
    REQUIRE(doubleAreEqual(OE_DECODE_DOUBLE(13784), -8.27442e+116, 1e+112));
    REQUIRE(doubleAreEqual(OE_DECODE_DOUBLE(0x12345678), 4.69197e+271, 1e+267));
    REQUIRE(doubleAreEqual(OE_DECODE_DOUBLE(0x78563412), 5.62635e-221, 1e-225));
    REQUIRE(doubleAreEqual(OE_DECODE_DOUBLE(0x123456789abcdef0), -4.88646e+235, 1e+231));
    REQUIRE(doubleAreEqual(OE_DECODE_DOUBLE(0xf23456789abcde01), 1.14742e-299, 1e-304));
}
