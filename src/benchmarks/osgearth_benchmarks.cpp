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
#include <benchmark/benchmark.h>
#include <osgDB/ReadFile>

void VectorPushback(unsigned int range)
{
    std::vector< int > values;
    for (unsigned int i = 0; i < range; i++)
    {
        values.push_back(i);
    }
}

void VectorPushbackWithReserve(unsigned int range)
{
    std::vector< int > values;
    values.reserve(range);
    for (unsigned int i = 0; i < range; i++)
    {
        values.emplace_back(i);
    }
}

void VectorAssign(unsigned int range)
{
    std::vector< int > values;
    values.resize(range);
    for (unsigned int i = 0; i < range; i++)
    {
        values[i] = i;
        benchmark::ClobberMemory();
    }
}

static void BM_VectorPushback(benchmark::State& state) {
    unsigned int count = state.range(0);
    for (auto _ : state) {
        VectorPushback(count);
    }
}
BENCHMARK(BM_VectorPushback)->Range(8, 8 << 10);

static void BM_VectorPushbackWithReserve(benchmark::State& state) {
    unsigned int count = state.range(0);
    for (auto _ : state) {
        VectorPushbackWithReserve(count);
    }
}
BENCHMARK(BM_VectorPushbackWithReserve)->Range(8, 8 << 10);

static void BM_VectorAssign(benchmark::State& state) {
    unsigned int count = state.range(0);
    for (auto _ : state) {
        VectorAssign(count);
    }
}
BENCHMARK(BM_VectorAssign)->Range(8, 8 << 10);

static void BM_ReadTiff(benchmark::State& state) {
    for (auto _ : state) {
        osg::ref_ptr< osg::Image > image = osgDB::readImageFile("D:/geodata/116_tms/10/1340/379.tif");
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_ReadTiff);

static void BM_ReadLerc(benchmark::State& state) {
    for (auto _ : state) {
        osg::ref_ptr< osg::Image > image = osgDB::readImageFile("D:/geodata/116_tms/10/1340/379.lerc");
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_ReadLerc);


// Run the benchmark
BENCHMARK_MAIN();