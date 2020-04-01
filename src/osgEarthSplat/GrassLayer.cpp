/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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
#include "GrassLayer"
#include "SplatShaders"

#define LC "[GrassLayer] " << getName() << ": "

// The TS/GS implementation uses GL_PATCHES to pass the terrain tile geometry
// to the tessellation shader and then the geometry shader where it generates
// billboards. Undef this to use a VS-only implementation (which is slower
// and still needs some work with the colors, etc.) but could be useful if
// GS are extremely slow or unavailable on your target platform.
//#define USE_GEOMETRY_SHADER

// If we're not using the GS, we have the option of using instancing or not.
// OFF benchmarks faster on older cards. On newer cards (RTX 2070 e.g.)
// it's slightly faster than using a giant DrawElementsUInt.
#define USE_INSTANCING_IN_VERTEX_SHADER

// Test instanced model substitution.
// This works, but we need a compute or geometry shader or something to
// do culling, because everything makes its way tot the fragment shader
// and we're relying on discards to skip drawing
//#define TEST_MODEL_INSTANCING

using namespace osgEarth::Splat;

REGISTER_OSGEARTH_LAYER(grass, GrassLayer);

Config
GrassLayer::Options::getConfig() const
{
    Config conf = GroundCoverLayer::Options::getConfig();
    return conf;
}

void
GrassLayer::Options::fromConfig(const Config& conf)
{
    //nop
}

void
GrassLayer::init()
{
    GroundCoverLayer::init();

    //_useCoverageToAlpha = false;

    if (!options().lod().isSet())
        options().lod().init(19u);
}

void
GrassLayer::loadShaders(VirtualProgram* vp, const osgDB::Options* options) const
{
    GroundCoverShaders s;
    s.load(vp, s.Grass_VS, options);
}


osg::Geometry*
GrassLayer::createGeometry(unsigned vboTileDim) const    
{
    unsigned numInstances = vboTileDim * vboTileDim;
    const unsigned vertsPerInstance = 16;
    const unsigned indiciesPerInstance = 54;

#ifdef TEST_MODEL_INSTANCING
    geom = makeShape();
    geom->getPrimitiveSet(0)->setNumInstances(numInstances);
    return;
#endif

    osg::Geometry* out_geom = new osg::Geometry();
    out_geom->setUseVertexBufferObjects(true);

#ifdef USE_INSTANCING_IN_VERTEX_SHADER

    static const GLubyte indices[54] = {
        0,1,4, 4,1,5, 1,2,5, 5,2,6, 2,3,6, 6,3,7,
        4,5,8, 8,5,9, 5,6,9, 9,6,10, 6,7,10, 10,7,11,
        8,9,12, 12,9,13, 9,10,13, 13,10,14, 10,11,14, 14,11,15
    };

    out_geom->addPrimitiveSet(new osg::DrawElementsUByte(GL_TRIANGLES, indiciesPerInstance, &indices[0], numInstances));

    // We don't actually need any verts. Is it OK not to set an array?
    //geom->setVertexArray(new osg::Vec3Array(8));

    osg::Vec3Array* normals = new osg::Vec3Array(osg::Array::BIND_OVERALL);
    normals->push_back(osg::Vec3f(0,0,1));
    out_geom->setNormalArray(normals);

#else // big giant drawelements (NOT WRITTEN YET)

    // Do we need this at all?
    unsigned int numverts = numInstances * vertsPerInstance;
    osg::Vec3Array* coords = new osg::Vec3Array(numverts);
    out_geom->setVertexArray(coords);

    osg::Vec3Array* normals = new osg::Vec3Array(osg::Array::BIND_OVERALL);
    normals->push_back(osg::Vec3f(0, 0, 1));
    out_geom->setNormalArray(normals);

    osg::DrawElements* de =
        numverts > 0xFFFF ? (osg::DrawElements*)new osg::DrawElementsUInt(GL_TRIANGLES)
        : (osg::DrawElements*)new osg::DrawElementsUShort(GL_TRIANGLES);

    const unsigned totalIndicies = numInstances * indiciesPerInstance;
    de->reserveElements(totalIndicies);

    for (unsigned i = 0; i < numInstances; ++i)
    {
        unsigned offset = i * vertsPerInstance;

        de->addElement(0 + offset);
        de->addElement(1 + offset);
        de->addElement(2 + offset);
        de->addElement(2 + offset);
        de->addElement(1 + offset);
        de->addElement(3 + offset);

        de->addElement(4 + offset);
        de->addElement(5 + offset);
        de->addElement(6 + offset);
        de->addElement(6 + offset);
        de->addElement(5 + offset);
        de->addElement(7 + offset);

        if (false) //settings->_grass) // third crosshatch
        {
            de->addElement(8 + offset);
            de->addElement(9 + offset);
            de->addElement(10 + offset);
            de->addElement(10 + offset);
            de->addElement(9 + offset);
            de->addElement(11 + offset);
        }
    }

    out_geom->addPrimitiveSet(de);

#endif // USE_INSTANCING_IN_VERTEX_SHADER

    return out_geom;
}