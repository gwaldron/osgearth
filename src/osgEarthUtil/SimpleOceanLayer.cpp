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
#include <osgEarthUtil/SimpleOceanLayer>
#include <osgEarth/VirtualProgram>
#include <osgEarth/Lighting>
#include <osg/CullFace>
#include <osg/Material>

using namespace osgEarth;
using namespace osgEarth::Util;

#define LC "[SimpleOceanLayer] "


/** Register this layer so it can be used in an earth file */
REGISTER_OSGEARTH_LAYER(simple_ocean, SimpleOceanLayer);


SimpleOceanLayer::SimpleOceanLayer() :
Layer(),
SimpleOceanLayerOptions()
{
    ctor();
}

SimpleOceanLayer::SimpleOceanLayer(const SimpleOceanLayerOptions& options) :
Layer(),
SimpleOceanLayerOptions(options)
{
    ctor();
}

void
SimpleOceanLayer::ctor()
{
    this->setName("Simple Ocean");

    OE_INFO << LC << "Creating a Simple Ocean Layer\n";

    const char* oceanFS =
        "#version 330 \n"
        "float oe_terrain_getElevation(); \n"
        "uniform vec4 ocean_color; \n"

        "float ocean_remap( float val, float vmin, float vmax, float r0, float r1 ) { \n"
        "    float vr = (clamp(val, vmin, vmax)-vmin)/(vmax-vmin); \n"
        "    return r0 + vr * (r1-r0); \n"
        "}\n"

        "void ocean_FS(inout vec4 color) { \n"
        "    const float lowF = -100.0;\n"
        "    const float hiF = -10.0;\n"
        "    const float seaLevel = 0.0;\n"

        "    float elevation = oe_terrain_getElevation(); \n"
        "    float alpha = ocean_remap(elevation, seaLevel+lowF, seaLevel+hiF, 1.0, 0.0); \n"
        "    color = vec4(ocean_color.rgb, alpha*ocean_color.a); \n"
        "} \n";

    setRenderType(RENDERTYPE_TILE);

    osg::StateSet* ss = getOrCreateStateSet();

    ss->setDefine("OE_TERRAIN_RENDER_ELEVATION", osg::StateAttribute::OFF);
    ss->setDefine("OE_TERRAIN_RENDER_NORMAL_MAP", osg::StateAttribute::OFF);

    VirtualProgram* vp = VirtualProgram::getOrCreate(ss);
    vp->setFunction("ocean_FS", oceanFS, ShaderComp::LOCATION_FRAGMENT_COLORING);

    // remove backface culling so we can see underwater
    // (use OVERRIDE since the terrain engine sets back face culling.)
    ss->setAttributeAndModes(
        new osg::CullFace(),
        osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

    // Material.
    osg::Material* m = new osg::Material();
    m->setAmbient(m->FRONT_AND_BACK, osg::Vec4(.5, .5, .5, 1));
    m->setDiffuse(m->FRONT_AND_BACK, osg::Vec4(1, 1, 1, 1));
    m->setSpecular(m->FRONT_AND_BACK, osg::Vec4(0.2, 0.2, 0.2, 1));
    m->setEmission(m->FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1));
    m->setShininess(m->FRONT_AND_BACK, 40.0);
    ss->setAttributeAndModes(m, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    m->setUpdateCallback(new MaterialCallback());

    applyOptions();
}

void
SimpleOceanLayer::applyOptions()
{
    osg::StateSet* ss = getOrCreateStateSet();

    ss->getOrCreateUniform("ocean_color", osg::Uniform::FLOAT_VEC4)->set(color().get());
}

void 
SimpleOceanLayer::modifyTileBoundingBox(const TileKey& key, osg::BoundingBox& box) const
{
    // Force the max Z to be at least sea level, to satisfy the culling pass
    box.zMax() = std::max(box.zMax(), 0.0f);
}

Config
SimpleOceanLayer::getConfig() const
{
    Config conf = SimpleOceanLayerOptions::getConfig();
    conf.key() = "simple_ocean";
    return conf;
}
