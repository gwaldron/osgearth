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
#include <osgEarthUtil/Shaders>
#include <osgEarth/VirtualProgram>
#include <osgEarth/Lighting>
#include <osgEarth/ImageLayer>
#include <osgEarth/Map>
#include <osg/CullFace>
#include <osg/Material>


using namespace osgEarth;
using namespace osgEarth::Util;

#define LC "[SimpleOceanLayer] "


/** Register this layer so it can be used in an earth file */
REGISTER_OSGEARTH_LAYER(simple_ocean, SimpleOceanLayer);



SimpleOceanLayer::SimpleOceanLayer() :
VisibleLayer(&_optionsConcrete),
_options(&_optionsConcrete)
{
    init();
}

SimpleOceanLayer::SimpleOceanLayer(const SimpleOceanLayerOptions& options) :
VisibleLayer(&_optionsConcrete),
_options(&_optionsConcrete),
_optionsConcrete(options)
{
    init();
}

void
SimpleOceanLayer::init()
{
    OE_INFO << LC << "Creating a Simple Ocean Layer\n";

    VisibleLayer::init();

    this->setName("Simple Ocean");
    setRenderType(RENDERTYPE_TERRAIN_SURFACE);

    osg::StateSet* ss = getOrCreateStateSet();
    ss->setDataVariance(ss->DYNAMIC);
    
    VirtualProgram* vp = VirtualProgram::getOrCreate(ss);
    Shaders shaders;
    shaders.load(vp, shaders.SimpleOceanLayer_Vertex);
    shaders.load(vp, shaders.SimpleOceanLayer_Fragment);

    ss->setDefine("OE_TERRAIN_RENDER_ELEVATION", osg::StateAttribute::OFF);
    ss->setDefine("OE_TERRAIN_RENDER_NORMAL_MAP", osg::StateAttribute::OFF);

    if (options().useBathymetry() == true)
    {
        ss->setDefine("OE_OCEAN_USE_BATHYMETRY");
    }

    // remove backface culling so we can see underwater
    // (use OVERRIDE since the terrain engine sets back face culling.)
    ss->setAttributeAndModes(
        new osg::CullFace(),
        osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

    // Material.
#if 0
    osg::Material* m = new MaterialGL3();
    m->setAmbient(m->FRONT, osg::Vec4(.5, .5, .5, 1));
    m->setDiffuse(m->FRONT, osg::Vec4(1, 1, 1, 1));
    m->setSpecular(m->FRONT, osg::Vec4(1, 1, 1, 1)); //0.2, 0.2, 0.2, 1));
    m->setEmission(m->FRONT, osg::Vec4(0, 0, 0, 1));
    m->setShininess(m->FRONT, 100.0);
    ss->setAttributeAndModes(m, 1); //osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    m->setUpdateCallback(new MaterialCallback());
#endif
    
    setColor(options().color().get());
    setMaxAltitude(options().maxAltitude().get());
}

void
SimpleOceanLayer::setMaskLayer(const ImageLayer* maskLayer)
{
    if (maskLayer)
    {
        if (!maskLayer->getEnabled())
        {
            OE_WARN << LC << "Mask layer \"" << maskLayer->getName() << "\" disabled\n";
            return;
        }

        if (!maskLayer->isShared())
        {
            OE_WARN << LC << "Mask layer \"" << maskLayer->getName() << "\" is not a shared\n";
            return;
        }

        // activate the mask.
        osg::StateSet* ss = getOrCreateStateSet();
        ss->setDefine("OE_OCEAN_MASK", maskLayer->shareTexUniformName().get());
        ss->setDefine("OE_OCEAN_MASK_MATRIX", maskLayer->shareTexMatUniformName().get());

        OE_INFO << LC << "Installed \"" << maskLayer->getName() << "\" as mask layer\n";
    }

    else
    {
        osg::StateSet* ss = getOrCreateStateSet();
        ss->removeDefine("OE_OCEAN_MASK");
        ss->removeDefine("OE_OCEAN_MASK_MATRIX");

        OE_INFO << LC << "Uninstalled mask layer\n";
    }
}

void
SimpleOceanLayer::addedToMap(const Map* map)
{    
    if (options().maskLayer().isSet())
    {
        // listen for the mask layer.
        _layerListener.listen(map, options().maskLayer().get(), this, &SimpleOceanLayer::setMaskLayer);
    }      
}

void
SimpleOceanLayer::removedFromMap(const Map* map)
{
    if (options().maskLayer().isSet())
    {
        _layerListener.clear();
        setMaskLayer(0L);
    }
}

void
SimpleOceanLayer::setColor(const Color& color)
{
    options().color() = color;
    getOrCreateStateSet()->getOrCreateUniform(
        "ocean_color", osg::Uniform::FLOAT_VEC4)->set(color);
}

const Color&
SimpleOceanLayer::getColor() const
{
    return options().color().get();
}

void
SimpleOceanLayer::setMaxAltitude(float alt)
{
    options().maxAltitude() = alt;
    getOrCreateStateSet()->getOrCreateUniform(
        "ocean_maxAltitude", osg::Uniform::FLOAT)->set(alt);
}

float
SimpleOceanLayer::getMaxAltitude() const
{
    return options().maxAltitude().get();
}

void 
SimpleOceanLayer::modifyTileBoundingBox(const TileKey& key, osg::BoundingBox& box) const
{
    // Force the max Z to be at least sea level, to satisfy the culling pass
    box.zMax() = std::max(box.zMax(), (osg::BoundingBox::value_type)0.0);
}

Config
SimpleOceanLayer::getConfig() const
{
    Config conf = options().getConfig();
    conf.key() = "simple_ocean";
    return conf;
}
