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
#include <osgEarth/Map>
#include <osg/CullFace>
#include <osg/Material>


using namespace osgEarth;
using namespace osgEarth::Util;

#define LC "[SimpleOceanLayer] "


/** Register this layer so it can be used in an earth file */
REGISTER_OSGEARTH_LAYER(simple_ocean, SimpleOceanLayer);


namespace
{
    struct MapCallbackAdapter : public MapCallback
    {
        MapCallbackAdapter(SimpleOceanLayer* obj) : _ocean(obj) { }
        
        void onImageLayerAdded(ImageLayer* layer, unsigned index)
        {
            osg::ref_ptr<SimpleOceanLayer> ocean;
            if (_ocean.lock(ocean))
            {
                if (ocean->getOptions().maskLayer().isSetTo(layer->getName()))
                {
                    if (ocean->setMaskLayer(layer))
                    {
                        _activeMaskLayerName = layer->getName();
                    }
                }
            }
        }

        void onImageLayerRemoved(ImageLayer* layer, unsigned index)
        {
            osg::ref_ptr<SimpleOceanLayer> ocean;
            if (_ocean.lock(ocean))
            {
                if (layer->getName() == _activeMaskLayerName)
                {
                    ocean->setMaskLayer(0L);
                }
            }
        }

        osg::observer_ptr<SimpleOceanLayer> _ocean;
        std::string _activeMaskLayerName;
    };
}


SimpleOceanLayer::SimpleOceanLayer() :
Layer(&_layerOptionsConcrete),
_layerOptions(&_layerOptionsConcrete)
{
    ctor();
}

SimpleOceanLayer::SimpleOceanLayer(const SimpleOceanLayerOptions& options) :
Layer(&_layerOptionsConcrete),
_layerOptions(&_layerOptionsConcrete),
_layerOptionsConcrete(options)
{
    ctor();
}

void
SimpleOceanLayer::ctor()
{
    OE_INFO << LC << "Creating a Simple Ocean Layer\n";

    _mapCallback = 0L;

    this->setName("Simple Ocean");
    setRenderType(RENDERTYPE_TILE);

    osg::StateSet* ss = getOrCreateStateSet();
    ss->setDataVariance(ss->DYNAMIC);
    
    VirtualProgram* vp = VirtualProgram::getOrCreate(ss);
    Shaders shaders;
    shaders.load(vp, shaders.SimpleOceanLayer_Vertex);
    shaders.load(vp, shaders.SimpleOceanLayer_Fragment);

    ss->setDefine("OE_TERRAIN_RENDER_ELEVATION", osg::StateAttribute::OFF);
    ss->setDefine("OE_TERRAIN_RENDER_NORMAL_MAP", osg::StateAttribute::OFF);

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
    
    setColor(getOptions().color().get());
    setMaxAltitude(getOptions().maxAltitude().get());
}

bool
SimpleOceanLayer::setMaskLayer(const ImageLayer* maskLayer)
{
    if (maskLayer)
    {
        if (!maskLayer->getEnabled())
        {
            OE_WARN << LC << "Mask layer \"" << maskLayer->getName() << "\" disabled\n";
            return false;
        }

        if (!maskLayer->isShared())
        {
            OE_WARN << LC << "Mask layer \"" << maskLayer->getName() << "\" is not a shared\n";
            return false;
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

    return true;
}

void
SimpleOceanLayer::addedToMap(const Map* map)
{    
    if (getOptions().maskLayer().isSet())
    {
        // subscribe so we know if the mask layer arrives or departs:
        _mapCallback = map->addMapCallback(new MapCallbackAdapter(this));

        // see if it's already there. If so, try to install it; 
        // if not, rely on the MapCallback to let us know if and when
        // it arrives.
        ImageLayer* maskLayer = map->getLayerByName<ImageLayer>(getOptions().maskLayer().get());
        if (maskLayer)
        {
            setMaskLayer(maskLayer);
        }
    }      
}

void
SimpleOceanLayer::removedFromMap(const Map* map)
{
    if (getOptions().maskLayer().isSet())
    {
        if (_mapCallback)
            map->removeMapCallback(_mapCallback);
        _mapCallback = 0L;
        setMaskLayer(0L);
    }
}

void
SimpleOceanLayer::setColor(const Color& color)
{
    mutableOptions().color() = color;
    getOrCreateStateSet()->getOrCreateUniform(
        "ocean_color", osg::Uniform::FLOAT_VEC4)->set(color);
}

const Color&
SimpleOceanLayer::getColor() const
{
    return getOptions().color().get();
}

void
SimpleOceanLayer::setMaxAltitude(float alt)
{
    mutableOptions().maxAltitude() = alt;
    getOrCreateStateSet()->getOrCreateUniform(
        "ocean_maxAltitude", osg::Uniform::FLOAT)->set(alt);
}

float
SimpleOceanLayer::getMaxAltitude() const
{
    return getOptions().maxAltitude().get();
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
    Config conf = getOptions().getConfig();
    conf.key() = "simple_ocean";
    return conf;
}
