/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2020 Pelican Mapping
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
#include <osgEarth/SimpleOceanLayer>
#include <osgEarth/Shaders>
#include <osgEarth/VirtualProgram>
#include <osgEarth/ImageLayer>
#include <osgEarth/Lighting>
#include <osgEarth/TerrainEngineNode>
#include <osg/CullFace>
#include <osg/Texture2D>

using namespace osgEarth;

#define LC "[SimpleOceanLayer] "

//...................................................................

Config
SimpleOceanLayer::Options::getConfig() const
{
    Config conf = VisibleLayer::Options::getConfig();
    conf.set("color", _color);
    conf.set("max_altitude", _maxAltitude);
    conf.set("use_bathymetry", _useBathymetry);
    conf.set("texture", _textureURI);
    conf.set("texture_lod", _textureLOD);
    maskLayer().set(conf, "mask_layer");
    return conf;
}

void
SimpleOceanLayer::Options::fromConfig(const Config& conf)
{
    color().setDefault(Color("#050D1AFF"));
    maxAltitude().setDefault(500000.f);
    useBathymetry().setDefault(true);
    textureLOD().setDefault(13u);

    conf.get("color", _color);
    conf.get("max_altitude", _maxAltitude);
    conf.get("use_bathymetry", _useBathymetry);
    conf.get("texture", _textureURI);
    conf.get("texture_lod", _textureLOD);
    maskLayer().get(conf, "mask_layer");
}

//...................................................................


/** Register this layer so it can be used in an earth file */
REGISTER_OSGEARTH_LAYER(ocean, SimpleOceanLayer);
REGISTER_OSGEARTH_LAYER(simple_ocean, SimpleOceanLayer);

OE_LAYER_PROPERTY_IMPL(SimpleOceanLayer, bool, UseBathymetry, useBathymetry);
OE_LAYER_PROPERTY_IMPL(SimpleOceanLayer, unsigned, SurfaceTextureLOD, textureLOD);


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
    vp->setName("SimpleOceanLayer");
    Shaders shaders;
    shaders.load(vp, shaders.SimpleOceanLayer);

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
    osg::Material* m = new MaterialGL3();
    m->setAmbient(m->FRONT, osg::Vec4(1, 1, 1, 1));
    m->setDiffuse(m->FRONT, osg::Vec4(1, 1, 1, 1));
    m->setSpecular(m->FRONT, osg::Vec4(1, 1, 1, 1));
    m->setEmission(m->FRONT, osg::Vec4(0, 0, 0, 1));
    m->setShininess(m->FRONT, 32.0);
    ss->setAttributeAndModes(m, 1);
    MaterialCallback().operator()(m, 0L);
    
    setColor(options().color().get());
    setMaxAltitude(options().maxAltitude().get());
    setSeaLevel(0.0f); // option?

    installDefaultOpacityShader();
}

Config
SimpleOceanLayer::getConfig() const
{
    Config c = VisibleLayer::getConfig();
    return c;
}

Status
SimpleOceanLayer::openImplementation()
{
    Status parent = VisibleLayer::openImplementation();
    if (parent.isError())
        return parent;

    Status fsStatus = options().maskLayer().open(getReadOptions());
    if (fsStatus.isError())
        return fsStatus;

    updateMaskLayer();

    return Status::NoError;
}

void
SimpleOceanLayer::prepareForRendering(TerrainEngine* engine)
{
    VisibleLayer::prepareForRendering(engine);

    if (options().textureURI().isSet()) // texture
    {
        if (engine->getResources()->reserveTextureImageUnitForLayer(_texReservation, this) == false)
        {
            OE_WARN << LC << "Failed to reserve a TIU...will not apply texture" << std::endl;
            return;
        }

        ReadResult r = options().textureURI()->readImage(getReadOptions());
        if (r.failed())
        {
            OE_WARN << LC << "Failed to load ocean texture: " << r.errorDetail() << std::endl;
            return;
        }

        setSurfaceImage(r.getImage());
    }
}

void
SimpleOceanLayer::setSurfaceImage(osg::Image* image)
{
    if (image)
    {
        osg::Texture2D* tex = new osg::Texture2D(image);
        tex->setFilter(tex->MIN_FILTER, tex->LINEAR_MIPMAP_LINEAR);
        tex->setFilter(tex->MAG_FILTER, tex->LINEAR);
        tex->setWrap(tex->WRAP_S, tex->REPEAT);
        tex->setWrap(tex->WRAP_T, tex->REPEAT);
        tex->setUnRefImageDataAfterApply(true);

        osg::StateSet* ss = getOrCreateStateSet();
        ss->setTextureAttributeAndModes(_texReservation.unit(), tex, 1);
        ss->setDefine("OE_OCEAN_TEXTURE", "oe_ocean_tex");
        ss->addUniform(new osg::Uniform("oe_ocean_tex", _texReservation.unit()));

        ss->setDefine("OE_OCEAN_TEXTURE_LOD", Stringify() << options().textureLOD().get());

        if (image->getFileName().empty() == false)
        {
            options().textureURI() = image->getFileName();
        }
    }    
}

void
SimpleOceanLayer::setMaskLayer(ImageLayer* maskLayer)
{
    if (maskLayer != options().maskLayer().getLayer())
    {
        options().maskLayer().setLayer(maskLayer);
        updateMaskLayer();
    }
}

ImageLayer*
SimpleOceanLayer::getMaskLayer() const
{
    return options().maskLayer().getLayer();
}

void
SimpleOceanLayer::updateMaskLayer()
{
    ImageLayer* layer = options().maskLayer().getLayer();

    if (layer && layer->isOpen())
    {
        if (!layer->isShared())
        {
            OE_WARN << LC << "Mask layer \"" << layer->getName() << "\" is not a shared\n";
            return;
        }

        // activate the mask.
        osg::StateSet* ss = getOrCreateStateSet();
        ss->setDefine("OE_OCEAN_MASK", layer->getSharedTextureUniformName());
        ss->setDefine("OE_OCEAN_MASK_MATRIX", layer->getSharedTextureMatrixUniformName());

        OE_INFO << LC << "Installed \"" << layer->getName() << "\" as mask layer\n";
    }

    else
    {
        osg::StateSet* ss = getOrCreateStateSet();
        ss->removeDefine("OE_OCEAN_MASK");
        ss->removeDefine("OE_OCEAN_MASK_MATRIX");

        //OE_INFO << LC << "Uninstalled mask layer\n";
    }
}

void
SimpleOceanLayer::addedToMap(const Map* map)
{    
    VisibleLayer::addedToMap(map);
    
    options().maskLayer().addedToMap(map);
    updateMaskLayer();
}

void
SimpleOceanLayer::removedFromMap(const Map* map)
{
    options().maskLayer().removedFromMap(map);
    updateMaskLayer();

    VisibleLayer::removedFromMap(map);
}

void
SimpleOceanLayer::setColor(const Color& color)
{
    options().color() = color;
    getOrCreateStateSet()->getOrCreateUniform(
        "oe_ocean_color", osg::Uniform::FLOAT_VEC4)->set(color);
}

const Color&
SimpleOceanLayer::getColor() const
{
    return options().color().get();
}

void
SimpleOceanLayer::setMaxAltitude(const float& value)
{
    options().maxAltitude() = value;
    getOrCreateStateSet()->getOrCreateUniform(
        "oe_ocean_maxAltitude", osg::Uniform::FLOAT)->set(value);

    // temporary shiv:
    setMaxVisibleRange(value);
}

const float&
SimpleOceanLayer::getMaxAltitude() const
{
    return options().maxAltitude().get();
}

void 
SimpleOceanLayer::modifyTileBoundingBox(const TileKey& key, osg::BoundingBox& box) const
{
    // Force the max Z to be at least sea level, to satisfy the culling pass
    box.zMax() = osg::maximum(box.zMax(), (osg::BoundingBox::value_type)0.0);
}

void
SimpleOceanLayer::setSeaLevel(const float& value)
{
    options().seaLevel() = value;
    getOrCreateStateSet()->getOrCreateUniform(
        "oe_ocean_seaLevel", osg::Uniform::FLOAT)->set(value);
}

const float&
SimpleOceanLayer::getSeaLevel() const
{
    return options().seaLevel().get();
}
