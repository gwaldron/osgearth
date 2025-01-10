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
#include <osgEarth/ContourMap>
#include <osgEarth/Shaders>
#include <osgEarth/Registry>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/MapNode>

#define LC "[ContourMap] "

using namespace osgEarth;

//........................................................................

Config
ContourMapLayer::Options::getMetadata()
{
    return Config::readJSON(R"(
        { "name" : "ContourMap",
          "properties" : [
          ]
        }
    )");
}

Config
ContourMapLayer::Options::getConfig() const
{
    Config conf = VisibleLayer::Options::getConfig();
    Config stopsConf("stops");
    for (auto& stop : stops())
    {
        Config s("stop");
        s.set("elevation", stop.elevation);
        s.set("color", stop.color);
        stopsConf.add(s);
    }
    if (!stopsConf.empty())
        conf.add(stopsConf);
    return conf;
}

void
ContourMapLayer::Options::fromConfig(const Config& conf)
{
    stops().clear();
    const ConfigSet& stopsConf = conf.child("stops").children();
    for (auto& stop : stopsConf)
    {
        Stop s;
        if (stop.get("elevation", s.elevation) && stop.get("color", s.color))
            stops().emplace_back(s);
    }
}

//........................................................................

REGISTER_OSGEARTH_LAYER(contourmap, ContourMapLayer);

void
ContourMapLayer::setTransferFunction(osg::TransferFunction1D* xfer)
{
    _xfer = xfer;
    dirty();
}

void
ContourMapLayer::dirty()
{
    OE_SOFT_ASSERT_AND_RETURN(_xfer.valid() && _xferMin.valid() && _xferRange.valid(), void());

    _xferTexture->setImage(_xfer->getImage());
    _xferMin->set(_xfer->getMinimum());
    _xferRange->set(_xfer->getMaximum() - _xfer->getMinimum());
}

void
ContourMapLayer::init()
{
    super::init();

    setRenderType(RENDERTYPE_TERRAIN_SURFACE);

    osg::StateSet* stateset = getOrCreateStateSet();

    // uniforms we'll need:
    _xferMin = new osg::Uniform(osg::Uniform::FLOAT, "oe_contour_min");
    stateset->addUniform(_xferMin.get());

    _xferRange = new osg::Uniform(osg::Uniform::FLOAT, "oe_contour_range");
    stateset->addUniform(_xferRange.get());

#if defined(OSG_GLES2_AVAILABLE) || defined(OSG_GLES3_AVAILABLE)
    _xferSampler = new osg::Uniform(osg::Uniform::SAMPLER_2D, "oe_contour_xfer");
#else
    _xferSampler = new osg::Uniform(osg::Uniform::SAMPLER_1D, "oe_contour_xfer");
#endif
    stateset->addUniform(_xferSampler.get());

    // Create a 1D texture from the transfer function's image.
    _xferTexture = new TextureType();
    _xferTexture->setResizeNonPowerOfTwoHint(false);
    _xferTexture->setUseHardwareMipMapGeneration(false);
    _xferTexture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
    _xferTexture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    _xferTexture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
    // defer installing the texture until we get a unit

    // shaders:
    VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
    vp->setName(typeid(*this).name());
    Shaders pkg;
    pkg.load(vp, pkg.ContourMap);

    // build a transfer function.
    osg::TransferFunction1D* xfer = new osg::TransferFunction1D();

    if (options().stops().empty() == false)
    {
        for (auto& stop : options().stops())
        {
            xfer->setColor(stop.elevation, stop.color, false);
        }
    }
    else
    {
        xfer->setColor(-7500.0f, Color("#000016"));
        xfer->setColor(-2500.0f, Color("#00007f"));
        xfer->setColor(-625.0f, Color("#0000ff"));
        xfer->setColor(0.0f, Color("#007fff"));
        xfer->setColor(2.5f, Color("#c2b280"));
        xfer->setColor(50.0f, Color("#d6d63f"));
        xfer->setColor(250.0f, Color("#1f9e00"));
        xfer->setColor(1000.0f, Color("#30632c"));
        xfer->setColor(1750.0f, Color("#a37714"));
        xfer->setColor(2250.0f, Color("#6c2f2f"));
        xfer->setColor(2600.0f, Color("#7f7f7f"));
        xfer->setColor(3000.0f, Color("#ffffff"));
    }

    xfer->updateImage();
    this->setTransferFunction(xfer);
}

void
ContourMapLayer::prepareForRendering(TerrainEngine* engine)
{
    super::prepareForRendering(engine);

    if (!engine->getResources()->reserveTextureImageUnitForLayer(_reservation, this, "ContourMap"))
    {
        setStatus(Status::ResourceUnavailable, "No texture image units available");
        return;
    }

    osg::StateSet* stateset = getOrCreateStateSet();
    stateset->setTextureAttributeAndModes(_reservation.unit(), _xferTexture.get(), osg::StateAttribute::ON);
    _xferSampler->set(_reservation.unit());
}

Status
ContourMapLayer::closeImplementation()
{
    _reservation.release();

    return super::closeImplementation();
}
