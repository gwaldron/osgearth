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
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include <osgEarth/LayerShader>
#include <osgEarth/ShaderLoader>
#include <osgEarth/VirtualProgram>
#include <osgEarth/Layer>
#include <osg/Texture2D>
#include <osg/Texture2DArray>

#undef  LC
#define LC "[LayerShader] "

using namespace osgEarth;

//...................................................................

Config
ShaderOptions::getConfig() const
{
    Config conf = ConfigOptions::getConfig();

    if (!_code.empty())
    {
        conf.setValue(_code);
    }

    conf.remove("sampler");
    for (unsigned i = 0; i < _samplers.size(); ++i) {
        Config c("sampler");
        c.add("name", _samplers[i]._name);
        if (_samplers[i]._uris.size() > 1) {
            Config urlarray("array");
            c.add(urlarray);
            for (std::vector<URI>::const_iterator j = _samplers[i]._uris.begin(); j != _samplers[i]._uris.end(); ++j) {
                urlarray.add(j->getConfig());
            }
        }
        else if (_samplers[i]._uris.size() == 1) {
            c.add("url", _samplers[i]._uris.back().getConfig());
        }
        conf.add(c);
    }

    conf.remove("uniform");
    for (unsigned i = 0; i < _uniforms.size(); ++i) {
        Config c("uniform");
        c.set("name", _uniforms[i]._name);
        c.set("value", _uniforms[i]._value);
        conf.add(c);
    }

    return conf;
}

void
ShaderOptions::fromConfig(const Config& conf)
{
    _code = conf.value();

    _samplers.clear();
    ConfigSet s = conf.children("sampler");
    for (ConfigSet::const_iterator i = s.begin(); i != s.end(); ++i) {
        _samplers.push_back(Sampler());
        _samplers.back()._name = i->value("name");
        const Config* urlarray = i->find("array");
        if (urlarray) {
            ConfigSet uris = urlarray->children("url");
            for (ConfigSet::const_iterator j = uris.begin(); j != uris.end(); ++j) {
                URI uri(j->value(), URIContext(conf.referrer()));
                _samplers.back()._uris.push_back(uri);
            }
        }
        else {
            optional<URI> uri;
            i->get("url", uri);
            if (uri.isSet())
                _samplers.back()._uris.push_back(uri.get());
        }
    }

    s = conf.children("uniform");
    for (ConfigSet::const_iterator i = s.begin(); i != s.end(); ++i) {
        _uniforms.push_back(Uniform());
        _uniforms.back()._name = i->value("name");
        i->get("value", _uniforms.back()._value);
    }
}

//...................................................................

LayerShader::LayerShader(const ShaderOptions& options) :
_options(options)
{
    //nop
}

LayerShader::~LayerShader()
{
    //nop
}

void
LayerShader::install(Layer* layer, TerrainResources* res)
{
    if (!layer || !res)
        return;

    osg::StateSet* stateset = layer->getOrCreateStateSet();

    VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
    ShaderPackage package;
    package.add("", _options.code());
    package.loadAll(vp, layer->getReadOptions());

    for (int i = 0; i < _options.samplers().size(); ++i)
    {
        const ShaderOptions::Sampler& sampler = _options.samplers()[i];
        if (!sampler._name.empty())
        {
            _reservations.push_back(TextureImageUnitReservation());
            TextureImageUnitReservation& reservation = _reservations.back();

            if (sampler._uris.size() == 1) // Texture2D
            {
                if (res->reserveTextureImageUnitForLayer(reservation, layer, "User shader sampler"))
                {
                    osg::Image* image = sampler._uris[0].getImage(layer->getReadOptions());
                    if (image)
                    {
                        osg::Texture2D* tex = new osg::Texture2D(image);
                        tex->setFilter(tex->MIN_FILTER, tex->NEAREST_MIPMAP_LINEAR);
                        tex->setFilter(tex->MAG_FILTER, tex->LINEAR);
                        tex->setWrap(tex->WRAP_S, tex->REPEAT);
                        tex->setWrap(tex->WRAP_T, tex->REPEAT);
                        tex->setUnRefImageDataAfterApply(true);
                        tex->setMaxAnisotropy(4.0);
                        tex->setResizeNonPowerOfTwoHint(false);

                        stateset->setTextureAttribute(reservation.unit(), tex);
                        stateset->addUniform(new osg::Uniform(sampler._name.c_str(), reservation.unit()));
                    }
                }
                else
                {
                    OE_WARN << LC << "Failed to allocate a texture image unit for this terrain shader sampler!\n";
                }
            }

            else if (sampler._uris.size() > 1) // Texture2DArray
            {
                if (res->reserveTextureImageUnitForLayer(reservation, layer, "User shader sampler array"))
                {
                    int sizeX = 0, sizeY = 0;
                    osg::Texture2DArray* tex = new osg::Texture2DArray();
                    tex->setTextureSize(512, 512, sampler._uris.size());
                    tex->setTextureDepth(sampler._uris.size());

                    for (int j = 0; j < sampler._uris.size(); ++j)
                    {
                        const URI& uri = sampler._uris[j];

                        osg::ref_ptr<osg::Image> image = uri.getImage(layer->getReadOptions());
                        if (image)
                        {
                            OE_INFO << LC << "   Added image from \"" << uri.full() << "\"\n";
                            tex->setImage(i, image);
                            tex->setFilter(tex->MIN_FILTER, tex->NEAREST_MIPMAP_LINEAR);
                            tex->setFilter(tex->MAG_FILTER, tex->LINEAR);
                            tex->setWrap(tex->WRAP_S, tex->CLAMP_TO_EDGE);
                            tex->setWrap(tex->WRAP_T, tex->CLAMP_TO_EDGE);
                            tex->setUnRefImageDataAfterApply(true);
                            tex->setResizeNonPowerOfTwoHint(false);

                            stateset->setTextureAttribute(reservation.unit(), tex);
                            stateset->addUniform(new osg::Uniform(sampler._name.c_str(), reservation.unit()));

                            if (sizeX == 0)
                            {
                                sizeX = image->s();
                                sizeY = image->t();
                                tex->setTextureSize(sizeX, sizeY, sampler._uris.size());
                            }
                        }
                    }
                }
                else
                {
                    OE_WARN << LC << "Failed to allocate a texture image unit for this terrain shader sampler!\n";
                }
            }
        }
    }

    for (int i = 0; i < _options.uniforms().size(); ++i)
    {
        const ShaderOptions::Uniform& uniform = _options.uniforms()[i];
        if (!uniform._name.empty() && uniform._value.isSet())
        {
            osg::Uniform* u = new osg::Uniform(uniform._name.c_str(), (float)uniform._value.get());
            stateset->addUniform(u);
        }
    }
}

//...................................................................
