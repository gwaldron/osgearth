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
#include "MaterialLoader"
#include "URI"
#include "XmlUtils"
#include "ImageUtils"
#include "Elevation"

#include <osg/Texture2D>
#include <osg/Texture2DArray>
#include <osgDB/FileNameUtils>
#include <osgDB/ReadFile>

using namespace osgEarth;
using namespace osgEarth::Util;

#undef LC
#define LC "[MaterialUtils] "

MaterialUtils::Mangler
MaterialUtils::getDefaultNormalMapNameMangler()
{
    static Mangler getNormalMapFileName = [](const std::string& filename)
    {
        const std::string pattern = "_NML";

        std::string dot_ext = osgDB::getFileExtensionIncludingDot(filename);
        if (Strings::ciEquals(dot_ext, ".meif"))
        {
            auto underscore_pos = filename.find_last_of('_');
            if (underscore_pos != filename.npos)
            {
                return
                    filename.substr(0, underscore_pos)
                    + pattern
                    + filename.substr(underscore_pos);
            }
        }

        return
            osgDB::getNameLessExtension(filename)
            + pattern
            + dot_ext;
    };

    return getNormalMapFileName;
}

MaterialUtils::Mangler
MaterialUtils::getDefaultPBRMapNameMangler()
{
    static Mangler getPBRMapFileName = [](const std::string& filename)
    {
        const std::string pattern = "_MTL_GLS_AO";

        std::string dot_ext = osgDB::getFileExtensionIncludingDot(filename);
        if (Strings::ciEquals(dot_ext, ".meif"))
        {
            auto underscore_pos = filename.find_last_of('_');
            if (underscore_pos != filename.npos)
            {
                return
                    filename.substr(0, underscore_pos)
                    + pattern
                    + filename.substr(underscore_pos);
            }
        }

        return
            osgDB::getNameLessExtension(filename)
            + pattern
            + dot_ext;
    };

    return getPBRMapFileName;
}



#undef LC
#define LC "[MaterialLoader] "

MaterialLoader::MaterialLoader()
{
    setTraversalMode(TRAVERSE_ALL_CHILDREN);
    setNodeMaskOverride(~0);
}

void
MaterialLoader::setOptions(const osgDB::Options* options)
{
    _options = options;
}

void
MaterialLoader::setMangler(
    int unit,
    MaterialLoader::Mangler mangler)
{
    _manglers[unit] = mangler;
}

void
MaterialLoader::setTextureFactory(
    int unit,
    MaterialLoader::TextureFactory factory)
{
    _factories[unit] = factory;
}

void
MaterialLoader::apply(osg::Node& node)
{
    if (node.getStateSet())
        apply(node.getStateSet());
    traverse(node);
}

void
MaterialLoader::apply(osg::StateSet* ss)
{
    OE_HARD_ASSERT(ss != nullptr);

    if (ss->getTextureAttributeList().empty())
        return;

    osg::Texture* t = dynamic_cast<osg::Texture*>(ss->getTextureAttribute(0, osg::StateAttribute::TEXTURE));
    if (t == nullptr || t->getImage(0) == nullptr)
        return;

    std::string filename = osgDB::getSimpleFileName(t->getImage(0)->getFileName());

    for (auto& m : _manglers)
    {
        int unit = m.first;

        // if the unit is already occupied, ignore it
        // GW: removed this. Just replace it, nothing we can really do about it
        //if (ss->getTextureAttribute(unit, osg::StateAttribute::TEXTURE) != nullptr)
        //    continue;

        MaterialLoader::Mangler& mangler = m.second;
        URI materialURI(mangler(filename), URIContext(_referrer));

        // if it's already loaded re-use it
        osg::ref_ptr<osg::Texture> mat_tex;
        auto cache_iter = _cache.find(materialURI.full());
        if (cache_iter != _cache.end())
        {
            mat_tex = cache_iter->second;
        }
        else
        {
            //OE_INFO << LC << "Looking for: " << materialURI.full() << std::endl;
            osg::ref_ptr<osg::Image> image = materialURI.getImage(_options);
            if (image.valid())
            {
                auto iter = _factories.find(unit);
                if (iter != _factories.end())
                {
                    mat_tex = iter->second(image.get());
                }
                else
                {
                    mat_tex = new osg::Texture2D(image);
                }

                // match the params of the albedo texture
                mat_tex->setFilter(osg::Texture::MIN_FILTER, t->getFilter(osg::Texture::MIN_FILTER));
                mat_tex->setFilter(osg::Texture::MAG_FILTER, t->getFilter(osg::Texture::MAG_FILTER));
                mat_tex->setWrap(osg::Texture::WRAP_S, t->getWrap(osg::Texture::WRAP_S));
                mat_tex->setWrap(osg::Texture::WRAP_T, t->getWrap(osg::Texture::WRAP_T));
                mat_tex->setWrap(osg::Texture::WRAP_R, t->getWrap(osg::Texture::WRAP_R));
                mat_tex->setMaxAnisotropy(t->getMaxAnisotropy());

                _cache[materialURI.full()] = mat_tex;
                OE_INFO << LC << "Loaded material tex '" << materialURI.base() << "' to unit " << unit << std::endl;
            }
        }   

        if (mat_tex.valid())
        {
            ss->setTextureAttribute(unit, mat_tex, 1);
        }
    }
}


namespace
{
    const float DEFAULT_DISPLACEMENT = 0.0f;
    const float DEFAULT_ROUGHNESS = 0.5f;
    const float DEFAULT_AO = 1.0f;
    const float DEFAULT_METAL = 0.0f;

    osg::ref_ptr<osg::Image> assemble_RGBA(
        osg::ref_ptr<osg::Image>& color,
        osg::ref_ptr<osg::Image>& opacity)
    {
        osg::ref_ptr<osg::Image> output;

        if (color.valid())
        {
            output = new osg::Image();
            output->allocateImage(color->s(), color->t(), 1, GL_RGBA, GL_UNSIGNED_BYTE);
            ImageUtils::PixelWriter write_output(output.get());
            ImageUtils::PixelReader read_color(color.get());
            ImageUtils::PixelReader read_opacity(opacity.get());
            ImageUtils::ImageIterator iter(output.get());
            osg::Vec4 a, b;
            iter.forEachPixel([&]()
                {
                    read_color(a, iter.s(), iter.t());
                    if (opacity.valid())
                    {
                        read_opacity(b, iter.u(), iter.v());
                        a.a() = b[0];
                    }
                    write_output(a, iter.s(), iter.t());
                });
        }
        else
        {
            // default to red
            output = new osg::Image();
            output->allocateImage(1, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE);
            output->setColor(osg::Vec4(1, 0, 0, 1), 0, 0);
        }

        return output;
    }

    osg::ref_ptr<osg::Image> assemble_NORM(osg::ref_ptr<osg::Image> normal)
    {
        osg::ref_ptr<osg::Image> output;
        if (normal.valid())
        {
            return normal;

#if 0
            // disable compression for now b/c it's not working :(
            if (normal->getPixelFormat() == GL_COMPRESSED_RED_GREEN_RGTC2_EXT)
            {
                // DO nothing.
                output = normal;
            }
            else
            {
                output = new osg::Image();
                output->allocateImage(normal->s(), normal->t(), 1, GL_RG, GL_UNSIGNED_BYTE);
                output->setInternalTextureFormat(GL_COMPRESSED_RED_GREEN_RGTC2_EXT);
                ImageUtils::PixelReader read(normal.get());
                ImageUtils::PixelWriter write(output.get());
                osg::Vec4 a, packed;
                osg::Vec3 temp;
                ImageUtils::ImageIterator iter(output.get());
                iter.forEachPixel([&]()
                    {
                        read(a, iter.s(), iter.t());
                        temp.set(a.x() * 2.0f - 1.0f, a.y() * 2.0f - 1.0f, a.z() * 2.0f - 1.0f);
                        NormalMapGenerator::pack(temp, packed);
                        write(packed, iter.s(), iter.t());
                    });
            }
#endif
        }
        else
        {
            osg::ref_ptr<osg::Image> output = new osg::Image();
            output->allocateImage(1, 1, 1, GL_RG, GL_UNSIGNED_BYTE);
            output->setInternalTextureFormat(GL_RG8);
            output->setColor(osg::Vec4(0.5, 0.5, 1.0, 1.0), 0, 0);
        }
        return output;
    }

    osg::ref_ptr<osg::Image> assemble_DRAM(
        osg::ref_ptr<osg::Image>& displacement,
        osg::ref_ptr<osg::Image>& roughness,
        osg::ref_ptr<osg::Image>& ao,
        osg::ref_ptr<osg::Image>& metal)
    {
        osg::ref_ptr<osg::Image> output;

        if (displacement.valid() || roughness.valid() || ao.valid() || metal.valid())
        {
            // use the dimensions of the largest image
            int s = 0, t = 0;
            for (auto& image : { displacement, roughness, ao, metal })
            {
                if (image.valid() && image->s() > s)
                    s = image->s();
                if (image.valid() && image->t() > t)
                    t = image->t();
            }

            if (s > 0 && t > 0)
            {
                output = new osg::Image();
                output->allocateImage(s, t, 1, GL_RGBA, GL_UNSIGNED_BYTE);

                ImageUtils::PixelReader
                    read_displacement(displacement.get()),
                    read_roughness(roughness.get()),
                    read_ao(ao.get()),
                    read_metal(metal.get());

                ImageUtils::PixelWriter write_output(output.get());

                osg::Vec4 in, out;
                ImageUtils::ImageIterator iter(output.get());
                iter.forEachPixel([&]()
                    {
                        if (displacement.valid())
                        {
                            read_displacement(in, iter.u(), iter.v());
                            out.r() = in.r();
                        }
                        else
                        {
                            out.r() = DEFAULT_DISPLACEMENT;
                        }

                        if (roughness.valid())
                        {
                            read_roughness(in, iter.u(), iter.v());
                            out.g() = in.r();
                        }
                        else
                        {
                            out.g() = DEFAULT_ROUGHNESS;
                        }

                        if (ao.valid())
                        {
                            read_ao(in, iter.u(), iter.v());
                            out.b() = in.r();
                        }
                        else
                        {
                            out.b() = DEFAULT_AO;
                        }

                        if (metal.valid())
                        {
                            read_metal(in, iter.u(), iter.v());
                            out.a() = in.r();
                        }
                        else
                        {
                            out.a() = DEFAULT_METAL;
                        }

                        write_output(out, iter.s(), iter.t());
                    });
            }
        }

        if (!output.valid())
        {
            // fallback - use defaults.
            osg::ref_ptr<osg::Image> output = new osg::Image();
            output->allocateImage(1, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE);
            output->setColor(osg::Vec4(DEFAULT_DISPLACEMENT, DEFAULT_ROUGHNESS, DEFAULT_AO, DEFAULT_METAL), 0, 0);
        }

        return output;
    }

#if 0
    osg::ref_ptr<osg::Image> assemble_RGBH(
        osg::ref_ptr<osg::Image> color,
        osg::ref_ptr<osg::Image> height,
        osg::ref_ptr<osg::Image> opacity)
    {
        // output will be input :)
        osg::ref_ptr<osg::Image> output = new osg::Image();
        output->allocateImage(color->s(), color->t(), 1, GL_RGBA, GL_UNSIGNED_BYTE);

        OE_SOFT_ASSERT_AND_RETURN(color.valid(), output, "Color must be present");

        ImageUtils::PixelReader readColor(color.get());
        ImageUtils::PixelReader readOpacity(opacity.get());
        ImageUtils::PixelReader readHeight(height.get());

        ImageUtils::PixelWriter write(output.get());

        osg::Vec4 temp, temp2;
        //float minh = 1.0f, maxh = 0.0f;

        ImageUtils::ImageIterator iter(output.get());
        iter.forEachPixel([&]()
            {
                readColor(temp, iter.s(), iter.t());
                temp2[0] = 1.0f; // default
                if (opacity.valid())
                {
                    readOpacity(temp2, iter.u(), iter.v());
                }
                else if (height.valid())
                {
                    // use (u,v) in case textures are different sizes
                    readHeight(temp2, iter.u(), iter.v());
                }
                temp.a() = temp2[0];

                //minh = std::min(minh, temp.a());
                //maxh = std::max(maxh, temp.a());

                write(temp, iter.s(), iter.t());
            });

#if 0
        //Resize the image to the nearest power of two
        if (!ImageUtils::isPowerOfTwo(output.get()))
        {
            unsigned s = osg::Image::computeNearestPowerOfTwo(output->s());
            unsigned t = osg::Image::computeNearestPowerOfTwo(output->t());
            osg::ref_ptr<osg::Image> resized;
            if (ImageUtils::resizeImage(output.get(), s, t, resized))
                output = resized.release();
        }
#endif

        ImageUtils::compressImageInPlace(output.get(), "cpu");
        ImageUtils::mipmapImageInPlace(output.get());

        return output;
    }

    osg::ref_ptr<osg::Image> assemble_NNRA(
        osg::ref_ptr<osg::Image> color,
        osg::ref_ptr<osg::Image> normals,
        const osg::Vec3f& normal_scale,
        osg::ref_ptr<osg::Image> roughness,
        int roughness_channel,
        bool roughness_inverted,
        osg::ref_ptr<osg::Image> ao,
        int ao_channel)
    {
        int s = color->s();
        int t = color->t();

        osg::ref_ptr<osg::Image> output = new osg::Image();
        output->allocateImage(s, t, 1, GL_RGBA, GL_UNSIGNED_BYTE);

        ImageUtils::PixelReader readNormals(normals.get());
        ImageUtils::PixelReader readRoughness(roughness.get());
        ImageUtils::PixelReader readAO(ao.get());

        ImageUtils::PixelWriter write(output.get());

        osg::Vec3 normal3(0, 0, 1);
        osg::Vec4 normal;
        osg::Vec4 roughnessVal;
        osg::Vec4 aoVal;
        osg::Vec4 packed;

        ImageUtils::ImageIterator iter(output.get());
        iter.forEachPixel([&]()
            {
                if (normals.valid())
                {
                    readNormals(normal, iter.u(), iter.v());

                    if (normals->getPixelFormat() == GL_COMPRESSED_RED_GREEN_RGTC2_EXT)
                    {
                        // do nothing
                        packed.x() = normal.x();
                        packed.y() = normal.y();
                    }
                    else
                    {
                        normal3.set(
                            normal_scale.x() * (normal.x() * 2.0 - 1.0),
                            normal_scale.y() * (normal.y() * 2.0 - 1.0),
                            normal_scale.z() * (normal.z() * 2.0 - 1.0));

                        NormalMapGenerator::pack(normal3, packed);
                    }
                }
                else
                {
                    NormalMapGenerator::pack(normal3, packed);
                }

                if (roughness.valid())
                {
                    readRoughness(roughnessVal, iter.u(), iter.v());
                    if (roughness_inverted)
                        packed[2] = 1.0f - roughnessVal[roughness_channel];
                    else
                        packed[2] = roughnessVal[roughness_channel];
                }
                else packed[2] = DEFAULT_ROUGHNESS;

                if (ao.valid())
                {
                    readAO(aoVal, iter.u(), iter.v());
                    packed[3] = aoVal[ao_channel];
                }
                else packed[3] = DEFAULT_AO;

                write(packed, iter.s(), iter.t());
            });

#if 0
        //Resize the image to the nearest power of two
        if (!ImageUtils::isPowerOfTwo(output.get()))
        {
            unsigned s = osg::Image::computeNearestPowerOfTwo(output->s());
            unsigned t = osg::Image::computeNearestPowerOfTwo(output->t());
            osg::ref_ptr<osg::Image> resized;
            if (ImageUtils::resizeImage(output.get(), s, t, resized, 0u, false))
                output = resized.release();
        }
#endif

        // Do NOT compress this image; it messes with the normal maps.
        ImageUtils::compressImageInPlace(output.get(), "cpu");
        ImageUtils::mipmapImageInPlace(output.get());

        return output;
    }
#endif
}

#include <osgDB/WriteFile>

void
PBRTexture::load(const PBRMaterial& mat, const osgDB::Options* options)
{
    osg::ref_ptr<osg::Image> color_image, normal_image, roughness_image, metal_image, ao_image, displacement_image, opacity_image;
    if (mat.color().isSet()) color_image = mat.color()->getImage(options);
    if (mat.normal().isSet()) normal_image = mat.normal()->getImage(options);
    if (mat.roughness().isSet()) roughness_image = mat.roughness()->getImage(options);
    if (mat.metal().isSet()) metal_image = mat.metal()->getImage(options);
    if (mat.ao().isSet()) ao_image = mat.ao()->getImage(options);
    if (mat.displacement().isSet()) displacement_image = mat.displacement()->getImage(options);
    if (mat.opacity().isSet()) opacity_image = mat.opacity()->getImage(options);

    albedo = new osg::Texture2D(assemble_RGBA(color_image, opacity_image));
    albedo->setName(mat.name() + " albedo");

    normal = new osg::Texture2D(assemble_NORM(normal_image));
    normal->setName(mat.name() + " normal");

    pbr = new osg::Texture2D(assemble_DRAM(displacement_image, roughness_image, ao_image, metal_image));
    pbr->setName(mat.name() + " PBR");

    for (auto& tex : { albedo, normal, pbr })
    {
        tex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
        tex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
        tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
        tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
        tex->setMaxAnisotropy(4.0f);
    }
}
