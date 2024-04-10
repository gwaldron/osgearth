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
    const float DEFAULT_ROUGHNESS = 0.5f;
    const float DEFAULT_AO = 1.0f;

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
        ImageUtils::PixelReader readHeight(height.get());
        ImageUtils::PixelReader readOpacity(opacity.get());

        ImageUtils::PixelWriter write(output.get());

        osg::Vec4 temp, temp2;
        //float minh = 1.0f, maxh = 0.0f;

        ImageUtils::ImageIterator iter(output.get());
        iter.forEachPixel([&]()
            {
                readColor(temp, iter.s(), iter.t());
                temp2[0] = 1.0f; // default
                if (height.valid())
                {
                    // use (u,v) in case textures are different sizes
                    readHeight(temp2, iter.u(), iter.v());
                }
                else if (opacity.valid())
                {
                    readOpacity(temp2, iter.u(), iter.v());
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
}


osg::ref_ptr<osg::Texture>
PBRMaterial::createTexture(const osgDB::Options* options) const
{
    osg::ref_ptr<osg::Image> color_image, normal_image, roughness_image, ao_image, height_image, opacity_image;
    if (color().isSet()) color_image = color()->getImage(options);
    if (normal().isSet()) normal_image = normal()->getImage(options);
    if (roughness().isSet()) roughness_image = roughness()->getImage(options);
    if (ao().isSet()) ao_image = ao()->getImage(options);
    if (height().isSet()) height_image = height()->getImage(options);
    if (opacity().isSet()) opacity_image = opacity()->getImage(options);

    auto rgbh = assemble_RGBH(color_image, height_image, opacity_image);
    auto nnra = assemble_NNRA(color_image, normal_image, { 1,1,1 }, roughness_image, 0, false, ao_image, 0);

#if 0
    auto tex = new osg::Texture2D(rgbh);
    //tex->setInternalFormat(GL_RGBA8);
    tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
    tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
    return tex;
#else
    osg::ref_ptr<osg::Texture2DArray> tex = new osg::Texture2DArray();
    tex->setTextureSize(rgbh->s(), rgbh->t(), 2);
    tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
    tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
    tex->setImage(0, rgbh);
    tex->setImage(1, nnra);

    // TODO: this is a temporary hack so the shader generator can detect a PBR material.
    tex->setName("PBRMaterial");

    return tex;
#endif
}
