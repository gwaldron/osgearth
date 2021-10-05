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
#include "TextureSplattingMaterials"
#include <osgEarth/URI>
#include <osgEarth/ImageUtils>
#include <osgEarth/Elevation>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/FileNameUtils>

#define LC "[TextureSplattingMaterials] "

using namespace osgEarth;
using namespace osgEarth::Procedural;

#define DEFAULT_ROUGHNESS 0.75
#define DEFAULT_AO 1.0

REGISTER_OSGPLUGIN(oe_splat_rgbh, RGBH_Loader)

RGBH_Loader::RGBH_Loader() :
    osgDB::ReaderWriter()
{
    supportsExtension("oe_splat_rgbh", "OE Encoded Splat Texture RGBH");
}

osgDB::ReaderWriter::ReadResult
RGBH_Loader::readImage(
    const std::string& filename,
    const osgDB::Options* options) const
{
    std::string ext = osgDB::getFileExtension(filename);
    if (ext != "oe_splat_rgbh")
        return ReadResult::FILE_NOT_HANDLED;

    ReadResult rr = readImageEncoded(filename, options);

    return rr.success() ? rr : readImageFromSourceData(
        osgDB::getNameLessExtension(filename), 
        options);
}

namespace
{
    osgDB::ReaderWriter::ReadResult assemble_RGBH(
        osg::ref_ptr<osg::Image> color,
        osg::ref_ptr<osg::Image> height,
        int height_channel)
    {
        osg::ref_ptr<osg::Image> output;
        
        if (height.valid() || color->getPixelFormat() != GL_RGBA)
        {
            output = new osg::Image();
            output->allocateImage(color->s(), color->t(), 1, GL_RGBA, GL_UNSIGNED_BYTE);

            ImageUtils::PixelReader readColor(color.get());
            ImageUtils::PixelReader readHeight(height.get());

            ImageUtils::PixelWriter write(output.get());

            osg::Vec4 temp, temp2;
            float minh = 1.0f, maxh = 0.0f;

            ImageUtils::ImageIterator iter(output.get());
            iter.forEachPixel([&]()
                {
                    readColor(temp, iter.s(), iter.t());
                    if (height.valid())
                    {
                        // use (u,v) in case textures are different sizes
                        readHeight(temp2, iter.u(), iter.v());
                        //readHeight(temp2, iter.s(), iter.t());
                        temp.a() = temp2[height_channel];
                    }
                    else
                    {
                        temp.a() = 0.0f; // default height
                    }

                    minh = osg::minimum(minh, temp.a());
                    maxh = osg::maximum(maxh, temp.a());

                    write(temp, iter.s(), iter.t());
                });
        }
        else
        {
            output = color;
        }

        //Resize the image to the nearest power of two
        if (!ImageUtils::isPowerOfTwo(output.get()))
        {
            unsigned s = osg::Image::computeNearestPowerOfTwo(output->s());
            unsigned t = osg::Image::computeNearestPowerOfTwo(output->t());
            osg::ref_ptr<osg::Image> resized;
            if (ImageUtils::resizeImage(output.get(), s, t, resized))
                output = resized.release();
        }

        ImageUtils::compressImageInPlace(output.get(), "cpu");

        return output;
    }

    osgDB::ReaderWriter::ReadResult assemble_NNRA(
        osg::ref_ptr<osg::Image> normals,
        const osg::Vec3f& normal_scale,
        osg::ref_ptr<osg::Image> roughness,
        int roughness_channel,
        bool roughness_inverted,
        osg::ref_ptr<osg::Image> ao,
        int ao_channel)
    {
        int s = normals.valid() ? normals->s() :
            roughness.valid() ? roughness->s() :
            ao.valid() ? ao->s() :
            1;
        int t = normals.valid() ? normals->t() :
            roughness.valid() ? roughness->t() :
            ao.valid() ? ao->t() :
            1;

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
                    //readNormals(normal, iter.s(), iter.t());
                    readNormals(normal, iter.u(), iter.v());

                    // Note: Y-down is standard practice for normal maps
                    normal3.set(
                        normal_scale.x() * (normal.x()*2.0 - 1.0),
                        normal_scale.y() * (normal.y()*2.0 - 1.0),
                        normal_scale.z() * (normal.z()*2.0 - 1.0));
                }
                NormalMapGenerator::pack(normal3, packed);

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

        //Resize the image to the nearest power of two
        if (!ImageUtils::isPowerOfTwo(output.get()))
        {
            unsigned s = osg::Image::computeNearestPowerOfTwo(output->s());
            unsigned t = osg::Image::computeNearestPowerOfTwo(output->t());
            osg::ref_ptr<osg::Image> resized;
            if (ImageUtils::resizeImage(output.get(), s, t, resized))
                output = resized.release();
        }

        // Do NOT compress this image; it messes with the normal maps.
        //ImageUtils::compressImageInPlace(output.get(), "cpu");
        //ImageUtils::mipmapImageInPlace(output.get());

        return output;
    }
}

osgDB::ReaderWriter::ReadResult
RGBH_Loader::readImageFromSourceData(
    const std::string& color_filename,
    const osgDB::Options* options) const
{
    // isolate common image extension
    std::string extension = osgDB::getLowerCaseFileExtension(color_filename);
    std::string basename = osgDB::getNameLessExtension(color_filename);

    osg::ref_ptr<osg::Image> color;

    // attempt to read "materialize" file layout. This includes
    //   filename_Color.jpg (albedo)
    //   filename_Displacement.jpg (height)
    if (Strings::endsWith(basename, "_Color", false))
    {
        URI colorURI(color_filename);
        color = colorURI.getImage(options);
        if (color.valid())
        {
            basename = basename.substr(0, basename.length() - 6); // strip "_Color"
            URI heightURI(basename + "_Displacement." + extension);
            osg::ref_ptr<osg::Image> height = heightURI.getImage(options);
            if (!height.valid())
            {
                OE_WARN << LC << "Failed to load \"" << heightURI.full() << "\"" << std::endl;
            }

            return assemble_RGBH(color, height, 0); // height is in RED
        }
        else
        {
            OE_WARN << LC << "Failed to load \"" << colorURI.full() << "\"" << std::endl;
        }
    }

    // failing that attempt to read the "vtm" layout. This includes
    //   filename.png (color)
    //   filename_MTL_GLS_AO.png (R=metal, G=smoothness, B=ao)
    //   filename_NML.png (normal)
    {
        URI colorURI(color_filename);
        color = colorURI.getImage(options);
        if (color.valid())
        {
            URI heightURI(basename + "_HGT." + extension);
            osg::ref_ptr<osg::Image> height = heightURI.getImage(options);
            return assemble_RGBH(color, height, 0);
        }
        else
        {
            OE_WARN << LC << "Failed to load \"" << colorURI.full() << "\"" << std::endl;
        }
    }

    return ReadResult::FILE_NOT_FOUND;
}

osgDB::ReaderWriter::ReadResult
RGBH_Loader::readImageEncoded(
    const std::string& filename,
    const osgDB::Options* options) const
{
    osgDB::ReaderWriter* rw = osgDB::Registry::instance()->getReaderWriterForExtension("osgb");
    std::ifstream f(filename, std::ios::binary);
    return rw ? rw->readImage(f, options) : ReadResult::FILE_NOT_HANDLED;
}

osgDB::ReaderWriter::WriteResult
RGBH_Loader::writeImage(
    const osg::Image& image,
    const std::string& filename,
    const osgDB::Options* options) const
{
    if (osgDB::getFileExtension(filename) != "oe_splat_rgbh")
        return WriteResult::FILE_NOT_HANDLED;

    osgDB::ReaderWriter* rw = osgDB::Registry::instance()->getReaderWriterForExtension("osgb");
    if (!rw)
        return WriteResult::NOT_IMPLEMENTED;

    if (image.isCompressed() == false)
    {
        osg::ref_ptr<osg::Image> c = osg::clone(&image, osg::CopyOp::DEEP_COPY_ALL);
        if (!c.valid())
            return WriteResult::ERROR_IN_WRITING_FILE;

        ImageUtils::compressImageInPlace(c.get(), "cpu");
        std::ofstream f(filename, std::ios::binary);
        return rw->writeImage(*c.get(), f, options);
    }
    else
    {
        std::ofstream f(filename, std::ios::binary);
        return rw->writeImage(image, f, options);
    }
}



REGISTER_OSGPLUGIN(oe_splat_nnra, NNRA_Loader)

NNRA_Loader::NNRA_Loader() :
    osgDB::ReaderWriter()
{
    supportsExtension("oe_splat_nnra", "OE Encoded Splat Texture NNRA");
}

osgDB::ReaderWriter::ReadResult
NNRA_Loader::readImage(
    const std::string& filename,
    const osgDB::Options* options) const
{
    if (osgDB::getFileExtension(filename) != "oe_splat_nnra")
        return ReadResult::FILE_NOT_HANDLED;

    ReadResult rr = readImageEncoded(filename, options);

    return rr.success() ? rr : readImageFromSourceData(
        osgDB::getNameLessExtension(filename), 
        options);
}

osgDB::ReaderWriter::ReadResult
NNRA_Loader::readImageFromSourceData(
    const std::string& color_filename,
    const osgDB::Options* options) const
{
    // isolate common image extension
    std::string extension = osgDB::getLowerCaseFileExtension(color_filename);
    std::string basename = osgDB::getNameLessExtension(color_filename);

    osg::ref_ptr<osg::Image> normals, roughness, ao;

    // Try the "materialize" layout first. Files include:
    //   filename_Normal.jpg (normals, X -Y Z)
    //   filename_Roughness.jpg (roughness)
    //   filename_AmbientOcclusion (ao)
    if (Strings::endsWith(basename, "_Color", false))
    {
        basename = basename.substr(0, basename.length() - 6); // strip "_Color"

        URI normalsURI(basename + "_Normal." + extension);
        URI roughnessURI(basename + "_Roughness." + extension);
        URI aoURI(basename + "_AmbientOcclusion." + extension);

        normals = normalsURI.getImage(options);
        if (!normals.valid())
            OE_WARN << LC << "Failed to load \"" << normalsURI.full() << "\"" << std::endl;

        roughness = roughnessURI.getImage(options);
        if (!normals.valid())
            OE_WARN << LC << "Failed to load \"" << roughnessURI.full() << "\"" << std::endl;

        ao = aoURI.getImage(options);
        if (!normals.valid())
            OE_WARN << LC << "Failed to load \"" << aoURI.full() << "\"" << std::endl;

        if (normals.valid() || roughness.valid() || ao.valid())
        {
            return assemble_NNRA(
                normals, osg::Vec3f(1.0f, -1.0f, 1.0f), // -Y
                roughness, 0, false,
                ao, 0);
        }
    }

    // Try the "vtm" layout next. Files include:
    //   filename_MTL_GLS_AO.png (metal, inverse roughness, ao)
    {
        URI normalsURI(basename + "_NML." + extension);
        URI metalGlossAoURI(basename + "_MTL_GLS_AO." + extension);

        normals = normalsURI.getImage(options);
        if (!normals.valid())
            OE_WARN << LC << "Failed to load \"" << normalsURI.full() << "\"" << std::endl;

        roughness = metalGlossAoURI.getImage(options);
        if (!normals.valid())
            OE_WARN << LC << "Failed to load \"" << metalGlossAoURI.full() << "\"" << std::endl;

        ao = roughness;

        if (normals.valid() || roughness.valid() || ao.valid())
        {
            return assemble_NNRA(
                normals, osg::Vec3f(1.0f, 1.0f, 1.0f),
                roughness, 1, true,
                ao, 2);
        }
    }

    return ReadResult::FILE_NOT_FOUND;
}

osgDB::ReaderWriter::ReadResult
NNRA_Loader::readImageEncoded(
    const std::string& filename,
    const osgDB::Options* options) const
{
    osgDB::ReaderWriter* rw = osgDB::Registry::instance()->getReaderWriterForExtension("osgb");
    std::ifstream f(filename, std::ios::binary);
    return rw ? rw->readImage(f, options) : ReadResult::FILE_NOT_HANDLED;
}

osgDB::ReaderWriter::WriteResult
NNRA_Loader::writeImage(
    const osg::Image& image,
    const std::string& filename,
    const osgDB::Options* options) const
{
    if (osgDB::getFileExtension(filename) != "oe_splat_nnra")
        return WriteResult::FILE_NOT_HANDLED;

    osgDB::ReaderWriter* rw = osgDB::Registry::instance()->getReaderWriterForExtension("osgb");
    if (!rw)
        return WriteResult::ERROR_IN_WRITING_FILE;

    // Do NOT compress, normal maps don't like it

    std::ofstream f(filename, std::ios::binary);
    return rw->writeImage(image, f, options);
}
