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
    if (osgDB::getFileExtension(filename) != "oe_splat_rgbh")
        return ReadResult::FILE_NOT_HANDLED;

    std::string fn = osgDB::getNameLessExtension(filename);
    ReadResult rr = readImageEncoded(filename, options);
    return rr.success() ? rr : readImageFromSourceData(fn + ".jpg", options);
}

osgDB::ReaderWriter::ReadResult
RGBH_Loader::readImageFromSourceData(
    const std::string& filename,
    const osgDB::Options* options) const
{
    // isolate common image extension
    std::string extension = osgDB::getLowerCaseFileExtension(filename);
    // isolate base filename
    std::string basename = osgDB::getNameLessExtension(filename);

    URI colorURI(basename + "_Color." + extension);
    URI heightURI(basename + "_Displacement." + extension);

    osg::ref_ptr<osg::Image> color = colorURI.getImage(options);
    if (!color.valid())
        return ReadResult::FILE_NOT_FOUND;

    osg::ref_ptr<osg::Image> height = heightURI.getImage(options);

    osg::ref_ptr<osg::Image> output = new osg::Image();
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
                readHeight(temp2, iter.s(), iter.t());
                temp.a() = temp2.r();
            }
            else
            {
                temp.a() = 0.0f;
            }

            minh = osg::minimum(minh, temp.a());
            maxh = osg::maximum(maxh, temp.a());

            write(temp, iter.s(), iter.t());
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

    ImageUtils::compressImageInPlace(output.get(), "cpu");

    return output;
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

    std::string fn = osgDB::getNameLessExtension(filename);
    ReadResult rr = readImageEncoded(filename, options);
    return rr.success() ? rr : readImageFromSourceData(fn + ".jpg", options);
}

osgDB::ReaderWriter::ReadResult
NNRA_Loader::readImageFromSourceData(
    const std::string& filename,
    const osgDB::Options* options) const
{
    // isolate common image extension
    std::string extension = osgDB::getLowerCaseFileExtension(filename);
    // isolate base filename
    std::string basename = osgDB::getNameLessExtension(filename);

    URI normalsURI(basename + "_Normal." + extension);
    URI roughnessURI(basename + "_Roughness." + extension);
    URI aoURI(basename + "_AmbientOcclusion." + extension);

    osg::ref_ptr<osg::Image> normals = normalsURI.getImage(options);
    osg::ref_ptr<osg::Image> roughness = roughnessURI.getImage(options);
    osg::ref_ptr<osg::Image> ao = aoURI.getImage(options);

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
                readNormals(normal, iter.s(), iter.t());

                // Note: Y-down is standard practice for normal maps
                normal3.set(
                    normal.x()*2.0 - 1.0, 
                    -(normal.y()*2.0 - 1.0),
                    normal.z()*2.0 - 1.0);
            }
            NormalMapGenerator::pack(normal3, packed);

            if (roughness.valid())
            {
                readRoughness(roughnessVal, iter.s(), iter.t());
                packed[2] = roughnessVal.r();
            }
            else packed[2] = DEFAULT_ROUGHNESS;

            if (ao.valid())
            {
                readAO(aoVal, iter.s(), iter.t());
                packed[3] = aoVal.r();
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
    //if (image.isCompressed() == false)
    //{
    //    osg::ref_ptr<osg::Image> c = osg::clone(&image, osg::CopyOp::DEEP_COPY_ALL);
    //    OE_SOFT_ASSERT_AND_RETURN(c.valid(), WriteResult::ERROR_IN_WRITING_FILE);

    //    ImageUtils::compressImageInPlace(c.get(), "cpu");
    //    std::ofstream f(filename, std::ios::binary);
    //    return rw->writeImage(*c.get(), f, options);
    //}
    //else
    {
        std::ofstream f(filename, std::ios::binary);
        return rw->writeImage(image, f, options);
    }
}
