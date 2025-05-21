/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "DecalLayer"
#include <osgEarth/Map>
#include <osgEarth/Profile>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/Color>
#include <osg/BlendEquation>

using namespace osgEarth;
using namespace osgEarth::Contrib;

#define LC "[DecalImageLayer] "

REGISTER_OSGEARTH_LAYER(decalimage, DecalImageLayer);

Config
DecalImageLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    return conf;
}

void
DecalImageLayer::Options::fromConfig(const Config& conf)
{
    //nop
}

//........................................................................

void
DecalImageLayer::init()
{
    ImageLayer::init();

    // Set the layer profile.
    setProfile(Profile::create(Profile::GLOBAL_GEODETIC));

    // Never cache decals
    layerHints().cachePolicy() = CachePolicy::NO_CACHE;

    // blending defaults
    _src[0] = GL_SRC_ALPHA;
    _dst[0] = GL_ONE_MINUS_SRC_ALPHA;
    _src[1] = GL_SRC_ALPHA;
    _dst[1] = GL_ONE_MINUS_SRC_ALPHA;
    _src[2] = GL_SRC_ALPHA;
    _dst[2] = GL_ONE_MINUS_SRC_ALPHA;
    _src[3] = GL_ONE;
    _dst[3] = GL_ZERO;
    _equation[0] = GL_FUNC_ADD;
    _equation[1] = GL_FUNC_ADD;
    _equation[2] = GL_FUNC_ADD;
    _equation[3] = GL_FUNC_ADD;
}

void
DecalImageLayer::setBlendFuncs(
    GLenum R_source, GLenum R_dest,
    GLenum G_source, GLenum G_dest,
    GLenum B_source, GLenum B_dest,
    GLenum A_source, GLenum A_dest)
{
    _src[0] = R_source; _dst[0] = R_dest;
    _src[1] = G_source; _dst[1] = G_dest;
    _src[2] = B_source; _dst[2] = B_dest;
    _src[3] = A_source; _dst[3] = A_dest;
}

void
DecalImageLayer::setBlendEquations(
    GLenum R_equation,
    GLenum G_equation,
    GLenum B_equation,
    GLenum A_equation)
{
    _equation[0] = R_equation;
    _equation[1] = G_equation;
    _equation[2] = B_equation;
    _equation[3] = A_equation;
}

namespace
{
    template<typename T>
    inline float get_blend(GLenum blend, const T& src, const T& dst) {
        return
            blend == GL_SRC_ALPHA ? src.a() :
            blend == GL_ONE_MINUS_SRC_ALPHA ? (1.0f - src.a()) :
            blend == GL_DST_ALPHA ? dst.a() :
            blend == GL_ONE_MINUS_DST_ALPHA ? (1.0f - dst.a()) :
            blend == GL_ONE ? 1.0f :
            blend == GL_ZERO ? 0.0f :
            1.0f;
    }
}

GeoImage
DecalImageLayer::createImageImplementation(
    const GeoImage& canvas,
    const TileKey& key,
    ProgressCallback* progress) const
{
    std::vector<Decal> decals;
    std::vector<GeoExtent> outputExtentsInDecalSRS;
    std::vector<GeoExtent> intersections;

    const GeoExtent& outputExtent = key.getExtent();

    // thread-safe collection of intersecting decals
    {
        Threading::ScopedReadLock lock(_data_mutex);

        for (auto& decal : _decalList)
        {
            GeoExtent outputExtentInDecalSRS = outputExtent.transform(decal._extent.getSRS());
            GeoExtent intersectionExtent = decal._extent.intersectionSameSRS(outputExtentInDecalSRS);
            if (intersectionExtent.isValid())
            {
                decals.push_back(decal);
                outputExtentsInDecalSRS.push_back(outputExtentInDecalSRS);
                intersections.push_back(intersectionExtent);
            }
        }
    }

    if (decals.empty())
        return canvas;

    osg::ref_ptr<osg::Image> output = new osg::Image();
    output->allocateImage(getTileSize(), getTileSize(), 1, GL_RGBA, GL_UNSIGNED_BYTE);
    output->setInternalTextureFormat(GL_RGBA8);

    // Read and write from the output:
    ImageUtils::PixelWriter writeOutput(output.get());
    ImageUtils::PixelReader readOutput(output.get());

    osg::Vec4 src, dst, out;

    // Start by copying the canvas to the output. Use a scale/bias
    // since the canvas might be larger (lower resolution) than the
    // tile we are building.
    if (canvas.valid())
    {
        // Canvas reader with the appropriate scale/bias matrix
        ImageUtils::PixelReader readCanvas(canvas.getImage());
        osg::Matrix csb;
        key.getExtent().createScaleBias(canvas.getExtent(), csb);

        writeOutput.forEachPixel([&](auto& iter)
            {
                double cu = iter.u() * csb(0, 0) + csb(3, 0);
                double cv = iter.v() * csb(1, 1) + csb(3, 1);
                readCanvas(dst, cu, cv);
                writeOutput(dst, iter);
            });
    }
    else
    {
        ::memset(output->data(), 0, output->getTotalSizeInBytes());
    }

    // for each decal...
    for (unsigned d = 0; d < decals.size(); ++d)
    {
        const Decal& decal = decals[d];
        const GeoExtent& decalExtent = decal._extent;
        ImageUtils::PixelReader readInput(decal._image.get());
        const GeoExtent& outputExtentInDecalSRS = outputExtentsInDecalSRS[d];
        const GeoExtent& intersection = intersections[d];
        bool normalizeX = decalExtent.crossesAntimeridian();

        for (unsigned t = 0; t < (unsigned)output->t(); ++t)
        {
            double out_v = (double)t / (double)(output->t() - 1);
            double out_y = outputExtentInDecalSRS.yMin() + (double)out_v * outputExtentInDecalSRS.height();

            double in_v = (out_y - decalExtent.yMin()) / decalExtent.height();

            // early out if we're outside the decal's extent
            if (in_v < 0.0 || in_v > 1.0)
                continue;

            for (unsigned s = 0; s < (unsigned)output->s(); ++s)
            {
                double out_u = (double)s / (double)(output->s() - 1);
                double out_x = outputExtentInDecalSRS.xMin() + (double)out_u * outputExtentInDecalSRS.width();

                if (normalizeX)
                {
                    while (out_x < decalExtent.xMin())
                        out_x += 360.0;
                    while (out_x > decalExtent.xMax())
                        out_x -= 360.0;
                }

                double in_u = (out_x - decalExtent.xMin()) / decalExtent.width();

                // early out if we're outside the decal's extent
                if (in_u < 0.0 || in_u > 1.0)
                    continue;

                // read the existing data and the new decal input:
                readOutput(dst, out_u, out_v);
                readInput(src, in_u, in_v);

                // figure out how to blend them:
                for (int n = 0; n < 4; ++n)
                {
                    float sf = get_blend(_src[n], src, dst);
                    float df = get_blend(_dst[n], src, dst);

                    if (_equation[n] == GL_FUNC_ADD)
                    {
                        out[n] = src[n] * sf + dst[n] * df;
                    }
                    else if (_equation[n] == GL_MAX)
                    {
                        out[n] = std::max(src[n] * sf, dst[n] * df);
                    }
                    else if (_equation[n] == GL_MIN)
                    {
                        out[n] = std::min(src[n] * sf, dst[n] * df);
                    }

                    out[n] = clamp(out[n], 0.0f, 1.0f);
                }

                writeOutput(out, s, t);
            }
        }
    }

    return GeoImage(output.get(), outputExtent);
}

GeoImage
DecalImageLayer::createImageImplementation(
    const TileKey& key, 
    ProgressCallback* progress) const
{
    static GeoImage s_empty;
    return createImageImplementation(s_empty, key, progress);
}

bool
DecalImageLayer::addDecal(const std::string& id, const GeoExtent& extent, const osg::Image* image)
{
    Threading::ScopedWriteLock lock(_data_mutex);

    DecalIndex::iterator i = _decalIndex.find(id);
    if (i != _decalIndex.end())
        return false;

    _decalList.push_back(Decal());
    Decal& decal = _decalList.back();
    decal._extent = extent;
    decal._image = image;

    _decalIndex[id] = --_decalList.end();

    // Update the data extents
    addDataExtent(getProfile()->clampAndTransformExtent(extent));

    // data changed so up the revsion.
    bumpRevision();
    return true;
}

void
DecalImageLayer::removeDecal(const std::string& id)
{
    Threading::ScopedWriteLock lock(_data_mutex);

    DecalIndex::iterator i = _decalIndex.find(id);
    if (i != _decalIndex.end())
    {
        _decalList.erase(i->second);
        _decalIndex.erase(i);

        // Rebuild the data extents
        DataExtentList dataExtents;
        for (auto& decal : _decalList)
            dataExtents.push_back(getProfile()->clampAndTransformExtent(decal._extent));
        setDataExtents(dataExtents);

        // data changed so up the revsion.
        bumpRevision();
    }
}

const GeoExtent&
DecalImageLayer::getDecalExtent(const std::string& id) const
{
    Threading::ScopedReadLock lock(_data_mutex);
    DecalIndex::const_iterator i = _decalIndex.find(id);
    if (i != _decalIndex.end())
    {
        return i->second->_extent;
    }
    return GeoExtent::INVALID;
}

void
DecalImageLayer::clearDecals()
{
    Threading::ScopedWriteLock lock(_data_mutex);
    _decalIndex.clear();
    _decalList.clear();
    // Clear the data extents
    DataExtentList dataExtents;
    setDataExtents(dataExtents);
    bumpRevision();
}

//........................................................................

#undef  LC
#define LC "[DecalElevationLayer] "

REGISTER_OSGEARTH_LAYER(decalelevation, DecalElevationLayer);


Config
DecalElevationLayer::Options::getConfig() const
{
    Config conf = ElevationLayer::Options::getConfig();
    return conf;
}

void
DecalElevationLayer::Options::fromConfig(const Config& conf)
{
    //nop
}

//........................................................................

void
DecalElevationLayer::init()
{
    ElevationLayer::init();

    // Set the layer profile.
    setProfile(Profile::create(Profile::GLOBAL_GEODETIC));

    // This is an offset layer (the elevation values are offsets)
    setInterpretValuesAsOffsets(true);

    // Never cache decals
    layerHints().cachePolicy() = CachePolicy::NO_CACHE;
}

GeoHeightField
DecalElevationLayer::createHeightFieldImplementation(const TileKey& key, ProgressCallback* progress) const
{
    std::vector<Decal> decals;
    std::vector<GeoExtent> outputExtentsInDecalSRS;
    std::vector<GeoExtent> intersections;

    const GeoExtent& outputExtent = key.getExtent();

    // thread-safe collection of intersecting decals
    {
        Threading::ScopedReadLock lock(_data_mutex);

        for(auto& decal : _decalList)
        {
            GeoExtent outputExtentInDecalSRS = outputExtent.transform(decal._heightfield.getExtent().getSRS());
            GeoExtent intersectionExtent = decal._heightfield.getExtent().intersectionSameSRS(outputExtentInDecalSRS);
            if (intersectionExtent.isValid())
            {
                decals.push_back(decal);
                outputExtentsInDecalSRS.push_back(outputExtentInDecalSRS);
                intersections.push_back(intersectionExtent);
            }
        }
    }

    if (decals.empty())
        return GeoHeightField::INVALID;

    osg::ref_ptr<osg::HeightField> output = new osg::HeightField();
    output->allocate(getTileSize(), getTileSize());
    output->getFloatArray()->assign(output->getFloatArray()->size(), 0.0f);
    unsigned writes = 0u;

    for(unsigned i=0; i<decals.size(); ++i)
    {
        const Decal& decal = decals[i];

        const GeoExtent& decalExtent = decal._heightfield.getExtent();
        const GeoExtent& outputExtentInDecalSRS = outputExtentsInDecalSRS[i];
        const GeoExtent& intersection = intersections[i];
        const osg::HeightField* decal_hf = decal._heightfield.getHeightField();

        double xInterval = outputExtentInDecalSRS.width() / (double)(output->getNumColumns()-1);
        double yInterval = outputExtentInDecalSRS.height() / (double)(output->getNumRows()-1);

        for(unsigned row=0; row<output->getNumRows(); ++row)
        {
            double y = outputExtentInDecalSRS.yMin() + yInterval*(double)row;
            double v = (y-outputExtentInDecalSRS.yMin())/outputExtentInDecalSRS.height();

            for(unsigned col=0; col<output->getNumColumns(); ++col)
            {
                double x = outputExtentInDecalSRS.xMin() + xInterval*(double)col;
                double u = (x-outputExtentInDecalSRS.xMin())/outputExtentInDecalSRS.width();

                if (intersection.contains(x, y))
                {
                    double uu = (x-decalExtent.xMin())/decalExtent.width();
                    double vv = (y-decalExtent.yMin())/decalExtent.height();

                    float h_prev = HeightFieldUtils::getHeightAtNormalizedLocation(output.get(), u, v);

                    float h = HeightFieldUtils::getHeightAtNormalizedLocation(decal_hf, uu, vv);

                    // "blend" heights together by adding them.
                    if (h != NO_DATA_VALUE)
                    {
                        float final_h = h_prev != NO_DATA_VALUE ? h+h_prev : h;
                        output->setHeight(col, row, final_h);
                        ++writes;
                    }
                }
            }
        }
    }

    return writes > 0u ? GeoHeightField(output.get(), outputExtent) : GeoHeightField::INVALID;
}

bool
DecalElevationLayer::addDecal(
    const std::string& id, 
    const GeoExtent& extent,
    const osg::Image* image, 
    float scale,
    GLenum channel)
{
    if (!extent.isValid() || !image)
        return false;

    Threading::ScopedWriteLock lock(_data_mutex);

    DecalIndex::iterator i = _decalIndex.find(id);
    if (i != _decalIndex.end())
        return false;

    osg::HeightField* hf = new osg::HeightField();
    hf->allocate(image->s(), image->t());

    ImageUtils::PixelReader read(image);

    unsigned c =
        channel == GL_RED   ? 0u :
        channel == GL_GREEN ? 1u :
        channel == GL_BLUE  ? 2u :
        3u;

    c = std::min(c, osg::Image::computeNumComponents(image->getPixelFormat())-1u);

    // scale up the values so that [0...1/2] is below ground
    // and [1/2...1] is above ground.
    osg::Vec4 value;
    for(int t=0; t<read.t(); ++t)
    {
        for(int s=0; s<read.s(); ++s)
        {
            read(value, s, t);
            float h = scale * value[c];
            hf->setHeight(s, t, h);
        }
    }

    _decalList.push_back(Decal());
    Decal& decal = _decalList.back();
    decal._heightfield = GeoHeightField(hf, extent);

    _decalIndex[id] = --_decalList.end();

    // Update the data extents
    addDataExtent(getProfile()->clampAndTransformExtent(extent));

    // data changed so up the revsion.
    bumpRevision();
    return true;
}

bool
DecalElevationLayer::addDecal(
    const std::string& id, 
    const GeoExtent& extent, 
    const osg::Image* image, 
    float minOffset, 
    float maxOffset,
    GLenum channel)
{
    if (!extent.isValid() || !image)
        return false;

    Threading::ScopedWriteLock lock(_data_mutex);

    DecalIndex::iterator i = _decalIndex.find(id);
    if (i != _decalIndex.end())
        return false;

    osg::HeightField* hf = new osg::HeightField();
    hf->allocate(image->s(), image->t());

    ImageUtils::PixelReader read(image);

    unsigned c =
        channel == GL_RED   ? 0u :
        channel == GL_GREEN ? 1u :
        channel == GL_BLUE  ? 2u :
        3u;

    c = std::min(c, osg::Image::computeNumComponents(image->getPixelFormat())-1u);

    osg::Vec4 value;
    for(int t=0; t<read.t(); ++t)
    {
        for(int s=0; s<read.s(); ++s)
        {
            read(value, s, t);
            float h = minOffset + (maxOffset-minOffset)*value[c];
            hf->setHeight(s, t, h);
        }
    }

    _decalList.push_back(Decal());
    Decal& decal = _decalList.back();
    decal._heightfield = GeoHeightField(hf, extent);

    _decalIndex[id] = --_decalList.end();

    addDataExtent(getProfile()->clampAndTransformExtent(extent));

    // data changed so up the revsion.
    bumpRevision();
    return true;
}

void
DecalElevationLayer::removeDecal(const std::string& id)
{
    Threading::ScopedWriteLock lock(_data_mutex);

    DecalIndex::iterator i = _decalIndex.find(id);
    if (i != _decalIndex.end())
    {
        _decalList.erase(i->second);
        _decalIndex.erase(i);

        DataExtentList dataExtents;
        for (auto& decal : _decalList)
            dataExtents.push_back(getProfile()->clampAndTransformExtent(decal._heightfield.getExtent()));
        setDataExtents(dataExtents);

        // data changed so up the revsion.
        bumpRevision();
    }
}

const GeoExtent&
DecalElevationLayer::getDecalExtent(const std::string& id) const
{
    Threading::ScopedReadLock lock(_data_mutex);

    DecalIndex::const_iterator i = _decalIndex.find(id);
    if (i != _decalIndex.end())
    {
        return i->second->_heightfield.getExtent();
    }
    return GeoExtent::INVALID;
}

void
DecalElevationLayer::clearDecals()
{
    Threading::ScopedWriteLock lock(_data_mutex);
    _decalIndex.clear();
    _decalList.clear();
    // Clear the data extents
    DataExtentList dataExtents;
    setDataExtents(dataExtents);
    bumpRevision();
}

//........................................................................


REGISTER_OSGEARTH_LAYER(decallandcover, DecalLandCoverLayer);

Config
DecalLandCoverLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    return conf;
}

void
DecalLandCoverLayer::Options::fromConfig(const Config& conf)
{
    //nop
}

//........................................................................

void
DecalLandCoverLayer::init()
{
    LandCoverLayer::init();

    // Set the layer profile.
    setProfile(Profile::create(Profile::GLOBAL_GEODETIC));

    // Never cache decals
    layerHints().cachePolicy() = CachePolicy::NO_CACHE;
}

Status
DecalLandCoverLayer::openImplementation()
{
    // skip LandCoverLayer::openImplementation because we're replacing it
    Status parent = ImageLayer::openImplementation();
    if (parent.isError())
        return parent;

    const Profile* profile = getProfile();
    if (!profile)
    {
        profile = Profile::create(Profile::GLOBAL_GEODETIC);
        setProfile(profile);
    }

    return Status::NoError;
}

GeoImage
DecalLandCoverLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
    std::vector<Decal> decals;
    std::vector<GeoExtent> outputExtentsInDecalSRS;
    std::vector<GeoExtent> intersections;

    const GeoExtent& outputExtent = key.getExtent();

    // thread-safe collection of intersecting decals
    {
        Threading::ScopedReadLock lock(_data_mutex);

        for(auto& decal : _decalList)
        {
            const GeoExtent& decalExtent = decal._extent;
            GeoExtent outputExtentInDecalSRS = outputExtent.transform(decalExtent.getSRS());
            GeoExtent intersectionExtent = decalExtent.intersectionSameSRS(outputExtentInDecalSRS);
            if (intersectionExtent.isValid())
            {
                decals.push_back(decal);
                outputExtentsInDecalSRS.push_back(outputExtentInDecalSRS);
                intersections.push_back(intersectionExtent);
            }
        }
    }

    if (decals.empty())
        return GeoImage::INVALID;

    osg::ref_ptr<osg::Image> output = LandCover::createImage(getTileSize());
    
    // initialize to nodata
    ImageUtils::PixelWriter writeOutput(output.get());
    writeOutput.assign(Color(NO_DATA_VALUE));

    ImageUtils::PixelReader readOutput(output.get());
    readOutput.setBilinear(false);

    osg::Vec4 value;

    for(unsigned i=0; i<decals.size(); ++i)
    {
        const Decal& decal = decals[i];
        const GeoExtent& decalExtent = decal._extent;
        ImageUtils::PixelReader readInput(decal._image.get());
        const GeoExtent& outputExtentInDecalSRS = outputExtentsInDecalSRS[i];
        const GeoExtent& intersection = intersections[i];

        for(unsigned t=0; t<(unsigned)output->t(); ++t)
        {
            double out_v = (double)t/(double)(output->t()-1);
            double out_y = outputExtentInDecalSRS.yMin() + (double)out_v * outputExtentInDecalSRS.height();

            double in_v = (out_y-decalExtent.yMin())/decalExtent.height();

            if (in_v < 0.0 || in_v > 1.0)
                continue;

            for(unsigned s=0; s<(unsigned)output->s(); ++s)
            { 
                double out_u = (double)s/(double)(output->s()-1);
                double out_x = outputExtentInDecalSRS.xMin() + (double)out_u * outputExtentInDecalSRS.width();

                double in_u = (out_x-decalExtent.xMin())/decalExtent.width();

                if (in_u < 0.0 || in_u > 1.0)
                    continue;

                readInput(value, in_u, in_v);

                if (value.r() != NO_DATA_VALUE)
                    writeOutput(value, s, t);
            }
        }
    }

    return GeoImage(output.get(), outputExtent);
}

bool
DecalLandCoverLayer::addDecal(const std::string& id, const GeoExtent& extent, const osg::Image* image)
{
    Threading::ScopedWriteLock lock(_data_mutex);

    DecalIndex::iterator i = _decalIndex.find(id);
    if (i != _decalIndex.end())
        return false;

    _decalList.push_back(Decal());
    Decal& decal = _decalList.back();
    decal._extent = extent;
    decal._image = image;

    _decalIndex[id] = --_decalList.end();

    addDataExtent(getProfile()->clampAndTransformExtent(extent)); 

    // data changed so up the revsion.
    bumpRevision();
    return true;
}

void
DecalLandCoverLayer::removeDecal(const std::string& id)
{
    Threading::ScopedWriteLock lock(_data_mutex);

    DecalIndex::iterator i = _decalIndex.find(id);
    if (i != _decalIndex.end())
    {
        _decalList.erase(i->second);
        _decalIndex.erase(i);

        DataExtentList dataExtents;
        for (auto& decal : _decalList)
            dataExtents.push_back(getProfile()->clampAndTransformExtent(decal._extent));
        setDataExtents(dataExtents);

        // data changed so up the revsion.
        bumpRevision();
    }
}

const GeoExtent&
DecalLandCoverLayer::getDecalExtent(const std::string& id) const
{
    Threading::ScopedReadLock lock(_data_mutex);
    DecalIndex::const_iterator i = _decalIndex.find(id);
    if (i != _decalIndex.end())
    {
        return i->second->_extent;
    }
    return GeoExtent::INVALID;
}

void
DecalLandCoverLayer::clearDecals()
{
    Threading::ScopedWriteLock lock(_data_mutex);
    _decalIndex.clear();
    _decalList.clear();
    DataExtentList dataExtents;
    setDataExtents(dataExtents);
    bumpRevision();
}
