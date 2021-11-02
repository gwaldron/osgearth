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
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include "DecalLayer"
#include <osgEarth/Map>
#include <osgEarth/Profile>
#include <osgEarth/VirtualProgram>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/ImageToHeightFieldConverter>
#include <osgEarth/Color>
#include <osg/MatrixTransform>
#include <osg/BlendFunc>
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
        Threading::ScopedMutexLock lock(layerMutex());

        for (std::list<Decal>::const_iterator i = _decalList.begin();
            i != _decalList.end();
            ++i)
        {
            const Decal& decal = *i;
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

    osg::ref_ptr<osg::Image> output;
    
    if (canvas.valid())
    {
        // clones and converts to RGBA8 if necessary
        output = ImageUtils::convertToRGBA8(canvas.getImage());
    }
    else
    {
        output = new osg::Image();
        output->allocateImage(getTileSize(), getTileSize(), 1, GL_RGBA, GL_UNSIGNED_BYTE);
        output->setInternalTextureFormat(GL_RGBA8);
        ::memset(output->data(), 0, output->getTotalSizeInBytes());
    }

    ImageUtils::PixelWriter writeOutput(output.get());
    ImageUtils::PixelReader readOutput(output.get());

    osg::Vec4 existingValue;
    osg::Vec4 value;

    for (unsigned d = 0; d < decals.size(); ++d)
    {
        const Decal& decal = decals[d];
        const GeoExtent& decalExtent = decal._extent;
        ImageUtils::PixelReader readInput(decal._image.get());
        const GeoExtent& outputExtentInDecalSRS = outputExtentsInDecalSRS[d];
        const GeoExtent& intersection = intersections[d];
        bool normalizeX = decalExtent.crossesAntimeridian();

#if 0
        GeoImageIterator i(writeOutput, outputExtentInDecalSRS);
        i.forEachPixelOnCenter(
            [&]()
            {
                double in_v = (i.y() - decalExtent.yMin()) / decalExtent.height();
                if (in_v < 0.0 || in_v > 1.0)
                    return;

                double out_x = i.x();
                if (normalizeX)
                {
                    while (out_x < decalExtent.xMin())
                        out_x += 360.0;
                    while (out_x > decalExtent.xMax())
                        out_x -= 360.0;
                }

                double in_u = (out_x - decalExtent.xMin()) / decalExtent.width();

                if (in_u < 0.0 || in_u > 1.0)
                    return;

                readOutput(existingValue, i.s(), i.t());
                readInput(value, in_u, in_v);

                value.r() = value.r()*value.a() + (existingValue.r()*(1.0 - value.a()));
                value.g() = value.g()*value.a() + (existingValue.g()*(1.0 - value.a()));
                value.b() = value.b()*value.a() + (existingValue.b()*(1.0 - value.a()));
                value.a() = osg::maximum(value.a(), existingValue.a());

                writeOutput(value, i.s(), i.t());
            }
        );

#else
        for (unsigned t = 0; t < (unsigned)output->t(); ++t)
        {
            double out_v = (double)t / (double)(output->t() - 1);
            double out_y = outputExtentInDecalSRS.yMin() + (double)out_v * outputExtentInDecalSRS.height();

            double in_v = (out_y - decalExtent.yMin()) / decalExtent.height();

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

                if (in_u < 0.0 || in_u > 1.0)
                    continue;

                readOutput(existingValue, s, t);
                readInput(value, in_u, in_v);

                value.r() = value.r()*value.a() + (existingValue.r()*(1.0 - value.a()));
                value.g() = value.g()*value.a() + (existingValue.g()*(1.0 - value.a()));
                value.b() = value.b()*value.a() + (existingValue.b()*(1.0 - value.a()));
                value.a() = osg::maximum(value.a(), existingValue.a());

                writeOutput(value, s, t);
            }
        }
#endif
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
    Threading::ScopedMutexLock lock(layerMutex());

    DecalIndex::iterator i = _decalIndex.find(id);
    if (i != _decalIndex.end())
        return false;

    _decalList.push_back(Decal());
    Decal& decal = _decalList.back();
    decal._extent = extent;
    decal._image = image;

    _decalIndex[id] = --_decalList.end();

    _extent.expandToInclude(extent);

    // data changed so up the revsion.
    bumpRevision();
    return true;
}

void
DecalImageLayer::removeDecal(const std::string& id)
{
    Threading::ScopedMutexLock lock(layerMutex());

    DecalIndex::iterator i = _decalIndex.find(id);
    if (i != _decalIndex.end())
    {
        _decalList.erase(i->second);
        _decalIndex.erase(i);

        _extent = GeoExtent();
        for (auto& decal : _decalList)
            _extent.expandToInclude(decal._extent);

        // data changed so up the revsion.
        bumpRevision();
    }
}

const GeoExtent&
DecalImageLayer::getDecalExtent(const std::string& id) const
{
    Threading::ScopedMutexLock lock(layerMutex());
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
    Threading::ScopedMutexLock lock(layerMutex());
    _decalIndex.clear();
    _decalList.clear();
    _extent = GeoExtent();
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
    setOffset(true);

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
        Threading::ScopedMutexLock lock(layerMutex());

        for(std::list<Decal>::const_iterator i = _decalList.begin();
            i != _decalList.end();
            ++i)
        {
            const Decal& decal = *i;
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

    Threading::ScopedMutexLock lock(layerMutex());

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
    c = osg::minimum(c, osg::Image::computeNumComponents(image->getPixelFormat())-1u);

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

    _extent.expandToInclude(extent);

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

    Threading::ScopedMutexLock lock(layerMutex());

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
    c = osg::maximum(c, osg::Image::computeNumComponents(image->getPixelFormat())-1u);

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

    _extent.expandToInclude(extent);

    // data changed so up the revsion.
    bumpRevision();
    return true;
}

void
DecalElevationLayer::removeDecal(const std::string& id)
{
    Threading::ScopedMutexLock lock(layerMutex());

    DecalIndex::iterator i = _decalIndex.find(id);
    if (i != _decalIndex.end())
    {
        _decalList.erase(i->second);
        _decalIndex.erase(i);

        for (auto& decal : _decalList)
            _extent.expandToInclude(decal._heightfield.getExtent());

        // data changed so up the revsion.
        bumpRevision();
    }
}

const GeoExtent&
DecalElevationLayer::getDecalExtent(const std::string& id) const
{
    Threading::ScopedMutexLock lock(layerMutex());
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
    Threading::ScopedMutexLock lock(layerMutex());
    _decalIndex.clear();
    _decalList.clear();
    _extent = GeoExtent();
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
        Threading::ScopedMutexLock lock(layerMutex());

        for(std::list<Decal>::const_iterator i = _decalList.begin();
            i != _decalList.end();
            ++i)
        {
            const Decal& decal = *i;
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
    Threading::ScopedMutexLock lock(layerMutex());

    DecalIndex::iterator i = _decalIndex.find(id);
    if (i != _decalIndex.end())
        return false;

    _decalList.push_back(Decal());
    Decal& decal = _decalList.back();
    decal._extent = extent;
    decal._image = image;

    _decalIndex[id] = --_decalList.end();

    _extent.expandToInclude(extent);

    // data changed so up the revsion.
    bumpRevision();
    return true;
}

void
DecalLandCoverLayer::removeDecal(const std::string& id)
{
    Threading::ScopedMutexLock lock(layerMutex());

    DecalIndex::iterator i = _decalIndex.find(id);
    if (i != _decalIndex.end())
    {
        _decalList.erase(i->second);
        _decalIndex.erase(i);

        _extent = GeoExtent();
        for (auto& decal : _decalList)
            _extent.expandToInclude(decal._extent);

        // data changed so up the revsion.
        bumpRevision();
    }
}

const GeoExtent&
DecalLandCoverLayer::getDecalExtent(const std::string& id) const
{
    Threading::ScopedMutexLock lock(layerMutex());
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
    Threading::ScopedMutexLock lock(layerMutex());
    _decalIndex.clear();
    _decalList.clear();
    _extent = GeoExtent();
    bumpRevision();
}
