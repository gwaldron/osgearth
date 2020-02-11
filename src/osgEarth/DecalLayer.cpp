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
    setProfile(Profile::create("global-geodetic"));

    // Never cache decals
    layerHints().cachePolicy() = CachePolicy::NO_CACHE;
}

GeoImage
DecalImageLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
    std::vector<Decal> decals;
    std::vector<GeoExtent> outputExtentsInDecalSRS;
    std::vector<GeoExtent> intersections;

    const GeoExtent& outputExtent = key.getExtent();

    // thread-safe collection of intersecting decals
    {
        Threading::ScopedMutexLock lock(_mutex);

        for(std::list<Decal>::const_iterator i = _decalList.begin();
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
        return GeoImage::INVALID;

    osg::ref_ptr<osg::Image> output = new osg::Image();
    output->allocateImage(getTileSize(), getTileSize(), 1, GL_RGBA, GL_UNSIGNED_BYTE);
    output->setInternalTextureFormat(GL_RGBA8);
    ::memset(output->data(), 0, output->getTotalSizeInBytes());
    ImageUtils::PixelWriter writeOutput(output.get());
    ImageUtils::PixelReader readOutput(output.get());

    osg::Vec4 existingValue;
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

                readOutput(existingValue, s, t);
                readInput(value, in_u, in_v);

                value.r() = value.r()*value.a() + (existingValue.r()*(1.0-value.a()));
                value.g() = value.g()*value.a() + (existingValue.g()*(1.0-value.a()));
                value.b() = value.b()*value.a() + (existingValue.b()*(1.0-value.a()));
                value.a() = osg::maximum(value.a(), existingValue.a());

                writeOutput(value, s, t);
            }
        }
    }

    return GeoImage(output.get(), outputExtent);
}

void
DecalImageLayer::addDecal(const std::string& id, const GeoExtent& extent, const osg::Image* image)
{
    // make sure there are no dupes
    removeDecal(id);

    // safe lock
    {
        Threading::ScopedMutexLock lock(_mutex);

        _decalList.push_back(Decal());
        Decal& decal = _decalList.back();
        decal._extent = extent;
        decal._image = image;

        std::list<Decal>::iterator i = _decalList.end();
        _decalIndex[id] = --i;

        // data changed so up the revsion.
        bumpRevision();
    }
}

void
DecalImageLayer::removeDecal(const std::string& id)
{
    Threading::ScopedMutexLock lock(_mutex);

    DecalIndex::iterator i = _decalIndex.find(id);
    if (i != _decalIndex.end())
    {
        _decalList.erase(i->second);
        _decalIndex.erase(i);

        // data changed so up the revsion.
        bumpRevision();
    }
}

const GeoExtent&
DecalImageLayer::getDecalExtent(const std::string& id) const
{
    Threading::ScopedMutexLock lock(_mutex);
    DecalIndex::const_iterator i = _decalIndex.find(id);
    if (i != _decalIndex.end())
    {
        return i->second->_extent;
    }
    return GeoExtent::INVALID;
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
    setProfile(Profile::create("global-geodetic"));

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
        Threading::ScopedMutexLock lock(_mutex);

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
                    double uu = (y-decalExtent.yMin())/decalExtent.width();
                    double vv = (x-decalExtent.xMin())/decalExtent.height();

                    float h_prev = HeightFieldUtils::getHeightAtNormalizedLocation(output.get(), u, v);

                    float h = HeightFieldUtils::getHeightAtNormalizedLocation(decal_hf, uu, vv);

                    // "blend" heights together by adding them.
                    if (h != NO_DATA_VALUE)
                    {
                        float final_h = h_prev != NO_DATA_VALUE ? h+h_prev : h;
                        output->setHeight(col, row, final_h);
                    }
                }
            }
        }
    }

    return GeoHeightField(output.get(), outputExtent);
}

void
DecalElevationLayer::addDecal(const std::string& id, const GeoExtent& extent, const osg::Image* image, float scale)
{
    if (!extent.isValid() || !image)
        return;

    osg::HeightField* hf = new osg::HeightField();
    hf->allocate(image->s(), image->t());

    ImageUtils::PixelReader read(image);

    osg::Vec4 value;
    for(unsigned row=0; row<image->t(); ++row)
    {
        for(unsigned col=0; col<image->s(); ++col)
        {
            read(value, col, row);
            float h = value.a() * scale;
            hf->setHeight(col, row, h);
        }
    }

    // no dupes
    removeDecal(id);

    // safe lock
    {
        Threading::ScopedMutexLock lock(_mutex);

        _decalList.push_back(Decal());
        Decal& decal = _decalList.back();
        decal._heightfield = GeoHeightField(hf, extent);

        std::list<Decal>::iterator i = _decalList.end();
        _decalIndex[id] = --i;

        // data changed so up the revsion.
        bumpRevision();
    }
}

void
DecalElevationLayer::removeDecal(const std::string& id)
{
    Threading::ScopedMutexLock lock(_mutex);

    DecalIndex::iterator i = _decalIndex.find(id);
    if (i != _decalIndex.end())
    {
        _decalList.erase(i->second);
        _decalIndex.erase(i);

        // data changed so up the revsion.
        bumpRevision();
    }
}

const GeoExtent&
DecalElevationLayer::getDecalExtent(const std::string& id) const
{
    Threading::ScopedMutexLock lock(_mutex);
    DecalIndex::const_iterator i = _decalIndex.find(id);
    if (i != _decalIndex.end())
    {
        return i->second->_heightfield.getExtent();
    }
    return GeoExtent::INVALID;
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
    setProfile(Profile::create("global-geodetic"));

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
        profile = Profile::create("global-geodetic");
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
        Threading::ScopedMutexLock lock(_mutex);

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

    osg::ref_ptr<osg::Image> output = new osg::Image();
    
    output->allocateImage(getTileSize(), getTileSize(), 1, GL_RED, GL_FLOAT);
    output->setInternalTextureFormat(GL_R16F);
    
    // initialize to nodata
    float* ptr = (float*)output->data();
    for(unsigned i=0; i<getTileSize()*getTileSize(); ++i)
        *ptr++ = NO_DATA_VALUE;

    ImageUtils::PixelWriter writeOutput(output.get());

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

void
DecalLandCoverLayer::addDecal(const std::string& id, const GeoExtent& extent, const osg::Image* image)
{
    removeDecal(id);
    {
        Threading::ScopedMutexLock lock(_mutex);

        _decalList.push_back(Decal());
        Decal& decal = _decalList.back();
        decal._extent = extent;
        decal._image = image;

        std::list<Decal>::iterator i = _decalList.end();
        _decalIndex[id] = --i;

        // data changed so up the revsion.
        bumpRevision();
    }
}

void
DecalLandCoverLayer::removeDecal(const std::string& id)
{
    Threading::ScopedMutexLock lock(_mutex);

    DecalIndex::iterator i = _decalIndex.find(id);
    if (i != _decalIndex.end())
    {
        _decalList.erase(i->second);
        _decalIndex.erase(i);

        // data changed so up the revsion.
        bumpRevision();
    }
}

const GeoExtent&
DecalLandCoverLayer::getDecalExtent(const std::string& id) const
{
    Threading::ScopedMutexLock lock(_mutex);
    DecalIndex::const_iterator i = _decalIndex.find(id);
    if (i != _decalIndex.end())
    {
        return i->second->_extent;
    }
    return GeoExtent::INVALID;
}