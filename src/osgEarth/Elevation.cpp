
/* osgEarth
* Copyright 2008-2016 Pelican Mapping
* MIT License
*/
#include <osgEarth/Elevation>
#include <osgEarth/Registry>
#include <osgEarth/Map>
#include <osgEarth/Progress>
#include <osgEarth/Metrics>

// for OSGEARTH_USE_16BIT_ELEVATION_TEXTURES
#include <osgEarth/BuildConfig>

using namespace osgEarth;

// if defined, elevation textures are encoded as 2 8-bit channels
// with the high order byte in RED and the low order byte in GREEN.
// the encoded value maps to [0..1] where 0 corresponds to the minimum height
// and 1 corresponds to the maximum height as encoded in the tile.

#ifdef OSGEARTH_USE_16BIT_ELEVATION_TEXTURES
    #define ELEV_PIXEL_FORMAT GL_RG
    #define ELEV_INTERNAL_FORMAT GL_RG8
    #define ELEV_DATA_TYPE GL_UNSIGNED_BYTE
#else
    #define ELEV_PIXEL_FORMAT GL_RED
    #define ELEV_INTERNAL_FORMAT GL_R32F
    #define ELEV_DATA_TYPE GL_FLOAT
#endif

osg::Texture*
osgEarth::createEmptyElevationTexture()
{
    osg::Image* image = new osg::Image();
    image->allocateImage(1, 1, 1, ELEV_PIXEL_FORMAT, ELEV_DATA_TYPE);
    image->setInternalTextureFormat(ELEV_INTERNAL_FORMAT);
    ImageUtils::PixelWriter write(image);
    write(osg::Vec4f(0.0f, 0.0f, 0.0f, 0.0f), 0, 0); // zero out
    osg::Texture2D* tex = new osg::Texture2D(image);
    tex->setInternalFormat(ELEV_INTERNAL_FORMAT);
    tex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
    tex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
    tex->setUnRefImageDataAfterApply(Registry::instance()->unRefImageDataAfterApply().get());
    tex->setName("empty:elevation");
    return tex;
}

osg::Texture*
osgEarth::createEmptyNormalMapTexture()
{
    osg::Image* image = new osg::Image();
    image->allocateImage(1, 1, 1, GL_RG, GL_UNSIGNED_BYTE);
    image->setInternalTextureFormat(GL_RG8);
    ImageUtils::PixelWriter write(image);
    write(osg::Vec4(0.5f, 0.5f, 0.0f, 0.0f), 0, 0); // BC5 octahedral encoded Z-up
    osg::Texture2D* tex = new osg::Texture2D(image);
    tex->setInternalFormat(image->getInternalTextureFormat());
    tex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
    tex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
    tex->setUnRefImageDataAfterApply(Registry::instance()->unRefImageDataAfterApply().get());
    tex->setName("empty:normalmap");
    return tex;
}

ElevationTile::ElevationTile(const TileKey& key, const GeoHeightField& in_hf, std::vector<float>&& resolutions) :
    _tilekey(key),
    _extent(in_hf.getExtent()),
    _resolutions(std::move(resolutions))
{
    if (in_hf.valid())
    {
        _heightField = in_hf.getHeightField();

        _maxima = { in_hf.getMinHeight(), in_hf.getMaxHeight() };

        osg::Vec4 value;

        // Encode elevation as a 2-channel texture, RG, where R = high byte and G = low byte of a 16-bit integer.
        // The 16-bit integer is a normalized value between 0 and 65535, where 0 corresponds to the minimum height
        // and 65535 corresponds to the maximum height as encoded in the tile metadata.
        osg::Image* heights = new osg::Image();
        heights->allocateImage(_heightField->getNumColumns(), _heightField->getNumRows(), 1, ELEV_PIXEL_FORMAT, ELEV_DATA_TYPE);
        heights->setInternalTextureFormat(ELEV_INTERNAL_FORMAT);

        // GL_RG = 16-bit encoded relative height values (0..1 from min to max)
        if (heights->getPixelFormat() == GL_RG)
        {
            auto* data = heights->data();
            for (auto h : _heightField->getHeightList())
            {
                float t = (h - _maxima.first) / (_maxima.second - _maxima.first);
                int ti = (int)(65535.0f * t);
                *data++ = (std::int8_t)((ti >> 8) & 0xFF); // high byte
                *data++ = (std::int8_t)(ti & 0xFF); // low byte
            }

            _is16bitEncoded = true;
        }
        else // GL_R32F
        {
            memcpy(heights->data(), _heightField->getHeightList().data(), sizeof(float) * _heightField->getNumRows() * _heightField->getNumColumns());
        }

        // Can't compress the elevation because it will no longer match up 
        // at the tile seams due to the lossy compression of BC5 or BC7.
        //ImageUtils::compressImageInPlace(heights);

        _elevationTex = new osg::Texture2D(heights);
        _elevationTex->setName(key.str() + ":elevation");

        _elevationTex->setDataVariance(osg::Object::STATIC);
        _elevationTex->setInternalFormat(heights->getInternalTextureFormat());
        _elevationTex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
        _elevationTex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
        _elevationTex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
        _elevationTex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
        _elevationTex->setResizeNonPowerOfTwoHint(false);
        _elevationTex->setMaxAnisotropy(1.0f);

        // Pooled, so never expire them.
        _elevationTex->setUnRefImageDataAfterApply(false);

        _read.setTexture(_elevationTex.get());
        _read.setSampleAsTexture(false);

        _resolution = Distance(
            getExtent().height() / ((double)(heights->t() - 1)),
            getExtent().getSRS()->getUnits());
    }
}

ElevationSample
ElevationTile::getElevation(double x, double y) const
{
    double u = (x - getExtent().xMin()) / getExtent().width();
    double v = (y - getExtent().yMin()) / getExtent().height();

    return getElevationUV(u, v);
}

ElevationSample
ElevationTile::getElevationUV(double u, double v) const
{
    osg::Vec4f sample;
    u = clamp(u, 0.0, 1.0), v = clamp(v, 0.0, 1.0);
    _read(sample, u, v);

#ifdef OSGEARTH_USE_16BIT_ELEVATION_TEXTURES
    float t = (sample.r() * 65280.0f + sample.g() * 255.0) / 65535.0f; // [0..1]
    sample.r() = _maxima.first + t * (_maxima.second - _maxima.first); // scale to min/max
#endif

    return ElevationSample(Distance(sample.r(),Units::METERS), _resolution);
}

osg::Vec3
ElevationTile::getNormal(double x, double y) const
{
    osg::Vec3 normal(0,0,1);

    if (_readNormal.valid())
    {
        double u = (x - getExtent().xMin()) / getExtent().width();
        double v = (y - getExtent().yMin()) / getExtent().height();
        osg::Vec4 value;
        _readNormal(value, u, v);
        normal.set(value.r(), value.g(), value.b());
    }
    return normal;
}

void
ElevationTile::generateNormalMap(const Map* map, void* workingSet, ProgressCallback* progress)
{
    std::lock_guard<std::mutex> lock(_mutex);

    if (!_normalTex.valid())
    {
        NormalMapGenerator gen;

        _normalTex = gen.createNormalMap(getTileKey(), map, workingSet, progress);

        if (_normalTex.valid())
        {
            // these are pooled, so do not expire them.
            _normalTex->setUnRefImageDataAfterApply(false);

            if (_normalTex->getImage())
            {
                _readNormal.setImage(_normalTex->getImage());
                _readNormal.setBilinear(true);
            }
        }
    }
}


#undef LC
#define LC "[NormalMapGenerator] "

osg::Texture2D*
NormalMapGenerator::createNormalMap(const TileKey& key, const Map* map, void* ws, ProgressCallback* progress)
{
    if (!map)
        return NULL;

    OE_PROFILING_ZONE;

    ElevationPool::WorkingSet* workingSet = static_cast<ElevationPool::WorkingSet*>(ws);

    osg::ref_ptr<osg::Image> image = new osg::Image();
    image->allocateImage(ELEVATION_TILE_SIZE, ELEVATION_TILE_SIZE, 1, GL_RG, GL_UNSIGNED_BYTE);
    image->setInternalTextureFormat(GL_RG8);

    ElevationPool* pool = map->getElevationPool();

    ImageUtils::PixelWriter write(image.get());

    osg::Vec3 normal;
    osg::Vec2 packedNormal;
    osg::Vec4 pixel;

    GeoPoint
        north(key.getProfile()->getSRS()),
        south(key.getProfile()->getSRS()),
        east(key.getProfile()->getSRS()),
        west(key.getProfile()->getSRS());

    osg::Vec3 a[4];

    const GeoExtent& ex = key.getExtent();

    // fetch the base tile in order to get resolutions data.
    osg::ref_ptr<ElevationTile> heights;
    pool->getTile(key, true, heights, workingSet, progress);

    if (!heights.valid())
        return NULL;

    // build the sample set.
    struct Workspace {
        std::map<osg::Vec4d, int> uniquePoints; // map a point to its index in vectorToSample
        std::vector<osg::Vec4d> vectorToSample; // actula points we'll send to the elevation pool
        std::vector<int> rasterIndex; // maps the raster offset to an entry in vectorToSample.
        std::vector<osg::Vec4d> points; // final sampled points
    };
    static thread_local Workspace w;


    w.uniquePoints.clear();

    w.vectorToSample.clear();
    w.vectorToSample.reserve(write.s() * write.t() * 4);

    w.rasterIndex.clear();
    w.rasterIndex.reserve(write.s() * write.t() * 4);

    int p = 0;
    for (int t = 0; t < write.t(); ++t)
    {
        double v = (double)t / (double)(write.t() - 1);
        double y = ex.yMin() + v * ex.height();
        east.y() = y;
        west.y() = y;

        for (int s = 0; s < write.s(); ++s)
        {
            int i = 4 * write.s() * t + 4 * s;

            double u = (double)s / (double)(write.s() - 1);
            double x = ex.xMin() + u * ex.width();
            north.x() = x;
            south.x() = x;

            double r = heights->getResolution(s, t);

            east.x() = x + r;
            west.x() = x - r;
            north.y() = y + r;
            south.y() = y - r;

            {
                auto& [iter, isNew] = w.uniquePoints.emplace(osg::Vec4d(west.x(), west.y(), 0.0, r), (int)w.vectorToSample.size());
                if (isNew) w.vectorToSample.emplace_back(iter->first);
                w.rasterIndex.emplace_back(iter->second);
            }
            {
                auto& [iter, isNew] = w.uniquePoints.emplace(osg::Vec4d(east.x(), east.y(), 0.0, r), (int)w.vectorToSample.size());
                if (isNew) w.vectorToSample.emplace_back(iter->first);
                w.rasterIndex.emplace_back(iter->second);
            }
            {
                auto& [iter, isNew] = w.uniquePoints.emplace(osg::Vec4d(south.x(), south.y(), 0.0, r), (int)w.vectorToSample.size());
                if (isNew) w.vectorToSample.emplace_back(iter->first);
                w.rasterIndex.emplace_back(iter->second);
            }
            {
                auto& [iter, isNew] = w.uniquePoints.emplace(osg::Vec4d(north.x(), north.y(), 0.0, r), (int)w.vectorToSample.size());
                if (isNew) w.vectorToSample.emplace_back(iter->first);
                w.rasterIndex.emplace_back(iter->second);
            }
        }
    }

    int sampleOK = map->getElevationPool()->sampleMapCoords(
        w.vectorToSample.begin(), w.vectorToSample.end(),
        workingSet,
        progress);

    if (progress && progress->isCanceled())
    {
        // canceled. Bail.
        return NULL;
    }

    if (sampleOK < 0)
    {
        OE_WARN << LC << "Internal error - contact support" << std::endl;
        return NULL;
    }

    // copy the results back into the points array.
    w.points.clear();
    w.points.reserve(write.s() * write.t() * 4);
    for (int p = 0; p < (int)w.rasterIndex.size(); ++p)
    {
        w.points.emplace_back(w.vectorToSample[w.rasterIndex[p]]);
    }

    auto* srs = key.getProfile()->getSRS();
    Distance res(0.0, srs->getUnits());
    double dx, dy;
    osg::Vec4 riPixel;

    for (int t = 0; t < write.t(); ++t)
    {
        double v = (double)t / (double)(write.t() - 1);
        double y_or_lat = ex.yMin() + v * ex.height();

        for (int s = 0; s < write.s(); ++s)
        {
            int p = (4 * write.s() * t + 4 * s);

            res.set(w.points[p].w(), res.getUnits());
            dx = srs->transformDistance(res, Units::METERS, y_or_lat);
            dy = srs->transformDistance(res, Units::METERS, 0.0);

            riPixel.r() = 0.0f;

            // only attempt to create a normal vector if all the data is valid:
            // a valid resolution value and four valid corner points.
            if (res.getValue() != FLT_MAX &&
                w.points[p + 0].z() != NO_DATA_VALUE &&
                w.points[p + 1].z() != NO_DATA_VALUE &&
                w.points[p + 2].z() != NO_DATA_VALUE &&
                w.points[p + 3].z() != NO_DATA_VALUE)
            {
                a[0].set(-dx, 0, w.points[p + 0].z());
                a[1].set(dx, 0, w.points[p + 1].z());
                a[2].set(0, -dy, w.points[p + 2].z());
                a[3].set(0, dy, w.points[p + 3].z());

                normal = (a[1] - a[0]) ^ (a[3] - a[2]);
                normal.normalize();
            }
            else
            {
                normal.set(0, 0, 1);
            }

            NormalMapGenerator::pack(normal, pixel);

            write(pixel, s, t);
        }
    }

    //ImageUtils::mipmapImageInPlace(image.get());
    ImageUtils::compressImageInPlace(image.get());

    osg::Texture2D* normalTex = new osg::Texture2D(image.get());

    normalTex->setDataVariance(osg::Object::STATIC);
    normalTex->setInternalFormat(image->getInternalTextureFormat());
    normalTex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    normalTex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
    normalTex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
    normalTex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
    normalTex->setResizeNonPowerOfTwoHint(false);
    normalTex->setMaxAnisotropy(1.0f);
    normalTex->setUnRefImageDataAfterApply(Registry::instance()->unRefImageDataAfterApply().get());

    return normalTex;
}

