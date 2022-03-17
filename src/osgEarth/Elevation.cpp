
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2016 Pelican Mapping
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
#include <osgEarth/Elevation>
#include <osgEarth/Registry>
#include <osgEarth/Map>
#include <osgEarth/Progress>
#include <osgEarth/Metrics>

using namespace osgEarth;

//#define USE_RUGGEDNESS

osg::Texture*
osgEarth::createEmptyElevationTexture()
{
    osg::Image* image = new osg::Image();
    image->allocateImage(1, 1, 1, GL_RED, GL_FLOAT);
    *((GLfloat*)image->data()) = 0.0f;
    osg::Texture2D* tex = new osg::Texture2D(image);
    tex->setInternalFormat(GL_R32F);
    tex->setUnRefImageDataAfterApply(Registry::instance()->unRefImageDataAfterApply().get());
    tex->setName("empty:elevation");
    return tex;
}

osg::Texture*
osgEarth::createEmptyNormalMapTexture()
{
    osg::Image* image = new osg::Image();
    image->allocateImage(1, 1, 1, GL_RG, GL_UNSIGNED_BYTE);
    ImageUtils::PixelWriter write(image);
    osg::Vec4 packed;
    NormalMapGenerator::pack(osg::Vec3(0,0,1), packed);
    write(packed, 0, 0);
    osg::Texture2D* tex = new osg::Texture2D(image);
    tex->setInternalFormat(GL_RG8);
    tex->setUnRefImageDataAfterApply(Registry::instance()->unRefImageDataAfterApply().get());
    return tex;
}

ElevationTexture::ElevationTexture(
    const TileKey& key,
    const GeoHeightField& in_hf,
    const std::vector<float>& resolutions) :

    _tilekey(key),
    _extent(in_hf.getExtent()),
    _resolutions(std::move(resolutions))
{
    setName(key.str() + ":elevation");

    if (in_hf.valid())
    {
        _heightField = in_hf.getHeightField();

        osg::Vec4 value;

        osg::Image* heights = new osg::Image();
        heights->allocateImage(_heightField->getNumColumns(), _heightField->getNumRows(), 1, GL_RED, GL_FLOAT);
        heights->setInternalTextureFormat(GL_R32F);

        ImageUtils::PixelWriter write(heights);
        // TODO: speed this up since we know the format
        for(unsigned row=0; row<_heightField->getNumRows(); ++row)
        {
            for(unsigned col=0; col<_heightField->getNumColumns(); ++col)
            {
                value.r() = _heightField->getHeight(col, row);
                write(value, col, row);
            }
        }
        setImage(heights);

        setDataVariance(osg::Object::STATIC);
        setInternalFormat(GL_R32F);
        setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
        setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
        setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
        setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
        setResizeNonPowerOfTwoHint(false);
        setMaxAnisotropy(1.0f);

        // Pooled, so never expire them.
        setUnRefImageDataAfterApply(false);

        _read.setTexture(this);
        _read.setSampleAsTexture(false);

        _resolution = Distance(
            getExtent().height() / ((double)(getImage(0)->s()-1)),
            getExtent().getSRS()->getUnits());
    }
}

ElevationTexture::~ElevationTexture()
{
    //nop
}

ElevationSample
ElevationTexture::getElevation(double x, double y) const
{
    double u = (x - getExtent().xMin()) / getExtent().width();
    double v = (y - getExtent().yMin()) / getExtent().height();

    return getElevationUV(u, v);
}

ElevationSample
ElevationTexture::getElevationUV(double u, double v) const
{
    osg::Vec4 value;
    u = osg::clampBetween(u, 0.0, 1.0), v = osg::clampBetween(v, 0.0, 1.0);
    _read(value, u, v);
    return ElevationSample(Distance(value.r(),Units::METERS), _resolution);
}

osg::Vec3
ElevationTexture::getNormal(double x, double y) const
{
    osg::Vec3 normal(0,0,1);
    if (_normalTex.valid())
    {
        double u = (x - getExtent().xMin()) / getExtent().width();
        double v = (y - getExtent().yMin()) / getExtent().height();
        osg::Vec4 value;
        _readNormal(value, u, v);
        NormalMapGenerator::unpack(value, normal);
    }
    return normal;
}

float
ElevationTexture::getRuggedness(double x, double y) const
{
    float result = 0.0f;
    if (_ruggedness.valid())
    {
        double u = (x - getExtent().xMin()) / getExtent().width();
        double v = (y - getExtent().yMin()) / getExtent().height();
        osg::Vec4 value;
        _readRuggedness(value, u, v);
        result = value.r();
    }
    return result;
}

osg::Texture2D*
ElevationTexture::getNormalMapTexture() const
{
    return _normalTex.get();
}

void
ElevationTexture::generateNormalMap(
    const Map* map,
    void* workingSet,
    ProgressCallback* progress)
{
    OE_PROFILING_ZONE;
    if (!_normalTex.valid())
    {
        // one thread allowed to generate the normal map
        static Gate<void*> s_thisGate("OE.ElevTexNormalMap");
        ScopedGate<void*> lockThis(s_thisGate, this);

        if (!_normalTex.valid())
        {
#ifdef USE_RUGGEDNESS
            if (!_ruggedness.valid())
            {
                _ruggedness = new osg::Image();
                _ruggedness->allocateImage(_read.s(), _read.t(), 1, GL_RED, GL_UNSIGNED_BYTE);
                _readRuggedness.setImage(_ruggedness.get());
                _readRuggedness.setBilinear(true);
            }
#endif

            NormalMapGenerator gen;

            _normalTex = gen.createNormalMap(
                getTileKey(),
                map,
                workingSet,
                _ruggedness.get(),
                progress);

            if (_normalTex.valid())
            {
                // these are pooled, so do not expire them.
                _normalTex->setUnRefImageDataAfterApply(false);

                _readNormal.setImage(_normalTex->getImage());
                _readNormal.setBilinear(true);
            }
        }
    }
}


#undef LC
#define LC "[NormalMapGenerator] "

osg::Texture2D*
NormalMapGenerator::createNormalMap(
    const TileKey& key,
    const Map* map,
    void* ws,
    osg::Image* ruggedness,
    ProgressCallback* progress)
{    
    if (!map)
        return NULL;

    OE_PROFILING_ZONE;

    ElevationPool::WorkingSet* workingSet = static_cast<ElevationPool::WorkingSet*>(ws);

    osg::ref_ptr<osg::Image> image = new osg::Image();
    image->allocateImage(
        ELEVATION_TILE_SIZE, ELEVATION_TILE_SIZE, 1,
        GL_RG, GL_UNSIGNED_BYTE);

    ElevationPool* pool = map->getElevationPool();

    ImageUtils::PixelWriter write(image.get());

    ImageUtils::PixelWriter writeRuggedness(ruggedness);

    osg::Vec3 normal;
    osg::Vec2 packedNormal;
    osg::Vec4 pixel;

    osg::Vec3 a[4];

    const GeoExtent& ex = key.getExtent();

    // fetch the base tile in order to get resolutions data.
    osg::ref_ptr<ElevationTexture> heights;
    pool->getTile(key, true, heights, workingSet, progress);

    if (!heights.valid())
        return NULL;

    // build the sample set.
    unsigned int sampleWidth = write.s() + 2;
    unsigned int sampleHeight = write.t() + 2;
    std::vector<osg::Vec4d> points(sampleWidth * sampleHeight);
   
    int p = 0;
    // Add the south row
    for (int s = 0; s < write.s(); ++s)
    {
        double u = (double)s / (double)(write.s() - 1);
        double x = ex.xMin() + u * ex.width();
        double r = heights->getResolution(s, 0);
        double y = ex.yMin();
        if (s == 0)
        {
            points[p++].set(x-r, y-r, 0.0, r);
        }
        points[p++].set(x, y-r, 0.0, r);
        if (s == write.s() - 1)
        {
            points[p++].set(x + r, y-r, 0.0, r);
        }
    }

    for (int t = 0; t < write.t(); ++t)
    {
        double v = (double)t / (double)(write.t() - 1);
        double y = ex.yMin() + v * ex.height();

        for (int s = 0; s < write.s(); ++s)
        {
            double u = (double)s / (double)(write.s() - 1);
            double x = ex.xMin() + u * ex.width();

            double r = heights->getResolution(s, t);

            if (s == 0)
            {
                points[p++].set(x - r, y, 0.0, r);
            }
            points[p++].set(x, y, 0.0, r);
            if (s == write.s() - 1)
            {
                points[p++].set(x + r, y, 0.0, r);
            }            
        }
    }

    // Add the north row
    for (int s = 0; s < write.s(); ++s)
    {
        double u = (double)s / (double)(write.s() - 1);
        double x = ex.xMin() + u * ex.width();
        double r = heights->getResolution(s, 0);
        double y = ex.yMax();
        if (s == 0)
        {
            points[p++].set(x - r, y + r, 0.0, r);
        }
        points[p++].set(x, y + r, 0.0, r);
        if (s == write.s() - 1)
        {
            points[p++].set(x + r, y + r, 0.0, r);
        }
    }

    int sampleOK = map->getElevationPool()->sampleMapCoords(
        points,
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

    Distance res(0.0, key.getProfile()->getSRS()->getUnits());
    double dx, dy;
    osg::Vec4 riPixel;

    for (int t = 0; t < write.t(); ++t)
    {
        double v = (double)t / (double)(write.t() - 1);
        double y_or_lat = ex.yMin() + v * ex.height();
        
        unsigned int sampleT = t + 1;

        for (int s = 0; s < write.s(); ++s)
        {
            unsigned int sampleS = s + 1;

            // Get the index of this point in the sample point array
            int p = sampleT * sampleWidth + sampleS;
            res.set(points[p].w(), res.getUnits());

            dx = res.asDistance(Units::METERS, y_or_lat);
            dy = res.asDistance(Units::METERS, 0.0);

            riPixel.r() = 0.0f;

            osg::Vec4d& west = points[p - 1];
            osg::Vec4d& east = points[p + 1];
            osg::Vec4d& north = points[p + sampleWidth];
            osg::Vec4d& south = points[p - sampleWidth];

            // only attempt to create a normal vector if all the data is valid:
            // a valid resolution value and four valid corner points.
            if (res.getValue() != FLT_MAX &&
                west.z() != NO_DATA_VALUE &&
                east.z() != NO_DATA_VALUE &&
                south.z() != NO_DATA_VALUE &&
                north.z() != NO_DATA_VALUE)
            {
                a[0].set(-dx, 0, west.z());
                a[1].set(dx, 0, east.z());
                a[2].set(0, -dy, south.z());
                a[3].set(0, dy, north.z());

                normal = (a[1] - a[0]) ^ (a[3] - a[2]);
                normal.normalize();

                if (ruggedness)
                {
                    // rudimentary normalized ruggedness index
                    riPixel.r() = 0.25 * (
                        fabs(west.z() - north.z()) +
                        fabs(east.z() - west.z()) +
                        fabs(south.z() - east.z()) +
                        fabs(north.z() - north.z()));
                    riPixel.r() = clamp(riPixel.r() / (float)dy, 0.0f, 1.0f);
                    riPixel.r() = harden(harden(riPixel.r()));
                }
            }
            else
            {
                normal.set(0, 0, 1);
            }

            NormalMapGenerator::pack(normal, pixel);

            // TODO: won't actually be written until we make the format GL_RGB
            // but we need to rewrite the curvature generator first
            //pixel.b() = 0.0f; // 0.5f*(1.0f+normalMap->getCurvature(s, t));

            write(pixel, s, t);

            if (ruggedness)
            {
                writeRuggedness(riPixel, s, t);
            }
        }
    }

    osg::Texture2D* normalTex = new osg::Texture2D(image.get());

    normalTex->setInternalFormat(GL_RG8);
    normalTex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    normalTex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
    normalTex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
    normalTex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
    normalTex->setResizeNonPowerOfTwoHint(false);
    normalTex->setMaxAnisotropy(1.0f);
    normalTex->setUnRefImageDataAfterApply(Registry::instance()->unRefImageDataAfterApply().get());

    return normalTex;
}
