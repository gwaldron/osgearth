
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

    GeoPoint
        north(key.getProfile()->getSRS()),
        south(key.getProfile()->getSRS()),
        east(key.getProfile()->getSRS()),
        west(key.getProfile()->getSRS());

    osg::Vec3 a[4];

    const GeoExtent& ex = key.getExtent();

    // fetch the base tile in order to get resolutions data.
    osg::ref_ptr<ElevationTexture> heights;
    pool->getTile(key, true, heights, workingSet, progress);

    if (!heights.valid())
        return NULL;

    // build the sample set.
    std::vector<osg::Vec4d> points(write.s() * write.t() * 4);
    int p = 0;
    for(int t=0; t<write.t(); ++t)
    {
        double v = (double)t/(double)(write.t()-1);
        double y = ex.yMin() + v*ex.height();
        east.y() = y;
        west.y() = y;

        for(int s=0; s<write.s(); ++s)
        {
            double u = (double)s/(double)(write.s()-1);
            double x = ex.xMin() + u*ex.width();
            north.x() = x;
            south.x() = x;

            double r = heights->getResolution(s, t);

            east.x() = x + r;
            west.x() = x - r;
            north.y() = y + r;
            south.y() = y - r;

            points[p++].set(west.x(), west.y(), 0.0, r);
            points[p++].set(east.x(), east.y(), 0.0, r);
            points[p++].set(south.x(), south.y(), 0.0, r);
            points[p++].set(north.x(), north.y(), 0.0, r);
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

    for(int t=0; t<write.t(); ++t)
    {
        double v = (double)t/(double)(write.t()-1);
        double y_or_lat = ex.yMin() + v*ex.height();

        for(int s=0; s<write.s(); ++s)
        {
            int p = (4*write.s()*t + 4*s);

            res.set(points[p].w(), res.getUnits());
            dx = res.asDistance(Units::METERS, y_or_lat);
            dy = res.asDistance(Units::METERS, 0.0);

            riPixel.r() = 0.0f;

            // only attempt to create a normal vector if all the data is valid:
            // a valid resolution value and four valid corner points.
            if (res.getValue() != FLT_MAX &&
                points[p+0].z() != NO_DATA_VALUE &&
                points[p+1].z() != NO_DATA_VALUE &&
                points[p+2].z() != NO_DATA_VALUE &&
                points[p+3].z() != NO_DATA_VALUE)
            {
                a[0].set(-dx, 0, points[p+0].z());
                a[1].set( dx, 0, points[p+1].z());
                a[2].set(0, -dy, points[p+2].z());
                a[3].set(0,  dy, points[p+3].z());

                normal = (a[1]-a[0]) ^ (a[3]-a[2]);
                normal.normalize();

                if (ruggedness)
                {
                    // rudimentary normalized ruggedness index
                    riPixel.r() = 0.25 * (
                        fabs(points[p + 0].z() - points[p + 3].z()) +
                        fabs(points[p + 1].z() - points[p + 0].z()) +
                        fabs(points[p + 2].z() - points[p + 1].z()) +
                        fabs(points[p + 3].z() - points[p + 2].z()));
                    riPixel.r() = clamp(riPixel.r() / (float)dy, 0.0f, 1.0f);
                    riPixel.r() = harden(harden(riPixel.r()));
                }
            }
            else
            {
                normal.set(0,0,1);
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

void
NormalMapGenerator::pack(const osg::Vec3& n, osg::Vec4& p)
{
    // octohodreal normal packing
    float d = 1.0/(fabs(n.x())+fabs(n.y())+fabs(n.z()));
    p.x() = n.x() * d;
    p.y() = n.y() * d;

    if (n.z() < 0.0)
    {
        p.x() = (1.0 - fabs(p.y())) * (p.x() >= 0.0? 1.0 : -1.0);
        p.y() = (1.0 - fabs(p.x())) * (p.y() >= 0.0? 1.0 : -1.0);
    }

    p.x() = 0.5f*(p.x()+1.0f);
    p.y() = 0.5f*(p.y()+1.0f);
}

void
NormalMapGenerator::unpack(const osg::Vec4& packed, osg::Vec3& normal)
{
    normal.x() = packed.x()*2.0-1.0;
    normal.y() = packed.y()*2.0-1.0;
    normal.z() = 1.0-fabs(normal.x())-fabs(normal.y());
    float t = osg::clampBetween(-normal.z(), 0.0f, 1.0f);
    normal.x() += (normal.x() > 0)? -t : t;
    normal.y() += (normal.y() > 0)? -t : t;
    normal.normalize();
}
