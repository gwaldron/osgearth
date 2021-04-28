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
#include "SDF"
#include "Math"
#include "Metrics"
#include "FeatureRasterizer"
#include <osgEarth/rtree.h>
#include <osgEarth/TransformFilter>
#include <osgEarth/GLUtils>
#include <osg/BindImageTexture>
#include <osgDB/WriteFile>
#include <memory>

using namespace osgEarth;
using namespace osgEarth::Util;

namespace
{
    inline bool isPositivePowerOfTwo(unsigned x) {
        return (x & (x - 1)) == 0;
    }

    // https://www.comp.nus.edu.sg/~tants/jfa/i3d06.pdf
    const char* jfa_cs = R"(
    #version 430
    layout(local_size_x=1, local_size_y=1, local_size_z=1) in;

    // output image binding
    layout(binding=0, rg16f) uniform image2D buf;

    uniform int L;

    #define NODATA 32767.0

    float unit_remap(in float a, in float lo, in float hi)
    {
        return clamp((a-lo)/(hi-lo), 0.0, 1.0);
    }

    float squared_distance_2d(in vec4 a, in vec4 b)
    {
        vec2 c = b.xy-a.xy;
        return dot(c, c);
    }

    void main()
    {
        vec2 pixel_uv = vec2(
            float(gl_WorkGroupID.x) / float(gl_NumWorkGroups.x-1),
            float(gl_WorkGroupID.y) / float(gl_NumWorkGroups.y-1));

        int s = int(gl_WorkGroupID.x);
        int t = int(gl_WorkGroupID.y);

        vec4 pixel_points_to = imageLoad(buf, ivec2(gl_WorkGroupID));
        if (pixel_points_to.x == NODATA)
            return;

        vec4 remote;
        vec4 remote_points_to;

        for(int rs = s - L; rs <= s + L; rs += L)
        {
            if (rs < 0 || rs >= gl_NumWorkGroups.x) continue;
            remote.x = float(rs)/float(gl_NumWorkGroups.x-1);

            for(int rt = t - L; rt <= t + L; rt += L)
            {
                if (rt < 0 || rt >= gl_NumWorkGroups.y) continue;
                remote.y = float(rt)/float(gl_NumWorkGroups.y-1);

                remote_points_to = imageLoad(buf, ivec2(rs,rt));
                if (remote_points_to.x == NODATA)
                {
                    imageStore(buf, ivec2(rs,rt), pixel_points_to);
                }
                else
                {
                    // compare the distances and pick the closest.
                    float d_existing = squared_distance_2d(remote, remote_points_to);
                    float d_possible = squared_distance_2d(remote, pixel_points_to);

                    if (d_possible < d_existing)
                    {
                        imageStore(buf, ivec2(rs,rt), pixel_points_to);
                    }
                }
            }
        }
    }
    )";
}

SDFGenerator::SDFGenerator()
{
    _program = new osg::Program();
    _program->addShader(new osg::Shader(osg::Shader::COMPUTE, jfa_cs));
}

bool
SDFGenerator::createNearestNeighborField(
    const FeatureList& features,
    Session* session,
    unsigned size,
    const GeoExtent& extent,
    osg::ref_ptr<osg::Image>& buf,
    Cancelable* progress) const
{
    if (features.empty())
        return false;

    OE_SOFT_ASSERT_AND_RETURN(extent.isValid(), __func__, false);
    OE_SOFT_ASSERT_AND_RETURN(isPositivePowerOfTwo(size), __func__, false);
    OE_SOFT_ASSERT_AND_RETURN(session, __func__, false);
    OE_SOFT_ASSERT_AND_RETURN(session->getFeatureSource(), __func__, false);
    OE_SOFT_ASSERT_AND_RETURN(session->getFeatureSource()->getFeatureProfile(), __func__, false);

    // allocate the field as 2 floats
    osg::ref_ptr<osg::Image> image = new osg::Image();
    image->allocateImage(size, size, 1, GL_RG, GL_FLOAT);
    image->setInternalTextureFormat(GL_RG16F);

    // Render features to a temporary image
    Style style;
    if (features.front()->getGeometry()->isLinear())
        style.getOrCreate<LineSymbol>()->stroke()->color() = Color::Black;
    else
        style.getOrCreate<PolygonSymbol>()->fill()->color() = Color::Black;

    FeatureRasterizer rasterizer(size, size, extent, Color(1, 1, 1, 0));
    rasterizer.render(session, style, session->getFeatureSource()->getFeatureProfile(), features);
    osg::ref_ptr<osg::Image> source = rasterizer.finalize();

    // Convert pixels to local coordinates relative to the lower-left corner of extent
    if (!buf.valid())
    {
        buf = new osg::Image();
        buf->allocateImage(size, size, 1, GL_RG, GL_FLOAT);
        buf->setInternalTextureFormat(GL_RG16F);
    }

    ImageUtils::PixelReader readSource(source);
    ImageUtils::PixelReader readBuf(buf);
    ImageUtils::PixelWriter writeBuf(buf);

    constexpr float NODATA = 32767.0f;
    osg::Vec4f nodata(NODATA, NODATA, NODATA, NODATA);
    osg::Vec4f pixel, coord;
    ImageUtils::ImageIterator iter(source.get());
    iter.forEachPixel([&]()
        {
            readSource(pixel, iter.s(), iter.t());
            if (pixel.a() > 0.0f)
                coord.set(iter.u(), iter.v(), 0, 0);
            else
                coord = nodata;

            writeBuf(coord, iter.s(), iter.t());
        }
    );

    if (GPUJobArena::arena().getGraphicsContext().valid())
    {
        compute_nnf_on_gpu(buf.get());
    }
    else
    {
        compute_nnf_on_cpu(buf.get());
    }

    return true;
}

void
SDFGenerator::encodeDistanceField(
    const osg::Image* in_nnfield,
    osg::ref_ptr<osg::Image>& sdf,
    float extent,
    float lo,
    float hi,
    Cancelable* progress) const
{
    if (!sdf.valid())
    {
        sdf = new osg::Image();
        sdf->allocateImage(in_nnfield->s(), in_nnfield->t(), 1, GL_RED, GL_UNSIGNED_BYTE);
        sdf->setInternalTextureFormat(GL_R8);
        ImageUtils::PixelWriter write(sdf);
        write.assign(Color(1, 1, 1, 1));
    }

    ImageUtils::PixelReader reafSDF(sdf);
    ImageUtils::PixelWriter writeSDF(sdf);
    ImageUtils::PixelReader readNNF(in_nnfield);
    ImageUtils::ImageIterator b_iter(in_nnfield);
    osg::Vec4f me, closest, pixel;
    int c = 0; // clamp((int)(channel - GL_RED), 0, 3);
    b_iter.forEachPixel([&]()
        {
            reafSDF(pixel, b_iter.s(), b_iter.t());
            if (pixel[c] > 0.0f)
            {
                me.set(b_iter.u(), b_iter.v(), 0, 0);
                readNNF(closest, b_iter.s(), b_iter.t());
                float d = distance2D(me, closest);
                d = unitremap(d * extent, lo, hi);
                if (d < pixel[c])
                {
                    pixel[c] = d;
                    writeSDF(pixel, b_iter.s(), b_iter.t());
                }
            }
        }
    );
}


void
SDFGenerator::compute_nnf_on_gpu(osg::Image* image) const
{
    _computeSessions.lock();
    NNFSessionPtr& session = _computeSessions[getCurrentThreadId()];
    _computeSessions.unlock();

    if (session == nullptr)
    {
        session = std::make_shared<NNFSession>();
        session->setProgram(_program.get());
    }

    session->setImage(image);
    session->execute();
}

void
SDFGenerator::NNFSession::renderImplementation(osg::State* state)
{
    if (_L_uniform < 0)
    {
        const osg::Program::PerContextProgram* pcp = state->getLastAppliedProgramObject();
        _L_uniform = pcp->getUniformLocation(osg::Uniform::getNameID("L"));
    }

    osg::GLExtensions* ext = state->get<osg::GLExtensions>();

    // https://www.comp.nus.edu.sg/~tants/jfa/i3d06.pdf
    for (int L = _image->s() / 2; L >= 1; L /= 2)
    {
        ext->glUniform1i(_L_uniform, L);
        ext->glDispatchCompute(_image->s(), _image->t(), 1);
        ext->glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
    }
}

void
SDFGenerator::compute_nnf_on_cpu(osg::Image* buf) const
{
    // Jump-Flood algorithm for computing discrete voronoi
    // https://www.comp.nus.edu.sg/~tants/jfa/i3d06.pdf
    osg::Vec4f pixel_points_to;
    osg::Vec4f remote;
    osg::Vec4f remote_points_to;
    ImageUtils::PixelReader readBuf(buf);
    ImageUtils::PixelWriter writeBuf(buf);
    int n = buf->s();
    constexpr float NODATA = 32767;

    for (int L = n / 2; L >= 1; L /= 2)
    {
        ImageUtils::ImageIterator iter(readBuf);
        iter.forEachPixel([&]()
            {
                readBuf(pixel_points_to, iter.s(), iter.t());

                // no data at this pixel yet? skip it; there is nothing to propagate.
                if (pixel_points_to.x() == NODATA)
                    return;

                for (int s = iter.s() - L; s <= iter.s() + L; s += L)
                {
                    if (s < 0 || s >= readBuf.s()) continue;

                    remote[0] = (float)s / (float)(readBuf.s() - 1);

                    for (int t = iter.t() - L; t <= iter.t() + L; t += L)
                    {
                        if (t < 0 || t >= readBuf.t()) continue;
                        if (s == iter.s() && t == iter.t()) continue;

                        remote[1] = (float)t / (float)(readBuf.t() - 1);

                        // fetch the coords the remote pixel points to:
                        readBuf(remote_points_to, s, t);

                        if (remote_points_to.x() == NODATA) // remote is unset? Just copy
                        {
                            writeBuf(pixel_points_to, s, t);
                        }
                        else
                        {
                            // compare the distances and pick the closest.
                            float d_existing = distanceSquared2D(remote, remote_points_to);
                            float d_possible = distanceSquared2D(remote, pixel_points_to);

                            if (d_possible < d_existing)
                            {
                                writeBuf(pixel_points_to, s, t);
                            }
                        }
                    }
                }
            }
        );
    }
}



void FeatureSDFLayer::jfa(
    Session* session,
    const FeatureList& features,
    osg::ref_ptr<osg::Image>& out_image,
    unsigned size,
    const GeoExtent& extent,
    GLenum channel,
    const NumericExpression& min_dist_meters,
    const NumericExpression& max_dist_meters,
    bool invert,
    Cancelable* progress) const
{
    if (features.empty())
        return;

    //int n = out_image->s();

    // Poor man's degrees-to-meters conversion
    double toMeters = 1.0;
    if (extent.getSRS()->isGeographic())
    {
        double R = extent.getSRS()->getEllipsoid()->getRadiusEquator();
        toMeters = (2.0 * osg::PI * R / 360.0) * cos(osg::DegreesToRadians(extent.yMin()));
    }

    float lo = min_dist_meters.eval();
    float hi = max_dist_meters.eval();

#if 0

    // STEP 1 - render features to a source image.
    Style style;

    if (features.front()->getGeometry()->isLinear())
        style.getOrCreate<LineSymbol>()->stroke()->color() = Color::Black;
    else
        style.getOrCreate<PolygonSymbol>()->fill()->color() = Color::Black;

    FeatureRasterizer rasterizer(n, n, extent, Color(1, 1, 1, 0));
    rasterizer.render(session, style, session->getFeatureSource()->getFeatureProfile(), features);
    osg::ref_ptr<osg::Image> source = rasterizer.finalize();

    ImageUtils::PixelReader readSource(source);


    // STEP 2 - convert pixels to local coordinates
    // R = x, G = y; in [0..1] UV space
    osg::ref_ptr<osg::Image> buf;
    buf = new osg::Image();
    buf->allocateImage(n, n, 1, GL_RG, GL_FLOAT);
    buf->setInternalTextureFormat(GL_RG16F);

    ImageUtils::PixelReader readBuf(buf);
    ImageUtils::PixelWriter writeBuf(buf);

    constexpr float NODATA = 32767.0f;
    osg::Vec4f nodata(NODATA, NODATA, NODATA, NODATA);
    osg::Vec4f pixel, coord;
    ImageUtils::ImageIterator iter(source.get());
    iter.forEachPixel([&]()
        {
            readSource(pixel, iter.s(), iter.t());
            if (pixel.a() > 0.0f)
                coord.set(iter.u(), iter.v(), 0, 0);
            else
                coord = nodata;

            writeBuf(coord, iter.s(), iter.t());
        }
    );

    if (GPUJobArena::arena().getGraphicsContext().valid())
    {
        compute_sdf_on_gpu(buf.get());
    }
    else
    {
        compute_sdf_on_cpu(buf.get());
    }
#endif
    
    osg::ref_ptr<osg::Image> nnfield;

    _sdfGenerator.createNearestNeighborField(
        features,
        session,
        size,
        extent,
        nnfield,
        progress);

    if (nnfield.valid())
    {
        _sdfGenerator.encodeDistanceField(
            nnfield.get(),
            out_image,
            extent.height() * toMeters,
            lo,
            hi,
            progress);
    }

#if 0
    ImageUtils::PixelReader readOutput(out_image);
    ImageUtils::PixelWriter writeOutput(out_image);
    ImageUtils::ImageIterator b_iter(readBuf);
    osg::Vec4f me, closest;
    int c = clamp((int)(channel - GL_RED), 0, 3);
    b_iter.forEachPixel([&]()
        {
            readOutput(pixel, b_iter.s(), b_iter.t());
            if (pixel[c] > 0.0f)
            {
                me.set(b_iter.u(), b_iter.v(), 0, 0);
                readBuf(closest, b_iter.s(), b_iter.t());
                float d = distance2D(me, closest);
                d = unitremap(d*(float)(extent.height()*toMeters), lo, hi);
                if (d < pixel[c])
                {
                    pixel[c] = d;
                    writeOutput(pixel, b_iter.s(), b_iter.t());
                }
            }
        }
    );
#endif
}

#if 0 // OLD VECTOR-BASED SDF CODE.
void
SDFGenerator::encodeSDF(
    const FeatureList& features,
    osg::Image* image,
    const GeoExtent& extent,
    GLenum channel,
    FilterContext& fctx,
    const NumericExpression& min_dist_meters,
    const NumericExpression& max_dist_meters,
    bool invert,
    Cancelable* progress)
{
    OE_SOFT_ASSERT_AND_RETURN(image != nullptr, __func__, );
    OE_SOFT_ASSERT_AND_RETURN(extent.isValid(), __func__, );

    OE_PROFILING_ZONE;

    int c = clamp((int)(channel - GL_RED), 0, 3);
    osg::Vec3d p;
    osg::Vec4 pixel;

    NumericExpression mindist(min_dist_meters);
    NumericExpression maxdist(max_dist_meters);

    ImageUtils::PixelReader read(image);
    ImageUtils::PixelWriter write(image);

    // Poor man's degrees-to-meters conversion
    double toMeters = 1.0;
    if (extent.getSRS()->isGeographic())
    {
        double R = extent.getSRS()->getEllipsoid()->getRadiusEquator();
        toMeters = (2.0 * osg::PI * R / 360.0) * cos(osg::DegreesToRadians(extent.yMin()));
    }

    // Table of min and max SDF distances per feature:
    std::unordered_map<Feature*, std::pair<double, double>> distLUT;

    // Build a spatial index of features we are considering.
    // This is WAY faster than just iterating over all features.
    RTree<Feature*, double, 2> index;

    // The search radius, to constrain our search, will be equal
    // to the highest "max distance" taken from the feature set.
    double searchRadius = 0.0;

    double a_min[2], a_max[2];
    for (auto& feature : features)
    {
        const GeoExtent& e = feature->getExtent();
        a_min[0] = e.xMin(), a_min[1] = e.yMin();
        a_max[0] = e.xMax(), a_max[1] = e.yMax();
        index.Insert(a_min, a_max, feature.get());

        std::pair<double, double> limits(
            feature->eval(mindist, &fctx),
            feature->eval(maxdist, &fctx));

        distLUT.emplace(feature.get(), limits);

        searchRadius = std::max(limits.second, searchRadius);
    }
    searchRadius *= 1.1;

    std::vector<Feature*> hits;
    std::vector<double> ranges_squared;

    GeoImageIterator iter(image, extent);

    iter.forEachPixelOnCenter([&]()
        {
            read(pixel, iter.s(), iter.t());
            if (pixel[c] > 0.0)
            {
                p.x() = iter.x(), p.y() = iter.y();
                double nearest = 1.0;

                // Find all features within the search radius. We can't just say "find the 
                // one closest element" because the RTree only operates on the bounding-box
                // level. So instead we have to grab everything within the radius and manually
                // find the closest one.
                if (index.KNNSearch(p.ptr(), &hits, &ranges_squared, 0, searchRadius) > 0)
                {
                    for (int i = 0; i < hits.size() && nearest > 0.0; ++i)
                    {
                        Feature* feature = hits[i];
                        double range_squared = ranges_squared[i];
                        std::pair<double, double>& limits = distLUT[feature];

                        if (range_squared*toMeters <= limits.second*limits.second)
                        {
                            double sd = feature->getGeometry()->getSignedDistance2D(p) * toMeters;
                            if (invert) sd = -sd;
                            double sd_unit = unitremap(sd, limits.first, limits.second);
                            nearest = std::min(nearest, sd_unit);
                        }
                    }
                }

                if (nearest < pixel[c])
                {
                    pixel[c] = nearest;
                    write(pixel, iter.s(), iter.t());
                }
            }
        }
    );
}
#endif

//..............................................................

REGISTER_OSGEARTH_LAYER(featuresdf, FeatureSDFLayer);

Config
FeatureSDFLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    featureSource().set(conf, "features");
    styleSheet().set(conf, "styles");
    conf.set("invert", invert());

    if (filters().empty() == false)
    {
        Config temp;
        for (unsigned i = 0; i < filters().size(); ++i)
            temp.add(filters()[i].getConfig());
        conf.set("filters", temp);
    }

    return conf;
}

void
FeatureSDFLayer::Options::fromConfig(const Config& conf)
{
    invert().setDefault(false);

    featureSource().get(conf, "features");
    styleSheet().get(conf, "styles");
    conf.get("invert", invert());

    const Config& filtersConf = conf.child("filters");
    for (ConfigSet::const_iterator i = filtersConf.children().begin(); i != filtersConf.children().end(); ++i)
        filters().push_back(ConfigOptions(*i));
}

//........................................................................

#if 0
namespace
{
    // https://www.comp.nus.edu.sg/~tants/jfa/i3d06.pdf
    const char* jfa_cs = R"(
    #version 430
    layout(local_size_x=1, local_size_y=1, local_size_z=1) in;

    // output image binding
    layout(binding=0, rg16f) uniform image2D buf;

    uniform int L;

    #define NODATA 32767.0

    float unit_remap(in float a, in float lo, in float hi)
    {
        return clamp((a-lo)/(hi-lo), 0.0, 1.0);
    }

    float squared_distance_2d(in vec4 a, in vec4 b)
    {
        vec2 c = b.xy-a.xy;
        return dot(c, c);
    }

    void main()
    {
        vec2 pixel_uv = vec2(
            float(gl_WorkGroupID.x) / float(gl_NumWorkGroups.x-1),
            float(gl_WorkGroupID.y) / float(gl_NumWorkGroups.y-1));

        int s = int(gl_WorkGroupID.x);
        int t = int(gl_WorkGroupID.y);

        vec4 pixel_points_to = imageLoad(buf, ivec2(gl_WorkGroupID));
        if (pixel_points_to.x == NODATA)
            return;

        vec4 remote;
        vec4 remote_points_to;

        for(int rs = s - L; rs <= s + L; rs += L)
        {
            if (rs < 0 || rs >= gl_NumWorkGroups.x) continue;
            remote.x = float(rs)/float(gl_NumWorkGroups.x-1);

            for(int rt = t - L; rt <= t + L; rt += L)
            {
                if (rt < 0 || rt >= gl_NumWorkGroups.y) continue;
                remote.y = float(rt)/float(gl_NumWorkGroups.y-1);

                remote_points_to = imageLoad(buf, ivec2(rs,rt));
                if (remote_points_to.x == NODATA)
                {
                    imageStore(buf, ivec2(rs,rt), pixel_points_to);
                }
                else
                {
                    // compare the distances and pick the closest.
                    float d_existing = squared_distance_2d(remote, remote_points_to);
                    float d_possible = squared_distance_2d(remote, pixel_points_to);

                    if (d_possible < d_existing)
                    {
                        imageStore(buf, ivec2(rs,rt), pixel_points_to);
                    }
                }
            }
        }
    }

    )";
}
#endif

#undef LC
#define LC "[FeatureSDF] "

void
FeatureSDFLayer::init()
{
    ImageLayer::init();

    // Default profile (WGS84) if not set
    if (!getProfile())
    {
        setProfile(Profile::create("global-geodetic"));
    }
}

Status
FeatureSDFLayer::openImplementation()
{
    Status parent = ImageLayer::openImplementation();
    if (parent.isError())
        return parent;

    // assert a feature source:
    Status fsStatus = options().featureSource().open(getReadOptions());
    if (fsStatus.isError())
        return fsStatus;

    Status ssStatus = options().styleSheet().open(getReadOptions());
    if (ssStatus.isError())
        return ssStatus;

    establishProfile();

    _filterChain = FeatureFilterChain::create(options().filters(), getReadOptions());

    //_program = new osg::Program();
    //_program->addShader(new osg::Shader(osg::Shader::COMPUTE, jfa_cs));

    return Status::NoError;
}

void
FeatureSDFLayer::establishProfile()
{
    if (getProfile() == nullptr && getFeatureSource() != nullptr)
    {
        const FeatureProfile* fp = getFeatureSource()->getFeatureProfile();

        if (fp->getTilingProfile())
        {
            setProfile(fp->getTilingProfile());
        }
        else if (fp->getSRS())
        {
            setProfile(Profile::create(fp->getSRS()));
        }
    }
}

void
FeatureSDFLayer::addedToMap(const Map* map)
{
    ImageLayer::addedToMap(map);

    options().featureSource().addedToMap(map);
    options().styleSheet().addedToMap(map);

    if (getFeatureSource())
    {
        establishProfile();
        _session = new Session(map, getStyleSheet(), getFeatureSource(), getReadOptions());
        updateSession();
    }
}

void
FeatureSDFLayer::removedFromMap(const Map* map)
{
    options().featureSource().removedFromMap(map);
    options().styleSheet().removedFromMap(map);

    ImageLayer::removedFromMap(map);
}

void
FeatureSDFLayer::setFeatureSource(FeatureSource* fs)
{
    if (getFeatureSource() != fs)
    {
        options().featureSource().setLayer(fs);
        _featureProfile = 0L;

        if (fs)
        {
            if (fs->getStatus().isError())
            {
                setStatus(fs->getStatus());
            }
            else
            {
                // with a new feature source, we need to re-establish
                // the data extents and open a new session.
                updateSession();
            }
        }
    }
}

void
FeatureSDFLayer::setStyleSheet(StyleSheet* value)
{
    if (getStyleSheet() != value)
    {
        options().styleSheet().setLayer(value);
        if (_session.valid())
        {
            _session->setStyles(getStyleSheet());
        }
    }
}

void
FeatureSDFLayer::updateSession()
{
    if (_session.valid() && getFeatureSource())
    {
        const FeatureProfile* fp = getFeatureSource()->getFeatureProfile();

        dataExtents().clear();

        if (fp)
        {
            // recalculate the data extents based on the feature source.
            if (fp->getTilingProfile() != NULL)
            {
                // Use specified profile's GeoExtent
                unsigned maxLevel = fp->getMaxLevel();
                if (options().maxDataLevel().isSet())
                    maxLevel = osg::maximum(maxLevel, options().maxDataLevel().get());
                dataExtents().push_back(DataExtent(fp->getTilingProfile()->getExtent(), fp->getFirstLevel(), maxLevel));
            }
            else if (fp->getExtent().isValid() == true)
            {
                // Use FeatureProfile's GeoExtent
                dataExtents().push_back(DataExtent(fp->getExtent()));
            }

            // warn the user if the feature data is tiled and the
            // layer profile doesn't match the feature source profile
            if (fp->isTiled() &&
                fp->getTilingProfile()->isHorizEquivalentTo(getProfile()) == false)
            {
                OE_WARN << LC << "Layer profile doesn't match feature tiling profile - data may not render properly" << std::endl;
                OE_WARN << LC << "(Feature tiling profile = " << fp->getTilingProfile()->toString() << ")" << std::endl;
            }
        }

        _session->setFeatureSource(getFeatureSource());
        _session->setStyles(getStyleSheet());
    }
}

GeoImage
FeatureSDFLayer::createImageImplementation(
    const TileKey& key, 
    ProgressCallback* progress) const
{
    if (getStatus().isError())
    {
        return GeoImage::INVALID;
    }

    if (!getFeatureSource())
    {
        setStatus(Status::ServiceUnavailable, "No feature source");
        return GeoImage::INVALID;
    }

    const FeatureProfile* featureProfile = getFeatureSource()->getFeatureProfile();
    if (!featureProfile)
    {
        setStatus(Status::ConfigurationError, "Feature profile is missing");
        return GeoImage::INVALID;
    }

    const SpatialReference* featureSRS = featureProfile->getSRS();
    if (!featureSRS)
    {
        setStatus(Status::ConfigurationError, "Feature profile has no SRS");
        return GeoImage::INVALID;
    }

    if (!_session.valid())
    {
        setStatus(Status::AssertionFailure, "_session is NULL - call support");
        return GeoImage::INVALID;
    }

    // allocate the image.
    osg::ref_ptr<osg::Image> image = new osg::Image();
    image->allocateImage(getTileSize(), getTileSize(), 1, GL_RED, GL_UNSIGNED_BYTE);
    image->setInternalTextureFormat(GL_R8);
    ImageUtils::PixelWriter write(image);
    write.assign(Color(1, 1, 1, 1)); // Color::Red);

    FeatureStyleSorter::Function renderer = [&](
        const Style& style,
        FeatureList& features,
        ProgressCallback* progress)
    {
        // A processing context to use with the filters:
        FilterContext context(_session.get());
        context.setProfile(getFeatureSource()->getFeatureProfile());

        // Transform to map SRS:
        TransformFilter xform(key.getExtent().getSRS());
        xform.setLocalizeCoordinates(false);
        xform.push(features, context);

        jfa(
            _session.get(),
            features,
            image,
            image->s(),
            key.getExtent(),
            GL_RED,
            style.get<RenderSymbol>()->sdfMinDistance().get(),
            style.get<RenderSymbol>()->sdfMaxDistance().get(),
            options().invert().get(),
            progress);
    };

    FeatureStyleSorter sorter;
    sorter.sort(key, _session.get(), _filterChain.get(), renderer, progress);

    return GeoImage(image.get(), key.getExtent());
}

#if 0
void
FeatureSDFLayer::compute_sdf_on_gpu(
    osg::Image* image) const
{
    _jfa.lock();
    JFASessionPtr& session = _jfa[getCurrentThreadId()];
    _jfa.unlock();

    if (session == nullptr)
    {
        session = std::make_shared<JFASession>();
        session->setProgram(_program.get());
    }

    session->setImage(image);
    session->execute();
}

void 
FeatureSDFLayer::JFASession::renderImplementation(osg::State* state)
{
    if (_L_uniform < 0)
    {
        const osg::Program::PerContextProgram* pcp = state->getLastAppliedProgramObject();
        _L_uniform = pcp->getUniformLocation(osg::Uniform::getNameID("L"));
    }

    osg::GLExtensions* ext = state->get<osg::GLExtensions>();

    // https://www.comp.nus.edu.sg/~tants/jfa/i3d06.pdf
    for (int L = _image->s() / 2; L >= 1; L /= 2)
    {
        ext->glUniform1i(_L_uniform, L);
        ext->glDispatchCompute(_image->s(), _image->t(), 1);
        ext->glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
    }
}

void
FeatureSDFLayer::compute_sdf_on_cpu(osg::Image* buf) const
{
    // Jump-Flood algorithm for computing discrete voronoi
    // https://www.comp.nus.edu.sg/~tants/jfa/i3d06.pdf
    osg::Vec4f pixel_points_to;
    osg::Vec4f remote;
    osg::Vec4f remote_points_to;
    ImageUtils::PixelReader readBuf(buf);
    ImageUtils::PixelWriter writeBuf(buf);
    int n = buf->s();
    constexpr float NODATA = 32767;

    for (int L = n / 2; L >= 1; L /= 2)
    {
        ImageUtils::ImageIterator iter(readBuf);
        iter.forEachPixel([&]()
            {
                readBuf(pixel_points_to, iter.s(), iter.t());

                // no data at this pixel yet? skip it; there is nothing to propagate.
                if (pixel_points_to.x() == NODATA)
                    return;

                for (int s = iter.s() - L; s <= iter.s() + L; s += L)
                {
                    if (s < 0 || s >= readBuf.s()) continue;

                    remote[0] = (float)s / (float)(readBuf.s() - 1);

                    for (int t = iter.t() - L; t <= iter.t() + L; t += L)
                    {
                        if (t < 0 || t >= readBuf.t()) continue;
                        if (s == iter.s() && t == iter.t()) continue;

                        remote[1] = (float)t / (float)(readBuf.t() - 1);

                        // fetch the coords the remote pixel points to:
                        readBuf(remote_points_to, s, t);

                        if (remote_points_to.x() == NODATA) // remote is unset? Just copy
                        {
                            writeBuf(pixel_points_to, s, t);
                        }
                        else
                        {
                            // compare the distances and pick the closest.
                            float d_existing = distanceSquared2D(remote, remote_points_to);
                            float d_possible = distanceSquared2D(remote, pixel_points_to);

                            if (d_possible < d_existing)
                            {
                                writeBuf(pixel_points_to, s, t);
                            }
                        }
                    }
                }
            }
        );
    }
}
#endif
