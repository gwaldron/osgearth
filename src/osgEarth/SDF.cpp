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
#include <osgEarth/rtree.h>
#include <osgEarth/TransformFilter>
#include <osgEarth/GLUtils>
#include <osg/BindImageTexture>
#include <memory>

using namespace osgEarth;
using namespace osgEarth::Util;

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

    setupCompute();

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
    osg::ref_ptr<osg::Image> image;

    image = new osg::Image();
    image->allocateImage(getTileSize(), getTileSize(), 1, GL_RED, GL_UNSIGNED_BYTE);
    ImageUtils::PixelWriter write(image);
    write.assign(Color::Red);

    bool ok = render(key, _session.get(), getStyleSheet(), image.get(), progress);

    OE_SOFT_ASSERT(ok, __func__);

    return ok ? GeoImage(image.get(), key.getExtent()) : GeoImage::INVALID;
}


bool
FeatureSDFLayer::renderFeaturesForStyle(
    Session* session,
    const Style& style,
    const FeatureList& in_features,
    const GeoExtent& imageExtent,
    osg::Image* out_image,
    Cancelable* progress) const
{
    OE_PROFILING_ZONE;

    OE_DEBUG << LC << "Rendering " << in_features.size() << " features for " << imageExtent.toString() << "\n";

    // A processing context to use with the filters:
    FilterContext context(session);
    context.setProfile(getFeatureSource()->getFeatureProfile());

    // local (shallow) copy
    FeatureList features(in_features);

    // Transform to map SRS:
    {
        OE_PROFILING_ZONE_NAMED("Transform");
        TransformFilter xform(imageExtent.getSRS());
        xform.setLocalizeCoordinates(false);
        xform.push(features, context);
    }

#if 1

    SDFGenerator sdf;

    sdf.encodeSDF(
        features,
        out_image,
        imageExtent,
        GL_RED,
        context,
        style.get<RenderSymbol>()->sdfMinDistance().get(),
        style.get<RenderSymbol>()->sdfMaxDistance().get(),
        options().invert().get(),
        progress);

#else

    compute_sdf_on_gpu(
        out_image,
        features,
        osg::Vec2d(imageExtent.xMin(), imageExtent.yMin()),
        osg::Vec2f(imageExtent.width(), imageExtent.height()),
        osg::Vec2f(
            style.get<RenderSymbol>()->sdfMinDistance().get().eval(),
            style.get<RenderSymbol>()->sdfMaxDistance().get().eval()
        )
    );

#endif

    return true;
}

const char* sdf_cs = R"(
#version 430
layout(local_size_x=1, local_size_y=1, local_size_z=1) in;

// output image binding
layout(binding=0, r8) uniform image2D sdf;

// input data
struct Segment {
    vec2 a, b;
};
layout(binding=0, std430) readonly buffer Segments {
    Segment segments[];
};

// number of line segments
uniform int num_segments;
// width and height of image in local coords
uniform vec2 image_extent;
// min and max signed distances (mapped to 0 and 1 respectively)
uniform vec2 domain;

float unit_remap(in float a, in float lo, in float hi)
{
    return clamp((a-lo)/(hi-lo), 0.0, 1.0);
}

float squared_distance_to_line_segment(in vec2 p, in vec2 a, in vec2 b)
{
    vec2 n = b - a;
    vec2 pa = a - p;
    float c = dot(n, pa);
    if (c > 0.0) return dot(pa,pa);
    vec2 bp = p - b;
    if (dot(n,bp) > 0.0) return dot(bp,bp);   
    vec2 e = pa - n*(c/dot(n,n));
    return dot(e,e);
}

float min_signed_distance(in vec2 local)
{
    float d_squared = 9999999;
    for(int i=0; i<num_segments && d_squared > 0; ++i)
    {
        d_squared = min(
            d_squared,
            squared_distance_to_line_segment(local, segments[i].a, segments[i].b));
    }
    return unit_remap(sqrt(d_squared), domain[0], domain[1]);
}

void main()
{
    // coords in [0..1]
    vec2 pixelNDC = vec2(
        float(gl_WorkGroupID.x) / float(gl_NumWorkGroups.x-1),
        float(gl_WorkGroupID.y) / float(gl_NumWorkGroups.y-1));

    // coords in local space
    vec2 pixel_local = pixelNDC * image_extent.xy;

    // calculate SDF value
    float sd = min_signed_distance(pixel_local);

    // write it to the texture
    imageStore(sdf, ivec2(gl_WorkGroupID), vec4(sd));
}

)";

// EXPERIMENTAL.
// ONLY works for LINE SEGMENTS today.
void
FeatureSDFLayer::setupCompute()
{    
    osg::Shader* cs = new osg::Shader(osg::Shader::COMPUTE);
    cs->setShaderSource(sdf_cs);
    _computeProgram = new osg::Program();
    _computeProgram->addShader(cs);
}

void
FeatureSDFLayer::compute_sdf_on_gpu(
    osg::Image* out_image,
    const FeatureList& lines,
    const osg::Vec2d& local_origin,
    const osg::Vec2f& local_extent,
    const osg::Vec2f& distance_domain) const
{
    // populate the input buffer:
    _gpuData.lock();
    GPUSessionPtr& session = _gpuData[getCurrentThreadId()];
    _gpuData.unlock();
    if (session == nullptr)
    {
        session = std::make_shared<GPUSession>();
        session->_segments.setBindingIndex(0);

        session->_stateSet = new osg::StateSet();
        session->_stateSet->setAttribute(_computeProgram.get(), 1);

        session->_numSegments_uniform = session->_stateSet->getOrCreateUniform("num_segments", osg::Uniform::INT);
        session->_imageExtent_uniform = session->_stateSet->getOrCreateUniform("image_extent", osg::Uniform::FLOAT_VEC2);
        session->_domain_uniform = session->_stateSet->getOrCreateUniform("domain", osg::Uniform::FLOAT_VEC2);
        
        osg::Texture2D* tex = new osg::Texture2D();
        tex->setTextureSize(out_image->s(), out_image->t());
        tex->setSourceFormat(GL_RED);
        tex->setSourceType(GL_UNSIGNED_BYTE);
        tex->setInternalFormat(GL_R8);

        session->_stateSet->setTextureAttribute(0, tex, 1);
        session->_stateSet->addUniform(new osg::Uniform("sdf", 0));
        session->_stateSet->setAttribute(new osg::BindImageTexture(0, tex, osg::BindImageTexture::WRITE_ONLY, GL_R8, 0, GL_TRUE));

        session->_pbo = 0;
    }

    // calculate the required buffer size:
    int num_segments = 0;
    for (auto& feature : lines) {
        ConstGeometryIterator iter(feature->getGeometry());
        while (iter.hasMore()) {
            num_segments += iter.next()->size() - 1;
        }
    }

    session->_segments.setNumElements(num_segments);

    int ptr = 0;
    for (auto& feature : lines) {
        ConstGeometryIterator iter(feature->getGeometry());
        while (iter.hasMore()) {
            auto part = iter.next();
            for(int i=0; i<part->size()-1; ++i) {
                auto& segment = session->_segments[ptr];
                segment.endpoints[0] = (*part)[i].x() - local_origin.x();
                segment.endpoints[1] = (*part)[i].y() - local_origin.y();
                segment.endpoints[2] = (*part)[i+1].x() - local_origin.x();
                segment.endpoints[3] = (*part)[i+1].y() - local_origin.y();
                ++ptr;
            }
        }
    }

    // new data so mark dirty for new download.
    session->_segments.dirty();

    session->_numSegments_uniform->set(num_segments);
    session->_imageExtent_uniform->set(local_extent);
    session->_domain_uniform->set(distance_domain);
    
    auto render_job = GPUJob<bool>().dispatch(
        [session, out_image](osg::State* state, Cancelable* progress)
        {
            session->render(out_image, state);
            return true;
        }
    );

    auto readback_job = GPUJob<bool>().dispatch(
        [session, out_image](osg::State* state, Cancelable* progress)
        {
            session->readback(out_image, state);
            return true;
        }
    );

    // wait for completion
    readback_job.join();
}

void
FeatureSDFLayer::GPUSession::render(osg::Image* out_image, osg::State* state)
{
    osg::GLExtensions* ext = state->get<osg::GLExtensions>();

    if (_pbo == 0)
    {
        int size = out_image->s() * out_image->t() * 1u; // GL_RED
        ext->glGenBuffers(1, &_pbo);
        ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, _pbo);
        ext->glBufferData(GL_PIXEL_PACK_BUFFER_ARB, size, 0, GL_STREAM_READ);
    }

    _segments.apply(*state);
    state->apply(_stateSet.get());

    ext->glDispatchCompute(out_image->s(), out_image->t(), 1);
    //ext->glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

    // Post an async readback to the GL queue
    ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, _pbo);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RED, GL_UNSIGNED_BYTE, 0);
}

void
FeatureSDFLayer::GPUSession::readback(osg::Image* out_image, osg::State* state)
{
    using Clock = std::chrono::high_resolution_clock;
    using Tick = std::chrono::time_point<Clock>;
    Tick start = Clock::now();

    osg::GLExtensions* ext = state->get<osg::GLExtensions>();

    ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, _pbo);
    GLubyte* src = (GLubyte*)ext->glMapBuffer(GL_PIXEL_PACK_BUFFER_ARB, GL_READ_ONLY_ARB);
    if (src)
    {
        ::memcpy(out_image->data(), src, out_image->getTotalSizeInBytes());
        ext->glUnmapBuffer(GL_PIXEL_PACK_BUFFER_ARB);
    }
    ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, 0);

    Tick end = Clock::now();
    OE_WARN << "readback: " <<
        std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
        << "us " << std::endl;
}

