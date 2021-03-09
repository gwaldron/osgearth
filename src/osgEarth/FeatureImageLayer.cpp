/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
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
#include <osgEarth/FeatureImageLayer>
#include <osgEarth/Session>
#include <osgEarth/FeatureCursor>
#include <osgEarth/TransformFilter>
#include <osgEarth/BufferFilter>
#include <osgEarth/ResampleFilter>
#include <osgEarth/AGG.h>
#include <osgEarth/StyleSheet>
#include <osgEarth/Registry>
#include <osgEarth/Progress>
#include <osgEarth/LandCover>
#include <osgEarth/Metrics>
#include <osgEarth/SDF>
#include <blend2d.h>

using namespace osgEarth;

#define LC "[FeatureImageLayer] " << getName() << ": "

#ifndef OSGEARTH_HAVE_BLEND2D
#define USE_AGGLITE
#endif

REGISTER_OSGEARTH_LAYER(featureimage, FeatureImageLayer);
REGISTER_OSGEARTH_LAYER(feature_image, FeatureImageLayer);

//........................................................................

namespace osgEarth { namespace FeatureImageLayerImpl
{
    struct RenderFrame
    {
        double xmin, ymin;
        double xf, yf;
    };

#ifdef USE_AGGLITE
    struct float32
    {
        float32() : value(NO_DATA_VALUE) { }
        float32(float v) : value(v) { }
        float value;
    };

    struct span_coverage32
    {
        static void render(unsigned char* ptr,
                           int x,
                           unsigned count,
                           const unsigned char* covers,
                           const float32& c)
        {
            unsigned char* p = ptr + (x << 2);
            float* f = (float*)p;
            do
            {
                unsigned char cover = *covers++;
                int hasData = cover > 127;
                *f++ = hasData ? c.value : NO_DATA_VALUE;
            }
            while(--count);
        }

        static void hline(unsigned char* ptr,
                          int x,
                          unsigned count,
                          const float32& c)
        {
            unsigned char* p = ptr + (x << 2);
            float* f = (float*)p;
            do {
                *f++ = c.value;
            }
            while(--count);
        }

        static float32 get(unsigned char* ptr, int x)
        {
            unsigned char* p = ptr + (x << 2);
            float* f = (float*)p;
            return float32(*f);
        }
    };

    // rasterizes a geometry to color
    void rasterize_agglite(
        const Geometry* geometry,
        const osg::Vec4& color,
        RenderFrame& frame,
        agg::rasterizer& ras,
        agg::rendering_buffer& buffer)
    {
        unsigned a = (unsigned)(127.0f+(color.a()*255.0f)/2.0f); // scale alpha up

        agg::rgba8 fgColor = agg::rgba8( 
            (unsigned)(color.r()*255.0f), 
            (unsigned)(color.g()*255.0f), 
            (unsigned)(color.b()*255.0f), 
            a );

        ConstGeometryIterator gi( geometry );
        while( gi.hasMore() )
        {
            const Geometry* g = gi.next();

            for( Geometry::const_iterator p = g->begin(); p != g->end(); p++ )
            {
                const osg::Vec3d& p0 = *p;
                double x0 = frame.xf*(p0.x()-frame.xmin);
                double y0 = frame.yf*(p0.y()-frame.ymin);

                if ( p == g->begin() )
                    ras.move_to_d( x0, y0 );
                else
                    ras.line_to_d( x0, y0 );
            }
        }
        agg::renderer<agg::span_abgr32, agg::rgba8> ren(buffer);
        ras.render(ren, fgColor);

        ras.reset();
    }


    void rasterizeCoverage_agglite(
        const Geometry* geometry,
        float value,
        RenderFrame& frame,
        agg::rasterizer& ras,
        agg::rendering_buffer& buffer)
    {
        ConstGeometryIterator gi(geometry);
        while (gi.hasMore())
        {
            const Geometry* g = gi.next();

            for (Geometry::const_iterator p = g->begin(); p != g->end(); p++)
            {
                const osg::Vec3d& p0 = *p;
                double x0 = frame.xf*(p0.x() - frame.xmin);
                double y0 = frame.yf*(p0.y() - frame.ymin);

                if (p == g->begin())
                    ras.move_to_d(x0, y0);
                else
                    ras.line_to_d(x0, y0);
            }
        }

        agg::renderer<span_coverage32, float32> ren(buffer);
        ras.render(ren, value);
        ras.reset();
    }

#else

    void rasterizePolygons(
        const Geometry* geometry, 
        const PolygonSymbol* symbol, 
        RenderFrame& frame, 
        BLContext& ctx)
    {
        OE_PROFILING_ZONE;

        BLPath path;

        geometry->forEachPart([&](const Geometry* part)
            {
                for (Geometry::const_iterator p = part->begin(); p != part->end(); p++)
                {
                    const osg::Vec3d& p0 = *p;
                    double x = frame.xf*(p0.x() - frame.xmin);
                    double y = frame.yf*(p0.y() - frame.ymin);

                    if (p == part->begin())
                        path.moveTo(x, y);
                    else
                        path.lineTo(x, y);
                }
            });

        osg::Vec4 color = symbol->fill().isSet() ? symbol->fill()->color() : Color::White;
        ctx.setFillStyle(BLRgba32(color.asRGBA()));
        ctx.fillPath(path);
    }

    void rasterizeLines(
        const Geometry* geometry, 
        const LineSymbol* symbol, 
        float lineWidth_px,
        RenderFrame& frame, 
        BLContext& ctx)
    {
        OE_HARD_ASSERT(geometry != nullptr, __func__);
        OE_HARD_ASSERT(symbol != nullptr, __func__);

        OE_PROFILING_ZONE;

        BLPath path;

        geometry->forEachPart(true, [&](const Geometry* part)
            {
                for (Geometry::const_iterator p = part->begin(); p != part->end(); p++)
                {
                    const osg::Vec3d& p0 = *p;
                    double x = frame.xf*(p0.x() - frame.xmin);
                    double y = frame.yf*(p0.y() - frame.ymin);

                    if (p == part->begin())
                        path.moveTo(x, y);
                    else
                        path.lineTo(x, y);
                }

                if ((part->getType() == Geometry::TYPE_RING || part->getType() == Geometry::TYPE_POLYGON) &&
                    (part->front() != part->back()))
                {
                    const osg::Vec3d& p0 = part->front();
                    double x = frame.xf*(p0.x() - frame.xmin);
                    double y = frame.yf*(p0.y() - frame.ymin);
                    path.lineTo(x, y);
                }
            });

        Color color(Color::White);
        uint32_t cap = BL_STROKE_CAP_ROUND;
        uint32_t join = BL_STROKE_JOIN_ROUND;

        if (symbol->stroke().isSet())
        {
            color = symbol->stroke()->color();

            if (symbol->stroke()->lineCap().isSet())
            {
                cap =
                    symbol->stroke()->lineCap() == Stroke::LINECAP_FLAT ? BL_STROKE_CAP_BUTT :
                    symbol->stroke()->lineCap() == Stroke::LINECAP_SQUARE ? BL_STROKE_CAP_SQUARE :
                    BL_STROKE_CAP_ROUND;
            }

            if (symbol->stroke()->lineJoin().isSet())
            {
                join =
                    symbol->stroke()->lineJoin() == Stroke::LINEJOIN_MITRE ? BL_STROKE_JOIN_MITER_BEVEL :
                    BL_STROKE_JOIN_ROUND;
            }
        }



        //BLImage texture;
        //texture.readFromFile("../data/icon.png");
        //BLPattern pattern(texture);
        //ctx.setStrokeStyle(pattern);

        ctx.setStrokeStyle(BLRgba32(color.asRGBA()));

        ctx.setStrokeWidth(lineWidth_px);
        ctx.setStrokeCaps(cap);
        ctx.setStrokeJoin(join);
        ctx.strokePath(path);
    }

#endif

    //FeatureCursor* createCursor(
    //    FeatureSource* fs, 
    //    FeatureFilterChain* chain, 
    //    FilterContext& cx, 
    //    const Query& query, 
    //    ProgressCallback* progress)
    //{
    //    FeatureCursor* cursor = fs->createFeatureCursor(query, progress);
    //    if (cursor && chain)
    //    {
    //        cursor = new FilteredFeatureCursor(cursor, chain, cx);
    //    }
    //    return cursor;
    //}
}}

//........................................................................

Config
FeatureImageLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    featureSource().set(conf, "features");
    styleSheet().set(conf, "styles");
    conf.set("gamma", gamma());
    conf.set("sdf", sdf());
    conf.set("sdf_invert", sdf_invert());

    if (filters().empty() == false)
    {
        Config temp;
        for(unsigned i=0; i<filters().size(); ++i)
            temp.add( filters()[i].getConfig() );
        conf.set( "filters", temp );
    }

    return conf;
}

void
FeatureImageLayer::Options::fromConfig(const Config& conf)
{
    gamma().setDefault(1.3);
    sdf().setDefault(false);
    sdf_invert().setDefault(false);

    featureSource().get(conf, "features");
    styleSheet().get(conf, "styles");
    conf.get("gamma", gamma());
    conf.get("sdf", sdf());
    conf.get("sdf_invert", sdf_invert());

    const Config& filtersConf = conf.child("filters");
    for(ConfigSet::const_iterator i = filtersConf.children().begin(); i != filtersConf.children().end(); ++i)
        filters().push_back( ConfigOptions(*i) );
}

//........................................................................

using namespace osgEarth::FeatureImageLayerImpl;

void
FeatureImageLayer::init()
{
    ImageLayer::init();

    // Default profile (WGS84) if not set
    if (!getProfile())
    {
        setProfile(Profile::create("global-geodetic"));
    }
}

Status
FeatureImageLayer::openImplementation()
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

    return Status::NoError;
}

void
FeatureImageLayer::establishProfile()
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
FeatureImageLayer::addedToMap(const Map* map)
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
FeatureImageLayer::removedFromMap(const Map* map)
{
    options().featureSource().removedFromMap(map);
    options().styleSheet().removedFromMap(map);

    ImageLayer::removedFromMap(map);
}

void
FeatureImageLayer::setFeatureSource(FeatureSource* fs)
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
FeatureImageLayer::setStyleSheet(StyleSheet* value)
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
FeatureImageLayer::updateSession()
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
FeatureImageLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
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

    if ( options().coverage() == true )
    {
        image = LandCover::createImage(getTileSize());
    }
    else if (options().sdf() == true)
    {
        image = new osg::Image();
        image->allocateImage(getTileSize(), getTileSize(), 1, GL_RED, GL_UNSIGNED_BYTE);
    }
    else
    {
        image = new osg::Image();
        image->allocateImage(getTileSize(), getTileSize(), 1, GL_RGBA, GL_UNSIGNED_BYTE);
    }

    preProcess(image.get());

    bool ok = render(key, _session.get(), getStyleSheet(), image.get(), progress);

    if (ok)
    {
        postProcess(image.get());
        return GeoImage(image.get(), key.getExtent());
    }
    else
    {
        return GeoImage::INVALID;
    }
}

bool
FeatureImageLayer::preProcess(osg::Image* image) const
{
    OE_PROFILING_ZONE;

    if (options().sdf() == true)
    {
        ImageUtils::PixelWriter write(image);
        write.assign(Color::Red);
    }
    else
    {
        ::memset(image->data(), 0x00, image->getTotalSizeInBytes());
    }
    return true;
}

bool
FeatureImageLayer::postProcess(osg::Image* image) const
{
    OE_PROFILING_ZONE;

#ifdef USE_AGGLITE
    if (options().coverage() == false)
    {
        //convert from ABGR to RGBA
        unsigned char* pixel = image->data();
        for (int i = 0; i < image->getTotalSizeInBytes(); i += 4, pixel += 4)
        {
            std::swap(pixel[0], pixel[3]);
            std::swap(pixel[1], pixel[2]);
        }
    }
#endif

    return true;
}

bool
FeatureImageLayer::renderFeaturesForStyle(
    Session*           session,
    const Style&       style,
    const FeatureList& in_features,
    const GeoExtent&   imageExtent,
    osg::Image*        image,
    Cancelable*        progress) const
{
    OE_PROFILING_ZONE;

    OE_DEBUG << LC << "Rendering " << in_features.size() << " features for " << imageExtent.toString() << "\n";

    // A processing context to use with the filters:
    FilterContext context(session);
    context.setProfile(getFeatureSource()->getFeatureProfile());

    // local (shallow) copy
    FeatureList features(in_features);

    // TODO: do we need to resample?

    // Transform to map SRS:
    {
        OE_PROFILING_ZONE_NAMED("Transform");
        TransformFilter xform(imageExtent.getSRS());
        xform.setLocalizeCoordinates(false);
        xform.push(features, context);
    }

    if (options().sdf() == true)
    {
        SDFGenerator sdf;

        sdf.encodeSDF(
            features,
            image,
            imageExtent,
            GL_RED,
            context,
            style.get<RenderSymbol>()->sdfMinDistance().get(),
            style.get<RenderSymbol>()->sdfMaxDistance().get(),
            options().sdf_invert().get(),
            progress);

        return true;
    }

    // find the symbology:
    const LineSymbol* masterLine = style.getSymbol<LineSymbol>();
    const PolygonSymbol* masterPoly = style.getSymbol<PolygonSymbol>();
    const CoverageSymbol* masterCov = style.getSymbol<CoverageSymbol>();

    // Converts coordinates to image space (s,t):
    RenderFrame frame;
    frame.xmin = imageExtent.xMin();
    frame.ymin = imageExtent.yMin();
    frame.xf = (double)image->s() / imageExtent.width();
    frame.yf = (double)image->t() / imageExtent.height();

#ifndef USE_AGGLITE

    // set up the render target:
    BLImage buf;
    buf.createFromData(image->s(), image->t(), BL_FORMAT_PRGB32, image->data(), image->s() * 4);

    BLContext ctx(buf);
    ctx.setCompOp(BL_COMP_OP_SRC_OVER);

    // render polygons:
    if (masterPoly)
    {
        for (const auto& feature : features)
        {
            if (feature->getGeometry())
            {
                rasterizePolygons(feature->getGeometry(), masterPoly, frame, ctx);
            }
        }
    }

    if (masterLine)
    {
        float lineWidth_px = 1.0f;

        // Calculate the line width in pixels:
        if (masterLine->stroke()->width().isSet())
        {
            double lineWidthValue = masterLine->stroke()->width().value();

            // if the width units are specified, convert to pixels
            const optional<Units> widthUnits = masterLine->stroke()->widthUnits();

            if (widthUnits.isSet() && widthUnits != Units::PIXELS)
            {
                // NOTE. If the layer is projected (e.g. spherical meractor) but the map
                // is geographic, the line width will be inaccurate; this is because the
                // meractor image will be reprojected and the line widths will shrink
                // by varying degrees depending on location on the globe. Someday we will
                // address this but not today.

                Distance lineWidth(lineWidthValue, widthUnits.get());

                double lineWidth_map_south = lineWidth.asDistance(
                    imageExtent.getSRS()->getUnits(),
                    imageExtent.yMin());

                double lineWidth_map_north = lineWidth.asDistance(
                    imageExtent.getSRS()->getUnits(),
                    imageExtent.yMax());

                double lineWidth_map = std::min(lineWidth_map_south, lineWidth_map_north);

                double pixelSize_map = imageExtent.height() / (double)image->t();

                lineWidth_px = (lineWidth_map / pixelSize_map);

                // enfore a minimum width of one pixel.
                float minPixels = masterLine->stroke()->minPixels().getOrUse(1.0f);
                lineWidth_px = osg::clampAbove(lineWidth_px, minPixels);
            }

            else // pixels already
            {
                lineWidth_px = lineWidthValue;
            }
        }

        // Rasterize the lines:
        for (const auto& feature : features)
        {
            if (feature->getGeometry())
            {
                rasterizeLines(feature->getGeometry(), masterLine, lineWidth_px, frame, ctx);
            }
        }
    }

#else

    // sort into bins, making a copy for lines that require buffering.
    FeatureList polygons;
    FeatureList lines;

    for (FeatureList::const_iterator f = features.begin(); f != features.end(); ++f)
    {
        if (f->get()->getGeometry())
        {
            bool hasPoly = false;
            bool hasLine = false;

            if (masterPoly || f->get()->style()->has<PolygonSymbol>())
            {
                polygons.push_back(f->get());
                hasPoly = true;
            }

            if (masterLine || f->get()->style()->has<LineSymbol>())
            {
                // Use the GeometryIterator to get all the geometries so we can clone them as rings
                GeometryIterator gi(f->get()->getGeometry());
                while (gi.hasMore())
                {
                    Geometry* geom = gi.next();
                    // Create a new feature for each geometry
                    Feature* newFeature = new Feature(*f->get());
                    newFeature->setGeometry(geom);
                    if (!newFeature->getGeometry()->isLinear())
                    {
                        newFeature->setGeometry(newFeature->getGeometry()->cloneAs(Geometry::TYPE_RING));
                    }
                    lines.push_back(newFeature);
                    hasLine = true;
                }
            }

            // if there are no geometry symbols but there is a coverage symbol, default to polygons.
            if (!hasLine && !hasPoly)
            {
                if (masterCov || f->get()->style()->has<CoverageSymbol>())
                {
                    polygons.push_back(f->get());
                }
            }
        }
    }

    if (lines.size() > 0)
    {
        // We are buffering in the features native extent, so we need to use the
        // transformed extent to get the proper "resolution" for the image
        const SpatialReference* featureSRS = context.profile()->getSRS();
        GeoExtent transformedExtent = imageExtent.transform(featureSRS);

        double trans_xf = (double)image->s() / transformedExtent.width();
        double trans_yf = (double)image->t() / transformedExtent.height();

        // resolution of the image (pixel extents):
        double xres = 1.0 / trans_xf;
        double yres = 1.0 / trans_yf;

        // downsample the line data so that it is no higher resolution than to image to which
        // we intend to rasterize it. If you don't do this, you run the risk of the buffer
        // operation taking forever on very high-res input data.
        if (true) //options().optimizeLineSampling() == true)
        {
            OE_PROFILING_ZONE;
            ResampleFilter resample;
            resample.minLength() = osg::minimum(xres, yres);
            context = resample.push(lines, context);
        }

        // now run the buffer operation on all lines:
        BufferFilter buffer;
        double lineWidth = 1.0;
        if (masterLine)
        {
            buffer.capStyle() = masterLine->stroke()->lineCap().value();

            if (masterLine->stroke()->width().isSet())
            {
                lineWidth = masterLine->stroke()->width().value();

                GeoExtent imageExtentInFeatureSRS = imageExtent.transform(featureSRS);
                double pixelWidth = imageExtentInFeatureSRS.width() / (double)image->s();

                // if the width units are specified, process them:
                if (masterLine->stroke()->widthUnits().isSet() &&
                    masterLine->stroke()->widthUnits().get() != Units::PIXELS)
                {
                    const Units& featureUnits = featureSRS->getUnits();
                    const Units& strokeUnits = masterLine->stroke()->widthUnits().value();

                    // if the units are different than those of the feature data, we need to
                    // do a units conversion.
                    if (featureUnits != strokeUnits)
                    {
                        if (Units::canConvert(strokeUnits, featureUnits))
                        {
                            // linear to linear, no problem
                            lineWidth = strokeUnits.convertTo(featureUnits, lineWidth);
                        }
                        else if (strokeUnits.isLinear() && featureUnits.isAngular())
                        {
                            // linear to angular? approximate degrees per meter at the
                            // latitude of the tile's centroid.
                            double lineWidthM = masterLine->stroke()->widthUnits()->convertTo(Units::METERS, lineWidth);
                            double mPerDegAtEquatorInv = 360.0 / (featureSRS->getEllipsoid()->getRadiusEquator() * 2.0 * osg::PI);
                            GeoPoint ll = imageExtent.getCentroid();
                            lineWidth = lineWidthM * mPerDegAtEquatorInv * cos(osg::DegreesToRadians(ll.y()));
                        }
                    }

                    // enfore a minimum width of one pixel.
                    float minPixels = masterLine->stroke()->minPixels().getOrUse(1.0f);
                    lineWidth = osg::clampAbove(lineWidth, pixelWidth*minPixels);
                }

                else // pixels
                {
                    lineWidth *= pixelWidth;
                }
            }
        }

        OE_PROFILING_ZONE_NAMED("Buffer");
        buffer.distance() = lineWidth * 0.5;   // since the distance is for one side
        buffer.push(lines, context);
    }

    // Transform the features into the map's SRS:
    {
        OE_PROFILING_ZONE_NAMED("Transform");
        TransformFilter xform(imageExtent.getSRS());
        xform.setLocalizeCoordinates(false);
        xform.push(polygons, context);
        xform.push(lines, context);
    }

    // set up the AGG renderer:
    agg::rendering_buffer rbuf(image->data(), image->s(), image->t(), image->s() * 4);

    // Create the renderer and the rasterizer
    agg::rasterizer ras;

    // Setup the rasterizer
    if (options().coverage() == true)
        ras.gamma(1.0);
    else
        ras.gamma(options().gamma().get());

    ras.filling_rule(agg::fill_even_odd);

    // construct an extent for cropping the geometry to our tile.
    // extend just outside the actual extents so we don't get edge artifacts:
    GeoExtent cropExtent = GeoExtent(imageExtent);
    cropExtent.scale(1.1, 1.1);
    double cropXMin, cropYMin, cropXMax, cropYMax;
    cropExtent.getBounds(cropXMin, cropYMin, cropXMax, cropYMax);

    // GEOS crop won't abide by weird extents, so if we're in geographic space
    // we must clamp the scaled extent back to a legal range.
    if (cropExtent.crossesAntimeridian())
    {
        GeoPoint centroid = imageExtent.getCentroid();
        if (centroid.x() < 0.0) // tile is east of antimeridian
        {
            cropXMin = -180.0;
            cropXMax = cropExtent.east();
        }
        else
        {
            cropXMin = cropExtent.west();
            cropXMax = 180.0;
        }
    }

    osg::ref_ptr<Polygon> cropPoly = new Polygon(4);
    cropPoly->push_back(osg::Vec3d(cropXMin, cropYMin, 0));
    cropPoly->push_back(osg::Vec3d(cropXMax, cropYMin, 0));
    cropPoly->push_back(osg::Vec3d(cropXMax, cropYMax, 0));
    cropPoly->push_back(osg::Vec3d(cropXMin, cropYMax, 0));

    // If there's a coverage symbol, make a copy of the expressions so we can evaluate them
    optional<NumericExpression> covValue;
    const CoverageSymbol* covsym = style.get<CoverageSymbol>();
    if (covsym && covsym->valueExpression().isSet())
        covValue = covsym->valueExpression().get();

    {
        OE_PROFILING_ZONE_NAMED("Crop/Render");

        // render the polygons
        for (FeatureList::iterator i = polygons.begin(); i != polygons.end(); i++)
        {
            Feature*  feature = i->get();
            Geometry* geometry = feature->getGeometry();

            osg::ref_ptr<Geometry> croppedGeometry;
            if (geometry->crop(cropPoly.get(), croppedGeometry))
            {
                const PolygonSymbol* poly =
                    feature->style().isSet() && feature->style()->has<PolygonSymbol>() ? feature->style()->get<PolygonSymbol>() :
                    masterPoly;

                if (options().coverage() == true && covValue.isSet())
                {
                    float value = (float)feature->eval(covValue.mutable_value(), &context);
                    rasterizeCoverage_agglite(croppedGeometry.get(), value, frame, ras, rbuf);
                }
                else
                {
                    Color color = poly ? poly->fill()->color() : Color::White;
                    rasterize_agglite(croppedGeometry.get(), color, frame, ras, rbuf);
                }
            }
        }

        if (!lines.empty())
        {
            const SpatialReference* featureSRS = context.profile()->getSRS();
            float lineWidth = masterLine->stroke()->width().value();
            lineWidth = masterLine->stroke()->width().value();
            GeoExtent imageExtentInFeatureSRS = imageExtent.transform(featureSRS);
            double pixelWidth = imageExtentInFeatureSRS.width() / (double)image->s();

            // if the width units are specified, process them:
            if (masterLine->stroke()->widthUnits().isSet() &&
                masterLine->stroke()->widthUnits().get() != Units::PIXELS)
            {
                const Units& featureUnits = featureSRS->getUnits();
                const Units& strokeUnits = masterLine->stroke()->widthUnits().value();

                // if the units are different than those of the feature data, we need to
                // do a units conversion.
                if (featureUnits != strokeUnits)
                {
                    if (Units::canConvert(strokeUnits, featureUnits))
                    {
                        // linear to linear, no problem
                        lineWidth = strokeUnits.convertTo(featureUnits, lineWidth);
                    }
                    else if (strokeUnits.isLinear() && featureUnits.isAngular())
                    {
                        // linear to angular? approximate degrees per meter at the
                        // latitude of the tile's centroid.
                        double lineWidthM = masterLine->stroke()->widthUnits()->convertTo(Units::METERS, lineWidth);
                        double mPerDegAtEquatorInv = 360.0 / (featureSRS->getEllipsoid()->getRadiusEquator() * 2.0 * osg::PI);
                        double lon, lat;
                        imageExtent.getCentroid(lon, lat);
                        lineWidth = lineWidthM * mPerDegAtEquatorInv * cos(osg::DegreesToRadians(lat));
                    }
                }

                // enfore a minimum width of one pixel.
                float minPixels = masterLine->stroke()->minPixels().getOrUse(1.0f);
                lineWidth = osg::clampAbove(lineWidth, (float)pixelWidth*minPixels);
            }

            else // pixels
            {
                lineWidth *= pixelWidth;
            }

            for (FeatureList::iterator i = lines.begin(); i != lines.end(); i++)
            {
                Feature*  feature = i->get();
                Geometry* geometry = feature->getGeometry();

                osg::ref_ptr<Geometry> croppedGeometry;
                if (geometry->crop(cropPoly.get(), croppedGeometry))
                {
                    const LineSymbol* line =
                        feature->style().isSet() && feature->style()->has<LineSymbol>() ? feature->style()->get<LineSymbol>() :
                        masterLine;

                    if (options().coverage() == true && covValue.isSet())
                    {
                        float value = (float)feature->eval(covValue.mutable_value(), &context);
                        rasterizeCoverage_agglite(croppedGeometry.get(), value, frame, ras, rbuf);
                    }
                    else
                    {
                        osg::Vec4f color = line ? static_cast<osg::Vec4>(line->stroke()->color()) : osg::Vec4(1, 1, 1, 1);
                        rasterize_agglite(croppedGeometry.get(), color, frame, ras, rbuf);
                    }
                }
            }
        }
    }
#endif

    return true;
}

//........................................................................

bool
FeatureImageRenderer::render(
    const TileKey& key,
    Session* session,
    const StyleSheet* styles,
    osg::Image* target,
    ProgressCallback* progress) const
{
    OE_PROFILING_ZONE;

    Query defaultQuery;
    defaultQuery.tileKey() = key;

    FeatureSource* features = session->getFeatureSource();
    if (!features)
        return false;

    // figure out if and how to style the geometry.
    if (features->hasEmbeddedStyles() )
    {
        // Each feature has its own embedded style data, so use that:
        FilterContext context;

        osg::ref_ptr<FeatureCursor> cursor = features->createFeatureCursor(
            defaultQuery,
            _filterChain.get(),
            &context,
            progress);

        while( cursor.valid() && cursor->hasMore() )
        {
            osg::ref_ptr< Feature > feature = cursor->nextFeature();
            if ( feature )
            {
                FeatureList list;
                list.push_back( feature );

                renderFeaturesForStyle(
                    session,
                    *feature->style(),
                    list,
                    key.getExtent(),
                    target,
                    progress);
            }
        }
    }
    else if (styles)
    {
        if (styles->getSelectors().size() > 0 )
        {
            for(StyleSelectors::const_iterator i = styles->getSelectors().begin();
                i != styles->getSelectors().end();
                ++i)
            {
                const StyleSelector& sel = i->second;

                if ( sel.styleExpression().isSet() )
                {
                    const FeatureProfile* featureProfile = features->getFeatureProfile();

                    // establish the working bounds and a context:
                    FilterContext context(session, featureProfile);
                    StringExpression styleExprCopy(  sel.styleExpression().get() );

                    FeatureList features;
                    getFeatures(session, defaultQuery, key.getExtent(), features, progress);
                    if (!features.empty())
                    {
                        std::unordered_map<std::string, Style> literal_styles;
                        std::map<const Style*, FeatureList> style_buckets;

                        for (FeatureList::iterator itr = features.begin(); itr != features.end(); ++itr)
                        {
                            Feature* feature = itr->get();

                            const std::string& styleString = feature->eval( styleExprCopy, &context );
                            if (!styleString.empty() && styleString != "null")
                            {
                                // resolve the style:
                                //Style combinedStyle;
                                const Style* resolved_style = nullptr;

                                // if the style string begins with an open bracket, it's an inline style definition.
                                if ( styleString.length() > 0 && styleString[0] == '{' )
                                {
                                    Config conf( "style", styleString );
                                    conf.setReferrer( sel.styleExpression().get().uriContext().referrer() );
                                    conf.set( "type", "text/css" );
                                    Style& literal_style = literal_styles[conf.toJSON()];
                                    if (literal_style.empty())
                                        literal_style = Style(conf);
                                    resolved_style = &literal_style;
                                }

                                // otherwise, look up the style in the stylesheet. Do NOT fall back on a default
                                // style in this case: for style expressions, the user must be explicity about
                                // default styling; this is because there is no other way to exclude unwanted
                                // features.
                                else
                                {
                                    const Style* selected_style = session->styles()->getStyle(styleString, false);
                                    if (selected_style)
                                        resolved_style = selected_style;
                                }

                                if (resolved_style)
                                {
                                    style_buckets[resolved_style].push_back(feature);
                                }
                            }
                        }

                        for (auto& iter : style_buckets)
                        {
                            const Style* style = iter.first;
                            FeatureList& list = iter.second;

                            renderFeaturesForStyle(
                                session,
                                *style,
                                list,
                                key.getExtent(),
                                target,
                                progress);
                        }
                    }
                }
                else
                {
                    const Style* style = styles->getStyle( sel.getSelectedStyleName() );
                    Query query = sel.query().get();
                    query.tileKey() = key;
                    queryAndRenderFeaturesForStyle(session, *style, query, key.getExtent(), target, progress);
                }
            }
        }
        else
        {
            const Style* style = styles->getDefaultStyle();
            queryAndRenderFeaturesForStyle(session, *style, defaultQuery, key.getExtent(), target, progress);
        }
    }
    else
    {
        queryAndRenderFeaturesForStyle(session, Style(), defaultQuery, key.getExtent(), target, progress);
    }

    return true;
}


bool
FeatureImageRenderer::queryAndRenderFeaturesForStyle(
    Session*          session,
    const Style&      style,
    const Query&      query,
    const GeoExtent&  imageExtent,
    osg::Image*       out_image,
    ProgressCallback* progress) const
{
    OE_PROFILING_ZONE;

    // Get the features
    FeatureList features;
    getFeatures(session, query, imageExtent, features, progress);

    if (progress && progress->isCanceled())
        return false;

    if (!features.empty())
    {
        return renderFeaturesForStyle(
            session, 
            style, 
            features,
            imageExtent,
            out_image, 
            progress);
    }
    return false;
}

void
FeatureImageRenderer::getFeatures(
    Session* session,
    const Query& query,
    const GeoExtent& imageExtent,
    FeatureList& features,
    ProgressCallback* progress) const
{
    OE_PROFILING_ZONE;

    // first we need the overall extent of the layer:
    const GeoExtent& featuresExtent = session->getFeatureSource()->getFeatureProfile()->getExtent();

    // convert them both to WGS84, intersect the extents, and convert back.
    GeoExtent featuresExtentWGS84 = featuresExtent.transform( featuresExtent.getSRS()->getGeographicSRS() );
    GeoExtent imageExtentWGS84 = imageExtent.transform( featuresExtent.getSRS()->getGeographicSRS() );
    GeoExtent queryExtentWGS84 = featuresExtentWGS84.intersectionSameSRS( imageExtentWGS84 );
    if ( queryExtentWGS84.isValid() )
    {
        GeoExtent queryExtent = queryExtentWGS84.transform( featuresExtent.getSRS() );

        // incorporate the image extent into the feature query for this style:
        Query localQuery = query;
        localQuery.bounds() =
            query.bounds().isSet() ? query.bounds()->unionWith( queryExtent.bounds() ) :
            queryExtent.bounds();

        FilterContext context(session, session->getFeatureSource()->getFeatureProfile(), queryExtent);

        // now copy the resulting feature set into a list, converting the data
        // types along the way if a geometry override is in place:
        while (features.empty())
        {
            if (progress && progress->isCanceled())
                break;

            // query the feature source:
            //osg::ref_ptr<FeatureCursor> cursor = createCursor(session->getFeatureSource(), _filterChain.get(), context, localQuery, progress);

            osg::ref_ptr<FeatureCursor> cursor = session->getFeatureSource()->createFeatureCursor(
                localQuery,
                _filterChain.get(),
                &context,
                progress);

            while( cursor.valid() && cursor->hasMore() )
            {
                Feature* feature = cursor->nextFeature();
                if (feature->getGeometry())
                {
                    features.push_back( feature );
                }
            }

            // If we didn't get any features and we have a tilekey set, try falling back.
            if (features.empty() && 
                localQuery.tileKey().isSet() &&
                localQuery.tileKey()->valid())
            {
                localQuery.tileKey() = localQuery.tileKey().get().createParentKey();
                if (!localQuery.tileKey()->valid())
                {
                    // We fell back all the way to lod 0 and got nothing, so bail.
                    break;
                }
            }
            else
            {
                // Just bail, we didn't get any features and aren't using tilekeys
                break;
            }
        }
    }
}

