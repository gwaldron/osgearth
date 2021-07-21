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
#include <osgEarth/FeatureRasterizer>
#include <osgEarth/Metrics>
#include <osgEarth/Registry>

#include <osgEarth/FeatureSource>
#include <osgEarth/StyleSheet>

using namespace osgEarth;

#define LC "[FeatureRasterizer] : "

#ifdef OSGEARTH_HAVE_BLEND2D
#define USE_BLEND2D
#endif

#ifdef USE_BLEND2D
#include <blend2d.h>
#endif

#include <osgEarth/AGG.h>
#include <osgEarth/BufferFilter>
#include <osgEarth/ResampleFilter>

namespace osgEarth {
    namespace FeatureImageLayerImpl
    {
        struct RenderFrame
        {
            double xmin, ymin, xmax, ymax;
            double xf, yf;
        };

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
                } while (--count);
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
                } while (--count);
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
            unsigned a = (unsigned)(127.0f + (color.a()*255.0f) / 2.0f); // scale alpha up

            agg::rgba8 fgColor = agg::rgba8(
                (unsigned)(color.r()*255.0f),
                (unsigned)(color.g()*255.0f),
                (unsigned)(color.b()*255.0f),
                a);

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

#ifdef USE_BLEND2D

        void rasterizePolygons_blend2d(
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
                    y = ctx.targetHeight() - y;

                    if (p == part->begin())
                        path.moveTo(x, y);
                    else
                        path.lineTo(x, y);
                }
            });

            osg::Vec4 color = symbol->fill().isSet() ? symbol->fill()->color() : Color::White;
            // Fill the path
            ctx.setFillStyle(BLRgba(color.r(), color.g(), color.b(), color.a()));
            ctx.fillPath(path);

            // Also stroke a 1 pixel path around the polygon with the same color to help cover up any gaps between any adjoining features.
            ctx.setStrokeStyle(BLRgba(color.r(), color.g(), color.b(), color.a()));
            ctx.setStrokeWidth(1.0);
            ctx.strokePath(path);

        }

        void rasterizeLines(
            const Geometry* geometry,
            const LineSymbol* symbol,
            float lineWidth_px,
            RenderFrame& frame,
            BLContext& ctx)
        {
            OE_HARD_ASSERT(geometry != nullptr);
            OE_HARD_ASSERT(symbol != nullptr);

            OE_PROFILING_ZONE;

            BLPath path;

            geometry->forEachPart(true, [&](const Geometry* part)
            {
                for (Geometry::const_iterator p = part->begin(); p != part->end(); p++)
                {
                    const osg::Vec3d& p0 = *p;
                    double x = frame.xf*(p0.x() - frame.xmin);
                    double y = frame.yf*(p0.y() - frame.ymin);
                    y = ctx.targetHeight() - y;

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
                    y = ctx.targetHeight() - y;

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

            ctx.setStrokeStyle(BLRgba(color.r(), color.g(), color.b(), color.a()));

            ctx.setStrokeWidth(lineWidth_px);
            ctx.setStrokeCaps(cap);
            ctx.setStrokeJoin(join);
            ctx.strokePath(path);
        }

        static const BLFontFace& getOrCreateFontFace()
        {
            // TODO:  Proper font support
            static BLFontFace fontFace;
            static std::mutex fontMutex;
            if (fontFace.empty())
            {
                std::lock_guard<std::mutex> lock(fontMutex);
                auto defaultFont = osgEarth::Registry::instance()->getDefaultFont();
                fontFace.createFromFile(defaultFont->getFileName().c_str());
            }
            return fontFace;
        }


        // Simpler than StringExpression for doing template replacement.  Need to fix StringExpression so it supports inline templates as well like road_{number}
        std::string templateReplace(const Feature* feature, const std::string& expression)
        {
            std::string result = expression;
            for (auto& attr : feature->getAttrs())
            {
                std::string replaceText = Stringify() << "{" << attr.first << "}";
                osgEarth::replaceIn(result, replaceText, attr.second.getString());
            }
            return result;
        }

        void rasterizeSymbols(
            const Feature* feature,
            const StyleSheet* styleSheet,
            const TextSymbol* textSymbol,
            const SkinSymbol* skinSymbol,
            RenderFrame& frame,
            BLContext& ctx)
        {
            OE_HARD_ASSERT(feature != nullptr);

            OE_PROFILING_ZONE;

            BLFont font;

            Session* session = nullptr;

            if (styleSheet && skinSymbol && skinSymbol->name().isSet())
            {
                if (skinSymbol->library().isSet())
                {
                    osg::ref_ptr< ResourceLibrary > library = styleSheet->getResourceLibrary(skinSymbol->library().get());

                    StringExpression expression = skinSymbol->name().get();
                    std::string iconName = templateReplace(feature, expression.expr());

                    auto skin = library->getSkin(iconName);

                    if (skin)
                    {
                        osg::ref_ptr< osg::Image > image = skin->image().get();
                        if (!image.valid())
                        {
                            image = skin->createImage(nullptr);
                        }
                        if (image.valid())
                        {
                            // TODO:  Cache the flipped image as a BLImage so we don't need to flip it each time.
                            osg::ref_ptr< osg::Image > flippedImage = new osg::Image(*image);
                            flippedImage->flipVertical();
                            BLRectI iconRect(*skin->imageBiasS() * image->s(), *skin->imageBiasT() * image->t(), *skin->imageScaleS() * image->s(), *skin->imageScaleT() * image->t());

                            BLImage sprite;
                            sprite.createFromData(flippedImage->s(), flippedImage->t(), BL_FORMAT_PRGB32, flippedImage->data(), flippedImage->s() * 4);

                            ctx.setCompOp(BL_COMP_OP_SRC_OVER);

                            feature->getGeometry()->forEachPart([&](const Geometry* part)
                            {
                                // Only label points for now
                                for (Geometry::const_iterator p = part->begin(); p != part->end(); p++)
                                {
                                    const osg::Vec3d& p0 = *p;
                                    double x = frame.xf*(p0.x() - frame.xmin);
                                    double y = frame.yf*(p0.y() - frame.ymin);
                                    y = ctx.targetHeight() - y;

                                    ctx.translate(x, y);
                                    ctx.blitImage(BLPoint(-iconRect.w / 2.0, -iconRect.h / 2.0), sprite, iconRect);
                                    ctx.resetMatrix();
                                }
                            });
                        }
                    }
                }
            }

            if (textSymbol)
            {
                NumericExpression fontSizeExpression = textSymbol->size().get();
                std::string fontSizeText = templateReplace(feature, fontSizeExpression.expr());

                float fontSize = as<float>(fontSizeText, 12);//feature->eval(fontSizeExpression, session);
                font.createFromFace(getOrCreateFontFace(), fontSize);
                StringExpression expression = textSymbol->content().get();
                //std::string text = feature->eval(expression, session);
                std::string text = templateReplace(feature, expression.expr());

                feature->getGeometry()->forEachPart([&](const Geometry* part)
                {
                    for (Geometry::const_iterator p = part->begin(); p != part->end(); p++)
                    {
                        const osg::Vec3d& p0 = *p;
                        double x = frame.xf*(p0.x() - frame.xmin);
                        double y = frame.yf*(p0.y() - frame.ymin);
                        y = ctx.targetHeight() - y;

                        if (textSymbol->fill().isSet())
                        {
                            osgEarth::Color fillColor = textSymbol->fill()->color();
                            ctx.setFillStyle(BLRgba(fillColor.r(), fillColor.g(), fillColor.b(), fillColor.a()));
                            ctx.fillUtf8Text(BLPoint(x, y), font, text.c_str());
                        }

                        if (textSymbol->halo().isSet())
                        {
                            osgEarth::Color haloColor = textSymbol->halo()->color();
                            ctx.setStrokeStyle(BLRgba(haloColor.r(), haloColor.g(), haloColor.b(), haloColor.a()));
                            ctx.setStrokeWidth(1);
                            ctx.strokeUtf8Text(BLPoint(x, y), font, text.c_str());
                        }
                    }
                });
            }


        }

#endif
    }
}


using namespace osgEarth::FeatureImageLayerImpl;


FeatureRasterizer::FeatureRasterizer(
    unsigned int width, unsigned int height, 
    const GeoExtent& extent, 
    const Color& backgroundColor) :

    _extent(extent)
{
    // Allocate the image and initialize it to the background color
    _image = new osg::Image();
    _image->allocateImage(width, height, 1, GL_RGBA, GL_UNSIGNED_BYTE);
    ImageUtils::PixelWriter write(_image.get());

#ifdef USE_BLEND2D
    osg::Vec4 bg(backgroundColor.b(), backgroundColor.g(), backgroundColor.r(), backgroundColor.a());
    _implPixelFormat = RF_BGRA;
    _inverted = true;
#else
    osg::Vec4 bg(backgroundColor.a(), backgroundColor.b(), backgroundColor.g(), backgroundColor.r());
    _implPixelFormat = RF_ABGR;
    _inverted = false;
#endif

    write.assign(bg);    
}


FeatureRasterizer::FeatureRasterizer(
    osg::Image* image,
    const GeoExtent& extent) :

    _image(image),
    _extent(extent)
{
    //nop
}

void
FeatureRasterizer::render_blend2d(
    const FeatureList& features,
    const Style& style,
    const FeatureProfile* profile,
    const StyleSheet* sheet)
{
#ifdef USE_BLEND2D

    // agglite renders in this format:
    _implPixelFormat = RF_BGRA;
    _inverted = true;

    // find the symbology:
    const LineSymbol* masterLine = style.getSymbol<LineSymbol>();
    const PolygonSymbol* masterPoly = style.getSymbol<PolygonSymbol>();
    const CoverageSymbol* masterCov = style.getSymbol<CoverageSymbol>();
    const TextSymbol* masterText = style.getSymbol<TextSymbol>();
    const SkinSymbol* masterSkin = style.getSymbol<SkinSymbol>();

    // Converts coordinates to image space (s,t):
    RenderFrame frame;
    frame.xmin = _extent.xMin();
    frame.ymin = _extent.yMin();
    frame.xmax = _extent.xMax();
    frame.ymax = _extent.yMax();
    frame.xf = (double)_image->s() / _extent.width();
    frame.yf = (double)_image->t() / _extent.height();

    // set up the render target:
    BLImage buf;
    buf.createFromData(_image->s(), _image->t(), BL_FORMAT_PRGB32, _image->data(), _image->s() * 4);

    BLContext ctx(buf);
    ctx.setCompOp(BL_COMP_OP_SRC_OVER);

    // render polygons:
    if (masterPoly)
    {
        for (const auto& feature : features)
        {
            if (feature->getGeometry())
            {
                rasterizePolygons_blend2d(feature->getGeometry(), masterPoly, frame, ctx);
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
                    _extent.getSRS()->getUnits(),
                    _extent.yMin());

                double lineWidth_map_north = lineWidth.asDistance(
                    _extent.getSRS()->getUnits(),
                    _extent.yMax());

                double lineWidth_map = std::min(lineWidth_map_south, lineWidth_map_north);

                double pixelSize_map = _extent.height() / (double)_image->t();

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

    if (masterText || masterSkin)
    {
        // Rasterize the symbols:
        for (const auto& feature : features)
        {
            if (feature->getGeometry())
            {
                rasterizeSymbols(feature.get(), sheet, masterText, masterSkin, frame, ctx);
            }
        }
    }

    ctx.end();

#endif // USE_BLEND2D
}

void
FeatureRasterizer::render_agglite(
    const FeatureList& features,
    const Style& style,
    const FeatureProfile* profile,
    const StyleSheet* sheet)
{
    // agglite renders in this format:
    _implPixelFormat = RF_ABGR;
    _inverted = false;

    // find the symbology:
    const LineSymbol* masterLine = style.getSymbol<LineSymbol>();
    const PolygonSymbol* masterPoly = style.getSymbol<PolygonSymbol>();
    const CoverageSymbol* masterCov = style.getSymbol<CoverageSymbol>();

    // Converts coordinates to image space (s,t):
    RenderFrame frame;
    frame.xmin = _extent.xMin();
    frame.ymin = _extent.yMin();
    frame.xmax = _extent.xMax();
    frame.ymax = _extent.yMax();
    frame.xf = (double)_image->s() / _extent.width();
    frame.yf = (double)_image->t() / _extent.height();

    // sort into bins, making a copy for lines that require buffering.
    FeatureList polygons;
    FeatureList lines;

    FilterContext context;
    const SpatialReference* featureSRS = features.front()->getSRS();

    OE_SOFT_ASSERT_AND_RETURN(featureSRS != nullptr, void());

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
        GeoExtent transformedExtent = _extent.transform(featureSRS);

        double trans_xf = (double)_image->s() / transformedExtent.width();
        double trans_yf = (double)_image->t() / transformedExtent.height();

        // resolution of the image (pixel extents):
        double xres = 1.0 / trans_xf;
        double yres = 1.0 / trans_yf;

        // downsample the line data so that it is no higher resolution than to image to which
        // we intend to rasterize it. If you don't do this, you run the risk of the buffer
        // operation taking forever on very high-res input data.
        {
            OE_PROFILING_ZONE;
            ResampleFilter resample;
            resample.minLength() = osg::minimum(xres, yres);
            context = resample.push(lines, context);
        }

        // now run the buffer operation on all lines:
        BufferFilter buffer;

        GeoExtent imageExtentInFeatureSRS = _extent.transform(featureSRS);
        double pixelWidth = imageExtentInFeatureSRS.width() / (double)_image->s();
        double lineWidth = pixelWidth;

        if (masterLine)
        {
            buffer.capStyle() = masterLine->stroke()->lineCap().value();

            if (masterLine->stroke()->width().isSet())
            {
                lineWidth = masterLine->stroke()->width().value();

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
                            double mPerDegAtEquatorInv = 360.0 / (featureSRS->getEllipsoid().getRadiusEquator() * 2.0 * osg::PI);
                            GeoPoint ll = _extent.getCentroid();
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
        for (auto& polygon : polygons)
            polygon->transform(_extent.getSRS());
        for (auto& line : lines)
            line->transform(_extent.getSRS());
    }

    // set up the AGG renderer:
    agg::rendering_buffer rbuf(_image->data(), _image->s(), _image->t(), _image->s() * 4);

    // Create the renderer and the rasterizer
    agg::rasterizer ras;

    //ras.gamma(options().gamma().get());

    ras.filling_rule(agg::fill_even_odd);

    // construct an extent for cropping the geometry to our tile.
    // extend just outside the actual extents so we don't get edge artifacts:
    GeoExtent cropExtent = GeoExtent(_extent);
    cropExtent.scale(1.1, 1.1);
    double cropXMin, cropYMin, cropXMax, cropYMax;
    cropExtent.getBounds(cropXMin, cropYMin, cropXMax, cropYMax);

    // GEOS crop won't abide by weird extents, so if we're in geographic space
    // we must clamp the scaled extent back to a legal range.
    if (cropExtent.crossesAntimeridian())
    {
        GeoPoint centroid = _extent.getCentroid();
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
                if (!covValue.isSet())
                {
                    const PolygonSymbol* poly =
                        feature->style().isSet() && feature->style()->has<PolygonSymbol>() ? feature->style()->get<PolygonSymbol>() :
                        masterPoly;

                    Color color = poly ? poly->fill()->color() : Color::White;
                    rasterize_agglite(croppedGeometry.get(), color, frame, ras, rbuf);
                }
                else
                {
                    float value = feature->eval(covValue.mutable_value(), &context);
                    rasterizeCoverage_agglite(croppedGeometry.get(), value, frame, ras, rbuf);
                }
            }
        }


        if (!lines.empty())
        {
            float lineWidth = masterLine->stroke()->width().value();
            lineWidth = masterLine->stroke()->width().value();
            GeoExtent imageExtentInFeatureSRS = _extent.transform(featureSRS);
            double pixelWidth = imageExtentInFeatureSRS.width() / (double)_image->s();

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
                        double mPerDegAtEquatorInv = 360.0 / (featureSRS->getEllipsoid().getRadiusEquator() * 2.0 * osg::PI);
                        double lon, lat;
                        _extent.getCentroid(lon, lat);
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

                    osg::Vec4f color = line ? static_cast<osg::Vec4>(line->stroke()->color()) : osg::Vec4(1, 1, 1, 1);
                    rasterize_agglite(croppedGeometry.get(), color, frame, ras, rbuf);
                }
            }
        }
    }
}

void
FeatureRasterizer::render(
    const FeatureList& features,
    const Style& style,
    const FeatureProfile* profile,
    const StyleSheet* sheet)
{
    if (features.empty())
        return;

    OE_PROFILING_ZONE;

    OE_DEBUG << LC << "Rendering " << features.size() << " features" << std::endl;

    const SpatialReference* featureSRS = features.front()->getSRS();
    OE_SOFT_ASSERT_AND_RETURN(featureSRS != nullptr, void());

    // Transform to map SRS:
    if (!featureSRS->isHorizEquivalentTo(_extent.getSRS()))
    {
        OE_PROFILING_ZONE_NAMED("Transform");
        for (auto& feature : features)
            feature->transform(_extent.getSRS());
    }

#ifdef USE_BLEND2D
    if (style.get<CoverageSymbol>())
        render_agglite(features, style, profile, sheet);
    else
        render_blend2d(features, style, profile, sheet);
#else
    render_agglite(features, style, profile, sheet);
#endif
}

GeoImage
FeatureRasterizer::finalize()
{
    if (_image->getPixelSizeInBits() == 32 &&
        _image->getDataType() == GL_UNSIGNED_BYTE)
    {
        if (_implPixelFormat == RF_BGRA)
        {
            //convert from BGRA to RGBA
            unsigned char* pixel = _image->data();
            for (unsigned i = 0; i < _image->getTotalSizeInBytes(); i += 4, pixel += 4)
            {
                std::swap(pixel[0], pixel[2]);
            }
        }
        else if (_implPixelFormat == RF_ABGR)
        {
            //convert from ABGR to RGBA
            unsigned char* pixel = _image->data();
            for (unsigned i = 0; i < _image->getTotalSizeInBytes(); i += 4, pixel += 4)
            {
                std::swap(pixel[0], pixel[3]);
                std::swap(pixel[1], pixel[2]);
            }
        }
    }

    if (_inverted)
    {
        _image->flipVertical();
    }

    return GeoImage(_image.release(), _extent);
}

//........................................................................

FeatureStyleSorter::FeatureStyleSorter()
{
    //nop
}

void
FeatureStyleSorter::sort(
    const TileKey& key,
    const Distance& buffer,
    Session* session,
    FeatureFilterChain* filters,
    Function processFeaturesForStyle,
    ProgressCallback* progress) const
{
    OE_SOFT_ASSERT_AND_RETURN(session, void());
    OE_SOFT_ASSERT_AND_RETURN(session->getFeatureSource(), void());
    OE_SOFT_ASSERT_AND_RETURN(session->getFeatureSource()->getFeatureProfile(), void());

    OE_PROFILING_ZONE;

    Query defaultQuery;
    defaultQuery.tileKey() = key;

    FeatureSource* features = session->getFeatureSource();

    // figure out if and how to style the geometry.
    if (features->hasEmbeddedStyles())
    {
        // Each feature has its own embedded style data, so use that:
        FilterContext context;

        osg::ref_ptr<FeatureCursor> cursor = features->createFeatureCursor(
            key,
            buffer,
            filters,
            &context,
            progress);

        while (cursor.valid() && cursor->hasMore())
        {
            osg::ref_ptr< Feature > feature = cursor->nextFeature();
            if (feature.valid())
            {
                FeatureList data;
                data.push_back(feature);
                processFeaturesForStyle(feature->style().get(), data, progress);
            }
        }
    }
    else if (session->styles())
    {
        if (session->styles()->getSelectors().size() > 0)
        {
            for(auto& iter : session->styles()->getSelectors())
            {
                const StyleSelector& sel = iter.second;
                if (sel.styleExpression().isSet())
                {
                    const FeatureProfile* featureProfile = features->getFeatureProfile();

                    // establish the working bounds and a context:
                    FilterContext context(session, featureProfile);
                    StringExpression styleExprCopy(sel.styleExpression().get());

                    FeatureList features;
                    getFeatures(session, defaultQuery, buffer, key.getExtent(), filters, features, progress);
                    if (!features.empty())
                    {
                        std::unordered_map<std::string, Style> literal_styles;
                        std::map<const Style*, FeatureList> style_buckets;

                        for (FeatureList::iterator itr = features.begin(); itr != features.end(); ++itr)
                        {
                            Feature* feature = itr->get();

                            const std::string& styleString = feature->eval(styleExprCopy, &context);
                            if (!styleString.empty() && styleString != "null")
                            {
                                // resolve the style:
                                //Style combinedStyle;
                                const Style* resolved_style = nullptr;

                                // if the style string begins with an open bracket, it's an inline style definition.
                                if (styleString.length() > 0 && styleString[0] == '{')
                                {
                                    Config conf("style", styleString);
                                    conf.setReferrer(sel.styleExpression().get().uriContext().referrer());
                                    conf.set("type", "text/css");
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
                            processFeaturesForStyle(*style, list, progress);
                        }
                    }
                }
                else
                {
                    const Style* style = session->styles()->getStyle(sel.getSelectedStyleName());
                    Query query = sel.query().get();
                    query.tileKey() = key;

                    // Get the features
                    FeatureList features;
                    getFeatures(session, query, buffer, key.getExtent(), filters, features, progress);

                    processFeaturesForStyle(*style, features, progress);
                }
            }
        }
        else
        {
            const Style* style = session->styles()->getDefaultStyle();

            // Get the features
            FeatureList features;
            getFeatures(session, defaultQuery, buffer, key.getExtent(), filters, features, progress);
            
            processFeaturesForStyle(*style, features, progress);
        }
    }
    else
    {
        FeatureList features;
        getFeatures(session, defaultQuery, buffer, key.getExtent(), filters, features, progress);

        // Render the features
        Style emptyStyle;
        processFeaturesForStyle(emptyStyle, features, progress);
    }
}

void
FeatureStyleSorter::getFeatures(
    Session* session,
    const Query& query,
    const Distance& buffer,
    const GeoExtent& workingExtent,
    FeatureFilterChain* filters,
    FeatureList& features,
    ProgressCallback* progress) const
{
    OE_SOFT_ASSERT_AND_RETURN(session != nullptr, void());
    OE_SOFT_ASSERT_AND_RETURN(session->getFeatureSource() != nullptr, void());
    OE_SOFT_ASSERT_AND_RETURN(session->getFeatureSource()->getFeatureProfile() != nullptr, void());
    OE_SOFT_ASSERT_AND_RETURN(workingExtent.isValid(), void());

    OE_PROFILING_ZONE;

    // first we need the overall extent of the layer:
    const GeoExtent& featuresExtent = session->getFeatureSource()->getFeatureProfile()->getExtent();

    // convert them both to WGS84, intersect the extents, and convert back.
    GeoExtent featuresExtentWGS84 = featuresExtent.transform(featuresExtent.getSRS()->getGeographicSRS());
    GeoExtent workingExtentWGS84 = workingExtent.transform(featuresExtent.getSRS()->getGeographicSRS());
    GeoExtent queryExtentWGS84 = featuresExtentWGS84.intersectionSameSRS(workingExtentWGS84);
    if (queryExtentWGS84.isValid())
    {
        GeoExtent queryExtent = queryExtentWGS84.transform(featuresExtent.getSRS());

        // incorporate the image extent into the feature query for this style:
        Query localQuery = query;
        localQuery.bounds() =
            query.bounds().isSet() ? query.bounds()->unionWith(queryExtent.bounds()) :
            queryExtent.bounds();

        FilterContext context(session, session->getFeatureSource()->getFeatureProfile(), queryExtent);

        // now copy the resulting feature set into a list, converting the data
        // types along the way if a geometry override is in place:
        while (features.empty())
        {
            if (progress && progress->isCanceled())
                break;

            osg::ref_ptr<FeatureCursor> cursor;
            
            if (localQuery.tileKey().isSet())
            {
                cursor = session->getFeatureSource()->createFeatureCursor(
                    localQuery.tileKey().get(),
                    buffer,
                    filters,
                    &context,
                    progress);
            }
            else
            {
                cursor = session->getFeatureSource()->createFeatureCursor(
                    localQuery,
                    filters,
                    &context,
                    progress);
            }

            while (cursor.valid() && cursor->hasMore())
            {
                Feature* feature = cursor->nextFeature();
                if (feature->getGeometry())
                {
                    features.push_back(feature);
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
