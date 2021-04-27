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
#include <osgEarth/TransformFilter>
#include <osgEarth/BufferFilter>
#include <osgEarth/ResampleFilter>
#include <osgEarth/AGG.h>
#include <osgEarth/Registry>

using namespace osgEarth;

#define LC "[FeatureRasterizer] : "

#ifdef OSGEARTH_HAVE_BLEND2D
#include <blend2d.h>
#endif

#ifndef OSGEARTH_HAVE_BLEND2D
#define USE_AGGLITE
#endif

namespace osgEarth {
    namespace FeatureImageLayerImpl
    {
        struct RenderFrame
        {
            double xmin, ymin, xmax, ymax;
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
                    y = ctx.targetHeight() - y;

                    if (p == part->begin())
                        path.moveTo(x, y);
                    else
                        path.lineTo(x, y);
                }
            });

            osg::Vec4 color = symbol->fill().isSet() ? symbol->fill()->color() : Color::White;
            ctx.setFillStyle(BLRgba(color.r(), color.g(), color.b(), color.a()));
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
            StyleSheet* styleSheet,
            const TextSymbol* textSymbol,
            const SkinSymbol* skinSymbol,
            RenderFrame& frame,
            BLContext& ctx)
        {
            OE_HARD_ASSERT(feature != nullptr, __func__);

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


FeatureRasterizer::FeatureRasterizer(unsigned int width, unsigned int height, const GeoExtent& extent, const Color& backgroundColor):
    _extent(extent)
{
    // Allocate the image and initialize it to the background color
    _image = new osg::Image();
    _image->allocateImage(width, height, 1, GL_RGBA, GL_UNSIGNED_BYTE);
    ImageUtils::PixelWriter write(_image.get());
    write.assign(backgroundColor);
}

void
FeatureRasterizer::render(
    Session* session,
    const Style& style,
    const FeatureProfile* profile,
    const FeatureList& in_features) const
{
    OE_PROFILING_ZONE;

    OE_DEBUG << LC << "Rendering " << in_features.size() << " features" << std::endl;

    // A processing context to use with the filters:
    FilterContext context(session);
    context.setProfile(profile);

    // local (shallow) copy
    FeatureList features(in_features);

    // TODO: do we need to resample?

    // Transform to map SRS:
    {
        OE_PROFILING_ZONE_NAMED("Transform");
        TransformFilter xform(_extent.getSRS());
        xform.setLocalizeCoordinates(false);
        xform.push(features, context);
    }

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

#ifndef USE_AGGLITE

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
                rasterizeSymbols(feature.get(), session->styles(), masterText, masterSkin, frame, ctx);
            }
        }
    }

    ctx.end();

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
        GeoExtent transformedExtent = _extent.transform(featureSRS);

        double trans_xf = (double)_image->s() / transformedExtent.width();
        double trans_yf = (double)_image->t() / transformedExtent.height();

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
                            double mPerDegAtEquatorInv = 360.0 / (featureSRS->getEllipsoid()->getRadiusEquator() * 2.0 * osg::PI);
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
        TransformFilter xform(_extent.getSRS());
        xform.setLocalizeCoordinates(false);
        xform.push(polygons, context);
        xform.push(lines, context);
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
                const PolygonSymbol* poly =
                    feature->style().isSet() && feature->style()->has<PolygonSymbol>() ? feature->style()->get<PolygonSymbol>() :
                    masterPoly;

                    Color color = poly ? poly->fill()->color() : Color::White;
                    rasterize_agglite(croppedGeometry.get(), color, frame, ras, rbuf);
            }
        }

        if (!lines.empty())
        {
            const SpatialReference* featureSRS = context.profile()->getSRS();
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
                        double mPerDegAtEquatorInv = 360.0 / (featureSRS->getEllipsoid()->getRadiusEquator() * 2.0 * osg::PI);
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
#endif
}

osg::Image* FeatureRasterizer::finalize()
{
#ifdef USE_AGGLITE
        //convert from ABGR to RGBA
        unsigned char* pixel = _image->data();
        for (int i = 0; i < _image->getTotalSizeInBytes(); i += 4, pixel += 4)
        {
            std::swap(pixel[0], pixel[3]);
            std::swap(pixel[1], pixel[2]);
        }
#else
    //convert from BGRA to RGBA
    unsigned char* pixel = _image->data();
    for (int i = 0; i < _image->getTotalSizeInBytes(); i += 4, pixel += 4)
    {
        std::swap(pixel[0], pixel[2]);
    }
    _image->flipVertical();
#endif

    return _image.release();
}
