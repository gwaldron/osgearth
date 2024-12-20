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
#include <osgText/String>
#include <osgEarth/BuildConfig>

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

#include <tuple>

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
                    if (cover > 0)
                        *f = c.value;
                    f++;
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

            if (!geometry->isPolygon())
            {
                return;
            }

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

        void lineSymbolToBLContext(const LineSymbol* line, BLContext& ctx)
        {
            auto cap = BL_STROKE_CAP_ROUND;
            auto join = BL_STROKE_JOIN_ROUND;

            if (line->stroke().isSet())
            {
                if (line->stroke()->lineCap().isSet())
                {
                    cap =
                        line->stroke()->lineCap() == Stroke::LINECAP_FLAT ? BL_STROKE_CAP_BUTT :
                        line->stroke()->lineCap() == Stroke::LINECAP_SQUARE ? BL_STROKE_CAP_SQUARE :
                        BL_STROKE_CAP_ROUND;
                }

                if (line->stroke()->lineJoin().isSet())
                {
                    join =
                        line->stroke()->lineJoin() == Stroke::LINEJOIN_MITRE ? BL_STROKE_JOIN_MITER_BEVEL :
                        BL_STROKE_JOIN_ROUND;
                }
            }

            ctx.setStrokeCaps(cap);
            ctx.setStrokeJoin(join);
        }

        void rasterizeLines(
            const Geometry* geometry,
            const Color& color,
            float lineWidth_px,
            RenderFrame& frame,
            BLContext& ctx)
        {
            OE_HARD_ASSERT(geometry != nullptr);

            if (!geometry->isPolygon() && !geometry->isLinear())
            {
                return;
            }

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

            //BLImage texture;
            //texture.readFromFile("../data/icon.png");
            //BLPattern pattern(texture);
            //ctx.setStrokeStyle(pattern);

            ctx.setStrokeStyle(BLRgba(color.r(), color.g(), color.b(), color.a()));
            ctx.setStrokeWidth(lineWidth_px);
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

        std::string templateReplace(const Feature* feature, const std::string& expression)
        {
            std::string result = expression;
            auto start = result.find('{');
            auto end = result.find('}');
            if (start != std::string::npos && end != std::string::npos)
            {
                std::string attribute = result.substr(start + 1, end - start - 1);
                if (feature->hasAttr(attribute))
                {
                    std::string value = feature->getString(attribute);
                    std::string replaceText = Stringify() << "{" << attribute << "}";
                    osgEarth::replaceIn(result, replaceText, value);
                }
            }

            if (result.find("{") != std::string::npos || result.find("}") != std::string::npos)
            {
                OE_INFO << LC << "Failed to replace attributes in template " << expression << std::endl;
                result = "";
            }
            return result;
        }

        void renderMapboxText(BLContext& ctx, float x, float y, const std::string& text, const TextSymbol* textSymbol, MapboxGLGlyphManager* glyphManager, float textScale, FeatureRasterizer::SymbolBoundingBoxes& symbolBounds)
        {
            if (!glyphManager)
            {
                return;
            }

#if 0
            // Draw the label position
            ctx.setFillStyle(BLRgba(0.0, 1.0, 0.0, 1.0));
            ctx.fillCircle(x, y, 4);
#endif

            float fontSize = textSymbol->size()->eval() * textScale;

            const float ONE_EM = 24.0;
            float scale = fontSize / ONE_EM;
            float baselineOffset = 7.0f;
            float GLYPH_PADDING = 3.0f;

            // Collect a list of glyphs for the text
            std::vector< osg::ref_ptr< MapboxGLGlyphManager::Glyph > > textGlyphs;
            glyphManager->getGlyphs(text, textSymbol->font().get(), textGlyphs);

            float cursorX = x;
            float cursorY = y;

            float lineWidth = 0.0f;
            float lineHeight = 0.0f;

            osg::BoundingBox bounds;

            // Compute the line width and line height
            for (auto& g : textGlyphs)
            {
                if (g.valid())
                {
                    float minX = cursorX + g->left * scale;
                    float minY = cursorY - g->top * scale;
                    float maxX = minX + g->width * scale;
                    float maxY = minY + g->height * scale;
                    bounds.expandBy(minX, minY, 0.0f);
                    bounds.expandBy(maxX, minY, 0.0f);
                    bounds.expandBy(maxX, maxY, 0.0f);
                    bounds.expandBy(minX, maxY, 0.0f);
                    cursorX += g->advance * scale;
                    //ctx.blitImage(BLPoint((double)g->left, (double)(-g->top)), sprite, glyphRect);
                    //lineWidth += g->advance * scale;
                    //lineHeight = std::max(lineHeight, (g->height) * scale);
                }
            }

            lineWidth = bounds.xMax() - bounds.xMin();
            lineHeight = bounds.yMax() - bounds.yMin();

            cursorX = x;
            cursorY = y;

            // Adjust the cursor based on the alignment
            auto alignment = textSymbol->alignment().get();

            switch (alignment)
            {
            case TextSymbol::ALIGN_CENTER_CENTER:
                cursorX -= lineWidth / 2.0;
                cursorY -= (ONE_EM * scale) * 0.5;
                break;
            case TextSymbol::ALIGN_LEFT_CENTER:
                cursorY -= (ONE_EM * scale) * 0.5;
                break;
            case TextSymbol::ALIGN_RIGHT_CENTER:
                cursorX -= lineWidth;
                cursorY -= (ONE_EM * scale) * 0.5;
                break;
            case TextSymbol::ALIGN_CENTER_TOP:
                cursorX -= lineWidth / 2.0;
                break;
            case TextSymbol::ALIGN_CENTER_BOTTOM:
                cursorX -= lineWidth / 2.0;
                cursorY -= (ONE_EM * scale);
                break;
            case TextSymbol::ALIGN_LEFT_TOP:
                // default
                break;
            case TextSymbol::ALIGN_RIGHT_TOP:
                cursorX -= lineWidth;
                break;
            case TextSymbol::ALIGN_LEFT_BOTTOM:
                cursorY -= (ONE_EM * scale);
                break;
            case TextSymbol::ALIGN_RIGHT_BOTTOM:
                cursorX -= lineWidth;
                cursorY -= (ONE_EM * scale);
                break;
            default:
                break;
            }            

            float offsetX = x - cursorX;
            float offsetY = y - cursorY;
            bounds.xMin() -= offsetX;
            bounds.yMin() -= offsetY;
            bounds.xMax() -= offsetX;
            bounds.yMax() -= offsetY;

            // Don't draw the text if it intersects an existing label.
            bool intersects = false;
            if (symbolBounds.intersects2d(bounds))
            {
                return;             
            }

            // Render each glyph
            for (unsigned int index = 0; index < textGlyphs.size(); ++index)
            {
                MapboxGLGlyphManager::Glyph* g = textGlyphs[index].get();

                if (!g)
                {                
                    continue;
                }

                //Write each glyph into the output image.
                unsigned glyphWidth = g->width + GLYPH_PADDING*2.0;
                unsigned glyphHeight = g->height + GLYPH_PADDING * 2.0;

                if (g->bitmap.size() > 0)
                {
                    auto textColor = textSymbol->fill()->color();

                    unsigned int numPixels = glyphWidth * glyphHeight;

                    unsigned char* glyphData = new unsigned char[g->bitmap.size() * 4]{ 0u };

                    for (unsigned int i = 0; i < numPixels; ++i)
                    {
                        unsigned char* base = &glyphData[i * 4];
                        unsigned char value = g->bitmap[i];
                        float maxValue = 192.0; // alpha 1
                        float minValue = 180.0;  // alpha 0
                        float alpha = osg::clampBetween(((value - minValue) / (maxValue - minValue)), 0.0f, 1.0f);
                        alpha *= textColor.a();

                        base[0] = (unsigned char)(textColor.b() * alpha * 255.0f);
                        base[1] = (unsigned char)(textColor.g() * alpha * 255.0f);
                        base[2] = (unsigned char)(textColor.r() * alpha * 255.0f);
                        base[3] = (unsigned char)(alpha * 255.0f);
                    }

                    BLImage sprite;
                    sprite.createFromData(glyphWidth, glyphHeight, BL_FORMAT_PRGB32, glyphData, glyphWidth * 4);

                    BLRectI glyphRect(0.0, 0.0, (double)glyphWidth, (double)glyphHeight);

                    ctx.translate(cursorX, cursorY);
                    ctx.scale(scale);
                    ctx.blitImage(BLPoint((double)g->left - GLYPH_PADDING, (double)(-g->top) - GLYPH_PADDING), sprite, glyphRect);
                    //ctx.blitImage(BLPoint(0, 0), sprite, glyphRect);

#if BL_VERSION >= 2820
                    ctx.resetTransform();
#else
                    ctx.resetMatrix();
#endif

#if 0
                    // Draw the text bounding box
                    ctx.setStrokeStyle(BLRgba(1.0, 0.0, 0.0, 1.0));
                    ctx.strokeBox(BLBoxI(bounds.xMin(), bounds.yMin(), bounds.xMax(), bounds.yMax()));
#endif

                    delete[] glyphData;
                }

                // Advance the cursor
                cursorX += (float)g->advance * scale;
            }

            symbolBounds._bounds.push_back(bounds);
        }

        void rasterizeSymbols(
            const Feature* feature,
            const StyleSheet* styleSheet,
            const TextSymbol* textSymbol,
            const SkinSymbol* skinSymbol,
            RenderFrame& frame,
            BLContext& ctx,
            MapboxGLGlyphManager* glyphManager,
            float scale,
            FeatureRasterizer::SymbolBoundingBoxes& symbolBounds)
        {
            OE_HARD_ASSERT(feature != nullptr);

            OE_PROFILING_ZONE;

            BLFont font;

            Session* session = nullptr;

            // Disable symbols for non-linear features until we have a better labeling strategy.
            if (!feature->getGeometry()->isPointSet())
            {
                return;
            }

            if (styleSheet && skinSymbol && skinSymbol->name().isSet())
            {
                if (skinSymbol->library().isSet())
                {
                    osg::ref_ptr< ResourceLibrary > library = styleSheet->getResourceLibrary(skinSymbol->library().get());

                    if (library.valid())
                    {
                        StringExpression expression = skinSymbol->name().get();
                        std::string iconName = templateReplace(feature, expression.expr());

                        auto skin = library->getSkin(iconName);

                        if (skin)
                        {
                            osg::ref_ptr< osg::Image > image = skin->image().get();
                            if (!image.valid())
                            {
                                image = skin->createColorImage(nullptr);
                            }
                            if (image.valid())
                            {
                                BLRectI iconRect(*skin->imageBiasS() * image->s(), *skin->imageBiasT() * image->t(), *skin->imageScaleS() * image->s(), *skin->imageScaleT() * image->t());

                                BLImage sprite;
                                sprite.createFromData(image->s(), image->t(), BL_FORMAT_PRGB32, image->data(), image->s() * 4);

                                ctx.setCompOp(BL_COMP_OP_SRC_OVER);

                                feature->getGeometry()->forEachPart([&](const Geometry* part)
                                    {
                                        // Only label points for now
                                        for (Geometry::const_iterator p = part->begin(); p != part->end(); p++)
                                        {
                                            const osg::Vec3d& p0 = *p;
                                            double x = frame.xf * (p0.x() - frame.xmin);
                                            double y = frame.yf * (p0.y() - frame.ymin);
                                            y = ctx.targetHeight() - y;

                                            ctx.translate(x, y);
                                            ctx.scale(scale);
                                            ctx.blitImage(BLPoint(-iconRect.w / 2.0, -iconRect.h / 2.0), sprite, iconRect);

#if BL_VERSION >= 2820
                                            ctx.resetTransform();
#else
                                            ctx.resetMatrix();
#endif
                                        }
                                    });
                            }
                        }
                        else
                        {
                            //OE_WARN << "Failed to get skin for " << iconName << std::endl;
                        }
                    }
                }
            }

            if (textSymbol)
            {
                NumericExpression fontSizeExpression = textSymbol->size().get();
                std::string fontSizeText = templateReplace(feature, fontSizeExpression.expr());

                float fontSize = as<float>(fontSizeText, 12) * scale;//feature->eval(fontSizeExpression, session);
                font.createFromFace(getOrCreateFontFace(), fontSize);
                StringExpression expression = textSymbol->content().get();
                //std::string text = feature->eval(expression, session);
                std::string text = templateReplace(feature, expression.expr());

                feature->getGeometry()->forEachPart([&](const Geometry* part)
                    {
                        for (Geometry::const_iterator p = part->begin(); p != part->end(); p++)
                        {
                            const osg::Vec3d& p0 = *p;
                            double x = frame.xf * (p0.x() - frame.xmin);
                            double y = frame.yf * (p0.y() - frame.ymin);
                            y = ctx.targetHeight() - y;

                            if (glyphManager)
                            {
                                // Use the mapboxgl font to render the text.
                                renderMapboxText(ctx, x, y, text, textSymbol, glyphManager, scale, symbolBounds);
                            }
                            else
                            {
                                // Just use Blend's default font rendering
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

MapboxGLGlyphManager* FeatureRasterizer::getGlyphManager() const
{
    return _glyphManager.get();
}

void FeatureRasterizer::setGlyphManager(MapboxGLGlyphManager* glyphManager)
{
    _glyphManager = glyphManager;
}

float FeatureRasterizer::getPixelScale() const
{
    return _pixelScale;
}

void FeatureRasterizer::setPixelScale(float pixelScale)
{
    _pixelScale = pixelScale;
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
    const PointSymbol* masterPoint = style.get<PointSymbol>();
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
        float outlineWidth_px = 0.0f;

        // Calculate the line width in pixels:
        if (masterLine->stroke()->width().isSet())
        {
            double lineWidthValue = masterLine->stroke()->width().value();

            // if the width units are specified, convert to pixels
            const optional<UnitsType> widthUnits = masterLine->stroke()->widthUnits();

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

        if (masterLine->stroke()->outlineWidth().isSet())
        {
            auto width = masterLine->stroke()->outlineWidth().get();

            if (width.getUnits() != Units::PIXELS)
            {
                double lineWidth_map_south = width.asDistance(
                    _extent.getSRS()->getUnits(),
                    _extent.yMin());

                double lineWidth_map_north = width.asDistance(
                    _extent.getSRS()->getUnits(),
                    _extent.yMax());

                double lineWidth_map = std::min(lineWidth_map_south, lineWidth_map_north);
                double pixelSize_map = _extent.height() / (double)_image->t();
                outlineWidth_px = (lineWidth_map / pixelSize_map);

                // enfore a minimum width of one pixel.
                float minPixels = masterLine->stroke()->minPixels().getOrUse(1.0f);
                outlineWidth_px = std::max(outlineWidth_px, minPixels);
            }

            else // pixels already
            {
                outlineWidth_px = width.getValue();
            }
        }

        lineSymbolToBLContext(masterLine, ctx);

        // Rasterize the lines:
        if (outlineWidth_px > 0.0f)
        {
            for (const auto& feature : features)
            {
                if (feature->getGeometry())
                {
                    rasterizeLines(
                        feature->getGeometry(),
                        masterLine->stroke()->outlineColor().get(),
                        outlineWidth_px,
                        frame, ctx);
                }
            }
        }

        for (const auto& feature : features)
        {
            if (feature->getGeometry())
            {
                rasterizeLines(
                    feature->getGeometry(),
                    masterLine->stroke()->color(),
                    lineWidth_px,
                    frame, ctx);
            }
        }
    }

    if (masterPoint)
    {
        float width = masterPoint->size().value();

        ctx.setFillStyle(BLRgba(
            masterPoint->fill()->color().r(), masterPoint->fill()->color().g(), masterPoint->fill()->color().b(), masterPoint->fill()->color().a()));

        for (const auto& feature : features)
        {
            feature->getGeometry()->forEachPart([&](const Geometry* part)
                {
                    for (auto& p : *part)
                    {
                        double x = frame.xf * (p.x() - frame.xmin);
                        double y = frame.yf * (p.y() - frame.ymin);
                        y = ctx.targetHeight() - y;
                        ctx.fillCircle(x, y, width / 2.0);
                    }
                });
        }
    }

    if (masterText || masterSkin)
    {   
        // Sort the features based their location.  We do this to ensure that features collected in a metatiling
        // fashion will always be rendered in the same order when rendered in multiple neighboring tiles
        // so the decluttering algorithm will work consistently across tiles.
        FeatureList sortedFeatures(features);
        std::sort(sortedFeatures.begin(), sortedFeatures.end(), [](auto& a, auto& b) {
            auto centerA = a->getGeometry()->getBounds().center();
            auto centerB = b->getGeometry()->getBounds().center();
            if (centerA.x() < centerB.x()) return true;
            if (centerA.x() > centerB.x()) return false;
            return centerA.y() < centerB.y();
            });

        // Rasterize the symbols:
        for (const auto& feature : sortedFeatures)
        {
            if (feature->getGeometry())
            {
                rasterizeSymbols(feature.get(), sheet, masterText, masterSkin, frame, ctx, _glyphManager.get(), getPixelScale(), _symbolBoundingBoxes);
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
    const LineSymbol* globalLineSymbol = style.getSymbol<LineSymbol>();
    const PolygonSymbol* globalPolySymbol = style.getSymbol<PolygonSymbol>();
    const CoverageSymbol* globalCovSymbol = style.getSymbol<CoverageSymbol>();

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

    for(auto& f : features)
    {
        const LineSymbol* lineSymbol = globalLineSymbol;
        const PolygonSymbol* polySymbol = globalPolySymbol;
        const CoverageSymbol* covSymbol = globalCovSymbol;

        if (f->getGeometry())
        {
            bool addPolygon = false;
            bool addLineOrOutline = false;

            // first see if the feature has overriding symbols:
            if (f->style()->has<PolygonSymbol>())
                polySymbol = f->style()->get<PolygonSymbol>();

            if (f->style()->has<LineSymbol>())
                lineSymbol = f->style()->get<LineSymbol>();

            if (f->style()->has<CoverageSymbol>())
                covSymbol = f->style()->get<CoverageSymbol>();

            // if it's a polygon and we have a polygon symbol, queue it
            if (f->getGeometry()->isPolygon())
            {
                if (polySymbol || covSymbol)
                {
                    addPolygon = true;
                }

                else if (lineSymbol || covSymbol)
                {
                    addLineOrOutline = true;
                }
            }

            // polygons can use the line symbol for an outline, so polygons AND lines 
            // can get queued as lines:
            else if (f->getGeometry()->isLinear())
            {
                if (lineSymbol || covSymbol)
                {
                    addLineOrOutline = true;
                }
            }

            if (addPolygon)
            {
                polygons.push_back(f);
            }

            if (addLineOrOutline)
            {
                // Use the GeometryIterator to get all the geometries so we can
                // clone the lines as rings
                GeometryIterator gi(f->getGeometry());
                while (gi.hasMore())
                {
                    Geometry* geom = gi.next();
                    Feature* newFeature = new Feature(*f.get());
                    newFeature->setGeometry(geom);
                    if (!newFeature->getGeometry()->isLinear())
                    {
                        newFeature->setGeometry(newFeature->getGeometry()->cloneAs(Geometry::TYPE_RING));
                    }
                    lines.push_back(newFeature);
                }
            }

            //// if there are no geometry symbols but there is a coverage symbol, default to polygons.
            //if (!hasLine && !hasPoly && covSymbol != nullptr)
            //{
            //    polygons.push_back(f);
            //}
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
        ResampleFilter resample;
        resample.minLength() = osg::minimum(xres, yres);
        context = resample.push(lines, context);

        // now run the buffer operation on all lines:
        BufferFilter buffer;

        GeoExtent imageExtentInFeatureSRS = _extent.transform(featureSRS);
        double pixelWidth = imageExtentInFeatureSRS.width() / (double)_image->s();
        double lineWidth = pixelWidth;

        if (globalLineSymbol)
        {
            buffer.capStyle() = globalLineSymbol->stroke()->lineCap().value();

            if (globalLineSymbol->stroke()->width().isSet())
            {
                lineWidth = globalLineSymbol->stroke()->width().value();

                // if the width units are specified, process them:
                if (globalLineSymbol->stroke()->widthUnits().isSet() &&
                    globalLineSymbol->stroke()->widthUnits().get() != Units::PIXELS)
                {
                    auto& featureUnits = featureSRS->getUnits();
                    auto& strokeUnits = globalLineSymbol->stroke()->widthUnits().value();

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
                            double lineWidthM = globalLineSymbol->stroke()->widthUnits()->convertTo(Units::METERS, lineWidth);
                            double mPerDegAtEquatorInv = 360.0 / (featureSRS->getEllipsoid().getRadiusEquator() * 2.0 * osg::PI);
                            GeoPoint ll = _extent.getCentroid();
                            lineWidth = lineWidthM * mPerDegAtEquatorInv * cos(osg::DegreesToRadians(ll.y()));
                        }
                    }

                    // enfore a minimum width of one pixel.
                    float minPixels = globalLineSymbol->stroke()->minPixels().getOrUse(1.0f);
                    lineWidth = osg::clampAbove(lineWidth, pixelWidth*minPixels);
                }

                else // pixels
                {
                    lineWidth *= pixelWidth;
                }
            }
        }

        buffer.distance() = lineWidth * 0.5;   // since the distance is for one side
        buffer.push(lines, context);
    }

    // Transform the features into the map's SRS:
    for (auto& polygon : polygons)
        polygon->transform(_extent.getSRS());

    for (auto& line : lines)
        line->transform(_extent.getSRS());

    // set up the AGG renderer:
    agg::rendering_buffer rbuf(
        _image->data(),
        _image->s(), _image->t(),
        _image->s() * 4);

    // Create the renderer and the rasterizer
    agg::rasterizer ras;
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

    Polygon cropPoly(4);
    cropPoly.push_back(osg::Vec3d(cropXMin, cropYMin, 0));
    cropPoly.push_back(osg::Vec3d(cropXMax, cropYMin, 0));
    cropPoly.push_back(osg::Vec3d(cropXMax, cropYMax, 0));
    cropPoly.push_back(osg::Vec3d(cropXMin, cropYMax, 0));

    // If there's a coverage symbol, make a copy of the expressions so we can evaluate them
    optional<NumericExpression> covValue;
    const CoverageSymbol* covSymbol = style.get<CoverageSymbol>();
    if (covSymbol && covSymbol->valueExpression().isSet())
    {
        covValue = covSymbol->valueExpression().get();
    }

    // render the polygons
    for (auto& feature : polygons)
    {
        Geometry* geometry = feature->getGeometry();

        if (auto cropped = geometry->crop(&cropPoly))
        {
            if (covValue.isSet())
            {
                float value = feature->eval(covValue.mutable_value(), &context);
                rasterizeCoverage_agglite(cropped.get(), value, frame, ras, rbuf);
            }
            else
            {
                const PolygonSymbol* poly =
                    feature->style().isSet() && feature->style()->has<PolygonSymbol>() ? feature->style()->get<PolygonSymbol>() :
                    globalPolySymbol;

                Color color = poly ? poly->fill()->color() : Color::White;
                rasterize_agglite(cropped.get(), color, frame, ras, rbuf);
            }
        }
    }

    for (auto& feature : lines)
    {
        Geometry* geometry = feature->getGeometry();

        osg::ref_ptr<Geometry> croppedGeometry;
        if (auto cropped = geometry->crop(&cropPoly))
        {
            if (covValue.isSet())
            {
                float value = feature->eval(covValue.mutable_value(), &context);
                rasterizeCoverage_agglite(cropped.get(), value, frame, ras, rbuf);
            }
            else
            {
                const LineSymbol* line =
                    feature->style().isSet() && feature->style()->has<LineSymbol>() ? feature->style()->get<LineSymbol>() :
                    globalLineSymbol;

                osg::Vec4f color = line ? static_cast<osg::Vec4>(line->stroke()->color()) : osg::Vec4(1, 1, 1, 1);
                rasterize_agglite(cropped.get(), color, frame, ras, rbuf);
            }
        }
    }

#if 0
    if (!lines.empty())
    {
        float lineWidth = globalLineSymbol->stroke()->width().value();
        lineWidth = globalLineSymbol->stroke()->width().value();
        GeoExtent imageExtentInFeatureSRS = _extent.transform(featureSRS);
        double pixelWidth = imageExtentInFeatureSRS.width() / (double)_image->s();

        // if the width units are specified, process them:
        if (globalLineSymbol->stroke()->widthUnits().isSet() &&
            globalLineSymbol->stroke()->widthUnits().get() != Units::PIXELS)
        {
            auto& featureUnits = featureSRS->getUnits();
            auto& strokeUnits = globalLineSymbol->stroke()->widthUnits().value();

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
                    double lineWidthM = globalLineSymbol->stroke()->widthUnits()->convertTo(Units::METERS, lineWidth);
                    double mPerDegAtEquatorInv = 360.0 / (featureSRS->getEllipsoid().getRadiusEquator() * 2.0 * osg::PI);
                    double lon, lat;
                    _extent.getCentroid(lon, lat);
                    lineWidth = lineWidthM * mPerDegAtEquatorInv * cos(osg::DegreesToRadians(lat));
                }
            }

            // enfore a minimum width of one pixel.
            float minPixels = globalLineSymbol->stroke()->minPixels().getOrUse(1.0f);
            lineWidth = osg::clampAbove(lineWidth, (float)pixelWidth*minPixels);
        }

        else // pixels
        {
            lineWidth *= pixelWidth;
        }

        for (auto& feature : lines)
        {
            Geometry* geometry = feature->getGeometry();

            osg::ref_ptr<Geometry> croppedGeometry;
            if (geometry->crop(cropPoly.get(), croppedGeometry))
            {
                const LineSymbol* line =
                    feature->style().isSet() && feature->style()->has<LineSymbol>() ? feature->style()->get<LineSymbol>() :
                    globalLineSymbol;

                osg::Vec4f color = line ? static_cast<osg::Vec4>(line->stroke()->color()) : osg::Vec4(1, 1, 1, 1);
                rasterize_agglite(croppedGeometry.get(), color, frame, ras, rbuf);
            }
        }
    }
#endif
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
        unsigned int totalSizeInBytes = _image->getTotalSizeInBytes();
        unsigned char* pixel = _image->data();

        if (_implPixelFormat == RF_BGRA)
        {
            //convert from BGRA to RGBA
            for (unsigned i = 0; i < totalSizeInBytes; i += 4, pixel += 4)
            {
                std::swap(pixel[0], pixel[2]);
            }
        }
        else if (_implPixelFormat == RF_ABGR)
        {
            //convert from ABGR to RGBA
            for (unsigned i = 0; i < totalSizeInBytes; i += 4, pixel += 4)
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
FeatureStyleSorter::sort_usingEmbeddedStyles(
    const TileKey& key,
    const Distance& buffer,
    const FeatureFilterChain& filters,
    Session* session,
    FeatureStyleSorter::Function processFeaturesForStyle,
    ProgressCallback* progress) const
{
    // Each feature has its own embedded style data, so use that:
    FilterContext context;

    Query query;
    query.tileKey() = key;
    query.buffer() = buffer;

    osg::ref_ptr<FeatureCursor> cursor = session->getFeatureSource()->createFeatureCursor(query, filters, &context, progress);

    while (cursor.valid() && cursor->hasMore())
    {
        auto feature = cursor->nextFeature();
        if (feature)
        {
            FeatureList data;
            data.push_back(feature);
            processFeaturesForStyle(feature->style().get(), data, progress);
        }
    }
}

void
FeatureStyleSorter::sort_usingSelectors(
    const TileKey& key,
    const Distance& buffer,
    const FeatureFilterChain& filters,
    Session* session,
    FeatureStyleSorter::Function processFeaturesForStyle,
    ProgressCallback* progress) const
{
    FeatureSource* features = session->getFeatureSource();

    Query defaultQuery;
    defaultQuery.tileKey() = key;

    for (auto& iter : session->styles()->getSelectors())
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
                //std::unordered_map<std::string, Style> literal_styles;
                std::unordered_map<std::string, std::pair<Style, int>> literal_styles;
                //std::map<const Style*, FeatureList> style_buckets;
                // keep ordered.
                std::map<int, std::pair<const Style*, FeatureList>> style_buckets;

                for (FeatureList::iterator itr = features.begin(); itr != features.end(); ++itr)
                {
                    Feature* feature = itr->get();

                    const std::string& styleString = feature->eval(styleExprCopy, &context);
                    if (!styleString.empty() && styleString != "null")
                    {
                        // resolve the style:
                        const Style* resolved_style = nullptr;
                        int resolved_index = 0;

                        // if the style string begins with an open bracket, it's an inline style definition.
                        if (styleString.length() > 0 && styleString[0] == '{')
                        {
                            Config conf("style", styleString);
                            conf.setReferrer(sel.styleExpression().get().uriContext().referrer());
                            conf.set("type", "text/css");
                            auto& literal_style_and_index = literal_styles[conf.toJSON()];
                            if (literal_style_and_index.first.empty())
                            {
                                literal_style_and_index.first = Style(conf);
                                // literal styles always come AFTER sheet styles
                                literal_style_and_index.second = literal_styles.size() + session->styles()->getStyles().size();
                            }
                            resolved_style = &literal_style_and_index.first;
                            resolved_index = literal_style_and_index.second;
                        }

                        // otherwise, look up the style in the stylesheet. Do NOT fall back on a default
                        // style in this case: for style expressions, the user must be explicit about
                        // default styling; this is because there is no other way to exclude unwanted
                        // features.
                        else
                        {
                            auto style_and_index = session->styles()->getStyleAndIndex(styleString);

                            //const Style* selected_style = session->styles()->getStyle(styleString, false);
                            if (style_and_index.first)
                            {
                                resolved_style = style_and_index.first;
                                resolved_index = style_and_index.second;
                            }
                        }

                        if (resolved_style)
                        {
                            auto& bucket = style_buckets[resolved_index];
                            bucket.first = resolved_style;
                            bucket.second.emplace_back(feature);
                        }
                    }
                }

                // in order:
                for (auto& iter : style_buckets)
                {
                    const Style* style = iter.second.first;
                    FeatureList& list = iter.second.second;
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

void
FeatureStyleSorter::sort_usingOneStyle(
    const Style& style,
    const TileKey& key,
    const Distance& buffer,
    const FeatureFilterChain& filters,
    Session* session,
    Function processFeaturesForStyle,
    ProgressCallback* progress) const
{
    Query defaultQuery;
    defaultQuery.tileKey() = key;

    FeatureList features;
    getFeatures(session, defaultQuery, buffer, key.getExtent(), filters, features, progress);

    processFeaturesForStyle(style, features, progress);
}

void
FeatureStyleSorter::sort(
    const TileKey& key,
    const Distance& buffer,
    Session* session,
    const FeatureFilterChain& filters,
    Function processFeaturesForStyle,
    ProgressCallback* progress) const
{
    OE_SOFT_ASSERT_AND_RETURN(session, void());
    OE_SOFT_ASSERT_AND_RETURN(session->getFeatureSource(), void());
    OE_SOFT_ASSERT_AND_RETURN(session->getFeatureSource()->getFeatureProfile(), void());

    OE_PROFILING_ZONE;

    if (session->getFeatureSource()->hasEmbeddedStyles())
    {
        sort_usingEmbeddedStyles(
            key,
            buffer,
            filters,
            session,
            processFeaturesForStyle,
            progress);

    }
    else if (session->styles())
    {
        if (session->styles()->getSelectors().size() > 0)
        {
            sort_usingSelectors(
                key,
                buffer,
                filters,
                session,
                processFeaturesForStyle,
                progress);

        }
        else
        {
            sort_usingOneStyle(
                *session->styles()->getDefaultStyle(),
                key,
                buffer,
                filters,
                session,
                processFeaturesForStyle,
                progress);
        }
    }
    else
    {
        sort_usingOneStyle(
            Style(), // empty style
            key,
            buffer,
            filters,
            session,
            processFeaturesForStyle,
            progress);
    }
}

void
FeatureStyleSorter::getFeatures(
    Session* session,
    const Query& query,
    const Distance& buffer,
    const GeoExtent& workingExtent,
    const FeatureFilterChain& filters,
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
            query.bounds().isSet() ? unionOf(query.bounds().get(), queryExtent.bounds()) :
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
                localQuery.buffer() = buffer;
            }

            cursor = session->getFeatureSource()->createFeatureCursor(localQuery, filters, &context, progress);

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
