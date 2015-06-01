/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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

#include <osgEarthFeatures/FeatureTileSource>
#include <osgEarthFeatures/ResampleFilter>
#include <osgEarthFeatures/TransformFilter>
#include <osgEarthFeatures/BufferFilter>
#include <osgEarthSymbology/Style>
//TODO: replace this with GeometryRasterizer
#include <osgEarthSymbology/AGG.h>
#include <osgEarth/Registry>
#include <osgEarth/FileUtils>
#include <osgEarth/ImageUtils>

#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include "AGGLiteOptions"

#include <sstream>
#include <OpenThreads/Mutex>
#include <OpenThreads/ScopedLock>

#define LC "[AGGLite] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace osgEarth::Drivers;
using namespace OpenThreads;

namespace
{
    struct span_coverage32
    {
        //--------------------------------------------------------------------
        static void render(unsigned char* ptr, 
                           int x,
                           unsigned count, 
                           const unsigned char* covers, 
                           const agg::rgba8& c)
        {
            unsigned char* p = ptr + (x << 2);
            do
            {
                unsigned char cover = *covers++;
                int hasData = cover > 127;
                if ( hasData )
                {
                    *p++ = c.a;
                    *p++ = c.b;
                    *p++ = c.g;
                    *p++ = c.r;
                }
                else
                {
                    // 0xffffffff = nodata for coverages.
                    *p++ = 0xff;
                    *p++ = 0xff;
                    *p++ = 0xff;
                    *p++ = 0xff;
                }
            }
            while(--count);
        }

        //--------------------------------------------------------------------
        static void hline(unsigned char* ptr, 
                          int x,
                          unsigned count, 
                          const agg::rgba8& c)
        {
            unsigned char* p = ptr + (x << 2);
            do { *p++ = c.a; *p++ = c.b; *p++ = c.g; *p++ = c.r; } while(--count);
        }

        //--------------------------------------------------------------------
        static agg::rgba8 get(unsigned char* ptr, int x)
        {
            unsigned char* p = ptr + (x << 2);
            agg::rgba8 c;
            c.a = *p++; 
            c.b = *p++; 
            c.g = *p++;
            c.r = *p;
            return c;
        }
    };

    osg::Vec4f encodeCoverageValue(float value)
    {
        const float minValue = -8192.0f;
        const float maxValue =  8192.0f;
        const float tofixed   = 255.0/256.0;

        // normalize the input:
        float v = (value-minValue)/(maxValue-minValue); // [0..1)

        // encode as rgba. (http://goo.gl/6sGmZj)
        // this algorithm is incremental - use as many bytes as you need (starting
        // with r) for the precision you need. For example, we could use just RGB
        // and save the 'a' for another value like alpha/intensity.
        osg::Vec4f color;
        float integerPart;

        color.r() = modff(v * tofixed,               &integerPart);
        color.g() = modff(v * tofixed * 255.0f,      &integerPart);
        color.b() = modff(v * tofixed * 65025.0f,    &integerPart);
        color.a() = modff(v * tofixed * 16581375.0f, &integerPart);

        return color;
    }
}

/********************************************************************/

class AGGLiteRasterizerTileSource : public FeatureTileSource
{
public:
    struct RenderFrame {
        double xmin, ymin;
        double xf, yf;
    };

public:
    AGGLiteRasterizerTileSource( const TileSourceOptions& options ) : FeatureTileSource( options ),
        _options( options )
    {
        //nop
    }

    //override
    bool preProcess(osg::Image* image, osg::Referenced* buildData)
    {
        agg::rendering_buffer rbuf( image->data(), image->s(), image->t(), image->s()*4 );
        agg::renderer<agg::span_abgr32> ren(rbuf);

        // clear the buffer.
        if ( _options.coverage() == true )
        {
            // For coverage data, ~0 = no data.
            ren.clear(agg::rgba8(255,255,255,255));
        }
        else
        {
            ren.clear(agg::rgba8(0,0,0,0));
        }
        return true;
    }

    //override
    bool renderFeaturesForStyle(
        Session*           session,
        const Style&       style,
        const FeatureList& features,
        osg::Referenced*   buildData,
        const GeoExtent&   imageExtent,
        osg::Image*        image )
    {
        // A processing context to use with the filters:
        FilterContext context( session );
        context.setProfile( getFeatureSource()->getFeatureProfile() );

        const LineSymbol*    masterLine = style.getSymbol<LineSymbol>();
        const PolygonSymbol* masterPoly = style.getSymbol<PolygonSymbol>();
        const CoverageSymbol* masterCov = style.getSymbol<CoverageSymbol>();

        // sort into bins, making a copy for lines that require buffering.
        FeatureList polygons;
        FeatureList lines;

        for(FeatureList::const_iterator f = features.begin(); f != features.end(); ++f)
        {
            if ( f->get()->getGeometry() )
            {
                bool hasPoly = false;
                bool hasLine = false;

                if ( masterPoly || f->get()->style()->has<PolygonSymbol>() )
                {
                    polygons.push_back( f->get() );
                    hasPoly = true;
                }

                if ( masterLine || f->get()->style()->has<LineSymbol>() )
                {
                    Feature* newFeature = new Feature( *f->get() );
                    if ( !newFeature->getGeometry()->isLinear() )
                    {
                        newFeature->setGeometry( newFeature->getGeometry()->cloneAs(Geometry::TYPE_RING) );
                    }
                    lines.push_back( newFeature );
                    hasLine = true;
                }

                // if there are no geometry symbols but there is a coverage symbol, default to polygons.
                if ( !hasLine && !hasPoly )
                {
                    if ( masterCov || f->get()->style()->has<CoverageSymbol>() )
                    {
                        polygons.push_back( f->get() );
                    }
                }
            }
        }

        // initialize:
        RenderFrame frame;
        frame.xmin = imageExtent.xMin();
        frame.ymin = imageExtent.yMin();
        frame.xf   = (double)image->s() / imageExtent.width();
        frame.yf   = (double)image->t() / imageExtent.height();

        if ( lines.size() > 0 )
        {
            // We are buffering in the features native extent, so we need to use the
            // transformed extent to get the proper "resolution" for the image
            const SpatialReference* featureSRS = context.profile()->getSRS();
            GeoExtent transformedExtent = imageExtent.transform(featureSRS);

            double trans_xf = (double)image->s() / transformedExtent.width();
            double trans_yf = (double)image->t() / transformedExtent.height();

            // resolution of the image (pixel extents):
            double xres = 1.0/trans_xf;
            double yres = 1.0/trans_yf;

            // downsample the line data so that it is no higher resolution than to image to which
            // we intend to rasterize it. If you don't do this, you run the risk of the buffer 
            // operation taking forever on very high-res input data.
            if ( _options.optimizeLineSampling() == true )
            {
                ResampleFilter resample;
                resample.minLength() = osg::minimum( xres, yres );
                context = resample.push( lines, context );
            }

            // now run the buffer operation on all lines:
            BufferFilter buffer;
            double lineWidth = 1.0;
            if ( masterLine )
            {
                buffer.capStyle() = masterLine->stroke()->lineCap().value();

                if ( masterLine->stroke()->width().isSet() )
                {
                    lineWidth = masterLine->stroke()->width().value();

                    GeoExtent imageExtentInFeatureSRS = imageExtent.transform(featureSRS);
                    double pixelWidth = imageExtentInFeatureSRS.width() / (double)image->s();

                    // if the width units are specified, process them:
                    if (masterLine->stroke()->widthUnits().isSet() &&
                        masterLine->stroke()->widthUnits().get() != Units::PIXELS)
                    {
                        const Units& featureUnits = featureSRS->getUnits();
                        const Units& strokeUnits  = masterLine->stroke()->widthUnits().value();

                        // if the units are different than those of the feature data, we need to
                        // do a units conversion.
                        if ( featureUnits != strokeUnits )
                        {
                            if ( Units::canConvert(strokeUnits, featureUnits) )
                            {
                                // linear to linear, no problem
                                lineWidth = strokeUnits.convertTo( featureUnits, lineWidth );
                            }
                            else if ( strokeUnits.isLinear() && featureUnits.isAngular() )
                            {
                                // linear to angular? approximate degrees per meter at the 
                                // latitude of the tile's centroid.
                                double lineWidthM = masterLine->stroke()->widthUnits()->convertTo(Units::METERS, lineWidth);
                                double mPerDegAtEquatorInv = 360.0/(featureSRS->getEllipsoid()->getRadiusEquator() * 2.0 * osg::PI);
                                double lon, lat;
                                imageExtent.getCentroid(lon, lat);
                                lineWidth = lineWidthM * mPerDegAtEquatorInv * cos(osg::DegreesToRadians(lat));
                            }
                        }

                        // enfore a minimum width of one pixel.
                        float minPixels = masterLine->stroke()->minPixels().getOrUse( 1.0f );
                        lineWidth = osg::clampAbove(lineWidth, pixelWidth*minPixels);
                    }

                    else // pixels
                    {
                        lineWidth *= pixelWidth;
                    }
                }
            }

            buffer.distance() = lineWidth * 0.5;   // since the distance is for one side
            buffer.push( lines, context );
        }

        // Transform the features into the map's SRS:
        TransformFilter xform( imageExtent.getSRS() );
        xform.setLocalizeCoordinates( false );
        FilterContext polysContext = xform.push( polygons, context );
        FilterContext linesContext = xform.push( lines, context );

        // set up the AGG renderer:
        agg::rendering_buffer rbuf( image->data(), image->s(), image->t(), image->s()*4 );

        // Create the renderer and the rasterizer
        agg::rasterizer ras;

        // Setup the rasterizer
        if ( _options.coverage() == true )
            ras.gamma(1.0);
        else
            ras.gamma(_options.gamma().get());

        ras.filling_rule(agg::fill_even_odd);

        // construct an extent for cropping the geometry to our tile.
        // extend just outside the actual extents so we don't get edge artifacts:
        GeoExtent cropExtent = GeoExtent(imageExtent);
        cropExtent.scale(1.1, 1.1);

        osg::ref_ptr<Symbology::Polygon> cropPoly = new Symbology::Polygon( 4 );
        cropPoly->push_back( osg::Vec3d( cropExtent.xMin(), cropExtent.yMin(), 0 ));
        cropPoly->push_back( osg::Vec3d( cropExtent.xMax(), cropExtent.yMin(), 0 ));
        cropPoly->push_back( osg::Vec3d( cropExtent.xMax(), cropExtent.yMax(), 0 ));
        cropPoly->push_back( osg::Vec3d( cropExtent.xMin(), cropExtent.yMax(), 0 ));

        // If there's a coverage symbol, make a copy of the expressions so we can evaluate them
        optional<NumericExpression> covValue;
        const CoverageSymbol* covsym = style.get<CoverageSymbol>();
        if (covsym && covsym->valueExpression().isSet())
            covValue = covsym->valueExpression().get();

        // render the polygons
        for(FeatureList::iterator i = polygons.begin(); i != polygons.end(); i++)
        {
            Feature*  feature  = i->get();
            Geometry* geometry = feature->getGeometry();

            osg::ref_ptr<Geometry> croppedGeometry;
            if ( geometry->crop( cropPoly.get(), croppedGeometry ) )
            {
                const PolygonSymbol* poly =
                    feature->style().isSet() && feature->style()->has<PolygonSymbol>() ? feature->style()->get<PolygonSymbol>() :
                    masterPoly;
                
                osg::Vec4f color;

                if ( _options.coverage() == true && covValue.isSet() )
                {
                    float value = (float)feature->eval(covValue.mutable_value(), &context);
                    color = encodeCoverageValue( value );
                }
                else
                {
                    color = poly->fill()->color();
                }
                
                //const osg::Vec4 color = poly ? static_cast<osg::Vec4>(poly->fill()->color()) : osg::Vec4(1,1,1,1);
                rasterize(croppedGeometry.get(), color, frame, ras, rbuf);
            }
        }

        // render the lines
        for(FeatureList::iterator i = lines.begin(); i != lines.end(); i++)
        {
            Feature*  feature  = i->get();
            Geometry* geometry = feature->getGeometry();

            osg::ref_ptr<Geometry> croppedGeometry;
            if ( geometry->crop( cropPoly.get(), croppedGeometry ) )
            {
                const LineSymbol* line =
                    feature->style().isSet() && feature->style()->has<LineSymbol>() ? feature->style()->get<LineSymbol>() :
                    masterLine;

                osg::Vec4f color;

                if ( _options.coverage() == true && covValue.isSet() )
                {
                    float value = (float)feature->eval(covValue.mutable_value(), &context);
                    color = encodeCoverageValue( value );
                }
                else
                {
                    color = line ? static_cast<osg::Vec4>(line->stroke()->color()) : osg::Vec4(1,1,1,1);
                }

                rasterize(croppedGeometry.get(), color, frame, ras, rbuf);
            }
        }

        return true;
    }

    //override
    bool postProcess( osg::Image* image, osg::Referenced* data )
    {
        //convert from ABGR to RGBA
        unsigned char* pixel = image->data();
        for(int i=0; i<image->s()*image->t()*4; i+=4, pixel+=4)
        {
            std::swap( pixel[0], pixel[3] );
            std::swap( pixel[1], pixel[2] );
        }

        return true;
    }

    // rasterizes a geometry.
    void rasterize(const Geometry* geometry, const osg::Vec4& color, RenderFrame& frame, 
                   agg::rasterizer& ras, agg::rendering_buffer& buffer)
    {
        osg::Vec4 c = color;
        agg::rgba8 fgColor;

        if ( _options.coverage() == true )
        {
            // for a coverage value, copy the value exactly.
            fgColor = agg::rgba8( (unsigned)(c.r()*255.0f), (unsigned)(c.g()*255.0f), (unsigned)(c.b()*255.0f), (unsigned)(c.a()*255.0f) );
        }
        else
        {
            unsigned a = (unsigned)(127.0f+(c.a()*255.0f)/2.0f); // scale alpha up
            fgColor = agg::rgba8( (unsigned)(c.r()*255.0f), (unsigned)(c.g()*255.0f), (unsigned)(c.b()*255.0f), a );
        }

        ConstGeometryIterator gi( geometry );
        while( gi.hasMore() )
        {
            c = color;
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
        
        if (_options.coverage() == true )
        {
            agg::renderer<span_coverage32> ren(buffer);
            ras.render(ren, fgColor);
        }
        else
        {
            agg::renderer<agg::span_abgr32> ren(buffer);
            ras.render(ren, fgColor);
        }
        ras.reset();
    }


    virtual std::string getExtension()  const 
    {
        return "png";
    }

private:
    const AGGLiteOptions _options;
    std::string _configPath;
};


/**
 * Plugin entry point for the AGGLite feature rasterizer
 */
class AGGLiteRasterizerTileSourceDriver : public TileSourceDriver
{
    public:
        AGGLiteRasterizerTileSourceDriver() {}

        virtual const char* className()
        {
            return "AGG-Lite feature rasterizer";
        }
        
        virtual bool acceptsExtension(const std::string& extension) const
        {
            return
                osgDB::equalCaseInsensitive( extension, "osgearth_agglite" ) ||
                osgDB::equalCaseInsensitive( extension, "osgearth_rasterize" );
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* options) const
        {
            std::string ext = osgDB::getFileExtension( file_name );
            if ( !acceptsExtension( ext ) )
            {
                return ReadResult::FILE_NOT_HANDLED;
            }

            return new AGGLiteRasterizerTileSource( getTileSourceOptions(options) );
        }
};

REGISTER_OSGPLUGIN(osgearth_agglite, AGGLiteRasterizerTileSourceDriver)
