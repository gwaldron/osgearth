/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osgEarthSymbology/GeometrySymbol>
//TODO: replace this with ImageRasterizer
#include <osgEarthSymbology/AGG.h>
#include <osgEarth/Registry>
#include <osgEarth/FileUtils>

#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include "AGGLiteOptions"
//#include "agg.h"

#include <sstream>
#include <OpenThreads/Mutex>
#include <OpenThreads/ScopedLock>

#define LC "[AGGLite] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace osgEarth::Drivers;
using namespace OpenThreads;

/********************************************************************/

class AGGLiteRasterizerTileSource : public FeatureTileSource
{
public:
    AGGLiteRasterizerTileSource( const TileSourceOptions& options ) : FeatureTileSource( options ),
        _options( options )
    {
        //nop
    }

    struct BuildData : public osg::Referenced {
        BuildData() : _pass(0) { }
        int _pass;
    };

    //override
    osg::Referenced* createBuildData()
    {
        return new BuildData();
    }

    //override
    bool preProcess(osg::Image* image, osg::Referenced* buildData)
    {
        agg::rendering_buffer rbuf( image->data(), image->s(), image->t(), image->s()*4 );
        agg::renderer<agg::span_abgr32> ren(rbuf);
        ren.clear(agg::rgba8(0,0,0,0));
        //ren.clear(agg::rgba8(255,255,255,0));
        return true;
    }

    //override
    bool renderFeaturesForStyle(
        const Symbology::Style* style,
        const FeatureList& inFeatures,
        osg::Referenced* buildData,
        const GeoExtent& imageExtent,
        osg::Image* image )
    {
        // local copy of the features that we can process
        FeatureList features = inFeatures;

        BuildData* bd = static_cast<BuildData*>( buildData );

        // A processing context to use with the filters:
        FilterContext context;
        context.profile() = getFeatureSource()->getFeatureProfile();

        const LineSymbol* masterLine = style->getSymbol<LineSymbol>();
        const PolygonSymbol* masterPoly = style->getSymbol<PolygonSymbol>();

        //bool embeddedStyles = getFeatureSource()->hasEmbeddedStyles();

        // if only a line symbol exists, and there are polygons in the mix, draw them
        // as outlines (line rings).
        //OE_INFO << LC << "Line Symbol = " << (masterLine == 0L ? "null" : masterLine->getConfig().toString()) << std::endl;
        //OE_INFO << LC << "Poly SYmbol = " << (masterPoly == 0L ? "null" : masterPoly->getConfig().toString()) << std::endl;

        //bool convertPolysToRings = poly == 0L && line != 0L;
        //if ( convertPolysToRings )
        //    OE_INFO << LC << "No PolygonSymbol; will draw polygons to rings" << std::endl;

        // initialize:
        double xmin = imageExtent.xMin();
        double ymin = imageExtent.yMin();
        //double s = (double)image->s();
        //double t = (double)image->t();
        double xf = (double)image->s() / imageExtent.width();
        double yf = (double)image->t() / imageExtent.height();

        // strictly speaking we should iterate over the features and buffer each one that's a line,
        // rather then checking for the existence of a LineSymbol.
        FeatureList linesToBuffer;
        for(FeatureList::iterator i = features.begin(); i != features.end(); i++)
        {
            Feature* feature = i->get();
            Geometry* geom = feature->getGeometry();

            if ( geom )
            {
                // check for an embedded style:
                const LineSymbol* line = feature->style().isSet() ? 
                    feature->style()->get()->getSymbol<LineSymbol>() : masterLine;
                const PolygonSymbol* poly =
                    feature->style().isSet() ? feature->style()->get()->getSymbol<PolygonSymbol>() : masterPoly;

                // if we have polygons but only a LineSymbol, draw the poly as a line.
                if ( geom->getComponentType() == Geometry::TYPE_POLYGON )
                {
                    if ( !poly && line )
                    {
                        Feature* outline = new Feature( *feature );
                        geom = geom->cloneAs( Geometry::TYPE_RING );
                        outline->setGeometry( geom );
                        *i = outline;
                        feature = outline;
                    }
                    //TODO: fix to enable outlined polys. doesn't work, not sure why -gw
                    //else if ( poly && line )
                    //{
                    //    Feature* outline = new Feature();
                    //    geom = geom->cloneAs( Geometry::TYPE_LINESTRING );
                    //    outline->setGeometry( geom );
                    //    features.push_back( outline );
                    //}
                }

                bool needsBuffering =
                    geom->getComponentType() == Geometry::TYPE_LINESTRING || 
                    geom->getComponentType() == Geometry::TYPE_RING;

                if ( needsBuffering )
                {
                    linesToBuffer.push_back( feature );
                }
            }
        }

        if ( linesToBuffer.size() > 0 )
        {
            //We are buffering in the features native extent, so we need to use the transform extent to get the proper "resolution" for the image
            GeoExtent transformedExtent = imageExtent.transform(context.profile()->getSRS());

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
                context = resample.push( linesToBuffer, context );
            }

            // now run the buffer operation on all lines:
            BufferFilter buffer;
            float lineWidth = 0.5;
            if ( masterLine )
            {
                buffer.capStyle() = masterLine->stroke()->lineCap().value();

                if ( masterLine->stroke()->width().isSet() )
                    lineWidth = masterLine->stroke()->width().value();
            }

            // "relative line size" means that the line width is expressed in (approx) pixels
            // rather than in map units
            if ( _options.relativeLineSize() == true )
                buffer.distance() = xres * lineWidth;
            else
                buffer.distance() = lineWidth;

            buffer.push( linesToBuffer, context );
        }

        // First, transform the features into the map's SRS:
        TransformFilter xform( imageExtent.getSRS() );
        xform.setLocalizeCoordinates( false );
        context = xform.push( features, context );

        // set up the AGG renderer:
        agg::rendering_buffer rbuf( image->data(), image->s(), image->t(), image->s()*4 );

        // Create the renderer and the rasterizer
        agg::renderer<agg::span_abgr32> ren(rbuf);
        agg::rasterizer ras;

        // Setup the rasterizer
        ras.gamma(1.3);
        ras.filling_rule(agg::fill_even_odd);

        GeoExtent cropExtent = GeoExtent(imageExtent);
        cropExtent.scale(1.1, 1.1);

        osg::ref_ptr<Symbology::Polygon> cropPoly = new Symbology::Polygon( 4 );
        cropPoly->push_back( osg::Vec3d( cropExtent.xMin(), cropExtent.yMin(), 0 ));
        cropPoly->push_back( osg::Vec3d( cropExtent.xMax(), cropExtent.yMin(), 0 ));
        cropPoly->push_back( osg::Vec3d( cropExtent.xMax(), cropExtent.yMax(), 0 ));
        cropPoly->push_back( osg::Vec3d( cropExtent.xMin(), cropExtent.yMax(), 0 ));

        double lineWidth = 1.0;
        if ( masterLine )
            lineWidth = (double)masterLine->stroke()->width().value();

        osg::Vec4 color = osg::Vec4(1, 1, 1, 1);
        if ( masterLine )
            color = masterLine->stroke()->color();

        // render the features
        for(FeatureList::iterator i = features.begin(); i != features.end(); i++)
        {
            Feature* feature = i->get();
            //bool first = bd->_pass == 0 && i == features.begin();

            Geometry* geometry = feature->getGeometry();

            osg::ref_ptr< Geometry > croppedGeometry;
            if ( ! geometry->crop( cropPoly.get(), croppedGeometry ) )
                continue;

            // set up a default color:
            osg::Vec4 c = color;
            unsigned int a = 127+(c.a()*255)/2; // scale alpha up
            agg::rgba8 fgColor( c.r()*255, c.g()*255, c.b()*255, a );

            GeometryIterator gi( croppedGeometry.get() );
            while( gi.hasMore() )
            {
                c = color;
                Geometry* g = gi.next();
            
                const LineSymbol* line = feature->style().isSet() ? 
                    feature->style()->get()->getSymbol<LineSymbol>() : masterLine;

                const PolygonSymbol* poly =
                    feature->style().isSet() ? feature->style()->get()->getSymbol<PolygonSymbol>() : masterPoly;

                if (g->getType() == Geometry::TYPE_RING || g->getType() == Geometry::TYPE_LINESTRING)
                {
                    if ( line )
                        c = line->stroke()->color();
                    else if ( poly )
                        c = poly->fill()->color();
                }

                else if ( g->getType() == Geometry::TYPE_POLYGON )
                {
                    if ( poly )
                        c = poly->fill()->color();
                    else if ( line )
                        c = line->stroke()->color();
                }

                a = 127+(c.a()*255)/2; // scale alpha up
                fgColor = agg::rgba8( c.r()*255, c.g()*255, c.b()*255, a );

                ras.filling_rule( agg::fill_even_odd );
                for( Geometry::iterator p = g->begin(); p != g->end(); p++ )
                {
                    const osg::Vec3d& p0 = *p;
                    double x0 = xf*(p0.x()-xmin);
                    double y0 = yf*(p0.y()-ymin);

                    //const osg::Vec3d& p1 = p+1 != g->end()? *(p+1) : g->front();
                    //double x1 = xf*(p1.x()-xmin);
                    //double y1 = yf*(p1.y()-ymin);

                    if ( p == g->begin() )
                        ras.move_to_d( x0, y0 );
                    else
                        ras.line_to_d( x0, y0 );
                }
            }
            ras.render(ren, fgColor);
            ras.reset();
        }

        bd->_pass++;
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

    virtual std::string getExtension()  const 
    {
        return "png";
    }

private:
    const AGGLiteOptions _options;
    std::string _configPath;
};

// Reads tiles from a TileCache disk cache.
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
            return osgDB::equalCaseInsensitive( extension, "osgearth_agglite" );
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
