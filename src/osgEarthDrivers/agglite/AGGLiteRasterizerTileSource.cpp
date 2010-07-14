/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osgEarthFeatures/TransformFilter>
#include <osgEarthFeatures/BufferFilter>
#include <osgEarthSymbology/Style>
#include <osgEarthSymbology/GeometrySymbol>
#include <osgEarth/Registry>
#include <osgEarth/FileUtils>

#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include "AGGLiteOptions"
#include "agg.h"

#include <sstream>
#include <OpenThreads/Mutex>
#include <OpenThreads/ScopedLock>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace osgEarth::Drivers;
using namespace OpenThreads;

/********************************************************************/

static int firstCall = 0;
class AGGLiteRasterizerTileSource : public FeatureTileSource
{
public:
    AGGLiteRasterizerTileSource( const PluginOptions* options ) : FeatureTileSource( options )
    {
        _settings = dynamic_cast<const AGGLiteOptions*>( options );
        if ( !_settings.valid() )
            _settings = new AGGLiteOptions( options );
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
        FeatureList& features,
        osg::Referenced* buildData,
        const GeoExtent& imageExtent,
        osg::Image* image )
    {

#define NEW
#ifdef NEW        
#if 0
        // DEBUG INFO
        OE_NOTICE << "enter feature" << std::endl;
        for(FeatureList::iterator i = features.begin(); i != features.end(); i++)
        {
            GeometryIterator gi( i->get()->getGeometry() );
            while( gi.hasMore() )
            {
                Geometry* g = gi.next();
                switch(g->getType()) {
                case Geometry::TYPE_POLYGON:
                    OE_NOTICE << "type POLYGON" << std::endl;
                    break;
                case Geometry::TYPE_RING:
                    OE_NOTICE << "type RING" << std::endl;
                    break;
                case Geometry::TYPE_LINESTRING:
                    OE_NOTICE << "type LINESTRING" << std::endl;
                    break;
                }
            }
        }
#endif

        BuildData* bd = static_cast<BuildData*>( buildData );

        // A processing context to use with the filters:
        FilterContext context;
        context.profile() = getFeatureSource()->getFeatureProfile();
        const osgEarth::Symbology::LineSymbol* line = style->getSymbol<osgEarth::Symbology::LineSymbol>();

        // initialize:
        double xmin = imageExtent.xMin();
        double ymin = imageExtent.yMin();
        double s = (double)image->s();
        double t = (double)image->t();
        double xf = (double)image->s() / imageExtent.width();
        double yf = (double)image->t() / imageExtent.height();

        if (line) {
            BufferFilter buffer;
            buffer.capStyle() = line->stroke()->lineCap().value();
            if (_settings->relativeLineSize().value()) {
                double ratio = 1.0/xf;
                buffer.distance() = ratio * line->stroke()->width().value();
            } else {
                buffer.distance() = line->stroke()->width().value();
            }
            context = buffer.push( features, context );
        }

        // First, transform the features into the map's SRS:
        TransformFilter xform( imageExtent.getSRS() );
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

        OE_INFO << "agglite extent min " << cropExtent.xMin() <<  " " << cropExtent.yMin() << " max " << cropExtent.xMax() << " " << cropExtent.yMax() << std::endl;
        osg::ref_ptr<Symbology::Polygon> cropPoly = new Symbology::Polygon( 4 );
        cropPoly->push_back( osg::Vec3d( cropExtent.xMin(), cropExtent.yMin(), 0 ));
        cropPoly->push_back( osg::Vec3d( cropExtent.xMax(), cropExtent.yMin(), 0 ));
        cropPoly->push_back( osg::Vec3d( cropExtent.xMax(), cropExtent.yMax(), 0 ));
        cropPoly->push_back( osg::Vec3d( cropExtent.xMin(), cropExtent.yMax(), 0 ));

        double lineWidth = 1.0;
        if (line)
            lineWidth = (double)line->stroke()->width().value();

        //OE_NOTICE << "rendering " << features.size() << " features" << std::endl;

        osg::Vec4 color = osg::Vec4(1, 1, 1, 1);
        if (line) {
            color = line->stroke()->color();
        }
        // render the features
        for(FeatureList::iterator i = features.begin(); i != features.end(); i++)
        {
            bool first = bd->_pass == 0 && i == features.begin();

            osg::ref_ptr< Geometry > croppedGeometry = Feature::cropGeometry(cropPoly.get(), i->get()->getGeometry());
            //osg::ref_ptr< Geometry > croppedGeometry = i->get()->getGeometry();
            if (!croppedGeometry.valid()) continue;

            osg::Vec4 c = color;
            unsigned int a = 127+(c.a()*255)/2; // scale alpha up
            agg::rgba8 fgColor( c.r()*255, c.g()*255, c.b()*255, a );

            GeometryIterator gi( croppedGeometry.get() );
            while( gi.hasMore() )
            {
                c = color;
                Geometry* g = gi.next();
                if (g->getType() == Geometry::TYPE_POLYGON) {
                    const PolygonSymbol* symbol = style->getSymbol<PolygonSymbol>();
                    if (symbol)
                        c = symbol->fill()->color();
                } else if (g->getType() == Geometry::TYPE_RING || g->getType() == Geometry::TYPE_LINESTRING) {
                    const LineSymbol* symbol = style->getSymbol<LineSymbol>();
                    if (symbol)
                        c = symbol->stroke()->color();
                }

#if 0
                // For Debug
                switch(g->getType()) {
                case Geometry::TYPE_POLYGON:
                    OE_NOTICE << "converted type POLYGON" << std::endl;
                    break;
                case Geometry::TYPE_RING:
                    OE_NOTICE << "converted type RING" << std::endl;
                    break;
                case Geometry::TYPE_LINESTRING:
                    OE_NOTICE << "converted type LINESTRING" << std::endl;
                    break;
                }
#endif

                a = 127+(c.a()*255)/2; // scale alpha up
                fgColor = agg::rgba8( c.r()*255, c.g()*255, c.b()*255, a );

                ras.filling_rule( agg::fill_even_odd );
                for( Geometry::iterator p = g->begin(); p != g->end(); p++ )
                {
                    const osg::Vec3d& p0 = *p;
                    double x0 = xf*(p0.x()-xmin);
                    double y0 = yf*(p0.y()-ymin);

                    const osg::Vec3d& p1 = p+1 != g->end()? *(p+1) : g->front();
                    double x1 = xf*(p1.x()-xmin);
                    double y1 = yf*(p1.y()-ymin);

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
#else
        BuildData* bd = static_cast<BuildData*>( buildData );
        
        // First, transform the features into the map's SRS:
        FilterContext context;
        context.profile() = getFeatureSource()->getFeatureProfile();
        TransformFilter xform( imageExtent.getSRS() );
        context = xform.push( features, context );

        // set up the AGG renderer:
        agg::rendering_buffer rbuf( image->data(), image->s(), image->t(), image->s()*4 );

		// Create the renderer and the rasterizer
		agg::renderer<agg::span_abgr32> ren(rbuf);
		agg::rasterizer ras;

		// Setup the rasterizer
		ras.gamma(1.3);
        //ras.gamma(2.2);
		//ras.filling_rule(agg::fill_non_zero);
        ras.filling_rule(agg::fill_even_odd);

        // initialize:

        double xmin = imageExtent.xMin();
        double ymin = imageExtent.yMin();
        double s = (double)image->s();
        double t = (double)image->t();
        double xf = (double)image->s() / imageExtent.width();
        double yf = (double)image->t() / imageExtent.height();

        GeoExtent cropExtent = GeoExtent(imageExtent);
        cropExtent.scale(1.1, 1.1);

        osg::ref_ptr<Symbology::Polygon> cropPoly = new Symbology::Polygon( 4 );
        cropPoly->push_back( osg::Vec3d( cropExtent.xMin(), cropExtent.yMin(), 0 ));
        cropPoly->push_back( osg::Vec3d( cropExtent.xMax(), cropExtent.yMin(), 0 ));
        cropPoly->push_back( osg::Vec3d( cropExtent.xMax(), cropExtent.yMax(), 0 ));
        cropPoly->push_back( osg::Vec3d( cropExtent.xMin(), cropExtent.yMax(), 0 ));

        const LineSymbol* line = style->getSymbol<LineSymbol>();
        double lineWidth = 1.0;
        if (line)
            lineWidth = (double)line->stroke()->width().value();

        //OE_NOTICE << "rendering " << features.size() << " features" << std::endl;

        // render the features
        for(FeatureList::iterator i = features.begin(); i != features.end(); i++)
        {
            bool first = bd->_pass == 0 && i == features.begin();

            osg::ref_ptr< Geometry > croppedGeometry = Feature::cropGeometry(cropPoly.get(), i->get()->getGeometry());
            if (!croppedGeometry.valid()) continue;

            GeometryIterator gi( croppedGeometry.get() );
            while( gi.hasMore() )
            {
                Geometry* g = gi.next();
                osg::Vec4f c = osg::Vec4(1, 1, 1, 1);
                if (g->getType() == Geometry::TYPE_POLYGON) {
                    const PolygonSymbol* symbol = style->getSymbol<PolygonSymbol>();
                    if (symbol)
                        c = symbol->fill()->color();
                } else if (g->getType() == Geometry::TYPE_RING || g->getType() == Geometry::TYPE_LINESTRING) {
                    const LineSymbol* symbol = style->getSymbol<LineSymbol>();
                    if (symbol)
                        c = symbol->stroke()->color();
                }

                unsigned int a = 127+(c.a()*255)/2; // scale alpha up
                agg::rgba8 fgColor( c.r()*255, c.g()*255, c.b()*255, a );

                if ( g->getType() == Geometry::TYPE_LINESTRING )
                {
                    ras.filling_rule( agg::fill_non_zero );

                    double x1 = 0, y1 = 0, x2 = 0, y2 = 0;
                    double dx = 0, dy = 0;

                    for( Geometry::iterator p = g->begin(); p != g->end()-1; p++ )
                    {
                        const osg::Vec3d& p1 = *p;
                        x1 = xf*(p1.x()-xmin);
                        y1 = yf*(p1.y()-ymin);

                        const osg::Vec3d& p2 = *(p+1);
                        x2 = xf*(p2.x()-xmin);
                        y2 = yf*(p2.y()-ymin);
                        
                        dx = x2 - x1;
                        dy = y2 - y1;
                        double d = sqrt(dx*dx + dy*dy);
                        dx = lineWidth * (y2 - y1) / d;
                        dy = lineWidth * (x2 - x1) / d;

                        if ( p == g->begin() )
                            ras.move_to_d(x1-dx, y1+dy);
                        
                        ras.line_to_d(x2-dx, y2+dy);
                    }

                    // back the other way:
                    for( Geometry::reverse_iterator p = g->rbegin(); p != g->rend()-1; p++ )
                    {
                        const osg::Vec3d& p1 = *p;
                        x1 = xf*(p1.x()-xmin);
                        y1 = yf*(p1.y()-ymin);

                        const osg::Vec3d& p2 = *(p+1);
                        x2 = xf*(p2.x()-xmin);
                        y2 = yf*(p2.y()-ymin);
                        
                        dx = x2 - x1;
                        dy = y2 - y1;
                        double d = sqrt(dx*dx + dy*dy);
                        dx = lineWidth * (y2 - y1) / d;
                        dy = lineWidth * (x2 - x1) / d;

                        if( p == g->rbegin() )
                            ras.line_to_d(x1-dx, y1+dy);

                        ras.line_to_d(x2-dx, y2+dy);
                    }

                    ras.render(ren, fgColor);
                }
                else // polygon
                {
                    for( Geometry::iterator p = g->begin(); p != g->end(); p++ )
                    {
                        const osg::Vec3d& p0 = *p;
                        double x0 = xf*(p0.x()-xmin);
                        double y0 = yf*(p0.y()-ymin);

                        const osg::Vec3d& p1 = p+1 != g->end()? *(p+1) : g->front();
                        double x1 = xf*(p1.x()-xmin);
                        double y1 = yf*(p1.y()-ymin);

                        if ( p == g->begin() )
                            ras.move_to_d( x0, y0 );
                        else
                            ras.line_to_d( x0, y0 );
                    }

                    ras.render(ren, fgColor);
                }

                ras.reset();
            }            
        }

        bd->_pass++;


#endif
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
    osg::ref_ptr<const AGGLiteOptions> _settings;
    std::string _configPath;
};

// Reads tiles from a TileCache disk cache.
class AGGLiteRasterizerTileSourceFactory : public osgDB::ReaderWriter
{
    public:
        AGGLiteRasterizerTileSourceFactory() {}

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

            return new AGGLiteRasterizerTileSource( static_cast<const PluginOptions*>(options) );
        }
};

REGISTER_OSGPLUGIN(osgearth_agglite, AGGLiteRasterizerTileSourceFactory)
