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

#include <osg/Version>
#include <osgEarth/TileSource>
#include <osgEarth/Registry>
#include <osgEarth/ImageUtils>
#include <osgEarth/StringUtils>
#include <osgEarthSymbology/Geometry>
#include <osgEarthSymbology/GeometryRasterizer>
#include <osgDB/FileNameUtils>
#if !((OPENSCENEGRAPH_MAJOR_VERSION <= 2) && (OPENSCENEGRAPH_MINOR_VERSION <= 8))
# include <osgText/Glyph>
#endif
#include <osgText/Font>
#include <osg/Notify>
#include <sstream>

#include "DebugOptions"

using namespace osgEarth;
using namespace osgEarth::Drivers;
using namespace osgEarth::Symbology;

namespace
{
    static osg::Vec4 colors[4] = {
        osg::Vec4(1,0,0,1),
        osg::Vec4(0,1,0,1),
        osg::Vec4(0,0,1,1),
        osg::Vec4(1,0,1,1)
    };

    void copySubImageAndColorize( const osg::Image* src, osg::Image* dst, unsigned dx, unsigned dy, osg::Vec4& newColor)
    {
        ImageUtils::PixelReader read(src);
        ImageUtils::PixelWriter write(dst);

        for( int src_t=0, dst_t=dy; src_t < src->t(); src_t++, dst_t++ )
        {
            for( int src_s=0, dst_s=dx; src_s < src->s(); src_s++, dst_s++ )
            {           
                osg::Vec4 color = read(src_s, src_t);
                if ( color.a() > 0.5f )
                    color = newColor;
                write( color, dst_s, dst_t );
            }
        }
    }
}

class DebugTileSource : public TileSource
{
public:
    DebugTileSource( const DebugOptions& options ) : TileSource( options ), _options(options)
    {        
        _geom = new Ring();
        _geom->push_back( osg::Vec3(5, 5, 0) );
        _geom->push_back( osg::Vec3(250, 5, 0) );
        _geom->push_back( osg::Vec3(250, 250, 0) );
        _geom->push_back( osg::Vec3(5, 250, 0) );
        _font = osgText::readFontFile( "arial.ttf" );

        _color = osgEarth::htmlColorToVec4f( *_options.colorCode() );
    }

    // Yahoo! uses spherical mercator, but the top LOD is a 2x2 tile set.
    void initialize( const std::string& referenceURI, const Profile* overrideProfile)
    {
        if ( overrideProfile )
            setProfile( overrideProfile );
        else
            setProfile( Profile::create("global-geodetic") );
    }

    osg::Image* createImage( const TileKey& key, ProgressCallback* progress )
    {
        // first draw the colored outline:
        GeometryRasterizer rasterizer( 256, 256 );
        rasterizer.draw( _geom.get(), colors[key.getLevelOfDetail() % 4] );
        osg::Image* image = rasterizer.finalize();

        // next render the tile key text:
        std::stringstream buf;
        buf << key.str();
        std::string text = buf.str();

        unsigned x = 10, y = 10;

        osgText::FontResolution resolution(32, 32);
        for( unsigned i=0; i<text.length(); ++i )
        {
#if !((OPENSCENEGRAPH_MAJOR_VERSION <= 2) && (OPENSCENEGRAPH_MINOR_VERSION <= 8))
            osgText::Glyph* glyph = _font->getGlyph( resolution, text.at(i) );
            copySubImageAndColorize( glyph, image, x, y, _color );
            x += glyph->s() + 1;
#endif
        }

        return image;
    }

    virtual std::string getExtension() const 
    {
        return "png";
    }

    virtual bool supportsPersistentCaching() const
    {
        return false;
    }

private:
    const DebugOptions _options;
    osg::ref_ptr<Geometry> _geom;
    osg::ref_ptr<osgText::Font> _font;
    osg::Vec4 _color;
};


class DebugTileSourceDriver : public TileSourceDriver
{
    public:
        DebugTileSourceDriver()
        {
            supportsExtension( "osgearth_debug", "Debugging driver" );
        }

        virtual const char* className()
        {
            return "Debugging Driver";
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* options) const
        {
            if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
                return ReadResult::FILE_NOT_HANDLED;

            return new DebugTileSource( getTileSourceOptions(options) );
        }
};

REGISTER_OSGPLUGIN(osgearth_debug, DebugTileSourceDriver)
