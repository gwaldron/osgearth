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

#include <osgEarth/MapboxGLGlyphManager>
#include <osgEarth/StringUtils>
#include <osgDB/Registry>
#include <osgText/String>

#include "glyphs.pb.h"

using namespace osgEarth;
using namespace osgEarth::Util;

MapboxGLGlyphManager::MapboxGLGlyphManager(const std::string& uri, const std::string& key, const osgDB::Options* options):
_uri(uri),
_key(key),
_options(options)
{
}

const std::string& MapboxGLGlyphManager::getURI() const
{
    return _uri;
}

void MapboxGLGlyphManager::setURI(const std::string & uri)
{
    _uri = uri;
}

const std::string& MapboxGLGlyphManager::getKey() const
{
    return _key;
}

void MapboxGLGlyphManager::setKey(const std::string& key)
{
    _key = key;
}

void MapboxGLGlyphManager::getGlyphs(const std::string& text, const std::string& fontStack, std::vector< osg::ref_ptr< Glyph > >& result)
{
    std::lock_guard< std::mutex> lk(_mutex);
    osgText::String osgText(text, osgText::String::ENCODING_UTF8);

    for (unsigned int i = 0; i < osgText.size(); ++i)
    {     
        result.push_back(getGlyph(fontStack, (unsigned int)osgText[i]));
    }
}

MapboxGLGlyphManager::Glyph* MapboxGLGlyphManager::getGlyph(const std::string& font, unsigned int code)
{
    {
        auto stackItr = _fontsToGlyphs.find(font);
        if (stackItr != _fontsToGlyphs.end())
        {
            auto glyphItr = stackItr->second.find(code);
            if (glyphItr != stackItr->second.end())
            {
                return glyphItr->second;
            }
        }
    }

    std::string url = _uri;
    int range = std::floor(code / 256) * 256;
    std::stringstream buf;
    buf << range << "-" << range + 255;
    osgEarth::replaceIn(url, "{fontstack}", font);
    osgEarth::replaceIn(url, "{key}", _key);
    osgEarth::replaceIn(url, "{range}", buf.str());
    osgEarth::replaceIn(url, " ", "%20");    
    loadFont(url);

    // Try again.
    {
        auto stackItr = _fontsToGlyphs.find(font);
        if (stackItr != _fontsToGlyphs.end())
        {
            auto glyphItr = stackItr->second.find(code);
            if (glyphItr != stackItr->second.end())
            {
                return glyphItr->second;
            }
        }
    }

    return nullptr;
}

void MapboxGLGlyphManager::loadFont(const osgEarth::URI& glyphsURI)
{
    auto itr = _loadedFonts.find(glyphsURI.full());
    if (itr != _loadedFonts.end())
    {
        return;
    }

    //std::cout << "Loading " << glyphsURI.full() << std::endl;

    unsigned int numLoaded = 0;

    mapboxgl::glyphs::glyphs font;
    std::string original = glyphsURI.getString(_options);

    // Get the compressor
    osg::ref_ptr< osgDB::BaseCompressor> compressor = osgDB::Registry::instance()->getObjectWrapperManager()->findCompressor("zlib");

    std::istringstream in(original);

    std::string value;
    if (!compressor->decompress(in, value))
    {
        value = original;
    }

    if (font.ParseFromString(value))
    {
        for (auto& stack : font.stacks())
        {
            for (auto& g : stack.glyphs())
            {
                ++numLoaded;
                Glyph* glyph = new Glyph;
                glyph->id = g.id();
                glyph->width = g.width();
                glyph->height = g.height();
                glyph->left = g.left();
                glyph->top = g.top();
                glyph->bitmap = g.bitmap();
                glyph->advance = g.advance();
                _fontsToGlyphs[stack.name()][g.id()] = glyph;
            }
        }
    }

    //std::cout << "Loaded " << numLoaded << " glyphs from " << glyphsURI.full() << std::endl;
    _loadedFonts.insert(glyphsURI.full());
}
