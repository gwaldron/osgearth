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

#include <osgEarth/TileSource>
#include <osgEarth/ImageToHeightfieldConverter>
#include <osgDB/ReadFile>

using namespace osgEarth;

TileSource::TileSource():
_minLevel(0),
_maxLevel(INT_MAX)
{
}

TileSource::~TileSource()
{
}

const osgEarth::TileGridProfile&
TileSource::getProfile() const
{
    return _profile;
}

#define PROPERTY_MIN_LEVEL "min_level"
#define PROPERTY_MAX_LEVEL "max_level"

void
TileSource::init(const osgDB::ReaderWriter::Options* options)
{
    if ( options->getPluginData( PROPERTY_MIN_LEVEL ) )
        _minLevel = as<int>( (const char*)options->getPluginData( PROPERTY_MIN_LEVEL ), 0 );

    if ( options->getPluginData( PROPERTY_MAX_LEVEL ) )
        _maxLevel = as<int>( (const char*)options->getPluginData( PROPERTY_MAX_LEVEL ), INT_MAX );
}

osg::Image*
TileSource::readImage(const osgEarth::TileKey *key)
{
    osg::Image *image = 0;
    if (_cache.valid())
    {
        //Try to get the image from the cache.
        image = _cache->getImage(key, this);
    }

    if (!image)
    {
        //If we didn't get the image from the cache, go ahead and create it.
        image = createImage(key);

        //Write the image to the cache if we could create one and we have a cache.
        if (image && _cache.valid())
        {
            _cache->setImage(key, this, image);
        }
    }
    return image;
}

osg::HeightField*
TileSource::readHeightField(const osgEarth::TileKey *key)
{
    osg::HeightField* hf = 0;

    if (_cache.valid())
    {
        //Try to get the image from the cache.
        osg::ref_ptr<osg::Image> image = _cache->getImage(key, this);
        if (image.valid())
        {
            hf = ImageToHeightFieldConverter::convert(image.get());
        }
    }

    if (!hf)
    {
        hf = createHeightField(key);

        //Write the image to the cache if we could create one and we have a cache.
        if (hf && _cache.valid())
        {
            _cache->setImage(key, this, ImageToHeightFieldConverter::convert(hf));
        }
    }
    return hf;
}

osg::HeightField* TileSource::createHeightField( const TileKey* key )
{
    osg::ref_ptr<osg::Image> image = createImage(key);
    osg::HeightField* hf = 0;
    if (image.valid())
    {
        hf = ImageToHeightFieldConverter::convert(image.get());
    }      
    return hf;
}