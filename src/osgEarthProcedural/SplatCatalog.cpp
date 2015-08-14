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
#include "SplatCatalog"
#include <osgEarth/Config>
#include <osgEarth/ImageUtils>
#include <osg/Texture2DArray>

using namespace osgEarth;
using namespace osgEarth::Procedural;

#define LC "[SplatCatalog] "

#define SPLAT_CATALOG_CURRENT_VERSION 1


//............................................................................

SplatDetailData::SplatDetailData() :
_textureIndex( -1 )
{
    //nop
}

SplatDetailData::SplatDetailData(const Config& conf) :
_textureIndex( -1 )
{
    conf.getIfSet("image",      _imageURI);
    conf.getIfSet("saturation", _saturation);
    conf.getIfSet("threshold",  _threshold);
    conf.getIfSet("slope",      _slope);
}

Config
SplatDetailData::getConfig() const
{
    Config conf;
    conf.addIfSet("image",      _imageURI);
    conf.addIfSet("saturation", _saturation);
    conf.addIfSet("threshold",  _threshold);
    conf.addIfSet("slope",      _slope);
    return conf;
}

//............................................................................

SplatRangeData::SplatRangeData() :
_textureIndex( -1 )
{
    //nop
}

SplatRangeData::SplatRangeData(const Config& conf) :
_textureIndex( -1 )
{
    conf.getIfSet("image",      _imageURI);
    conf.getIfSet("model",      _modelURI);
    conf.getIfSet("modelCount", _modelCount);
    conf.getIfSet("modelLevel", _modelLevel);

    if ( conf.hasChild("detail") )
        _detail = SplatDetailData(conf.child("detail"));
}

Config
SplatRangeData::getConfig() const
{
    Config conf;
    conf.addIfSet("image",      _imageURI);
    conf.addIfSet("model",      _modelURI);
    conf.addIfSet("modelCount", _modelCount);
    conf.addIfSet("modelLevel", _modelLevel);
    if ( _detail.isSet() )
        conf.add( "detail", _detail->getConfig() );

    return conf;
}

//............................................................................

SplatClass::SplatClass()
{
    //nop
}

SplatClass::SplatClass(const Config& conf)
{
    _name = conf.key();

    // read the data definitions in order:
    for(ConfigSet::const_iterator i = conf.children().begin(); i != conf.children().end(); ++i)
    {
        if ( !i->empty() )
        {
            _ranges.push_back(SplatRangeData(*i));
        }
    }
}

Config
SplatClass::getConfig() const
{
    Config conf( _name );
    for(SplatRangeDataVector::const_iterator i = _ranges.begin(); i != _ranges.end(); ++i)
    {
        conf.add( i->getConfig() );
    }
    return conf;
}

//............................................................................

SplatCatalog::SplatCatalog()
{
    _version = SPLAT_CATALOG_CURRENT_VERSION;
}

void
SplatCatalog::fromConfig(const Config& conf)
{
    conf.getIfSet("version",     _version);
    conf.getIfSet("name",        _name);
    conf.getIfSet("description", _description);

    Config classesConf = conf.child("classes");
    if ( !classesConf.empty() )
    {
        for(ConfigSet::const_iterator i = classesConf.children().begin(); i != classesConf.children().end(); ++i)
        {
            if ( !i->key().empty() )
            {
                _classes[i->key()] = SplatClass(*i);
            }
        }
    }
}

Config
SplatCatalog::getConfig() const
{
    Config conf;
    conf.addIfSet("version",     _version);
    conf.addIfSet("name",        _name);
    conf.addIfSet("description", _description);
    
    Config classes("classes");
    {
        for(SplatClassMap::const_iterator i = _classes.begin(); i != _classes.end(); ++i)
        {
            classes.add( i->second.getConfig() );
        }
    }    
    conf.add( classes );

    return conf;
}

namespace
{
    osg::Image* loadImage(const URI& uri, const osgDB::Options* dbOptions, osg::Image* firstImage)
    {
        // try to load the image:
        ReadResult result = uri.readImage(dbOptions);
        if ( result.succeeded() )
        {
            // if this is the first image loaded, remember it so we can ensure that
            // all images are copatible.
            if ( firstImage == 0L )
            {
                firstImage = result.getImage();
            }
            else
            {
                // ensure compatibility, a requirement for texture arrays.
                // In the future perhaps we can resize/convert instead.
                if ( !ImageUtils::textureArrayCompatible(result.getImage(), firstImage) )
                {
                    OE_WARN << LC << "Image " << uri.base()
                        << " was found, but cannot be used because it is not compatible with "
                        << "other splat images (same dimensions, pixel format, etc.)\n";

                    return 0L;
                }
            }
        }
        else
        {
            OE_WARN << LC
                << "Image in the splat catalog failed to load: "
                << uri.full() << "; message = " << result.getResultCodeString()
                << std::endl;
        }

        return result.releaseImage();
    }
}

bool
SplatCatalog::createSplatTextureDef(const osgDB::Options* dbOptions,
                                    SplatTextureDef&      out      )
{
    // Reset all texture indices to default
    for(SplatClassMap::iterator i = _classes.begin(); i != _classes.end(); ++i)
    {
        SplatClass& c = i->second;
        for(SplatRangeDataVector::iterator range = c._ranges.begin(); range != c._ranges.end(); ++range)
        {
            range->_textureIndex = -1;
            if ( range->_detail.isSet() )
            {
                range->_detail->_textureIndex = -1;
            }
        }
    }

    typedef osgEarth::fast_map<URI, int> ImageIndexTable; // track images to prevent dupes
    ImageIndexTable imageIndices;
    std::vector< osg::ref_ptr<osg::Image> > imagesInOrder;
    int index = 0;
    osg::Image* firstImage  = 0L;

    // Load all referenced images in the catalog, and assign each a unique index.
    for(SplatClassMap::iterator i = _classes.begin(); i != _classes.end(); ++i)
    {
        SplatClass& c = i->second;

        for(SplatRangeDataVector::iterator range = c._ranges.begin(); range != c._ranges.end(); ++range)
        {
            // Load the main image and assign it an index:
            if (range->_imageURI.isSet())
            {
                int texIndex = -1;
                ImageIndexTable::iterator k = imageIndices.find(range->_imageURI.get());
                if ( k == imageIndices.end() )
                {
                    osg::ref_ptr<osg::Image> image = loadImage( range->_imageURI.get(), dbOptions, firstImage );
                    if ( image.valid() )
                    {
                        if ( !firstImage )
                            firstImage = image.get();

                        imageIndices[range->_imageURI.get()] = texIndex = index++;
                        imagesInOrder.push_back( image.get() );
                    }
                }
                else
                {
                    texIndex = k->second;
                }
                range->_textureIndex = texIndex;
            }

            // Load the detail texture if it exists:
            if (range->_detail.isSet() &&
                range->_detail->_imageURI.isSet())
            {
                int texIndex = -1;
                ImageIndexTable::iterator k = imageIndices.find(range->_detail->_imageURI.get());
                if ( k == imageIndices.end() )
                {
                    osg::ref_ptr<osg::Image> image = loadImage( range->_detail->_imageURI.get(), dbOptions, firstImage );
                    if ( image.valid() )
                    {
                        if ( !firstImage )
                            firstImage = image.get();
            
                        imageIndices[range->_detail->_imageURI.get()] = texIndex = index++;
                        imagesInOrder.push_back( image.get() );
                    }
                }
                else
                {
                    texIndex = k->second;
                }
                range->_detail->_textureIndex = texIndex;
            }
        }
    }

    // Next, go through the classes and build the splat lookup table.
    for(SplatClassMap::const_iterator i = _classes.begin(); i != _classes.end(); ++i)
    {
        const SplatClass& c = i->second;

        // selectors for this class (ordered):
        SplatSelectorVector selectors;

        // check each data element:
        for(SplatRangeDataVector::const_iterator range = c._ranges.begin(); range != c._ranges.end(); ++range)
        {
            // If the primary image exists, look up its index and add it to the selector set.
            ImageIndexTable::const_iterator k = imageIndices.find( range->_imageURI.get() );
            if ( k != imageIndices.end() )
            {
                std::string expression;
                if ( range->_minRange.isSet() )
                {
                    expression = Stringify()
                        << "env.range >= float(" << range->_minRange.get() << ")";
                }

                // insert into the lookup table.
                out._splatLUT[c._name].push_back( SplatSelector(expression, *range) );
            }
        }
    }

    // Create the texture array.
    if ( imagesInOrder.size() > 0 )
    {
        out._texture = new osg::Texture2DArray();
        out._texture->setTextureSize( firstImage->s(), firstImage->t(), imagesInOrder.size() );
        out._texture->setWrap( osg::Texture::WRAP_S, osg::Texture::REPEAT );
        out._texture->setWrap( osg::Texture::WRAP_T, osg::Texture::REPEAT );
        out._texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
        out._texture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
        out._texture->setResizeNonPowerOfTwoHint( false );
        out._texture->setMaxAnisotropy( 4.0f );

        for(unsigned i=0; i<imagesInOrder.size(); ++i)
        {
            out._texture->setImage( i, imagesInOrder[i].get() );
        }

    }

    return out._texture.valid();
}
