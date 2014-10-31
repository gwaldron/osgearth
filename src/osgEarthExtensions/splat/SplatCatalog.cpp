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
using namespace osgEarth::Splat;

#define LC "[SplatCatalog] "

#define SPLAT_CATALOG_CURRENT_VERSION 1

//............................................................................

SplatData::SplatData()
{
    //nop
}

SplatData::SplatData(const Config& conf)
{
    conf.getIfSet("expression", _expression);
    conf.getIfSet("image",      _imageURI);
    conf.getIfSet("model",      _modelURI);
    conf.getIfSet("modelCount", _modelCount);
    conf.getIfSet("modelLevel", _modelLevel);
}

Config
SplatData::getConfig() const
{
    Config conf;
    conf.addIfSet("expression", _expression);
    conf.addIfSet("image",      _imageURI);
    conf.addIfSet("model",      _modelURI);
    conf.addIfSet("modelCount", _modelCount);
    conf.addIfSet("modelLevel", _modelLevel);
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
            _data.push_back(SplatData(*i));
        }
    }
}

Config
SplatClass::getConfig() const
{
    Config conf( _name );
    for(SplatDataVector::const_iterator i = _data.begin(); i != _data.end(); ++i)
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
                _classes.push_back( SplatClass(*i) );
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
        for(SplatClassVector::const_iterator i = _classes.begin(); i != _classes.end(); ++i)
        {
            classes.add( i->getConfig() );
        }
    }    
    conf.add( classes );

    return conf;
}

bool
SplatCatalog::createSplatTextureDef(const osgDB::Options* options,
                                    SplatTextureDef&      out      ) const
{
    // Load all the splatting images, preventing duplicates. If a splatting
    // texture fails to load, it will not appear in the array and it will be
    // ignored in the selection code.
    typedef osgEarth::fast_map<URI, unsigned> ImageIndexTable;
    ImageIndexTable imageIndexByURI;
    std::vector< osg::ref_ptr<osg::Image> > imagesInOrder;

    unsigned    index       = 0;
    osg::Image* firstImage  = 0L;

    for(SplatClassVector::const_iterator i = _classes.begin(); i != _classes.end(); ++i)
    {
        const SplatClass& c = *i;
        for(SplatDataVector::const_iterator j = c._data.begin(); j != c._data.end(); ++j)
        {
            // if the URI is set and it's not already in the table:
            if (j->_imageURI.isSet() && imageIndexByURI.find(j->_imageURI.get()) == imageIndexByURI.end())
            {
                // try to load the image:
                ReadResult result = j->_imageURI->readImage(options);
                if ( result.succeeded() )
                {
                    bool okToAdd = true;

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
                            okToAdd = false;

                            OE_WARN << LC << "Image " << j->_imageURI->base()
                                << " was found, but cannot be used because it is not compatible with "
                                << "other splat images (same dimensions, pixel format, etc.)\n";
                        }
                    }

                    if ( okToAdd )
                    {
                        // Assign the image the next available array index and put it in the table.
                        imageIndexByURI[j->_imageURI.get()] = index++;
                        imagesInOrder.push_back(result.getImage());
                    }
                }
            }
        }
    }

    // Next, go through the classes and build the splat lookup table.
    for(SplatClassVector::const_iterator i = _classes.begin(); i != _classes.end(); ++i)
    {
        const SplatClass& c = *i;

        // selectors for this class (ordered):
        SplatIndexSelectorSet selectors;

        // check each data element:
        for(SplatDataVector::const_iterator j = c._data.begin(); j != c._data.end(); ++j)
        {
            const SplatData& d = *j;

            // If the image exists, look up its index and add it to the selector set.
            ImageIndexTable::const_iterator k = imageIndexByURI.find( d._imageURI.get() );
            if ( k != imageIndexByURI.end() )
            {
                OE_DEBUG << "Class " << c._name << ", index " << k->second << " : expression = " << d._expression.get() << "\n";
                selectors.push_back( SplatIndexSelector(d._expression.get(), k->second) );
            }
        }

        // put it in the lookup table for this class:
        if ( selectors.size() > 0 )
        {
            out._lut[c._name] = selectors;
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
