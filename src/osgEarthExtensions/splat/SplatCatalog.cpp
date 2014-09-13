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
using namespace osgEarth::Extensions::Splat;

#define LC "[SplatCatalog] "

//............................................................................

SplatClass::SplatClass()
{
    //nop
}

SplatClass::SplatClass(const Config& conf)
{
    _name = conf.key();
    conf.getIfSet("url", _imageURI);
}

//............................................................................

SplatCatalog::SplatCatalog()
{
    //nop
}

void
SplatCatalog::fromConfig(const Config& conf)
{
    conf.getIfSet("name",        _name);
    conf.getIfSet("description", _description);

    Config classesObj = conf.child("classes");
    if ( !classesObj.empty() )
    {
        ConfigSet classes = classesObj.children();
        for(ConfigSet::const_iterator i = classes.begin(); i != classes.end(); ++i)
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

    conf.addIfSet("name", _name);
    conf.addIfSet("description", _description);
    
    Config classes("classes");
    {
        for(SplatClasses::const_iterator i = _classes.begin(); i != _classes.end(); ++i)
        {
            Config classConf( i->_name );
            classConf.addIfSet( "url", i->_imageURI );
            classes.add( classConf );
        }
    }    
    conf.add( classes );

    return conf;
}

bool
SplatCatalog::createTextureAndIndex(const osgDB::Options* options,
                                    osg::ref_ptr<osg::Texture2DArray>& out_texture,
                                    SplatArrayIndex&                   out_index) const
{
    out_texture = 0L;
    out_index.clear();

    // load the images and build the index along the way.
    std::vector< osg::ref_ptr<osg::Image> > images;

    for(unsigned i=0; i<_classes.size(); ++i)
    {
        ReadResult result = _classes[i]._imageURI->readImage(options);
        if ( result.succeeded() )
        {
            osg::ref_ptr<osg::Image> image = result.getImage();

            // ensure all images are compatible.
            // todo: perhaps in the "future" we can forcably make them the same
            if ( i == 0 || ImageUtils::textureArrayCompatible(image.get(), images.back().get()) )
            {
                images.push_back( image );
                out_index[_classes[i]._name] = i;
            }
        }
    }

    // create the texture array:
    if ( images.size() == 0 )
        return false;

    out_texture = new osg::Texture2DArray();
    out_texture->setTextureSize( images[0]->s(), images[0]->t(), images.size() );
    out_texture->setWrap( osg::Texture::WRAP_S, osg::Texture::REPEAT );
    out_texture->setWrap( osg::Texture::WRAP_T, osg::Texture::REPEAT );
    out_texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
    out_texture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
    out_texture->setResizeNonPowerOfTwoHint( false );

    for(unsigned i=0; i<images.size(); ++i)
    {
        out_texture->setImage( i, images[i].get() );
    }

    return true;
}
