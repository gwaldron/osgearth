/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
#include <osgEarthUtil/AtlasBuilder>
#include <osgEarthSymbology/Skins>
#include <osgEarth/ImageUtils>
#include <osgDB/Options>
#include <osgUtil/Optimizer>
#include <vector>
#include <string>

#define LC "[AtlasBuilder] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Symbology;


namespace
{
    /** Subclass so we can expose the internal lists. */
    struct TextureAtlasBuilderEx : public osgUtil::Optimizer::TextureAtlasBuilder
    {
        typedef AtlasList AtlasListEx;

        const AtlasListEx& getAtlasList()
        {
            return _atlasList;
        }
    };
}


AtlasBuilder::AtlasBuilder(const osgDB::Options* options) :
_options( options ),
_width  ( 1024 ),
_height ( 1024 )
{
    //nop
}

void
AtlasBuilder::setSize(unsigned width, unsigned height)
{
    _width = width;
    _height = height;
}

bool
AtlasBuilder::build(ResourceLibrary* inputLib, Atlas& out) const
{
    if ( !inputLib )
        return false;

    // prepare an atlaser:
    TextureAtlasBuilderEx tab;

    // maximum size of the texture (x,y)
    tab.setMaximumAtlasSize( (int)_width, (int)_height );

    // texels between atlased images
    tab.setMargin( 1 );

    // fetch all the skins from the catalog:
    SkinResourceVector skins;
    inputLib->getSkins( skins );
    for(SkinResourceVector::const_iterator i = skins.begin(); i != skins.end(); ++i)
    {
        const SkinResource* skin = i->get();
        osg::Image* image = skin->createImage( _options );
        if ( image )
        {
            tab.addSource( image );
            OE_INFO << LC << "Added skin: \"" << skin->name() << "\"" << std::endl;
        }
        else
        {
            OE_WARN << LC << "Failed to load image from catalog: \""
                << skin->name() << "\" ... skipped" << std::endl;
        }
    }

    unsigned numSources = tab.getNumSources();
    OE_INFO << LC << "Added " << numSources << " images ... building atlas ..." << std::endl;

    tab.buildAtlas();

    const TextureAtlasBuilderEx::AtlasListEx& atlasList = tab.getAtlasList();

    // create the target multi-layer image.
    out._image = new osg::Image();
    out._image->allocateImage(
        _width,
        _height,
        atlasList.size(),
        GL_RGBA,
        GL_UNSIGNED_BYTE);

    // initialize to all zeros
    memset(out._image->data(), 0, out._image->getTotalSizeInBytesIncludingMipmaps());

    // combine each of the atlas images into the corresponding "r" slot of the composed image:
    for(int r=0; r<(int)atlasList.size(); ++r)
    {
        osg::Image* atlasImage = atlasList[r]->_image.get();

        ImageUtils::PixelReader read (atlasImage);
        ImageUtils::PixelWriter write(out._image.get());

        for(int s=0; s<atlasImage->s(); ++s)
            for(int t=0; t<atlasImage->t(); ++t)
                write(read(s, t, 0), s, t, r);
    }

    out._lib = 0L;
    return true;
}
