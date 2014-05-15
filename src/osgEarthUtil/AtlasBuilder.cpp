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
        
        typedef SourceList SourceListEx;
        typedef Source SourceEx;
        const SourceListEx& getSourceList()
        {
            return _sourceList;
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
AtlasBuilder::build(const ResourceLibrary* inputLib,
                    const std::string&     newAtlasURI,
                    Atlas&                 out          ) const
{
    if ( !inputLib )
        return false;

    // prepare an atlaser:
    TextureAtlasBuilderEx tab;

    // maximum size of the texture (x,y)
    tab.setMaximumAtlasSize( (int)_width, (int)_height );

    // texels between atlased images
    tab.setMargin( 1 );

    // clone the Resource library so we can re-write the URIs and add 
    // texture matrix information.
    out._lib = new ResourceLibrary( inputLib->getConfig() );
    out._lib->initialize( _options );
    out._lib->uri().unset();

    // store a mapping from atlasbuilder source to skin.
    typedef std::map<TextureAtlasBuilderEx::SourceEx*, SkinResource*> SourceSkinMap;
    SourceSkinMap sourceSkins;

    // fetch all the skins from the catalog:
    SkinResourceVector skins;
    out._lib->getSkins( skins );
    for(SkinResourceVector::iterator i = skins.begin(); i != skins.end(); ++i)
    {
        SkinResource* skin = i->get();

        // skip tiled skins, for now
        if ( skin->isTiled() == true )
        {
            continue;
        }

        osg::Image* image = skin->createImage( _options );
        if ( image )
        {
            // ensure we're not trying to atlas an atlas.
            if ( image->r() > 1 )
            {
                OE_WARN << LC <<
                    "Found an image with more than one layer. You cannot create an "
                    "altas from another atlas. Stopping." << std::endl;
                return false;
            }

            tab.addSource( image );

            // re-write the URI to point at our new atlas:
            skin->imageURI() = newAtlasURI;

            // save the associate so we can come back later:
            sourceSkins[tab.getSourceList().back().get()] = skin;

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

    // build the atlas images.
    tab.buildAtlas();

    const TextureAtlasBuilderEx::AtlasListEx& atlasList = tab.getAtlasList();

    // Atlas images are not all the same size. Figure out the largest size
    unsigned maxS=0, maxT=0;

    for(unsigned r=0; r<atlasList.size(); ++r)
    {
        unsigned s = atlasList[r]->_image->s();
        unsigned t = atlasList[r]->_image->t();

        OE_INFO << LC
            << "Altas image " << r << ": size = (" << s << ", " << t << ")" << std::endl;

        if ( s > maxS )
            maxS = s;
        if ( t > maxT )
            maxT = t;
    }

    OE_INFO << LC <<
        "Final atlas size will be (" << maxS << ", " << maxT << ")" << std::endl;

    // create the target multi-layer image.
    out._image = new osg::Image();
    out._image->allocateImage(
        maxS,
        maxT,
        atlasList.size(),
        GL_RGBA,
        GL_UNSIGNED_BYTE);

    // initialize to all zeros
    memset(out._image->data(), 0, out._image->getTotalSizeInBytesIncludingMipmaps());

    // combine each of the atlas images into the corresponding "r" slot of the composed image:
    for(int r=0; r<(int)atlasList.size(); ++r)
    {
        // copy the atlas image into the image array:
        osg::Image* atlasImage = atlasList[r]->_image.get();

        ImageUtils::PixelReader read (atlasImage);
        ImageUtils::PixelWriter write(out._image.get());

        for(int s=0; s<atlasImage->s(); ++s)
            for(int t=0; t<atlasImage->t(); ++t)
                write(read(s, t, 0), s, t, r);

        // for each source in this atlas layer, apply its texture matrix info
        // to the new catalog.
        for(int k=0; k<atlasList[r]->_sourceList.size(); ++k)
        {
            TextureAtlasBuilderEx::SourceEx* source = atlasList[r]->_sourceList[k].get();
            SourceSkinMap::iterator n = sourceSkins.find(source);
            if ( n != sourceSkins.end() )
            {
                SkinResource* skin = n->second;
                skin->imageLayer()  = r;
                skin->imageBiasS()  = (float)source->_x/(float)maxS;
                skin->imageBiasT()  = (float)source->_y/(float)maxT;
                skin->imageScaleS() = (float)source->_image->s()/(float)maxS;
                skin->imageScaleT() = (float)source->_image->t()/(float)maxT;
            }
        }
    }

    return true;
}
