/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include <osgDB/FileNameUtils>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgUtil/Optimizer>
#include <vector>
#include <string>
#include <stdlib.h> // for ::getenv

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
    
    /** PixelVisitor functor to fill an image with a value */
    struct SetDefaults
    {
        osg::Vec4f _value;
        bool operator()(osg::Vec4f& inout) {
            inout = _value;
            return true;
        }
    };
}


AtlasBuilder::AtlasBuilder(const osgDB::Options* options) :
_options( options ),
_width  ( 1024 ),
_height ( 1024 ),
_debug  ( false ),
_rgb    ( false )
{
    //nop
    if (::getenv("OSGEARTH_ATLAS_DEBUG"))
        _debug = true;
}

void
AtlasBuilder::setSize(unsigned width, unsigned height)
{
    _width = width;
    _height = height;
}

void
AtlasBuilder::addAuxFilePattern(const std::string& pattern,
                                const osg::Vec4f&  defaults)
{
    _auxPatterns.push_back(pattern);
    _auxDefaults.push_back(defaults);
}

bool
AtlasBuilder::build(const ResourceLibrary* inputLib,
                    const std::string&     newAtlasURI,
                    Atlas&                 out          ) const
{
    if ( !inputLib )
        return false;

    // prepare an atlaser:
    typedef std::vector<TextureAtlasBuilderEx> TABs;
    TABs tabs(1 + _auxPatterns.size());
    TABs::iterator maintab = tabs.begin();

    for(TABs::iterator tab = tabs.begin(); tab != tabs.end(); ++tab)
    {
        // maximum size of the texture (x,y)
        tab->setMaximumAtlasSize( (int)_width, (int)_height );

        // texels between atlased images
        tab->setMargin( 1 );
    }

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

        // skip skins that say "no atlas please"
        if ( skin->atlasHint() == false )
        {
            continue;
        }

        osg::ref_ptr<osg::Image> image = skin->createImage( _options );
        if ( image.valid() )
        {
            OE_INFO << LC << "Loaded skin file: " << skin->imageURI()->full() << std::endl;

            // ensure we're not trying to atlas an atlas.
            if ( image->r() > 1 )
            {
                OE_WARN << LC <<
                    "Found an image with more than one layer. You cannot create an "
                    "altas from another atlas. Stopping." << std::endl;
                return false;
            }

            // normalize to RGBA8
            image = ImageUtils::convertToRGBA8(image);

            maintab->addSource( image );

            // for each aux pattern, either load and resize the aux image or create
            // an empty placeholder.
            TABs::iterator tab = maintab;
            ++tab;
            for(int a=0; a<_auxPatterns.size(); ++a, ++tab)
            {
                const std::string& pattern      = _auxPatterns[a];
                const osg::Vec4f&  defaultValue = _auxDefaults[a];

                std::string base = osgDB::getNameLessExtension(skin->imageURI()->full());
                std::string ext  = osgDB::getFileExtension(skin->imageURI()->base());
                std::string auxFile = base + "_" + pattern + "." + ext;

                // read in the auxiliary image:
                osg::ref_ptr<osg::Image> auxImage;
                auxImage = osgDB::readImageFile( auxFile, _options.get() );

                // if that didn't work, try alternate extensions:
                const char* alternateExtensions[3] = {"png", "jpg", "osgb"};
                for(int b = 0; b < 3 && !auxImage.valid(); ++b)
                {
                    auxFile = base + "_" + pattern + "." + alternateExtensions[b];
                    auxImage = osgDB::readImageFile( auxFile, _options.get() );
                }

                if ( auxImage.valid() )
                {
                    OE_INFO << LC << "  Found aux file: " << auxFile << std::endl;
                    auxImage = ImageUtils::convertToRGBA8(auxImage);
                }
                else
                {
                    // failing that, create an empty one as a placeholder.
                    auxImage = new osg::Image();
                    auxImage->allocateImage(image->s(), image->t(), 1, GL_RGBA, GL_UNSIGNED_BYTE);
                    ImageUtils::PixelVisitor<SetDefaults> filler;
                    filler._value = defaultValue;
                    filler.accept(auxImage.get());
                }

                if ( auxImage->s() != image->s() || auxImage->t() != image->t() )
                {
                    osg::ref_ptr<osg::Image> temp;
                    osgEarth::ImageUtils::resizeImage(auxImage.get(), image->s(), image->t(), temp);
                    auxImage = temp.get();
                    OE_INFO << "  ...resized " << auxFile << " to match atlas size" << std::endl;
                }

                if ( !ImageUtils::sameFormat(image, auxImage.get()) ) 
                {
                    auxImage = ImageUtils::convertToRGBA8(auxImage.get());
                }

                tab->addSource( auxImage.get() );
            }

            // re-write the URI to point at our new atlas:
            skin->imageURI() = newAtlasURI;

            // save the associate so we can come back later:
            sourceSkins[maintab->getSourceList().back().get()] = skin;

            //OE_INFO << LC << "Added skin: \"" << skin->name() << "\"" << std::endl;
        }
        else
        {
            OE_WARN << LC << "Failed to load image from catalog: \""
                << skin->name() << "\" ... skipped" << std::endl;
        }
    }

    unsigned numSources = maintab->getNumSources();
    OE_INFO << LC << "Added " << numSources << " images ... building atlas ..." << std::endl;

    // build the atlas images.
    for(TABs::iterator tab = tabs.begin(); tab != tabs.end(); ++tab )
    {
        tab->buildAtlas();
    }

    const TextureAtlasBuilderEx::AtlasListEx& mainAtlasList = maintab->getAtlasList();

    // Atlas images are not all the same size. Figure out the largest size
    unsigned maxS=0, maxT=0;

    for(unsigned r=0; r<mainAtlasList.size(); ++r)
    {
        unsigned s = mainAtlasList[r]->_image->s();
        unsigned t = mainAtlasList[r]->_image->t();

        OE_INFO << LC
            << "Altas image " << r << ": size = (" << s << ", " << t << ")" << std::endl;

        if ( s > maxS )
            maxS = s;
        if ( t > maxT )
            maxT = t;
    }

    // protecte against a div0
    maxS = std::max(maxS, 1u);
    maxT = std::max(maxT, 1u);

    OE_INFO << LC <<
        "Final atlas size will be (" << maxS << ", " << maxT << ")" << std::endl;

    
    for(TABs::iterator tab = tabs.begin(); tab != tabs.end(); ++tab )
    {
        const TextureAtlasBuilderEx::AtlasListEx& atlasList = tab->getAtlasList();

        // create the target multi-layer image.
        osg::Image* imageArray = new osg::Image();

        imageArray->allocateImage(
            maxS,
            maxT,
            atlasList.size(),
            _rgb ? GL_RGB : GL_RGBA,
            GL_UNSIGNED_BYTE);

        // initialize to all zeros
        memset(imageArray->data(), 0, imageArray->getTotalSizeInBytesIncludingMipmaps());

        // combine each of the atlas images into the corresponding "r" slot of the composed image:
        for(int r=0; r<(int)atlasList.size(); ++r)
        {
            // copy the atlas image into the image array:
            osg::Image* atlasImage = atlasList[r]->_image.get();

            if ( _debug )
            {
                std::string name = Stringify() << "image_" << (int)(tab-tabs.begin()) << "_" << r << ".png";
                osgDB::writeImageFile(*atlasImage, name);
            }

            ImageUtils::PixelReader read (atlasImage);
            ImageUtils::PixelWriter write(imageArray);

            for(int s=0; s<atlasImage->s(); ++s)
                for(int t=0; t<atlasImage->t(); ++t)
                    write(read(s, t, 0), s, t, r);
        }

        out._images.push_back(imageArray);
    }

    
    // for each source in this atlas layer, apply its texture matrix info
    // to the new catalog.
    maxS = std::max(maxS, 1u);
    maxT = std::max(maxT, 1u);

    for(int r=0; r<(int)mainAtlasList.size(); ++r)
    {
        for(int k=0; k<mainAtlasList[r]->_sourceList.size(); ++k)
        {
            TextureAtlasBuilderEx::SourceEx* source = mainAtlasList[r]->_sourceList[k].get();
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
