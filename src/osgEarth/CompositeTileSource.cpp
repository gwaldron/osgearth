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
#include <osgEarth/CompositeTileSource>
#include <osgEarth/ImageUtils>
#include <osgDB/FileNameUtils>

#define LC "[CompositeTileSource] "

using namespace osgEarth;

//------------------------------------------------------------------------

CompositeTileSourceOptions::CompositeTileSourceOptions( const TileSourceOptions& options ) :
TileSourceOptions( options )
{
    setDriver( "composite" );
    fromConfig( _conf );
}

void
CompositeTileSourceOptions::addTileSource( const TileSourceOptions& options )
{
    _optionsVector.push_back( options );
}

void
CompositeTileSourceOptions::addTileSource( TileSource* source )
{
    _sources.push_back( source );
}

Config 
CompositeTileSourceOptions::getConfig() const
{
    Config conf = TileSourceOptions::getConfig();

    for( TileSourceOptionsVector::const_iterator i = _optionsVector.begin(); i != _optionsVector.end(); ++i )
    {
        conf.add( "image", i->getConfig() );
    }

    return conf;
}

void 
CompositeTileSourceOptions::mergeConfig( const Config& conf )
{
    TileSourceOptions::mergeConfig( conf );
    fromConfig( conf );
}

void 
CompositeTileSourceOptions::fromConfig( const Config& conf )
{
    const ConfigSet& children = conf.children("image");
    for( ConfigSet::const_iterator i = children.begin(); i != children.end(); ++i )
    {
        addTileSource( TileSourceOptions( *i ) );
    }
}

//------------------------------------------------------------------------

CompositeTileSource::CompositeTileSource( const TileSourceOptions& options ) :
_options( options ),
_initialized( false )
{
    // first try the options:
    const TileSourceOptionsVector& ov = _options.getTileSourceOptionsVector();
    for( TileSourceOptionsVector::const_iterator i = ov.begin(); i != ov.end(); ++i )
    {
        TileSource* source = TileSourceFactory::create( *i );
        if ( source )
            _sources.push_back( source );
        else
            OE_WARN << LC << "Could not find a TileSource for driver '" << i->getDriver() << "'" << std::endl;
    }

    // then try the instances. so far this is not really written for mixing the two.
    const TileSourceVector& tv = _options.getTileSources();
    std::copy( tv.begin(), tv.end(), std::back_inserter(_sources) );
}

osg::Image*
CompositeTileSource::createImage( const TileKey& key, ProgressCallback* progress )
{
    //TODO
    int tileSize = getPixelsPerTile();

    std::vector< osg::ref_ptr<osg::Image> > images;
    images.reserve( _sources.size() );

    for( TileSourceVector::iterator i = _sources.begin(); i != _sources.end(); ++i )
    {
        TileSource* source = i->get();

        if ( !source->getBlacklist()->contains( key.getTileId() ) )
        {
            //Only try to get data if the source actually has data
            if ( source->hasData( key ) )
            {
                osg::ref_ptr<osg::Image> image;
                source->getImage( key, image, progress );

                //If the image is not valid and the progress was not cancelled, blacklist
                if (!image.valid() && (!progress || !progress->isCanceled()))
                {
                    //Add the tile to the blacklist
                    OE_DEBUG << LC << "Adding tile " << key.str() << " to the blacklist" << std::endl;
                    source->getBlacklist()->add( key.getTileId() );
                }

                if ( image.valid() )
                    images.push_back( image.get() );
            }
            else
            {
                OE_DEBUG << LC << "Source has no data at " << key.str() << std::endl;
            }
        }
        else
        {
            OE_DEBUG << LC << "Tile " << key.str() << " is blacklisted, not checking" << std::endl;
        }
    }

    if ( images.size() == 0 )
    {
        return 0L;
    }
    else if ( images.size() == 1 )
    {
        return images[0].get();
    }
    else
    {
        osg::Image* result = new osg::Image( *images[0].get() );
        for( int i=1; i<images.size(); ++i )
        {
            float opacity = 1.0f; // todo: this should be the layer opacity....
            ImageUtils::mix( result, images[i].get(), opacity );
        }
        return result;
    }
}

void
CompositeTileSource::initialize( const std::string& referenceURI, const Profile* overrideProfile )
{
    osg::ref_ptr<const Profile> profile = 0L;

    if ( overrideProfile )
        profile = overrideProfile;

    for( TileSourceVector::iterator i = _sources.begin(); i != _sources.end(); ++i )
    {
        TileSource* source = i->get();
        if ( source )
        {
            source->initialize( referenceURI, overrideProfile );
            if ( !profile.valid() )
                profile = source->getProfile();
        }
    }

    setProfile( profile.get() );

    _initialized = true;
}

//------------------------------------------------------------------------

namespace
{
    struct CompositeTileSourceDriver : public TileSourceDriver
    {
        CompositeTileSourceDriver()
        {
            supportsExtension( "osgearth_composite", "Composite tile source driver" );
        }

        virtual const char* className()
        {
            return "CompositeTileSourceDriver";
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* options) const
        {
            if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
                return ReadResult::FILE_NOT_HANDLED;

            return new CompositeTileSource( getTileSourceOptions(options) );
        }
    };
}
REGISTER_OSGPLUGIN(osgearth_composite, CompositeTileSourceDriver)
