/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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
#include <osgEarth/StringUtils>
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
CompositeTileSourceOptions::add( const ImageLayerOptions& options )
{
    Component c;
    c._imageLayerOptions = options;
    _components.push_back( c );
}

Config 
CompositeTileSourceOptions::getConfig() const
{
    Config conf = TileSourceOptions::newConfig();

    for( ComponentVector::const_iterator i = _components.begin(); i != _components.end(); ++i )
    {
        if ( i->_imageLayerOptions.isSet() )
            conf.add( "image", i->_imageLayerOptions->getConfig() );
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
        add( ImageLayerOptions( *i ) );
    }

    if (conf.children("elevation").size() > 0 || conf.children("heightfield").size() > 0 ||
        conf.children("model").size() > 0 || conf.children("overlay").size() > 0 )
    {
        OE_WARN << LC << "Illegal - composite driver only supports image layers" << std::endl;
    }
}

//------------------------------------------------------------------------

namespace
{
    // some helper types.
    typedef std::pair< osg::ref_ptr<osg::Image>, float> ImageOpacityPair;
    typedef std::vector<ImageOpacityPair> ImageMixVector;

    // same op that occurs in ImageLayer.cpp ... maybe consilidate
    struct ImageLayerPreCacheOperation : public TileSource::ImageOperation
    {
        void operator()( osg::ref_ptr<osg::Image>& image )
        {
            _processor.process( image );
        }

        ImageLayerTileProcessor _processor;
    };
}

//-----------------------------------------------------------------------

CompositeTileSource::CompositeTileSource( const TileSourceOptions& options ) :
_options    ( options ),
_initialized( false ),
_dynamic    ( false )
{
#if 0
    for(CompositeTileSourceOptions::ComponentVector::iterator i = _options._components.begin(); 
        i != _options._components.end(); )

    {
        //if ( i->_imageLayerOptions.isSet() )
        //{
        //    if ( i->_imageLayerOptions->driver().isSet() )
        //        i->_tileSourceOptions = i->_imageLayerOptions->driver().value();
        //}

        if ( i->_tileSourceOptions.isSet() )
        {
            if ( !i->_tileSourceInstance->valid() )
                i->_tileSourceInstance = TileSourceFactory::create( i->_tileSourceOptions.value() );
            
            if ( !i->_tileSourceInstance->valid() )
                OE_WARN << LC << "Could not find a TileSource for driver [" << i->_tileSourceOptions->getDriver() << "]" << std::endl;
        }

        if ( !i->_tileSourceInstance->valid() )
        {
            OE_WARN << LC << "A component has no valid TileSource ... removing." << std::endl;
            i = _options._components.erase( i );
        }
        else
        {
            ++i;
        }
    }
#endif
}

osg::Image*
CompositeTileSource::createImage(const TileKey&    key,
                                 ProgressCallback* progress )
{
    ImageMixVector images;
    images.reserve( _options._components.size() );

    for(CompositeTileSourceOptions::ComponentVector::const_iterator i = _options._components.begin();
        i != _options._components.end();
        ++i )
    {
        if ( progress && progress->isCanceled() )
            return 0L;

        TileSource* source = i->_tileSourceInstance.get();
        if ( source )
        {
            //TODO:  This duplicates code in ImageLayer::isKeyValid.  Maybe should move that to TileSource::isKeyValid instead
            int minLevel = 0;
            int maxLevel = INT_MAX;
            if (i->_imageLayerOptions->minLevel().isSet())
            {
                minLevel = i->_imageLayerOptions->minLevel().value();
            }
            else if (i->_imageLayerOptions->minLevelResolution().isSet())
            {
                minLevel = source->getProfile()->getLevelOfDetailForHorizResolution( i->_imageLayerOptions->minLevelResolution().value(), source->getPixelsPerTile());            
            }

            if (i->_imageLayerOptions->maxLevel().isSet())
            {
                maxLevel = i->_imageLayerOptions->maxLevel().value();
            }
            else if (i->_imageLayerOptions->maxLevelResolution().isSet())
            {
                maxLevel = source->getProfile()->getLevelOfDetailForHorizResolution( i->_imageLayerOptions->maxLevelResolution().value(), source->getPixelsPerTile());            
            }

            // check that this source is within the level bounds:
            if (minLevel > (int)key.getLevelOfDetail() ||
                maxLevel < (int)key.getLevelOfDetail() )
            {
                continue;
            }

            if ( !source->getBlacklist()->contains( key.getTileId() ) )
            {
                //Only try to get data if the source actually has data
                if ( source->hasData( key ) )
                {
                    osg::ref_ptr< ImageLayerPreCacheOperation > preCacheOp;
                    if ( i->_imageLayerOptions.isSet() )
                    {
                        preCacheOp = new ImageLayerPreCacheOperation();
                        preCacheOp->_processor.init( i->_imageLayerOptions.value(), _dbOptions.get(), true );                        
                    }

                    ImageOpacityPair imagePair(
                        source->createImage( key, preCacheOp.get(), progress ),
                        1.0f );

                    //If the image is not valid and the progress was not cancelled, blacklist
                    if (!imagePair.first.valid() && (!progress || !progress->isCanceled()))
                    {
                        //Add the tile to the blacklist
                        OE_DEBUG << LC << "Adding tile " << key.str() << " to the blacklist" << std::endl;
                        source->getBlacklist()->add( key.getTileId() );
                    }

                    if ( imagePair.first.valid() )
                    {
                        // check for opacity:
                        imagePair.second = i->_imageLayerOptions.isSet() ? i->_imageLayerOptions->opacity().value() : 1.0f;

                        images.push_back( imagePair );
                    }
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
    }

    if ( progress && progress->isCanceled() )
    {
        return 0L;
    }
    else if ( images.size() == 0 )
    {
        return 0L;
    }
    else if ( images.size() == 1 )
    {
        return images[0].first.release();
    }
    else
    {
        osg::Image* result = new osg::Image( *images[0].first.get() );
        for( unsigned int i=1; i<images.size(); ++i )
        {
            ImageOpacityPair& pair = images[i];
            if ( pair.first.valid() )
            {
                ImageUtils::mix( result, pair.first.get(), pair.second );
            }
        }
        return result;
    }
}

bool
CompositeTileSource::add( TileSource* ts )
{
    if ( _initialized )
    {
        OE_WARN << LC << "Illegal: cannot add a tile source after initialization" << std::endl;
        return false;
    }

    if ( !ts )
    {
        OE_WARN << LC << "Illegal: tried to add a NULL tile source" << std::endl;
        return false;
    }

    CompositeTileSourceOptions::Component comp;
    comp._tileSourceInstance = ts;
    _options._components.push_back( comp );

    return true;
}

bool
CompositeTileSource::add( TileSource* ts, const ImageLayerOptions& options )
{
    if ( add(ts) )
    {
        _options._components.back()._imageLayerOptions = options;
        return true;
    }
    else
    {
        return false;
    }
}

void
CompositeTileSource::initialize(const osgDB::Options* dbOptions, 
                                const Profile*        overrideProfile )
{
    _dbOptions = dbOptions;
    osg::ref_ptr<const Profile> profile = overrideProfile;

    for(CompositeTileSourceOptions::ComponentVector::iterator i = _options._components.begin();
        i != _options._components.end(); )
    {
        if ( i->_imageLayerOptions.isSet() )
        {
            if ( !i->_tileSourceInstance.valid() )
            {
                i->_tileSourceInstance = TileSourceFactory::create( i->_imageLayerOptions->driver().value() );
            
                if ( !i->_tileSourceInstance.valid() )
                {
                    OE_WARN << LC << "Could not find a TileSource for driver [" << i->_imageLayerOptions->driver()->getDriver() << "]" << std::endl;
                }
            }
        }

        if ( !i->_tileSourceInstance.valid() )
        {
            OE_WARN << LC << "A component has no valid TileSource ... removing." << std::endl;
            i = _options._components.erase( i );
        }
        else
        {
            TileSource* source = i->_tileSourceInstance.get();
            if ( source )
            {
                osg::ref_ptr<const Profile> localOverrideProfile = overrideProfile;

                const TileSourceOptions& opt = source->getOptions();
                if ( opt.profile().isSet() )
                    localOverrideProfile = Profile::create( opt.profile().value() );

                source->initialize( dbOptions, localOverrideProfile.get() );

                if ( !profile.valid() )
                {
                    // assume the profile of the first source to be the overall profile.
                    profile = source->getProfile();
                }
                else if ( !profile->isEquivalentTo( source->getProfile() ) )
                {
                    // if sub-sources have different profiles, print a warning because this is
                    // not supported!
                    OE_WARN << LC << "Components with differing profiles are not supported. " 
                        << "Visual anomalies may result." << std::endl;
                }
                
                _dynamic = _dynamic || source->isDynamic();

                // gather extents
                const DataExtentList& extents = source->getDataExtents();
                for( DataExtentList::const_iterator j = extents.begin(); j != extents.end(); ++j )
                {
                    getDataExtents().push_back( *j );
                }
            }
        }

        ++i;
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
