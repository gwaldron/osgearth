/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#include <limits.h>

#include <osgEarth/TileSource>
#include <osgEarth/ImageToHeightFieldConverter>
#include <osgEarth/Registry>
#include <osgEarth/Progress>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#define LC "[TileSource] "

using namespace osgEarth;

//#undef OE_DEBUG
//#define OE_DEBUG OE_INFO

//------------------------------------------------------------------------

TileBlacklist::TileBlacklist() :
_tiles(true, 1024)
{
    //NOP
}

void
TileBlacklist::add(const TileKey& key)
{
    _tiles.insert(key, true);
    OE_DEBUG << "Added " << key.str() << " to blacklist" << std::endl;
}

void
TileBlacklist::remove(const TileKey& key)
{
    _tiles.erase(key);
    OE_DEBUG << "Removed " << key.str() << " from blacklist" << std::endl;
}

void
TileBlacklist::clear()
{
    _tiles.clear();
    OE_DEBUG << "Cleared blacklist" << std::endl;
}

bool
TileBlacklist::contains(const TileKey& key) const
{
    return _tiles.has(key);
}

TileBlacklist*
TileBlacklist::read(std::istream &in)
{
    osg::ref_ptr< TileBlacklist > result = new TileBlacklist();

    while (!in.eof())
    {
        std::string line;
        std::getline(in, line);
        if (!line.empty())
        {
            int z, x, y;
            if (sscanf(line.c_str(), "%d %d %d", &z, &x, &y) == 3)
            {
                result->add(TileKey(z, x, y, 0L));
            }

        }
    }

    return result.release();
}

TileBlacklist*
TileBlacklist::read(const std::string &filename)
{
    if (osgDB::fileExists(filename) && (osgDB::fileType(filename) == osgDB::REGULAR_FILE))
    {
        std::ifstream in( filename.c_str() );
        return read( in );
    }
    return NULL;
}

void
TileBlacklist::write(const std::string &filename) const
{
    std::string path = osgDB::getFilePath(filename);
    if (!path.empty() && !osgDB::fileExists(path) && !osgDB::makeDirectory(path))
    {
        OE_NOTICE << "Couldn't create path " << path << std::endl;
        return;
    }
    std::ofstream out(filename.c_str());
    write(out);
}

namespace {
    struct WriteFunctor : public LRUCache<TileKey,bool>::Functor {
        std::ostream& _out;
        WriteFunctor(std::ostream& out) : _out(out) { }
        void operator()(const TileKey& key, const bool& value) {
            _out << key.getLOD() << ' ' << key.getTileX() << ' ' << key.getTileY() << std::endl;
        }
    };
}

void
TileBlacklist::write(std::ostream &output) const
{
    WriteFunctor writer(output);
    _tiles.iterate(writer);
}


//------------------------------------------------------------------------


TileSourceOptions::TileSourceOptions( const ConfigOptions& options ) :
DriverConfigOptions   ( options ),
_L2CacheSize          ( 16 ),
_bilinearReprojection ( true ),
_coverage             ( false )
{
    fromConfig( _conf );
}


Config
TileSourceOptions::getConfig() const
{
    Config conf = DriverConfigOptions::getConfig();
    conf.set( "blacklist_filename", _blacklistFilename);
    conf.set( "l2_cache_size", _L2CacheSize );
    conf.set( "bilinear_reprojection", _bilinearReprojection );
    conf.set( "coverage", _coverage );
    conf.set( "osg_option_string", _osgOptionString );
    conf.set( "profile", _profileOptions );
    return conf;
}


void
TileSourceOptions::mergeConfig( const Config& conf )
{
    DriverConfigOptions::mergeConfig( conf );
    fromConfig( conf );
}


void
TileSourceOptions::fromConfig( const Config& conf )
{
    conf.get( "blacklist_filename", _blacklistFilename);
    conf.get( "l2_cache_size", _L2CacheSize );
    conf.get( "bilinear_reprojection", _bilinearReprojection );
    conf.get( "coverage", _coverage );
    conf.get( "osg_option_string", _osgOptionString );
    conf.get( "profile", _profileOptions );
}


//------------------------------------------------------------------------

// statics

const char* TileSource::INTERFACE_NAME = "osgEarth::TileSource";

const TileSource::Mode TileSource::MODE_READ   = 0x01;
const TileSource::Mode TileSource::MODE_WRITE  = 0x02;
const TileSource::Mode TileSource::MODE_CREATE = 0x04;


TileSource::TileSource(const TileSourceOptions& options) :
_options( options ),
_status ( Status::Error("Not initialized") ),
_mode   ( 0 ),
_openCalled( false ),
_tileSize(256),
_noDataValue( (float)SHRT_MIN ),
_minValidValue( -32000.0f ),
_maxValidValue(  32000.0f )
{
    if (_options.blacklistFilename().isSet())
    {
        _blacklistFilename = _options.blacklistFilename().value();
    }


    if (!_blacklistFilename.empty() && osgDB::fileExists(_blacklistFilename))
    {
        _blacklist = TileBlacklist::read(_blacklistFilename);
        if (_blacklist.valid())
        {
            OE_INFO << "Read blacklist from file" << _blacklistFilename << std::endl;
        }
    }

    if (!_blacklist.valid())
    {
        //Initialize the blacklist if we couldn't read it.
        _blacklist = new TileBlacklist();
    }
}

TileSource::~TileSource()
{
    if (_blacklist.valid() && !_blacklistFilename.empty())
    {
        _blacklist->write(_blacklistFilename);
    }
}

void
TileSource::setDefaultL2CacheSize(int size)
{
    if (_options.L2CacheSize().isSet() == false)
    {
        _options.L2CacheSize().init(size);
    }
}

const Status&
TileSource::open(const Mode&           openMode,
                 const osgDB::Options* readOptions)
{
    if (!_openCalled)
    {
        _mode = openMode;

        // Initialize the l2 cache size to the options.
        int l2CacheSize = _options.L2CacheSize().get();

        // See if it was overridden with an env var.
        char const* l2env = ::getenv( "OSGEARTH_L2_CACHE_SIZE" );
        if ( l2env )
        {
            l2CacheSize = as<int>( std::string(l2env), 0 );
        }

        // Env cache-only mode also disables the L2 cache.
        char const* noCacheEnv = ::getenv( "OSGEARTH_MEMORY_PROFILE" );
        if ( noCacheEnv )
        {
            l2CacheSize = 0;
        }

        // Initialize the l2 cache if it's size is > 0
        if ( l2CacheSize > 0 )
        {
            _memCache = new MemCache( l2CacheSize );
        }

        // Initialize the underlying data store
        Status status = initialize(readOptions);

        // Check the return status. The TileSource MUST have a valid
        // Profile after initialization.
        if ( status == STATUS_OK )
        {
            if ( getProfile() != 0L )
            {
                _status = status;
            }
            else
            {
                _status = Status::Error("No profile available");
            }
        }
        else
        {
            _status = status;
        }

        _openCalled = true;
    }

    return _status;
}

int
TileSource::getPixelsPerTile() const
{
    return _tileSize;
}

void
TileSource::setPixelsPerTile(unsigned size)
{
    _tileSize = size;
}

osg::Image*
TileSource::createImage(const TileKey&        key,
                        ImageOperation*       prepOp,
                        ProgressCallback*     progress )
{
    if (getStatus().isError())
        return 0L;

    // Try to get it from the memcache fist
    if (_memCache.valid())
    {
        ReadResult r = _memCache->getOrCreateDefaultBin()->readImage(key.str(), 0L);
        if ( r.succeeded() )
            return r.releaseImage();
    }

    osg::ref_ptr<osg::Image> newImage = createImage(key, progress);

    // Check for cancelation. The TileSource implementation should do this
    // internally but we check here once last time just in case the
    // implementation does not.
    if (progress && progress->isCanceled())
    {
        return 0L;
    }

    // Run the pre-caching operation if there is one:
    if ( prepOp )
        (*prepOp)( newImage );

    // Cache to the L2 cache:
    if ( newImage.valid() && _memCache.valid() )
    {
        _memCache->getOrCreateDefaultBin()->write(key.str(), newImage.get(), 0L);
    }

    return newImage.release();
}

osg::HeightField*
TileSource::createHeightField(const TileKey&        key,
                              HeightFieldOperation* prepOp,
                              ProgressCallback*     progress )
{
    if (getStatus().isError())
        return 0L;

    // Try to get it from the memcache first:
    if (_memCache.valid())
    {
        ReadResult r = _memCache->getOrCreateDefaultBin()->readObject(key.str(), 0L);
        if ( r.succeeded() )
        {
            return r.release<osg::HeightField>();
        }
    }

    osg::ref_ptr<osg::HeightField> newHF = createHeightField( key, progress );

    // Check for cancelation. The TileSource implementation should do this
    // internally but we check here once last time just in case the
    // implementation does not.
    if (progress && progress->isCanceled())
    {
        return 0L;
    }

    if ( prepOp )
        (*prepOp)( newHF );

    if ( newHF.valid() && _memCache.valid() )
    {
        _memCache->getOrCreateDefaultBin()->write(key.str(), newHF.get(), 0L);
    }

    return newHF.release();
}

osg::Image*
TileSource::createImage(const TileKey&    key,
                        ProgressCallback* progress)
{
    return 0L;
}

osg::HeightField*
TileSource::createHeightField(const TileKey&        key,
                              ProgressCallback*     progress)
{
    if (getStatus().isError())
        return 0L;

    osg::ref_ptr<osg::Image> image = createImage(key, progress);
    osg::HeightField* hf = 0;
    if (image.valid())
    {
        ImageToHeightFieldConverter conv;
        hf = conv.convert( image.get() );
    }
    return hf;
}

bool
TileSource::storeHeightField(const TileKey&     key,
                             const osg::HeightField*  hf,
                             ProgressCallback* progress)
{
    if (getStatus().isError() || hf == 0L )
        return 0L;

    ImageToHeightFieldConverter conv;
    osg::ref_ptr<osg::Image> image = conv.convert(hf, 32);
    if (image.valid())
    {
        return storeImage(key, image.get(), progress);
    }
    return false;
}

bool
TileSource::isOK() const
{
    return _status.isOK();
}

void
TileSource::setProfile( const Profile* profile )
{
    _profile = profile;
}

const Profile*
TileSource::getProfile() const
{
    return _profile.get();
}

TileBlacklist*
TileSource::getBlacklist()
{
    return _blacklist.get();
}

const TileBlacklist*
TileSource::getBlacklist() const
{
    return _blacklist.get();
}

//------------------------------------------------------------------------

#undef  LC
#define LC "[TileSourceFactory] "
#define TILESOURCE_OPTIONS_TAG   "__osgEarth::TileSourceOptions"
#define TILESOURCE_INTERFACE_TAG "__osgEarth::Interface"

TileSource*
TileSourceFactory::create(const TileSourceOptions& options)
{
    osg::ref_ptr<TileSource> source;

    std::string driver = options.getDriver();
    if ( driver.empty() )
    {
        OE_WARN << LC << "ILLEGAL- no driver set for tile source" << std::endl;
        return 0L;
    }

    osg::ref_ptr<osgDB::Options> dbopt = Registry::instance()->cloneOrCreateOptions();
    dbopt->setPluginData      ( TILESOURCE_OPTIONS_TAG,   (void*)&options );
    dbopt->setPluginStringData( TILESOURCE_INTERFACE_TAG, TileSource::INTERFACE_NAME );

    std::string driverExt = std::string( ".osgearth_" ) + driver;
    osg::ref_ptr<osg::Object> object = osgDB::readRefObjectFile( driverExt, dbopt.get() );
    source = dynamic_cast<TileSource*>( object.release() );
    if ( !source )
    {
        OE_INFO << LC << "Failed to load TileSource driver \"" << driver << "\"" << std::endl;
    }
    else
    {
        OE_DEBUG << LC << "Tile source Profile = " << (source->getProfile() ? source->getProfile()->toString() : "NULL") << std::endl;

        // apply an Override Profile if provided.
        if ( options.profile().isSet() )
        {
            const Profile* profile = Profile::create(*options.profile());
            if ( profile )
            {
                source->setProfile( profile );
            }
        }
    }

    return source.release();
}


//------------------------------------------------------------------------

const TileSourceOptions&
TileSourceDriver::getTileSourceOptions(const osgDB::Options* dbopt ) const
{
    static TileSourceOptions s_default;
    const void* data = dbopt->getPluginData(TILESOURCE_OPTIONS_TAG);
    return data ? *static_cast<const TileSourceOptions*>(data) : s_default;
}

const std::string
TileSourceDriver::getInterfaceName(const osgDB::Options* dbopt) const
{
    return dbopt->getPluginStringData(TILESOURCE_INTERFACE_TAG);
}
