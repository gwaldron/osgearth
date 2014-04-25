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
#include <osgEarthUtil/TMSPackager>
#include <osgEarthUtil/TMS>
#include <osgEarth/ImageUtils>
#include <osgEarth/ImageToHeightFieldConverter>
#include <osgEarth/TaskService>
#include <osgEarth/CacheEstimator>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/WriteFile>


#define LC "[TMSPackager] "

using namespace osgEarth::Util;
using namespace osgEarth;

#if 0
// A global mutex to make sure that we're not creating directories concurrently.
// If you don't do this you will get 
static OpenThreads::Mutex s_dirLock;


namespace
{
    struct CreateImageTileTask : public TaskRequest
    {
        CreateImageTileTask(TMSPackager* packager, ImageLayer* layer, const TileKey& key, const std::string& path, const std::string& extension, osgDB::Options* imageWriteOptions, bool keepEmpties, bool verbose)
        {
          _layer = layer;
          _key = key;
          _path = path;
          _extension = extension;
          _imageWriteOptions = imageWriteOptions;
          _keepEmptyImageTiles = keepEmpties;
          _verbose = verbose;
          _packager = packager;
        }

        virtual void operator()(ProgressCallback* progress )
        {
            bool tileOK = false;

            std::stringstream message;
            GeoImage image = _layer->createImage( _key );
            if ( image.valid() )
            {
                // check for empty:
                if ( !_keepEmptyImageTiles && ImageUtils::isEmptyImage(image.getImage()) )
                {                    
                    message << "Skipping empty tile " << _key.str();
                }
                else
                {
                    // convert to RGB if necessary
                    osg::ref_ptr<osg::Image> final = image.getImage();
                    if ( _extension == "jpg" && final->getPixelFormat() != GL_RGB )
                        final = ImageUtils::convertToRGB8( image.getImage() );

                    // dump it to disk
                    {
                        OpenThreads::ScopedLock< OpenThreads::Mutex > lk( s_dirLock );
                        osgDB::makeDirectoryForFile( _path );
                    }
                    tileOK = osgDB::writeImageFile( *final.get(), _path, _imageWriteOptions);

                    if ( !tileOK )
                    {
                        std::string errMsg = "Error writing image for tile " + _key.str();

                        OE_WARN << LC << errMsg << std::endl;

                        _packager->cancel(errMsg);
                    }

                    if ( _verbose )
                    {
                        if ( tileOK ) {
                            message << "Wrote tile " << _key.str() << " (" << _key.getExtent().toString() << ")";
                        }
                        else {
                            message << "Error writing image for tile " << _key.str();
                        }
                    }                   
                }
            }
            else
            {
                std::string errMsg = "Image creation failed for tile " + _key.str();

                OE_WARN << LC << errMsg << std::endl;

                _packager->cancel(errMsg);
            }

            _packager->incrementCompleted();
            if (_verbose)
            {
                _packager->reportProgress( message.str() );
            }
        }

    private:
        osg::ref_ptr<ImageLayer> _layer;
        TileKey _key;
        std::string _path;
        std::string _extension;
        osg::ref_ptr<osgDB::Options> _imageWriteOptions;
        bool _keepEmptyImageTiles;
        bool _verbose;
        TMSPackager* _packager;
    };


    struct CreateElevationTileTask : public TaskRequest
    {        
        CreateElevationTileTask(TMSPackager* packager, ElevationLayer* layer, const TileKey& key, const std::string& path, osgDB::Options* elevationWriteOptions, bool verbose)
        {
          _layer = layer;
          _key = key;
          _path = path;
          _elevationWriteOptions = elevationWriteOptions;
          _verbose = verbose;
          _packager = packager;
        }

        virtual void operator()(ProgressCallback* progress )
        {
            bool tileOK = false;
            
            std::stringstream message;

            GeoHeightField hf = _layer->createHeightField( _key );
            if ( hf.valid() )
            {      
                // convert the HF to an image
                ImageToHeightFieldConverter conv;
                osg::ref_ptr<osg::Image> image = conv.convert( hf.getHeightField(), _packager->getElevationPixelDepth() );

                // dump it to disk
                // attempt to create the output folder:
                {
                    OpenThreads::ScopedLock< OpenThreads::Mutex > lk( s_dirLock );
                    osgDB::makeDirectoryForFile( _path );
                }
				tileOK = osgDB::writeImageFile(*image.get(), _path, _elevationWriteOptions);

                if ( _verbose )
                {
                    if ( tileOK ) {
                        message << "Wrote tile " << _key.str() << " (" << _key.getExtent().toString() << ")";
                    }
                    else
                    {
                        message << "Error write tile " << _key.str();
                    }
                }              
            }
            _packager->incrementCompleted();
            if (_verbose)
            {
                _packager->reportProgress( message.str() );
            }
        }        

    private:
        osg::ref_ptr<ElevationLayer> _layer;
        TileKey _key;
        std::string _path;
		osg::ref_ptr<osgDB::Options> _elevationWriteOptions;
		bool _verbose;
        TMSPackager* _packager;
    };    
}


TMSPackager::TMSPackager(const Profile* outProfile, osgDB::Options* writeOptions) :
_outProfile         ( outProfile ),
_minLevel           ( 0 ),
_maxLevel           ( 99 ),
_verbose            ( false ),
_overwrite          ( false ),
_keepEmptyImageTiles( false ),
_subdivideSingleColorImageTiles ( false ),
_elevationPixelDepth(32),
_abortOnError       ( true ),
_writeOptions  (writeOptions),
_numAdded           ( 0 ),
_total              ( 0 ),
_completed          ( 0 ),
_errorMessage       ("")
{
    // Pre-load some extensions since the getReaderWriterForExtension function isn't threadsafe and can cause tiles to not be written.
    osgDB::Registry::instance()->getReaderWriterForExtension("png");
    osgDB::Registry::instance()->getReaderWriterForExtension("jpg");
    osgDB::Registry::instance()->getReaderWriterForExtension("tiff");    
}


void
TMSPackager::addExtent( const GeoExtent& extent )
{
    _extents.push_back(extent);
}


bool
TMSPackager::shouldPackageKey( const TileKey& key ) const
{
    // if there are no extent filters, or we're at a sufficiently low level, 
    // always package the key.
    if ( _extents.size() == 0 || key.getLevelOfDetail() <= 1 )
        return true;

    // check for intersection with one of the filter extents.
    for( std::vector<GeoExtent>::const_iterator i = _extents.begin(); i != _extents.end(); ++i )
    {
        if ( i->intersects( key.getExtent() ) )
            return true;
    }

    return false;
}


void
TMSPackager::packageImageTile(ImageLayer*                  layer,
                              const TileKey&               key,
                              const std::string&           rootDir,
                              const std::string&           extension,
                              TaskService*                 service,
                              unsigned&                    out_maxLevel )
{
    unsigned minLevel = layer->getImageLayerOptions().minLevel().isSet() ?
        *layer->getImageLayerOptions().minLevel() : 0;

    minLevel = osg::maximum( minLevel, _minLevel );
    
    int taskCount = 0;

    bool hasData = layer->getTileSource()->hasData( key );   

    if ( shouldPackageKey(key) )
    {        
        bool tileOK = false;
        bool isSingleColor = false;
        if ( key.getLevelOfDetail() >= minLevel && hasData )
        {
            OE_DEBUG << "Packaging key " << key.str() << std::endl;
            unsigned w, h;
            key.getProfile()->getNumTiles( key.getLevelOfDetail(), w, h );

            std::string path = Stringify() 
                << rootDir 
                << "/" << key.getLevelOfDetail() 
                << "/" << key.getTileX() 
                << "/" << h - key.getTileY() - 1
                << "." << extension;

            tileOK = osgDB::fileExists(path) && !_overwrite;
            if ( !tileOK )
            {
                CreateImageTileTask *task = new CreateImageTileTask(this, layer, key, path, extension, _writeOptions, _keepEmptyImageTiles, _verbose);                                
                service->add( task );                
                _numAdded++;
                tileOK = true;
            }
            else
            {
                if ( _verbose )
                {
                    OE_NOTICE << LC << "Tile " << key.str() << " already exists" << std::endl;
                }
            }

            // increment the maximum detected tile level:
            if ( tileOK && key.getLevelOfDetail() > out_maxLevel )
            {
                out_maxLevel = key.getLevelOfDetail();
            }
        }

        // see if subdivision should continue.
        unsigned lod = key.getLevelOfDetail();
        const ImageLayerOptions& options = layer->getImageLayerOptions();

        unsigned layerMaxLevel = (options.maxLevel().isSet()? *options.maxLevel() : 99);
        unsigned maxLevel = std::min(_maxLevel, layerMaxLevel);
        bool subdivide =            
            (lod < minLevel) ||
            (tileOK && lod < maxLevel);

        // subdivide if necessary:
        if ( (subdivide == true) && (isSingleColor == false) )
        {
            for( unsigned q=0; q<4; ++q )
            {                
                TileKey childKey = key.createChildKey(q);

                packageImageTile( layer, childKey, rootDir, extension, service, out_maxLevel );                
            }
        }
    }  
}


void
TMSPackager::packageElevationTile(ElevationLayer*               layer,
                                  const TileKey&                key,
                                  const std::string&            rootDir,
                                  const std::string&            extension,
                                  TaskService*                  service,
                                  unsigned&                     out_maxLevel)
{
    unsigned minLevel = layer->getElevationLayerOptions().minLevel().isSet() ?
        *layer->getElevationLayerOptions().minLevel() : 0;

    minLevel = osg::maximum( minLevel, _minLevel );    

    int taskCount = 0;

    bool hasData = layer->getTileSource()->hasData( key );

    if ( shouldPackageKey(key) )
    {
        bool tileOK = false;
        if ( key.getLevelOfDetail() >= minLevel && hasData )
        {            
            unsigned w, h;
            key.getProfile()->getNumTiles( key.getLevelOfDetail(), w, h );

            std::string path = Stringify() 
                << rootDir 
                << "/" << key.getLevelOfDetail() 
                << "/" << key.getTileX() 
                << "/" << h - key.getTileY() - 1
                << "." << extension;

            tileOK = osgDB::fileExists(path) && !_overwrite;
            if ( !tileOK )
            {                
				CreateElevationTileTask* task = new CreateElevationTileTask(this, layer, key, path, _writeOptions, _verbose);
                service->add( task );
                _numAdded++;
                tileOK = true;
            }
            else
            {
                if ( _verbose )
                {
                    OE_NOTICE << LC << "Tile " << key.str() << " already exists" << std::endl;
                }
            }

            // increment the maximum detected tile level:
            if ( tileOK && key.getLevelOfDetail() > out_maxLevel )
            {
                out_maxLevel = key.getLevelOfDetail();
            }
        }

        // see if subdivision should continue.
        unsigned lod = key.getLevelOfDetail();
        const ElevationLayerOptions& options = layer->getElevationLayerOptions();

        unsigned layerMaxLevel = (options.maxLevel().isSet()? *options.maxLevel() : 99);
        unsigned maxLevel = std::min(_maxLevel, layerMaxLevel);
        bool subdivide =
            (lod < minLevel ) ||
            (tileOK && lod < maxLevel);

        // subdivide if necessary:
        if ( subdivide )
        {            
            for( unsigned q=0; q<4; ++q )
            {
                TileKey childKey = key.createChildKey(q);                
                packageElevationTile( layer, childKey, rootDir, extension, service, out_maxLevel );
            }
        }        
    }    
}


TMSPackager::Result
TMSPackager::package(ImageLayer*        layer,
                     const std::string& rootFolder,
                     osgEarth::ProgressCallback* progress,
                     const std::string& overrideExtension )
{
    if (progress && progress->isCanceled())
        return Result();

     // Initialize the TaskService
    unsigned int maxSize = 1000;
    unsigned num = 2 * OpenThreads::GetNumberOfProcessors();
    _taskService = new osgEarth::TaskService("TMS Packager", num, maxSize);

    resetStats();

    _progress = progress;
    osg::Timer* timer = osg::Timer::instance();
    osg::Timer_t start_t = timer->tick();

    if ( !layer || !_outProfile.valid() )
        return Result( "Illegal null layer or profile" );

    // attempt to create the output folder:
    {
        OpenThreads::ScopedLock< OpenThreads::Mutex > lk( s_dirLock );
        osgDB::makeDirectory( rootFolder );
    }
    if ( !osgDB::fileExists( rootFolder ) )
        return Result( "Unable to create output folder" );

    // collect the root tile keys in preparation for packaging:
    std::vector<TileKey> rootKeys;
    _outProfile->getRootKeys( rootKeys );

    if ( rootKeys.size() == 0 )
        return Result( "Unable to calculate root key set" );

    // fetch one tile to see what the image size should be
    
    GeoImage testImage;
    for( std::vector<TileKey>::iterator i = rootKeys.begin(); i != rootKeys.end() && !testImage.valid(); ++i )
    {
        testImage = layer->createImage( *i );
    }

    if ( !testImage.valid() )
        return Result( "Unable to get a test image!" );

    // try to determine the image extension:
    std::string extension = overrideExtension;

    if ( extension.empty() && testImage.valid() )
    {
        extension = toLower( osgDB::getFileExtension( testImage.getImage()->getFileName() ) );
        if ( extension.empty() )
        {
            if ( ImageUtils::hasAlphaChannel(testImage.getImage()) )
            {
                extension = "png";
            }
            else
            {
                extension = "jpg";
            }
        }
    }

    // compute a mime type
    std::string mimeType;
    if ( extension == "png" )
        mimeType = "image/png";
    else if ( extension == "jpg" || extension == "jpeg" )
        mimeType = "image/jpeg";
    else if ( extension == "tif" || extension == "tiff" )
        mimeType = "image/tiff";
    else {
        OE_WARN << LC << "Unable to determine mime-type for extension \"" << extension << "\"" << std::endl;
    }

    if ( _verbose )
    {
        OE_NOTICE << LC << "MIME-TYPE = " << mimeType << ", Extension = " << extension << std::endl;
    }

    //Estimate the number of tiles
    _total = 0;    
    CacheEstimator est;
    est.setMinLevel( _minLevel );
    est.setMaxLevel( _maxLevel );
    est.setProfile( _outProfile ); 
    for (unsigned int i = 0; i < _extents.size(); i++)
    {                
        est.addExtent( _extents[ i ] );
    } 
    _total = est.getNumTiles();
    
    // package the tile hierarchy
    unsigned maxLevel = 0;
    for( std::vector<TileKey>::const_iterator i = rootKeys.begin(); i != rootKeys.end(); ++i )
    {
        packageImageTile( layer, *i, rootFolder, extension, _taskService, maxLevel );
    }

    // Set the total to the actual # that was added
    _total = _numAdded;

    // Send a poison pill to kill all the threads
    _taskService->add( new PoisonPill() );

    _taskService->waitforThreadsToComplete();

    osg::Timer_t end_t = timer->tick();
    double elapsed = osg::Timer::instance()->delta_s( start_t, end_t );
    OE_DEBUG << LC << "Packaging image layer\"" << layer->getName() << "\" complete. Seconds elapsed: " << elapsed << std::endl;

    // create the tile map metadata:
    osg::ref_ptr<TMS::TileMap> tileMap = TMS::TileMap::create(
        "",
        _outProfile.get(),
        extension,
        testImage.getImage()->s(),
        testImage.getImage()->t() );

    tileMap->setTitle( layer->getName() );
    tileMap->setVersion( "1.0.0" );
    tileMap->getFormat().setMimeType( mimeType );
    tileMap->generateTileSets( std::min(23u, maxLevel+1) );

    // write out the tilemap catalog:
    std::string tileMapFilename = osgDB::concatPaths(rootFolder, "tms.xml");
    TMS::TileMapReaderWriter::write( tileMap.get(), tileMapFilename );
    reportProgress("Completed");

    return _errorMessage.empty() ? Result() : Result(_errorMessage);
}


TMSPackager::Result
TMSPackager::package(ElevationLayer*    layer,
                     const std::string& rootFolder,
                     osgEarth::ProgressCallback* progress )
{
     // Initialize the TaskService
    unsigned int maxSize = 1000;
    unsigned num = 2 * OpenThreads::GetNumberOfProcessors();
    _taskService = new osgEarth::TaskService("TMS Packager", num, maxSize);

    if (progress && progress->isCanceled())
        return Result();

    resetStats();

    osg::Timer* timer = osg::Timer::instance();
    osg::Timer_t start_t = timer->tick();

    if ( !layer || !_outProfile.valid() )
        return Result( "Illegal null layer or profile" );

    // attempt to create the output folder:
    // attempt to create the output folder:
    {
        OpenThreads::ScopedLock< OpenThreads::Mutex > lk( s_dirLock );
        osgDB::makeDirectory( rootFolder );
    }
    if ( !osgDB::fileExists( rootFolder ) )
        return Result( "Unable to create output folder" );

    // collect the root tile keys in preparation for packaging:
    std::vector<TileKey> rootKeys;
    _outProfile->getRootKeys( rootKeys );

    if ( rootKeys.size() == 0 )
        return Result( "Unable to calculate root key set" );

    std::string extension = "tif", mimeType = "image/tiff";
    if ( _verbose )
    {
        OE_NOTICE << LC << "MIME-TYPE = " << mimeType << ", Extension = " << extension << std::endl;
    }

    // fetch one tile to see what the tile size will be
    GeoHeightField testHF;
    for( std::vector<TileKey>::iterator i = rootKeys.begin(); i != rootKeys.end() && !testHF.valid(); ++i )
    {
        testHF = layer->createHeightField( *i );
    }

    if ( !testHF.valid() )
        return Result( "Unable to determine heightfield size" );

    //Estimate the number of tiles
    _total = 0;    
    CacheEstimator est;
    est.setMinLevel( _minLevel );
    est.setMaxLevel( _maxLevel );
    est.setProfile( _outProfile ); 
    for (unsigned int i = 0; i < _extents.size(); i++)
    {                
        est.addExtent( _extents[ i ] );
    } 
    _total = est.getNumTiles();

    // package the tile hierarchy
    unsigned maxLevel = 0;
    for( std::vector<TileKey>::const_iterator i = rootKeys.begin(); i != rootKeys.end(); ++i )
    {
        packageElevationTile( layer, *i, rootFolder, extension, _taskService, maxLevel );
    }

    // Set the total to the actual # that was added
    _total = _numAdded;

    // Send a poison pill to kill all the threads
    _taskService->add( new PoisonPill() );

    _taskService->waitforThreadsToComplete();

    reportProgress( "Completed"); 


    // create the tile map metadata:
    osg::ref_ptr<TMS::TileMap> tileMap = TMS::TileMap::create(
        "",
        _outProfile.get(),
        extension,
        testHF.getHeightField()->getNumColumns(),
        testHF.getHeightField()->getNumRows() );

    tileMap->setTitle( layer->getName() );
    tileMap->setVersion( "1.0.0" );
    tileMap->getFormat().setMimeType( mimeType );
    tileMap->generateTileSets( std::min(23u, maxLevel+1) );

    // write out the tilemap catalog:
    std::string tileMapFilename = osgDB::concatPaths(rootFolder, "tms.xml");
    TMS::TileMapReaderWriter::write( tileMap.get(), tileMapFilename );

    osg::Timer_t end_t = timer->tick();
    double elapsed = osg::Timer::instance()->delta_s( start_t, end_t );
    OE_DEBUG << LC << "Packaging elevation layer \"" << layer->getName() << "\" complete. Seconds elapsed: " << elapsed << std::endl;

    return _errorMessage.empty() ? Result() : Result(_errorMessage);
}

void TMSPackager::resetStats()
{
    _total = 0;
    _completed.exchange( 0 );
    _numAdded = 0;
}

void TMSPackager::incrementCompleted()
{    
    ++_completed;    
}

void TMSPackager::reportProgress( const std::string& message )
{
    if ( _progress.valid() )
    {     
        OpenThreads::ScopedLock< OpenThreads::Mutex > lock( _mutex );
        _progress->reportProgress(_completed, _total, message );

        if ( _progress->isCanceled() )
            cancel();
    }
}

void TMSPackager::cancel(const std::string& msg)
{
    _errorMessage = msg;
    _taskService->cancelAll();
}
#else


TMSPackager::TMSPackager():
_visitor(new TileVisitor()),
    _extension("jpg"),
    _destination("out"),
    _elevationPixelDepth(32)
{
}

const std::string& TMSPackager::getDestination() const
{
    return _destination;
}

void TMSPackager::setDestination( const std::string& destination)
{
    _destination = destination;
}

const std::string& TMSPackager::getExtension() const
{
    return _extension;
}

void TMSPackager::setExtension( const std::string& extension)
{
    _extension = extension;
}

 void TMSPackager::setElevationPixelDepth(unsigned value)
 {
     _elevationPixelDepth = value;
 }

 unsigned TMSPackager::getElevationPixelDepth() const
 {
     return _elevationPixelDepth;
 }

osgDB::Options* TMSPackager::getOptions() const
{
    return _writeOptions.get();
}

void TMSPackager::setWriteOptions( osgDB::Options* options )
{
    _writeOptions = options;
}

TileVisitor* TMSPackager::getTileVisitor() const
{
    return _visitor;
}

void TMSPackager::setVisitor(TileVisitor* visitor)
{
    _visitor = visitor;
}    

void TMSPackager::run( TerrainLayer* layer,  const Profile* profile  )
{ 
    _handler = new WriteTMSTileHandler(layer, _destination, _extension);
    _handler->setElevationPixelDepth( _elevationPixelDepth );
    _handler->setOptions(_writeOptions.get());
    _visitor->setTileHandler( _handler );
    _visitor->run( profile );    
}

void TMSPackager::writeXML( TerrainLayer* layer, const Profile* profile)
{
     // create the tile map metadata:
    osg::ref_ptr<TMS::TileMap> tileMap = TMS::TileMap::create(
        "",
        profile,
        _extension,
        _handler->getWidth(),
        _handler->getHeight()
        );

    std::string mimeType;
    if ( _extension == "png" )
        mimeType = "image/png";
    else if ( _extension == "jpg" || _extension == "jpeg" )
        mimeType = "image/jpeg";
    else if ( _extension == "tif" || _extension == "tiff" )
        mimeType = "image/tiff";
    else {
        OE_WARN << LC << "Unable to determine mime-type for extension \"" << _extension << "\"" << std::endl;
    }


    unsigned int maxLevel = _handler->getMaxLevel();
    tileMap->setTitle( layer->getName() );
    tileMap->setVersion( "1.0.0" );
    tileMap->getFormat().setMimeType( mimeType );
    tileMap->generateTileSets( std::min(23u, maxLevel+1) );
    

    // write out the tilemap catalog:
    std::string tileMapFilename = osgDB::concatPaths( osgDB::concatPaths(_destination, toLegalFileName( layer->getName() )), "tms.xml");
    TMS::TileMapReaderWriter::write( tileMap.get(), tileMapFilename );
}

#endif