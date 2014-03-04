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


namespace
{
    /**
     * Convert elapsed time in seconds to HH::MM:SS.S
     */
    // NOTE: Should be placed somewhere better.
    std::string toHMS( double elapsed )
    {
        using namespace std;
        std::ostringstream os;
        int hours = elapsed/3600;
        int minutes = (elapsed-hours*3600)/60;
        double seconds = (elapsed-hours*3600-minutes*60);
        os << setw(2) << setfill('0') << hours << ":";
        os << setw(2) << setfill('0') << minutes << ":"; 
        os << setw(4) << fixed << setprecision(1) << seconds;
        return os.str();
    }

    /**
     * Common data required to process a tile.
     * NOTE: Templated to allow the layer type to be variable.
     */
    template< class LayerType >
    struct LayerProcessingData : public osg::Referenced
    {
        osg::ref_ptr<LayerType>      layer;
        std::string                  rootDir;
        std::string                  extension;
        osg::ref_ptr<osgDB::Options> imageWriteOptions;
        TMSPackager*                 packager;
        std::vector<GeoExtent>*      extents;
        unsigned                     minLevel;
        bool                         keepEmptyImageTiles;
        bool                         divideSingleColor;
        bool                         overwrite;
        bool                         verbose;
    };

    /**
     * Common tile processing code.
     * NOTE: This holds the logical results of processing the tile
     *   such as tileOK and isSingleColor.
     */
    template< class LayerType >
    struct CreateTileTask : public TaskRequest
    {
        CreateTileTask( LayerProcessingData<LayerType>* layerData, const TileKey& key )
            : _layerData( layerData )
        {
            _key = key;
            _hasData = _layerData->layer->getTileSource()->hasData( _key );
            _tileOK = false;
            _isSingleColor = false;

            // Create the text for the key.  Use the "reversed" key to be 
            // consistent with what is given as the root key and what will
            // be written out.
            unsigned w, h;
            _key.getProfile()->getNumTiles( _key.getLevelOfDetail(), w, h );
            unsigned z = _key.getLevelOfDetail();
            unsigned x = _key.getTileX();
            unsigned y = h - _key.getTileY() - 1;
            _keystr = Stringify() << z << "/" << x << "/" << y;

            // Create file in which to store the tile
            _path = Stringify() 
                << _layerData->rootDir << "/" << _keystr << "." << _layerData->extension;
        }

        bool isSingleColor() const { return _isSingleColor; }

        bool isTileOK() const { return _tileOK; }

        bool hasData() const { return _hasData; }

        const TileKey& getKey() const { return _key; }

    protected:
        std::string getMessage()
        {
            std::ostringstream message;
            if ( _tileOK )
            {
                // TSD Just indicate the key.
                message << "Wrote tile " << _keystr;
            }
            else
            {
                message << "Error writing tile " << _keystr;
            }
            return message.str();
        }

        void finish( const std::string& message )
        {
            if ( _layerData->verbose )
            {
                _layerData->packager->reportProgress( message ); 
            }
            _layerData->packager->incrementCompleted();
        }

        bool inExtents() const
        {
            // if there are no extent filters, or we're at a sufficiently low level, 
            // always package the key.
            if ( _layerData->extents->size() == 0 || _key.getLevelOfDetail() <= 1 )
                return true;

            // check for intersection with one of the filter extents.
            for( std::vector<GeoExtent>::const_iterator i = _layerData->extents->begin(); i != _layerData->extents->end(); ++i )
            {
                if ( i->intersects( _key.getExtent() ) )
                    return true;
            }

            return false;
        }

        TileKey     _key;
        std::string _keystr;
        std::string _path;
        bool        _hasData;
        bool        _tileOK;
        bool        _isSingleColor;
        osg::ref_ptr< LayerProcessingData<LayerType> > _layerData;
    };


    struct CreateImageTileTask : public CreateTileTask<ImageLayer>
    {
        CreateImageTileTask( LayerProcessingData<ImageLayer>* layerData, const TileKey& key )
            : CreateTileTask( layerData, key )
        {
        }

        virtual void operator()(ProgressCallback* progress )
        {            
            std::string message;

            // Check if we should generate an image
            _tileOK = !_layerData->overwrite && osgDB::fileExists(_path);
            bool inLodRange = _key.getLevelOfDetail() >= _layerData->minLevel;
            bool generateImage = inExtents() && _hasData && !_tileOK && inLodRange;
            if ( _tileOK && !_layerData->divideSingleColor )
            {
                // If the tile exists, we could read it and test for single
                // color and set _isSingleColor here.
            }
            if ( !generateImage ) 
            {
                message = Stringify() << "Skipping existing tile " << _keystr;
                finish( message );
                return;
            }

            _layerData->packager->incrementTileCount();
            GeoImage image = _layerData->layer->createImage( _key );
            if ( image.valid() )
            {
                // check for single color image
                if ( !_layerData->divideSingleColor ) 
                    _isSingleColor = ImageUtils::isSingleColorImage( image.getImage() );

                // check for empty:
                if ( !_layerData->keepEmptyImageTiles && ImageUtils::isEmptyImage(image.getImage()) )
                {                    
                    message = Stringify() << "Skipping empty tile " << _keystr;
                }
                else
                {
                    // convert to RGB if necessary
                    osg::ref_ptr<osg::Image> final = image.getImage();
                    if ( _layerData->extension == "jpg" && final->getPixelFormat() != GL_RGB )
                        final = ImageUtils::convertToRGB8( image.getImage() );

                    // dump it to disk
                    osgDB::makeDirectoryForFile( _path );
                    _tileOK = osgDB::writeImageFile( *final.get(), _path, _layerData->imageWriteOptions);

                    if ( _layerData->verbose ) message = getMessage();          
                }
            }
            finish( message );
        }
    };


    struct CreateElevationTileTask : public CreateTileTask<ElevationLayer>
    {        
        CreateElevationTileTask( LayerProcessingData<ElevationLayer>* layerData, const TileKey& key )
            : CreateTileTask( layerData, key )
        {
        }

        virtual void operator()(ProgressCallback* progress )
        {
            std::string message;

            // Check if we should generate a image
            _tileOK = !_layerData->overwrite && osgDB::fileExists(_path);
            bool inLodRange = _key.getLevelOfDetail() >= _layerData->minLevel;
            bool generateImage = inExtents() && _hasData && !_tileOK && inLodRange;
            if ( _tileOK && !_layerData->divideSingleColor )
            {
                // If the tile exists, we could read it and test for single
                // color and set _isSingleColor here.
            }
            if ( !generateImage ) 
            {
                message = Stringify() << "Skipping existing tile " << _keystr;
                finish( message );
                return;
            }

            _layerData->packager->incrementTileCount();
            GeoHeightField hf = _layerData->layer->createHeightField( _key );
            if ( hf.valid() )
            {
                // convert the HF to an image
                ImageToHeightFieldConverter conv;
                osg::ref_ptr<osg::Image> image = conv.convert( hf.getHeightField() );

                if ( !_layerData->divideSingleColor ) _isSingleColor = ImageUtils::isSingleColorImage( image );

                // dump it to disk
                osgDB::makeDirectoryForFile( _path );
                _tileOK = osgDB::writeImageFile( *image.get(), _path );

                if ( _layerData->verbose ) message = getMessage();         
            }
            finish( message );
        }        
    };  

    // Helper function to create the task and schedule an image creation task.
    TaskRequest* createImageAsync( 
        const TileKey& key, 
        LayerProcessingData<ImageLayer>* layerData, 
        TaskService* service )
    {
        osg::ref_ptr<CreateImageTileTask> task = new CreateImageTileTask( layerData, key );
        service->add( task );
        return task.release();
    }

    // Helper function to create the task and schedule an elevation tile creation task.
    TaskRequest* createElevationTileAsync( 
        const TileKey& key, 
        LayerProcessingData<ElevationLayer>* layerData, 
        TaskService* service )
    {
        osg::ref_ptr<CreateElevationTileTask> task = new CreateElevationTileTask( layerData, key );
        service->add( task );
        return task.release();
    }

    // Help function to find completed tasks
    bool isComplete( const osg::ref_ptr<TaskRequest>& task )
    {
        return task->isCompleted();
    }
}


TMSPackager::TMSPackager(const Profile* outProfile, osgDB::Options* imageWriteOptions) :
_outProfile         ( outProfile ),
_maxLevel           ( 99 ),
_minLevel           ( 0 ),
_verbose            ( false ),
_overwrite          ( false ),
_keepEmptyImageTiles( false ),
_subdivideSingleColorImageTiles ( false ),
_abortOnError       ( true ),
_imageWriteOptions  (imageWriteOptions),
_numAdded           ( 0 ),
_total              ( 0 ),
_completed          ( 0 ),
_completeTasks      ( 0 )
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
TMSPackager::subdivideTile( const TileKey& key, bool tileOK, bool singleColor )
{
    // Subdivide the tile if needed
    // NOTE: singleColor is always false if subdivideSingleColorImages is false.
    unsigned lod = key.getLevelOfDetail();
    bool inExtents = shouldPackageKey( key );
    if ( inExtents && ( (tileOK && !singleColor && lod < _maxLevelForLayer) || 
        (lod < _minLevelForLayer && _minLevelForLayer < _maxLevelForLayer ) ) )
    {
        // Add keys to queue
        for ( unsigned q=0; q<4; ++q )
        {
            TileKey childKey = key.createChildKey(q);
            _key_queue.insert( childKey );
        }
    }
}


void
TMSPackager::packageImageTile(ImageLayer*                  layer,
                              const TileKey&               key,
                              const std::string&           rootDir,
                              const std::string&           extension,
                              TaskService*                 service,
                              unsigned&                    out_maxLevel )
{
    // Setup layer processing data
    osg::ref_ptr< LayerProcessingData<ImageLayer> > layerData = new LayerProcessingData<ImageLayer>;
    layerData->layer = layer;
    layerData->rootDir = rootDir;
    layerData->extension = extension;
    layerData->imageWriteOptions = _imageWriteOptions;
    layerData->packager = this;
    layerData->extents = &_extents;
    layerData->minLevel = _minLevelForLayer;
    layerData->keepEmptyImageTiles = _keepEmptyImageTiles;
    layerData->divideSingleColor = _subdivideSingleColorImageTiles;
    layerData->overwrite = _overwrite;
    layerData->verbose = _verbose;

    // Add initial key to key queue
    _key_queue.insert( key );

    // Process all keys
    if ( _subdivideSingleColorImageTiles )
    {
        while ( !_key_queue.empty() )
        {
            TileKey key = *_key_queue.begin();
            _key_queue.erase( _key_queue.begin() );
            osg::ref_ptr<TaskRequest> task = createImageAsync( key, layerData, service );
            osg::ref_ptr<CreateImageTileTask> tile = dynamic_cast<CreateImageTileTask*>(task.get());
            subdivideTile( tile->getKey(), tile->hasData(), tile->isSingleColor() );
        }
    }

    // Don't divide single color tiles
    else
    {
        // Continue processing all keys and any remaining tasks
        while ( !_key_queue.empty() || !_task_list.empty() )
        {
            // Start generating tiles for queued keys
            while ( !_key_queue.empty() && _task_list.size() < 128 )
            {
                TileKey key = *_key_queue.begin();
                _task_list.push_back( createImageAsync( key, layerData, service ) );
                _key_queue.erase( _key_queue.begin() );
            }

            // Wait for a completed task
            if ( !_task_list.empty() )
            {
                _completeTasks.acquire();

                // Spin until we find the completed task
                osg::ref_ptr<CreateImageTileTask> tile;
                std::vector<osg::ref_ptr<TaskRequest>>::iterator last = _task_list.end();
                while ( !tile.valid() )
                {
                    std::vector<osg::ref_ptr<TaskRequest>>::iterator task;
                    task = std::find_if( _task_list.begin(), _task_list.end(), isComplete );
                    if ( task == last ) OpenThreads::Thread::YieldCurrentThread();
                    else
                    {
                        tile = dynamic_cast<CreateImageTileTask*>(task->get());
                        _task_list.erase( task );
                    }
                }

                // Add the four child keys to the key_queue.
                subdivideTile( tile->getKey(), tile->isTileOK(), tile->isSingleColor() );
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
    // Setup layer processing data
    osg::ref_ptr< LayerProcessingData<ElevationLayer> > layerData = new LayerProcessingData<ElevationLayer>;
    layerData->layer = layer;
    layerData->rootDir = rootDir;
    layerData->extension = extension;
    layerData->imageWriteOptions = _imageWriteOptions;
    layerData->packager = this;
    layerData->extents = &_extents;
    layerData->minLevel = _minLevelForLayer;
    layerData->keepEmptyImageTiles = _keepEmptyImageTiles;
    layerData->divideSingleColor = _subdivideSingleColorImageTiles;
    layerData->overwrite = _overwrite;
    layerData->verbose = _verbose;

    // Add initial key to key queue
    _key_queue.insert( key );

    // Process all keys
    if ( _subdivideSingleColorImageTiles )
    {
        while ( !_key_queue.empty() )
        {
            TileKey key = *_key_queue.begin();
            _key_queue.erase( _key_queue.begin() );
            osg::ref_ptr<TaskRequest> task = createElevationTileAsync( key, layerData, service );
            osg::ref_ptr<CreateElevationTileTask> tile = dynamic_cast<CreateElevationTileTask*>(task.get());
            subdivideTile( tile->getKey(), tile->hasData(), tile->isSingleColor() );
        }
    }

    // Don't divide single color tiles
    else
    {
        // Continue processing all keys and any remaining tasks
        while ( !_key_queue.empty() || !_task_list.empty() )
        {
            // Start generating tiles for queued keys
            while ( !_key_queue.empty() && _task_list.size() < 128 )
            {
                TileKey key = *_key_queue.begin();
                _task_list.push_back( createElevationTileAsync( key, layerData, service ) );
                _key_queue.erase( _key_queue.begin() );
            }

            // Wait for a completed task
            if ( !_task_list.empty() )
            {
                _completeTasks.acquire();

                // Spin until we find the completed task
                osg::ref_ptr<CreateElevationTileTask> tile;
                std::vector<osg::ref_ptr<TaskRequest>>::iterator last = _task_list.end();
                while ( !tile.valid() )
                {
                    std::vector<osg::ref_ptr<TaskRequest>>::iterator task;
                    task = std::find_if( _task_list.begin(), _task_list.end(), isComplete );
                    if ( task == last ) OpenThreads::Thread::YieldCurrentThread();
                    else
                    {
                        tile = dynamic_cast<CreateElevationTileTask*>(task->get());
                        _task_list.erase( task );
                    }
                }

                // Add the four child keys to the key_queue.
                subdivideTile( tile->getKey(), tile->isTileOK(), tile->isSingleColor() );
            }
        }
    }
}


TMSPackager::Result
TMSPackager::package(ImageLayer*        layer,
                     const std::string& rootFolder,
                     const TileKey&     rootKey,
                     osgEarth::ProgressCallback* progress,
                     const std::string& overrideExtension )
{
    resetStats();

    _progress = progress;
    osg::Timer* timer = osg::Timer::instance();
    osg::Timer_t start_t = timer->tick();

    if ( !layer || !_outProfile.valid() )
        return Result( "Illegal null layer or profile" );

    // attempt to create the output folder:
    osgDB::makeDirectory( rootFolder );
    if ( !osgDB::fileExists( rootFolder ) )
        return Result( "Unable to create output folder" );

    // collect the root tile keys in preparation for packaging:
    std::vector<TileKey> rootKeys;
    if ( !rootKey.valid() )
    {
        _outProfile->getRootKeys( rootKeys );

        if ( rootKeys.size() == 0 )
            return Result( "Unable to calculate root key set" );
    }
    else
    {
        rootKeys.push_back( rootKey );
    }

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

    // Determine min/max levels to process for this layer
    const ImageLayerOptions& options = layer->getImageLayerOptions();
    unsigned minLevel = options.minLevel().isSet() ? *options.minLevel() : 0;
    _minLevelForLayer = std::max( minLevel, _minLevel );

    // The user may not have set a maxLevel or maxDataLevel, so we need to
    // query the tile source to see what it supports.
    unsigned lod = 0;
    unsigned sourceMaxLevel = 0;
    while ( layer->getTileSource()->hasDataAtLOD(lod) && lod < 99 ) 
    {
        sourceMaxLevel = lod;
        ++lod;
    }

    unsigned layerMaxLevel = (options.maxLevel().isSet()? *options.maxLevel() : 99);
    unsigned maxDataLevel = (options.maxDataLevel().isSet() ? *options.maxDataLevel() : 99);
    _maxLevelForLayer = std::min( _maxLevel, std::min( maxDataLevel, layerMaxLevel ));
    _maxLevelForLayer = std::min( sourceMaxLevel, _maxLevelForLayer );

    //Estimate the number of tiles
    // This doesn't make sense if not dividing single color tiles
    _total = 0;    
    if ( _subdivideSingleColorImageTiles )
    {
        CacheEstimator est;
        est.setMinLevel( _minLevelForLayer );
        est.setMaxLevel( _maxLevelForLayer );
        est.setProfile( _outProfile ); 
        for (unsigned int i = 0; i < _extents.size(); i++)
        {                
            est.addExtent( _extents[ i ] );
        } 
        _total = est.getNumTiles();
    }
    
    unsigned num = 2 * OpenThreads::GetNumberOfProcessors();
    osg::ref_ptr<osgEarth::TaskService> taskService = new osgEarth::TaskService("TMS Packager", num, 2*num);

    // package the tile hierarchy
    unsigned maxLevel = 0;
    for( std::vector<TileKey>::const_iterator i = rootKeys.begin(); i != rootKeys.end(); ++i )
    {
        packageImageTile( layer, *i, rootFolder, extension, taskService, maxLevel );
    }

    // Send a poison pill to kill all the threads
    taskService->add( new PoisonPill() );

    taskService->waitforThreadsToComplete();

    // Set the total to the actual # that was added
    _total = _numAdded;

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

    osg::Timer_t end_t = timer->tick();
    double elapsed = osg::Timer::instance()->delta_s( start_t, end_t );
    OE_NOTICE << LC << "Completed image layer\"" << layer->getName() << "\" in " << toHMS(elapsed) << std::endl;

    Result result( _total );
    resetStats();

    return result;
}


TMSPackager::Result
TMSPackager::package(ElevationLayer*    layer,
                     const std::string& rootFolder,
                     const TileKey&     rootKey,
                     osgEarth::ProgressCallback* progress )
{
    resetStats();

    _progress = progress;
    osg::Timer* timer = osg::Timer::instance();
    osg::Timer_t start_t = timer->tick();

    if ( !layer || !_outProfile.valid() )
        return Result( "Illegal null layer or profile" );

    // attempt to create the output folder:
    osgDB::makeDirectory( rootFolder );
    if ( !osgDB::fileExists( rootFolder ) )
        return Result( "Unable to create output folder" );

    // collect the root tile keys in preparation for packaging:
    std::vector<TileKey> rootKeys;
    if ( !rootKey.valid() )
    {
        _outProfile->getRootKeys( rootKeys );

        if ( rootKeys.size() == 0 )
            return Result( "Unable to calculate root key set" );
    }
    else
    {
        rootKeys.push_back( rootKey );
    }

    // fetch one tile to see what the tile size will be
    GeoHeightField testHF;
    for( std::vector<TileKey>::iterator i = rootKeys.begin(); i != rootKeys.end() && !testHF.valid(); ++i )
    {
        testHF = layer->createHeightField( *i );
    }
    if ( !testHF.valid() )
        return Result( "Unable to determine heightfield size" );

    std::string extension = "tif", mimeType = "image/tiff";
    if ( _verbose )
    {
        OE_NOTICE << LC << "MIME-TYPE = " << mimeType << ", Extension = " << extension << std::endl;
    }

    // Determine min/max levels to process for this layer
    const ImageLayerOptions& options = layer->getElevationLayerOptions();
    unsigned minLevel = options.minLevel().isSet() ? *options.minLevel() : 0;
    _minLevelForLayer = std::max( minLevel, _minLevel );

    // The user may not have set a maxLevel or maxDataLevel, so we need to
    // query the tile source to see what it supports.
    unsigned lod = 0;
    unsigned sourceMaxLevel = 0;
    while ( layer->getTileSource()->hasDataAtLOD(lod) && lod < 99 ) 
    {
        sourceMaxLevel = lod;
        ++lod;
    }

    unsigned layerMaxLevel = (options.maxLevel().isSet()? *options.maxLevel() : 99);
    unsigned maxDataLevel = (options.maxDataLevel().isSet() ? *options.maxDataLevel() : 99);
    _maxLevelForLayer = std::min( _maxLevel, std::min( maxDataLevel, layerMaxLevel ));
    _maxLevelForLayer = std::min( sourceMaxLevel, _maxLevelForLayer );

    //Estimate the number of tiles
    // This doesn't make sense if not dividing single color tiles
    _total = 0;    
    if ( _subdivideSingleColorImageTiles )
    {
        CacheEstimator est;
        est.setMinLevel( _minLevelForLayer );
        est.setMaxLevel( _maxLevelForLayer );
        est.setProfile( _outProfile ); 
        for (unsigned int i = 0; i < _extents.size(); i++)
        {                
            est.addExtent( _extents[ i ] );
        } 
        _total = est.getNumTiles();
    }

    unsigned num = 2 * OpenThreads::GetNumberOfProcessors();
    osg::ref_ptr<osgEarth::TaskService> taskService = new osgEarth::TaskService("TMS Elevation Packager", num, 2*num);

    // package the tile hierarchy
    unsigned maxLevel = 0;
    for( std::vector<TileKey>::const_iterator i = rootKeys.begin(); i != rootKeys.end(); ++i )
    {
        packageElevationTile( layer, *i, rootFolder, extension, taskService, maxLevel );
    }

    // Send a poison pill to kill all the threads
    taskService->add( new PoisonPill() );

    taskService->waitforThreadsToComplete();

    // Set the total to the actual # that was added
    _total = _numAdded;

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

    reportProgress( "Completed"); 

    osg::Timer_t end_t = timer->tick();
    double elapsed = osg::Timer::instance()->delta_s( start_t, end_t );
    OE_NOTICE << LC << "Completed elevation layer \"" << layer->getName() << "\" in " << toHMS(elapsed) << std::endl;

    Result result( _total );
    resetStats();

    return result;
}

void TMSPackager::resetStats()
{
    _total = 0;
    _completed.exchange( 0 );
    _numAdded.exchange( 0 );
}

void TMSPackager::incrementCompleted()
{    
    ++_completed;  

    // Let packager know that a task is completed.
    _completeTasks.release();
}

void TMSPackager::reportProgress( const std::string& message )
{
    if ( _progress.valid() )
    {     
        OpenThreads::ScopedLock< OpenThreads::Mutex > lock( _mutex );
        _progress->reportProgress(_completed, _total, message );
    }
}
