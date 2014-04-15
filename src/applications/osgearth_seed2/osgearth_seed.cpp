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

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/WriteFile>

#include <osg/io_utils>
#include <osgEarth/ImageUtils>

#include <osgEarth/Common>
#include <osgEarth/Map>
#include <osgEarth/MapFrame>
#include <osgEarth/Cache>
#include <osgEarth/CacheEstimator>
#include <osgEarth/CacheSeed>
#include <osgEarth/MapNode>
#include <osgEarth/Registry>
#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>
#include <osgEarth/ImageToHeightFieldConverter>
#include <osgEarth/FileUtils>
#include <osgEarthUtil/TMS>

#include <osgEarth/TaskService>

#include <osgEarth/TileVisitor>

#include <iostream>
#include <sstream>
#include <iterator>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Drivers;

#include <zmq.h>

#define LC "[osgearth_cache] "


//  Receive 0MQ string from socket and convert into C string
//  Caller must free returned string. Returns NULL if the context
//  is being terminated.
static char *
s_recv (void *socket) {
    char buffer [1024];
    int size = zmq_recv(socket, buffer, 1023, 0);
    if (size == -1)
        return NULL;
    if (size > 1023)
        size = 1023;
    buffer [size] = 0;
    return strdup (buffer);
}

//  Convert C string to 0MQ string and send to socket
static int
s_send (void *socket, const char *string) {
    int size = zmq_send (socket, string, strlen (string), 0);
    return size;
}


// A global mutex to make sure that we're not creating directories concurrently.
// If you don't do this you will get 
static OpenThreads::Mutex s_dirLock;


/******************************************************************************************************************/
/**
 * Simple example of a single threaded cache seeder.  It's a TileVisitor that calls into the CacheTileHandler
 */
class LayerSeeder : public TileVisitor
{
public:
    LayerSeeder(TerrainLayer* layer ):
      TileVisitor( new CacheTileHandler( layer ) )
    {
    }     
};

/******************************************************************************************************************/

/**
 * A TileHandler that packages a tile in a TMS structure
 */
class PackageTileHandler : public TileHandler
{
public:
    PackageTileHandler(TerrainLayer* layer, const std::string& destination, const std::string& extension="jpg" ):
      _layer( layer ),
          _destination( destination ),
          _extension(extension),
          _width(0),
          _height(0)      
      {
      }

      const std::string& getExtension() const { return _extension; }
      const std::string& getDestination() const { return _destination;}
      unsigned int getWidth() const { return _width;}
      unsigned int getHeight() const { return _height; }
      TerrainLayer* getLayer() { return _layer; }

      std::string getPathForTile( const TileKey &key )
      {
          std::string layerFolder = toLegalFileName( _layer->getName() );         
          unsigned w, h;
          key.getProfile()->getNumTiles( key.getLevelOfDetail(), w, h );         

          return Stringify() 
              << _destination 
              << "/" << layerFolder
              << "/" << key.getLevelOfDetail() 
              << "/" << key.getTileX() 
              << "/" << h - key.getTileY() - 1
              << "." << _extension;
      }


      virtual bool handleTile( const TileKey& key )
      {
          // Get the path to write to
          std::string path = getPathForTile( key );

          // attempt to create the output folder:        
          {
              OpenThreads::ScopedLock< OpenThreads::Mutex > lk( s_dirLock );
              osgDB::makeDirectoryForFile( path );       
          }

          ImageLayer* imageLayer = dynamic_cast< ImageLayer* >( _layer.get() );
          ElevationLayer* elevationLayer = dynamic_cast< ElevationLayer* >( _layer.get() );

          if (imageLayer)
          {                
              GeoImage geoImage = imageLayer->createImage( key );

              if (geoImage.valid())
              {                
                  if (_width == 0 || _height == 0)
                  {
                      _width = geoImage.getImage()->s();
                      _height = geoImage.getImage()->t();
                  }
                  // OE_NOTICE << "Created image for " << key.str() << std::endl;
                  osg::ref_ptr< const osg::Image > final = geoImage.getImage();

                  // convert to RGB if necessary            
                  if ( _extension == "jpg" && final->getPixelFormat() != GL_RGB )
                  {
                      final = ImageUtils::convertToRGB8( final );
                  }                
                  return osgDB::writeImageFile(*final, path);
              }            
          }
          else if (elevationLayer )
          {
              GeoHeightField hf = elevationLayer->createHeightField( key );
              if (hf.valid())
              {
                  // convert the HF to an image
                  ImageToHeightFieldConverter conv;
                  osg::ref_ptr< osg::Image > image = conv.convert( hf.getHeightField(), 32 );				
                  return osgDB::writeImageFile(*image.get(), path);
              }            
          }
          return false;        
      }  

protected:
    std::string _extension;
    std::string _destination;
    osg::ref_ptr< TerrainLayer > _layer;
    unsigned int _width;
    unsigned int _height;
};

/******************************************************************************************************************/

/**
 * Simple TMS Packager.  It's a TileVisitor that calls into a PackageTileHandler.  
 * It also has an override run method that dumps out a tms.xml file as well.
 */
class TMSPackager : public TileVisitor
{    
public:
     TMSPackager(TerrainLayer* layer, const std::string& destination, const std::string& extension="jpg" ):
      TileVisitor( new PackageTileHandler( layer, destination, extension ) )
    {
    }   
      
    virtual void run(const Profile* mapProfile)
    {
        // Run through the tiles
        TileVisitor::run( mapProfile );

        PackageTileHandler* h = static_cast<PackageTileHandler*>( _tileHandler.get() );

        // Now write out the TMS XML.
        // create the tile map metadata:
        osg::ref_ptr<TMS::TileMap> tileMap = TMS::TileMap::create(
            "",
            mapProfile,
            h->getExtension(),
            h->getWidth(),
            h->getHeight()
            );

        // compute a mime type
        std::string mimeType;
        std::string extension = h->getExtension();
        if ( extension == "png" )
            mimeType = "image/png";
        else if ( extension == "jpg" || extension == "jpeg" )
            mimeType = "image/jpeg";
        else if ( extension == "tif" || extension == "tiff" )
            mimeType = "image/tiff";
        else {
            OE_WARN << LC << "Unable to determine mime-type for extension \"" << extension << "\"" << std::endl;
        }

        tileMap->setTitle( h->getLayer()->getName() );
        tileMap->setVersion( "1.0.0" );
        tileMap->getFormat().setMimeType( mimeType );
        // TODO:  Compute max level
        tileMap->generateTileSets( 19 );

        std::string layerFolder = toLegalFileName( h->getLayer()->getName() );

        // write out the tilemap catalog:
        std::string tileMapFilename = osgDB::concatPaths(h->getDestination() + "/" + layerFolder, "tms.xml");
        TMS::TileMapReaderWriter::write( tileMap.get(), tileMapFilename );      

    }    

};


class ProcessThread : public OpenThreads::Thread, public osg::Referenced
{
public:
    ProcessThread( const std::string& command ):
      _command( command )
    {
    }

    void run()
    {        
        system(_command.c_str());
        //OE_NOTICE << "Finished" << std::endl;
    }

    std::string _command;
};


class ZMQTileVisitor : public TileVisitor
{
public:
    ZMQTileVisitor():
      _sender(0),
      _context(0)
    {        
    }

    ~ZMQTileVisitor()
    {
        //TODO:  Close stuff.
        //zmq_close (_sender);
        //zmq_ctx_destroy (_context);
    }

    virtual void run(const Profile* mapProfile)
    {
        _context = zmq_ctx_new ();

        //  Socket to send messages on
        _sender = zmq_socket(_context, ZMQ_PUSH);
        //int hwm = 10000;
        //zmq_setsockopt( _sender, ZMQ_SNDHWM, &hwm, sizeof(hwm));
        zmq_bind (_sender, "tcp://*:5557");

        unsigned int numWorkers = 16;
        
        std::vector< osg::ref_ptr< ProcessThread > > processes;
        for (unsigned int i = 0; i < numWorkers; i++)
        {        
            ProcessThread* process = new ProcessThread("osgearth_cache2 --seed --client gdal_tiff.earth");
            process->start();            
            processes.push_back(process);
        }        

        // Wait for the threads to actually launch their processes and begin listening
        OpenThreads::Thread::microSleep( 30000000 );

               

        
        /*
        printf ("Press Enter when the workers are ready: ");
        getchar ();
        printf ("Sending tasks to workers…\n");        
        */

        
        // Produce the tiles
        TileVisitor::run( mapProfile );        
        
        
        bool isRunning = true;
        while (isRunning)
        {
            unsigned int numRunning = 0;
            isRunning = false;
            // Wait for all processes to die
            for (unsigned int i = 0; i < numWorkers; i++)
            {
                if (processes[i]->isRunning())
                {
                    numRunning++;
                    isRunning = true;
                    //break;
                }
                //processes[i]->join();
            }            

            if (isRunning)
            {
                OE_NOTICE << "Still have " << numRunning << " processes" << std::endl;
                s_send( _sender, "DIE");                
            }
            OpenThreads::Thread::microSleep( 5000000 );
        }

        OE_NOTICE << "All processes have ended" << std::endl;

    }

    protected:

    virtual bool handleTile( const TileKey& key )        
    {
        // Queue the task
        std::stringstream buf;
        buf << key.getLevelOfDetail() << "/" << key.getTileX() << "/" << key.getTileY();        
        std::string message = buf.str();        
        s_send( _sender, message.c_str() ); 
        return true;
    }  

    void* _sender;
    void* _context;
};


class ZMQTileHandler
{
public:
    ZMQTileHandler(TileHandler* handler):
      _handler( handler )
    {
    }

    virtual void run(const Profile* mapProfile)
    {
        //  Socket to receive messages on
        void *context = zmq_ctx_new ();
        void *receiver = zmq_socket (context, ZMQ_PULL);
        zmq_connect (receiver, "tcp://localhost:5557");    

        //  Process tasks forever
        while (1) {            
            char *string = s_recv (receiver);
            unsigned int lod, x, y;
            if (strcmp(string, "DIE") == 0)
            {
                std::cout << "Got poison pill" << std::endl;
                break;
            }

            sscanf(string, "%d/%d/%d", &lod, &x, &y);
            free (string);        

            TileKey key( lod, x, y, mapProfile);        

            // OE_NOTICE << "Processing tile " << lod << "(" << x << ", " << y << ")" << std::endl;

            if (_handler)
            {                
                _handler->handleTile( key );
            }            
        }
        zmq_close (receiver);    
        zmq_ctx_destroy (context);        
    }

    osg::ref_ptr< TileHandler > _handler;
};



typedef std::vector< TileKey > TileKeyList;

class TaskList
{    
public:
    TaskList(const Profile* profile):
      _profile( profile )
    {
    }

    bool load( const std::string &filename)
    {          
        std::ifstream in( filename.c_str(), std::ios::in );

        std::string line;
        while( getline(in, line) )
        {            
            std::vector< std::string > parts;
            StringTokenizer(line, parts, "," );
            

            _keys.push_back( TileKey(as<unsigned int>(parts[0], 0), 
                                     as<unsigned int>(parts[1], 0), 
                                     as<unsigned int>(parts[2], 0),
                                     _profile ) );
        }
        

        return true;
    }

    void save( const std::string& filename )
    {        
        std::ofstream out( filename.c_str() );
        for (TileKeyList::iterator itr = _keys.begin(); itr != _keys.end(); ++itr)
        {
            out << (*itr).getLevelOfDetail() << ", " << (*itr).getTileX() << ", " << (*itr).getTileY() << std::endl;
        }
    }

    TileKeyList& getKeys()
    {
        return _keys;
    };


    TileKeyList _keys;
    osg::ref_ptr< const Profile > _profile;
};


/**
* Executes a command in an external process
*/
class ExecuteTask : public TaskRequest
{
public:
    ExecuteTask(const std::string& command, TileVisitor* visitor, unsigned int count):            
      _command( command ),
      _visitor( visitor ),
      _count( count )
      {
      }

      virtual void operator()(ProgressCallback* progress )
      {         
          system(_command.c_str());          
          cleanupTempFiles();
          _visitor->incrementProgress( _count );
      }

      void addTempFile( const std::string& filename )
      {
          _tempFiles.push_back(filename);
      }

      void cleanupTempFiles()
      {
          for (unsigned int i = 0; i < _tempFiles.size(); i++)
          {
              remove( _tempFiles[i].c_str() );
          }
      }


      std::vector< std::string > _tempFiles;
      std::string _command;
      TileVisitor* _visitor;
      unsigned int _count;
};


/**
 * Given a list of tasks and a TileHandler, run through them and call the handler for each TileKey.
 */
class TaskRunner
{
public:
    TaskRunner(TileHandler* handler, const TaskList& tasks):
      _handler( handler ),
      _tasks( tasks )
    {
    }

    virtual void run(const Profile* mapProfile)
    {        
        for (TileKeyList::iterator itr = _tasks.getKeys().begin(); itr != _tasks.getKeys().end(); ++itr)
        {
            _handler->handleTile( (*itr) );
        }
    }

    osg::ref_ptr< TileHandler > _handler;    
    TaskList _tasks;
};


/**
 * A TileVisitor that pushes all of it's generated keys onto a TaskService queue and handles them in background threads.
 */
class MultiprocessTileVisitor: public TileVisitor
{
public:
    MultiprocessTileVisitor():
      _numProcesses( OpenThreads::GetNumberOfProcessors() ),
      _batchSize(200)
      {
          osgDB::ObjectWrapper* wrapper = osgDB::Registry::instance()->getObjectWrapperManager()->findWrapper( "osg::Image" );
      }

      MultiprocessTileVisitor( TileHandler* handler ):
      TileVisitor( handler ),
      _numProcesses( OpenThreads::GetNumberOfProcessors() ),
      _batchSize(200)
      {
      }

      unsigned int getNumProcesses() const { return _numProcesses; }
      void setNumProcesses( unsigned int numProcesses) { _numProcesses = numProcesses; }

      unsigned int getBatchSize() const { return _batchSize; }
      void setBatchSize( unsigned int batchSize ) { _batchSize = batchSize; }

      void setBaseCommand(const std::string& baseCommand)
      {
          _baseCommand = baseCommand;
      }

      virtual void run(const Profile* mapProfile)
      {                             
          // Start up the task service          
          _taskService = new TaskService( "MPTileHandler", _numProcesses, 5000 );
          
          // Produce the tiles
          TileVisitor::run( mapProfile );

          // Process any remaining tasks in the final batch
          processBatch();
          
          // Send a poison pill to kill all the threads
          _taskService->add( new PoisonPill() );

          // Wait for everything to finish
          _taskService->waitforThreadsToComplete();          
      }

protected:

    virtual bool handleTile( const TileKey& key )        
    {        
        _batch.push_back( key );

        if (_batch.size() == _batchSize)
        {
            processBatch();
        }         
        return true;
    }

    void processBatch()
    {       
        TaskList tasks( 0 );
        for (unsigned int i = 0; i < _batch.size(); i++)
        {
            tasks.getKeys().push_back( _batch[i] );
        }
        // Save the task file out.
        std::string filename = getTempName("batch", ".tiles");        
        tasks.save( filename );        

        std::stringstream command;
        //command << "osgearth_cache2 --seed  gdal_tiff.earth --tiles " << filename;
        command << _baseCommand << " --tiles " << filename;
        OE_NOTICE << "Running command " << command.str() << std::endl;
        osg::ref_ptr< ExecuteTask > task = new ExecuteTask( command.str(), this, tasks.getKeys().size() );
        // Add the task file as a temp file to the task to make sure it gets deleted
        task->addTempFile( filename );

        _taskService->add(task);
        _batch.clear();
    }

    TileKeyList _batch;

    unsigned int _batchSize;
    unsigned int _numProcesses;    

    // The work queue to pass seed operations to
    osg::ref_ptr<osgEarth::TaskService> _taskService;

    std::string _baseCommand;
};


int list( osg::ArgumentParser& args );
int seed( osg::ArgumentParser& args );
int purge( osg::ArgumentParser& args );
int usage( const std::string& msg );
int message( const std::string& msg );


int
main(int argc, char** argv)
{
    osg::ArgumentParser args(&argc,argv);

    if ( args.read( "--seed") )
        return seed( args );
    else if ( args.read( "--list" ) )
        return list( args );
    else if ( args.read( "--purge" ) )
        return purge( args );    
    else
        return usage("");
}

int
usage( const std::string& msg )
{
    if ( !msg.empty() )
    {
        std::cout << msg << std::endl;
    }

    std::cout
        << std::endl
        << "USAGE: osgearth_cache" << std::endl
        << std::endl
        << "    --list file.earth                   ; Lists info about the cache in a .earth file" << std::endl       
        << std::endl
        << "    --seed file.earth                   ; Seeds the cache in a .earth file"  << std::endl
        << "        [--estimate]                    ; Print out an estimation of the number of tiles, disk space and time it will take to perform this seed operation" << std::endl
        << "        [--min-level level]             ; Lowest LOD level to seed (default=0)" << std::endl
        << "        [--max-level level]             ; Highest LOD level to seed (defaut=highest available)" << std::endl
        << "        [--bounds xmin ymin xmax ymax]* ; Geospatial bounding box to seed (in map coordinates; default=entire map)" << std::endl
        << "        [--index shapefile]             ; Use the feature extents in a shapefile to set the bounding boxes for seeding" << std::endl
        << "        [--cache-path path]             ; Overrides the cache path in the .earth file" << std::endl
        << "        [--cache-type type]             ; Overrides the cache type in the .earth file" << std::endl
        << "        [--threads]                     ; The number of threads to use for the seed operation (default=1)" << std::endl
        << "        [--verbose]                     ; Displays progress of the seed operation" << std::endl
        << std::endl
        << "    --purge file.earth                  ; Purges a layer cache in a .earth file (interactive)" << std::endl
        << std::endl;

    return -1;
}

int message( const std::string& msg )
{
    if ( !msg.empty() )
    {
        std::cout << msg << std::endl << std::endl;
    }
    return 0;
}

int
seed( osg::ArgumentParser& args )
{    
    osgDB::Registry::instance()->getReaderWriterForExtension("png");
    osgDB::Registry::instance()->getReaderWriterForExtension("jpg");
    osgDB::Registry::instance()->getReaderWriterForExtension("tiff");

    //Read the min level
    unsigned int minLevel = 0;
    while (args.read("--min-level", minLevel));
    
    //Read the max level
    unsigned int maxLevel = 5;
    while (args.read("--max-level", maxLevel));

    bool estimate = args.read("--estimate");        

    unsigned int threads = 1;
    while (args.read("--threads", threads));
    

    std::vector< Bounds > bounds;
    // restrict packaging to user-specified bounds.    
    double xmin=DBL_MAX, ymin=DBL_MAX, xmax=DBL_MIN, ymax=DBL_MIN;
    while (args.read("--bounds", xmin, ymin, xmax, ymax ))
    {        
        Bounds b;
        b.xMin() = xmin, b.yMin() = ymin, b.xMax() = xmax, b.yMax() = ymax;
        bounds.push_back( b );
    }    

    std::string tileList;
    while (args.read( "--tiles", tileList ) );

    //Read the cache override directory
    std::string cachePath;
    while (args.read("--cache-path", cachePath));

    //Read the cache type
    std::string cacheType;
    while (args.read("--cache-type", cacheType));

    bool verbose = args.read("--verbose");

    //Read in the earth file.
    osg::ref_ptr<osg::Node> node = osgDB::readNodeFiles( args );
    if ( !node.valid() )
        return usage( "Failed to read .earth file." );

    MapNode* mapNode = MapNode::findMapNode( node.get() );
    if ( !mapNode )
        return usage( "Input file was not a .earth file" );
    
    // Read in an index shapefile
    std::string index;
    while (args.read("--index", index))
    {        
        //Open the feature source
        OGRFeatureOptions featureOpt;
        featureOpt.url() = index;        

        osg::ref_ptr< FeatureSource > features = FeatureSourceFactory::create( featureOpt );
        features->initialize();
        features->getFeatureProfile();

        osg::ref_ptr< FeatureCursor > cursor = features->createFeatureCursor();
        while (cursor.valid() && cursor->hasMore())
        {
            osg::ref_ptr< Feature > feature = cursor->nextFeature();
            osgEarth::Bounds featureBounds = feature->getGeometry()->getBounds();
            GeoExtent ext( feature->getSRS(), featureBounds );
            ext = ext.transform( mapNode->getMapSRS() );
            bounds.push_back( ext.bounds() );            
        }
    }

    // If they requested to do an estimate then don't do the the seed, just print out the estimated values.
    if (estimate)
    {        
        CacheEstimator est;
        est.setMinLevel( minLevel );
        est.setMaxLevel( maxLevel );
        est.setProfile( mapNode->getMap()->getProfile() );

        for (unsigned int i = 0; i < bounds.size(); i++)
        {
            GeoExtent extent(mapNode->getMapSRS(), bounds[i]);
            OE_DEBUG << "Adding extent " << extent.toString() << std::endl;
            est.addExtent( extent );
        } 

        unsigned int numTiles = est.getNumTiles();
        double size = est.getSizeInMB();
        double time = est.getTotalTimeInSeconds();
        std::cout << "Cache Estimation " << std::endl
                  << "---------------- " << std::endl
                  << "Total number of tiles: " << numTiles << std::endl
                  << "Size on disk:          " << osgEarth::prettyPrintSize( size ) << std::endl
                  << "Total time:            " << osgEarth::prettyPrintTime( time ) << std::endl;

        return 0;
    }

    if (args.read( "--client") )
    {        
        ZMQTileHandler consumer( new CacheTileHandler( mapNode->getMap()->getImageLayerAt( 0 ) ) );
        consumer.run( mapNode->getMap()->getProfile() );
    }
    else
    {
        if (!tileList.empty())
        {        
            TaskList tasks( mapNode->getMap()->getProfile() );
            tasks.load( tileList );

            TaskRunner runner( new CacheTileHandler( mapNode->getMap()->getImageLayerAt( 0 ) ), tasks );
            runner.run( mapNode->getMap()->getProfile() );
        }
        else
        {            
            // Simple single threaded cache seeder
            /*
            LayerSeeder visitor( mapNode->getMap()->getImageLayerAt( 0 ) );    
            */

            osg::ref_ptr< ProgressCallback > progress = new ConsoleProgressCallback();

            // Multithread cache seeder    
            //MultithreadedTileVisitor visitor;
            //ZMQTileVisitor visitor;            
            //Get the first argument that is not an option, that is the earth file name.
            std::string earthFile;
            for(int pos=1;pos<args.argc();++pos)
            {
                if (!args.isOption(pos))
                {
                    earthFile  = args[ pos ];
                }
            }
            MultiprocessTileVisitor visitor;
            std::stringstream baseCommand;
            baseCommand << "osgearth_cache2 --seed " << earthFile;            
            visitor.setBaseCommand(baseCommand.str());

            //TileVisitor visitor;
            visitor.setProgressCallback( progress );
            visitor.setTileHandler( new CacheTileHandler( mapNode->getMap()->getImageLayerAt( 0 ) ) );    

            // Multithread TMS packager    
            /*
            MultithreadedTileVisitor visitor;
            visitor.setTileHandler( new PackageTileHandler(mapNode->getMap()->getImageLayerAt( 0 ), "out_tms", "jpg") );    
            */


            visitor.setMinLevel( minLevel );
            visitor.setMaxLevel( maxLevel );        


            for (unsigned int i = 0; i < bounds.size(); i++)
            {
                GeoExtent extent(mapNode->getMapSRS(), bounds[i]);
                OE_DEBUG << "Adding extent " << extent.toString() << std::endl;                
                visitor.addExtent( extent );
            }    


            osg::Timer_t start = osg::Timer::instance()->tick();

            visitor.run( mapNode->getMap()->getProfile() );

            osg::Timer_t end = osg::Timer::instance()->tick();

            OE_NOTICE << "Completed seeding in " << osgEarth::prettyPrintTime( osg::Timer::instance()->delta_s( start, end ) ) << std::endl;
        }
    }

    return 0;
}

int
list( osg::ArgumentParser& args )
{
    osg::ref_ptr<osg::Node> node = osgDB::readNodeFiles( args );
    if ( !node.valid() )
        return usage( "Failed to read .earth file." );

    MapNode* mapNode = MapNode::findMapNode( node.get() );
    if ( !mapNode )
        return usage( "Input file was not a .earth file" );

    Map* map = mapNode->getMap();
    const Cache* cache = map->getCache();

    if ( !cache )
        return message( "Earth file does not contain a cache." );

    std::cout 
        << "Cache config: " << std::endl
        << cache->getCacheOptions().getConfig().toJSON(true) << std::endl;

    MapFrame mapf( mapNode->getMap() );

    TerrainLayerVector layers;
    std::copy( mapf.imageLayers().begin(), mapf.imageLayers().end(), std::back_inserter(layers) );
    std::copy( mapf.elevationLayers().begin(), mapf.elevationLayers().end(), std::back_inserter(layers) );

    for( TerrainLayerVector::iterator i =layers.begin(); i != layers.end(); ++i )
    {
        TerrainLayer* layer = i->get();
        TerrainLayer::CacheBinMetadata meta;

        bool useMFP =
            layer->getProfile() &&
            layer->getProfile()->getSRS()->isSphericalMercator() &&
            mapNode->getMapNodeOptions().getTerrainOptions().enableMercatorFastPath() == true;

        const Profile* cacheProfile = useMFP ? layer->getProfile() : map->getProfile();

        if ( layer->getCacheBinMetadata( cacheProfile, meta ) )
        {
            Config conf = meta.getConfig();
            std::cout << "Layer \"" << layer->getName() << "\", cache metadata =" << std::endl
                << conf.toJSON(true) << std::endl;
        }
        else
        {
            std::cout << "Layer \"" << layer->getName() << "\": no cache information" 
                << std::endl;
        }
    }

    return 0;
}

struct Entry
{
    bool                   _isImage;
    std::string            _name;
    osg::ref_ptr<CacheBin> _bin;
};


int
purge( osg::ArgumentParser& args )
{
    //return usage( "Sorry, but purge is not yet implemented." );
    
    osg::ref_ptr<osg::Node> node = osgDB::readNodeFiles( args );
    if ( !node.valid() )
        return usage( "Failed to read .earth file." );

    MapNode* mapNode = MapNode::findMapNode( node.get() );
    if ( !mapNode )
        return usage( "Input file was not a .earth file" );

    Map* map = mapNode->getMap();

    if ( !map->getCache() )
        return message( "Earth file does not contain a cache." );

    std::vector<Entry> entries;


    ImageLayerVector imageLayers;
    map->getImageLayers( imageLayers );
    for( ImageLayerVector::const_iterator i = imageLayers.begin(); i != imageLayers.end(); ++i )
    {
        ImageLayer* layer = i->get();

        bool useMFP =
            layer->getProfile() &&
            layer->getProfile()->getSRS()->isSphericalMercator() &&
            mapNode->getMapNodeOptions().getTerrainOptions().enableMercatorFastPath() == true;

        const Profile* cacheProfile = useMFP ? layer->getProfile() : map->getProfile();

        CacheBin* bin = layer->getCacheBin( cacheProfile );
        if ( bin )
        {
            entries.push_back(Entry());
            entries.back()._isImage = true;
            entries.back()._name = i->get()->getName();
            entries.back()._bin = bin;
        }
    }

    ElevationLayerVector elevationLayers;
    map->getElevationLayers( elevationLayers );
    for( ElevationLayerVector::const_iterator i = elevationLayers.begin(); i != elevationLayers.end(); ++i )
    {
        ElevationLayer* layer = i->get();

        bool useMFP =
            layer->getProfile() &&
            layer->getProfile()->getSRS()->isSphericalMercator() &&
            mapNode->getMapNodeOptions().getTerrainOptions().enableMercatorFastPath() == true;

        const Profile* cacheProfile = useMFP ? layer->getProfile() : map->getProfile();

        CacheBin* bin = i->get()->getCacheBin( cacheProfile );
        if ( bin )
        {
            entries.push_back(Entry());
            entries.back()._isImage = false;
            entries.back()._name = i->get()->getName();
            entries.back()._bin = bin;
        }
    }

    if ( entries.size() > 0 )
    {
        std::cout << std::endl;

        for( unsigned i=0; i<entries.size(); ++i )
        {
            std::cout << (i+1) << ") " << entries[i]._name << " (" << (entries[i]._isImage? "image" : "elevation" ) << ")" << std::endl;
        }

        std::cout 
            << std::endl
            << "Enter number of cache to purge, or <enter> to quit: "
            << std::flush;

        std::string input;
        std::getline( std::cin, input );

        if ( !input.empty() )
        {
            unsigned k = as<unsigned>(input, 0L);
            if ( k > 0 && k <= entries.size() )
            {
                Config meta = entries[k-1]._bin->readMetadata();
                if ( !meta.empty() )
                {
                    std::cout
                        << std::endl
                        << "Cache METADATA:" << std::endl
                        << meta.toJSON() 
                        << std::endl << std::endl;
                }

                std::cout
                    << "Are you sure (y/N)? "
                    << std::flush;

                std::getline( std::cin, input );
                if ( input == "y" || input == "Y" )
                {
                    std::cout << "Purging.." << std::flush;
                    entries[k-1]._bin->purge();
                }
                else
                {
                    std::cout << "No action taken." << std::endl;
                }
            }
            else
            {
                std::cout << "Invalid choice." << std::endl;
            }
        }
        else
        {
            std::cout << "No action taken." << std::endl;
        }
    }

    return 0;
}