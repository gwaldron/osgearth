#include "ZMQ"

#ifdef OSGEARTH_HAVE_ZEROMQ

#include <zmq.h>


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




/**
* Launches a process in a background thread
*/
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
      }

      std::string _command;
};


ZMQTileVisitor::ZMQTileVisitor():
_sender(0),
    _context(0),
    _numWorkers( OpenThreads::GetNumberOfProcessors() )
{        
}

ZMQTileVisitor::~ZMQTileVisitor()
{
    //TODO:  Close stuff.
    //zmq_close (_sender);
    //zmq_ctx_destroy (_context);
}

unsigned int ZMQTileVisitor::getNumWorkers() const
{
    return _numWorkers;
}

void ZMQTileVisitor::setNumWorkers( unsigned int numWorkers )
{
    _numWorkers = numWorkers;
}

void ZMQTileVisitor::run(const Profile* mapProfile)
{
    _context = zmq_ctx_new ();

    //  Socket to send messages on
    _sender = zmq_socket(_context, ZMQ_PUSH);
    int hwm = 10000;
    zmq_setsockopt( _sender, ZMQ_SNDHWM, &hwm, sizeof(hwm));
    zmq_bind (_sender, "tcp://*:5557");    

    std::vector< osg::ref_ptr< ProcessThread > > processes;
    for (unsigned int i = 0; i < _numWorkers; i++)
    {        
        ProcessThread* process = new ProcessThread("osgearth_cache2 --seed --client gdal_tiff.earth");
        process->start();            
        processes.push_back(process);
    }        

    // Wait for the threads to actually launch their processes and begin listening
    OpenThreads::Thread::microSleep( 30000000 );            

    // Produce the tiles
    TileVisitor::run( mapProfile );        


    bool isRunning = true;
    while (isRunning)
    {
        unsigned int numRunning = 0;
        isRunning = false;
        // Wait for all processes to die
        for (unsigned int i = 0; i < _numWorkers; i++)
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


bool ZMQTileVisitor::handleTile( const TileKey& key )        
{
    // Queue the task
    std::stringstream buf;
    buf << key.getLevelOfDetail() << "/" << key.getTileX() << "/" << key.getTileY();        
    std::string message = buf.str();        
    s_send( _sender, message.c_str() ); 
    return true;
}  


/*****************************************************************************************/
ZMQTileHandler::ZMQTileHandler(TileHandler* handler):
_handler( handler )
{
}

void ZMQTileHandler::run(const Profile* mapProfile)
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

#endif