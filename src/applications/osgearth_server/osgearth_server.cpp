/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2015 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/Notify>
#include <osgEarth/Registry>
#include <osgEarth/TerrainEngineNode>
#include <osgEarthUtil/ExampleResources>
#include <osgDB/ReaderWriter>
#include <osgDB/ReadFile>
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>

#include <Poco/Net/HTTPServer.h>
#include <Poco/Net/HTTPRequestHandler.h>
#include <Poco/Net/HTTPRequestHandlerFactory.h>
#include <Poco/Net/HTTPServerParams.h>
#include <Poco/Net/HTTPServerRequest.h>
#include <Poco/Net/HTTPServerResponse.h>
#include <Poco/Net/HTTPServerParams.h>
#include <Poco/Net/ServerSocket.h>
#include <Poco/Timestamp.h>
#include <Poco/DateTimeFormatter.h>
#include <Poco/DateTimeFormat.h>
#include <Poco/Exception.h>
#include <Poco/ThreadPool.h>
#include <Poco/Util/ServerApplication.h>
#include <Poco/Util/Option.h>
#include <Poco/Util/OptionSet.h>
#include <Poco/Util/HelpFormatter.h>
#include <iostream>

using Poco::Net::ServerSocket;
using Poco::Net::HTTPRequestHandler;
using Poco::Net::HTTPRequestHandlerFactory;
using Poco::Net::HTTPServer;
using Poco::Net::HTTPServerRequest;
using Poco::Net::HTTPServerResponse;
using Poco::Net::HTTPServerParams;
using Poco::Timestamp;
using Poco::DateTimeFormatter;
using Poco::DateTimeFormat;
using Poco::ThreadPool;
using Poco::Util::ServerApplication;
using Poco::Util::Application;
using Poco::Util::Option;
using Poco::Util::OptionSet;
using Poco::Util::OptionCallback;
using Poco::Util::HelpFormatter;


#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgViewer;

class WindowCaptureCallback : public osg::Camera::DrawCallback
{
    public:
       
        enum FramePosition
        {
            START_FRAME,
            END_FRAME
        };
    
        struct ContextData : public osg::Referenced
        {
        
            ContextData(osg::GraphicsContext* gc, GLenum readBuffer):
                _gc(gc),
                _readBuffer(readBuffer),
                _pixelFormat(GL_BGRA),
                _type(GL_UNSIGNED_BYTE),
                _width(0),
                _height(0),
                _currentImageIndex(0),
                _currentPboIndex(0)
            {
                if (gc->getTraits())
                {
                    if (gc->getTraits()->alpha)
                    {
                        osg::notify(osg::NOTICE)<<"Select GL_BGRA read back format"<<std::endl;
                        _pixelFormat = GL_BGRA;
                    }
                    else 
                    {
                        osg::notify(osg::NOTICE)<<"Select GL_BGR read back format"<<std::endl;
                        _pixelFormat = GL_BGR; 
                    }
                }
            
                getSize(gc, _width, _height);
                
                std::cout<<"Window size "<<_width<<", "<<_height<<std::endl;
            
                // single buffered image
                _imageBuffer.push_back(new osg::Image);

                _pboBuffer.push_back(0);
            }
            
            void getSize(osg::GraphicsContext* gc, int& width, int& height)
            {
                if (gc->getTraits())
                {
                    width = gc->getTraits()->width;
                    height = gc->getTraits()->height;
                }
            }
           

            void read()
            {
#if OSG_VERSION_GREATER_OR_EQUAL(3,4,0)
                osg::GLExtensions* ext = osg::GLExtensions::Get(_gc->getState()->getContextID(),true);
#else
                osg::GLBufferObject::Extensions* ext = osg::GLBufferObject::getExtensions(_gc->getState()->getContextID(),true);
#endif

#if OSG_VERSION_GREATER_OR_EQUAL(3,4,0)
                if (ext->isPBOSupported && !_pboBuffer.empty())
#else
                if (ext->isPBOSupported() && !_pboBuffer.empty())
#endif
                {
                        singlePBO(ext);
                }
            }
           
#if OSG_VERSION_GREATER_OR_EQUAL(3,4,0)
            void singlePBO(osg::GLExtensions* ext);
#else
            void singlePBO(osg::GLBufferObject::Extensions* ext);
#endif

       
            typedef std::vector< osg::ref_ptr<osg::Image> >             ImageBuffer;
            typedef std::vector< GLuint > PBOBuffer;

       
            osg::GraphicsContext*   _gc;
            GLenum                  _readBuffer;
            
            GLenum                  _pixelFormat;
            GLenum                  _type;
            int                     _width;
            int                     _height;
            
            unsigned int            _currentImageIndex;
            ImageBuffer             _imageBuffer;
            
            unsigned int            _currentPboIndex;
            PBOBuffer               _pboBuffer;

            osg::ref_ptr< osg::Image > _lastImage;
        };
    
        WindowCaptureCallback(FramePosition position, GLenum readBuffer):
            _position(position),
            _readBuffer(readBuffer)
        {
        }

        FramePosition getFramePosition() const { return _position; }

        ContextData* createContextData(osg::GraphicsContext* gc) const
        {
            return new ContextData(gc, _readBuffer);
        }
        
        ContextData* getContextData(osg::GraphicsContext* gc) const
        {
            OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
            osg::ref_ptr<ContextData>& data = _contextDataMap[gc];
            if (!data) data = createContextData(gc);
            
            return data.get();
        }

        virtual void operator () (osg::RenderInfo& renderInfo) const
        {
            glReadBuffer(_readBuffer);

            osg::GraphicsContext* gc = renderInfo.getState()->getGraphicsContext();
            osg::ref_ptr<ContextData> cd = getContextData(gc);
            cd->read();
            const_cast<WindowCaptureCallback*>(this)->_image = cd->_lastImage.get();
        }
        
        typedef std::map<osg::GraphicsContext*, osg::ref_ptr<ContextData> > ContextDataMap;

        FramePosition               _position;
        GLenum                      _readBuffer;
        mutable OpenThreads::Mutex  _mutex;
        mutable ContextDataMap      _contextDataMap;
        osg::ref_ptr< osg::Image>   _image;
        
        
};

#if OSG_VERSION_GREATER_OR_EQUAL(3,4,0)
            void WindowCaptureCallback::ContextData::singlePBO(osg::GLExtensions* ext)
#else
            void WindowCaptureCallback::ContextData::singlePBO(osg::GLBufferObject::Extensions* ext)
#endif
{ 
    unsigned int nextImageIndex = (_currentImageIndex+1)%_imageBuffer.size();

    int width=0, height=0;
    getSize(_gc, width, height);
    if (width!=_width || _height!=height)
    {
        std::cout<<"   Window resized "<<width<<", "<<height<<std::endl;
        _width = width;
        _height = height;
    }

    GLuint& pbo = _pboBuffer[0];
    
    osg::Image* image = _imageBuffer[_currentImageIndex].get();
    if (image->s() != _width || 
        image->t() != _height)
    {
        osg::notify(osg::NOTICE)<<"Allocating image "<<std::endl;
        image->allocateImage(_width, _height, 1, _pixelFormat, _type);
        
        if (pbo!=0)
        {
            osg::notify(osg::NOTICE)<<"deleting pbo "<<pbo<<std::endl;
            ext->glDeleteBuffers (1, &pbo);
            pbo = 0;
        }
    }
    
    
    if (pbo==0)
    {
        ext->glGenBuffers(1, &pbo);
        ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, pbo);
        ext->glBufferData(GL_PIXEL_PACK_BUFFER_ARB, image->getTotalSizeInBytes(), 0, GL_STREAM_READ);

        osg::notify(osg::NOTICE)<<"Generating pbo "<<pbo<<std::endl;
    }
    else
    {
        ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, pbo);
    }

#if 1
    glReadPixels(0, 0, _width, _height, _pixelFormat, _type, 0);
#endif    

    GLubyte* src = (GLubyte*)ext->glMapBuffer(GL_PIXEL_PACK_BUFFER_ARB,
                                              GL_READ_ONLY_ARB);
    if(src)
    {
        memcpy(image->data(), src, image->getTotalSizeInBytes());
        
        ext->glUnmapBuffer(GL_PIXEL_PACK_BUFFER_ARB);
    }

    ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, 0);

    _lastImage = new osg::Image(*image);

    _currentImageIndex = nextImageIndex;
}



int
usage(const char* name)
{
    OE_NOTICE 
        << "\nUsage: " << name << " file.earth" << std::endl
        << MapNodeHelper().usage() << std::endl;

    return 0;
}

#define LOD_COUNT 26

// Note:  These are taken from Splat.frag.glsl so they line up exactly.
const float oe_LODRanges[LOD_COUNT] = {
       100000000.0, // 0
        75000000.0, // 1
        50000000.0, // 2
        10000000.0, // 3
         7500000.0, // 4
         5000000.0, // 5
         2500000.0, // 6
         1000000.0, // 7
          500000.0, // 8
          225000.0, // 9
          150000.0, // 10
           80000.0, // 11
           30000.0, // 12
           14000.0, // 13
            4000.0, // 14
            2500.0, // 15
            1000.0, // 16
             500.0, // 17
             250.0, // 18
             125.0, // 19
              50.0, // 20
              25.0, // 21
              12.0, // 22
               6.0, // 23
               3.0, // 24
               1.0  // 25
};




class TileImageServer
{
public:
    TileImageServer(MapNode* mapNode):
      _mapNode(mapNode),
      _viewer(0),
      _windowCaptureCallback(0)
      {
          _viewer = new osgViewer::Viewer;
          _viewer->getCamera()->setSmallFeatureCullingPixelSize(-1.0f);
          _viewer->setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
          float half = 256.0/2.0;
          _viewer->getCamera()->setProjectionMatrixAsOrtho2D(-half, half, -half, half);

          osg::GraphicsContext* gc = createGraphicsContext();
          if (gc)
          {
              OE_NOTICE << "Created graphics context" << std::endl;
              _viewer->getCamera()->setGraphicsContext( gc );
              _viewer->getCamera()->setViewport(new osg::Viewport(0,0,256,256));
          }
          else
          {
              OE_NOTICE << "Failed to create graphics context" << std::endl;
          }

          _windowCaptureCallback = new WindowCaptureCallback(WindowCaptureCallback::END_FRAME, GL_BACK);

           GLenum buffer = GL_BACK;
           _viewer->getCamera()->setDrawBuffer(buffer);
           _viewer->getCamera()->setReadBuffer(buffer);
           

          _root = new osg::Group;

          _root->addChild(mapNode);

          _viewer->setSceneData( _root.get() );

          _viewer->realize();
      }

      osg::GraphicsContext* createGraphicsContext()
      {       
          osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;  
          traits->x = 0;
          traits->y = 0;
          traits->width = 256;
          traits->height = 256;
          traits->windowDecoration = false;
          traits->doubleBuffer = true;
          traits->sharedContext = 0;
          traits->pbuffer = true;
          return osg::GraphicsContext::createGraphicsContext(traits.get());
      }

      ~TileImageServer()
      {
          if (_viewer)
          {
              delete _viewer;
          }
      }

      osg::Image* getTile(unsigned int z, unsigned int x, unsigned int y)
      {
          OpenThreads::ScopedLock< OpenThreads::Mutex> lk(_mutex);

          // Invert the y
          unsigned cols=0, rows=0;
          _mapNode->getMap()->getProfile()->getNumTiles( z, cols, rows );
          y = rows - y - 1;


          TileKey key(z, x, y, _mapNode->getMap()->getProfile());

          OE_DEBUG << "key=" << key.str() << std::endl;

          // Set the projection matrix to capture the tile.                    
          OE_DEBUG << "Key extent " << z << "(" << x << ", " << y << ") = " << key.getExtent().toString() << std::endl;
          //_viewer->getCamera()->setProjectionMatrixAsOrtho2D(key.getExtent().xMin(), key.getExtent().xMax(), key.getExtent().yMin(), key.getExtent().yMax());
          unsigned int heightIndex = osg::clampBetween(z, 0u, LOD_COUNT -1u);

          // Multiply by the min_tile_range_factor to get close to what the correct distance would be for this tile.
          double height = oe_LODRanges[heightIndex] * *_mapNode->getMapNodeOptions().getTerrainOptions().minTileRangeFactor();
          osg::Vec3d center = key.getExtent().getCentroid();
          osg::Vec3d eye = center + osg::Vec3d(0,0,height);
          _viewer->getCamera()->setViewMatrixAsLookAt(eye, center, osg::Vec3d(0,1,0));
          _viewer->getCamera()->setProjectionMatrixAsOrtho2D(-key.getExtent().width()/2.0,  key.getExtent().width()/2.0,
                                                             -key.getExtent().height()/2.0, key.getExtent().height()/2.0);
          _viewer->frame();
          int numFrames = 0;

          while (_viewer->getDatabasePager()->getRequestsInProgress())
          {
              OE_DEBUG << "Waiting on key  " << key.str() << std::endl
                        << "   file requests = " << _viewer->getDatabasePager()->getFileRequestListSize() << std::endl
                        << "   data to compile = " << _viewer->getDatabasePager()->getDataToCompileListSize() << std::endl
                        << "   data to merge = " << _viewer->getDatabasePager()->getDataToMergeListSize() << std::endl;
              _viewer->frame();
              numFrames++;
          }

          OE_DEBUG << "Took " << numFrames << " to load data for " << key.str() << std::endl;

          _viewer->getCamera()->setFinalDrawCallback(_windowCaptureCallback);
          _viewer->frame();
          _viewer->frame();
          _viewer->getCamera()->setFinalDrawCallback( 0 );
                   
          if (_windowCaptureCallback->_image)
          {
              return new osg::Image(*_windowCaptureCallback->_image);
          }
          return 0;
      }

    osgViewer::Viewer *_viewer;
    osg::ref_ptr< MapNode > _mapNode;
    osg::ref_ptr< osg::Group > _root;
    osg::ref_ptr< WindowCaptureCallback > _windowCaptureCallback;
    OpenThreads::Mutex _mutex;
};


static TileImageServer* _server;

class TileRequestHandler: public HTTPRequestHandler
{
public:
    TileRequestHandler()
    {
    }

    void handleRequest(HTTPServerRequest& request,
                       HTTPServerResponse& response)
    {
        StringTokenizer tok("/");
        StringVector tized;
        tok.tokenize(request.getURI(), tized);            
        if ( tized.size() == 4 )
        {
            int z = as<int>(tized[1], 0);
            int x = as<int>(tized[2], 0);
            unsigned int y = as<int>(osgDB::getNameLessExtension(tized[3]),0);
            std::string ext = osgDB::getFileExtension(tized[3]);

            OE_DEBUG << "z=" << z << std::endl;
            OE_DEBUG << "x=" << x << std::endl;
            OE_DEBUG << "y=" << y << std::endl;              
            OE_DEBUG << "ext=" << ext << std::endl;

            response.setChunkedTransferEncoding(true);

            osg::ref_ptr< osg::Image > image = _server->getTile(z, x, y);
            
            if (image)
            {
                osgDB::ReaderWriter* rw = osgDB::Registry::instance()->getReaderWriterForExtension(ext);
                if (rw)
                {
                    std::string mime = "image/png";
                    if (ext == "jpeg" || ext == "jpg")
                    {
                        mime = "image/jpeg";
                    }                    
                    response.setContentType(mime);
                    std::ostream& ostr = response.send();                 
                    rw->writeImage(*image.get(), ostr);                    
                }             

            }
        }
 
        response.setStatus(Poco::Net::HTTPResponse::HTTP_NOT_FOUND);                
    }

private:
    std::string _format;
};

class TileRequestHandlerFactory : public HTTPRequestHandlerFactory
{
    public:
    TileRequestHandlerFactory()
    {
    }

    HTTPRequestHandler* createRequestHandler(
        const HTTPServerRequest& request)
    {        
        StringTokenizer tok("/");
        StringVector tized;
        tok.tokenize(request.getURI(), tized);            
        if ( tized.size() == 4 )
        {
            int z = as<int>(tized[1], 0);
            int x = as<int>(tized[2], 0);
            unsigned int y = as<int>(osgDB::getNameLessExtension(tized[3]),0);
            std::string ext = osgDB::getFileExtension(tized[3]);

            OE_DEBUG << "z=" << z << std::endl;
            OE_DEBUG << "x=" << x << std::endl;
            OE_DEBUG << "y=" << y << std::endl;              
            OE_DEBUG << "ext=" << ext << std::endl;

            return new TileRequestHandler();
        }

        return 0;
    }
};

class TileHTTPServer: public Poco::Util::ServerApplication
{
public:
    TileHTTPServer(int port):
      _port(port)
    {
    }

    ~TileHTTPServer()
    {
    }

protected:
    void initialize(Application& self)
    {
        loadConfiguration();
        ServerApplication::initialize(self);
    }

    void uninitialize()
    {
        ServerApplication::uninitialize();
    }

    int main(const std::vector<std::string>& args)
    {
        ServerSocket svs(_port);
        HTTPServer srv(new TileRequestHandlerFactory(), svs, new HTTPServerParams);
        srv.start();
        waitForTerminationRequest();
        srv.stop();
        return Application::EXIT_OK;
    }

private:
    int _port;
};



int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    // help?
    if ( arguments.read("--help") )
        return usage(argv[0]);

    int port = 8000;
    arguments.read("--port", port);
    OE_NOTICE << "Listening on port " << port << std::endl;

    // thread-safe initialization of the OSG wrapper manager. Calling this here
    // prevents the "unsupported wrapper" messages from OSG
    osgDB::Registry::instance()->getObjectWrapperManager()->findWrapper("osg::Image");

        // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::ref_ptr< osg::Node> node = osgDB::readNodeFiles( arguments );
    osg::ref_ptr< MapNode > mapNode = MapNode::findMapNode( node );

    if (mapNode.valid())
    {
        OE_NOTICE << "Found map node" << std::endl;
    }

    _server = new TileImageServer( mapNode.get() );

    TileHTTPServer app(port);
    return app.run(argc, argv);
}
