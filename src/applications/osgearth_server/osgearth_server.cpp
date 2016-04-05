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
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgDB/ReaderWriter>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>

#include "mongoose.h"

#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgViewer;

class WindowCaptureCallback : public osg::Camera::DrawCallback
{
    public:
    
        enum Mode
        {
            READ_PIXELS,
            SINGLE_PBO,
            DOUBLE_PBO,
            TRIPLE_PBO
        };
    
        enum FramePosition
        {
            START_FRAME,
            END_FRAME
        };
    
        struct ContextData : public osg::Referenced
        {
        
            ContextData(osg::GraphicsContext* gc, Mode mode, GLenum readBuffer, const std::string& name):
                _gc(gc),
                _mode(mode),
                _readBuffer(readBuffer),
                _fileName(name),
                _pixelFormat(GL_BGRA),
                _type(GL_UNSIGNED_BYTE),
                _width(0),
                _height(0),
                _currentImageIndex(0),
                _currentPboIndex(0),
                _reportTimingFrequency(100),
                _numTimeValuesRecorded(0),
                _timeForReadPixels(0.0),
                _timeForFullCopy(0.0),
                _timeForMemCpy(0.0)

            {
                _previousFrameTick = osg::Timer::instance()->tick();

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
                
                // double buffer PBO.
                switch(_mode)
                {
                    case(READ_PIXELS):
                        osg::notify(osg::NOTICE)<<"Reading window usig glReadPixels, with out PixelBufferObject."<<std::endl;
                        break;
                    case(SINGLE_PBO): 
                        osg::notify(osg::NOTICE)<<"Reading window usig glReadPixels, with a single PixelBufferObject."<<std::endl;
                        _pboBuffer.push_back(0); 
                        break;
                    case(DOUBLE_PBO): 
                        osg::notify(osg::NOTICE)<<"Reading window usig glReadPixels, with a double buffer PixelBufferObject."<<std::endl;
                        _pboBuffer.push_back(0); 
                        _pboBuffer.push_back(0); 
                        break;
                    case(TRIPLE_PBO): 
                        osg::notify(osg::NOTICE)<<"Reading window usig glReadPixels, with a triple buffer PixelBufferObject."<<std::endl;
                        _pboBuffer.push_back(0); 
                        _pboBuffer.push_back(0); 
                        _pboBuffer.push_back(0); 
                        break;
                    default:
                        break;                                
                }
            }
            
            void getSize(osg::GraphicsContext* gc, int& width, int& height)
            {
                if (gc->getTraits())
                {
                    width = gc->getTraits()->width;
                    height = gc->getTraits()->height;
                }
            }
            
            void updateTimings(osg::Timer_t tick_start,
                               osg::Timer_t tick_afterReadPixels,
                               osg::Timer_t tick_afterMemCpy,
                               unsigned int dataSize);

            void read()
            {
                osg::GLExtensions* ext = osg::GLExtensions::Get(_gc->getState()->getContextID(),true);

                if (ext->isPBOSupported && !_pboBuffer.empty())
                {
                    if (_pboBuffer.size()==1)
                    {
                        singlePBO(ext);
                    }
                }
                else
                {
                    readPixels();
                }
            }
            
            void readPixels();

            void singlePBO(osg::GLExtensions* ext);
       
            typedef std::vector< osg::ref_ptr<osg::Image> >             ImageBuffer;
            typedef std::vector< GLuint > PBOBuffer;

       
            osg::GraphicsContext*   _gc;
            Mode                    _mode;
            GLenum                  _readBuffer;
            std::string             _fileName;
            
            GLenum                  _pixelFormat;
            GLenum                  _type;
            int                     _width;
            int                     _height;
            
            unsigned int            _currentImageIndex;
            ImageBuffer             _imageBuffer;
            
            unsigned int            _currentPboIndex;
            PBOBuffer               _pboBuffer;

            unsigned int            _reportTimingFrequency;
            unsigned int            _numTimeValuesRecorded;
            double                  _timeForReadPixels;
            double                  _timeForFullCopy;
            double                  _timeForMemCpy;
            osg::Timer_t            _previousFrameTick;

            osg::ref_ptr< osg::Image > _lastImage;
        };
    
        WindowCaptureCallback(Mode mode, FramePosition position, GLenum readBuffer):
            _mode(mode),
            _position(position),
            _readBuffer(readBuffer)
        {
        }

        FramePosition getFramePosition() const { return _position; }

        ContextData* createContextData(osg::GraphicsContext* gc) const
        {
            std::stringstream filename;
            filename << "test_"<<_contextDataMap.size()<<".jpg";
            return new ContextData(gc, _mode, _readBuffer, filename.str());
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

        Mode                        _mode;        
        FramePosition               _position;
        GLenum                      _readBuffer;
        mutable OpenThreads::Mutex  _mutex;
        mutable ContextDataMap      _contextDataMap;
        osg::ref_ptr< osg::Image>   _image;
        
        
};

void WindowCaptureCallback::ContextData::updateTimings(osg::Timer_t tick_start,
                                                       osg::Timer_t tick_afterReadPixels,
                                                       osg::Timer_t tick_afterMemCpy,
                                                       unsigned int dataSize)
{
    if (!_reportTimingFrequency) return;

    double timeForReadPixels = osg::Timer::instance()->delta_s(tick_start, tick_afterReadPixels);
    double timeForFullCopy = osg::Timer::instance()->delta_s(tick_start, tick_afterMemCpy);
    double timeForMemCpy = osg::Timer::instance()->delta_s(tick_afterReadPixels, tick_afterMemCpy);

    _timeForReadPixels += timeForReadPixels;
    _timeForFullCopy += timeForFullCopy;
    _timeForMemCpy += timeForMemCpy;
    
    ++_numTimeValuesRecorded;
    
    if (_numTimeValuesRecorded==_reportTimingFrequency)
    {
        timeForReadPixels = _timeForReadPixels/double(_numTimeValuesRecorded);
        timeForFullCopy = _timeForFullCopy/double(_numTimeValuesRecorded);
        timeForMemCpy = _timeForMemCpy/double(_numTimeValuesRecorded);
        
        double averageFrameTime =  osg::Timer::instance()->delta_s(_previousFrameTick, tick_afterMemCpy)/double(_numTimeValuesRecorded);
        double fps = 1.0/averageFrameTime;    
        _previousFrameTick = tick_afterMemCpy;

        _timeForReadPixels = 0.0;
        _timeForFullCopy = 0.0;
        _timeForMemCpy = 0.0;

        _numTimeValuesRecorded = 0;

        double numMPixels = double(_width * _height) / 1000000.0;
        double numMb = double(dataSize) / (1024*1024);

        int prec = osg::notify(osg::NOTICE).precision(5);

        if (timeForMemCpy==0.0)
        {
            osg::notify(osg::NOTICE)<<"fps = "<<fps<<", full frame copy = "<<timeForFullCopy*1000.0f<<"ms rate = "<<numMPixels / timeForFullCopy<<" Mpixel/sec, copy speed = "<<numMb / timeForFullCopy<<" Mb/sec"<<std::endl;
        }
        else
        {
            osg::notify(osg::NOTICE)<<"fps = "<<fps<<", full frame copy = "<<timeForFullCopy*1000.0f<<"ms rate = "<<numMPixels / timeForFullCopy<<" Mpixel/sec, "<<numMb / timeForFullCopy<< " Mb/sec "<<
                                      "time for memcpy = "<<timeForMemCpy*1000.0<<"ms  memcpy speed = "<<numMb / timeForMemCpy<<" Mb/sec"<<std::endl;
        }
        osg::notify(osg::NOTICE).precision(prec);

    }

}

void WindowCaptureCallback::ContextData::readPixels()
{
    // std::cout<<"readPixels("<<_fileName<<" image "<<_currentImageIndex<<" "<<_currentPboIndex<<std::endl;

    unsigned int nextImageIndex = (_currentImageIndex+1)%_imageBuffer.size();
    unsigned int nextPboIndex = _pboBuffer.empty() ? 0 : (_currentPboIndex+1)%_pboBuffer.size();

    int width=0, height=0;
    getSize(_gc, width, height);
    if (width!=_width || _height!=height)
    {
        std::cout<<"   Window resized "<<width<<", "<<height<<std::endl;
        _width = width;
        _height = height;
    }

    osg::Image* image = _imageBuffer[_currentImageIndex].get();

    osg::Timer_t tick_start = osg::Timer::instance()->tick();

#if 1
    image->readPixels(0,0,_width,_height,
                      _pixelFormat,_type);
#endif

    osg::Timer_t tick_afterReadPixels = osg::Timer::instance()->tick();

    updateTimings(tick_start, tick_afterReadPixels, tick_afterReadPixels, image->getTotalSizeInBytes());

    if (!_fileName.empty())
    {
        // osgDB::writeImageFile(*image, _fileName);
    }

    _currentImageIndex = nextImageIndex;
    _currentPboIndex = nextPboIndex;
}

void WindowCaptureCallback::ContextData::singlePBO(osg::GLExtensions* ext)
{
    // std::cout<<"singelPBO(  "<<_fileName<<" image "<<_currentImageIndex<<" "<<_currentPboIndex<<std::endl;

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

    osg::Timer_t tick_start = osg::Timer::instance()->tick();

#if 1
    glReadPixels(0, 0, _width, _height, _pixelFormat, _type, 0);
#endif

    osg::Timer_t tick_afterReadPixels = osg::Timer::instance()->tick();

    GLubyte* src = (GLubyte*)ext->glMapBuffer(GL_PIXEL_PACK_BUFFER_ARB,
                                              GL_READ_ONLY_ARB);
    if(src)
    {
        memcpy(image->data(), src, image->getTotalSizeInBytes());
        
        ext->glUnmapBuffer(GL_PIXEL_PACK_BUFFER_ARB);
    }

    ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, 0);

    osg::Timer_t tick_afterMemCpy = osg::Timer::instance()->tick();

    updateTimings(tick_start, tick_afterReadPixels, tick_afterMemCpy, image->getTotalSizeInBytes());

    OE_NOTICE << "Saving image" << std::endl;
    //osgDB::writeImageFile(*image, "test.png");
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

static std::string response(std::string const &content
    , std::string const &content_type= "text/plain"
    , std::string const &status= "200 OK"
    , std::string const &cookie= "")
  {
    std::ostringstream out;
    out<< "HTTP/1.1 "<< status<< "\r\n"
      << "Content-Type: "<< content_type<< "; charset=utf-8\r\n"
      << "Content-Length: "<< content.size()<< "\r\n"
      << "Connection: keep-alive\r\n"
      << "Cache-Control: no-cache\r\n"
      << "Access-Control-Allow-Origin: *\r\n";
    if(!cookie.empty())
      out<< "Set-Cookie: "<< cookie<< ";max-age=315569260\r\n";
    out<< "Server: osgearth_server\r\n\r\n"
      << content;
    return out.str();
  }


struct CloneImageHandler : public osgViewer::ScreenCaptureHandler::CaptureOperation
{
    virtual void operator()(const osg::Image& image, const unsigned int context_id)
    {        
        setImage(image);
    }

    void setImage(const osg::Image &image)
    {
        _image = new osg::Image(image);
    }

    osg::Image* getImage()
    {     
        if (_image.valid())
        {
            OE_NOTICE << "Returning image" << std::endl;
            return new osg::Image(*_image.get());
        }
        return 0;
    }

    osg::ref_ptr< osg::Image > _image;
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
          //_viewer->setUpViewInWindow(0,0,256,256);
          _viewer->getCamera()->setSmallFeatureCullingPixelSize(-1.0f);
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

          /*
          _screenCaptureHandler = new ScreenCaptureHandler;
          _cloneImageHandler = new CloneImageHandler();
          _screenCaptureHandler->setCaptureOperation(_cloneImageHandler.get());
          _viewer->addEventHandler(_screenCaptureHandler);
          */

          _windowCaptureCallback = new WindowCaptureCallback(WindowCaptureCallback::SINGLE_PBO, WindowCaptureCallback::END_FRAME, GL_BACK);

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

          OE_NOTICE << "key=" << key.str() << std::endl;

          // Set the projection matrix to capture the tile.                    
          OE_NOTICE << "Key extent " << z << "(" << x << ", " << y << ") = " << key.getExtent().toString() << std::endl;
          _viewer->getCamera()->setProjectionMatrixAsOrtho2D(key.getExtent().xMin(), key.getExtent().xMax(), key.getExtent().yMin(), key.getExtent().yMax());
          _viewer->frame();
          while (_viewer->getDatabasePager()->getRequestsInProgress())
          {
              OE_NOTICE << "Waiting...." << std::endl;
              _viewer->frame();
          }

          OE_NOTICE << "Snap!" << std::endl;

          _viewer->getCamera()->setFinalDrawCallback(_windowCaptureCallback);
          _viewer->frame();
          _viewer->frame();
          _viewer->getCamera()->setFinalDrawCallback( 0 );
         

           //return _cloneImageHandler->getImage();
          if (_windowCaptureCallback->_image)
          {
              return new osg::Image(*_windowCaptureCallback->_image);
          }
          return 0;
      }

    osgViewer::Viewer *_viewer;
    osg::ref_ptr< MapNode > _mapNode;
    osg::ref_ptr< osg::Group > _root;
    osg::ref_ptr< CloneImageHandler > _cloneImageHandler;
    osg::ref_ptr< ScreenCaptureHandler > _screenCaptureHandler;
    osg::ref_ptr< WindowCaptureCallback > _windowCaptureCallback;
    OpenThreads::Mutex _mutex;
};


static TileImageServer* _server;


static void ev_handler(struct mg_connection *nc, int ev, void *ev_data) {
  struct http_message *hm = (struct http_message *) ev_data;

  static OpenThreads::Mutex requestMutex;
  OpenThreads::ScopedLock< OpenThreads::Mutex > lk(requestMutex);
  
  switch (ev) {
  case MG_EV_HTTP_REQUEST:
      {
          std::string url(hm->uri.p, hm->uri.len);
          OE_NOTICE << "url=" << url << std::endl;
          StringTokenizer tok("/");
          StringVector tized;
          tok.tokenize(url, tized);            
          if ( tized.size() == 4 )
          {
              int z = as<int>(tized[1], 0);
              int x = as<int>(tized[2], 0);
              unsigned int y = as<int>(osgDB::getNameLessExtension(tized[3]),0);
              std::string ext = osgDB::getFileExtension(tized[3]);
              std::cout << "z=" << z << std::endl;
              std::cout << "x=" << x << std::endl;
              std::cout << "y=" << y << std::endl;              
              std::cout << "ext=" << ext << std::endl;

              osg::ref_ptr< osg::Image > image = _server->getTile(z, x, y );

              bool sent = false;
              if (image.valid())
              {              

                  osgDB::ReaderWriter* rw = osgDB::Registry::instance()->getReaderWriterForExtension(ext);
                  if (rw)
                  {
                      std::stringstream buf;
                      rw->writeImage(*image.get(), buf);
                      std::string mime = "image/png";
                      if (ext == "jpeg" || ext == "jpg")
                      {
                          mime = "image/jpeg";
                      }
                      std::string res = response(buf.str(), mime);
                      mg_send(nc, res.c_str(), res.size());       
                      //mg_send_http_chunk(nc, res.c_str(), res.size());
                      sent = true;
                  }             
              }

              if (!sent)
              {
                  std::string res = response("", "", "404 Not Found");
                  mg_send(nc, res.c_str(), res.size());
              }

              nc->flags |= MG_F_SEND_AND_CLOSE;
          }
          else
          {
              std::string res = response("", "", "404 Not Found");
              mg_send(nc, res.c_str(), res.size());
          }
      }
      break;
    default:
      break;
  }
}



int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    // help?
    if ( arguments.read("--help") )
        return usage(argv[0]);

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

   
    struct mg_connection *nc;
    struct mg_mgr mgr;

    mg_mgr_init(&mgr, NULL);

    // Note that many connections can be added to a single event manager
    // Connections can be created at any point, e.g. in event handler function
    nc = mg_bind(&mgr, "8000", ev_handler);
    mg_set_protocol_http_websocket(nc);

    for (;;) {
        mg_mgr_poll(&mgr, 1000);
    }

    mg_mgr_free(&mgr);

    return 0;
}
