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

#include "mongoose.h"

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
          _viewer->getCamera()->setProjectionMatrixAsOrtho2D(key.getExtent().xMin(), key.getExtent().xMax(), key.getExtent().yMin(), key.getExtent().yMax());
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


static void ev_handler(struct mg_connection *nc, int ev, void *ev_data) {
  struct http_message *hm = (struct http_message *) ev_data;

  static OpenThreads::Mutex requestMutex;
  OpenThreads::ScopedLock< OpenThreads::Mutex > lk(requestMutex);
  
  switch (ev) {
  case MG_EV_HTTP_REQUEST:
      {
          std::string url(hm->uri.p, hm->uri.len);
          OE_DEBUG << "url=" << url << std::endl;
          StringTokenizer tok("/");
          StringVector tized;
          tok.tokenize(url, tized);            
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

    int port = 8000;
    if (arguments.read("--port", port));
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
   
    struct mg_connection *nc;
    struct mg_mgr mgr;

    mg_mgr_init(&mgr, NULL);

    // Note that many connections can be added to a single event manager
    // Connections can be created at any point, e.g. in event handler function    
    std::string portStr = toString<unsigned int>(port);

    nc = mg_bind(&mgr, portStr.c_str(), ev_handler);
    mg_set_protocol_http_websocket(nc);

    for (;;) {
        mg_mgr_poll(&mgr, 1000);
    }

    mg_mgr_free(&mgr);

    return 0;
}
