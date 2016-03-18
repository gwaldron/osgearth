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
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>

#include "mongoose.h"

#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgViewer;

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
      _viewer(0)
      {
          _viewer = new osgViewer::Viewer;
          _viewer->setUpViewInWindow(0,0,256,256);
          _viewer->getCamera()->setSmallFeatureCullingPixelSize(-1.0f);
          float half = 256.0/2.0;
          _viewer->getCamera()->setProjectionMatrixAsOrtho2D(-half, half, -half, half);

          _screenCaptureHandler = new ScreenCaptureHandler;
          _cloneImageHandler = new CloneImageHandler();
          _screenCaptureHandler->setCaptureOperation(_cloneImageHandler.get());
          _viewer->addEventHandler(_screenCaptureHandler);

          _root = new osg::Group;

          _root->addChild(mapNode);

          _viewer->setSceneData( _root.get() );

          _viewer->realize();
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



          // Remove all the children

#if 0
          _root->removeChildren(0, _root->getNumChildren());

          osg::ref_ptr< osg::Node > tile = _mapNode->getTerrainEngine()->createTile( key );
          if (!tile.valid())
          {
              return 0;
          }

          _root->addChild( tile );
#endif

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

          // Actually take a shot of the tile.
           _screenCaptureHandler->setFramesToCapture(1);
           _screenCaptureHandler->captureNextFrame(*_viewer);
           _viewer->frame();
           _viewer->frame();

           return _cloneImageHandler->getImage();
      }

    osgViewer::Viewer *_viewer;
    osg::ref_ptr< MapNode > _mapNode;
    osg::ref_ptr< osg::Group > _root;
    osg::ref_ptr< CloneImageHandler > _cloneImageHandler;
    osg::ref_ptr< ScreenCaptureHandler > _screenCaptureHandler;
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
