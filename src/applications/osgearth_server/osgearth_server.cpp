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
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgDB/ReaderWriter>
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
      << "Connection: keep-alive\r\n";
    if(!cookie.empty())
      out<< "Set-Cookie: "<< cookie<< ";max-age=315569260\r\n";
    out<< "Server: osgearth_server\r\n\r\n"
      << content;
    return out.str();
  }

static osg::ref_ptr< osg::Image > s_image;
static OpenThreads::Mutex s_imageMutex;
static osg::ref_ptr< EarthManipulator > earthManipulator;
static osgViewer::Viewer* s_viewer;
static osg::ref_ptr< ScreenCaptureHandler > s_captureHandler;
static osg::ref_ptr< MapNode > s_mapNode;

static void setImage(const osg::Image &image)
{
    OpenThreads::ScopedLock< OpenThreads::Mutex > lk(s_imageMutex);
    s_image = new osg::Image(image);
}

static osg::Image* getImage()
{
    OpenThreads::ScopedLock< OpenThreads::Mutex > lk(s_imageMutex);
    return new osg::Image(*s_image.get());
}

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
          OE_NOTICE << "url=" << url << " size=" << tized.size() << std::endl;
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

              TileKey key(z, x, y, osgEarth::Registry::instance()->getGlobalGeodeticProfile());

              osgEarth::GeoPoint centroid;
              key.getExtent().getCentroid(centroid);
              double lon   = centroid.x(); 
              double lat   = centroid.y(); 
              double maxDim = osg::maximum(key.getExtent().width() , key.getExtent().height()); 
              double range = ((0.5 * maxDim) / 0.267949849) * 111000.0; 
              //double range = ((0.5 * maxDim) / 0.267949849); 
              OE_NOTICE << "range=" << range << std::endl;
              Viewpoint vp;
              vp.focalPoint() = centroid;
              vp.range()->set(range, Units::METERS);
              earthManipulator->setViewpoint(vp, 0.0);
              s_viewer->frame();
              s_viewer->frame();

              OE_NOTICE << "Centroid" << centroid.x() << ", " << centroid.y() << std::endl;

              s_captureHandler->setFramesToCapture(1);
              s_captureHandler->captureNextFrame(*s_viewer);
              s_viewer->frame();
              s_viewer->frame();
              s_viewer->frame();

              osg::ref_ptr< osg::Image > image = getImage();

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
              }             
          }
      }
      break;
    default:
      break;
  }
}

class ServerThread : public OpenThreads::Thread
{
    virtual void run()
    {
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
  }
};

class CloneImageHandler : public osgViewer::ScreenCaptureHandler::CaptureOperation
{
    virtual void operator()(const osg::Image& image, const unsigned int context_id)
    {        
        setImage(image);
    }
};

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    // help?
    if ( arguments.read("--help") )
        return usage(argv[0]);

    // create a viewer:
    osgViewer::Viewer viewer(arguments);

    // Tell the database pager to not modify the unref settings
    viewer.getDatabasePager()->setUnrefImageDataAfterApplyPolicy( false, false );

    // thread-safe initialization of the OSG wrapper manager. Calling this here
    // prevents the "unsupported wrapper" messages from OSG
    osgDB::Registry::instance()->getObjectWrapperManager()->findWrapper("osg::Image");

    // install our default manipulator (do this before calling load)
    earthManipulator = new EarthManipulator(arguments);
    viewer.setCameraManipulator( earthManipulator );

    // disable the small-feature culling
    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

    // set a near/far ratio that is smaller than the default. This allows us to get
    // closer to the ground without near clipping. If you need more, use --logdepth
    viewer.getCamera()->setNearFarRatio(0.0001);

    s_viewer = &viewer;
    

        /*
        ServerThread server;
        server.start();
    */

    s_captureHandler = new ScreenCaptureHandler;
    s_captureHandler->setCaptureOperation(new CloneImageHandler());
    viewer.addEventHandler(s_captureHandler.get());   
    /*
    s_captureHandler->setFramesToCapture(-1);
    s_captureHandler->captureNextFrame(viewer);
    screenCaptureHandler->startCapture();
    */
    

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Node* node = MapNodeHelper().load( arguments, &viewer );
    if ( node )
    {
        viewer.setSceneData( node );

        /*
        while(!viewer.done())
        {
            viewer.frame();
        }
        */
    }
    else
    {
        return usage(argv[0]);
    }

    viewer.realize();


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
