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
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgDB/ReaderWriter>
#include <osgDB/Registry>

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
  
  switch (ev) {
    case MG_EV_HTTP_REQUEST:
        std::cout << hm->uri.p << std::endl;
      if (mg_vcmp(&hm->uri, "/osgearth") == 0) {
        std::cout << "osgearth" << std::endl;
        osg::ref_ptr< osg::Image > image = getImage();
       
        osgDB::ReaderWriter* rw = osgDB::Registry::instance()->getReaderWriterForExtension("jpg");
        if (rw)
        {
            std::stringstream buf;
            rw->writeImage(*image.get(), buf);
            std::string res = response(buf.str(), "image/jpeg");
            mg_send(nc, res.c_str(), res.size());       
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
        OSG_NOTICE << "Capturing..." << std::endl;
        // Just clone the image
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
    viewer.setCameraManipulator( new EarthManipulator(arguments) );

    // disable the small-feature culling
    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

    // set a near/far ratio that is smaller than the default. This allows us to get
    // closer to the ground without near clipping. If you need more, use --logdepth
    viewer.getCamera()->setNearFarRatio(0.0001);

    /*

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
    */

    ServerThread server;
    server.start();

    osg::ref_ptr< ScreenCaptureHandler > screenCaptureHandler = new ScreenCaptureHandler;
    screenCaptureHandler->setCaptureOperation(new CloneImageHandler());
    viewer.addEventHandler(screenCaptureHandler.get());   
    screenCaptureHandler->setFramesToCapture(-1);
    screenCaptureHandler->captureNextFrame(viewer);
    screenCaptureHandler->startCapture();
    

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Node* node = MapNodeHelper().load( arguments, &viewer );
    if ( node )
    {
        viewer.setSceneData( node );
        while(!viewer.done())
        {
            viewer.frame();
        }
    }
    else
    {
        return usage(argv[0]);
    }

    return 0;
}
