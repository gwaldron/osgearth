#include "OsgMainApp.hpp"

#include <osgDB/FileUtils>
#include <osgEarth/Capabilities>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/ObjectLocator>
#include <osgEarthDrivers/tms/TMSOptions>

#include <osgDB/WriteFile>

#include "GLES2ShaderGenVisitor.h"

using namespace osgEarth;
using namespace osgEarth::Drivers;
using namespace osgEarth::Util;

class ClampObjectLocatorCallback : public osgEarth::TerrainCallback
{
public:
    ClampObjectLocatorCallback(ObjectLocatorNode* locator):
    _locator(locator),
    _maxLevel(-1),
    _minLevel(0)
    {
    }
    
    virtual void onTileAdded(const osgEarth::TileKey& tileKey, osg::Node* terrain, TerrainCallbackContext&)
    {
        if ((int)tileKey.getLevelOfDetail() > _minLevel && _maxLevel < (int)tileKey.getLevelOfDetail())
        {
            osg::Vec3d position = _locator->getLocator()->getPosition();
            
            if (tileKey.getExtent().contains(position.x(), position.y()))
            {
                //Compute our location in geocentric
                const osg::EllipsoidModel* ellipsoid = tileKey.getProfile()->getSRS()->getEllipsoid();
                double x, y, z;
                ellipsoid->convertLatLongHeightToXYZ(
                                                     osg::DegreesToRadians(position.y()), osg::DegreesToRadians(position.x()), 0,
                                                     x, y, z);
                //Compute the up vector
                osg::Vec3d up = ellipsoid->computeLocalUpVector(x, y, z );
                up.normalize();
                osg::Vec3d world(x, y, z);
                
                double segOffset = 50000;
                
                osg::Vec3d start = world + (up * segOffset);
                osg::Vec3d end = world - (up * segOffset);
                
                osgUtil::LineSegmentIntersector* i = new osgUtil::LineSegmentIntersector( start, end );
                
                osgUtil::IntersectionVisitor iv;
                iv.setIntersector( i );
                terrain->accept( iv );
                
                osgUtil::LineSegmentIntersector::Intersections& results = i->getIntersections();
                if ( !results.empty() )
                {
                    const osgUtil::LineSegmentIntersector::Intersection& result = *results.begin();
                    osg::Vec3d hit = result.getWorldIntersectPoint();
                    double lat, lon, height;
                    ellipsoid->convertXYZToLatLongHeight(hit.x(), hit.y(), hit.z(),
                                                         lat, lon, height);
                    position.z() = height;
                    //OE_NOTICE << "Got hit, setting new height to " << height << std::endl;
                    _maxLevel = tileKey.getLevelOfDetail();
                    _locator->getLocator()->setPosition( position );
                }
            }
        }
        
    }
    
    osg::ref_ptr< ObjectLocatorNode > _locator;
    int _maxLevel;
    int _minLevel;
};


OsgMainApp::OsgMainApp(){

    _initialized = false;

}
OsgMainApp::~OsgMainApp(){

}


//Initialization function
void OsgMainApp::initOsgWindow(int x,int y,int width,int height){

    __android_log_write(ANDROID_LOG_ERROR, "OSGANDROID",
            "Initializing geometry");

    //Pending
    _notifyHandler = new OsgAndroidNotifyHandler();
    _notifyHandler->setTag("Osg Viewer");
    osg::setNotifyHandler(_notifyHandler);
    osgEarth::setNotifyHandler(_notifyHandler);
    
    osg::setNotifyLevel(osg::FATAL);
    osgEarth::setNotifyLevel(osg::INFO);

    osg::notify(osg::ALWAYS)<<"Testing"<<std::endl;

    //::setenv("OSGEARTH_HTTP_DEBUG", "1", 1);
    //::setenv("OSGEARTH_DUMP_SHADERS", "1", 1);
    
    osgEarth::Registry::instance()->setDefaultTerrainEngineDriverName("quadtree");
    
    _bufferWidth = width;
    _bufferHeight = height;
    
    _viewer = new osgViewer::Viewer();
    _viewer->setUpViewerAsEmbeddedInWindow(x, y, width, height);
    _viewer->getCamera()->setViewport(new osg::Viewport(0, 0, width, height));
    _viewer->getCamera()->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    _viewer->getCamera()->setClearColor(osg::Vec4(1.0f,0.0f,0.0f,0.0f));
    //_viewer->getCamera()->setClearStencil(0);
    _viewer->getCamera()->setProjectionMatrixAsPerspective(45.0f,(float)width/height,
                                                           0.1f, 10000.0f);
    // configure the near/far so we don't clip things that are up close
    _viewer->getCamera()->setNearFarRatio(0.00002);
    _viewer->setThreadingModel(osgViewer::ViewerBase::SingleThreaded);

    _viewer->getEventQueue()->getCurrentEventState()->setMouseYOrientation(osgGA::GUIEventAdapter::Y_INCREASING_UPWARDS);

    // install our default manipulator (do this before calling load)
    //_viewer->setCameraManipulator( new osgEarth::Util::EarthMultiTouchManipulator() );
    _viewer->setCameraManipulator( new osgEarth::Util::EarthManipulator() ); //EarthMultiTouchManipulator() );
    
#if 0
    osg::Light* light = new osg::Light( 0 );
    light->setPosition( osg::Vec4(0, -1, 0, 0 ) );
    light->setAmbient( osg::Vec4(0.4f, 0.4f, 0.4f ,1.0) );
    light->setDiffuse( osg::Vec4(1,1,1,1) );
    light->setSpecular( osg::Vec4(0,0,0,1) );
    
    osg::Material* material = new osg::Material();
    material->setAmbient(osg::Material::FRONT, osg::Vec4(0.4,0.4,0.4,1.0));
    material->setDiffuse(osg::Material::FRONT, osg::Vec4(0.9,0.9,0.9,1.0));
    material->setSpecular(osg::Material::FRONT, osg::Vec4(0.4,0.4,0.4,1.0));
#endif
    
    osg::Node* node = osgDB::readNodeFile("http://readymap.org/readymap/maps/public/2.earth");
    if ( !node )
    {
        OSG_ALWAYS << "Unable to load an earth file from the command line." << std::endl;
        return;
    }
    
    osg::ref_ptr<osgEarth::Util::MapNode> mapNode = osgEarth::Util::MapNode::findMapNode(node);
    if ( !mapNode.valid() )
    {
        OSG_ALWAYS << "Loaded scene graph does not contain a MapNode - aborting" << std::endl;
        return;
    }
    
    // warn about not having an earth manip
    osgEarth::Util::EarthManipulator* manip = dynamic_cast<osgEarth::Util::EarthManipulator*>(_viewer->getCameraManipulator());
    if ( manip == 0L )
    {
        OSG_ALWAYS << "Helper used before installing an EarthManipulator" << std::endl;
    }
    
    // a root node to hold everything:
    osg::Group* root = new osg::Group();
    root->addChild( mapNode.get() );
    //root->getOrCreateStateSet()->setAttribute(light);
    
#if 0
    //have to add these
    root->getOrCreateStateSet()->setAttribute(material);
    //root->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
#endif
    double hours = 12.0f;
    float ambientBrightness = 0.4f;
    osgEarth::Util::SkyNode* sky = new osgEarth::Util::SkyNode( mapNode->getMap() );
    sky->setAmbientBrightness( ambientBrightness );
    sky->setDateTime( 1984, 11, 8, hours );
    sky->attach( _viewer );
    root->addChild( sky );


    root->getOrCreateStateSet()->setMode(GL_LIGHTING,
    		osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
    
    //for some reason we have to do this as global stateset doesn't
    //appear to be in the statesetstack
    root->getOrCreateStateSet()->setAttribute(_viewer->getLight());
    
    _viewer->setSceneData( root );

    _viewer->setRunFrameScheme( osgViewer::ViewerBase::ON_DEMAND );

    _viewer->realize();

    _initialized = true;

}
//Draw
void OsgMainApp::draw(){

    _viewer->frame();
    
    //clear events for next frame
    _frameTouchBeganEvents = NULL;
    _frameTouchMovedEvents = NULL;
    _frameTouchEndedEvents = NULL;
}
//Events
static bool flipy = true;
void OsgMainApp::touchBeganEvent(int touchid,float x,float y){
    if (!_frameTouchBeganEvents.valid()) {
        if(_viewer.valid()){
            _frameTouchBeganEvents = _viewer->getEventQueue()->touchBegan(touchid, osgGA::GUIEventAdapter::TOUCH_BEGAN, x, flipy ? _bufferHeight-y : y);
        }
    } else {
        _frameTouchBeganEvents->addTouchPoint(touchid, osgGA::GUIEventAdapter::TOUCH_BEGAN, x, flipy ? _bufferHeight-y : y);
    }
}
void OsgMainApp::touchMovedEvent(int touchid,float x,float y){
    if (!_frameTouchMovedEvents.valid()) {
        if(_viewer.valid()){
            _frameTouchMovedEvents = _viewer->getEventQueue()->touchMoved(touchid, osgGA::GUIEventAdapter::TOUCH_MOVED, x, flipy ? _bufferHeight-y : y);
        }
    } else {
        _frameTouchMovedEvents->addTouchPoint(touchid, osgGA::GUIEventAdapter::TOUCH_MOVED, x, flipy ? _bufferHeight-y : y);
    }
}
void OsgMainApp::touchEndedEvent(int touchid,float x,float y,int tapcount){
    if (!_frameTouchEndedEvents.valid()) {
        if(_viewer.valid()){
            _frameTouchEndedEvents = _viewer->getEventQueue()->touchEnded(touchid, osgGA::GUIEventAdapter::TOUCH_ENDED, x, flipy ? _bufferHeight-y : y,tapcount);
        }
    } else {
        _frameTouchEndedEvents->addTouchPoint(touchid, osgGA::GUIEventAdapter::TOUCH_ENDED, x, flipy ? _bufferHeight-y : y,tapcount);
    }
}
void OsgMainApp::keyboardDown(int key){
    _viewer->getEventQueue()->keyPress(key);
}
void OsgMainApp::keyboardUp(int key){
    _viewer->getEventQueue()->keyRelease(key);
}

void OsgMainApp::clearEventQueue()
{
    //clear our groups
    _frameTouchBeganEvents = NULL;
    _frameTouchMovedEvents = NULL;
    _frameTouchEndedEvents = NULL;
    
    //clear the viewers queue
    _viewer->getEventQueue()->clear();
}
