#include "OsgMainApp.hpp"

#include <osgEarth/MapNode>
#include <osgEarthUtil/Sky>
#include <osgEarthUtil/EarthManipulator>

#include "osgPlugins.h"

OsgMainApp::OsgMainApp()
{
    _initialized = false;

}
OsgMainApp::~OsgMainApp()
{

}

//Initialization function
void OsgMainApp::initOsgWindow(int x,int y,int width,int height)
{
    __android_log_write(ANDROID_LOG_ERROR, "OSGANDROID",
            "Initializing geometry");

    //Pending
    OsgAndroidNotifyHandler* _notifyHandler = new OsgAndroidNotifyHandler();
    _notifyHandler->setTag("Osg Viewer");
    osg::setNotifyHandler(_notifyHandler);

    osg::notify(osg::ALWAYS)<<"Testing"<<std::endl;

    _viewer = new osgViewer::Viewer();

    _viewer->setUpViewerAsEmbeddedInWindow(0, 0, width, height);
	_viewer->getCamera()->setViewport(new osg::Viewport(0, 0, width, height));
	_viewer->getCamera()->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	_viewer->getCamera()->setClearColor(osg::Vec4(0.0f, 0.0f, 0.5f, 1.0f));
	_viewer->getCamera()->setProjectionMatrixAsPerspective(45.0f, (float)(width / height), 0.1f, 1000.0f);
	_viewer->getEventQueue()->getCurrentEventState()->setMouseYOrientation(osgGA::GUIEventAdapter::Y_INCREASING_UPWARDS);
	_viewer->getCamera()->setNearFarRatio(0.00002f);
	_viewer->setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
	_viewer->getCamera()->setLODScale(_viewer->getCamera()->getLODScale() * 1.5f);
	_viewer->getDatabasePager()->setIncrementalCompileOperation(new osgUtil::IncrementalCompileOperation());
	_viewer->getDatabasePager()->setDoPreCompile(true);
	_viewer->getDatabasePager()->setTargetMaximumNumberOfPageLOD(0);
	_viewer->getDatabasePager()->setUnrefImageDataAfterApplyPolicy(true, true);
	_viewer->setCameraManipulator(new osgEarth::Util::EarthManipulator());
	// _viewer->setRunFrameScheme( osgViewer::ViewerBase::ON_DEMAND );

	std::string filepath = "/sdcard/Download/readymap.earth";
	osg::Node* node = osgDB::readNodeFile(filepath);

	if(!node) {
		OSG_ALWAYS << "Unable to load an earth file from the command line." << std::endl;
		return;
	}

	osg::ref_ptr<osgEarth::Util::MapNode> mapNode = osgEarth::Util::MapNode::findMapNode(node);

	if(!mapNode.valid()) {
		OSG_ALWAYS << "Loaded scene graph does not contain a MapNode - aborting" << std::endl;

		return;
	}

	_viewer->setSceneData(mapNode.get());

    _viewer->realize();

    _initialized = true;

}

//Draw
void OsgMainApp::draw()
{
    _viewer->frame();
}

//Events
void OsgMainApp::mouseButtonPressEvent(float x,float y,int button)
{
    _viewer->getEventQueue()->mouseButtonPress(x, y, button);
}
void OsgMainApp::mouseButtonReleaseEvent(float x,float y,int button)
{
    _viewer->getEventQueue()->mouseButtonRelease(x, y, button);
}
void OsgMainApp::mouseMoveEvent(float x,float y)
{
    _viewer->getEventQueue()->mouseMotion(x, y);
}
void OsgMainApp::keyboardDown(int key)
{
    _viewer->getEventQueue()->keyPress(key);
}
void OsgMainApp::keyboardUp(int key)
{
    _viewer->getEventQueue()->keyRelease(key);
}
