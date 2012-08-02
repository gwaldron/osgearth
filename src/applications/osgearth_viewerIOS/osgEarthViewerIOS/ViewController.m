//
//  ViewController.m
//  osgEarthViewerIOS
//
//  Created by Thomas Hogarth on 14/07/2012.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#import "ViewController.h"

#include "osgPlugins.h"

#include <osgDB/FileUtils>

#include <osgGA/MultiTouchTrackballManipulator>

#include <osgViewer/api/IOS/GraphicsWindowIOS>

#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>

#include "EarthMultiTouchManipulator.h"
#include "GLES2ShaderGenVisitor.h"



@interface ViewController () {

}

@end


@implementation ViewController


- (void)dealloc
{
    OSG_ALWAYS << "dealloc" << std::endl;
    [super dealloc];
    
    [self stopAnimation];
    _viewer = NULL;
}

- (void)loadBasicScene{
    
    _viewer->setCameraManipulator(new osgGA::MultiTouchTrackballManipulator());
    
    osg::Node* node = osgDB::readNodeFile(osgDB::findDataFile("models/box.osg"));
    
    osgUtil::GLES2ShaderGenVisitor shaderGen;
    node->accept(shaderGen);
    
    _viewer->setSceneData(node);

}

- (void)loadOsgEarthDemoScene{

    // install our default manipulator (do this before calling load)
    _viewer->setCameraManipulator( new osgEarth::Util::EarthMultiTouchManipulator() );
    
    osg::Node* node = osgDB::readNodeFile(osgDB::findDataFile("tests/readymap.earth"));
    if ( !node )
    {
        OSG_WARN << "Unable to load an earth file from the command line." << std::endl;
        return;
    }
    
    osg::ref_ptr<osgEarth::Util::MapNode> mapNode = osgEarth::Util::MapNode::findMapNode(node);
    if ( !mapNode.valid() )
    {
        OSG_WARN << "Loaded scene graph does not contain a MapNode - aborting" << std::endl;
        return;
    }
    
    // warn about not having an earth manip
    osgEarth::Util::EarthManipulator* manip = dynamic_cast<osgEarth::Util::EarthManipulator*>(_viewer->getCameraManipulator());
    if ( manip == 0L )
    {
        OSG_WARN << "Helper used before installing an EarthManipulator" << std::endl;
    }
    
    // a root node to hold everything:
    osg::Group* root = new osg::Group();
    root->addChild( mapNode.get() );
    
    
    osgUtil::GLES2ShaderGenVisitor shaderGen;
    //root->accept(shaderGen);
    
    _viewer->setSceneData( root );
}

- (void)viewDidLoad
{
    [super viewDidLoad];
    
    osg::setNotifyLevel(osg::FATAL);
    //osgEarth::setNotifyLevel(osg::DEBUG_FP);
    
    //get screen scale
    UIScreen* screen = [UIScreen mainScreen];
    float scale = 1.0f;
#if defined(__IPHONE_4_0) && (__IPHONE_OS_VERSION_MIN_REQUIRED >= __IPHONE_4_0)
    scale = [screen scale];
#endif

	CGRect lFrame = [screen bounds];//[self.view bounds];
	unsigned int w = lFrame.size.width;
	unsigned int h = lFrame.size.height; 
    
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    
    //create the viewer
	_viewer = new osgViewer::Viewer();
    //_viewer->setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    
	// Setup the traits parameters
	traits->x = 0;
	traits->y = 0;
	traits->width = w*scale;
	traits->height = h*scale;
	traits->depth = 24; //keep memory down, default is currently 24
	traits->alpha = 8;
    //traits->samples = 4;
    //traits->sampleBuffers = 2;
	//traits->stencil = 1;
	traits->windowDecoration = false;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->setInheritedWindowPixelFormat = true;
    
	osg::ref_ptr<osg::Referenced> windata = new osgViewer::GraphicsWindowIOS::WindowData(self.view, osgViewer::GraphicsWindowIOS::WindowData::PORTRAIT_ORIENTATION, scale);
	traits->inheritedWindowData = windata;
    
	// Create the Graphics Context
	osg::ref_ptr<osg::GraphicsContext> graphicsContext = osg::GraphicsContext::createGraphicsContext(traits.get());
	
	if(graphicsContext)
	{
        _viewer->getCamera()->setGraphicsContext(graphicsContext);
    }
    
    _viewer->getCamera()->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
    _viewer->getCamera()->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    _viewer->getCamera()->setClearColor(osg::Vec4(1.0f,0.0f,0.0f,0.0f));
    _viewer->getCamera()->setProjectionMatrixAsPerspective(45.0f,(float)w/h,
                                                           0.1f, 10000.0f);


    //load
    [self loadOsgEarthDemoScene];
    //[self loadBasicScene];
    
    // configure the near/far so we don't clip things that are up close
    _viewer->getCamera()->setNearFarRatio(0.00002);
    
    // osgEarth benefits from pre-compilation of GL objects in the pager. In newer versions of
    // OSG, this activates OSG's IncrementalCompileOpeartion in order to avoid frame breaks.
    _viewer->getDatabasePager()->setDoPreCompile( true );

  
    _isAnimating=false;
    [self startAnimation];
    
}

- (void)viewDidUnload
{    
    [super viewDidUnload];
    
    [self stopAnimation];
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Release any cached data, images, etc. that aren't in use.
}

- (BOOL)shouldAutorotateToInterfaceOrientation:(UIInterfaceOrientation)interfaceOrientation
{
    return interfaceOrientation == UIInterfaceOrientationPortrait;
}

#pragma mark - update fired by timer to render update and render osgm

//
- (void)startAnimation
{
    if(!_isAnimating){
        _displayLink = [CADisplayLink displayLinkWithTarget:self selector:@selector(update:)];
        [_displayLink setFrameInterval:1];
        [_displayLink addToRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
        _isAnimating = true;
    }
}
- (void)stopAnimation
{
    if(_displayLink){
        [_displayLink invalidate];
        _displayLink = nil;
        _isAnimating = false;
    }
}


- (void)update:(CADisplayLink *)sender
{
    _viewer->frame();
}


@end
