//
//  ViewController.m
//  osgEarthViewerIOS
//
//  Created by Thomas Hogarth on 14/07/2012.
//

#import "ViewController.h"

#include "osgPlugins.h"

#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#include <osgViewer/api/IOS/GraphicsWindowIOS>

#include <osgEarth/Viewpoint>
#include <osgEarthUtil/SkyNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>

using namespace osgEarth;
using namespace osgEarth::Util;


@interface ViewController () {

}

@end


@implementation ViewController

- (id)intWithFileName:(NSString*)file
{
    self = [super init];
    if(self){
        
        _file = [file cStringUsingEncoding:NSASCIIStringEncoding];
        
    }
    return self;
}

- (void)dealloc
{
    OSG_ALWAYS << "dealloc" << std::endl;
    [super dealloc];
    
    [self stopAnimation];
    _viewer = NULL;
}

- (void)loadOsgEarthDemoScene{

    // install our default manipulator (do this before calling load)
    //    _viewer->setCameraManipulator( new osgEarth::Util::EarthManipulator() );
    
    // This chunk inverts the Y axis.
    osgEarth::Util::EarthManipulator* manip = new osgEarth::Util::EarthManipulator();
    osgEarth::Util::EarthManipulator::ActionOptions options;
    options.add(osgEarth::Util::EarthManipulator::OPTION_SCALE_Y, -1.0);
    manip->getSettings()->bindMouse(osgEarth::Util::EarthManipulator::ACTION_EARTH_DRAG, osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON, 0L, options);
    manip->getSettings()->setThrowingEnabled(true);
    manip->getSettings()->setThrowDecayRate(0.1);
    _viewer->setCameraManipulator( manip );
    
    osg::Node* node = osgDB::readNodeFile(osgDB::findDataFile("tests/" + _file));
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
    osgEarth::Util::EarthManipulator* manip_temp = dynamic_cast<osgEarth::Util::EarthManipulator*>(_viewer->getCameraManipulator());
    if ( manip_temp == 0L )
    {
        OSG_WARN << "Helper used before installing an EarthManipulator" << std::endl;
    }
    
    // a root node to hold everything:
    osg::Group* root = new osg::Group();
    root->addChild( mapNode.get() );

    
    //have to add these
    osg::Material* material = new osg::Material();
    material->setAmbient(osg::Material::FRONT, osg::Vec4(0.4,0.4,0.4,1.0));
    material->setDiffuse(osg::Material::FRONT, osg::Vec4(0.9,0.9,0.9,1.0));
    material->setSpecular(osg::Material::FRONT, osg::Vec4(0.4,0.4,0.4,1.0));
    root->getOrCreateStateSet()->setAttribute(material); //lighting doesn't work without a material for some reason
    root->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);//comment out to disable lighting
    
    double hours = 12.0f;
    float ambientBrightness = 1.0f;
    osgEarth::Util::SkyNode* sky = new osgEarth::Util::SkyNode( mapNode->getMap() );
    sky->setAmbientBrightness( ambientBrightness );
    sky->setDateTime( 1984, 11, 8, hours );
    sky->attach( _viewer, 0 );
    root->addChild( sky );
    
    _viewer->setSceneData( root );
}

- (void)viewDidLoad
{
    [super viewDidLoad];
    
    std::string fullPath = osgDB::findDataFile("gdal_data/gdal_datum.csv");
    std::string dataPath = osgDB::getFilePath(fullPath);
    
    setenv("GDAL_DATA", dataPath.c_str(), 1);
    
    osg::setNotifyLevel(osg::DEBUG_FP);
    osgEarth::setNotifyLevel(osg::DEBUG_FP);
    
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
    
	// Setup the traits parameters
	traits->x = 0;
	traits->y = 0;
	traits->width = w*scale;
	traits->height = h*scale;
	traits->depth = 24;
	traits->alpha = 8;
    //traits->samples = 4;
    //traits->sampleBuffers = 2;
	traits->stencil = 0;
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
    _viewer->getCamera()->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    _viewer->getCamera()->setClearColor(osg::Vec4(1.0f,0.0f,0.0f,0.0f));
    _viewer->getCamera()->setClearStencil(0);
    _viewer->getCamera()->setProjectionMatrixAsPerspective(45.0f,(float)w/h,
                                                           0.1f, 10000.0f);


    //load
    [self loadOsgEarthDemoScene];
    
    // configure the near/far so we don't clip things that are up close
    _viewer->getCamera()->setNearFarRatio(0.00002);
    
    //optimize viewer and db pager
    _viewer->setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    _viewer->getCamera()->setLODScale(_viewer->getCamera()->getLODScale()/2.0);
    
    // osgEarth benefits from pre-compilation of GL objects in the pager. In newer versions of
    // OSG, this activates OSG's IncrementalCompileOpeartion in order to avoid frame breaks.
//    _viewer->getDatabasePager()->setDoPreCompile( true );
//   _viewer->getDatabasePager()->setTargetMaximumNumberOfPageLOD(0);
//    _viewer->getDatabasePager()->setUnrefImageDataAfterApplyPolicy(true,true);

  
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

- (void)touchesBegan:(NSSet *)touches withEvent:(UIEvent *)event
{
	NSLog(@"touchesBegan");
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
    //
    _viewer->frame();
}


@end
