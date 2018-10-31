//
//  ViewController.m
//  osgEarthViewerIOS
//
//  Created by Thomas Hogarth on 14/07/2012.
//

#import "ViewController.h"

#include "osgPlugins.h"

#include <osg/Material>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#include <osgViewer/api/IOS/GraphicsWindowIOS>

#include <osgEarth/Viewpoint>
#include <osgEarthUtil/Sky>
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
    //self = [super init];
    self = [super initWithNibName:@"ViewController" bundle:nil];
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
    
    // a root node to hold everything:
    osg::Group* root = new osg::Group();
    root->addChild( mapNode.get() );

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
    
    // thread-safe initialization of the OSG wrapper manager. Calling this here
    // prevents the "unsupported wrapper" messages from OSG
    osgDB::Registry::instance()->getObjectWrapperManager()->findWrapper("osg::Image");
    
    // ensure tiff plugin is used over imageio
    osgDB::Registry::instance()->addFileExtensionAlias("tiff", "tiff");
    osgDB::Registry::instance()->addFileExtensionAlias("tif", "tiff");
    
    osg::DisplaySettings::instance()->setVertexBufferHint(osg::DisplaySettings::VertexBufferHint::VERTEX_BUFFER_OBJECT);
    
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
    // just do single threaded
    _viewer->setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    
	// Setup the traits parameters
	traits->x = 0;
	traits->y = 0;
	traits->width = w*scale;
	traits->height = h*scale;
	traits->depth = 24;
	traits->alpha = 8;
    //traits->samples = 4;
    //traits->sampleBuffers = 2;
	traits->stencil = 8;
	traits->windowDecoration = false;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->setInheritedWindowPixelFormat = true;
    
	osg::ref_ptr<osg::Referenced> windata = new osgViewer::GraphicsWindowIOS::WindowData(self.view, self, osgViewer::GraphicsWindowIOS::WindowData::PORTRAIT_ORIENTATION, scale);
	traits->inheritedWindowData = windata;
    
	// Create the Graphics Context
	osg::ref_ptr<osg::GraphicsContext> graphicsContext = osg::GraphicsContext::createGraphicsContext(traits.get());
	
	if(graphicsContext)
	{
        _viewer->getCamera()->setGraphicsContext(graphicsContext);
        _viewer->realize();
        osgViewer::GraphicsWindowIOS* osgWindow = dynamic_cast<osgViewer::GraphicsWindowIOS*>(_viewer->getCamera()->getGraphicsContext());
        if(osgWindow) [self.view sendSubviewToBack:(UIView*)osgWindow->getView()];
    }
    
    _viewer->getCamera()->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
    _viewer->getCamera()->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    _viewer->getCamera()->setClearColor(osg::Vec4(1.0f,0.0f,0.0f,0.0f));
    _viewer->getCamera()->setClearStencil(0);
    _viewer->getCamera()->setProjectionMatrixAsPerspective(45.0f,(float)w/h,
                                                           0.1f, 10000.0f);


    // Tell the database pager to not modify the unref settings
    _viewer->getDatabasePager()->setUnrefImageDataAfterApplyPolicy( true, false );


    // install our default manipulator
    osgEarth::Util::EarthManipulator* manip = new osgEarth::Util::EarthManipulator();
    //manip->getSettings()->setThrowingEnabled(true);
    //manip->getSettings()->setThrowDecayRate(0.1);
    _viewer->setCameraManipulator( manip );

    // disable the small-feature culling
    _viewer->getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

    // set a near/far ratio that is smaller than the default. This allows us to get
    // closer to the ground without near clipping. If you need more, use --logdepth
    _viewer->getCamera()->setNearFarRatio(0.0001);

    //load
    [self loadOsgEarthDemoScene];

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

- (IBAction)onBackClicked:(id)sender {
    [self stopAnimation];
    _viewer->done();
    //_viewer->stopThreading();
    //_viewer->getDatabasePager()->cancel();
    [self.view removeFromSuperview];
}


- (void)update:(CADisplayLink *)sender
{
    if(_viewer != NULL && _isAnimating) _viewer->frame();
}


@end
