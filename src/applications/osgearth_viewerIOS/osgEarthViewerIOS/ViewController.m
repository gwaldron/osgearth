//
//  ViewController.m
//  osgEarthViewerIOS
//
//  Created by Thomas Hogarth on 14/07/2012.
//

#import "ViewController.h"

#include "osgPlugins.h"

#include <osgDB/FileUtils>

#include <osgGA/MultiTouchTrackballManipulator>

#include <osgViewer/api/IOS/GraphicsWindowIOS>

#include <osgEarth/Viewpoint>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/ObjectLocator>
#include <osgEarthUtil/SkyNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>

#include "EarthMultiTouchManipulator.h"
#include "GLES2ShaderGenVisitor.h"

using namespace osgEarth;
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
    _viewer->setCameraManipulator( new osgEarth::Util::EarthMultiTouchManipulator() );
    
    osg::Light* light = new osg::Light( 0 );  
    light->setPosition( osg::Vec4(0, -1, 0, 0 ) );
    light->setAmbient( osg::Vec4(0.4f, 0.4f, 0.4f ,1.0) );
    light->setDiffuse( osg::Vec4(1,1,1,1) );
    light->setSpecular( osg::Vec4(0,0,0,1) );

    osg::Material* material = new osg::Material();
    material->setAmbient(osg::Material::FRONT, osg::Vec4(0.4,0.4,0.4,1.0));
    material->setDiffuse(osg::Material::FRONT, osg::Vec4(0.9,0.9,0.9,1.0));
    material->setSpecular(osg::Material::FRONT, osg::Vec4(0.4,0.4,0.4,1.0));
    
    
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
    osgEarth::Util::EarthManipulator* manip = dynamic_cast<osgEarth::Util::EarthManipulator*>(_viewer->getCameraManipulator());
    if ( manip == 0L )
    {
        OSG_WARN << "Helper used before installing an EarthManipulator" << std::endl;
    }
    
    // a root node to hold everything:
    osg::Group* root = new osg::Group();
    root->addChild( mapNode.get() );
    //root->getOrCreateStateSet()->setAttribute(light);
    
    //have to add these
    root->getOrCreateStateSet()->setAttribute(material);
    //root->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    
    double hours = 12.0f;
    float ambientBrightness = 0.4f;
    osgEarth::Util::SkyNode* sky = new osgEarth::Util::SkyNode( mapNode->getMap() );
    sky->setAmbientBrightness( ambientBrightness );
    sky->setDateTime( 1984, 11, 8, hours );
    sky->attach( _viewer, 0 );
    root->addChild( sky );
    
    
    //for some reason we have to do this as global stateset doesn't
    //appear to be in the statesetstack
    root->getOrCreateStateSet()->setAttribute(_viewer->getLight());
    
    
    //add model
     unsigned int numObjects = 2;
    osg::Group* treeGroup = new osg::Group();
    root->addChild(treeGroup);
    osg::Node* tree = osgDB::readNodeFile("./data/boxman.osg");         
    osg::MatrixTransform* mt = new osg::MatrixTransform();
    mt->setMatrix(osg::Matrixd::scale(1000,1000,1000));
    mt->addChild( tree );
    //Create bound around mt rainer
    double centerLat =  46.840866;
    double centerLon = -121.769846;
    double height = 0.2;
    double width = 0.2;
    double minLat = centerLat - (height/2.0);
    double minLon = centerLon - (width/2.0);
    
    OE_NOTICE << "Placing " << numObjects << " trees" << std::endl;
    
    for (unsigned int i = 0; i < numObjects; i++)
    {
        osgEarth::Util::ObjectLocatorNode* locator = new osgEarth::Util::ObjectLocatorNode( mapNode->getMap() );        
        double lat = minLat + height * (rand() * 1.0)/(RAND_MAX-1);
        double lon = minLon + width * (rand() * 1.0)/(RAND_MAX-1);        
        //OE_NOTICE << "Placing tree at " << lat << ", " << lon << std::endl;
        locator->getLocator()->setPosition(osg::Vec3d(lon,  lat, 0 ) );        
        locator->addChild( mt );
        treeGroup->addChild( locator );
        mapNode->getTerrain()->addTerrainCallback( new ClampObjectLocatorCallback(locator) );        
    }    
    
    //manip->setHomeViewpoint(Viewpoint( "Mt Rainier",        osg::Vec3d(    centerLon,   centerLat, 0.0 ), 0.0, -90, 45000 ));
    
    //attach a UpdateLightingUniformsHelper to the model
    UpdateLightingUniformsHelper* updateLightInfo = new UpdateLightingUniformsHelper();
    treeGroup->setCullCallback(updateLightInfo);
    
    osgUtil::GLES2ShaderGenVisitor shaderGen;
    treeGroup->accept(shaderGen);
    
    _viewer->setSceneData( root );
}

- (void)viewDidLoad
{
    [super viewDidLoad];
    
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
    
    //_viewer->setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    _viewer->getDatabasePager()->setTargetMaximumNumberOfPageLOD(0);
    _viewer->getDatabasePager()->setUnrefImageDataAfterApplyPolicy(true,true);
    
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
