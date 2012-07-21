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
#include "MultiTouchManipulator/MultiTouchTrackballManipulator2.h"
#include <osgViewer/api/IOS/GraphicsWindowIOS>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>

#include "GLES2ShaderGenVisitor.h"

#ifndef WIN32
#define SHADER_COMPAT \
"#ifndef GL_ES\n" \
"#if (__VERSION__ <= 110)\n" \
"#define lowp\n" \
"#define mediump\n" \
"#define highp\n" \
"#endif\n" \
"#endif\n"
#else
#define SHADER_COMPAT ""
#endif


static const char* texturedVertSource = { 
	SHADER_COMPAT 
	"attribute vec4 osg_Vertex;\n"
	"attribute vec4 osg_MultiTexCoord0;\n"
	"uniform mat4 osg_ModelViewProjectionMatrix;\n"
	"varying mediump vec2 texCoord0;\n"
	"void main(void) {\n" 
	"  gl_Position = osg_ModelViewProjectionMatrix * osg_Vertex;\n"
	"  texCoord0 = osg_MultiTexCoord0.xy;\n"
	"}\n" 
}; 

static const char* texturedFragSource = { 
	SHADER_COMPAT 
	"uniform sampler2D diffuseTexture;\n"
	"varying mediump vec2 texCoord0;\n"
	"void main(void) {\n" 
    "  gl_FragColor = texture2D(diffuseTexture, texCoord0);\n"
	"}\n" 
};


@interface ViewController () {

}

@end

osg::ref_ptr<osg::Program> g_defaultShader = NULL; 

//
//Visitor to enable VBO on all drawables
class SafeStateVisitor : public osg::NodeVisitor
{
public:
    
    SafeStateVisitor()
    : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
    {
    }
    virtual void apply(osg::Node& node){
        this->SetupStateSet(node.getStateSet());
        traverse(node);
    }
    virtual void apply(osg::Geode& geode)
    {
        this->SetupStateSet(geode.getStateSet());
        for(unsigned int i=0; i<geode.getNumDrawables(); i++)
        {
            geode.getDrawable(i)->setUseDisplayList(false);
            geode.getDrawable(i)->setUseVertexBufferObjects(true);
            this->SetupStateSet(geode.getDrawable(i)->getStateSet());
        }
        
        traverse(geode);
    }
protected:
    void SetupStateSet(osg::StateSet* state){
        if(!state){return;}
        OSG_ALWAYS << "SetupStateSet" << std::endl;
        state->removeAttribute(osg::StateAttribute::PROGRAM);
        //osg::StateSet::TextureAttributeList textures = state->getTextureAttributeList();
        
        if(g_defaultShader == NULL){
            g_defaultShader = new osg::Program();
            g_defaultShader->setName("defaultShader"); 
            g_defaultShader->addShader(new osg::Shader(osg::Shader::VERTEX, texturedVertSource)); 
            g_defaultShader->addShader(new osg::Shader(osg::Shader::FRAGMENT, texturedFragSource));
        }

        state->setAttributeAndModes(g_defaultShader, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE); 	
        
        
        for(unsigned int i=0; i<4; i++){
            osg::Texture2D* tex = dynamic_cast<osg::Texture2D*>(state->getTextureAttribute(i,
                                                                osg::StateAttribute::TEXTURE));
            if(tex){
                OSG_ALWAYS << "SafeTex" << std::endl;
                //tex->setUnRefImageDataAfterApply(true);
                //tex->setUseHardwareMipMapGeneration(false);
                //tex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
                //tex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
                //set to linear to disable mipmap generation
                //tex->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::NEAREST);
                //tex->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::NEAREST);
            }
        }
    }
};

@implementation ViewController


- (void)dealloc
{
    OSG_ALWAYS << "dealloc" << std::endl;
    [super dealloc];
    
    [self stopAnimation];
    _viewer = NULL;
}

- (void)loadBasicScene{
    
    _viewer->setCameraManipulator(new osgGA::MultiTouchTrackballManipulator2());
    
    osg::Node* node = osgDB::readNodeFile(osgDB::findDataFile("models/box.osg"));
    
    osgUtil::GLES2ShaderGenVisitor shaderGen;
    //node->accept(shaderGen);
    
    _viewer->setSceneData(node);

}

- (void)loadOsgEarthDemoScene{

    // install our default manipulator (do this before calling load)
    //_viewer->setCameraManipulator( new osgEarth::Util::EarthManipulator() );
    _viewer->setCameraManipulator(new osgGA::MultiTouchTrackballManipulator2());
    
    osg::Node* node = osgDB::readNodeFile(osgDB::findDataFile("tests/yahoo_maps.earth"));
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
    
    SafeStateVisitor stateVisitor;
    //root->accept(stateVisitor);
    
    osgUtil::GLES2ShaderGenVisitor shaderGen;
    //root->accept(shaderGen);
    
    _viewer->setSceneData( root );
}

- (void)viewDidLoad
{
    [super viewDidLoad];
    
    osg::setNotifyLevel(osg::DEBUG_FP);
    
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
	//traits->stencil = 8;
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
    SafeStateVisitor stateVisitor;
    _viewer->getSceneData()->accept(stateVisitor);

    
	//_viewer->getSceneData()->getOrCreateStateSet()->setAttributeAndModes(defaultShader, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE); 	

    
    osgUtil::GLES2ShaderGenVisitor shaderGen;
    //_viewer->getSceneData()->accept(shaderGen);
    _viewer->frame();
}


@end
