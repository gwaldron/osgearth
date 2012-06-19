/* OpenSceneGraph example, osgshadow.
*
*  Permission is hereby granted, free of charge, to any person obtaining a copy
*  of this software and associated documentation files (the "Software"), to deal
*  in the Software without restriction, including without limitation the rights
*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*  copies of the Software, and to permit persons to whom the Software is
*  furnished to do so, subject to the following conditions:
*
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
*  THE SOFTWARE.
*/

#include <osg/ArgumentParser>
#include <osg/ComputeBoundsVisitor>
#include <osg/Texture2D>
#include <osg/ShapeDrawable>
#include <osg/MatrixTransform>
#include <osg/Geometry>

#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/StateSetManipulator>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgShadow/ShadowedScene>
#include <osgShadow/ShadowVolume>
#include <osgShadow/ShadowTexture>
#include <osgShadow/ShadowMap>
#include <osgShadow/SoftShadowMap>
#include <osgShadow/ParallelSplitShadowMap>
#include <osgShadow/LightSpacePerspectiveShadowMap>
#include <osgShadow/StandardShadowMap>
#ifdef HAS_VDSM
#include <osgShadow/ViewDependentShadowMap>
#endif

#include <osgUtil/Optimizer>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <osg/io_utils>
#include <iostream>
#include <sstream>

#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/ShadowUtils>
#include <osgEarth/NodeUtils>

using namespace osgEarth;
using namespace osgEarth::Util;

class ChangeFOVHandler : public osgGA::GUIEventHandler
{
public:
    ChangeFOVHandler(osg::Camera* camera)
        : _camera(camera)
    {
        double fovy, aspectRatio, zNear, zFar;
        _camera->getProjectionMatrix().getPerspective(fovy, aspectRatio, zNear, zFar);
        std::cout << "FOV is " << fovy << std::endl;
    }

    /** Deprecated, Handle events, return true if handled, false otherwise. */
    virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        if (ea.getEventType() == osgGA::GUIEventAdapter::KEYUP)
        {
            if (ea.getKey() == '-' || ea.getKey() == '=' || ea.getKey() == '0')
            {
                double fovy, aspectRatio, zNear, zFar;
                _camera->getProjectionMatrix().getPerspective(fovy, aspectRatio, zNear, zFar);

                if (ea.getKey() == '-')
                {
                    fovy -= 5.0;
                }

                if (ea.getKey() == '=')
                {
                    fovy += 5.0;
                }

                if (ea.getKey() == '0')
                {
                    fovy = 45.0;
                }

                std::cout << "Setting FOV to " << fovy << std::endl;
                _camera->getProjectionMatrix().makePerspective(fovy, aspectRatio, zNear, zFar);

                return true;
            }
        }

        return false;
    }

    osg::ref_ptr<osg::Camera> _camera;
};


class DumpShadowVolumesHandler : public osgGA::GUIEventHandler
{
public:
    DumpShadowVolumesHandler(  )
    {
        set( false );
    }

    bool get() { return _value; }
    void set( bool value ) { _value = value; }

    /** Deprecated, Handle events, return true if handled, false otherwise. */
    virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        if (ea.getEventType() == osgGA::GUIEventAdapter::KEYUP)
        {
            if (ea.getKey() == 'D' )
            {
                set( true );
                return true;
            }
        }

        return false;
    }

    bool  _value;
};


class LightAnimationHandler : public osgGA::GUIEventHandler
{
public:
    LightAnimationHandler(bool flag=true): _animating(flag) {}

    void setAnimating(bool flag) { _animating = flag; }
    bool getAnimating() const { return _animating; }

    /** Deprecated, Handle events, return true if handled, false otherwise. */
    virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        if (ea.getEventType() == osgGA::GUIEventAdapter::KEYUP)
        {
            if (ea.getKey() == 'p' )
            {
                _animating = !_animating;
                return true;
            }
        }

        return false;
    }

    bool  _animating;
};


static int ReceivesShadowTraversalMask = 0x1;
static int CastsShadowTraversalMask = 0x2;

int main(int argc, char** argv)
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc, argv);

    // set up the usage document, in case we need to print out how to use this program.
    arguments.getApplicationUsage()->setDescription(arguments.getApplicationName() + " is the example which demonstrates using of GL_ARB_shadow extension implemented in osg::Texture class");
    arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName());
    arguments.getApplicationUsage()->addCommandLineOption("-h or --help", "Display this information");
    arguments.getApplicationUsage()->addCommandLineOption("--positionalLight", "Use a positional light.");
    arguments.getApplicationUsage()->addCommandLineOption("--directionalLight", "Use a direction light.");
    arguments.getApplicationUsage()->addCommandLineOption("--noUpdate", "Disable the updating the of light source.");

    arguments.getApplicationUsage()->addCommandLineOption("--castsShadowMask", "Override default castsShadowMask (default - 0x2)");
    arguments.getApplicationUsage()->addCommandLineOption("--receivesShadowMask", "Override default receivesShadowMask (default - 0x1)");

    arguments.getApplicationUsage()->addCommandLineOption("--base", "Add a base geometry to test shadows.");
    arguments.getApplicationUsage()->addCommandLineOption("--sv", "Select ShadowVolume implementation.");
    arguments.getApplicationUsage()->addCommandLineOption("--ssm", "Select SoftShadowMap implementation.");
    arguments.getApplicationUsage()->addCommandLineOption("--sm", "Select ShadowMap implementation.");

    arguments.getApplicationUsage()->addCommandLineOption("--pssm", "Select ParallelSplitShadowMap implementation.");//ADEGLI
    arguments.getApplicationUsage()->addCommandLineOption("--mapcount", "ParallelSplitShadowMap texture count.");//ADEGLI
    arguments.getApplicationUsage()->addCommandLineOption("--mapres", "ParallelSplitShadowMap texture resolution.");//ADEGLI
    arguments.getApplicationUsage()->addCommandLineOption("--debug-color", "ParallelSplitShadowMap display debugging color (only the first 3 maps are color r=0,g=1,b=2.");//ADEGLI
    arguments.getApplicationUsage()->addCommandLineOption("--minNearSplit", "ParallelSplitShadowMap shadow map near offset.");//ADEGLI
    arguments.getApplicationUsage()->addCommandLineOption("--maxFarDist", "ParallelSplitShadowMap max far distance to shadow.");//ADEGLI
    arguments.getApplicationUsage()->addCommandLineOption("--moveVCamFactor", "ParallelSplitShadowMap move the virtual frustum behind the real camera, (also back ground object can cast shadow).");//ADEGLI
    arguments.getApplicationUsage()->addCommandLineOption("--PolyOffset-Factor", "ParallelSplitShadowMap set PolygonOffset factor.");//ADEGLI
    arguments.getApplicationUsage()->addCommandLineOption("--PolyOffset-Unit", "ParallelSplitShadowMap set PolygonOffset unit.");//ADEGLI

    arguments.getApplicationUsage()->addCommandLineOption("--lispsm", "Select LightSpacePerspectiveShadowMap implementation.");
    arguments.getApplicationUsage()->addCommandLineOption("--msm", "Select MinimalShadowMap implementation.");
    arguments.getApplicationUsage()->addCommandLineOption("--ViewBounds", "MSM, LiSPSM optimize shadow for view frustum (weakest option)");
    arguments.getApplicationUsage()->addCommandLineOption("--CullBounds", "MSM, LiSPSM optimize shadow for bounds of culled objects in view frustum (better option).");
    arguments.getApplicationUsage()->addCommandLineOption("--DrawBounds", "MSM, LiSPSM optimize shadow for bounds of predrawn pixels in view frustum (best & default).");
    arguments.getApplicationUsage()->addCommandLineOption("--mapres", "MSM, LiSPSM & texture resolution.");
    arguments.getApplicationUsage()->addCommandLineOption("--maxFarDist", "MSM, LiSPSM max far distance to shadow.");
    arguments.getApplicationUsage()->addCommandLineOption("--moveVCamFactor", "MSM, LiSPSM move the virtual frustum behind the real camera, (also back ground object can cast shadow).");
    arguments.getApplicationUsage()->addCommandLineOption("--minLightMargin", "MSM, LiSPSM the same as --moveVCamFactor.");
    arguments.getApplicationUsage()->addCommandLineOption("--two-sided", "Use two-sided stencil extension for shadow volumes.");
    arguments.getApplicationUsage()->addCommandLineOption("--two-pass", "Use two-pass stencil for shadow volumes.");

    // construct the viewer.
    osgViewer::Viewer viewer(arguments);

    // if user request help write it out to cout.
    if (arguments.read("-h") || arguments.read("--help"))
    {
        arguments.getApplicationUsage()->write(std::cout);
        return 1;
    }

    double zNear=1.0, zMid=10.0, zFar=1000.0;
    if (arguments.read("--depth-partition",zNear, zMid, zFar))
    {
        // set up depth partitioning
        osg::ref_ptr<osgViewer::DepthPartitionSettings> dps = new osgViewer::DepthPartitionSettings;
        dps->_mode = osgViewer::DepthPartitionSettings::FIXED_RANGE;
        dps->_zNear = zNear;
        dps->_zMid = zMid;
        dps->_zFar = zFar;
        viewer.setUpDepthPartition(dps.get());
    }

    if (arguments.read("--dp"))
    {
        // set up depth partitioning
        viewer.setUpDepthPartition();
    }

    float fov = 0.0;
    while (arguments.read("--fov",fov)) {}

    osg::Vec4 lightpos(0.0,0.0,1,0.0);
    bool spotlight = false;
    bool skyNode = false;
    if (arguments.find("--sky") != -1)
        skyNode = true;
    // These will be ignored if there is a sky node.
    while (arguments.read("--positionalLight")) { lightpos.set(0.5,0.5,1.5,1.0); }
    while (arguments.read("--directionalLight")) { lightpos.set(0.0,0.0,1,0.0); }
    while (arguments.read("--spotLight")) { lightpos.set(0.5,0.5,1.5,1.0); spotlight = true; }

    bool keepLightPos = false;
    osg::Vec3 spotLookat(0.0,0.0,0.0);
    while ( arguments.read("--light-pos", lightpos.x(), lightpos.y(), lightpos.z(), lightpos.w())) { keepLightPos = true; }
    while ( arguments.read("--light-pos", lightpos.x(), lightpos.y(), lightpos.z())) { lightpos.w()=1.0; keepLightPos = true; }
    while ( arguments.read("--light-dir", lightpos.x(), lightpos.y(), lightpos.z())) { lightpos.w()=0.0; keepLightPos = true; }
    while ( arguments.read("--spot-lookat", spotLookat.x(), spotLookat.y(), spotLookat.z())) { }


    while (arguments.read("--castsShadowMask", CastsShadowTraversalMask ));
    while (arguments.read("--receivesShadowMask", ReceivesShadowTraversalMask ));

    bool updateLightPosition = !skyNode;
    while (arguments.read("--noUpdate")) updateLightPosition = false;

    // set up the camera manipulators.
    viewer.setCameraManipulator(new EarthManipulator);

    // add the record camera path handler
    viewer.addEventHandler(new osgViewer::RecordCameraPathHandler);

    osg::ref_ptr<osgShadow::ShadowedScene> shadowedScene = new osgShadow::ShadowedScene;

    shadowedScene->setReceivesShadowTraversalMask(ReceivesShadowTraversalMask);
    shadowedScene->setCastsShadowTraversalMask(CastsShadowTraversalMask);

    osg::ref_ptr<osgShadow::MinimalShadowMap> msm = NULL;
    if (arguments.read("--no-shadows"))
    {
        OSG_NOTICE<<"Not using a ShadowTechnique"<<std::endl;
        shadowedScene->setShadowTechnique(0);
    }
    else if (arguments.read("--sv"))
    {
        // hint to tell viewer to request stencil buffer when setting up windows
        osg::DisplaySettings::instance()->setMinimumNumStencilBits(8);

        osg::ref_ptr<osgShadow::ShadowVolume> sv = new osgShadow::ShadowVolume;
        sv->setDynamicShadowVolumes(updateLightPosition);
        while (arguments.read("--two-sided")) sv->setDrawMode(osgShadow::ShadowVolumeGeometry::STENCIL_TWO_SIDED);
        while (arguments.read("--two-pass")) sv->setDrawMode(osgShadow::ShadowVolumeGeometry::STENCIL_TWO_PASS);

        shadowedScene->setShadowTechnique(sv.get());
    }
    else if (arguments.read("--st"))
    {
        osg::ref_ptr<osgShadow::ShadowTexture> st = new osgShadow::ShadowTexture;
        shadowedScene->setShadowTechnique(st.get());
    }
    else if (arguments.read("--stsm"))
    {
        osg::ref_ptr<osgShadow::StandardShadowMap> st = new osgShadow::StandardShadowMap;
        shadowedScene->setShadowTechnique(st.get());
    }
    else if (arguments.read("--pssm"))
    {
        int mapcount = 3;
        while (arguments.read("--mapcount", mapcount));
        osg::ref_ptr<osgShadow::ParallelSplitShadowMap> pssm = new osgShadow::ParallelSplitShadowMap(NULL,mapcount);

        int mapres = 1024;
        while (arguments.read("--mapres", mapres))
            pssm->setTextureResolution(mapres);

        while (arguments.read("--debug-color")) { pssm->setDebugColorOn(); }


        int minNearSplit=0;
        while (arguments.read("--minNearSplit", minNearSplit))
            if ( minNearSplit > 0 ) {
                pssm->setMinNearDistanceForSplits(minNearSplit);
                std::cout << "ParallelSplitShadowMap : setMinNearDistanceForSplits(" << minNearSplit <<")" << std::endl;
            }

        int maxfardist = 0;
        while (arguments.read("--maxFarDist", maxfardist))
            if ( maxfardist > 0 ) {
                pssm->setMaxFarDistance(maxfardist);
                std::cout << "ParallelSplitShadowMap : setMaxFarDistance(" << maxfardist<<")" << std::endl;
            }

        int moveVCamFactor = 0;
        while (arguments.read("--moveVCamFactor", moveVCamFactor))
            if ( maxfardist > 0 ) {
                pssm->setMoveVCamBehindRCamFactor(moveVCamFactor);
                std::cout << "ParallelSplitShadowMap : setMoveVCamBehindRCamFactor(" << moveVCamFactor<<")" << std::endl;
            }



        double polyoffsetfactor = pssm->getPolygonOffset().x();
        double polyoffsetunit   = pssm->getPolygonOffset().y();
        while (arguments.read("--PolyOffset-Factor", polyoffsetfactor));
        while (arguments.read("--PolyOffset-Unit", polyoffsetunit));
        pssm->setPolygonOffset(osg::Vec2(polyoffsetfactor,polyoffsetunit));

        shadowedScene->setShadowTechnique(pssm.get());
    }
    else if (arguments.read("--ssm"))
    {
        osg::ref_ptr<osgShadow::SoftShadowMap> sm = new osgShadow::SoftShadowMap;
        shadowedScene->setShadowTechnique(sm.get());
    }
#ifdef HAS_VDSM
    else if( arguments.read("--vdsm") )
    {
        osgShadow::ShadowSettings* settings = new osgShadow::ShadowSettings;
        shadowedScene->setShadowSettings(settings);

        while( arguments.read("--debugHUD") ) settings->setDebugDraw( true );
        if (arguments.read("--persp")) settings->setShadowMapProjectionHint(osgShadow::ShadowSettings::PERSPECTIVE_SHADOW_MAP);
        if (arguments.read("--ortho")) settings->setShadowMapProjectionHint(osgShadow::ShadowSettings::ORTHOGRAPHIC_SHADOW_MAP);

        unsigned int unit=1;
        if (arguments.read("--unit",unit)) settings->setBaseShadowTextureUnit(unit);

        double n=0.0;
        if (arguments.read("-n",n)) settings->setMinimumShadowMapNearFarRatio(n);

        unsigned int numShadowMaps;
        if (arguments.read("--num-sm",numShadowMaps)) settings->setNumShadowMapsPerLight(numShadowMaps);

        if (arguments.read("--parallel-split") || arguments.read("--ps") ) settings->setMultipleShadowMapHint(osgShadow::ShadowSettings::PARALLEL_SPLIT);
        if (arguments.read("--cascaded")) settings->setMultipleShadowMapHint(osgShadow::ShadowSettings::CASCADED);


        int mapres = 1024;
        while (arguments.read("--mapres", mapres))
            settings->setTextureSize(osg::Vec2s(mapres,mapres));

        osg::ref_ptr<osgShadow::ViewDependentShadowMap> vdsm = new osgShadow::ViewDependentShadowMap;
        shadowedScene->setShadowTechnique(vdsm.get());
    }
#endif
    else if ( arguments.read("--lispsm") )
    {
        if( arguments.read( "--ViewBounds" ) )
            msm = new osgShadow::LightSpacePerspectiveShadowMapVB;
        else if( arguments.read( "--CullBounds" ) )
            msm = new osgShadow::LightSpacePerspectiveShadowMapCB;
        else // if( arguments.read( "--DrawBounds" ) ) // default
            msm = new osgShadow::LightSpacePerspectiveShadowMapDB;
    }
    else if( arguments.read("--msm") )
    {
       if( arguments.read( "--ViewBounds" ) )
            msm = new osgShadow::MinimalShadowMap;
       else if( arguments.read( "--CullBounds" ) )
            msm = new osgShadow::MinimalCullBoundsShadowMap;
       else // if( arguments.read( "--DrawBounds" ) ) // default
            msm = new osgShadow::MinimalDrawBoundsShadowMap;
    }
    else /* if (arguments.read("--sm")) */
    {
        osg::ref_ptr<osgShadow::ShadowMap> sm = new osgShadow::ShadowMap;
        shadowedScene->setShadowTechnique(sm.get());

        int mapres = 1024;
        while (arguments.read("--mapres", mapres))
            sm->setTextureSize(osg::Vec2s(mapres,mapres));
    }

    if( msm )// Set common MSM & LISPSM arguments
    {
        shadowedScene->setShadowTechnique( msm.get() );
        while( arguments.read("--debugHUD") ) msm->setDebugDraw( true );

        float minLightMargin = 10.f;
        float maxFarPlane = 0;
        unsigned int texSize = 1024;
        unsigned int baseTexUnit = 0;
        unsigned int shadowTexUnit = 1;

        while ( arguments.read("--moveVCamFactor", minLightMargin ) );
        while ( arguments.read("--minLightMargin", minLightMargin ) );
        while ( arguments.read("--maxFarDist", maxFarPlane ) );
        while ( arguments.read("--mapres", texSize ));
        while ( arguments.read("--baseTextureUnit", baseTexUnit) );
        while ( arguments.read("--shadowTextureUnit", shadowTexUnit) );

        msm->setMinLightMargin( minLightMargin );
        msm->setMaxFarPlane( maxFarPlane );
        msm->setTextureSize( osg::Vec2s( texSize, texSize ) );
        msm->setShadowTextureCoordIndex( shadowTexUnit );
        msm->setShadowTextureUnit( shadowTexUnit );
        msm->setBaseTextureCoordIndex( baseTexUnit );
        msm->setBaseTextureUnit( baseTexUnit );
    }

    OSG_INFO<<"shadowedScene->getShadowTechnique()="<<shadowedScene->getShadowTechnique()<<std::endl;

    osg::ref_ptr<osg::Group> root = shadowedScene;
    osg::ref_ptr<osg::Group> model = MapNodeHelper().load(arguments, &viewer);
    if (!model.valid())
    {
        OE_NOTICE
            << "\nUsage: " << argv[0] << " file.earth" << std::endl
            << MapNodeHelper().usage() << std::endl;
        exit(1);
    }

    setUpShadows(shadowedScene, model);
    // The ControlCanvas is a camera and doesn't play nicely with the
    // shadow traversal. Also, it shouldn't be shadowed, and the
    // ReceivesShadowTraversalMask doesn't really prevent that. So,
    // take the canvas out of the shadowed scene.
    for (int i = 0; i < model->getNumChildren(); ++i)
    {
        osg::ref_ptr<Controls::ControlCanvas> canvas
            = dynamic_cast<Controls::ControlCanvas*>(model->getChild(i));
        if (canvas.valid())
        {
            root = new osg::Group;
            root->addChild(shadowedScene);
            model->removeChild(i);
            root->addChild(canvas.get());
            break;
        }
    }
    model->setNodeMask(CastsShadowTraversalMask | ReceivesShadowTraversalMask);
    osg::ref_ptr<osg::LightSource> ls;
    osg::BoundingBox bb;
    if (!skyNode)
    {
        // get the bounds of the model.
        osg::ComputeBoundsVisitor cbbv;
        model->accept(cbbv);
        bb = cbbv.getBoundingBox();

        if (lightpos.w()==1.0 && !keepLightPos)
        {
            lightpos.x() = bb.xMin()+(bb.xMax()-bb.xMin())*lightpos.x();
            lightpos.y() = bb.yMin()+(bb.yMax()-bb.yMin())*lightpos.y();
            lightpos.z() = bb.zMin()+(bb.zMax()-bb.zMin())*lightpos.z();
        }

        ls = new osg::LightSource;
        ls->getLight()->setPosition(lightpos);

        if (spotlight)
        {
            osg::Vec3 center = spotLookat;
            osg::Vec3 lightdir = center - osg::Vec3(lightpos.x(), lightpos.y(), lightpos.z());
            lightdir.normalize();
            ls->getLight()->setDirection(lightdir);
            ls->getLight()->setSpotCutoff(25.0f);

            //set the LightSource, only for checking, there is only 1 light in the scene
            osgShadow::ShadowMap* shadowMap = dynamic_cast<osgShadow::ShadowMap*>(shadowedScene->getShadowTechnique());
            if( shadowMap ) shadowMap->setLight(ls.get());
        }

        if ( arguments.read("--coloured-light"))
        {
            ls->getLight()->setAmbient(osg::Vec4(1.0,0.0,0.0,1.0));
            ls->getLight()->setDiffuse(osg::Vec4(0.0,1.0,0.0,1.0));
        }
        else
        {
            ls->getLight()->setAmbient(osg::Vec4(0.2,0.2,0.2,1.0));
            ls->getLight()->setDiffuse(osg::Vec4(0.8,0.8,0.8,1.0));
        }
    }
    shadowedScene->addChild(model.get());
    if (ls.valid())
        shadowedScene->addChild(ls.get());

    viewer.setSceneData(root.get());

    osg::ref_ptr< DumpShadowVolumesHandler > dumpShadowVolumes = new DumpShadowVolumesHandler;

    viewer.addEventHandler(new ChangeFOVHandler(viewer.getCamera()));
    viewer.addEventHandler( dumpShadowVolumes.get() );

    // create the windows and run the threads.
    viewer.realize();

    if (fov!=0.0)
    {
        double fovy, aspectRatio, zNear, zFar;
        viewer.getCamera()->getProjectionMatrix().getPerspective(fovy, aspectRatio, zNear, zFar);

        std::cout << "Setting FOV to " << fov << std::endl;
        viewer.getCamera()->getProjectionMatrix().makePerspective(fov, aspectRatio, zNear, zFar);
    }

    // it is done after viewer.realize() so that the windows are already initialized
    if ( arguments.read("--debugHUD"))
    {
        osgViewer::Viewer::Windows windows;
        viewer.getWindows(windows);

        if (windows.empty()) return 1;

        osgShadow::ShadowMap* sm = dynamic_cast<osgShadow::ShadowMap*>(shadowedScene->getShadowTechnique());
        if( sm ) {
            osg::ref_ptr<osg::Camera> hudCamera = sm->makeDebugHUD();

            // set up cameras to rendering on the first window available.
            hudCamera->setGraphicsContext(windows[0]);
            hudCamera->setViewport(0,0,windows[0]->getTraits()->width, windows[0]->getTraits()->height);

            viewer.addSlave(hudCamera.get(), false);
        }
    }

    osg::ref_ptr<LightAnimationHandler> lightAnimationHandler = updateLightPosition ? new LightAnimationHandler : 0;
    if (lightAnimationHandler) viewer.addEventHandler(lightAnimationHandler.get());
    viewer.addEventHandler(new osgViewer::ScreenCaptureHandler);
    viewer.addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));

    // osgDB::writeNodeFile(*group,"test.osgt");

    while (!viewer.done())
    {
        {
            osgShadow::MinimalShadowMap * msm = dynamic_cast<osgShadow::MinimalShadowMap*>( shadowedScene->getShadowTechnique() );

            if( msm ) {

                // If scene decorated by CoordinateSystemNode try to find localToWorld
                // and set modellingSpaceToWorld matrix to optimize scene bounds computation

                osg::NodePath np = viewer.getCoordinateSystemNodePath();
                if( !np.empty() ) {
                    osg::CoordinateSystemNode * csn =
                        dynamic_cast<osg::CoordinateSystemNode *>( np.back() );

                    if( csn ) {
                        osg::Vec3d pos =
                            viewer.getCameraManipulator()->getMatrix().getTrans();

                        msm->setModellingSpaceToWorldTransform
                            ( csn->computeLocalCoordinateFrame( pos ) );
                    }
                }
            }
        }

        if (lightAnimationHandler.valid() && lightAnimationHandler ->getAnimating())
        {
            float t = viewer.getFrameStamp()->getSimulationTime();

            if (lightpos.w()==1.0)
            {
                lightpos.set(bb.center().x()+sinf(t)*bb.radius(), bb.center().y() + cosf(t)*bb.radius(), bb.zMax() + bb.radius()*3.0f  ,1.0f);
            }
            else
            {
                lightpos.set(sinf(t),cosf(t),1.0f,0.0f);
            }
            ls->getLight()->setPosition(lightpos);

            osg::Vec3f lightDir(-lightpos.x(),-lightpos.y(),-lightpos.z());
            if(spotlight)
                lightDir =  osg::Vec3(bb.center().x()+sinf(t)*bb.radius()/2.0, bb.center().y() + cosf(t)*bb.radius()/2.0, bb.center().z())
                - osg::Vec3(lightpos.x(), lightpos.y(), lightpos.z()) ;
            lightDir.normalize();
            ls->getLight()->setDirection(lightDir);
        }

        if( dumpShadowVolumes->get() )
        {
            dumpShadowVolumes->set( false );

            static int dumpFileNo = 0;
            dumpFileNo ++;
            char filename[256];
            std::sprintf( filename, "shadowDump%d.osgt", dumpFileNo );

            osgShadow::MinimalShadowMap * msm = dynamic_cast<osgShadow::MinimalShadowMap*>( shadowedScene->getShadowTechnique() );

            if( msm ) msm->setDebugDump( filename );
        }

        viewer.frame();
    }

    return 0;
}

