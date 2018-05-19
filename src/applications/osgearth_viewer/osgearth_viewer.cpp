
#include <osgViewer/Viewer>
#include <osgEarth/LineDrawable>
#include <osgEarth/GLUtils>
#include <osgEarth/CullingUtils>

osg::Node* genTacAid()
{
  osg::Group* group = new osg::Group;

  const float cacheLeft = -1.7046576866548355f;
  const float ringWidth = 0.1f;
  float x = cacheLeft + 0.005f;
  float y = -1.f + ringWidth * 0.5f;
  float deltay = ringWidth;

  // I know I can use LineGroup, but it's not being used here as I was debugging
  osgEarth::LineDrawable* line = new osgEarth::LineDrawable(GL_LINE_STRIP);
  line->installShader(group->getOrCreateStateSet());
  line->setColor(osg::Vec4f(1.f, 0.f, 0.f, 1.f));

#if 0
  line->setLineWidth(3.f);
#else
  // TODO: This fails to show lines
  line->setLineWidth(1.f);
#endif

#if 1
  // Use antialiasing to further demonstrate the problem
  osgEarth::GLUtils::setLineSmooth(line->getOrCreateStateSet(), osg::StateAttribute::ON);
#endif

  // Make sure it's not a blending or cull face problem
  line->getOrCreateStateSet()->setMode(GL_BLEND, 1);
  line->getOrCreateStateSet()->setMode(GL_CULL_FACE, 0);

  for (int k = 0; k < 6; ++k)
  {
    line->pushVertex(osg::Vec3f(x, y, 0.f));
    line->pushVertex(osg::Vec3f(0.f, y, 0.f));
    y += deltay;
  }

  line->dirty();
  group->addChild(line);

  return group;
}


int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    osgViewer::Viewer viewer(arguments);
    const float width = 1040;
    const float height = 730;
    const float ar = width / height;
    viewer.setUpViewInWindow(100, 100, width, height);
    viewer.setRealizeOperation(new osgEarth::GL3RealizeOperation);
    viewer.getCamera()->addCullCallback(new osgEarth::InstallViewportSizeUniform);
    viewer.getCamera()->setClearColor(osg::Vec4f(0,0,0,1));

    osg::Group* scene = new osg::Group;
    scene->getOrCreateStateSet()->setRenderBinDetails(0, "TraversalOrderBin");
    viewer.setSceneData(scene);

    osg::Camera* hud = new osg::Camera;
    hud->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    const float maxExtent = 1.2;
    hud->setProjectionMatrixAsOrtho2D(-maxExtent*ar, maxExtent*ar, -maxExtent, maxExtent);
    hud->setProjectionResizePolicy(osg::Camera::HORIZONTAL);
    hud->setViewMatrixAsLookAt(
      osg::Vec3d(0, 0, 100),
      osg::Vec3d(0, 0, 0),
      osg::Vec3d(0, 1, 0));
    hud->setClearMask(GL_DEPTH_BUFFER_BIT);
    hud->addChild(genTacAid());
    scene->addChild(hud);

    osgEarth::GLUtils::setGlobalDefaults(hud->getOrCreateStateSet());
    viewer.realize();

    return viewer.run();
}