# FAQ

## Runtime

#### Error: 'gl_ModelViewMatrix is removed after version 140'

osgEarth requires that you link against OpenSceneGraph that was built with support for OpenGL 3+. If you are building OSG yourself, be sure to set the CMake flag `OSG_GL3_AVAILABLE=ON`.

The most common symptom is error messages like this:
```
VERTEX Shader "oe_rex_morph" infolog:
0(94) : error C7616: global variable gl_ModelViewMatrix is removed after version 140
0(124) : error C7616: global variable gl_MultiTexCoord1 is removed after version 140
0(125) : error C7616: global variable gl_MultiTexCoord2 is removed after version 140
```

## Earth File

#### Placing a 3D model in the Earth File

You can place an object as an `Annotation`. Annotations must live inside an `Annotations` layer. With this approach is that you can model symbology to style your model. This approach looks like this:
```xml
<Annotations>
    <Model>
        <position srs="wgs84" lat="43" long="-100"/>
        <style>
            model: "../data/red_flag.osg.45.scale";
            model-scale: auto;
        </style>
    </Model>
</Annotations>
```

Another way is to use a `ModelLayer` like so:
```xml
<Model name="My Model Layer">
    <url>location_of_my_3d_model.osgb</url>
    <location srs="wgs84" lat="34.0" long="-80.4"/>
</Model>
```

Please refer rhe various `feature` earth files in the repository for more information and examples.

## API

#### Placing a 3D model using the API

The `osgEarth::GeoTransform` class inherits from `osg::Transform` and will convert map coordinates into OSG world coordinates for you. Place an object at a geospatial position like this:
```c++
GeoTransform* xform = new GeoTransform();
GeoPoint point(srs, -121.0, 34.0, 1000.0);
xform->setPosition(point);
```

If you want your object to automatically clamp to the terrain surface, just leave off the altitude:
```c++
GeoTransform* xform = new GeoTransform();
GeoPoint point(srs, -121.0, 34.0);
xform->setPosition(point);
```

That will work as long as your object is anywhere in the scene graph under the `osgEarth::MapNode`. If your node is *not* under the `MapNode`, you will need to manually tell it where to find the terrain:
```
xform->setTerrain(mapNode->getTerrain());
```

The `srs` object in these examples is the `SpatialReference` of the coordinates. For standard **WGS84** longitude/latitude coordinates, you can get a spatial reference by calling
```c++
auto srs = SpatialReference::get("wgs84");
```

---
#### Models have no textures or lighting

Everything under an osgEarth scene graph is rendered with shaders. So, when using your own models (or creating geometry by hand) you need to create shader components in order for them to render properly.

osgEarth has a built-in shader generator for this purpose.
Run the shader generator on your node like so:

```C++
osgEarth::Registry::shaderGenerator().run( myNode );
```

After that, your node will contain shader snippets that allows osgEarth to render it properly and for it to work with other osgEarth features like sky lighting.


---
#### Lines or Annotations do not render

Lines render using a shader that requires some initial state to be set. You can apply this state to your top-level camera (or anywhere else above the geometry) like so:

```c++
#include <osgEarth/GLUtils>
...
GLUtils::setGlobalDefaults(camera->getOrCreateStateSet());
```

For Annotations (`FeatureNode`, `PlaceNode`, etc.) best practice is to place an Annotation node as a descendant of the `MapNode` in your scene graph. You can also add them to an ```AnnotationLayer``` and add that layer to the Map.

Annotations need access to the `MapNode` in order to render properly.
If you cannot place them under the `MapNode`, you will have to manually install a few things to make them work:

```c++
#include <osgEarth/CullingUtils>
#include <osgEarth/GLUtils>
...
 
// Manully assign the MapNode to your annotation
annotationNode->setMapNode(mapNode);
 
// In some group above the annotation, install this callback
group->addCullCallback(new InstallViewportSizeUniform());
 
// In some group above the annotation, set the GL defaults
GLUtils::setGlobalDefaults(group->getOrCreateStateSet());
```

Again: `MapNode` does all this automatically so this is only necessary if you do not place your annotations as descendants of the `MapNode`.


---
#### Text annotations (LabelNode, PlaceNode) are not rendering

Rendering text requires that you disable OSG's small-feature culling like so:

```C++
view->getCamera()->setSmallFeatureCullingPixelSize(-1.0f);
```
Note: you must do this for each camera.


---
#### Rendering problems when using the Qt QOpenGLWidget

In your custom `QOpenGLWidget::paintGL()` method, do the following before making any OpenSceneGraph calls:

```c++
void MyQOpenGLWidget::paintGL()
{
   _viewer->getCamera()->getGraphicsContext()->setDefaultFboId(defaultFramebufferObject());
   ...
}
```

In this example `_viewer` refers to an `osgViewer::Viewer`, but any method of accessing the current `osg::Camera` will do.

Here's a more detailed explanation: 

The Qt5 `QOpenGLWidget` renders to an OpenGL framebuffer object (FBO). When you embed an OpenSceneGraph viewer in a `QOpenGLWidget`, OSG does not know about that FBO. It just assumes it's rendering to a display device directly. Some features in osgEarth or OpenSceneGraph also use an FBO for rendering - examples include feature draping and shadowing - and OSG has no way of knowing it needs to reactivate Qt's FBO rendering target afterwards. This can result in a blank display or other rendering issues. Calling `setDefaultFboId` lets OSG know how to reset the rendering target properly.


---
## Community and Support

#### What is the best practice for using GitHub?

The best way to work with the osgEarth repository is to make your own clone on GitHub and to work from that clone. Why not work directly against the main repository? You can, but if you need to make changes, bug fixes, etc., you will need your own clone in order to issue Pull Requests.

1. Create your own GitHub account and log in.
2. Clone the osgEarth repo.
3. Work from your clone. Sync it to the main repository periodically to get the latest changes.

#### How do I submit changes to osgEarth?

We accept contributions and bug fixes through GitHub's [Pull Request](https://help.github.com/articles/using-pull-requests) mechanism.

First you need your own GitHub account and a fork of the repo (see above). Next, follow these guidelines:

1. Create a *branch* in which to make your changes.
2. Make the change.
3. Issue a *pull request* against the main osgEarth repository.
4. We will review the *PR* for inclusion.

If we decide NOT to include your submission, you can still keep it in our cloned repository and use it yourself. Doing so maintains compliance with the osgEarth license since your changes are still available to the public - even if they are not merged into the master repository.


#### Can I hire someone to help me with osgEarth?

Of course! We at Pelican Mapping are in the business of supporting users of the osgEarth SDK and are available for contracting, training, and integration services. The easiest way to get in touch with us is through our web site [contact form](http://pelicanmapping.com/?page_id=2).

