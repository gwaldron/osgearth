Coordinate Systems
==================

Between OpenGL, OSG, and osgEarth, there are several different coordinate systems
and reference frames in use and it can get confusing sometimes which is which.
Here we will cover some of the basics.

OpenSceneGraph/OpenGL Coordinate Spaces
---------------------------------------
Here is a brief explaination of the various coordinate systems used in OpenGL and
OSG. For a more detailed explaination (with pictures!) we direct you to read this
excellent tutorial on the subject:

    `OpenGL Transformation`_

Model Coordinates
.................
Model (or Object) space refers to the actual coordinates in the geometry (like
terrain tiles, an airplane model, etc). In OSG, model coordinates might be absolute
or they might be transformed with an OSG ``Transform``.

We will often refer to two types of Model coordinates: *world* and *local*.

*World coordinates* are expressed in absolute terms; they are not transformed.
*Local coordinates* have been transformed to make them relative to some reference
point (in *world* coordinates).

Why use local coordinates? Because OpenGL hardware can only handle 32-bit values for
vertex locations. But in a system like osgEarth, we need to represent locations with
large values and we cannot do that without exceeding the limits of 32-bit precision.
The solution is to use *local coordinates*. OSG uses a double-precision ``MatrixTransform``
to create a local origin (0,0,0), and then we can express our data relative to that.

View Coordinates
................
View space (sometimes called *camera* or *eye* space) express the position of
geometry relative to the camera itself. The camera is at the origin (0,0,0) and
the coordinate axes are::

    +X : Right
    +Y : Up
    -Z : Forward (direction the camera is looking)

In osgEarth, View space is used quite a bit in *vertex shaders* -- they operate on
the GPU which is limited to 32-bit precision, and View space has a *local origin*
at the camera.

Clip Coordinates
................
*Clip* coordinate are what you get after applying the view volume (also know as the
camera frustum). The frustum defines the limits of what you can see from the eyepoint.
The resulting coordinates are in this system::

    +X : Right
    +Y : Up
    +Z : Forward
    
Clip spaces uses 4-dimensional homogeneous coordinates. The range of values in clip
space encompasses the camera frustum and is expressed thusly::

    X : [-w..w] (-w = left,   +w = right)
    Y : [-w..w] (-w = bottom, +w = top)
    Z : [-w..w] (-w = near,   +w = far)
    W : perspective divisor
    
Note that the Z value, which represents *depth*, is non-linear. There is much more 
precision closer to the near plane.

Clip space is useful in a *shader* when you need to sample or manipulator *depth*
information in the scene.


.. _`OpenGL Transformation`: http://www.songho.ca/opengl/gl_transform.html

