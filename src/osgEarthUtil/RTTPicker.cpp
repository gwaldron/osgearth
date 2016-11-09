/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
 * http://osgearth.org
 *
 * osgEarth is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include <osgEarthUtil/RTTPicker>
#include <osgEarth/VirtualProgram>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/ShaderLoader>
#include <osgEarth/ObjectIndex>

#include <osgDB/WriteFile>
#include <osg/BlendFunc>

using namespace osgEarth;
using namespace osgEarth::Util;

#define LC "[RTTPicker] "

namespace
{
    // SHADERS for the RTT pick camera.

    const char* pickVertexEncode =
        "#version " GLSL_VERSION_STR "\n"

        "#pragma vp_entryPoint oe_pick_encodeObjectID\n"
        "#pragma vp_location   vertex_clip\n"
        
        "uint oe_index_objectid; \n"                        // Vertex stage global containing the Object ID; set in ObjectIndex shader.

        "flat out vec4 oe_pick_encoded_objectid; \n"        // output encoded oid to fragment shader
        "flat out int oe_pick_color_contains_objectid; \n"  // whether color already contains oid (written by another RTT camera)

        "void oe_pick_encodeObjectID(inout vec4 vertex) \n"
        "{ \n"
        "    oe_pick_color_contains_objectid = (oe_index_objectid == 1u) ? 1 : 0; \n"
        "    if ( oe_pick_color_contains_objectid == 0 ) \n"
        "    { \n"
        "        float b0 = float((oe_index_objectid & 0xff000000u) >> 24u); \n"
        "        float b1 = float((oe_index_objectid & 0x00ff0000u) >> 16u); \n"
        "        float b2 = float((oe_index_objectid & 0x0000ff00u) >> 8u ); \n"
        "        float b3 = float((oe_index_objectid & 0x000000ffu)       ); \n"
        "        oe_pick_encoded_objectid = vec4(b0, b1, b2, b3) / 255.0; \n"
        "    } \n"
        "} \n";

    const char* pickFragment =
        "#version " GLSL_VERSION_STR "\n"

        "#pragma vp_entryPoint oe_pick_renderEncodedObjectID\n"
        "#pragma vp_location   fragment_output\n"
        "#pragma vp_order      last\n"

        "flat in vec4 oe_pick_encoded_objectid; \n"
        "flat in int oe_pick_color_contains_objectid; \n"
        
        "out vec4 fragColor; \n"

        "void oe_pick_renderEncodedObjectID(inout vec4 color) \n"
        "{ \n"
        "    if ( oe_pick_color_contains_objectid == 1 ) \n"
        "        fragColor = color; \n"
        "    else \n"
        "        fragColor = oe_pick_encoded_objectid; \n"
        "} \n";
}

VirtualProgram* 
RTTPicker::createRTTProgram()
{    
    VirtualProgram* vp = new VirtualProgram();
    vp->setName( "osgEarth::RTTPicker" );

    // Install RTT picker shaders:
    ShaderPackage pickShaders;
    pickShaders.add( "RTTPicker.vert.glsl", pickVertexEncode );
    pickShaders.add( "RTTPicker.frag.glsl", pickFragment );
    pickShaders.loadAll( vp );

    // Install shaders and bindings from the ObjectIndex:
    Registry::objectIndex()->loadShaders( vp );

    return vp;
}

RTTPicker::RTTPicker(int cameraSize)
{
    // group that will hold RTT children for all cameras
    _group = new osg::Group();

    // Size of the RTT camera image
    _rttSize = std::max(cameraSize, 4);    

    // pixels around the click to test
    _buffer = 2;
}

RTTPicker::~RTTPicker()
{
    // remove the RTT camera from all views
    for(int i=0; i<_pickContexts.size(); ++i)
    {
        PickContext& pc = _pickContexts[i];
        while( pc._pickCamera->getNumParents() > 0 )
        {
            pc._pickCamera->getParent(0)->removeChild( pc._pickCamera.get() );
        }
    }
}

osg::Texture2D*
RTTPicker::getOrCreateTexture(osg::View* view)
{
    PickContext& pc = getOrCreatePickContext(view);
    if ( !pc._tex.valid() )
    {
        pc._tex = new osg::Texture2D( pc._image.get() );
        pc._tex->setTextureSize(pc._image->s(), pc._image->t());
        pc._tex->setUnRefImageDataAfterApply(false);
        pc._tex->setFilter(pc._tex->MIN_FILTER, pc._tex->NEAREST); // no filtering
        pc._tex->setFilter(pc._tex->MAG_FILTER, pc._tex->NEAREST); // no filtering
        pc._tex->setMaxAnisotropy(1.0f); // no filtering
    }
    return pc._tex.get();
}

RTTPicker::PickContext&
RTTPicker::getOrCreatePickContext(osg::View* view)
{
    for(PickContextVector::iterator i = _pickContexts.begin(); i != _pickContexts.end(); ++i)
    {
        if ( i->_view.get() == view )
        {
            return *i;
        }
    }

    // Make a new one:
    _pickContexts.push_back( PickContext() );
    PickContext& c = _pickContexts.back();

    c._view = view;

    c._image = new osg::Image();
    c._image->allocateImage(_rttSize, _rttSize, 1, GL_RGBA, GL_UNSIGNED_BYTE);    
    
    // make an RTT camera and bind it to our imag:
    c._pickCamera = new osg::Camera();
    c._pickCamera->setName( "osgEarth::RTTPicker" );
    c._pickCamera->addChild( _group.get() );
    c._pickCamera->setClearColor( osg::Vec4(0,0,0,0) );
    c._pickCamera->setClearMask( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    c._pickCamera->setReferenceFrame( osg::Camera::ABSOLUTE_RF_INHERIT_VIEWPOINT ); 
    c._pickCamera->setViewport( 0, 0, _rttSize, _rttSize );
    c._pickCamera->setRenderOrder( osg::Camera::PRE_RENDER, 1 );
    c._pickCamera->setRenderTargetImplementation( osg::Camera::FRAME_BUFFER_OBJECT );
    c._pickCamera->attach( osg::Camera::COLOR_BUFFER0, c._image.get() );
    c._pickCamera->setSmallFeatureCullingPixelSize( -1.0f );
    
    osg::StateSet* rttSS = c._pickCamera->getOrCreateStateSet();

    // disable all the things that break ObjectID picking:
    osg::StateAttribute::GLModeValue disable = osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED;

    //rttSS->setMode(GL_BLEND,     disable );    
    rttSS->setMode(GL_LIGHTING,  disable );
    rttSS->setMode(GL_CULL_FACE, disable );
    rttSS->setMode(GL_ALPHA_TEST, disable );
    
    // Disabling GL_BLEND is not enough, because osg::Text re-enables it
    // without regard for the OVERRIDE.
    rttSS->setAttributeAndModes(new osg::BlendFunc(GL_ONE, GL_ZERO, GL_ONE, GL_ZERO), osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED);

    // install the picking shaders:
    VirtualProgram* vp = createRTTProgram();
    rttSS->setAttribute( vp );

    // designate this as a pick camera, overriding any defaults below
    rttSS->addUniform( new osg::Uniform("oe_isPickCamera", true), osg::StateAttribute::OVERRIDE );

    // default value for the objectid override uniform:
    rttSS->addUniform( new osg::Uniform(Registry::objectIndex()->getObjectIDUniformName().c_str(), 0u) );
    
    // install the pick camera on the main camera.
    view->getCamera()->addChild( c._pickCamera.get() );

    // associate the RTT camara with the view's camera.
    // (e.g., decluttering uses this to find the "true" viewport)
    c._pickCamera->setUserData( view->getCamera() );

    return c;
}

bool
RTTPicker::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    if ( ea.getEventType() == ea.FRAME )
    {
        osg::FrameStamp* fs = aa.asView() ? aa.asView()->getFrameStamp() : 0L;
        if ( fs )
        {
            runPicks( fs->getFrameNumber() );           
        }

        // if there are picks in the queue, need to continuing rendering:
        if ( !_picks.empty() )
        {
            aa.requestRedraw();
        }

        // synchronize the pick camera associated with this view
        osg::Camera* cam = aa.asView()->getCamera();
        if (cam)
        {
            PickContext& context = getOrCreatePickContext( aa.asView() );
            context._pickCamera->setViewMatrix( cam->getViewMatrix() );
            context._pickCamera->setProjectionMatrix( cam->getProjectionMatrix() );
        }
    }

    else if ( _defaultCallback.valid() && _defaultCallback->accept(ea, aa) )
    {        
        pick( aa.asView(), ea.getX(), ea.getY(), _defaultCallback.get() );
        aa.requestRedraw();
    }

    return false;
}

bool
RTTPicker::pick(osg::View* view, float mouseX, float mouseY)
{
    return pick(view, mouseX, mouseY, 0L);
}

bool
RTTPicker::pick(osg::View* view, float mouseX, float mouseY, Callback* callback)
{
    if ( !view )
        return false;

    Callback* callbackToUse = callback ? callback : _defaultCallback.get();
    if ( !callbackToUse )
        return false;
    
    osg::Camera* cam = view->getCamera();
    if ( !cam )
        return false;

    const osg::Viewport* vp = cam->getViewport();
    if ( !vp )
        return false;

    // normalize the input cooridnates [0..1]
    float u = (mouseX - (float)vp->x())/(float)vp->width();
    float v = (mouseY - (float)vp->y())/(float)vp->height();

    // check the bounds:
    if ( u < 0.0f || u > 1.0f || v < 0.0f || v > 1.0f )
        return false;

    // install the RTT pick camera under this view's camera if it's not already:
    PickContext& context = getOrCreatePickContext( view );
    
    // Create a new pick
    Pick pick;
    pick._context  = &context;
    pick._u        = u;
    pick._v        = v;
    pick._callback = callbackToUse;
    pick._frame    = view->getFrameStamp() ? view->getFrameStamp()->getFrameNumber() : 0u;
    
    // Queue it up.
    _picks.push( pick );
    
    return true;
}

void
RTTPicker::runPicks(unsigned frameNumber)
{
    while( _picks.size() > 0 )
    {
        Pick& pick = _picks.front();
        if ( frameNumber > pick._frame )
        {
            checkForPickResult(pick);
            _picks.pop();
        }
        else
        {
            break;
        }
    }
}

namespace
{
    // Iterates through the pixels in a grid, starting at u,v [0..1] and spiraling out.
    // It will stop when it reaches the "max ring", which is basically a distance from
    // the starting point.
    // Inspiration: http://stackoverflow.com/a/14010215/4218920
    struct SpiralIterator
    {
        unsigned _ring;
        unsigned _maxRing;
        unsigned _leg;
        int      _x, _y;
        int      _w, _h;
        int      _offsetX, _offsetY;
        unsigned _count;

        SpiralIterator(int w, int h, int maxDist, float u, float v) : 
            _w(w), _h(h), _maxRing(maxDist), _count(0), _ring(1), _leg(0), _x(0), _y(0)
        {
            _offsetX = (int)(u * (float)w);
            _offsetY = (int)(v * (float)h);
        }

        bool next()
        {
            // first time, just use the start point
            if ( _count++ == 0 )
                return true;

            // spiral until we get to the next valid in-bounds pixel:
            do {
                switch(_leg) {
                case 0: ++_x; if (  _x == _ring ) ++_leg; break;
                case 1: ++_y; if (  _y == _ring ) ++_leg; break;
                case 2: --_x; if ( -_x == _ring ) ++_leg; break;
                case 3: --_y; if ( -_y == _ring ) { _leg = 0; ++_ring; } break;
                }
            }
            while(_ring <= _maxRing && (_x+_offsetX < 0 || _x+_offsetX >= _w || _y+_offsetY < 0 || _y+_offsetY >= _h));

            return _ring <= _maxRing;
        }

        int s() const { return _x+_offsetX; }

        int t() const { return _y+_offsetY; }
    };
}

void
RTTPicker::checkForPickResult(Pick& pick)
{
    // decode the results
    osg::Image* image = pick._context->_image.get();
    ImageUtils::PixelReader read( image );

    // uncomment to see the RTT image.
    //osg::ref_ptr<osgDB::Options> o = new osgDB::Options();
    //osgDB::writeImageFile(*image, "out.tif", o.get());

    osg::Vec4f value;
    SpiralIterator iter(image->s(), image->t(), std::max(_buffer,1), pick._u, pick._v);
    while(iter.next())
    {
        value = read(iter.s(), iter.t());

        ObjectID id = (ObjectID)(
            ((unsigned)(value.r()*255.0) << 24) +
            ((unsigned)(value.g()*255.0) << 16) +
            ((unsigned)(value.b()*255.0) <<  8) +
            ((unsigned)(value.a()*255.0)));

        if ( id > 0 )
        {
            pick._callback->onHit( id );
            return;
        }
    }

    pick._callback->onMiss();
}

bool
RTTPicker::addChild(osg::Node* child)
{
    return _group->addChild( child );
}

bool
RTTPicker::insertChild(unsigned i, osg::Node* child)
{
    return _group->insertChild( i, child );
}

bool
RTTPicker::removeChild(osg::Node* child)
{
    return _group->removeChild( child );
}

bool
RTTPicker::replaceChild(osg::Node* oldChild, osg::Node* newChild)
{
    return _group->replaceChild( oldChild, newChild );
}
