/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#include <osgEarthUtil/Shaders>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/GLUtils>

#include <osg/BlendFunc>

using namespace osgEarth;
using namespace osgEarth::Util;

#define LC "[RTTPicker] "

namespace
{
    // Callback to set the "far plane" uniform just before drawing.
    struct CallHostCameraPreDrawCallback : public osg::Camera::DrawCallback
    {
        osg::observer_ptr<osg::Camera> _hostCamera;

        CallHostCameraPreDrawCallback( osg::Camera* hostCamera ) :
            _hostCamera( hostCamera )
        {
        }

        void operator () (osg::RenderInfo& renderInfo) const
        {
            osg::ref_ptr<osg::Camera> hostCamera;
            if ( _hostCamera.lock(hostCamera) )
            {
                const osg::Camera::DrawCallback* hostCallback = hostCamera->getPreDrawCallback();
                if ( hostCallback )
                    hostCallback->operator()( renderInfo );
            }
        }
    };
}

VirtualProgram* 
RTTPicker::createRTTProgram()
{    
    VirtualProgram* vp = new VirtualProgram();
    vp->setName( "osgEarth::RTTPicker" );

    // Install RTT picker shaders:
    Shaders shaders;
    shaders.load(vp, shaders.RTTPicker);

    // Install shaders and bindings from the ObjectIndex:
    Registry::objectIndex()->loadShaders( vp );

    return vp;
}

RTTPicker::RTTPicker(int cameraSize)
{
    // group that will hold RTT children for all cameras
    _group = new osg::Group();

    // Size of the RTT camera image
    _rttSize = osg::maximum(cameraSize, 4);    

    // pixels around the click to test
    _buffer = 2;
    
    // Cull mask for RTT cameras
    _cullMask = ~0u;
}

RTTPicker::~RTTPicker()
{
    // remove the RTT camera from all views
    for (PickContexts::iterator i = _pickContexts.begin(); i != _pickContexts.end(); ++i)
    {
        PickContext& pc = *i;

        osg::ref_ptr<osg::View> view;
        if (pc._view.lock(view))
        {
            unsigned numSlaves = pc._view->getNumSlaves();
            for (unsigned k = 0; k < numSlaves; ++k)
            {
                if (pc._view->getSlave(k)._camera.get() == pc._pickCamera.get())
                {
                    // Remove children of the pick camera so GL objects don't get released when
                    // we remove the slave
                    pc._pickCamera->removeChildren(0, pc._pickCamera->getNumChildren());
                    pc._view->removeSlave(k);
                    break;
                }
            }
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

void
RTTPicker::setCullMask(osg::Node::NodeMask nm)
{
    if ( _cullMask == nm )
        return;
    _cullMask = nm;
    for(PickContexts::const_iterator i = _pickContexts.begin(); i != _pickContexts.end(); ++i)
    {
        i->_pickCamera->setCullMask( _cullMask );
    }
}

namespace
{
    struct MyUpdateSlave : public osg::View::Slave::UpdateSlaveCallback
    {
        void updateSlave(osg::View& view, osg::View::Slave& slave)
        {
            osg::Camera* cam = slave._camera.get();
            cam->setProjectionResizePolicy(view.getCamera()->getProjectionResizePolicy());
            cam->setProjectionMatrix(view.getCamera()->getProjectionMatrix());
            cam->setViewMatrix(view.getCamera()->getViewMatrix());
            cam->inheritCullSettings(*(view.getCamera()), cam->getInheritanceMask());            
        }
    };
}

RTTPicker::PickContext&
RTTPicker::getOrCreatePickContext(osg::View* view)
{
    for(PickContexts::iterator i = _pickContexts.begin(); i != _pickContexts.end(); ++i)
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
    memset(c._image->data(), 0, _rttSize * _rttSize * 4);
    
    // Make an RTT camera and bind it to our image.
    // Note: don't use RF_INHERIT_VIEWPOINT because it's unnecessary and
    //       doesn't work with a slave camera anyway
    // Note: NESTED_RENDER mode makes the RTT camera track the clip planes
    //       etc. of the master camera; since the master renderes first,
    //       the setup should always be in place for the slave
    c._pickCamera = new osg::Camera();
    c._pickCamera->setName( "osgEarth::RTTPicker" );
    c._pickCamera->addChild( _group.get() );
    c._pickCamera->setClearColor( osg::Vec4(0,0,0,0) );
    c._pickCamera->setClearMask( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    c._pickCamera->setViewport( 0, 0, _rttSize, _rttSize );
    c._pickCamera->setRenderOrder( osg::Camera::NESTED_RENDER );
    c._pickCamera->setRenderTargetImplementation( osg::Camera::FRAME_BUFFER_OBJECT );
    c._pickCamera->attach( osg::Camera::COLOR_BUFFER0, c._image.get() );
    c._pickCamera->setSmallFeatureCullingPixelSize( -1.0f );
    c._pickCamera->setCullMask( _cullMask );

    // Necessary to connect the slave to the master (why is this not automatic?)
    c._pickCamera->setGraphicsContext(view->getCamera()->getGraphicsContext());
    
    osg::StateSet* rttSS = c._pickCamera->getOrCreateStateSet();

    // disable all the things that break ObjectID picking:
    osg::StateAttribute::GLModeValue disable = 
        osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED;

    GLUtils::setLighting(rttSS, disable);
    rttSS->setMode(GL_CULL_FACE, disable );
    rttSS->setMode(GL_ALPHA_TEST, disable );

#if !(defined(OSG_GLES2_AVAILABLE) || defined(OSG_GLES3_AVAILABLE) || defined(OSG_GL3_AVAILABLE) )
    rttSS->setMode(GL_POINT_SMOOTH, disable );
    rttSS->setMode(GL_LINE_SMOOTH, disable );
#endif
    
    // Disabling GL_BLEND is not enough, because osg::Text re-enables it
    // without regard for the OVERRIDE.
    rttSS->setAttributeAndModes(
        new osg::BlendFunc(GL_ONE, GL_ZERO, GL_ONE, GL_ZERO),
        osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED);

    // install the picking shaders:
    VirtualProgram* vp = createRTTProgram();
    rttSS->setAttribute( vp );

    // designate this as a pick camera
    rttSS->setDefine("OE_IS_PICK_CAMERA");
    rttSS->setDefine("OE_LIGHTING", osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

    // default value for the objectid override uniform:
    rttSS->addUniform( new osg::Uniform(Registry::objectIndex()->getObjectIDUniformName().c_str(), 0u) );
    
    // install the pick camera as a slave of the view's camera so it will
    // duplicate the view matrix and projection matrix during the update traversal
    // The "false" means the pick camera has its own separate subgraph.
    view->addSlave(c._pickCamera.get(), false);
    osg::View::Slave& slave = view->getSlave(view->getNumSlaves()-1);
    slave._updateSlaveCallback = new MyUpdateSlave();

    // Pick camera starts out deactivated.
    c._numPicks = 0;
    c._pickCamera->setNodeMask(0);

    // Add a pre-draw callback that calls the view camera's pre-draw callback.  This
    // is better than assigning the same pre-draw callback, because the callback can
    // change over time (such as installing or uninstalling a Logarithmic Depth Buffer)
    c._pickCamera->setPreDrawCallback( new CallHostCameraPreDrawCallback(view->getCamera()) );

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
        if ( _picks.empty() == false )
        {
            aa.requestRedraw();
        }
    }

    if ( _defaultCallback.valid() && _defaultCallback->accept(ea, aa) )
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
    if ( u < 0.0f || u >= 1.0f || v < 0.0f || v >= 1.0f )
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
    _picks.push_back( pick );
    
    // Activate the pick camera if necessary:
    pick._context->_numPicks++;
    if (pick._context->_numPicks == 1)
    {
        pick._context->_pickCamera->setNodeMask(~0u);
    }
    
    return true;
}

void
RTTPicker::runPicks(unsigned frameNumber)
{
    if (_picks.size() > 0)
    {
        for (std::vector<Pick>::iterator i = _picks.begin(); i != _picks.end(); )
        {
            bool pickExpired = false;
            Pick& pick = *i;
            if (frameNumber > pick._frame)
            {
                pickExpired = checkForPickResult(pick, frameNumber);
                if (pickExpired)
                {
                    // Decrement the pick count for this pick's camera. If it reaches zero,
                    // disable the camera.
                    pick._context->_numPicks--;
                    if (pick._context->_numPicks == 0)
                    {
                        pick._context->_pickCamera->setNodeMask(0);
                    }

                    // Remove the pick.
                    i = _picks.erase(i);
                }
            }

            if (pickExpired == false)
            {
                ++i;
            }
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
            if (_count == 0)
            {
                if (_offsetX < 0 || _offsetX >= _w || _offsetY < 0 || _offsetY >= _h)
                    return false;
                else
                {
                    _count++;
                    return true;
                }
            }

            // spiral until we get to the next valid in-bounds pixel:
            do {
                switch(_leg) {
                case 0: ++_x; if (  _x == (int)_ring ) ++_leg; break;
                case 1: ++_y; if (  _y == (int)_ring ) ++_leg; break;
                case 2: --_x; if ( -_x == (int)_ring ) ++_leg; break;
                case 3: --_y; if ( -_y == (int)_ring ) { _leg = 0; ++_ring; } break;
                }
            }
            while(_ring <= _maxRing && (_x+_offsetX < 0 || _x+_offsetX >= _w || _y+_offsetY < 0 || _y+_offsetY >= _h));

            return _ring <= _maxRing;
        }

        int s() const { return _x+_offsetX; }

        int t() const { return _y+_offsetY; }
    };
}

bool
RTTPicker::checkForPickResult(Pick& pick, unsigned frameNumber)
{
    // decode the results
    osg::Image* image = pick._context->_image.get();
    ImageUtils::PixelReader read( image );

    // uncomment to see the RTT image.
    //osg::ref_ptr<osgDB::Options> o = new osgDB::Options();
    //osgDB::writeImageFile(*image, "out.tif", o.get());

    bool hit = false;
    osg::Vec4f value;
    SpiralIterator iter(image->s(), image->t(), osg::maximum(_buffer,1), pick._u, pick._v);
    while (iter.next() && (hit == false))
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
            hit = true;
        }
    }

    // A pick expires if (a) it registers a hit, or (b) is registers a miss
    // for 2 frames in a row. Why 2? Because the RTT picker itself delays 
    // pick results by one frame, and the osgEarth draping/clamping systems
    // also delay drawing by one frame. So we need 2 frames to positively
    // register a hit on draped/clamped geometry.
    bool pickExpired =
        hit == true ||
        frameNumber - pick._frame >= 2u;

    if ((hit == false) && (pickExpired == true))
    {
        pick._callback->onMiss();
    }

    return pickExpired;
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
