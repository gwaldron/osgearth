/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2010 Pelican Mapping
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
#include <osgEarth/OverlayDecorator>
#include <osgEarth/FindNode>
#include <osgEarth/Registry>
#include <osgEarth/TextureCompositor>
#include <osg/Texture2D>
#include <osg/TexEnv>
#include <osg/ComputeBoundsVisitor>
#include <osgShadow/ConvexPolyhedron>
#include <osgUtil/LineSegmentIntersector>
#include <iomanip>

#define LC "[OverlayDecorator] "

using namespace osgEarth;

//---------------------------------------------------------------------------

OverlayDecorator::OverlayDecorator() :
_textureUnit( 1 ),
_textureSize( 1024 ),
_reservedTextureUnit( false ),
_useShaders( false ),
_useWarping( true ),
_warp( 1.0f ),
_visualizeWarp( false )
{
    // force an update traversal:
    ADJUST_UPDATE_TRAV_COUNT( this, 1 );

    // points to children of this group. We will override the traverse to route through
    // this container. That way we can assign a stateset to the children without 
    // actually modifying them
    _subgraphContainer = new osg::Group();
}

void
OverlayDecorator::reinit()
{
    // need to pre-allocate the image here, otherwise the RTT images won't have an alpha channel:
    osg::Image* image = new osg::Image();
    image->allocateImage( *_textureSize, *_textureSize, 1, GL_RGBA, GL_UNSIGNED_BYTE );
    image->setInternalTextureFormat( GL_RGBA8 );    

    _projTexture = new osg::Texture2D( image );
    _projTexture->setTextureSize( *_textureSize, *_textureSize );
    _projTexture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
    _projTexture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
    _projTexture->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_BORDER );
    _projTexture->setWrap( osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_BORDER );
    _projTexture->setWrap( osg::Texture::WRAP_R, osg::Texture::CLAMP_TO_BORDER );
    _projTexture->setBorderColor( osg::Vec4(0,0,0,0) );

    // set up the RTT camera:
    _rttCamera = new osg::Camera();
    _rttCamera->setClearColor( osg::Vec4f(0,0,0,0) );
    _rttCamera->setReferenceFrame( osg::Camera::ABSOLUTE_RF );
    _rttCamera->setViewport( 0, 0, *_textureSize, *_textureSize );
    _rttCamera->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );
    _rttCamera->setRenderOrder( osg::Camera::PRE_RENDER );
    _rttCamera->setRenderTargetImplementation( osg::Camera::FRAME_BUFFER_OBJECT );
    _rttCamera->attach( osg::Camera::COLOR_BUFFER0, _projTexture.get() );
    _rttCamera->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );

    // texture coordinate generator:
    _texGenNode = new osg::TexGenNode();
    _texGenNode->setTextureUnit( *_textureUnit );
    
    // attach the overlay graph to the RTT camera.
    if ( _overlayGraph.valid() && ( _overlayGraph->getNumParents() == 0 || _overlayGraph->getParent(0) != _rttCamera.get() ))
    {
        if ( _rttCamera->getNumChildren() > 0 )
            _rttCamera->replaceChild( 0, _overlayGraph.get() );
        else
            _rttCamera->addChild( _overlayGraph.get() );
    }

    // assemble the subgraph stateset:
    _subgraphStateSet = new osg::StateSet();
    //osg::StateSet* subgraphSet = new osg::StateSet();
    _subgraphContainer->setStateSet( _subgraphStateSet.get() );

    if ( _overlayGraph.valid() )
    {
        // set up the subgraph to receive the projected texture:
        _subgraphStateSet->setTextureMode( *_textureUnit, GL_TEXTURE_GEN_S, osg::StateAttribute::ON );
        _subgraphStateSet->setTextureMode( *_textureUnit, GL_TEXTURE_GEN_T, osg::StateAttribute::ON );
        _subgraphStateSet->setTextureMode( *_textureUnit, GL_TEXTURE_GEN_R, osg::StateAttribute::ON );
        _subgraphStateSet->setTextureMode( *_textureUnit, GL_TEXTURE_GEN_Q, osg::StateAttribute::ON );
        _subgraphStateSet->setTextureAttributeAndModes( *_textureUnit, _projTexture.get(), osg::StateAttribute::ON );

        // decalling:
        osg::TexEnv* env = new osg::TexEnv();
        env->setMode( osg::TexEnv::DECAL );
        _subgraphStateSet->setTextureAttributeAndModes( *_textureUnit, env, osg::StateAttribute::ON );
        
        // set up the shaders
        if ( _useShaders )
        {            
            initSubgraphShaders( _subgraphStateSet.get() );
            initRTTShaders( _rttCamera->getOrCreateStateSet() );

            _warpUniform = this->getOrCreateStateSet()->getOrCreateUniform( "warp", osg::Uniform::FLOAT );
            _warpUniform->set( 1.0f );
        }
    }
}

void
OverlayDecorator::initRTTShaders( osg::StateSet* set )
{
    //TODO: convert this to VP so the overlay graph can use shadercomp too.
    osg::Program* program = new osg::Program();
    program->setName( "OverlayDecorator RTT shader" );
    set->setAttributeAndModes( program, osg::StateAttribute::ON );

    std::stringstream buf;
    buf << "#version 110 \n"
        << "uniform float warp; \n"

        // because the built-in pow() is busted
        << "float mypow( in float x, in float y ) \n"
        << "{ \n"
        << "    return x/(x+y-y*x); \n"
        << "} \n"

        << "vec4 warpVertex( in vec4 src ) \n"
        << "{ \n"
        //      normalize to [-1..1], then take the absolute values since we
        //      want to apply the warping in [0..1] on each side of zero:
        << "    vec2 srct = vec2( abs(src.x)/src.w, abs(src.y)/src.w ); \n"
        << "    vec2 sign = vec2( src.x > 0.0 ? 1.0 : -1.0, src.y > 0.0 ? 1.0 : -1.0 ); \n"

        //      apply the deformation using a "deceleration" curve:
        << "    vec2 srcp = vec2( 1.0-mypow(1.0-srct.x,warp), 1.0-mypow(1.0-srct.y,warp) ); \n"

        //      re-apply the sign. no need to un-normalize, just use w=1 instead
        << "    return vec4( sign.x*srcp.x, sign.y*srcp.y, src.z/src.w, 1.0 ); \n"
        << "} \n"

        << "void main() \n"
        << "{ \n"
        << "    gl_Position = warpVertex( gl_ModelViewProjectionMatrix * gl_Vertex ); \n"
        //<< "    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex; \n"
        << "    gl_FrontColor = gl_Color; \n"
        << "} \n";

    std::string vertSource = buf.str();
    program->addShader( new osg::Shader( osg::Shader::VERTEX, vertSource ) );
}

void
OverlayDecorator::initSubgraphShaders( osg::StateSet* set )
{
    VirtualProgram* vp = new VirtualProgram();
    vp->setName( "OverlayDecorator subgraph shader" );
    set->setAttributeAndModes( vp, osg::StateAttribute::ON );

    // sampler for projected texture:
    set->getOrCreateUniform( "osgearth_overlay_ProjTex", osg::Uniform::SAMPLER_2D )->set( *_textureUnit );

    // the texture projection matrix uniform.
    _texGenUniform = set->getOrCreateUniform( "osgearth_overlay_TexGenMatrix", osg::Uniform::FLOAT_MAT4 );

    std::stringstream buf;

    // vertex shader - subgraph
    buf << "#version 110 \n"
        << "uniform mat4 osgearth_overlay_TexGenMatrix; \n"
        << "uniform mat4 osg_ViewMatrixInverse; \n"

        << "void osgearth_overlay_vertex(void) \n"
        << "{ \n"
        << "    gl_TexCoord["<< *_textureUnit << "] = osgearth_overlay_TexGenMatrix * osg_ViewMatrixInverse * gl_ModelViewMatrix * gl_Vertex; \n"
        << "} \n";

    std::string vertexSource = buf.str();
    vp->setFunction( "osgearth_overlay_vertex", vertexSource, ShaderComp::LOCATION_VERTEX_POST_LIGHTING );

    // fragment shader - subgraph
    buf.str("");
    buf << "#version 110 \n"
        << "uniform sampler2D osgearth_overlay_ProjTex; \n"
        << "uniform float warp; \n"

        // because the built-in pow() is busted
        << "float mypow( in float x, in float y ) \n"
        << "{ \n"
        << "    return x/(x+y-y*x); \n"
        << "} \n"

        << "vec2 warpTexCoord( in vec2 src ) \n"
        << "{ \n"
        //      incoming tex coord is [0..1], so we scale to [-1..1]
        << "    vec2 srcn = vec2( src.x*2.0 - 1.0, src.y*2.0 - 1.0 ); \n" 
        
        //      we want to work in the [0..1] space on each side of 0, so can the abs
        //      and store the signs for later:
        << "    vec2 srct = vec2( abs(srcn.x), abs(srcn.y) ); \n"
        << "    vec2 sign = vec2( srcn.x > 0.0 ? 1.0 : -1.0, srcn.y > 0.0 ? 1.0 : -1.0 ); \n"

        //      apply the deformation using a deceleration curve:
        << "    vec2 srcp = vec2( 1.0-mypow(1.0-srct.x,warp), 1.0-mypow(1.0-srct.y,warp) ); \n"

        //      reapply the sign, and scale back to [0..1]:
        << "    vec2 srcr = vec2( sign.x*srcp.x, sign.y*srcp.y ); \n"
        << "    return vec2( 0.5*(srcr.x + 1.0), 0.5*(srcr.y + 1.0) ); \n"
        << "} \n"

        << "void osgearth_overlay_fragment( inout vec4 color ) \n"
        << "{ \n"
        << "    vec2 texCoord = gl_TexCoord["<< *_textureUnit << "].xy / gl_TexCoord["<< *_textureUnit << "].q; \n";

    if ( !_visualizeWarp )
        buf  << "    texCoord = warpTexCoord( texCoord ); \n";

    buf << "    vec4 texel = texture2D(osgearth_overlay_ProjTex, texCoord); \n"
        << "    color = vec4( mix( color.rgb, texel.rgb, texel.a ), color.a); \n"
        << "} \n";

    std::string fragmentSource = buf.str();
    vp->setFunction( "osgearth_overlay_fragment", fragmentSource, ShaderComp::LOCATION_FRAGMENT_PRE_LIGHTING );
}

void
OverlayDecorator::setOverlayGraph( osg::Node* node )
{
    if ( _overlayGraph.get() != node )
    {
        _overlayGraph = node;
        reinit();
    }
}

void
OverlayDecorator::setTextureSize( int texSize )
{
    if ( texSize != _textureSize.value() )
    {
        _textureSize = texSize;
        reinit();
    }
}

void
OverlayDecorator::setTextureUnit( int texUnit )
{
    if ( texUnit != _textureUnit.value() )
    {
        _textureUnit = texUnit;
        reinit();
    }
}

void
OverlayDecorator::onInstall( TerrainEngineNode* engine )
{
    // establish the earth's major axis:
    MapInfo info(engine->getMap());
    _ellipsoid = info.getProfile()->getSRS()->getEllipsoid();

    // see whether we want shader support:
    _useShaders = engine->getTextureCompositor()->usesShaderComposition();

    if ( !_textureUnit.isSet() && _useShaders )
    {
        int texUnit;
        if ( engine->getTextureCompositor()->reserveTextureImageUnit( texUnit ) )
        {
            _textureUnit = texUnit;
            _reservedTextureUnit = true;
            OE_INFO << LC << "Reserved texture image unit " << *_textureUnit << std::endl;
        }
    }

    if ( !_textureSize.isSet() )
    {
        int maxSize = Registry::instance()->getCapabilities().getMaxTextureSize();
        _textureSize.init( osg::minimum( 4096, maxSize ) );

        OE_INFO << LC << "Using texture size = " << *_textureSize << std::endl;
    }

    // rebuild dynamic elements.
    reinit();
}

void
OverlayDecorator::onUninstall( TerrainEngineNode* engine )
{
    if ( _reservedTextureUnit )
    {
        engine->getTextureCompositor()->releaseTextureImageUnit( *_textureUnit );
        _textureUnit.unset();
        _reservedTextureUnit = false;
    }
}

void
OverlayDecorator::updateRTTCamera( osg::NodeVisitor& nv )
{
    // configure the RTT camera:
    _rttCamera->setViewMatrix( _rttViewMatrix );
    _rttCamera->setProjectionMatrix( _rttProjMatrix );

    // configure the Projector camera:
    osg::Matrix MVP = _projectorViewMatrix * _projectorProjMatrix;
    osg::Matrix MVPT = MVP * osg::Matrix::translate(1.0,1.0,1.0) * osg::Matrix::scale(0.5,0.5,0.5);
    _texGenNode->getTexGen()->setMode( osg::TexGen::EYE_LINEAR );
    _texGenNode->getTexGen()->setPlanesFromMatrix( MVPT );
    
    // uniform update:
    if ( _useShaders )
    {
        _texGenUniform->set( MVPT );
        if ( _useWarping )
            _warpUniform->set( _warp );
    }
}

static int s_frame = 1;

namespace
{
    /**
     * This method takes a set of verts and finds the nearest and farthest distances from
     * the points to the camera. It does this calculation in the plane defined by the
     * look vector. 
     */
    void
    getMinMaxExtentInSilhouette(const osg::Vec3d& cam, const osg::Vec3d& look, 
                                std::vector<osg::Vec3d>& verts,
                                double& out_eMin, double& out_eMax )
    {
        double minSqrDist2D = DBL_MAX;
        double maxSqrDist2D = -DBL_MAX;
        osg::Plane plane( look, cam );

        for( std::vector<osg::Vec3d>::iterator i = verts.begin(); i != verts.end(); ++i )
        {
            osg::Vec3d& point = *i;

            // project the vert onto the camera plane:
            double signedDist = plane.distance( point );
            point += (-plane.getNormal() * signedDist);

            // then calculate the 2D distance to the camera:
            double sqrDist2D = (cam-point).length2();
            if ( sqrDist2D > maxSqrDist2D )
                maxSqrDist2D = sqrDist2D;
            if ( sqrDist2D < minSqrDist2D )
                minSqrDist2D = sqrDist2D;
        }

        out_eMin = sqrt( minSqrDist2D );
        out_eMax = sqrt( maxSqrDist2D );
    }
}

void
OverlayDecorator::cull( osgUtil::CullVisitor* cv )
{
    osg::Vec3 eye = cv->getEyePoint();
    double eyeLen = eye.length();

    // point the RTT camera straight down from the eyepoint:
    _rttViewMatrix = osg::Matrixd::lookAt( eye, osg::Vec3(0,0,0), osg::Vec3(0,0,1) );

    // find our HAE (not including terrain), and clamp it to some arbitrary value.
    double hae, lat, lon;
    _ellipsoid->convertXYZToLatLongHeight( eye.x(), eye.y(), eye.z(), lat, lon, hae );
    hae = osg::maximum( hae, 100.0 );

    osg::Vec3d worldUp = _ellipsoid->computeLocalUpVector(eye.x(), eye.y(), eye.z());
    osg::Vec3d lookVector = cv->getLookVectorLocal();
    double haeWeight = osg::absolute(worldUp * lookVector);

    // radius of the earth under the eyepoint
    double radius = eyeLen - hae; 

    // approximate distance to the visible horizon
    double horizonDistance = sqrt( 2.0 * radius * hae ); 

    // unit look-vector of the eye:
    osg::Vec3d from, to, up;
    const osg::Matrix& mvMatrix = *cv->getModelViewMatrix();
    mvMatrix.getLookAt( from, to, up, eyeLen);
    osg::Vec3 camLookVec = to-from;
    camLookVec.normalize();

    // unit look-vector of the RTT camera:
    osg::Vec3d rttLookVec = -worldUp;
    
    // calculate the distance to the horizon, projected into the RTT camera plane.
    // This is the maximum limit of eMax since there is no point in drawing overlay
    // data beyond the visible horizon.
    double pitchAngleOfHorizon_rad = acos( horizonDistance/eyeLen );
    double horizonDistanceInRTTPlane = horizonDistance * sin( pitchAngleOfHorizon_rad );

    // cull the subgraph here, since we need its clip planes.
    _subgraphContainer->accept( *cv );
    cv->computeNearPlane();

    // clamp the projection matrix to optimized clip planes.
    osg::Matrixd projMatrix = *cv->getProjectionMatrix();
    double fovy, aspectRatio, zfar, znear;
    cv->getProjectionMatrix()->getPerspective( fovy, aspectRatio, znear, zfar );
    double maxDistance = (1.0 - haeWeight)  * horizonDistance  + haeWeight * hae;
    maxDistance *= 1.5;
    if (zfar - znear >= maxDistance)
        zfar = znear + maxDistance;
    projMatrix.makePerspective( fovy, aspectRatio, znear, zfar );
   
    // contruct the polyhedron representing the viewing frustum.
    osgShadow::ConvexPolyhedron frustumPH;
    frustumPH.setToUnitFrustum( true, true );
    osg::Matrixd MVP = *cv->getModelViewMatrix() * projMatrix;
    osg::Matrixd inverseMVP;
    inverseMVP.invert(MVP);
    frustumPH.transform( inverseMVP, MVP );

    // make a polyhedron representing the viewing frustum of the overlay, and cut it to
    // intersect the viewing frustum:
    osgShadow::ConvexPolyhedron visiblePH;

    const osg::BoundingSphere& bs = _subgraphContainer->getBound();
    //visiblePH.setToBoundingBox( osg::BoundingBox( -bs.radius(), -bs.radius(), -bs.radius(), bs.radius(), bs.radius(), bs.radius() ) );

    // get the bounds of the model. 
    osg::ComputeBoundsVisitor cbbv(osg::NodeVisitor::TRAVERSE_ACTIVE_CHILDREN);
    _subgraphContainer->accept(cbbv);
    visiblePH.setToBoundingBox(cbbv.getBoundingBox());

    // this intersects the viewing frustum with the subgraph's bounding box, basically giving us
    // a "minimal" polyhedron containing all potentially visible geometry. (It can't be truly 
    // minimal without clipping at the geometry level, but that would probably be too expensive.)
    visiblePH.cut( frustumPH );

#if 0
    // dumps a copy of the PH to disk...handy
    if ( s_frame++ % 100 == 0 )
    {
        visiblePH.dumpGeometry();
        OE_INFO << "DUMP" << std::endl;
    }
#endif

    // calculate the extents for our orthographic RTT camera (clamping it to the
    // visible horizon)
    std::vector<osg::Vec3d> verts;
    visiblePH.getPoints( verts );

    double eMin, eMax;
    getMinMaxExtentInSilhouette( from, rttLookVec, verts, eMin, eMax ); //rttLookVec, verts, eMin, eMax );
    eMax = osg::minimum( eMax, horizonDistanceInRTTPlane ); 

    _rttProjMatrix = osg::Matrix::ortho( -eMax, eMax, -eMax, eMax, /*1.0*/ -eyeLen, eyeLen );

    if ( _useWarping )
    {
        // calculate the warping paramaters. This uses shaders to warp the verts and
        // tex coords to favor data closer to the camera when necessary.

    #define WARP_LIMIT 3.0

        double devStrength = 1.0 - ( camLookVec * rttLookVec ); // eye pitch relative to rtt pitch
        double haeStrength = 1.0 - osg::clampBetween( hae/radius, 0.0, 1.0 );

        _warp = 1.0 + devStrength * haeStrength * WARP_LIMIT;

        if ( _visualizeWarp )
            _warp = 4.0;

        //OE_INFO << LC << std::fixed
        //    << "hae=" << hae
        //    << ", eMin=" << eMin
        //    << ", eMax=" << eMax
        //    //<< ", ratio=" << ratio
        //    //<< ", dev=" << devStrength
        //    //<< ", has=" << haeStrength
        //    << ", warp=" << _warp
        //    << std::endl;
    }

#if 0
    if ( s_frame++ % 100 == 0 )
    {
        osgShadow::ConvexPolyhedron rttPH;
        rttPH.setToUnitFrustum( true, true );
        osg::Matrixd MVP = _rttViewMatrix * _rttProjMatrix;
        osg::Matrixd inverseMVP;
        inverseMVP.invert(MVP);
        rttPH.transform( inverseMVP, MVP );
        rttPH.dumpGeometry();
    }
#endif

    // projector matrices are the same as for the RTT camera. Tim was right.
    _projectorViewMatrix = _rttViewMatrix;
    _projectorProjMatrix = _rttProjMatrix;
}

void
OverlayDecorator::traverse( osg::NodeVisitor& nv )
{
    if ( _overlayGraph.valid() )
    {
        if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
        {
            osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>( &nv );
            if ( cv )
            {
                cull( cv );
            }
            _rttCamera->accept( nv );
            
            // note: texgennode doesn't need a cull, and subgraphContainer
            // is traversed in cull().
            //_texGenNode->accept( nv );
            //_subgraphContainer->accept( nv );
        }

        else
        {
            if ( nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
            {
                updateRTTCamera( nv );
            }
            _rttCamera->accept( nv );
            _texGenNode->accept( nv );
            _subgraphContainer->accept( nv );
        }    
    }
    else
    {
        _subgraphContainer->accept( nv );
    }
}


/** Override all the osg::Group methods: */

bool 
OverlayDecorator::addChild( Node *child ) {
    if ( !child ) return false;
    dirtyBound();
    return _subgraphContainer->addChild( child );
}
bool 
OverlayDecorator::insertChild( unsigned int index, Node *child ) {
    if ( !child ) return false;
    dirtyBound();
    return _subgraphContainer->insertChild( index, child );
}
bool 
OverlayDecorator::removeChildren(unsigned int pos,unsigned int numChildrenToRemove) {
    dirtyBound();
    return _subgraphContainer->removeChildren( pos, numChildrenToRemove );
}
bool 
OverlayDecorator::replaceChild( Node *origChild, Node* newChild ) {
    dirtyBound();
    return _subgraphContainer->replaceChild( origChild, newChild );
}
bool 
OverlayDecorator::setChild( unsigned  int i, Node* node ) {
    dirtyBound();
    return _subgraphContainer->setChild( i, node );
}

osg::BoundingSphere
OverlayDecorator::computeBound() const {
    return _subgraphContainer->computeBound();
}
