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
#include <iomanip>

#define LC "[OverlayDecorator] "

using namespace osgEarth;

//---------------------------------------------------------------------------

OverlayDecorator::OverlayDecorator() :
_textureUnit( 1 ),
_textureSize( 1024 ),
_reservedTextureUnit( false ),
_useShaders( false ),
_earthRadiusMajor( 6384000.0 )
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
    osg::StateSet* subgraphSet = new osg::StateSet();
    _subgraphContainer->setStateSet( subgraphSet );

    if ( _overlayGraph.valid() )
    {
        // set up the subgraph to receive the projected texture:
        subgraphSet->setTextureMode( *_textureUnit, GL_TEXTURE_GEN_S, osg::StateAttribute::ON );
        subgraphSet->setTextureMode( *_textureUnit, GL_TEXTURE_GEN_T, osg::StateAttribute::ON );
        subgraphSet->setTextureMode( *_textureUnit, GL_TEXTURE_GEN_R, osg::StateAttribute::ON );
        subgraphSet->setTextureMode( *_textureUnit, GL_TEXTURE_GEN_Q, osg::StateAttribute::ON );
        subgraphSet->setTextureAttributeAndModes( *_textureUnit, _projTexture.get(), osg::StateAttribute::ON );

        // decalling:
        osg::TexEnv* env = new osg::TexEnv();
        env->setMode( osg::TexEnv::DECAL );
        subgraphSet->setTextureAttributeAndModes( *_textureUnit, env, osg::StateAttribute::ON );
        
        // set up the shaders
        if ( _useShaders )
        {            
            initSubgraphShaders( subgraphSet );
            initRTTShaders( _rttCamera->getOrCreateStateSet() );

            _warpUniform = new osg::Uniform( osg::Uniform::FLOAT, "warp" );
            _warpUniform->set( 1.0f );
            subgraphSet->addUniform( _warpUniform.get() );
            _rttCamera->getOrCreateStateSet()->addUniform( _warpUniform.get() );
        }
    }
}

void
OverlayDecorator::initRTTShaders( osg::StateSet* set )
{
    //TODO: convert this to VP so the overlay graph can use shadercomp too.
    osg::Program* program = new osg::Program();
    set->setAttributeAndModes( program, osg::StateAttribute::ON );

    std::stringstream buf;
    buf << "#version 110 \n"
        << "uniform float warp; \n"

        << "vec4 warpVertex( in vec4 src ) \n"
        << "{ \n"
#if 0
        << "    float xa = abs(src.x); \n"
        << "    float ya = abs(src.y); \n"
        << "    float wa = abs(src.w); \n"

        << "    float xt = xa/wa; \n"
        << "    float yt = ya/wa; \n"

        << "    float m = 1.0+warp; \n"

        << "    float xtp = 1.0 - pow(1.0-xt, m); \n"
        << "    float ytp = 1.0 - pow(1.0-yt, m); \n"

        << "    float xr = src.x < 0.0 ? -xtp*src.w : xtp*src.w; \n"
        << "    float yr = src.y < 0.0 ? -ytp*src.w : ytp*src.w; \n"

        << "    return vec4( xr, yr, src.z, src.w ); \n"
#endif
        << "    return src; \n"
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

        << "vec2 warpTexCoord( in vec2 src ) \n"
        << "{ \n"
#if 0
        << "    float xn = (2.0*src.x)-1.0; \n"
        << "    float yn = (2.0*src.y)-1.0; \n"

        << "    float xa = abs(xn); \n"
        << "    float ya = abs(yn); \n"
        << "    float wa = 1.0; \n"

        << "    float xt = xa/wa; \n"
        << "    float yt = ya/wa; \n"

        << "    float m = 1.0+warp; \n"

        << "    float xtp = 1.0 - pow(1.0-xt, m); \n"
        << "    float ytp = 1.0 - pow(1.0-yt, m); \n"

        << "    float xr = xn < 0.0 ? -xtp*wa : xtp*wa; \n"
        << "    float yr = yn < 0.0 ? -ytp*wa : ytp*wa; \n"

        << "    xr = 0.5*(xr+1.0); \n"
        << "    yr = 0.5*(yr+1.0); \n"

        << "    return vec2( xr, yr ); \n"
#endif
        << "    return src; \n"
        << "} \n"

        << "void osgearth_overlay_fragment( inout vec4 color ) \n"
        << "{ \n"
        << "    vec2 texCoord = gl_TexCoord["<< *_textureUnit << "].xy / gl_TexCoord["<< *_textureUnit << "].q; \n"
        << "    texCoord = warpTexCoord( texCoord ); \n"
        //<< "    vec4 texel = texture2DProj(osgearth_overlay_ProjTex, texCoord); \n"
        << "    vec4 texel = texture2D(osgearth_overlay_ProjTex, texCoord); \n"
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
    _earthRadiusMajor = info.getProfile()->getSRS()->getEllipsoid()->getRadiusEquator();
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
    if ( nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
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
            _warpUniform->set( static_cast<float>(_warp) );
        }
    }

    else if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
    {
        osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>( &nv );
        if ( !cv ) return;

        osg::Vec3 eye = cv->getEyePoint();
        double eyeLen = eye.length();

        // point the RTT camera straight down from the eyepoint:
        _rttViewMatrix = osg::Matrixd::lookAt( eye, osg::Vec3(0,0,0), osg::Vec3(0,0,1) );

        // find our HAE (not including terrain)
        double hae, lat, lon;
        _ellipsoid->convertXYZToLatLongHeight( eye.x(), eye.y(), eye.z(), lat, lon, hae );
        hae = osg::maximum( hae, 100.0 );        

        // calculate the approximate distance from the eye to the horizon. This is our maximum
        // possible RTT extent:
        double radius = eyeLen-hae;
        double horizonDistance = sqrt( 2.0 * radius * hae ); // distance to the horizon

        // calculate the approximate extent viewed from the camera if it's pointing
        // at the ground. This is the minimum acceptable RTT extent.
        double vfov, camAR, znear, zfar;
        cv->getProjectionMatrix()->getPerspective( vfov, camAR, znear, zfar );
        double eMin = hae * tan( osg::DegreesToRadians(0.5*vfov) );

        // figure out the maximum view distance, based on the camera's pitch and the
        // camera's VFOV.
        osg::Vec3 from, to, up;
        
        const osg::Matrix& mvMatrix = *cv->getModelViewMatrix();
        mvMatrix.getLookAt( from, to, up, eyeLen);
        osg::Vec3 camLookVec = to-from;
        camLookVec.normalize();

        _rttViewMatrix.getLookAt(from,to,up,eyeLen);
        osg::Vec3 rttLookVec = to-from;
        rttLookVec.normalize();

        //double deviation = (rttLookVec ^ camLookVec).length();

        double pitchUp = osg::RadiansToDegrees( acos( rttLookVec * camLookVec ));
        double maxAngle = osg::minimum(89.0, pitchUp + 0.5*vfov);
        double maxVisibleDistance = hae * tan( osg::DegreesToRadians(maxAngle) );

        // get the maximum RTT extent.
        double eMax = osg::minimum( maxVisibleDistance, horizonDistance );

        // calculation the warping factor (a work in progress.)
        double ratio = eMin/eMax;
        double wg1 = _warp = (1.0+ratio)/(1.0-ratio);
        double wg2 = _warp = (1.0-ratio)/(1.0+ratio);
        _warp = ratio;
        //_warp = deviation;
        _warp = pitchUp > 65.0 ? osg::clampBetween( wg2, 0.001, 0.9 ) : 0.0;
        //_warp = 1000.0/ratio;

        // accounts for the curvature of the earth, somewhat.. 
        // TODO: need a better approach here..
        eMax += hae*0.12;

        // adjust the projection matrix for the viewport's aspect ratio:
        double hf = camAR > 1.0 ? camAR : 1.0;
        double vf = camAR > 1.0 ? 1.0 : 1.0/camAR;
        double xfov = eMax*hf;
        double yfov = eMax*vf;
        _rttProjMatrix = osg::Matrix::ortho( -xfov, xfov, -yfov, yfov, 1.0, eyeLen );

#if 0
        OE_INFO << LC
            << std::fixed
            << ", hae=" << hae 
            << ", pitch=" << pitchUp
            << ", vfov=" << vfov 
            << ", horizon=" << horizonDistance 
            << ", mvd=" << maxVisibleDistance
            << ", maxangle= " << maxAngle
            << ", eMin=" << eMin 
            << ", eMax=" << eMax
            //<< ", dev=" << deviation 
            << ", ratio=" << ratio 
            << ", warp=" << _warp
            << ", wg2=" << wg2 << std::endl;
#endif

        // projector matrices are the same as for the RTT camera. Tim was right.
        _projectorViewMatrix = _rttViewMatrix;
        _projectorProjMatrix = _rttProjMatrix;
    }
}

void
OverlayDecorator::traverse( osg::NodeVisitor& nv )
{
    if ( _overlayGraph.valid() )
    {
        updateRTTCamera( nv );
    
        _rttCamera->accept( nv );

        _texGenNode->accept( nv );
    }

    _subgraphContainer->accept( nv );
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
