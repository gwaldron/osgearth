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
    _projTexture->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP );
    _projTexture->setWrap( osg::Texture::WRAP_T, osg::Texture::CLAMP );
    _projTexture->setWrap( osg::Texture::WRAP_R, osg::Texture::CLAMP );

    // set up the RTT camera:
    _rttCamera = new osg::Camera();
    _rttCamera->setClearColor( osg::Vec4f(0,0,0,0) );
    _rttCamera->setReferenceFrame( osg::Camera::ABSOLUTE_RF );
    _rttCamera->setViewport( 0, 0, *_textureSize, *_textureSize );
    _rttCamera->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );
    _rttCamera->setRenderOrder( osg::Camera::PRE_RENDER );
    _rttCamera->setRenderTargetImplementation( osg::Camera::FRAME_BUFFER_OBJECT );
    _rttCamera->attach( osg::Camera::COLOR_BUFFER, _projTexture.get() );
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
    osg::StateSet* set = new osg::StateSet();

    if ( _overlayGraph.valid() )
    {
        // set up the subgraph to receive the projected texture:
        set->setTextureMode( *_textureUnit, GL_TEXTURE_GEN_S, osg::StateAttribute::ON );
        set->setTextureMode( *_textureUnit, GL_TEXTURE_GEN_T, osg::StateAttribute::ON );
        set->setTextureMode( *_textureUnit, GL_TEXTURE_GEN_R, osg::StateAttribute::ON );
        set->setTextureMode( *_textureUnit, GL_TEXTURE_GEN_Q, osg::StateAttribute::ON );
        set->setTextureAttributeAndModes( *_textureUnit, _projTexture.get(), osg::StateAttribute::ON );

        // decalling:
        osg::TexEnv* env = new osg::TexEnv();
        env->setMode( osg::TexEnv::DECAL );
        set->setTextureAttributeAndModes( *_textureUnit, env, osg::StateAttribute::ON );
        
        // set up the shaders
        if ( _useShaders )
            initShaders( set );
    }

    _subgraphContainer->setStateSet( set );
}

void
OverlayDecorator::initShaders( osg::StateSet* set )
{
    VirtualProgram* vp = new VirtualProgram();
    set->setAttributeAndModes( vp, osg::StateAttribute::ON );

    // sampler for projected texture:
    set->getOrCreateUniform( "osgearth_overlay_ProjTex", osg::Uniform::SAMPLER_2D )->set( *_textureUnit );

    // the texture projection matrix uniform.
    _texGenUniform = set->getOrCreateUniform( "osgearth_overlay_TexGenMatrix", osg::Uniform::FLOAT_MAT4 );

    std::stringstream buf;

    // vertex shader
    buf << "#version 110 \n"
        << "uniform mat4 osgearth_overlay_TexGenMatrix; \n"
        << "uniform mat4 osg_ViewMatrixInverse; \n"

        << "void osgearth_overlay_vertex(void) \n"
        << "{ \n"
        << "    gl_TexCoord["<< *_textureUnit << "] = osgearth_overlay_TexGenMatrix * osg_ViewMatrixInverse * gl_ModelViewMatrix * gl_Vertex; \n"
        << "} \n";

    std::string vertexSource = buf.str();
    vp->setFunction( "osgearth_overlay_vertex", vertexSource, ShaderComp::LOCATION_VERTEX_POST_LIGHTING );

    buf.str("");
    buf << "#version 110 \n"
        << "uniform sampler2D osgearth_overlay_ProjTex; \n"

        << "void osgearth_overlay_fragment( inout vec4 color ) \n"
        << "{ \n"
        << "    vec4 texel = texture2DProj(osgearth_overlay_ProjTex, gl_TexCoord["<< *_textureUnit << "]); \n"
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
        _textureSize = osg::minimum( 4096, maxSize );

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
            _texGenUniform->set( MVPT );
    }

    else if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
    {
        osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>( &nv );
        if ( !cv ) return;

        osg::Vec3 eye = cv->getEyePoint();
        double eyeLen = eye.length();

        // point the RTT camera straight down from the eyepoint:
        _rttViewMatrix = osg::Matrixd::lookAt( eye, osg::Vec3(0,0,0), osg::Vec3(0,0,1) );

        // calculate the approximate distance from the eye to the horizon. This is out maximum
        // possible RTT extent:
        double hae = eyeLen - _earthRadiusMajor; // height above "max spheroid" (TODO: limit hae to a minimum value)
        double haeAdj = hae*1.5;    // wiggle room, since the ellipsoid is different from the spheroid.
        double eMax = sqrt( haeAdj*haeAdj + 2.0 * _earthRadiusMajor * haeAdj ); // distance to the horizon

        // calculate the approximate extent viewed from the camera if it's pointing
        // at the ground. This is the minimum acceptable RTT extent.
        double vfov, camAR, znear, zfar;
        cv->getProjectionMatrix()->getPerspective( vfov, camAR, znear, zfar );
        double eMin = haeAdj * tan( osg::DegreesToRadians(0.5*vfov) );

        // calculate the deviation between the RTT camera's look-vector and the main camera's
        // look-vector (cross product). This gives us a [0..1] multiplier that will vary the
        // RTT extent as the camera's pitch varies from [-90..0].
        osg::Vec3 from, to, up;
        
        const osg::Matrix& mvMatrix = *cv->getModelViewMatrix();
        mvMatrix.getLookAt( from, to, up, eyeLen);
        osg::Vec3 camLookVec = to-from;
        camLookVec.normalize();

        _rttViewMatrix.getLookAt(from,to,up,eyeLen);
        osg::Vec3 rttLookVec = to-from;
        rttLookVec.normalize();

        double deviation = (rttLookVec ^ camLookVec).length();
        double t = deviation; // (deviation*deviation); // interpolation factor
        double eIdeal = eMin + t * (eMax-eMin); 

        //OE_INFO << "dev=" << deviation << ", ext=" << eIdeal << std::endl;

        // adjust the projection matrix for the viewport's aspect ratio:
        double hf = camAR > 1.0 ? camAR : 1.0;
        double vf = camAR > 1.0 ? 1.0 : 1.0/camAR;
        _rttProjMatrix = osg::Matrix::ortho( -eIdeal*hf, eIdeal*hf, -eIdeal*vf, eIdeal*vf, 1.0, eyeLen );

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
