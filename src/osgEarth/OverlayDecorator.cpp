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
#include <osg/BlendFunc>
#include <osg/ComputeBoundsVisitor>
#include <osgShadow/ConvexPolyhedron>
#include <osgUtil/LineSegmentIntersector>
#include <iomanip>
#include <stack>

#define LC "[OverlayDecorator] "

using namespace osgEarth;

//---------------------------------------------------------------------------

namespace
{
    /**
     * Extends ConvexPolyhedron to add bounds tests.
     */
    class MyConvexPolyhedron : public osgShadow::ConvexPolyhedron
    {
    public:       
        bool
        contains(const osg::BoundingSphere& bs) const
        {
            for( Faces::const_iterator i = _faces.begin(); i != _faces.end(); ++i )
            {
                osg::Plane up = i->plane;
                up.makeUnitLength();
                if ( up.distance( bs.center() ) < -bs.radius() )
                    return false;
            }
            return true;
        }
    };

    /**
     * Visits a scene graph (in our case, the overlay graph) and calculates a
     * geometry bounding box that intersects the provided polytope (which in out case is the
     * view frustum).
     *
     * It's called "Coarse" because it does not traverse to the Drawable level, just to
     * the Geode bounding sphere level.
     */
    struct CoarsePolytopeIntersector : public osg::NodeVisitor
    {
        CoarsePolytopeIntersector(const MyConvexPolyhedron& polytope, osg::BoundingBox& out_bbox)
            : osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ),
              _bbox(out_bbox),
              _original( polytope )
        {
            _polytopeStack.push( polytope );
            _matrixStack.push( osg::Matrix::identity() );
        }

        void apply( osg::Node& node )
        {
            const osg::BoundingSphere& bs = node.getBound();
            if ( _polytopeStack.top().contains( bs ) )
            {
                traverse( node );
            }
        }

        void apply( osg::Geode& node )
        {
            const osg::BoundingSphere& bs = node.getBound();

            if ( _polytopeStack.top().contains( bs ) )
            {
                _bbox.expandBy(
                    osg::BoundingSphere( bs.center() * _matrixStack.top(), bs.radius() ) );

                //for( int i=0; i < node.getNumDrawables(); ++i )
                //{
                //    applyDrawable( node.getDrawable(i) );
                //}
            }
        }

        void apply( osg::Transform& transform )
        {
            osg::Matrixd matrix;
            transform.computeLocalToWorldMatrix( matrix, this );

            _matrixStack.push( matrix );
            _polytopeStack.push( _original );
            _polytopeStack.top().transform( osg::Matrixd::inverse( matrix ), matrix );

            traverse(transform);

            _matrixStack.pop();
            _polytopeStack.pop();
        }

        osg::BoundingBox& _bbox;
        MyConvexPolyhedron _original;
        std::stack<MyConvexPolyhedron> _polytopeStack;
        std::stack<osg::Matrixd> _matrixStack;
    };

    /**
     * This method takes a set of verts and finds the nearest and farthest distances from
     * the points to the camera. It does this calculation in the plane defined by the
     * look vector.
     *
     * IOW, all the test points are "projected" on to the plane defined by the camera point
     * and the look (normal) vector, and then the distances from the camera point to each
     * projected point are tested in order to find the min/max extent.
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
    
    /**
     * Same as the method above, but extracts the verts from a bounding box.
     */
    void
    getMinMaxExtentInSilhouette(const osg::Vec3d& cam, const osg::Vec3d& look, 
                                const osg::BoundingBox& bbox,
                                double& out_eMin, double& out_eMax )
    {
        std::vector<osg::Vec3d> verts(8);
        verts[0].set( bbox.xMin(), bbox.yMin(), bbox.zMin() );
        verts[1].set( bbox.xMin(), bbox.yMin(), bbox.zMax() );
        verts[2].set( bbox.xMin(), bbox.yMax(), bbox.zMin() );
        verts[3].set( bbox.xMin(), bbox.yMax(), bbox.zMax() );
        verts[4].set( bbox.xMax(), bbox.yMin(), bbox.zMin() );
        verts[5].set( bbox.xMax(), bbox.yMin(), bbox.zMax() );
        verts[6].set( bbox.xMax(), bbox.yMax(), bbox.zMin() );
        verts[7].set( bbox.xMax(), bbox.yMax(), bbox.zMax() );
        getMinMaxExtentInSilhouette( cam, look, verts, out_eMin, out_eMax );
    }
}

//---------------------------------------------------------------------------

OverlayDecorator::OverlayDecorator() :
_textureUnit( 1 ),
_textureSize( 1024 ),
_useShaders( false ),
_useWarping( false ),
_warp( 1.0f ),
_visualizeWarp( false ),
_mipmapping( true )
{
    // nop
}

void
OverlayDecorator::reinit()
{
    if ( !_engine.valid() ) return;

    if ( _overlayGraph.valid() )
    {
        // apply the user-request texture unit, if applicable:
        if ( _explicitTextureUnit.isSet() )
        {
            if ( !_textureUnit.isSet() || *_textureUnit != *_explicitTextureUnit )
            {
                _textureUnit = *_explicitTextureUnit;
            }
        }

        // otherwise, automatically allocate a texture unit if necessary:
        else if ( !_textureUnit.isSet() && _useShaders )
        {
            int texUnit;
            if ( _engine->getTextureCompositor()->reserveTextureImageUnit( texUnit ) )
            {
                _textureUnit = texUnit;
                OE_INFO << LC << "Reserved texture image unit " << *_textureUnit << std::endl;
            }
            else
            {
                OE_WARN << LC << "Uh oh, no texture image units available." << std::endl;
            }
        }

        if ( _textureUnit.isSet() )
        {
            _projTexture = new osg::Texture2D();
            _projTexture->setTextureSize( *_textureSize, *_textureSize );
            _projTexture->setInternalFormat( GL_RGBA8 );
            _projTexture->setSourceFormat( GL_RGBA );
            _projTexture->setSourceType( GL_UNSIGNED_BYTE );
            _projTexture->setFilter( osg::Texture::MIN_FILTER, _mipmapping? osg::Texture::LINEAR_MIPMAP_LINEAR : osg::Texture::LINEAR );
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
            _rttCamera->attach( osg::Camera::COLOR_BUFFER, _projTexture.get(), 0, 0, _mipmapping );
            _rttCamera->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );

            // force disable GL_BLEND, because otherwise the RTT geom will blend with the transparent clear color
            // of the RTT camera and produce low alpha values. TODO: look into another solution for this, since 
            // doing this turns off all blending on the subgraph.
            _rttCamera->getOrCreateStateSet()->setMode( GL_BLEND, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED );

            // texture coordinate generator:
            _texGenNode = new osg::TexGenNode();
            _texGenNode->setTextureUnit( *_textureUnit );
            _texGenNode->getTexGen()->setMode( osg::TexGen::EYE_LINEAR );
            
            // attach the overlay graph to the RTT camera.
            if ( _overlayGraph.valid() && ( _overlayGraph->getNumParents() == 0 || _overlayGraph->getParent(0) != _rttCamera.get() ))
            {
                if ( _rttCamera->getNumChildren() > 0 )
                    _rttCamera->replaceChild( 0, _overlayGraph.get() );
                else
                    _rttCamera->addChild( _overlayGraph.get() );
            }
        }
    }

    // assemble the subgraph stateset:
    _subgraphStateSet = new osg::StateSet();

    if ( _overlayGraph.valid() && _textureUnit.isSet() )
    {
        // set up the subgraph to receive the projected texture:
        _subgraphStateSet->setTextureMode( *_textureUnit, GL_TEXTURE_GEN_S, osg::StateAttribute::ON );
        _subgraphStateSet->setTextureMode( *_textureUnit, GL_TEXTURE_GEN_T, osg::StateAttribute::ON );
        _subgraphStateSet->setTextureMode( *_textureUnit, GL_TEXTURE_GEN_R, osg::StateAttribute::ON );
        _subgraphStateSet->setTextureMode( *_textureUnit, GL_TEXTURE_GEN_Q, osg::StateAttribute::ON );
        _subgraphStateSet->setTextureAttributeAndModes( *_textureUnit, _projTexture.get(), osg::StateAttribute::ON );

        // decalling (note: this has no effect when using shaders.. remove? -gw)
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
    buf << "#version 110 \n";

    if ( _useWarping )
    {
        buf << "uniform float warp; \n"

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
            << "    gl_FrontColor = gl_Color; \n"
            << "    gl_TexCoord[0] = gl_MultiTexCoord0;\n"
            << "} \n";
    }

    else // no vertex warping
    {
        buf << "void main() \n"
            << "{ \n"
            << "    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex; \n"
            << "    gl_FrontColor = gl_Color; \n"
            << "    gl_TexCoord[0] = gl_MultiTexCoord0;\n"
            << "} \n";
    }

    std::string vertSource = buf.str();
    program->addShader( new osg::Shader( osg::Shader::VERTEX, vertSource ) );

    std::stringstream fragBuf;
    fragBuf    << "#version 110 \n"
               << "uniform sampler2D texture_0; \n"
               << "void main() \n"
               << "{\n"                              
               << "    vec4 tex = texture2D(texture_0, gl_TexCoord[0].xy);\n"
               << "    vec3 mixed_color = mix(gl_Color.rgb, tex.rgb, tex.a);\n"
               << "    gl_FragColor = vec4(mixed_color, gl_Color.a); \n"
               << "}\n";
    
    std::string fragSource = fragBuf.str();
    
    program->addShader( new osg::Shader( osg::Shader::FRAGMENT, fragSource ) );
    set->addUniform(new osg::Uniform("texture_0",0));
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
        << "uniform sampler2D osgearth_overlay_ProjTex; \n";

    if ( _useWarping )
    {
        buf << "uniform float warp; \n"

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
            << "} \n";
    }

    buf << "void osgearth_overlay_fragment( inout vec4 color ) \n"
        << "{ \n"
        << "    vec2 texCoord = gl_TexCoord["<< *_textureUnit << "].xy / gl_TexCoord["<< *_textureUnit << "].q; \n";

    if ( _useWarping && !_visualizeWarp )
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
        if ( _overlayGraph.valid() && node == 0L )
        {
            ADJUST_UPDATE_TRAV_COUNT( this, -1 );
        }
        else if ( !_overlayGraph.valid() && node != 0L )
        {
            ADJUST_UPDATE_TRAV_COUNT( this, 1 );
        }

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
    if ( !_explicitTextureUnit.isSet() || texUnit != _explicitTextureUnit.value() )
    {
        _explicitTextureUnit = texUnit;
        reinit();
    }
}

void
OverlayDecorator::setMipmapping( bool value )
{
    if ( value != _mipmapping )
    {
        _mipmapping = value;
        reinit();
    }
}

void
OverlayDecorator::setVertexWarping( bool value )
{
    if ( value != _useWarping )
    {
        _useWarping = value;
        reinit();
    }
}

void
OverlayDecorator::onInstall( TerrainEngineNode* engine )
{
    _engine = engine;

    // establish the earth's major axis:
    MapInfo info(engine->getMap());
    _isGeocentric = info.isGeocentric();
    _ellipsoid = info.getProfile()->getSRS()->getEllipsoid();

    // the maximum extent (for projected maps only)
    if ( !_isGeocentric )
    {
        const GeoExtent& extent = info.getProfile()->getExtent();
        _maxProjectedMapExtent = osg::maximum( extent.width(), extent.height() );
    }

    // see whether we want shader support:
    // TODO: this is not stricty correct; you might still want to use shader overlays
    // in multipass mode, AND you might want FFP overlays in multitexture-FFP mode.
    _useShaders = engine->getTextureCompositor()->usesShaderComposition();

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
    if ( !_explicitTextureUnit.isSet() && _textureUnit.isSet() )
    {
        _engine->getTextureCompositor()->releaseTextureImageUnit( *_textureUnit );
        _textureUnit.unset();
    }

    _engine = 0L;
}

void
OverlayDecorator::updateRTTCamera( osg::NodeVisitor& nv )
{
    static osg::Matrix normalizeMatrix = 
        osg::Matrix::translate(1.0,1.0,1.0) * osg::Matrix::scale(0.5,0.5,0.5);

    // configure the RTT camera:
    _rttCamera->setViewMatrix( _rttViewMatrix );
    _rttCamera->setProjectionMatrix( _rttProjMatrix );

    // configure the Projector camera:
    //osg::Matrix MVP = _projectorViewMatrix * _projectorProjMatrix;
    //osg::Matrix MVPT = MVP * normalizeMatrix; //osg::Matrix::translate(1.0,1.0,1.0) * osg::Matrix::scale(0.5,0.5,0.5);

    osg::Matrix MVPT = _projectorViewMatrix * _projectorProjMatrix * normalizeMatrix;

    //_texGenNode->getTexGen()->setMode( osg::TexGen::EYE_LINEAR ); // moved to initialization
    _texGenNode->getTexGen()->setPlanesFromMatrix( MVPT );
    
    // uniform update:
    if ( _useShaders )
    {
        _texGenUniform->set( MVPT );
        if ( _useWarping )
            _warpUniform->set( _warp );
    }
}

void
OverlayDecorator::cull( osgUtil::CullVisitor* cv )
{
    static int s_frame = 1;

    osg::Vec3 eye = cv->getEyePoint();

    double eyeLen;
    osg::Vec3d worldUp;

    // height above sea level
    double hasl;

    // weight of the HASL value when calculating extent compensation
    double haslWeight;

    // approximate distance to the visible horizon
    double horizonDistance; 

    // distance to the horizon, projected into the RTT camera's tangent plane.
    double horizonDistanceInRTTPlane;

    if ( _isGeocentric )
    {
        double lat, lon;
        _ellipsoid->convertXYZToLatLongHeight( eye.x(), eye.y(), eye.z(), lat, lon, hasl );
        hasl = osg::maximum( hasl, 100.0 );

        worldUp = _ellipsoid->computeLocalUpVector(eye.x(), eye.y(), eye.z());

        eyeLen = eye.length();

        // radius of the earth under the eyepoint
        double radius = eyeLen - hasl; 
        horizonDistance = sqrt( 2.0 * radius * hasl ); 
    
        // calculate the distance to the horizon, projected into the RTT camera plane.
        // This is the maximum limit of eMax since there is no point in drawing overlay
        // data beyond the visible horizon.
        double pitchAngleOfHorizon_rad = acos( horizonDistance/eyeLen );
        horizonDistanceInRTTPlane = horizonDistance * sin( pitchAngleOfHorizon_rad );
    }
    else // projected map
    {
        hasl = eye.z();
        hasl = osg::maximum( hasl, 100.0 );
        worldUp.set( 0.0, 0.0, 1.0 );
        eyeLen = hasl * 2.0;

        // there is no maximum horizon distance in a projected map
        horizonDistance = DBL_MAX;
        horizonDistanceInRTTPlane = DBL_MAX;

        _rttViewMatrix = osg::Matrixd::lookAt( eye, eye-worldUp*hasl, osg::Vec3(0,1,0) );
    }

    // create a "weighting" that weights HASL against the camera's pitch.
    osg::Vec3d lookVector = cv->getLookVectorLocal();
    haslWeight = osg::absolute(worldUp * lookVector);

    // unit look-vector of the eye:
    osg::Vec3d from, to, up;
    const osg::Matrix& mvMatrix = *cv->getModelViewMatrix();
    mvMatrix.getLookAt( from, to, up, eyeLen);
    osg::Vec3 camLookVec = to-from;
    camLookVec.normalize();

    // unit look-vector of the RTT camera:
    osg::Vec3d rttLookVec = -worldUp;

    // the minimum and maximum extents of the overlay ortho projector:
    double eMin = 0.1;
    double eMax = DBL_MAX;

    // cull the subgraph here. This doubles as the subgraph's official cull traversal
    // and a gathering of its clip planes.
    cv->pushStateSet( _subgraphStateSet.get() );
    osg::Group::traverse( *cv );
    cv->popStateSet();

#if 0
    // --- FIRST PASS ------------------------
    // First, intersect the view frustum with the overlay geometry. This will provide
    // a maximum required extent for our ortho RTT camera. Depending on the layout of
    // the geometry in the overlay graph, this may or may not be optimal ... we will
    // work to refine it in later passes if necessary.

    cv->computeNearPlane();
    double znear = cv->getCalculatedNearPlane();
    double zfar  = cv->getCalculatedFarPlane();
    osg::Matrixd projMatrix = *cv->getProjectionMatrix();
    cv->clampProjectionMatrixImplementation( projMatrix, znear, zfar );

    // collect the bounds of overlay geometry that intersects the view frustum.
    MyConvexPolyhedron viewPT;
    viewPT.setToUnitFrustum( true, true );
    osg::Matrixd viewMVP = (*cv->getModelViewMatrix()) * projMatrix;
    viewPT.transform( osg::Matrix::inverse(viewMVP), viewMVP );

    osg::BoundingBox viewbbox;
    CoarsePolytopeIntersector cpi( viewPT, viewbbox );
    _overlayGraph->accept( cpi );

    //TODO: sometimes this viewbbox goes invalid even though there's clearly goemetry
    //      in view. Happens when you zoom in really close. Need to investigate -gw
    //OE_INFO << LC << "OV radius = " << viewbbox.radius() << std::endl;
    if ( viewbbox.valid() )
    {
        getMinMaxExtentInSilhouette( from, rttLookVec, viewbbox, eMin, eMax );
        eMax = osg::minimum( eMax, horizonDistanceInRTTPlane ); 
    }

    if ( !_isGeocentric )
        eyeLen = zfar;
#endif

    // --- SECOND PASS --------------------
    // Remake the projection matrix with better hueristic for the far clipping plane.
    // (Jason's method)
    osg::Matrixd projMatrix = *cv->getProjectionMatrix();
    double fovy, aspectRatio, zfar, znear;
    cv->getProjectionMatrix()->getPerspective( fovy, aspectRatio, znear, zfar );
    double maxDistance = (1.0 - haslWeight)  * horizonDistance  + haslWeight * hasl;
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

    // get the bounds of the model. 
    //const osg::BoundingSphere& bs = osg::Group::getBound();
    osg::ComputeBoundsVisitor cbbv(osg::NodeVisitor::TRAVERSE_ACTIVE_CHILDREN);
    this->accept(cbbv);
    visiblePH.setToBoundingBox(cbbv.getBoundingBox());

    // this intersects the viewing frustum with the subgraph's bounding box, basically giving us
    // a "minimal" polyhedron containing all potentially visible geometry. (It can't be truly 
    // minimal without clipping at the geometry level, but that would probably be too expensive.)
    visiblePH.cut( frustumPH );

    // calculate the extents for our orthographic RTT camera (clamping it to the
    // visible horizon)
    std::vector<osg::Vec3d> verts;
    visiblePH.getPoints( verts );

    if ( _isGeocentric )
    {
        // for a geocentric map, try to place the RTT camera position at an optimal point
        // that will minimize the span of the RTT texture. Take the centroid of the 
        // visible polyhedron and clamp it's distance to the eyepoint by half the horizon
        // distance.
        osg::BoundingBox box = visiblePH.computeBoundingBox();
        osg::Vec3d bc = box.center();
        osg::Vec3d eye2bc = eye - bc;
        if ( eye2bc.length() > horizonDistance )
        {
            eye2bc.normalize();
            bc = eye + eye2bc * 0.5*horizonDistance;
        }
        
        rttLookVec = -bc;
        rttLookVec.normalize();

        double new_eMax;
        getMinMaxExtentInSilhouette( bc, rttLookVec, verts, eMin, new_eMax );
        eMax = std::min( eMax, new_eMax );
        _rttViewMatrix = osg::Matrixd::lookAt( bc, osg::Vec3d(0,0,0), osg::Vec3d(0,0,1) );
        _rttProjMatrix = osg::Matrixd::ortho( -eMax, eMax, -eMax, eMax, -eyeLen, bc.length() );

        //OE_INFO << std::fixed << std::setprecision(1)
        //    << "eMax = " << eMax
        //    << ", bc = " << bc.x() << ", " << bc.y() << ", " << bc.z()
        //    << ", eye = " << eye.x() << ", " << eye.y() << ", " << eye.z()
        //    << ", eyeLen = " << eyeLen
        //    << std::endl;
    }
    else
    {
        // for a projected map, just point the RTT straight down at the camera position.
        // TODO: this could be optimized, probably.
        double new_eMax;
        getMinMaxExtentInSilhouette( from, osg::Vec3d(0,0,-1), verts, eMin, new_eMax );   
        eMax = std::min( eMax, new_eMax ); 
        _rttProjMatrix = osg::Matrix::ortho( -eMax, eMax, -eMax, eMax, -eyeLen, eyeLen );
    }

    if ( _useWarping )
    {
        // calculate the warping paramaters. This uses shaders to warp the verts and
        // tex coords to favor data closer to the camera when necessary.

    #define WARP_LIMIT 3.0

        double pitchStrength = ( camLookVec * rttLookVec ); // eye pitch relative to rtt pitch
        double devStrength = 1.0 - (pitchStrength*pitchStrength);
        double haslStrength = 1.0 - osg::clampBetween( hasl/1e6, 0.0, 1.0 );

        _warp = 1.0 + devStrength * haslStrength * WARP_LIMIT;

        if ( _visualizeWarp )
            _warp = 4.0;

#if 0
        OE_INFO << LC << std::fixed
            << "hasl=" << hasl
            << ", eMin=" << eMin
            << ", eMax=" << eMax
            << ", eyeLen=" << eyeLen
            //<< ", ratio=" << ratio
            //<< ", dev=" << devStrength
            //<< ", has=" << haeStrength
            << ", warp=" << _warp
            << std::endl;
#endif
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
    bool isCull = nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR;

    if ( _overlayGraph.valid() && _textureUnit.isSet() )
    {
        if ( isCull )
        {
            osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>( &nv );
            if ( cv )
            {
                cull( cv );
            }
            _rttCamera->accept( nv );
            
            // note: texgennode doesn't need a cull, and the subgraph
            // is traversed in cull().
        }

        else
        {
            if ( nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
            {
                updateRTTCamera( nv );
            }
            _rttCamera->accept( nv );
            _texGenNode->accept( nv );

            //_subgraphContainer->accept( nv );
            osg::Group::traverse( nv );
        }    
    }
    else
    {
        osgUtil::CullVisitor* cv = 0L;
        if ( isCull )
            cv = dynamic_cast<osgUtil::CullVisitor*>( &nv );

        if ( cv )
            cv->pushStateSet( _subgraphStateSet.get() );

        osg::Group::traverse( nv );

        if ( cv )
            cv->popStateSet();
        //_subgraphContainer->accept( nv );
    }
}


#if 0
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
#endif
