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
#include <osgEarth/NodeUtils>
#include <osgEarth/Registry>
#include <osgEarth/TextureCompositor>
#include <osg/Texture2D>
#include <osg/TexEnv>
#include <osg/BlendFunc>
#include <osg/ComputeBoundsVisitor>
#include <osgGA/EventVisitor>
#include <osgShadow/ConvexPolyhedron>
#include <osgUtil/LineSegmentIntersector>
#include <iomanip>
#include <stack>

#define LC "[OverlayDecorator] "

//#define OE_TEST if (_dumpRequested) OE_INFO << std::setprecision(9)
#define OE_TEST OE_NULL

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
        bool intersects(const osg::BoundingSphere& bs) const
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

        bool intersects(const osg::BoundingBox& box) const
        {
            for( Faces::const_iterator i = _faces.begin(); i != _faces.end(); ++i )
            {
                osg::Plane up = i->plane;
                up.makeUnitLength();

                if ( up.intersect(box) < 0 )
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
    struct CoarsePolytopeIntersector : public OverlayDecorator::InternalNodeVisitor
    {
        CoarsePolytopeIntersector(
            const MyConvexPolyhedron& polytope,
            osg::NodeVisitor*         proxyNV,
            osg::BoundingBox&         out_bbox) :

        OverlayDecorator::InternalNodeVisitor(osg::NodeVisitor::TRAVERSE_ACTIVE_CHILDREN),
            _bbox    (out_bbox),
            _original( polytope ),
            _proxyNV ( proxyNV ),
            _coarse  ( false )
        {
            _polytopeStack.push( polytope );
            _matrixStack.push( osg::Matrix::identity() );
        }

        // override from NodeVisitor to support LOD processing
        virtual float getDistanceToViewPoint(const osg::Vec3& pos, bool useLODScale) const
        {
            return _proxyNV->getDistanceToViewPoint(pos, useLODScale);
        }

        void apply( osg::Node& node )
        {
            const osg::BoundingSphere& bs = node.getBound();
            if ( _polytopeStack.top().intersects( bs ) )
            {
                traverse( node );
            }
        }

        void apply( osg::Geode& node )
        {
            const osg::BoundingSphere& bs = node.getBound();

            if ( _polytopeStack.top().intersects( bs ) )
            {
                if ( _coarse )
                {
                    osg::BoundingSphere bsphere = bs;

                    osg::BoundingSphere::vec_type xdash = bsphere._center;
                    xdash.x() += bsphere._radius;
                    xdash = xdash*_matrixStack.top();

                    osg::BoundingSphere::vec_type ydash = bsphere._center;
                    ydash.y() += bsphere._radius;
                    ydash = ydash*_matrixStack.top();

                    osg::BoundingSphere::vec_type zdash = bsphere._center;
                    zdash.z() += bsphere._radius;
                    zdash = zdash*_matrixStack.top();

                    bsphere._center = bsphere._center*_matrixStack.top();

                    xdash -= bsphere._center;
                    osg::BoundingSphere::value_type len_xdash = xdash.length();

                    ydash -= bsphere._center;
                    osg::BoundingSphere::value_type len_ydash = ydash.length();

                    zdash -= bsphere._center;
                    osg::BoundingSphere::value_type len_zdash = zdash.length();

                    bsphere._radius = len_xdash;
                    if (bsphere._radius<len_ydash) bsphere._radius = len_ydash;
                    if (bsphere._radius<len_zdash) bsphere._radius = len_zdash;

                    _bbox.expandBy(bsphere);
                }
                else
                {
                    for( unsigned i=0; i < node.getNumDrawables(); ++i )
                    {
                        applyDrawable( node.getDrawable(i) );
                    }
                }
            }
        }

        void applyDrawable( osg::Drawable* drawable )
        {
            const osg::BoundingBox& box = drawable->getBound();

            if ( _polytopeStack.top().intersects( box ) )
            {
                // apply an eplison to avoid a bbox with a zero dimension
                static float e = 0.001;

                osg::Vec3d b0 = osg::Vec3(box.xMin(), box.yMin(), box.zMin()) * _matrixStack.top();
                osg::Vec3d b1 = osg::Vec3(box.xMax(), box.yMax(), box.zMax()) * _matrixStack.top();
                  
                _bbox.expandBy( std::min(b0.x(),b1.x())-e, std::min(b0.y(),b1.y())-e, std::min(b0.z(),b1.z())-e );
                _bbox.expandBy( std::max(b0.x(),b1.x())+e, std::max(b0.y(),b1.y())+e, std::max(b0.z(),b1.z())+e );
            }
        }

        void apply( osg::Transform& transform )
        {
            osg::Matrixd matrix;
            if ( !_matrixStack.empty() ) matrix = _matrixStack.top();
            transform.computeLocalToWorldMatrix( matrix, this );

            _matrixStack.push( matrix );
            _polytopeStack.push( _original );
            _polytopeStack.top().transform( osg::Matrixd::inverse( matrix ), matrix );

            traverse(transform);

            _matrixStack.pop();
            _polytopeStack.pop();
        }

        osg::BoundingBox& _bbox;
        osg::NodeVisitor* _proxyNV;
        MyConvexPolyhedron _original;
        std::stack<MyConvexPolyhedron> _polytopeStack;
        std::stack<osg::Matrixd> _matrixStack;
        bool _coarse;
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
_textureUnit  ( 1 ),
_textureSize  ( 1024 ),
_useShaders   ( false ),
_useWarping   ( false ),
_warp         ( 1.0f ),
_visualizeWarp( false ),
_mipmapping   ( false ),
_rttBlending  ( true ),
_updatePending( false ),
_dumpRequested( false )
{
    // nop
}

void
OverlayDecorator::initializeForOverlayGraph()
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
    }
}


void
OverlayDecorator::initializePerViewData( PerViewData& pvd )
{
    if ( !_textureUnit.isSet() || !_overlayGraph.valid() )
        return;

    // create the projected texture:
    osg::Texture2D* projTexture = new osg::Texture2D();
    projTexture->setTextureSize( *_textureSize, *_textureSize );
    projTexture->setInternalFormat( GL_RGBA8 );
    projTexture->setSourceFormat( GL_RGBA );
    projTexture->setSourceType( GL_UNSIGNED_BYTE );
    projTexture->setFilter( osg::Texture::MIN_FILTER, _mipmapping? osg::Texture::LINEAR_MIPMAP_LINEAR: osg::Texture::LINEAR );
    projTexture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
    projTexture->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_BORDER );
    projTexture->setWrap( osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_BORDER );
    projTexture->setWrap( osg::Texture::WRAP_R, osg::Texture::CLAMP_TO_BORDER );
    projTexture->setBorderColor( osg::Vec4(0,0,0,0) );

    // set up the RTT camera:
    pvd._rttCamera = new osg::Camera();
    pvd._rttCamera->setClearColor( osg::Vec4f(0,0,0,0) );
    pvd._rttCamera->setClearStencil( 0 );
    pvd._rttCamera->setClearMask( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT );
    // this ref frame causes the RTT to inherit its viewpoint from above (in order to properly
    // process PagedLOD's etc. -- it doesn't affect the perspective of the RTT camera though)
    pvd._rttCamera->setReferenceFrame( osg::Camera::ABSOLUTE_RF_INHERIT_VIEWPOINT );
    pvd._rttCamera->setViewport( 0, 0, *_textureSize, *_textureSize );
    pvd._rttCamera->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );
    pvd._rttCamera->setRenderOrder( osg::Camera::PRE_RENDER );
    pvd._rttCamera->setRenderTargetImplementation( osg::Camera::FRAME_BUFFER_OBJECT );

    pvd._rttCamera->attach( osg::Camera::COLOR_BUFFER, projTexture, 0, 0, _mipmapping );

    // try a depth-packed buffer. failing that, try a normal one.. if the FBO doesn't support
    // that (which is doesn't on some GPUs like Intel), it will automatically fall back on 
    // a PBUFFER_RTT impl
    if ( Registry::instance()->getCapabilities().supportsDepthPackedStencilBuffer() )
        pvd._rttCamera->attach( osg::Camera::PACKED_DEPTH_STENCIL_BUFFER, GL_DEPTH_STENCIL_EXT );
    else
        pvd._rttCamera->attach( osg::Camera::STENCIL_BUFFER, GL_STENCIL_INDEX );

    osg::StateSet* rttStateSet = pvd._rttCamera->getOrCreateStateSet();

    rttStateSet->setMode( GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );

    if ( _rttBlending )
    {
        osg::BlendFunc* blendFunc = new osg::BlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
        //Sure it shouldn't be this? -gw
        //osg::BlendFunc* blendFunc = new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        rttStateSet->setAttributeAndModes(blendFunc, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    }
    else
    {
        rttStateSet->setMode(GL_BLEND, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
    }

    // attach the overlay graph to the RTT camera.
    if ( _overlayGraph.valid() && ( _overlayGraph->getNumParents() == 0 || _overlayGraph->getParent(0) != pvd._rttCamera.get() ))
    {
        if ( pvd._rttCamera->getNumChildren() > 0 )
            pvd._rttCamera->replaceChild( 0, _overlayGraph.get() );
        else
            pvd._rttCamera->addChild( _overlayGraph.get() );
    }

    // overlay geometry is rendered with no depth testing, and in the order it's found in the
    // scene graph... until further notice...
    rttStateSet->setMode(GL_DEPTH_TEST, 0);
    rttStateSet->setBinName( "TraversalOrderBin" );


    // assemble the subgraph stateset:
    pvd._subgraphStateSet = new osg::StateSet();

    // set up the subgraph to receive the projected texture:
    pvd._subgraphStateSet->setTextureMode( *_textureUnit, GL_TEXTURE_GEN_S, osg::StateAttribute::ON );
    pvd._subgraphStateSet->setTextureMode( *_textureUnit, GL_TEXTURE_GEN_T, osg::StateAttribute::ON );
    pvd._subgraphStateSet->setTextureMode( *_textureUnit, GL_TEXTURE_GEN_R, osg::StateAttribute::ON );
    pvd._subgraphStateSet->setTextureMode( *_textureUnit, GL_TEXTURE_GEN_Q, osg::StateAttribute::ON );
    pvd._subgraphStateSet->setTextureAttributeAndModes( *_textureUnit, projTexture, osg::StateAttribute::ON );
    
    // set up the shaders
    if ( _useShaders )
    {            
        initSubgraphShaders( pvd ); //._subgraphStateSet.get() );
        initRTTShaders( pvd ); //._rttCamera->getOrCreateStateSet() );
        //_warpUniform = this->getOrCreateStateSet()->getOrCreateUniform( "warp", osg::Uniform::FLOAT );
        //_warpUniform->set( 1.0f );
    }
}

void
OverlayDecorator::initRTTShaders( PerViewData& pvd )
{
    osg::StateSet* set = pvd._rttCamera->getOrCreateStateSet();

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

    std::string vertSource;
    vertSource = buf.str();
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
    
    std::string fragSource;
    fragSource = fragBuf.str();
    
    program->addShader( new osg::Shader( osg::Shader::FRAGMENT, fragSource ) );
    set->addUniform(new osg::Uniform("texture_0",0));
}

void
OverlayDecorator::initSubgraphShaders( PerViewData& pvd )
{
    osg::StateSet* set = pvd._subgraphStateSet.get();

    VirtualProgram* vp = new VirtualProgram();
    vp->setName( "OverlayDecorator subgraph shader" );
    set->setAttributeAndModes( vp, osg::StateAttribute::ON );

    // sampler for projected texture:
    set->getOrCreateUniform( "osgearth_overlay_ProjTex", osg::Uniform::SAMPLER_2D )->set( *_textureUnit );

    // the texture projection matrix uniform.
    pvd._texGenUniform = set->getOrCreateUniform( "osgearth_overlay_TexGenMatrix", osg::Uniform::FLOAT_MAT4 );

    std::stringstream buf;

    // vertex shader - subgraph
    buf << "#version 110 \n"
        << "uniform mat4 osgearth_overlay_TexGenMatrix; \n"
        << "uniform mat4 osg_ViewMatrixInverse; \n"

        << "void osgearth_overlay_vertex(void) \n"
        << "{ \n"
        << "    gl_TexCoord["<< *_textureUnit << "] = osgearth_overlay_TexGenMatrix * osg_ViewMatrixInverse * gl_ModelViewMatrix * gl_Vertex; \n"
        << "} \n";

    std::string vertexSource;
    vertexSource = buf.str();
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

    std::string fragmentSource;
    fragmentSource = buf.str();
    vp->setFunction( "osgearth_overlay_fragment", fragmentSource, ShaderComp::LOCATION_FRAGMENT_PRE_LIGHTING );
}

void
OverlayDecorator::setOverlayGraph( osg::Node* node )
{
    if ( _overlayGraph.get() != node )
    {
        if ( _overlayGraph.valid() && node == 0L )
        {
            // un-register for traversals.
            if ( _updatePending )
            {
                _updatePending = false;
                ADJUST_EVENT_TRAV_COUNT( this, -1 );
            }

            ADJUST_EVENT_TRAV_COUNT( this, -1 );
        }
        else if ( !_overlayGraph.valid() && node != 0L )
        {
            // request that OSG give this node an event traversal.
            ADJUST_EVENT_TRAV_COUNT( this, 1 );
        }

        _overlayGraph = node;

        initializeForOverlayGraph();

        // go through and install the NEW overlay graph on any existing cameras.
        {
            Threading::ScopedWriteLock exclude( _perViewDataMutex );
            for( PerViewDataMap::iterator i = _perViewData.begin(); i != _perViewData.end(); ++i )
            {
                PerViewData& pvd = i->second;
                if ( pvd._rttCamera->getNumChildren() > 0 )
                    pvd._rttCamera->replaceChild( 0, _overlayGraph.get() );
                else
                    pvd._rttCamera->addChild( _overlayGraph.get() );
            }
        }

        //reinit();
    }
}

void
OverlayDecorator::setTextureSize( int texSize )
{
    if ( texSize != _textureSize.value() )
    {
        _textureSize = texSize;
        //reinit();
    }
}

void
OverlayDecorator::setTextureUnit( int texUnit )
{
    if ( !_explicitTextureUnit.isSet() || texUnit != _explicitTextureUnit.value() )
    {
        _explicitTextureUnit = texUnit;
        //reinit();
    }
}

void
OverlayDecorator::setMipMapping( bool value )
{
    if ( value != _mipmapping )
    {
        _mipmapping = value;
        //reinit();

        if ( _mipmapping )
            OE_INFO << LC << "Overlay mipmapping " << (value?"enabled":"disabled") << std::endl;
    }
}

void
OverlayDecorator::setVertexWarping( bool value )
{
    if ( value != _useWarping )
    {
        _useWarping = value;
        //reinit();
        
        if ( _useWarping )
            OE_INFO << LC << "Vertex warping " << (value?"enabled":"disabled")<< std::endl;
    }
}

void
OverlayDecorator::setOverlayBlending( bool value )
{
    if ( value != _rttBlending )
    {
        _rttBlending = value;
        //reinit();
        
        if ( _rttBlending )
            OE_INFO << LC << "Overlay blending " << (value?"enabled":"disabled")<< std::endl;
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
        unsigned maxSize = Registry::instance()->getCapabilities().getMaxFastTextureSize();
        _textureSize.init( osg::minimum( 4096u, maxSize ) );

        OE_INFO << LC << "Using texture size = " << *_textureSize << std::endl;
    }

    // rebuild dynamic elements.
    initializeForOverlayGraph();
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
OverlayDecorator::updateRTTCameras()
{
    static osg::Matrix normalizeMatrix = 
        osg::Matrix::translate(1.0,1.0,1.0) * osg::Matrix::scale(0.5,0.5,0.5);

    Threading::ScopedWriteLock exclusive( _perViewDataMutex );

    for( PerViewDataMap::iterator i = _perViewData.begin(); i != _perViewData.end(); ++i )
    {
        PerViewData& pvd = i->second;
    
        pvd._rttCamera->setViewMatrix( pvd._rttViewMatrix );
        pvd._rttCamera->setProjectionMatrix( pvd._rttProjMatrix );

        if ( pvd._texGenUniform.valid() )
        {
            osg::Matrix MVPT = pvd._rttViewMatrix * pvd._rttProjMatrix * normalizeMatrix;
            pvd._texGenUniform->set( MVPT );
            //if ( _useWarping )
            //    _warpUniform->set( _warp );
        }
    }
}

void
OverlayDecorator::cull( osgUtil::CullVisitor* cv, OverlayDecorator::PerViewData& pvd )
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

    OE_TEST << LC << "------- OD CULL ------------------------" << std::endl;

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

        OE_TEST << LC << "RTT distance to horizon: " << horizonDistanceInRTTPlane << std::endl;
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

        pvd._rttViewMatrix = osg::Matrixd::lookAt( eye, eye-worldUp*hasl, osg::Vec3(0,1,0) );
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

    // Save and reset the current near/far planes before traversing the subgraph.
    // We do this because we want a projection matrix that includes ONLY the clip
    // planes from the subgraph, and not anything traversed up to this point.
    double zSavedNear = cv->getCalculatedNearPlane();
    double zSavedFar  = cv->getCalculatedFarPlane();

    cv->setCalculatedNearPlane( FLT_MAX );
    cv->setCalculatedFarPlane( -FLT_MAX );

    // cull the subgraph (i.e. the terrain) here. This doubles as the subgraph's official 
    // cull traversal and a gathering of its clip planes.
    cv->pushStateSet( pvd._subgraphStateSet.get() );
    osg::Group::traverse( *cv );
    cv->popStateSet();

    // Pull a copy of the projection matrix; we will use this to calculate the optimum
    // projected texture extent
    osg::Matrixd projMatrix = *cv->getProjectionMatrix();

    // Clamp the projection matrix to the newly calculated clip planes. This prevents
    // any "leakage" from outside the subraph.
    double zNear = cv->getCalculatedNearPlane();
    double zFar  = cv->getCalculatedFarPlane();
    cv->clampProjectionMatrix( projMatrix, zNear, zFar );

    OE_TEST << LC << "Subgraph clamp: zNear = " << zNear << ", zFar = " << zFar << std::endl;

    if ( _isGeocentric )
    {
        // in geocentric mode, clamp the far clip plane to the horizon.
        double maxDistance = (1.0 - haslWeight)  * horizonDistance  + haslWeight * hasl;
        maxDistance *= 1.5;
        if (zFar - zNear >= maxDistance)
            zFar = zNear + maxDistance;

        cv->clampProjectionMatrix( projMatrix, zNear, zFar );

        OE_TEST << LC << "Horizon clamp: zNear = " << zNear << ", zFar = " << zFar << std::endl;
    }

    // restore the clip planes in the cull visitor, now that we have our subgraph
    // projection matrix.
    cv->setCalculatedNearPlane( osg::minimum(zSavedNear, zNear) );
    cv->setCalculatedFarPlane( osg::maximum(zSavedFar, zFar) );
       
    // contruct the polyhedron representing the viewing frustum.
    MyConvexPolyhedron frustumPH;
    frustumPH.setToUnitFrustum( true, true );
    osg::Matrixd MVP = *cv->getModelViewMatrix() * projMatrix;
    osg::Matrixd inverseMVP;
    inverseMVP.invert(MVP);
    frustumPH.transform( inverseMVP, MVP );

    // make a polyhedron representing the viewing frustum of the overlay, and cut it to
    // intersect the viewing frustum:
    osgShadow::ConvexPolyhedron visiblePH;

    // get the bounds of the overlay graph model. 
    osg::BoundingBox visibleOverlayBBox;
    CoarsePolytopeIntersector cpi( frustumPH, cv, visibleOverlayBBox );
    _overlayGraph->accept( cpi );

    // adjust the bounding box to account for "flat" geometry. The Bbox must have 
    // 3D volume or it won't intersect properly with the frustum PH.
    visibleOverlayBBox.expandBy( osg::BoundingSphere(visibleOverlayBBox.center(), 1.0) );
    visiblePH.setToBoundingBox( visibleOverlayBBox );

    osgShadow::ConvexPolyhedron visiblePHBeforeCut;
    if ( _dumpRequested )
        visiblePHBeforeCut = visiblePH;

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
        pvd._rttViewMatrix = osg::Matrixd::lookAt( bc, osg::Vec3d(0,0,0), osg::Vec3d(0,0,1) );
        pvd._rttProjMatrix = osg::Matrixd::ortho( -eMax, eMax, -eMax, eMax, -eyeLen, bc.length() );

        OE_TEST << LC 
            << "1/2 RTT ortho span: " << eMax << ", near=" << -eyeLen << ", far=" << bc.length() << std::endl;

        OE_TEST << LC
            << "eMax = " << eMax
            << ", bc = " << bc.x() << ", " << bc.y() << ", " << bc.z()
            << ", eye = " << eye.x() << ", " << eye.y() << ", " << eye.z()
            << ", eyeLen = " << eyeLen
            << std::endl;
    }
    else
    {
        // for a projected map, just point the RTT straight down at the camera position.
        // TODO: this could be optimized, probably.
        double new_eMax;
        getMinMaxExtentInSilhouette( from, osg::Vec3d(0,0,-1), verts, eMin, new_eMax );   
        eMax = std::min( eMax, new_eMax ); 
        pvd._rttProjMatrix = osg::Matrix::ortho( -eMax, eMax, -eMax, eMax, -eyeLen, eyeLen );
    }

    //OE_NOTICE << LC << "EMIN = " << eMin << ", EMAX = " << eMax << std::endl;

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

    if ( _dumpRequested )
    {
#if 0
        // RTT frustum:
        osgShadow::ConvexPolyhedron rttPH;
        rttPH.setToUnitFrustum( true, true );
        osg::Matrixd MVP = pvd._rttViewMatrix * pvd._rttProjMatrix;
        osg::Matrixd inverseMVP;
        inverseMVP.invert(MVP);
        rttPH.transform( inverseMVP, MVP );
        rttPH.dumpGeometry();
#endif

#if 1
        // camera frustum:
        MyConvexPolyhedron frustumPH;
        frustumPH.setToUnitFrustum( true, true );
        osg::Matrixd MVP = *cv->getModelViewMatrix() * projMatrix;
        osg::Matrixd inverseMVP;
        inverseMVP.invert(MVP);
        frustumPH.transform( inverseMVP, MVP );
        frustumPH.dumpGeometry();
#endif

#if 0
        // visible PH or overlay:
        visiblePHBeforeCut.dumpGeometry();
#endif

#if 0
        // visible overlay Polyherdron AFTER frustum intersection:
        visiblePH.dumpGeometry();
#endif

        _dump = osgDB::readNodeFile("convexpolyhedron.osg");
        _dumpRequested = false;
    }
}

OverlayDecorator::PerViewData&
OverlayDecorator::getPerViewData(osg::View* key)
{
    // first check for it:
    {
        Threading::ScopedReadLock shared( _perViewDataMutex );
        PerViewDataMap::iterator i = _perViewData.find(key);
        if ( i != _perViewData.end() )
        {
            if ( !i->second._rttCamera.valid() )
                initializePerViewData( i->second );

            return i->second;
        }
    }

    // then exclusive lock and make/check it:
    {
        Threading::ScopedWriteLock exclusive( _perViewDataMutex );

        // double check pattern:
        PerViewDataMap::iterator i = _perViewData.find(key);
        if ( i != _perViewData.end() )
            return i->second;

        PerViewData& pvd = _perViewData[key];
        initializePerViewData(pvd);

        return pvd;
    }    
}

void
OverlayDecorator::traverse( osg::NodeVisitor& nv )
{
    bool isCull = nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR;

    if ( _overlayGraph.valid() && _textureUnit.isSet() )
    {
        // in the CULL traversal, find the per-view data associated with the 
        // cull visitor's current camera view and work with that:
        if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
        {
            osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>( &nv );
            if ( cv->getCurrentCamera() )
            {
                PerViewData& pvd = getPerViewData( cv->getCurrentCamera()->getView() );
                cull( cv, pvd );
                pvd._rttCamera->accept( nv );
            }
        }

        else if ( nv.getVisitorType() == osg::NodeVisitor::EVENT_VISITOR )
        {
            // during the event traversal, check to see whether any of the camera
            // matrices have changed from the previous frame. If so, active the update
            // visitor to update the RTT camera.

            osgGA::EventVisitor* ev = static_cast<osgGA::EventVisitor*>(&nv);
            osg::View* view = ev->getActionAdapter()->asView();
            if ( view )
            {
                PerViewData& pvd = getPerViewData(view);

                // first, check whether we already have an update coming.
                if ( !_updatePending && checkNeedsUpdate(pvd) )
                {
                    // need it, so schedule it.
                    //ev->getActionAdapter()->requestRedraw(); // not needed since we are bumping the update trav
                    _updatePending = true;
                    ADJUST_UPDATE_TRAV_COUNT( this, 1 );
                }

                // send the event traversal down the overlay graph
                pvd._rttCamera->accept( nv );
            }

            // traverse the overlay decorators subgraph
            osg::Group::traverse( nv );
        }

        else if ( nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
        {
            // if we get an update traversal, recalculate the RTT camera parameters.
            // TODO: update this to support multiple views..?
            if ( _updatePending )
            {
                OE_DEBUG << LC << "Update RTT camera, frame = " << nv.getFrameStamp()->getFrameNumber() << std::endl;
                updateRTTCameras();
                _updatePending = false;
                ADJUST_UPDATE_TRAV_COUNT( this, -1 );
            }

            // skip the camera, go straight to the overlay subgraph.
            if ( _overlayGraph.valid() )
            {
                _overlayGraph->accept( nv );
            }

            osg::Group::traverse( nv );
        }    

        else
        {
            // Some other type of visitor (like an intersection). Skip the RTT camera and
            // traverse the overlay graph directly.
            if ( _overlayGraph.valid() )
            {
                _overlayGraph->accept( nv );
            }

            osg::Group::traverse( nv );
        }
    }
    else
    {
        //osgUtil::CullVisitor* cv = 0L;
        //if ( isCull )
        //    cv = dynamic_cast<osgUtil::CullVisitor*>( &nv );

        //if ( cv )
        //    cv->pushStateSet( _subgraphStateSet.get() );

        osg::Group::traverse( nv );

        //if ( cv )
        //    cv->popStateSet();
    }
}


bool
OverlayDecorator::checkNeedsUpdate( OverlayDecorator::PerViewData& pvd )
{
    return
        pvd._rttCamera->getViewMatrix()       != pvd._rttViewMatrix ||
        pvd._rttCamera->getProjectionMatrix() != pvd._rttProjMatrix;
}


//----------------------------------------------------------------------------

#if 0
namespace
{
    // don't delete this.
    // it's not used by osgEarth, but you can copy this code into a viewer app and
    // use it to visualize the various polyhedra created by the overlay decorator.
    // see the end of OverlayDecorator::cull for the dump types.
    struct PHDumper : public osgGA::GUIEventHandler {
        MapNode* _mapNode;
        osg::Group* _group;
        PHDumper(MapNode* mapNode) : _mapNode(mapNode) {
            _group = new osg::Group();
            _mapNode->addChild( _group );
        }
        bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa ) {
            if ( ea.getEventType() == ea.KEYDOWN && ea.getKey() == ea.KEY_R ) {
                _mapNode->getOverlayDecorator()->requestDump();
                aa.requestRedraw();
            }
            else if ( ea.getEventType() == ea.FRAME ) {
                osg::Node* dump = _mapNode->getOverlayDecorator()->getDump();
                if ( dump ) {
                    dump->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, 1 | osg::StateAttribute::OVERRIDE);
                    _group->removeChildren(0, _group->getNumChildren());
                    _group->addChild( dump );
                    aa.requestRedraw();
                }
            }
            return false;
        }
    };
}
#endif
