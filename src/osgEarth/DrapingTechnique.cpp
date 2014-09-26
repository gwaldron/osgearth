/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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
#include <osgEarth/DrapingTechnique>
#include <osgEarth/Capabilities>
#include <osgEarth/Registry>
#include <osgEarth/VirtualProgram>

#include <osg/BlendFunc>
#include <osg/TexGen>
#include <osg/Texture2D>
#include <osg/Uniform>

#define LC "[DrapingTechnique] "

//#define OE_TEST if (_dumpRequested) OE_INFO << std::setprecision(9)
#define OE_TEST OE_NULL

using namespace osgEarth;

//---------------------------------------------------------------------------

namespace
{
    // Additional per-view data stored by the draping technique.
    struct LocalPerViewData : public osg::Referenced
    {
        osg::ref_ptr<osg::Uniform> _texGenUniform;
    };
}

namespace
{
    struct Line2d
    {
        bool intersectRaysXY(
            const osg::Vec3d& p0, const osg::Vec3d& d0,
            const osg::Vec3d& p1, const osg::Vec3d& d1,
            osg::Vec3d& out_p,
            double&     out_u,
            double&     out_v) const
        {
            static const double epsilon = 0.001;

            double det = d0.y()*d1.x() - d0.x()*d1.y();
            if ( osg::equivalent(det, 0.0, epsilon) )
                return false; // parallel

            out_u = (d1.x()*(p1.y()-p0.y())+d1.y()*(p0.x()-p1.x()))/det;
            out_v = (d0.x()*(p1.y()-p0.y())+d0.y()*(p0.x()-p1.x()))/det;
            out_p = p0 + d0*out_u;
            return true;
        }

        osg::Vec3d _a, _b;

        Line2d(const osg::Vec3d& p0, const osg::Vec3d& p1) : _a(p0), _b(p1) { }

        Line2d(const osg::Vec4d& p0, const osg::Vec4d& p1)
            : _a(p0.x()/p0.w(), p0.y()/p0.w(), p0.x()/p0.w()), _b(p1.x()/p1.w(), p1.y()/p1.w(), p1.z()/p1.w()) { }

        bool intersect(const Line2d& rhs, osg::Vec4d& out) const {
            double u, v;
            osg::Vec3d temp;
            bool ok = intersectRaysXY(_a, (_b-_a), rhs._a, (rhs._b-rhs._a), temp, u, v);
            out.set( temp.x(), temp.y(), temp.z(), 1.0 );
            return ok;
        }
        bool intersect(const Line2d& rhs, osg::Vec3d& out) const {
            double u, v;
            return intersectRaysXY(_a, (_b-_a), rhs._a, (rhs._b-rhs._a), out, u, v);
        }
    };

    // Experimental.
    void optimizeProjectionMatrix(OverlayDecorator::TechRTTParams& params, double maxFarNearRatio)
    {
        LocalPerViewData& local = *static_cast<LocalPerViewData*>(params._techniqueData.get());

        //TODO: add this to the local
        //local._rttLimitZ->set( 0.0f );

        // t0,t1,t2,t3 will form a polygon that tightly fits the
        // main camera's frustum. Texture near the camera will get
        // more resolution then texture far away.
        //
        // The line segment t0t1 represents the near clip plane and is
        // along the y=-1 line of the clip volume. t2t3 represents the
        // far plane and lies long y=+1. This code calculates the optimal
        // width of those 2 line segments off-center. Since the view
        // frustum is symmertical (as calculated in OverlayDecorator)
        // we only need find the half-width of each.
        //
        // NOTE: this algorithm only works with the top-down RTT camera 
        // created by the OverlayDecorator, AND assumes a "level" view
        // camera (no roll) with respect to the RTT camera.
        osg::Vec4d t0, t1, t2, t3;
        {
            // cap the width of the far line w.r.t to y-axis of the clip
            // space. (derived empirically)
            const double maxApsectRatio   = 1.0; 

            // this matrix xforms the verts from model to clip space.
            osg::Matrix rttMVP = params._rttViewMatrix * params._rttProjMatrix;

            // if the eyepoint lies within the RTT clip space, don't bother to
            // optimize because the camera is looking downish and the existing
            // rectangular volume is sufficient.
            osg::Vec3d eyeClip = params._eyeWorld * rttMVP;
            if ( eyeClip.y() >= -1.0 && eyeClip.y() <= 1.0 )
                return;

            // sanity check. 6 faces requires since we need near and far
            if ( params._visibleFrustumPH._faces.size() < 6 )
                return;

            // discover the max near-plane width.
            double halfWidthNear = 0.0;
            osgShadow::ConvexPolyhedron::Faces::iterator f = params._visibleFrustumPH._faces.begin();
            f++; f++; f++; f++; // the near plane Face
            // f->vertices.size() should always be 4, I would think.. but it's not..
            for(unsigned i=0; i<f->vertices.size(); ++i)
            {
                osg::Vec3d p = f->vertices[i] * rttMVP;
                if ( fabs(p.x()) > halfWidthNear )
                    halfWidthNear = fabs(p.x());
            }

            double aspectRatio  = DBL_MAX;
            double farNearRatio = DBL_MAX;
            double halfWidthFar = DBL_MAX;

            // Next, find the point in the camera frustum that forms the largest angle
            // with the center line (line of sight). This is simply the minimum dot
            // product of LOS vector and the vector from (0,-1,0) to the point.
            osg::Vec3d look(0,1,0);
            double     min_dp   = 1.0;
            osg::Vec3d rightmost_p;
            f++; // the Far plane face
            for(unsigned i=0; i<f->vertices.size(); ++i)
            {
                osg::Vec3d p = f->vertices[i] * rttMVP;
                // only check points on the right (since it's symmetrical)
                if ( p.x() > 0 ) 
                {
                    osg::Vec3d pv(p.x(), p.y()+1.0, 0); pv.normalize();
                    double dp = look * pv;
                    if ( dp < min_dp )
                    {
                        min_dp = dp;
                        rightmost_p = p;
                    }
                }
            }

            // Now calculate the far extent. This is an iterative process;
            // If either the aspectRatio or far/near-ratio limits are exceeded
            // by the value we calculate, reset the near width to accomodate
            // and try again. Worst case this should be no more than 3 iterations.
            double minHalfWidthNear = halfWidthNear;

            Line2d farLine( osg::Vec3d(-1,1,0), osg::Vec3d(1,1,0) );

            int iterations = 0;
            while(
                (aspectRatio > maxApsectRatio || farNearRatio > maxFarNearRatio) &&
                (halfWidthFar > halfWidthNear) &&
                (iterations++ < 10) )
            {
                // make sure all the far-clip verts are inside our polygon.
                // stretch out the far line to accomodate them.
                osg::Vec3d NR( halfWidthNear, -1, 0);

                osg::Vec3d i;
                Line2d( NR, rightmost_p ).intersect( farLine, i );
                halfWidthFar = i.x();

                aspectRatio  = (halfWidthFar-halfWidthNear)/2.0;
                if ( aspectRatio > maxApsectRatio )
                {
                    halfWidthNear = halfWidthFar - 2.0*maxApsectRatio;
                }

                farNearRatio = halfWidthFar/halfWidthNear;
                if ( farNearRatio > maxFarNearRatio )
                {
                    halfWidthNear = halfWidthFar / maxFarNearRatio;
                    //break;
                }

                halfWidthNear = std::max(halfWidthNear, minHalfWidthNear);
            }

            // if the far plane is narrower than the near plane, bail out and 
            // fall back on a simple rectangular clip camera.
            if ( halfWidthFar <= halfWidthNear )
                return;

            //OE_NOTICE  << "\n"
            //    << "HN = " << halfWidthNear << "\n"
            //    << "HF = " << halfWidthFar << "\n"
            //    << "AR = " << aspectRatio << "\n"
            //    << "FNR= " << farNearRatio << "\n"
            //    << std::endl;

            // construct the polygon.
            t0.set(  halfWidthFar,   1.0, 0.0, 1.0 );
            t1.set( -halfWidthFar,   1.0, 0.0, 1.0 );
            t2.set( -halfWidthNear, -1.0, 0.0, 1.0 );
            t3.set(  halfWidthNear, -1.0, 0.0, 1.0 );
        }

        // OK now warp our polygon t0,t1,t2,t3 into a clip-space square
        // through a series of matrix operations.
        osg::Vec4d  u, v;
        osg::Matrix M;
        
        // translate the center of the near plane to the origin
        u = (t2 + t3) / 2.0;
        osg::Matrix T1;
        T1.makeTranslate(-u.x(), -u.y(), 0.0);
        M = T1;

        // find the intersection of the side lines t0,t3 and t1,t2
        // and translate that point is at the origin:
        osg::Vec4d i;
        Line2d(t0, t3).intersect( Line2d(t1, t2), i );
        u = i*M;
        osg::Matrix T2;
        T2.makeTranslate( -u.x(), -u.y(), 0.0 );
        M = T2*M;

        // scale the near corners to [-1,1] and [1,1] respectively:
        u = t3*M; // ...not t2.
        osg::Matrix S1;
        S1.makeScale( 1/u.x(), 1/u.y(), 1.0 );
        M = M*S1;

        // project onto the Y plane and translate the whole thing
        // back down to the origin at the same time.
        osg::Matrix N(
            1,  0, 0, 0,
            0,  1, 0, 1,
            0,  0, 1, 0,
            0, -1, 0, 0);
        M = M*N;

        // scale it back to unit size:
        u = t0*M;
        v = t3*M;
        osg::Matrix S3;
        S3.makeScale( 1.0, 2.0/(u.y()/u.w() - v.y()/v.w()), 1.0 );
        M = M*S3;

        // finally, translate it to it lines up with the clip space boundaries.
        osg::Matrix T4;
        T4.makeTranslate( 0.0, -1.0, 0.0 );
        M = M*T4;

        // apply the result to the projection matrix.
        params._rttProjMatrix.postMult( M );

        // btw, this new clip matrix distorts the Z coordinate as
        // y approaches +1. That can cause bleed-through in a geocentric
        // terrain from the other side of the globe. To prevent that, sample a 
        // point at the near plane and record that as the Maximum allowable
        // Z coordinate; a vertex shader in the RTT camera will enforce this.
        osg::Vec4d sampleFar = osg::Vec4d(0,1,1,1) * M;

        //TODO: add this to the shader.
        //local._rttLimitZ->set( (float)sampleFar.z() );
    }
}

//---------------------------------------------------------------------------

DrapingTechnique::DrapingTechnique() :
_textureUnit     ( 1 ),
_textureSize     ( 1024 ),
_mipmapping      ( false ),
_rttBlending     ( true ),
_attachStencil   ( false ),
_maxFarNearRatio ( 5.0 )
{
    _supported = Registry::capabilities().supportsGLSL();

    // try newer version
    const char* nfr2 = ::getenv("OSGEARTH_OVERLAY_RESOLUTION_RATIO");
    if ( nfr2 )
        _maxFarNearRatio = as<double>(nfr2, 0.0);
}


bool
DrapingTechnique::hasData(OverlayDecorator::TechRTTParams& params) const
{
    return params._group->getNumChildren() > 0;
}


void
DrapingTechnique::reestablish(TerrainEngineNode* engine)
{
    if ( !_textureUnit.isSet() )
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
        else if ( !_textureUnit.isSet() )
        {
            int texUnit;
            if ( engine->getTextureCompositor()->reserveTextureImageUnit( texUnit ) )
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
DrapingTechnique::setUpCamera(OverlayDecorator::TechRTTParams& params)
{
    // create the projected texture:
    osg::Texture2D* projTexture = new osg::Texture2D();
    projTexture->setTextureSize( *_textureSize, *_textureSize );
    projTexture->setInternalFormat( GL_RGBA );
    projTexture->setSourceFormat( GL_RGBA );
    projTexture->setSourceType( GL_UNSIGNED_BYTE );
    projTexture->setFilter( osg::Texture::MIN_FILTER, _mipmapping? osg::Texture::LINEAR_MIPMAP_LINEAR: osg::Texture::LINEAR );
    projTexture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
    projTexture->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_BORDER );
    projTexture->setWrap( osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_BORDER );
    //projTexture->setWrap( osg::Texture::WRAP_R, osg::Texture::CLAMP_TO_EDGE );
    projTexture->setBorderColor( osg::Vec4(0,0,0,0) );

    // set up the RTT camera:
    params._rttCamera = new osg::Camera();
    params._rttCamera->setClearColor( osg::Vec4f(0,0,0,0) );
    // this ref frame causes the RTT to inherit its viewpoint from above (in order to properly
    // process PagedLOD's etc. -- it doesn't affect the perspective of the RTT camera though)
    params._rttCamera->setReferenceFrame( osg::Camera::ABSOLUTE_RF_INHERIT_VIEWPOINT );
    params._rttCamera->setViewport( 0, 0, *_textureSize, *_textureSize );
    params._rttCamera->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );
    params._rttCamera->setRenderOrder( osg::Camera::PRE_RENDER );
    params._rttCamera->setRenderTargetImplementation( osg::Camera::FRAME_BUFFER_OBJECT );
    params._rttCamera->setImplicitBufferAttachmentMask(0, 0);
    params._rttCamera->attach( osg::Camera::COLOR_BUFFER0, projTexture, 0, 0, _mipmapping );

    if ( _attachStencil )
    {
        OE_INFO << LC << "Attaching a stencil buffer to the RTT camera" << std::endl;

        // try a depth-packed buffer. failing that, try a normal one.. if the FBO doesn't support
        // that (which is doesn't on some GPUs like Intel), it will automatically fall back on 
        // a PBUFFER_RTT impl
        if ( Registry::capabilities().supportsDepthPackedStencilBuffer() )
        {
#ifdef OSG_GLES2_AVAILABLE 
            params._rttCamera->attach( osg::Camera::PACKED_DEPTH_STENCIL_BUFFER, GL_DEPTH24_STENCIL8_EXT );
#else
            params._rttCamera->attach( osg::Camera::PACKED_DEPTH_STENCIL_BUFFER, GL_DEPTH_STENCIL_EXT );
#endif
        }
        else
        {
            params._rttCamera->attach( osg::Camera::STENCIL_BUFFER, GL_STENCIL_INDEX );
        }

        params._rttCamera->setClearStencil( 0 );
        params._rttCamera->setClearMask( GL_COLOR_BUFFER_BIT | GL_STENCIL_BUFFER_BIT ); //GL_DEPTH_BUFFER_BIT |  );
    }
    else
    {
        params._rttCamera->setClearMask( GL_COLOR_BUFFER_BIT ); //| GL_DEPTH_BUFFER_BIT );
    }

    // set up a StateSet for the RTT camera.
    osg::StateSet* rttStateSet = params._rttCamera->getOrCreateStateSet();

    // lighting is off. We don't want draped items to be lit.
    rttStateSet->setMode( GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );

    // install a new default shader program that replaces anything from above.
    VirtualProgram* rtt_vp = VirtualProgram::getOrCreate(rttStateSet);
    rtt_vp->setName( "DrapingTechnique RTT" );
    rtt_vp->setInheritShaders( false );
    
    // activate blending within the RTT camera's FBO
    if ( _rttBlending )
    {
        //Setup a separate blend function for the alpha components and the RGB components.  
        //Because the destination alpha is initialized to 0 instead of 1
        osg::BlendFunc* blendFunc = 0;        
        if (Registry::capabilities().supportsGLSL(140u))
        {
            //Blend Func Separate is only available on OpenGL 1.4 and above
            blendFunc = new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
        }
        else
        {
            blendFunc = new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        }

        rttStateSet->setAttributeAndModes(blendFunc, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    }
    else
    {
        rttStateSet->setMode(GL_BLEND, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
    }

    // attach the overlay group to the camera. 
    // TODO: we should probably lock this since other cull traversals might be accessing the group
    //       while we are changing its children.
    params._rttCamera->addChild( params._group );

    // overlay geometry is rendered with no depth testing, and in the order it's found in the
    // scene graph... until further notice.
    rttStateSet->setMode(GL_DEPTH_TEST, 0);
    rttStateSet->setBinName( "TraversalOrderBin" );

    // add to the terrain stateset, i.e. the stateset that the OverlayDecorator will
    // apply to the terrain before cull-traversing it. This will activate the projective
    // texturing on the terrain.
    params._terrainStateSet->setTextureAttributeAndModes( *_textureUnit, projTexture, osg::StateAttribute::ON );

    // fire up the local per-view data:
    LocalPerViewData* local = new LocalPerViewData();
    params._techniqueData = local;
    

    // Assemble the terrain shaders that will apply projective texturing.
    VirtualProgram* terrain_vp = VirtualProgram::getOrCreate(params._terrainStateSet);
    terrain_vp->setName( "DrapingTechnique terrain shaders");

    // sampler for projected texture:
    params._terrainStateSet->getOrCreateUniform(
        "oe_overlay_tex", osg::Uniform::SAMPLER_2D )->set( *_textureUnit );

    // the texture projection matrix uniform.
    local->_texGenUniform = params._terrainStateSet->getOrCreateUniform(
        "oe_overlay_texmatrix", osg::Uniform::FLOAT_MAT4 );

    // vertex shader - subgraph
    std::string vs =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"
        "uniform mat4 oe_overlay_texmatrix; \n"
        "varying vec4 oe_overlay_texcoord; \n"

        "void oe_overlay_vertex(inout vec4 VertexVIEW) \n"
        "{ \n"
        "    oe_overlay_texcoord = oe_overlay_texmatrix * VertexVIEW; \n"
        "} \n";

    terrain_vp->setFunction( "oe_overlay_vertex", vs, ShaderComp::LOCATION_VERTEX_VIEW );

    // fragment shader - subgraph
    std::string fs =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"
        "uniform sampler2D oe_overlay_tex; \n"
        "varying vec4      oe_overlay_texcoord; \n"

        "void oe_overlay_fragment( inout vec4 color ) \n"
        "{ \n"
        "    vec4 texel = texture2DProj(oe_overlay_tex, oe_overlay_texcoord); \n"
        "    color = vec4( mix( color.rgb, texel.rgb, texel.a ), color.a); \n"
        "} \n";

    terrain_vp->setFunction( "oe_overlay_fragment", fs, ShaderComp::LOCATION_FRAGMENT_COLORING );
}


void
DrapingTechnique::preCullTerrain(OverlayDecorator::TechRTTParams& params,
                                 osgUtil::CullVisitor*             cv )
{
    if ( !params._rttCamera.valid() && params._group->getNumChildren() > 0 && _textureUnit.isSet() )
    {
        setUpCamera( params );
    }
}


void
DrapingTechnique::cullOverlayGroup(OverlayDecorator::TechRTTParams& params,
                                   osgUtil::CullVisitor*            cv )
{
    if ( params._rttCamera.valid() )
    {
        // this xforms from clip [-1..1] to texture [0..1] space
        static osg::Matrix s_scaleBiasMat = 
            osg::Matrix::translate(1.0,1.0,1.0) * 
            osg::Matrix::scale(0.5,0.5,0.5);

        // resolution weighting based on camera distance.
        if ( _maxFarNearRatio > 1.0 )
        {
            optimizeProjectionMatrix( params, _maxFarNearRatio );
        }

        params._rttCamera->setViewMatrix      ( params._rttViewMatrix );
        params._rttCamera->setProjectionMatrix( params._rttProjMatrix );

        osg::Matrix VPT = params._rttViewMatrix * params._rttProjMatrix * s_scaleBiasMat;

        LocalPerViewData& local = *static_cast<LocalPerViewData*>(params._techniqueData.get());

        if ( local._texGenUniform.valid() )
        {
            // premultiply the inv view matrix so we don't have precision problems in the shader 
            // (and it's faster too)

            // TODO:
            // This only works properly if the terrain tiles have a DYNAMIC data variance.
            // That is because we are setting a Uniform value during the CULL traversal, and
            // it's possible that the stateset from the previous frame has not yet been
            // dispatched to render. So we need to come up with a way to address this.
            // In the meantime, I patched the MP engine to set a DYNAMIC data variance on
            // terrain tiles to work around the problem.
            //
            // Note that we require the InverseViewMatrix, but it is OK to invert the ModelView matrix as the model matrix is identity here.
            osg::Matrix vm;
            vm.invert( *cv->getModelViewMatrix() );
            local._texGenUniform->set( vm * VPT );
        }

        // traverse the overlay group (via the RTT camera).
        params._rttCamera->accept( *cv );
    }
}


void
DrapingTechnique::setTextureSize( int texSize )
{
    _textureSize = texSize;
}

void
DrapingTechnique::setTextureUnit( int texUnit )
{
    if ( !_explicitTextureUnit.isSet() || texUnit != _explicitTextureUnit.value() )
    {
        _explicitTextureUnit = texUnit;
    }
}

void
DrapingTechnique::setMipMapping( bool value )
{
    if ( value != _mipmapping )
    {
        _mipmapping = value;

        if ( _mipmapping )
            OE_INFO << LC << "Overlay mipmapping " << (value?"enabled":"disabled") << std::endl;
    }
}

void
DrapingTechnique::setOverlayBlending( bool value )
{
    if ( value != _rttBlending )
    {
        _rttBlending = value;
        
        if ( _rttBlending )
            OE_INFO << LC << "Overlay blending " << (value?"enabled":"disabled")<< std::endl;
    }
}

bool
DrapingTechnique::getAttachStencil() const
{
    return _attachStencil;
}

void
DrapingTechnique::setAttachStencil( bool value )
{
    _attachStencil = value;
}

void
DrapingTechnique::setResolutionRatio(float value)
{
    // not a typo. "near/far resolution" is equivalent to "far/near clip plane extent"
    // with respect to the overlay projection frustum.
    _maxFarNearRatio = (double)osg::clampAbove(value, 1.0f);
}

float
DrapingTechnique::getResolutionRatio() const
{
    // not a typo. "near/far resolution" is equivalent to "far/near clip plane extent"
    // with respect to the overlay projection frustum.
    return (float)_maxFarNearRatio;
}

void
DrapingTechnique::onInstall( TerrainEngineNode* engine )
{
    if ( !_textureSize.isSet() )
    {
        unsigned maxSize = Registry::capabilities().getMaxFastTextureSize();
        _textureSize.init( osg::minimum( 2048u, maxSize ) );
    }
    OE_INFO << LC << "Using texture size = " << *_textureSize << std::endl;
}

void
DrapingTechnique::onUninstall( TerrainEngineNode* engine )
{
    if ( !_explicitTextureUnit.isSet() && _textureUnit.isSet() )
    {
        engine->getTextureCompositor()->releaseTextureImageUnit( *_textureUnit );
        _textureUnit.unset();
    }
}
