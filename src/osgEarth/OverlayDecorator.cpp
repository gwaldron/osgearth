/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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
#include <osgEarth/ShaderComposition>
#include <osgEarth/Capabilities>
#include <osg/Texture2D>
#include <osg/TexEnv>
#include <osg/BlendFunc>
#include <osg/ShapeDrawable>
#include <osg/AutoTransform>
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
#if 0
    /**
     * Creates a polytope that minimally bounds a bounding sphere
     * in world space.
     */
    void computeWorldBoundingPolytope(const osg::BoundingSphere&  bs, 
                                      const SpatialReference*     srs,
                                      bool                        geocentric,
                                      osg::Polytope&              out_polytope)
    {
        out_polytope.clear();
        const osg::EllipsoidModel* ellipsoid = srs->getEllipsoid();

        // add planes for the four sides of the BS. Normals point inwards.
        out_polytope.add( osg::Plane(osg::Vec3d( 1, 0,0), osg::Vec3d(-bs.radius(),0,0)) );
        out_polytope.add( osg::Plane(osg::Vec3d(-1, 0,0), osg::Vec3d( bs.radius(),0,0)) );
        out_polytope.add( osg::Plane(osg::Vec3d( 0, 1,0), osg::Vec3d(0, -bs.radius(),0)) );
        out_polytope.add( osg::Plane(osg::Vec3d( 0,-1,0), osg::Vec3d(0,  bs.radius(),0)) );

        // for a projected map, we're done. For a geocentric one, add a bottom cap.
        if ( geocentric )
        {
            // add a bottom cap, unless the bounds are sufficiently large.
            double minRad = std::min(ellipsoid->getRadiusPolar(), ellipsoid->getRadiusEquator());
            double maxRad = std::max(ellipsoid->getRadiusPolar(), ellipsoid->getRadiusEquator());
            double zeroOffset = bs.center().length();
            if ( zeroOffset > minRad * 0.1 )
            {
                out_polytope.add( osg::Plane(osg::Vec3d(0,0,1), osg::Vec3d(0,0,-maxRad+zeroOffset)) );
            }
        }

        // transform the clipping planes ito world space localized about the center point
        GeoPoint refPoint;
        refPoint.fromWorld( srs, bs.center() );
        osg::Matrix local2world;
        refPoint.createLocalToWorld( local2world );

        out_polytope.transform( local2world );
    }


    /**
     * Tests whether a "cohesive" point set intersects a polytope. This differs from
     * Polytope::contains(verts); that function tests each point individually, whereas
     * this method tests the point set as a whole (i.e. as the border points of a solid --
     * we are testing whether this solid intersects the polytope.)
     */
    bool pointSetIntersectsClippingPolytope(const std::vector<osg::Vec3>& points, osg::Polytope& pt)
    {
        osg::Polytope::PlaneList& planes = pt.getPlaneList();
        for( osg::Polytope::PlaneList::iterator plane = planes.begin(); plane != planes.end(); ++plane )
        {
            unsigned outsides = 0;
            for( std::vector<osg::Vec3>::const_iterator point = points.begin(); point != points.end(); ++point )
            {
                bool outside = plane->distance( *point ) < 0.0f;
                if ( outside ) outsides++;
                else break;
            }
            if ( outsides == points.size() ) 
                return false;
        }
        return true;
    }


    /**
     * Visitor that computes a bounding sphere for the geometry that intersects
     * a frustum polyhedron. Since this is used to project geometry on to the 
     * terrain surface, it has to account for geometry that is not clamped --
     * so instead of using the normal bounding sphere it computes a world-space
     * polytope for each geometry and interests that with the frustum.
     */
    struct ComputeBoundsWithinFrustum : public OverlayDecorator::InternalNodeVisitor
    {
        std::vector<osg::Vec3>  _frustumVerts;

        ComputeBoundsWithinFrustum(const osgShadow::ConvexPolyhedron& frustumPH, 
                                   const SpatialReference* srs,
                                   bool                    geocentric,
                                   osg::BoundingSphere&    out_bs)
            : InternalNodeVisitor(),
              _srs       ( srs ),
              _geocentric( geocentric ),
              _bs        ( out_bs )
        {
            frustumPH.getPolytope( _originalPT );

            _polytopeStack.push( _originalPT );
            _local2worldStack.push( osg::Matrix::identity() );
            _world2localStack.push( osg::Matrix::identity() );

            // extract the corner verts from the frustum polyhedron; we will use those to
            // test for intersection.
            std::vector<osg::Vec3d> temp;
            temp.reserve( 8 );
            frustumPH.getPoints( temp );
            for( unsigned i=0; i<temp.size(); ++i )
                _frustumVerts.push_back(temp[i]);
        }

        bool contains( const osg::BoundingSphere& bs )
        {
            osg::BoundingSphere worldBS( bs.center() * _local2worldStack.top(), bs.radius() );
            osg::Polytope bsWorldPT;
            computeWorldBoundingPolytope( worldBS, _srs, _geocentric, bsWorldPT );
            return pointSetIntersectsClippingPolytope( _frustumVerts, bsWorldPT );
        }

        void apply( osg::Node& node )
        {
            const osg::BoundingSphere& bs = node.getBound();
            if ( contains(bs) )
            {
                traverse( node );
            }
        }

        void apply( osg::Geode& node )
        {
            const osg::BoundingSphere& bs = node.getBound();
            if ( contains(bs) )
            {
                _bs.expandBy( osg::BoundingSphere(
                    bs.center() * _local2worldStack.top(),
                    bs.radius() ) );
            }
        }

        void apply( osg::Transform& transform )
        {
            osg::Matrixd local2world;
            transform.computeLocalToWorldMatrix( local2world, this );

            _local2worldStack.push( local2world );

            _polytopeStack.push( _originalPT );
            _polytopeStack.top().transformProvidingInverse( local2world );

            osg::Matrix world2local;
            world2local.invert( local2world );
            _world2localStack.push( world2local );

            traverse(transform);

            _local2worldStack.pop();
            _polytopeStack.pop();
        }

        osg::BoundingSphere&      _bs;
        const SpatialReference*   _srs;
        bool                      _geocentric;
        osg::Polytope             _originalPT;
        std::stack<osg::Polytope> _polytopeStack;
        std::stack<osg::Matrixd>  _local2worldStack, _world2localStack;
    };
#endif


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
_textureUnit     ( 1 ),
_textureSize     ( 1024 ),
_useShaders      ( false ),
_mipmapping      ( false ),
_rttBlending     ( true ),
_updatePending   ( false ),
_dumpRequested   ( false ),
_rttTraversalMask( ~0 )
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
        else if ( !_textureUnit.isSet() ) //&& _useShaders )
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

    // install a new default shader program that replaces anything from above.
    VirtualProgram* vp = new VirtualProgram();
    vp->setName( "overlay rtt" );
    vp->installDefaultColoringAndLightingShaders();
    vp->setInheritShaders( false );
    rttStateSet->setAttributeAndModes( vp, osg::StateAttribute::ON );
    
    if ( _rttBlending )
    {
        //osg::BlendFunc* blendFunc = new osg::BlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
        //osg::BlendFunc* blendFunc = new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        //Setup a separate blend function for the alpha components and the RGB components.  
        //Because the destination alpha is initialized to 0 instead of 1
        osg::BlendFunc* blendFunc = 0;        
        if (Registry::instance()->getCapabilities().supportsGLSL(1.4f))
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
    
    if ( _useShaders )
    {            
        // set up the shaders
        initSubgraphShaders( pvd );
    }
    else
    {
        // FFP path:
        pvd._texGenNode = new osg::TexGenNode();
        pvd._texGenNode->setTexGen( new osg::TexGen() );
    }
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

    // vertex shader - subgraph
    std::string vertexSource = Stringify()
        << "#version " << GLSL_VERSION_STR << "\n"
#ifdef OSG_GLES2_AVAILABLE
        << "precision mediump float;\n"
#endif
        << "uniform mat4 osgearth_overlay_TexGenMatrix; \n"
        << "uniform mat4 osg_ViewMatrixInverse; \n"

        << "void osgearth_overlay_vertex(void) \n"
        << "{ \n"
        << "    osg_TexCoord["<< *_textureUnit << "] = osgearth_overlay_TexGenMatrix * osg_ViewMatrixInverse * gl_ModelViewMatrix * gl_Vertex; \n"
        << "} \n";

    vp->setFunction( "osgearth_overlay_vertex", vertexSource, ShaderComp::LOCATION_VERTEX_POST_LIGHTING );

    // fragment shader - subgraph
    std::string fragmentSource = Stringify()
        << "#version " << GLSL_VERSION_STR << "\n"
#ifdef OSG_GLES2_AVAILABLE
        << "precision mediump float;\n"
#endif
        << "uniform sampler2D osgearth_overlay_ProjTex; \n"
        << "void osgearth_overlay_fragment( inout vec4 color ) \n"
        << "{ \n"
        << "    vec2 texCoord = osg_TexCoord["<< *_textureUnit << "].xy / osg_TexCoord["<< *_textureUnit << "].q; \n"
        << "    vec4 texel = texture2D(osgearth_overlay_ProjTex, texCoord); \n"  
        << "    color = vec4( mix( color.rgb, texel.rgb, texel.a ), color.a); \n"
        << "} \n";

    vp->setFunction( "osgearth_overlay_fragment", fragmentSource, ShaderComp::LOCATION_FRAGMENT_POST_LIGHTING );
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
OverlayDecorator::setOverlayGraphTraversalMask( unsigned mask )
{
    _rttTraversalMask = mask;
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
    _srs = info.getProfile()->getSRS();
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
OverlayDecorator::updateRTTCamera( OverlayDecorator::PerViewData& pvd )
{
    static osg::Matrix normalizeMatrix = 
        osg::Matrix::translate(1.0,1.0,1.0) * osg::Matrix::scale(0.5,0.5,0.5);

    pvd._rttCamera->setViewMatrix( pvd._rttViewMatrix );
    pvd._rttCamera->setProjectionMatrix( pvd._rttProjMatrix );

    osg::Matrix MVPT = pvd._rttViewMatrix * pvd._rttProjMatrix * normalizeMatrix;

    if ( pvd._texGenUniform.valid() )
    {
        pvd._texGenUniform->set( MVPT );
    }
    else
    {
        pvd._texGenNode->getTexGen()->setPlanesFromMatrix( MVPT );
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
        
        //Actually sample the terrain to get the height and adjust the eye position so it's a tighter fit to the real data.
        double height;
        if (_engine->getTerrain()->getHeight( SpatialReference::create("epsg:4326"), osg::RadiansToDegrees( lon ), osg::RadiansToDegrees( lat ), &height))
        {
            hasl -= height;
        }
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

    // restore the clip planes in the cull visitor, now that we have our subgraph
    // projection matrix.
    cv->setCalculatedNearPlane( osg::minimum(zSavedNear, zNear) );
    cv->setCalculatedFarPlane( osg::maximum(zSavedFar, zFar) );

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
       
    // contruct the polyhedron representing the viewing frustum.
    osgShadow::ConvexPolyhedron frustumPH;
    //MyConvexPolyhedron frustumPH;
    frustumPH.setToUnitFrustum( true, true );
    osg::Matrixd MVP = *cv->getModelViewMatrix() * projMatrix;
    osg::Matrixd inverseMVP;
    inverseMVP.invert(MVP);
    frustumPH.transform( inverseMVP, MVP );

    // take the bounds of the overlay graph, constrained to the view frustum:
    osg::BoundingSphere visibleOverlayBS;
    osg::Polytope frustumPT;
    frustumPH.getPolytope( frustumPT );

    // get a bounds of the overlay graph as a whole, and convert that to a
    // bounding box. We can probably do better with a ComputeBoundsVisitor but it
    // will be slower.
    visibleOverlayBS = _overlayGraph->getBound();
    osg::BoundingBox visibleOverlayBBox;
    visibleOverlayBBox.expandBy( visibleOverlayBS );

    // intersect that bound with the camera frustum:
    osg::Polytope visibleOverlayPT;
    visibleOverlayPT.setToBoundingBox( visibleOverlayBBox );
    osgShadow::ConvexPolyhedron visiblePH( frustumPH );
    visiblePH.cut( visibleOverlayPT );

#if 0
    // This method does not work. Like with larged paged feature sets.

    ComputeBoundsWithinFrustum cbwp( frustumPH, _srs.get(), _isGeocentric, visibleOverlayBS );
    _overlayGraph->accept( cbwp );

    // convert the visible geometry bounding sphere into a world-space polytope:
    osg::Polytope visiblePT;
    osgShadow::ConvexPolyhedron visiblePH( frustumPH );

    // this intersects the viewing frustum with the subgraph's bounding box, basically giving us
    // a "minimal" polyhedron containing all potentially visible geometry. (It can't be truly 
    // minimal without clipping at the geometry level, but that would probably be too expensive.)

    computeWorldBoundingPolytope( visibleOverlayBS, _srs.get(), _isGeocentric, visiblePT );
    visiblePH.cut( visiblePT );
#endif

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

    if ( _dumpRequested )
    {
        static const char* fn = "convexpolyhedron.osg";

        // camera frustum:
        {
            osgShadow::ConvexPolyhedron frustumPH;
            frustumPH.setToUnitFrustum( true, true );
            osg::Matrixd MVP = *cv->getModelViewMatrix() * projMatrix;
            osg::Matrixd inverseMVP;
            inverseMVP.invert(MVP);
            frustumPH.transform( inverseMVP, MVP );
            frustumPH.dumpGeometry(0,0,0,fn);
        }
        osg::Node* camNode = osgDB::readNodeFile(fn);
        camNode->setName("camera");

        //// visible PH or overlay:
        //visiblePHBeforeCut.dumpGeometry(0,0,0,fn,osg::Vec4(0,1,1,1),osg::Vec4(0,1,1,.25));
        //osg::Node* overlay = osgDB::readNodeFile(fn);
        //overlay->setName("overlay");

        // visible overlay Polyherdron AFTER frustum intersection:
        visiblePH.dumpGeometry(0,0,0,fn,osg::Vec4(1,.5,1,1),osg::Vec4(1,.5,0,.25));
        osg::Node* intersection = osgDB::readNodeFile(fn);
        intersection->setName("intersection");

        // RTT frustum:
        {
            osgShadow::ConvexPolyhedron rttPH;
            rttPH.setToUnitFrustum( true, true );
            osg::Matrixd MVP = pvd._rttViewMatrix * pvd._rttProjMatrix;
            osg::Matrixd inverseMVP;
            inverseMVP.invert(MVP);
            rttPH.transform( inverseMVP, MVP );
            rttPH.dumpGeometry(0,0,0,fn,osg::Vec4(1,1,0,1),osg::Vec4(1,1,0,0.25));
        }
        osg::Node* rttNode = osgDB::readNodeFile(fn);
        rttNode->setName("rtt");

        // EyePoint
        osg::Geode* dsg = new osg::Geode();
        dsg->addDrawable( new osg::ShapeDrawable(new osg::Box(osg::Vec3f(0,0,0), 10.0f)));
        osg::AutoTransform* dsgmt = new osg::AutoTransform();
        dsgmt->setPosition( osg::Vec3d(0,0,0) * osg::Matrix::inverse(*cv->getModelViewMatrix()) );
        dsgmt->setAutoScaleToScreen(true);
        dsgmt->addChild( dsg );

        osg::Group* g = new osg::Group();
        g->getOrCreateStateSet()->setAttribute(new osg::Program(), 0);
        g->addChild(camNode);
        //g->addChild(overlay);
        g->addChild(intersection);
        g->addChild(rttNode);
        g->addChild(dsgmt);

        _dump = g;
        _dumpRequested = false;
    }
}

OverlayDecorator::PerViewData&
OverlayDecorator::getPerViewData(osg::NodeVisitor* key)
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
            PerViewData& pvd = getPerViewData( cv );
            if ( cv->getCurrentCamera() )
            {
                if ( (_rttTraversalMask & nv.getTraversalMask()) != 0 )
                {
                    if (checkNeedsUpdate(pvd))
                    {
                        updateRTTCamera(pvd);
                    }

                    if ( pvd._texGenNode.valid() ) // FFP only
                        pvd._texGenNode->accept( nv );

                    cull( cv, pvd );
                    pvd._rttCamera->accept( nv );
                }
                else
                {
                    osg::Group::traverse(nv);
                }
            }

            // debug-- (draws the overlay at its native location as well)
            //_overlayGraph->accept(nv);
        }

        else if ( nv.getVisitorType() == nv.UPDATE_VISITOR )
        {
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
        osg::Group::traverse( nv );
    }
}


bool
OverlayDecorator::checkNeedsUpdate( OverlayDecorator::PerViewData& pvd )
{
    return
        pvd._rttCamera->getViewMatrix()       != pvd._rttViewMatrix ||
        pvd._rttCamera->getProjectionMatrix() != pvd._rttProjMatrix ||
        (_overlayGraph.valid() && _overlayGraph->getNumChildrenRequiringUpdateTraversal() > 0);
}
