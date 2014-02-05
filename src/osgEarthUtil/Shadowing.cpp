/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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
#include <osgEarthUtil/Shadowing>
#include <osgEarth/CullingUtils>
#include <osgEarth/VirtualProgram>
#include <osg/Texture2D>
#include <osg/CullFace>
#include <osg/PolygonOffset>
#include <osgShadow/ConvexPolyhedron>

using namespace osgEarth::Util;

namespace
{
    /**
     * Takes a set of world verts and finds their bounding box in the 
     * plane of the camera represented by the specified view matrix.
     */
    void
    getExtentInView(const osg::Matrix& viewMatrix,
                    std::vector<osg::Vec3d>& verts,
                    osg::BoundingBoxd& bbox)
    {
        bbox.set(DBL_MAX, DBL_MAX, DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX);

        for( std::vector<osg::Vec3d>::iterator i = verts.begin(); i != verts.end(); ++i )
        {
            osg::Vec3d d = (*i) * viewMatrix; // world to view
            if ( d.x() < bbox.xMin() ) bbox.xMin() = d.x();
            if ( d.x() > bbox.xMax() ) bbox.xMax() = d.x();
            if ( d.y() < bbox.yMin() ) bbox.yMin() = d.y();
            if ( d.y() > bbox.yMax() ) bbox.yMax() = d.y();
            if ( d.z() < bbox.zMin() ) bbox.zMin() = d.z();
            if ( d.z() > bbox.zMax() ) bbox.zMax() = d.z();
        }
    }
}



ShadowCaster::ShadowCaster() :
_size( 2048 ),
_unit( 7 )
{
    reinitialize();
}

void
ShadowCaster::reinitialize()
{
    // create the projected texture:
    _shadowmap = new osg::Texture2D();
    _shadowmap->setTextureSize( _size, _size );
    _shadowmap->setInternalFormat( GL_DEPTH_COMPONENT );
    _shadowmap->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
    _shadowmap->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
    _shadowmap->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_BORDER );
    _shadowmap->setWrap( osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_BORDER );
    _shadowmap->setBorderColor(osg::Vec4(1,1,1,1));
    _shadowmap->setShadowComparison(true);
    _shadowmap->setShadowTextureMode(osg::Texture::LUMINANCE);

    // set up the RTT camera:
    _rtt = new osg::Camera();
    _rtt->setReferenceFrame( osg::Camera::ABSOLUTE_RF_INHERIT_VIEWPOINT );
    _rtt->setClearColor( osg::Vec4f(0,0,0,0) );
    _rtt->setClearMask( GL_DEPTH_BUFFER_BIT );
    _rtt->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );
    _rtt->setViewport( 0, 0, _size, _size );
    _rtt->setRenderOrder( osg::Camera::PRE_RENDER );
    _rtt->setRenderTargetImplementation( osg::Camera::FRAME_BUFFER_OBJECT );
    _rtt->setImplicitBufferAttachmentMask(0, 0);
    _rtt->attach( osg::Camera::DEPTH_BUFFER, _shadowmap.get() );

    _rttStateSet = new osg::StateSet();
    _rttStateSet->setAttributeAndModes( 
        new osg::CullFace(osg::CullFace::FRONT),
        osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    
    _renderStateSet = new osg::StateSet();
    
    const char* vertex =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"
        "uniform mat4 oe_shadowmap_matrix; \n"
        "varying float oe_shadow_ambient; \n"
        "varying vec4 oe_shadow_coord; \n"
        "void oe_shadow_vertex(inout vec4 VertexVIEW) \n"
        "{ \n"
        "    oe_shadow_coord = oe_shadowmap_matrix * VertexVIEW; \n"
        "    oe_shadow_ambient = 0.5; \n"
        "} \n";

    const char* fragment =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"
        "uniform sampler2DShadow oe_shadow_map; \n"
        "varying vec4 oe_shadow_coord; \n"
        "varying float oe_shadow_ambient; \n"
        "void oe_shadow_fragment( inout vec4 color )\n"
        "{\n"
        "    float alpha = color.a; \n"
        "    float shadowFac = shadow2DProj(oe_shadow_map, oe_shadow_coord).r; \n"
        "    vec4 colorInFullShadow = color * oe_shadow_ambient; \n"
        "    color = mix(colorInFullShadow, color, shadowFac); \n"
        "    color.a = alpha;\n"
        "}\n";

    VirtualProgram* vp = VirtualProgram::getOrCreate(_renderStateSet.get());

    vp->setFunction(
        "oe_shadow_vertex", 
        vertex, 
        ShaderComp::LOCATION_VERTEX_VIEW );

    vp->setFunction(
        "oe_shadow_fragment",
        fragment,
        ShaderComp::LOCATION_FRAGMENT_LIGHTING, 10.0f);

    // the texture coord generator matrix (from the caster):
    _shadowMapTexGenUniform = new osg::Uniform(
        osg::Uniform::FLOAT_MAT4, 
        "oe_shadowmap_matrix");

    _renderStateSet->addUniform( _shadowMapTexGenUniform.get() ); 

    // bind the shadow map texture itself:
    _renderStateSet->setTextureAttributeAndModes(
        _unit,
        _shadowmap.get(),
        osg::StateAttribute::ON );

    _renderStateSet->addUniform( new osg::Uniform("oe_shadow_map", _unit) );
}

void
ShadowCaster::traverse(osg::NodeVisitor& nv)
{
    if ( nv.getVisitorType() == nv.CULL_VISITOR )
    {
        osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);
        osg::Camera* camera = cv->getCurrentCamera();
        if ( camera )
        {
            osg::Matrix world2view    = camera->getViewMatrix();
            osg::Quat   world2viewRot = world2view.getRotate();

            osg::Matrix MV = *cv->getModelViewMatrix();
            osg::Matrix inverseMV;
            inverseMV.invert(MV);

            osg::Vec3d camEye, camTo, camUp;
            MV.getLookAt( camEye, camTo, camUp, 1.0 );

            // position the light. We only really care about the directional vector.
            osg::Vec4d lp4 = _light->getPosition();
            osg::Vec3d lightVectorWorld( -lp4.x(), -lp4.y(), -lp4.z() );
            lightVectorWorld.normalize();
            osg::Vec3d lightPosWorld = osg::Vec3d(0,0,0) * inverseMV;

            // construct the view matrix for the light. The up vector doesn't really
            // matter so we'll just use the camera's.
            osg::Matrix lightViewMat;
            lightViewMat.makeLookAt(lightPosWorld, lightPosWorld+lightVectorWorld, camUp);
            
            // take the view frustum and clamp it's far plane.
            double n = 1.0, f = 3000.0;
            osg::Matrix proj = _prevProjMatrix;
            cv->clampProjectionMatrix(proj, n, f);

            osg::Matrix MVP = MV * proj;
            osg::Matrix inverseMVP;
            inverseMVP.invert(MVP);

            // extract the corner points of the camera frustum.
            osgShadow::ConvexPolyhedron frustumPH;
            frustumPH.setToUnitFrustum(true, true);
            frustumPH.transform( inverseMVP, MVP );
            std::vector<osg::Vec3d> verts;
            frustumPH.getPoints( verts );

            // project those on to the plane of the light camera and fit them
            // to a bounding box. That box will form the extent of our orthographic camera.
            osg::BoundingBoxd bbox;
            getExtentInView(lightViewMat, verts, bbox);
            osg::Matrix lightProjMat;
            n = -std::max(bbox.zMin(), bbox.zMax());
            f = -std::min(bbox.zMin(), bbox.zMax());
            lightProjMat.makeOrtho(bbox.xMin(), bbox.xMax(), bbox.yMin(), bbox.yMax(), n, f);

            // done! set up the RTT camera and go.
            _rtt->setViewMatrix( lightViewMat );
            _rtt->setProjectionMatrix( lightProjMat );

            // this xforms from clip [-1..1] to texture [0..1] space
            static osg::Matrix s_scaleBiasMat = 
                osg::Matrix::translate(1.0,1.0,1.0) * 
                osg::Matrix::scale(0.5,0.5,0.5);
                
            // set the texture coordinate generation matrix that the shadow
            // receiver will use to sample the shadow map.
            osg::Matrix VPS = lightViewMat * lightProjMat * s_scaleBiasMat;
            _shadowMatrix = inverseMV * VPS;
            _shadowMapTexGenUniform->set( _shadowMatrix );

            cv->pushStateSet( _rttStateSet.get() );
            if ( _rtt->getNumChildren() == 0 )
                _rtt->addChild( this->getChild(0) ); // REDO - testing only
            _rtt->accept( nv );
            cv->popStateSet();
            
            cv->pushStateSet( _renderStateSet.get() );
            osg::Group::traverse( nv );
            cv->popStateSet();

            // save the projection matrix for the next frame.
            _prevProjMatrix = *cv->getProjectionMatrix();

            return;
        }
    }

    osg::Group::traverse(nv);
}
