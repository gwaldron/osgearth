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
#include <osgEarthUtil/Shadowing>
#include <osgEarth/CullingUtils>
#include <osgEarth/VirtualProgram>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osg/Texture2D>
#include <osg/CullFace>
#include <osgShadow/ConvexPolyhedron>

#define LC "[ShadowCaster] "

using namespace osgEarth::Util;


ShadowCaster::ShadowCaster() :
_size        ( 2048 ),
_texImageUnit( 7 ),
_blurFactor  ( 0.002f ),
_color       ( osg::Vec4f(.4f, .4f, .4f, 1) )
{
    _castingGroup = new osg::Group();

    _supported = Registry::capabilities().supportsGLSL();
    if ( _supported )
    {
        // defaults to 4 slices.
        _ranges.push_back(0.0f);
        _ranges.push_back(100.0f);
        _ranges.push_back(500.0f);
        _ranges.push_back(1750.0f);
        _ranges.push_back(5000.0f);

        reinitialize();
    }
    else
    {
        OE_WARN << LC << "ShadowCaster not supported (no GLSL); disabled." << std::endl;
    }
}

void
ShadowCaster::setRanges(const std::vector<float>& ranges)
{
    _ranges = ranges;
    reinitialize();
}

void
ShadowCaster::setTextureImageUnit(int unit)
{
    _texImageUnit = unit;
    reinitialize();
}

void
ShadowCaster::setTextureSize(unsigned size)
{
    _size = size;
    reinitialize();
}

void
ShadowCaster::setBlurFactor(float value)
{
    _blurFactor = value;
    if ( _shadowBlurUniform.valid() )
        _shadowBlurUniform->set(value);
}

void
ShadowCaster::setShadowColor(const osg::Vec4f& value)
{
    _color = value;
    if ( _shadowColorUniform.valid() )
        _shadowColorUniform->set(value);
}

void
ShadowCaster::reinitialize()
{
    if ( !_supported )
        return;

    _shadowmap = 0L;
    _rttCameras.clear();

    int numSlices = (int)_ranges.size() - 1;
    if ( numSlices < 1 )
    {
        OE_WARN << LC << "Illegal. Must have at least one range slice." << std::endl;
        return ;
    }

    // create the projected texture:
    _shadowmap = new osg::Texture2DArray();
    _shadowmap->setTextureSize( _size, _size, numSlices );
    _shadowmap->setInternalFormat( GL_DEPTH_COMPONENT );
    _shadowmap->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
    _shadowmap->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
    _shadowmap->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_BORDER );
    _shadowmap->setWrap( osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_BORDER );
    _shadowmap->setBorderColor(osg::Vec4(1,1,1,1));

    // set up the RTT camera:
    for(int i=0; i<numSlices; ++i)
    {
        osg::Camera* rtt = new osg::Camera();
        rtt->setReferenceFrame( osg::Camera::ABSOLUTE_RF_INHERIT_VIEWPOINT );
        rtt->setClearDepth( 1.0 );
        rtt->setClearMask( GL_DEPTH_BUFFER_BIT );
        rtt->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );
        rtt->setViewport( 0, 0, _size, _size );
        rtt->setRenderOrder( osg::Camera::PRE_RENDER );
        rtt->setRenderTargetImplementation( osg::Camera::FRAME_BUFFER_OBJECT );
        rtt->setImplicitBufferAttachmentMask(0, 0);
        rtt->attach( osg::Camera::DEPTH_BUFFER, _shadowmap.get(), 0, i );
        rtt->addChild( _castingGroup.get() );
        _rttCameras.push_back(rtt);
    }

    _rttStateSet = new osg::StateSet();

    // only draw back faces to the shadow depth map
    _rttStateSet->setAttributeAndModes( 
        new osg::CullFace(osg::CullFace::FRONT),
        osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

    _renderStateSet = new osg::StateSet();
    
    std::string vertex = Stringify() << 
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"
        "uniform mat4 oe_shadow_matrix[" << numSlices << "]; \n"
        "varying vec4 oe_shadow_coord[" << numSlices << "]; \n"
        "void oe_shadow_vertex(inout vec4 VertexVIEW) \n"
        "{ \n"
        "    for(int i=0; i<" << numSlices << "; ++i) \n"
        "        oe_shadow_coord[i] = oe_shadow_matrix[i] * VertexVIEW;\n"
        "} \n";

    std::string fragment = Stringify() << 
        "#version 120\n" //" GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"
        "#extension GL_EXT_texture_array : enable \n"

        "uniform sampler2DArray oe_shadow_map; \n"
        "uniform vec4 oe_shadow_color; \n"
        "uniform float oe_shadow_blur; \n"
        "varying vec3 oe_Normal; \n"
        "varying vec4 oe_shadow_coord[" << numSlices << "]; \n"

        //TODO-run a generator and rplace
        "#define OE_SHADOW_NUM_SAMPLES 16\n"
        "const vec2 oe_shadow_samples[OE_SHADOW_NUM_SAMPLES] = vec2[]( vec2( -0.942016, -0.399062 ), vec2( 0.945586, -0.768907 ), vec2( -0.094184, -0.929389 ), vec2( 0.344959, 0.293878 ), vec2( -0.915886, 0.457714 ), vec2( -0.815442, -0.879125 ), vec2( -0.382775, 0.276768 ), vec2( 0.974844, 0.756484 ), vec2( 0.443233, -0.975116 ), vec2( 0.53743, -0.473734 ), vec2( -0.264969, -0.41893 ), vec2( 0.791975, 0.190909 ), vec2( -0.241888, 0.997065 ), vec2( -0.8141, 0.914376 ), vec2( 0.199841, 0.786414 ), vec2( 0.143832, -0.141008 )); \n"

        "float oe_shadow_rand(vec2 co){\n"
        "   return fract(sin(dot(co.xy ,vec2(12.9898,78.233))) * 43758.5453);\n"
        "}\n"
        
        "vec2 oe_shadow_rot(vec2 p, float a) { \n"
        "    vec2 sincos = vec2(sin(a), cos(a)); \n"
        "    return vec2(dot(p, vec2(sincos.y, -sincos.x)), dot(p, sincos.xy)); \n"
        "}\n"

        // slow PCF sampling.
        "float oe_shadow_multisample(in vec3 c, in float refvalue, in float blur) \n"
        "{ \n"
        "    float shadowed = 0.0; \n"
        "    float a = 6.283185 * oe_shadow_rand(c.xy); \n"
        "    vec4 b = vec4(oe_shadow_rot(vec2(1,0),a), oe_shadow_rot(vec2(0,1),a)); \n"
        "    for(int i=0; i<OE_SHADOW_NUM_SAMPLES; ++i) { \n"
        "        vec2 off = oe_shadow_samples[i];\n"
        "        off = vec2(dot(off,b.xz), dot(off,b.yw)); \n"
        "        vec3 pc = vec3(c.xy + off*blur, c.z); \n"
        "        float depth = texture2DArray(oe_shadow_map, pc).r; \n"
        "        if ( depth < 1.0 && depth < refvalue ) { \n"
        "           shadowed += 1.0; \n"
        "        } \n"
        "    } \n"
        "    return 1.0-(shadowed/OE_SHADOW_NUM_SAMPLES); \n"
        "} \n"

        "void oe_shadow_fragment( inout vec4 color )\n"
        "{\n"
        "    float alpha = color.a; \n"
        "    float factor = 1.0; \n"

        // pre-pixel biasing to reduce moire/acne
        "    const float b0 = 0.001; \n"
        "    const float b1 = 0.01; \n"
        "    vec3 L = normalize(gl_LightSource[0].position.xyz); \n"
        "    vec3 N = normalize(oe_Normal); \n"
        "    float costheta = clamp(dot(L,N), 0.0, 1.0); \n"
        "    float bias = b0*tan(acos(costheta)); \n"

        // loop over the slices:
        "    for(int i=0; i<" << numSlices << " && factor > 0.0; ++i) \n"
        "    { \n"
        "        vec4 c = oe_shadow_coord[i]; \n"
        "        vec3 coord = vec3(c.x, c.y, float(i)); \n"

        "        if ( oe_shadow_blur > 0.0 ) \n"
        "        { \n"
        "            factor = min(factor, oe_shadow_multisample(coord, c.z-bias, oe_shadow_blur)); \n"
        "        } \n"
        "        else \n"
        "        { \n"
        "            float depth = texture2DArray(oe_shadow_map, coord).r; \n"
        "            if ( depth < 1.0 && depth < c.z-bias ) \n"
        "                factor = 0.0; \n"
        "        } \n"
        "    } \n"

        "    vec4 colorInFullShadow = color * oe_shadow_color; \n"
        "    color = mix(colorInFullShadow, color, factor); \n"
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

    // the texture coord generator matrix array (from the caster):
    _shadowMapTexGenUniform = _renderStateSet->getOrCreateUniform(
        "oe_shadow_matrix",
        osg::Uniform::FLOAT_MAT4,
        numSlices );

    // bind the shadow map texture itself:
    _renderStateSet->setTextureAttribute(
        _texImageUnit,
        _shadowmap.get(),
        osg::StateAttribute::ON );

    _renderStateSet->addUniform( new osg::Uniform("oe_shadow_map", _texImageUnit) );

    // blur factor:
    _shadowBlurUniform = _renderStateSet->getOrCreateUniform(
        "oe_shadow_blur",
        osg::Uniform::FLOAT);

    _shadowBlurUniform->set(_blurFactor);

    // shadow color:
    _shadowColorUniform = _renderStateSet->getOrCreateUniform(
        "oe_shadow_color",
        osg::Uniform::FLOAT_VEC4);

    _shadowColorUniform->set(_color);
}

void
ShadowCaster::traverse(osg::NodeVisitor& nv)
{
    if (_supported                             && 
        nv.getVisitorType() == nv.CULL_VISITOR && 
        _castingGroup->getNumChildren() > 0    && 
        _shadowmap.valid() )
    {
        osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);
        osg::Camera* camera = cv->getCurrentCamera();
        if ( camera )
        {
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
            
            //int i = nv.getFrameStamp()->getFrameNumber() % (_ranges.size()-1);
            int i;
            for(i=0; i < (int) _ranges.size()-1; ++i)
            {
                double n = _ranges[i];
                double f = _ranges[i+1];

                // take the camera's projection matrix and clamp it's near and far planes
                // to our shadow map slice range.
                osg::Matrix proj = _prevProjMatrix;
                //cv->clampProjectionMatrix(proj, n, f);
                double fovy,ar,zn,zf;
                proj.getPerspective(fovy,ar,zn,zf);
                proj.makePerspective(fovy,ar,std::max(n,zn),std::min(f,zf));
                
                // extract the corner points of the camera frustum in world space.
                osg::Matrix MVP = MV * proj;
                osg::Matrix inverseMVP;
                inverseMVP.invert(MVP);
                osgShadow::ConvexPolyhedron frustumPH;
                frustumPH.setToUnitFrustum(true, true);
                frustumPH.transform( inverseMVP, MVP );
                std::vector<osg::Vec3d> verts;
                frustumPH.getPoints( verts );

                // project those on to the plane of the light camera and fit them
                // to a bounding box. That box will form the extent of our orthographic camera.
                osg::BoundingBoxd bbox;
                for( std::vector<osg::Vec3d>::iterator v = verts.begin(); v != verts.end(); ++v )
                    bbox.expandBy( (*v) * lightViewMat );

                osg::Matrix lightProjMat;
                n = -std::max(bbox.zMin(), bbox.zMax());
                f = -std::min(bbox.zMin(), bbox.zMax());
                lightProjMat.makeOrtho(bbox.xMin(), bbox.xMax(), bbox.yMin(), bbox.yMax(), n, f);

                // configure the RTT camera for this slice:
                _rttCameras[i]->setViewMatrix( lightViewMat );
                _rttCameras[i]->setProjectionMatrix( lightProjMat );

                // this xforms from clip [-1..1] to texture [0..1] space
                static osg::Matrix s_scaleBiasMat = 
                    osg::Matrix::translate(1.0,1.0,1.0) * 
                    osg::Matrix::scale(0.5,0.5,0.5);
                
                // set the texture coordinate generation matrix that the shadow
                // receiver will use to sample the shadow map. Doing this on the CPU
                // prevents nasty precision issues!
                osg::Matrix VPS = lightViewMat * lightProjMat * s_scaleBiasMat;
                _shadowMapTexGenUniform->setElement(i, inverseMV * VPS);
            }

            // render the shadow maps.
            cv->pushStateSet( _rttStateSet.get() );
            for(i=0; i < (int) _rttCameras.size(); ++i)
            {
                _rttCameras[i]->accept( nv );
            }
            cv->popStateSet();
            
            // render the shadowed subgraph.
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
