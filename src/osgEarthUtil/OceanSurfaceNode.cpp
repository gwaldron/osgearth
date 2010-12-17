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

#include <osgEarthUtil/OceanSurfaceNode>

#include <osgEarth/FindNode>
#include <osgEarth/Notify>
#include <osgEarth/Registry>
#include <osgEarth/ShaderComposition>
#include <osgEarth/MapNode>
#include <osgEarth/FindNode>

#include <osg/Texture3D>
#include <osgDB/ReadFile>

#include <sstream>
#include <iomanip>

#define LC "[OceanSurfaceNode] "

using namespace osgEarth;
using namespace osgEarth::Util;

typedef std::vector< osg::ref_ptr< osg::Image > > ImageList;

OceanSurfaceNode::OceanSurfaceNode() :
_shadersDirty(false),
_oceanMaskLayerUID(-1),
_oceanSurfaceTextureUnit(-1),
_waveHeight(100),
_maxRange(800000),
_period(1024),
_enabled(true),
_invertMask(false),
_adjustToMSL(true),
_oceanAnimationPeriod(6.0),
_oceanSurfaceImageSizeRadians(osg::PI/500.0),
_oceanColor(osg::Vec4f(0,0,1,0)),
_oceanSurfaceTextureApplied(false)
{
    rebuildShaders();
 
    getOrCreateStateSet()->getOrCreateUniform("osgearth_OceanPeriod", osg::Uniform::FLOAT)->set(_period);   
    getOrCreateStateSet()->getOrCreateUniform("osgearth_OceanAnimationPeriod", osg::Uniform::FLOAT)->set(_oceanAnimationPeriod); 

    osg::Uniform* oceanHeightUniform = getOrCreateStateSet()->getOrCreateUniform("osgearth_OceanHeight", osg::Uniform::FLOAT);
    oceanHeightUniform->set( _waveHeight);
    oceanHeightUniform->setDataVariance( osg::Object::DYNAMIC);

    //Initialize the ocean surface texture
    _oceanSurfaceTexture = new osg::Texture3D();
    _oceanSurfaceTexture->setWrap(osg::Texture::WRAP_S,osg::Texture::REPEAT);
    _oceanSurfaceTexture->setWrap(osg::Texture::WRAP_T,osg::Texture::REPEAT);
    _oceanSurfaceTexture->setWrap(osg::Texture::WRAP_R,osg::Texture::REPEAT);
    _oceanSurfaceTexture->setFilter(osg::Texture3D::MIN_FILTER,osg::Texture3D::LINEAR);
    _oceanSurfaceTexture->setFilter(osg::Texture3D::MAG_FILTER,osg::Texture3D::LINEAR);
}

void
OceanSurfaceNode::shadersDirty(bool value)
{
    if ( _shadersDirty != value )
    {
        _shadersDirty = value;
        ADJUST_UPDATE_TRAV_COUNT( this, _shadersDirty ? 1 : -1 );
    }
}

void
OceanSurfaceNode::setOceanMaskImageLayer( const ImageLayer* layer )
{
    if ( _maskLayer.get() != layer )
    {
        _maskLayer = layer;
        shadersDirty(true);
    }
}

bool
OceanSurfaceNode::getAdjustToMSL() const
{
	return _adjustToMSL;
}

void
OceanSurfaceNode::setAdjustToMSL(bool adjustToMSL)
{
	if (_adjustToMSL != adjustToMSL)
	{
		_adjustToMSL = adjustToMSL;
        shadersDirty( true );
	}
}

osg::Image*
OceanSurfaceNode::getOceanSurfaceImage() const
{
    return _oceanSurfaceImage.get();
}

void
OceanSurfaceNode::setOceanSurfaceImage(osg::Image* image)
{
    if (_oceanSurfaceImage.get() != image)
    {
        _oceanSurfaceImage = image;
        _oceanSurfaceTexture->setImage( _oceanSurfaceImage.get() );
        
        shadersDirty( true );
    }
}

float
OceanSurfaceNode::getWaveHeight() const
{
    return _waveHeight;
}

void
OceanSurfaceNode::setWaveHeight(float waveHeight)
{
    if (_waveHeight != waveHeight)
    {
        _waveHeight = waveHeight;
        getOrCreateStateSet()->getOrCreateUniform("osgearth_OceanHeight", osg::Uniform::FLOAT)->set(_waveHeight);
        //TODO: consider rebuildShaders() instead..
    }
}

float
OceanSurfaceNode::getMaxRange() const
{
    return _maxRange;
}

void
OceanSurfaceNode::setMaxRange(float maxRange)
{
    if (_maxRange != maxRange)
    {
        _maxRange = maxRange;
        shadersDirty(true);
    }
}

float
OceanSurfaceNode::getPeriod() const
{
    return _period;
}

void
OceanSurfaceNode::setPeriod(float period)
{
    if (_period !=period)
    {
        _period = period;
        getOrCreateStateSet()->getOrCreateUniform("osgearth_OceanPeriod", osg::Uniform::FLOAT)->set(_period); 
        //TODO: consider rebuildShaders() instead..    
    }
}

bool
OceanSurfaceNode::getEnabled() const
{
    return _enabled;
}

void
OceanSurfaceNode::setEnabled(bool enabled)
{
    if (_enabled != enabled)
    {
        _enabled = enabled;
        shadersDirty(true);
    }
}

bool
OceanSurfaceNode::getInvertMask() const
{
    return _invertMask;
}

void
OceanSurfaceNode::setInvertMask(bool invertMask)
{
    if (_invertMask != invertMask)
    {
        _invertMask = invertMask;
        shadersDirty( true );
    }
}

void
OceanSurfaceNode::setModulationColor( const osg::Vec4f& color )
{
    if ( !_oceanColor.isSetTo( color ) )
    {
        _oceanColor = color;
        shadersDirty( true );
    }
}

osg::Vec4f
OceanSurfaceNode::getModulationColor() const
{
    return _oceanColor.value();
}

float
OceanSurfaceNode::getOceanAnimationPeriod() const
{
    return _oceanAnimationPeriod;
}

void
OceanSurfaceNode::setOceanAnimationPeriod(float oceanAnimationPeriod)
{
    if (_oceanAnimationPeriod != oceanAnimationPeriod)
    {
        _oceanAnimationPeriod = oceanAnimationPeriod;
        getOrCreateStateSet()->getOrCreateUniform("osgearth_OceanAnimationPeriod", osg::Uniform::FLOAT)->set(oceanAnimationPeriod); 
        //TODO: consider rebuildShaders() instead..
    }
}

float
OceanSurfaceNode::getOceanSurfaceImageSizeRadians() const
{
    return _oceanSurfaceImageSizeRadians;
}

void
OceanSurfaceNode::setOceanSurfaceImageSizeRadians(float size)
{
    if (_oceanSurfaceImageSizeRadians != size)
    {
        _oceanSurfaceImageSizeRadians = size;
        shadersDirty( true );
    }
}

void
OceanSurfaceNode::traverse( osg::NodeVisitor& nv )
{
    if ( _shadersDirty && nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
    {
        rebuildShaders();
        shadersDirty( false );
    }

    osg::Group::traverse( nv );
}

#define MASK_SAMPLER_FUNC "osgearth_ocean_sampleMask"

void
OceanSurfaceNode::rebuildShaders()
{
    // need the terrain engine so we can get at the compositor.
    TerrainEngineNode* engine = osgEarth::findTopMostNodeOfType<TerrainEngineNode>( this );
    if ( !engine ) {
        OE_DEBUG << LC << "No terrain engine found in the map node; abort" << std::endl;
        return;
    }

    // access the compositor because we are going to be sampling map layers.
    TextureCompositor* comp = engine->getTextureCompositor();
    if ( !comp ) {
        OE_INFO << LC << "No texture compositor found in the terrain engine; abort" << std::endl;
        return;
    }

    // reserve a texture unit for the surface texture (if we haven't already)
    if ( !_oceanSurfaceTextureApplied && _oceanSurfaceTextureUnit < 0 && _oceanSurfaceTexture.valid() )
    {
        if ( comp->reserveTextureImageUnit( _oceanSurfaceTextureUnit ) )
        {
            getOrCreateStateSet()->setTextureAttributeAndModes(
                _oceanSurfaceTextureUnit, _oceanSurfaceTexture.get(), osg::StateAttribute::ON);
            _oceanSurfaceTextureApplied = true;
        }
        else
        {
            OE_WARN << LC << "Sorry, failed to allocate a texture image unit for the surface texture." << std::endl;
        }
    }

    // create a VP to store our custom shader components.
    osgEarth::VirtualProgram* vp = new osgEarth::VirtualProgram();
    getOrCreateStateSet()->setAttributeAndModes( vp, osg::StateAttribute::ON );

    // if the ocean is disabled, just return without injecting any shaders.
    if ( !_enabled )
        return;

    // build the sampler function if necessary
    osg::ref_ptr<const ImageLayer> safeMaskLayer = _maskLayer.get();
    osg::Shader* maskSampler = 0L;
    if ( safeMaskLayer.valid() )
    {
        maskSampler = comp->createSamplerFunction( safeMaskLayer->getUID(), MASK_SAMPLER_FUNC, osg::Shader::VERTEX );        
        if ( maskSampler )
            vp->setShader( MASK_SAMPLER_FUNC, maskSampler );
    }

    // make the helper functions.
    {
        std::stringstream buf;

        buf << "vec3 xyz_to_lat_lon_height(in vec3 xyz) \n"
            << "{ \n"
            << "    float X = xyz.x;\n"
            << "    float Y = xyz.y;\n"
            << "    float Z = xyz.z;\n"
            << "    float _radiusEquator = 6378137.0;\n"
            << "    float _radiusPolar   = 6356752.3142;\n"
            << "    float flattening = (_radiusEquator-_radiusPolar)/_radiusEquator;\n"
            << "    float _eccentricitySquared = 2.0*flattening - flattening*flattening;\n"
            << "    float p = sqrt(X*X + Y*Y);\n"
            << "    float theta = atan(Z*_radiusEquator , (p*_radiusPolar));\n"
            << "    float eDashSquared = (_radiusEquator*_radiusEquator - _radiusPolar*_radiusPolar)/(_radiusPolar*_radiusPolar);\n"
            << "    float sin_theta = sin(theta);\n"
            << "    float cos_theta = cos(theta);\n"
            << "    float latitude = atan( (Z + eDashSquared*_radiusPolar*sin_theta*sin_theta*sin_theta), (p - _eccentricitySquared*_radiusEquator*cos_theta*cos_theta*cos_theta) );\n"
            << "    float longitude = atan(Y,X);\n"
            << "    float sin_latitude = sin(latitude);\n"
            << "    float N = _radiusEquator / sqrt( 1.0 - _eccentricitySquared*sin_latitude*sin_latitude);\n"
            << "    float height = p/cos(latitude) - N;\n"
            << "    return vec3(longitude, latitude, height);\n"
            << "} \n"
            << "\n";

        std::string str = buf.str();
        vp->setShader( "xyz_to_lat_lon_height", new osg::Shader(osg::Shader::VERTEX, str) );
    }

    // next make the vertex shader function that will morph the ocean verts and prepare
    // the texture coordinates for the surface effects.
    {
        std::stringstream buf;

        buf << std::fixed;

        buf << "uniform float osg_SimulationTime; \n"
            << "uniform mat4  osg_ViewMatrixInverse;\n"
            << "uniform mat4  osg_ViewMatrix;\n"
            << "uniform float osgearth_OceanHeight;\n"
            << "uniform float osgearth_OceanPeriod;\n"
            << "uniform float osgearth_OceanAnimationPeriod;\n"
            << "varying float osgearth_OceanAlpha;\n"
            << "varying float osgearth_CameraRange; \n"

            << "vec3 xyz_to_lat_lon_height(in vec3 xyz); \n";

        if ( _oceanSurfaceTextureApplied )
        {
            buf << "varying vec3 osgearth_oceanSurfaceTexCoord; \n";
        }

        if ( maskSampler )
        {
            buf << "vec4 " << MASK_SAMPLER_FUNC << "(); \n";
        }

        buf << "void osgearth_ocean_morphSurface() \n"
            << "{ \n"
            << "   mat4 modelMatrix = osg_ViewMatrixInverse * gl_ModelViewMatrix; \n"
            << "   vec4 vert = modelMatrix  * gl_Vertex; \n"           
            << "   vec3 vert3 = vec3(vert.x, vert.y, vert.z); \n"
            << "   vec3 latlon = xyz_to_lat_lon_height(vert3); \n"
            << "   osgearth_OceanAlpha = 1.0; \n";

        if ( maskSampler )
        {
            buf << "    osgearth_OceanAlpha = 1.0 - (" << MASK_SAMPLER_FUNC << "()).a; \n";
        }

        if ( _invertMask )
            buf << "    osgearth_OceanAlpha = 1.0 - osgearth_OceanAlpha; \n";

        buf << "   if ( osgearth_CameraRange <= " << _maxRange << " ) \n"
            << "   { \n"
            << "       float s = mix(1.0, 0.0, osgearth_CameraRange / " << _maxRange << "); \n" //Invert so it's between 0 and 1
            << "       osgearth_OceanAlpha *= s; \n"
            << "   } \n"
            << "   else \n"
            << "   { \n"
            << "        osgearth_OceanAlpha = 0.0; \n"
            << "   } \n"

            << "   if (osgearth_OceanAlpha > 0.0) \n"
            << "   { \n"
            << "       float PI_2 = 3.14158 * 2.0; \n"
            << "       float period = PI_2/osgearth_OceanPeriod; \n"
            << "       float half_period = period / 2.0; \n"
            << "       vec3 n = normalize(vert3);\n"
            << "       float theta = (mod(latlon.x, period) / period) * PI_2; \n"  
            << "       float phi = (mod(latlon.y, half_period) / half_period) * PI_2; \n"
            << "       float phase1 = osg_SimulationTime * 2.0; \n"
            << "       float phase2 = osg_SimulationTime * 4.0; \n"
            << "       float waveHeight = (osgearth_OceanAlpha) * osgearth_OceanHeight; \n"
            << "       float scale1 = sin(theta + phase1) * waveHeight; \n"
            << "       float scale2 = cos(phi + phase2) * waveHeight; \n"
            << "       float scale3 = sin(theta + phase2) * cos(phi + phase1) * waveHeight * 1.6; \n"
            << "       float scale = (scale1 + scale2 + scale3)/3.0; \n";

        // flatten verts to MSL:
        if ( _adjustToMSL )
        {
            buf << "        vec3 offset = n * -latlon.z; \n"
                << "        vert += vec4( offset.xyz, 0 ); \n";
        }

        // apply the save scale:
        buf << "       n = n * scale; \n"
            << "       vert += vec4(n.x, n.y,n.z,0); \n"
            << "       vert = osg_ViewMatrix * vert; \n"
            << "       gl_Position = gl_ProjectionMatrix * vert; \n"
            << "   }\n";

        // set up the coords for the surface texture:
        if ( _oceanSurfaceTextureApplied )
        {
            buf << "   osgearth_oceanSurfaceTexCoord.x =  latlon.x / " << _oceanSurfaceImageSizeRadians << "; \n"
                << "   osgearth_oceanSurfaceTexCoord.y =  latlon.y / " << _oceanSurfaceImageSizeRadians << "; \n"
                << "   osgearth_oceanSurfaceTexCoord.z = fract(osg_SimulationTime/osgearth_OceanAnimationPeriod); \n";
        }

        buf << "}\n";

        // add as a custom user function in the shader composition:
        std::string vertSource = buf.str();
        vp->setFunction( "osgearth_ocean_morphSurface", vertSource, osgEarth::ShaderComp::LOCATION_VERTEX_PRE_TEXTURING );
    }
    
    // now we need a fragment function that will apply the ocean surface texture.
    if ( _oceanSurfaceTextureApplied )
    {
        getOrCreateStateSet()->getOrCreateUniform( "osgearth_oceanSurfaceTex", osg::Uniform::SAMPLER_3D )->set( _oceanSurfaceTextureUnit );

        std::stringstream buf;

        buf << "uniform sampler3D osgearth_oceanSurfaceTex; \n"
            << "varying vec3      osgearth_oceanSurfaceTexCoord; \n"
            << "varying float     osgearth_OceanAlpha; \n"

            << "void osgearth_ocean_applySurfaceTex( inout vec4 color ) \n"
            << "{ \n"
            << "    vec4 texel = texture3D(osgearth_oceanSurfaceTex, osgearth_oceanSurfaceTexCoord); \n"
            << "    color = vec4( mix( color.rgb, texel.rgb, texel.a * osgearth_OceanAlpha ), color.a); \n"
            << "} \n";

        std::string str = buf.str();
        vp->setFunction( "osgearth_ocean_applySurfaceTex", str, osgEarth::ShaderComp::LOCATION_FRAGMENT_PRE_LIGHTING );
    }
}

