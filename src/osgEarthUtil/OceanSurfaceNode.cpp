/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2009 Pelican Ventures, Inc.
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

#include <osgDB/ReadFile>

#include <osgUtil/CullVisitor>

#include <list>
#include <sstream>

using namespace osgEarth;
using namespace osgEarthUtil;

char xyz_to_lat_lon_height_source[] =
"vec3 xyz_to_lat_lon_height(in vec3 xyz)\n"
"{\n"
"  float X = xyz.x;\n"
"  float Y = xyz.y;\n"
"  float Z = xyz.z;\n"
"  float _radiusEquator = 6378137.0;\n"
"  float _radiusPolar   = 6356752.3142;\n"
"  float flattening = (_radiusEquator-_radiusPolar)/_radiusEquator;\n"
"  float _eccentricitySquared = 2*flattening - flattening*flattening;\n"
"  float p = sqrt(X*X + Y*Y);\n"
"  float theta = atan(Z*_radiusEquator , (p*_radiusPolar));\n"
"  float eDashSquared = (_radiusEquator*_radiusEquator - _radiusPolar*_radiusPolar)/(_radiusPolar*_radiusPolar);\n"
"  float sin_theta = sin(theta);\n"
"  float cos_theta = cos(theta);\n"
"\n"
"   float latitude = atan( (Z + eDashSquared*_radiusPolar*sin_theta*sin_theta*sin_theta), (p - _eccentricitySquared*_radiusEquator*cos_theta*cos_theta*cos_theta) );\n"
"   float longitude = atan(Y,X);\n"
"   float sin_latitude = sin(latitude);\n"
"   float N = _radiusEquator / sqrt( 1.0 - _eccentricitySquared*sin_latitude*sin_latitude);\n"
"   float height = p/cos(latitude) - N;\n"
"   return vec3(longitude, latitude, height);\n"
"}\n";



char fnormal_source[] = 
"vec3 fnormal(void)\n"
"{\n"
"    //Compute the normal \n"
"    vec3 normal = gl_NormalMatrix * gl_Normal;\n"
"    normal = normalize(normal);\n"
"    return normal;\n"
"}\n";

char directionalLight_source[] = 
"void directionalLight(in int i, \n"
"                      in vec3 normal, \n"
"                      inout vec4 ambient, \n"
"                      inout vec4 diffuse, \n"
"                      inout vec4 specular) \n"
"{ \n"
"   float nDotVP;         // normal . light direction \n"
"   float nDotHV;         // normal . light half vector \n"
"   float pf;             // power factor \n"
" \n"
"   nDotVP = max(0.0, dot(normal, normalize(vec3 (gl_LightSource[i].position)))); \n"
"   nDotHV = max(0.0, dot(normal, vec3 (gl_LightSource[i].halfVector))); \n"
" \n"
"   if (nDotVP == 0.0) \n"
"   { \n"
"       pf = 0.0; \n"
"   } \n"
"   else \n"
"   { \n"
"       pf = pow(nDotHV, gl_FrontMaterial.shininess); \n"
" \n"
"   } \n"
"   ambient  += gl_LightSource[i].ambient; \n"
"   diffuse  += gl_LightSource[i].diffuse * nDotVP; \n"
"   specular += gl_LightSource[i].specular * pf; \n"
"} \n";


char vert_shader_source[] =
//"in vec3 osgEarth_LatLon;\n"
"varying vec2 texCoord0;\n"
"varying vec2 texCoord1;\n"
"varying vec3 texCoord2;\n"
"varying vec2 texCoord3;\n"
"uniform float osg_SimulationTime; \n"
"uniform mat4 osg_ViewMatrixInverse;\n"
"uniform mat4 osg_ViewMatrix;\n"
"uniform float osgEarth_oceanHeight;\n"
"uniform float osgEarth_oceanPeriod;\n"
"uniform sampler2D osgEarth_oceanMaskUnit;\n"
"uniform bool osgEarth_oceanMaskUnitValid;\n"
"uniform bool osgEarth_oceanEnabled;\n"
"uniform bool osgEarth_oceanInvertMask;\n"
"uniform float osgEarth_oceanAnimationPeriod;\n"
"uniform float osgEarth_oceanMaxRange;\n"
"uniform float osgEarth_cameraElevation;\n"

"varying vec2 oceanMaskTexCoord;\n"
"varying float osgEarth_oceanAlpha;\n"
"\n"
"\n"
"//Lighting\n"
"uniform bool osgEarth_lightingEnabled; \n"
"uniform bool osgEarth_lightsEnabled[8];\n"
"\n"
"\n"
"void main (void)\n"
"{\n"
"  if (osgEarth_lightingEnabled)\n"
"  {\n"
"    vec4 ambient = vec4(0.0); \n"
"    vec4 diffuse = vec4(0.0); \n"
"    vec4 specular = vec4(0.0); \n"
" \n"
"    vec3 normal = fnormal(); \n"
" \n"
"    //Compute lighting\n"
"    for (int i = 0; i < 8; i++)\n"
"    {\n"
"      if (osgEarth_lightsEnabled[i]) directionalLight(i, normal, ambient, diffuse, specular); \n"
"    }\n"
"     gl_FrontColor =gl_FrontLightModelProduct.sceneColor + \n"
"                    ambient  * gl_FrontMaterial.ambient + \n"
"                    diffuse  * gl_FrontMaterial.diffuse + \n"
"                    specular * gl_FrontMaterial.specular; \n"
"  }\n"
"  else\n"
"  {\n"
"    gl_FrontColor = gl_Color;\n"
"  }\n"
" \n"
"\n"
"   mat4 modelMatrix = osg_ViewMatrixInverse * gl_ModelViewMatrix;\n"
"   vec4 vert = modelMatrix  * gl_Vertex;\n"           
"   vec3 vert3 = vec3(vert.x, vert.y, vert.z);\n"
"   vec3 latlon = xyz_to_lat_lon_height(vert3);\n"
"   osgEarth_oceanAlpha = 1.0;\n"
"   if (osgEarth_oceanMaskUnitValid) osgEarth_oceanAlpha = 1.0 - texture2D( osgEarth_oceanMaskUnit, gl_MultiTexCoordOCEAN_MASK_UNIT.st).a;\n"
"   if (osgEarth_oceanInvertMask) osgEarth_oceanAlpha = 1.0 - osgEarth_oceanAlpha;\n"
"   if (osgEarth_oceanMaxRange >= osgEarth_cameraElevation)\n"
"   {\n"
"     //Invert so it's between 0 and 1\n"
"     float s = mix(1.0, 0.0, osgEarth_cameraElevation / osgEarth_oceanMaxRange);\n"
"     osgEarth_oceanAlpha *= s;\n"
"   }\n"
"   else {\n"
"      osgEarth_oceanAlpha = 0.0;\n"
"   }\n"

"   if (osgEarth_oceanEnabled && osgEarth_oceanAlpha > 0.0)\n"
"   {\n"
"     float PI_2 = 3.14158 * 2.0;\n"
"     float period = PI_2/osgEarth_oceanPeriod;\n"
"     float half_period = period / 2.0;\n"
//"     vec3 latlon = osgEarth_LatLon;\n"
"     vec3 n = normalize(vert3);\n"
"     float theta = (mod(latlon.x, period) / period) * PI_2;\n"  
"     float phi = (mod(latlon.y, half_period) / half_period) * PI_2;\n"
"     float phase1 = osg_SimulationTime * 2.0;\n"
"     float phase2 = osg_SimulationTime * 4.0;\n"
"     float waveHeight = (osgEarth_oceanAlpha) * osgEarth_oceanHeight;\n"
"     float scale1 = sin(theta + phase1) * waveHeight;\n"
"     float scale2 = cos(phi + phase2) * waveHeight;\n"
"     float scale3 = sin(theta + phase2) * cos(phi + phase1) * waveHeight *1.6;\n"
"     float scale = (scale1 + scale2 + scale3)/3.0;\n"
"     //if (scale < 0) scale = 0.0;\n"
"   #if OCEAN_ADJUST_TO_MSL \n"
"     //Adjust the vert to 0 MSL\n"
"     vec3 offset = n * -latlon.z;\n"
"     vert += vec4(offset.x, offset.y, offset.z, 0);\n"
"   #endif\n"
"     //Apply the wave scale\n"
"     n = n * scale;\n"
"     vert += vec4(n.x, n.y,n.z,0);\n"
"     vert = osg_ViewMatrix * vert;\n"
"     gl_Position = gl_ProjectionMatrix * vert;\n"
"   }\n"
"   else\n"
"   {\n"
"     gl_Position = ftransform();\n"
"   }\n"
"\n"
"   float PI = 3.14158;\n"
"   float textureSize = OCEAN_TEXTURE_SIZE;\n"
"	texCoord0 = gl_MultiTexCoord0.st;\n"
"	texCoord1 = gl_MultiTexCoord1.st;\n"
"	texCoord2.xy = gl_MultiTexCoord1.st;\n"
"	texCoord3 = gl_MultiTexCoord3.st;\n"
"   #if OCEAN_SURFACE_UNIT >= 0\n"
"   //Alter the z (r) coordinate based on the animation period\n"
"   texCoordOCEAN_SURFACE_UNIT.x =  latlon.x / textureSize;\n"
"   texCoordOCEAN_SURFACE_UNIT.y =  latlon.y / textureSize;\n"
"   texCoordOCEAN_SURFACE_UNIT.z = fract(osg_SimulationTime/osgEarth_oceanAnimationPeriod);\n"
"   #endif\n"

"}\n";

typedef std::vector< osg::ref_ptr< osg::Image > > ImageList;

OceanSurfaceNode::OceanSurfaceNode():
_oceanMaskTextureUnit(-1),
_oceanSurfaceTextureUnit(2),
_waveHeight(100),
_currentElevation(FLT_MAX),
_maxRange(800000),
_period(1024),
_enabled(true),
_invertMask(false),
_adjustToMSL(true),
_oceanAnimationPeriod(6.0),
_oceanSurfaceImageSizeRadians(osg::PI/500.0),
_oceanColor(osg::Vec4f(0,0,1,0))
{
    _program = new osg::Program;
    _vertShader = new osg::Shader(osg::Shader::VERTEX);
    _fragShader = new osg::Shader(osg::Shader::FRAGMENT);

    rebuildShaders();

    _program->addShader( _vertShader.get() );
    _program->addShader( _fragShader.get() );

    getOrCreateStateSet()->setAttributeAndModes(_program.get(), osg::StateAttribute::ON);

    getOrCreateStateSet()->getOrCreateUniform("osgEarth_oceanEnabled", osg::Uniform::BOOL)->set(_enabled);     
    getOrCreateStateSet()->getOrCreateUniform("osgEarth_oceanPeriod", osg::Uniform::FLOAT)->set(_period);   
    getOrCreateStateSet()->getOrCreateUniform("osgEarth_oceanInvertMask", osg::Uniform::BOOL)->set(_invertMask); 
    getOrCreateStateSet()->getOrCreateUniform("osgEarth_oceanAnimationPeriod", osg::Uniform::FLOAT)->set(_oceanAnimationPeriod); 
    getOrCreateStateSet()->getOrCreateUniform("osgEarth_cameraElevation", osg::Uniform::FLOAT)->set(_currentElevation);
    getOrCreateStateSet()->getOrCreateUniform("osgEarth_oceanMaxRange", osg::Uniform::FLOAT)->set(_maxRange);

    osg::Uniform* oceanHeightUniform = getOrCreateStateSet()->getOrCreateUniform("osgEarth_oceanHeight", osg::Uniform::FLOAT);
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

int
OceanSurfaceNode::getOceanMaskTextureUnit() const
{
    return _oceanMaskTextureUnit;
}

void
OceanSurfaceNode::setOceanMaskTextureUnit(int unit)
{
    if (_oceanMaskTextureUnit != unit)
    {
        _oceanMaskTextureUnit = unit;
        rebuildShaders();
    }
}

int
OceanSurfaceNode::getOceanSurfaceTextureUnit() const
{
    return _oceanSurfaceTextureUnit;
}

void
OceanSurfaceNode::setOceanSurfaceTextureUnit(int unit)
{
    if (_oceanSurfaceTextureUnit != unit)
    {
        //Remove the previous texture
        if (_oceanSurfaceTextureUnit >= 0)
        {
            getOrCreateStateSet()->setTextureAttributeAndModes(_oceanSurfaceTextureUnit, NULL, osg::StateAttribute::OFF);
        }

        //Set the new unit
        _oceanSurfaceTextureUnit = unit;

        //Set the texture on the new unit
        if (_oceanSurfaceTextureUnit >= 0 && _oceanSurfaceImage.valid())
        {
            getOrCreateStateSet()->setTextureAttributeAndModes(_oceanSurfaceTextureUnit, _oceanSurfaceTexture, osg::StateAttribute::ON);
        }
        rebuildShaders();
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
		rebuildShaders();
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
        getOrCreateStateSet()->setTextureAttributeAndModes(_oceanSurfaceTextureUnit, _oceanSurfaceTexture.get(), osg::StateAttribute::ON);
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
        getOrCreateStateSet()->getOrCreateUniform("osgEarth_oceanHeight", osg::Uniform::FLOAT)->set(_waveHeight);
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
        getOrCreateStateSet()->getOrCreateUniform("osgEarth_oceanMaxRange", osg::Uniform::FLOAT)->set(_maxRange);
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
        getOrCreateStateSet()->getOrCreateUniform("osgEarth_oceanPeriod", osg::Uniform::FLOAT)->set(_period);     
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
        getOrCreateStateSet()->getOrCreateUniform("osgEarth_oceanEnabled", osg::Uniform::BOOL)->set(_enabled);
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
        getOrCreateStateSet()->getOrCreateUniform("osgEarth_oceanInvertMask", osg::Uniform::BOOL)->set(_invertMask);   
        rebuildShaders();
    }
}

void
OceanSurfaceNode::setModulationColor( const osg::Vec4f& color )
{
    if ( !_oceanColor.isSetTo( color ) )
    {
        _oceanColor = color;
        rebuildShaders();
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
        getOrCreateStateSet()->getOrCreateUniform("osgEarth_oceanAnimationPeriod", osg::Uniform::FLOAT)->set(oceanAnimationPeriod); 
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
        rebuildShaders();
    }
}


void 
OceanSurfaceNode::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
    {
        //Get the current elevation
        _currentElevation = nv.getViewPoint().z();

        if (!_csn.valid())
        {
            //Try to find a coordiante system node above this node
            _csn = findFirstParentOfType<osg::CoordinateSystemNode>(this);
        }
        if (_csn.valid())
        {
            osg::EllipsoidModel* em = _csn->getEllipsoidModel();
            if (em)
            {
                double x = nv.getViewPoint().x();
                double y = nv.getViewPoint().y();
                double z = nv.getViewPoint().z();
                double latitude, longitude, elevation;
                em->convertXYZToLatLongHeight(x, y, z, latitude, longitude, elevation);
                _currentElevation = (float)elevation;
            }
        }

        //bool enableEffect = _currentElevation < _maxRange;
        getOrCreateStateSet()->getOrCreateUniform("osgEarth_cameraElevation", osg::Uniform::FLOAT)->set(_currentElevation);
    }
    osg::Group::traverse(nv);
}

static std::string replaceAll(const std::string &input, const std::string &toreplace, const std::string &replacewith)
{
    std::string str = input;
    std::size_t start = 0;
    while (true)
    {
        start = str.find(toreplace, start);
        if (start == str.npos) break;
        str.replace(start, toreplace.size(), replacewith);
        start += replacewith.size();
    }
    return str;
}


void
OceanSurfaceNode::rebuildShaders()
{
    OE_DEBUG << "Rebuilding shaders..." << std::endl;
    std::string vertShaderSource = std::string(fnormal_source) + 
        std::string(directionalLight_source) + 
        std::string(xyz_to_lat_lon_height_source) +
        std::string(vert_shader_source);

    {
        unsigned int maskUnit = _oceanMaskTextureUnit >= 0 ? _oceanMaskTextureUnit : 0;        
        std::stringstream ss;
        ss << maskUnit;
        vertShaderSource = replaceAll(vertShaderSource, "OCEAN_MASK_UNIT", ss.str());
        
        int surfaceUnit = _oceanSurfaceTextureUnit >= 0 ? _oceanSurfaceTextureUnit : 0;        
        ss.str("");
        ss << surfaceUnit;
        vertShaderSource = replaceAll(vertShaderSource, "OCEAN_SURFACE_UNIT", ss.str());

        ss.str("");
        ss << _oceanSurfaceImageSizeRadians;
        vertShaderSource = replaceAll(vertShaderSource, "OCEAN_TEXTURE_SIZE", ss.str());

		ss.str("");
        ss << _adjustToMSL;
        vertShaderSource = replaceAll(vertShaderSource, "OCEAN_ADJUST_TO_MSL", ss.str());
        
        OE_DEBUG << "Shader " << vertShaderSource << std::endl;
    }

    _vertShader->setShaderSource( vertShaderSource );

    unsigned int numUnits = 4;
    std::stringstream ss;
    for (unsigned int i = 0; i < numUnits; ++i)
    {
        //if (i != _oceanMaskTextureUnit)
        if (i != _oceanSurfaceTextureUnit)
        {
            ss << "varying vec2 texCoord" << i << ";" << std::endl;
            ss << "uniform sampler2D osgEarth_Layer" << i << "_unit;" << std::endl;
        }
        else
        {
            ss << "varying vec3 texCoord" << i << ";" << std::endl;
            ss << "uniform sampler3D osgEarth_Layer" << i << "_unit;" << std::endl;
        }
    }

    if ( _oceanColor.isSet() )
    {
        ss << "uniform bool osgEarth_oceanInvertMask;\n";
    }

    ss << "uniform bool osgEarth_oceanEnabled;\n";
    ss << "varying float osgEarth_oceanAlpha;\n";

    ss << "void main ( void ) " << std::endl
        << "{" << std::endl;

    if ( _oceanColor.isSet() )
    {
        for(unsigned int i=0; i < numUnits; ++i)
        {
            ss << "vec4 tex" << i << ";\n";
            if (i == _oceanSurfaceTextureUnit)
            {
              ss << "tex" << i << " =  texture3D(osgEarth_Layer" << i << "_unit, texCoord" << i << ");\n";
              ss << "if (osgEarth_oceanEnabled && osgEarth_oceanAlpha > 0) { " << std::endl
                     << "  tex" << i << ".a *= osgEarth_oceanAlpha;" << std::endl
                 << "}" << std::endl
                 << " else { tex" << i << ".a = 0.0; }" << std::endl;
            }
            else
            {
              ss << "tex" << i << " =  texture2D(osgEarth_Layer" << i << "_unit, texCoord" << i << ");\n";
            }

            if ( i == _oceanMaskTextureUnit )
            {
                ss << "float maskAlpha = 1.0 - tex" << i << ".a;\n";
                ss << "if(osgEarth_oceanInvertMask) maskAlpha = 1.0-maskAlpha;\n";                
                ss << "tex" << i << " = vec4("
                    << _oceanColor->r() << "," << _oceanColor->g() << ","
                    << _oceanColor->b() << ",maskAlpha*" << _oceanColor->a() << ");\n";
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < numUnits; ++i)
        {
            ss << "vec4 tex" << i << " = vec4(0.0,0.0,0.0,0.0);\n";
            if (i != _oceanMaskTextureUnit)
            {
                if (i == _oceanSurfaceTextureUnit)
                {
                  ss << "tex" << i << " = texture3D(osgEarth_Layer" << i << "_unit, texCoord" << i << ");" << std::endl;
                  ss << "if (osgEarth_oceanEnabled && osgEarth_oceanAlpha > 0) { " << std::endl
                     << "  tex" << i << ".a *= osgEarth_oceanAlpha;" << std::endl
                     << "}" << std::endl
                     << " else { tex" << i << ".a = 0.0; }" << std::endl;
                }
                else
                {
                  ss << "tex" << i << " = texture2D(osgEarth_Layer" << i << "_unit, texCoord" << i << ");" << std::endl;
                }
            }
        }
    }

    ss << "//Interpolate the color between the first layer and second" << std::endl
        << "vec3 c = mix(tex0.rgb, tex1.rgb, tex1.a);" << std::endl
        //<< "if (osgEarth_oceanEnabled && osgEarth_oceanAlpha > 0){ " << std::endl
        << "c = mix(c, tex2.rgb, tex2.a);" << std::endl
        //<< "}" << std::endl
        << "c = mix(c, tex3.rgb, tex3.a);" << std::endl
        << "float a = tex0.a;" << std::endl
        << "if (a < tex1.a) a = tex1.a;" << std::endl
        << "if (a < tex2.a) a = tex2.a;" << std::endl
        << "if (a < tex3.a) a = tex3.a;" << std::endl
        << "gl_FragColor = gl_Color * vec4(c, a);" << std::endl;
    ss << "}";


    std::string fragShaderSource = ss.str();
    //OE_NOTICE << "Fragment Shader " << std::endl << fragShaderSource << std::endl;
    _fragShader->setShaderSource( fragShaderSource );  

    getOrCreateStateSet()->getOrCreateUniform("osgEarth_oceanMaskUnitValid", osg::Uniform::BOOL)->set(_oceanMaskTextureUnit > 0);
    int unit = _oceanMaskTextureUnit >= 0 ? _oceanMaskTextureUnit : 0;
    getOrCreateStateSet()->getOrCreateUniform("osgEarth_oceanMaskUnit", osg::Uniform::BOOL)->set(unit);
}

