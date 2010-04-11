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
#include <osgUtil/CullVisitor>

#include <list>
#include <sstream>

using namespace osgEarth;
using namespace osgEarthUtil;

char xyz_to_lat_lon_source[] =
"vec2 xyz_to_lat_lon(in vec3 xyz)\n"
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
"   return vec2(longitude, latitude);\n"
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
"varying vec2 texCoord2;\n"
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
"varying vec2 oceanMaskTexCoord;\n"
"\n"
"//Lighting\n"
"uniform bool osgEarth_lightingEnabled; \n"
"uniform bool osgEarth_light0Enabled; \n"
"uniform bool osgEarth_light1Enabled; \n"
"uniform bool osgEarth_light2Enabled; \n"
"uniform bool osgEarth_light3Enabled; \n"
"uniform bool osgEarth_light4Enabled; \n"
"uniform bool osgEarth_light5Enabled; \n"
"uniform bool osgEarth_light6Enabled; \n"
"uniform bool osgEarth_light7Enabled; \n"
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
"    if (osgEarth_light0Enabled) directionalLight(0, normal, ambient, diffuse, specular); \n"
"    if (osgEarth_light1Enabled) directionalLight(1, normal, ambient, diffuse, specular); \n"
"    if (osgEarth_light2Enabled) directionalLight(2, normal, ambient, diffuse, specular); \n"
"    if (osgEarth_light3Enabled) directionalLight(3, normal, ambient, diffuse, specular); \n"
"    if (osgEarth_light4Enabled) directionalLight(4, normal, ambient, diffuse, specular); \n"
"    if (osgEarth_light5Enabled) directionalLight(5, normal, ambient, diffuse, specular); \n"
"    if (osgEarth_light6Enabled) directionalLight(6, normal, ambient, diffuse, specular); \n"
"    if (osgEarth_light7Enabled) directionalLight(7, normal, ambient, diffuse, specular); \n"

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
"   float alpha = 0.0;\n"
"   if (osgEarth_oceanMaskUnitValid) alpha = texture2D( osgEarth_oceanMaskUnit, gl_MultiTexCoordOCEAN_MASK_UNIT.st).a;\n"
"   if (osgEarth_oceanInvertMask) alpha = 1.0 - alpha;\n"
"   if (osgEarth_oceanEnabled && alpha < 0.1)\n"
"   {\n"
"     const float PI_2 = 3.14158 * 2.0;\n"
"     const float period = PI_2/osgEarth_oceanPeriod;\n"
"     const float half_period = period / 2.0;\n"
"     mat4 modelMatrix = osg_ViewMatrixInverse * gl_ModelViewMatrix;\n"
"     vec4 vert = modelMatrix  * gl_Vertex;\n"           
"     vec3 vert3 = vec3(vert.x, vert.y, vert.z);\n"
"     vec2 latlon = xyz_to_lat_lon(vec3(vert.x, vert.y, vert.z));\n"
//"     vec3 latlon = osgEarth_LatLon;\n"
"     vec3 n = normalize(vert3);\n"
"     float theta = (mod(latlon.x, period) / period) * PI_2;\n"  
"     float phi = (mod(latlon.y, half_period) / half_period) * PI_2;\n"
"     float phase1 = osg_SimulationTime * 2.0;\n"
"     float phase2 = osg_SimulationTime * 4.0;\n"
"     float scale1 = sin(theta + phase1) * osgEarth_oceanHeight;\n"
"     float scale2 = cos(phi + phase2) * osgEarth_oceanHeight;\n"
"     float scale3 = sin(theta + phase2) * cos(phi + phase1) * osgEarth_oceanHeight *1.6;\n"
"     float scale = (scale1 + scale2 + scale3)/3.0;\n"
"     //if (scale < 0) scale = 0.0;\n"
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
"	texCoord0 = gl_MultiTexCoord0.st;\n"
"	texCoord1 = gl_MultiTexCoord1.st;\n"
"	texCoord2 = gl_MultiTexCoord2.st;\n"
"	texCoord3 = gl_MultiTexCoord3.st;\n"
"}\n";


typedef std::list<const osg::StateSet*> StateSetStack;
static osg::StateAttribute::GLModeValue getModeValue(const StateSetStack& statesetStack, osg::StateAttribute::GLMode mode)
{
    osg::StateAttribute::GLModeValue base_val = osg::StateAttribute::ON;
    for(StateSetStack::const_iterator itr = statesetStack.begin();
        itr != statesetStack.end();
        ++itr)
    {
        osg::StateAttribute::GLModeValue val = (*itr)->getMode(mode);
        if ((val & ~osg::StateAttribute::INHERIT)!=0)
        {
            if ((val & osg::StateAttribute::PROTECTED)!=0 ||
                (base_val & osg::StateAttribute::OVERRIDE)==0)
            {
                base_val = val;
            }
        }
    }
    return base_val;
}

OceanSurfaceNode::OceanSurfaceNode():
_oceanMaskTextureUnit(-1),
_waveHeight(100),
_currentElevation(0),
_maxRange(250000),
_period(1024),
_enabled(true),
_invertMask(false)
{
    _program = new osg::Program;
    _vertShader = new osg::Shader(osg::Shader::VERTEX);
    _fragShader = new osg::Shader(osg::Shader::FRAGMENT);

    rebuildShaders();

    _program->addShader( _vertShader.get() );
    _program->addShader( _fragShader.get() );

    getOrCreateStateSet()->setAttributeAndModes(_program.get(), osg::StateAttribute::ON);

    getOrCreateStateSet()->getOrCreateUniform("osgEarth_Layer0_unit", osg::Uniform::INT)->set(0);
    getOrCreateStateSet()->getOrCreateUniform("osgEarth_Layer1_unit", osg::Uniform::INT)->set(1);
    getOrCreateStateSet()->getOrCreateUniform("osgEarth_Layer2_unit", osg::Uniform::INT)->set(2);
    getOrCreateStateSet()->getOrCreateUniform("osgEarth_Layer3_unit", osg::Uniform::INT)->set(3);
    getOrCreateStateSet()->getOrCreateUniform("osgEarth_oceanEnabled", osg::Uniform::BOOL)->set(_enabled);     
    getOrCreateStateSet()->getOrCreateUniform("osgEarth_oceanPeriod", osg::Uniform::FLOAT)->set(_period);   
    getOrCreateStateSet()->getOrCreateUniform("osgEarth_oceanInvertMask", osg::Uniform::BOOL)->set(_invertMask);       

    osg::Uniform* oceanHeightUniform = getOrCreateStateSet()->getOrCreateUniform("osgEarth_oceanHeight", osg::Uniform::FLOAT);
    oceanHeightUniform->set( _waveHeight);
    oceanHeightUniform->setDataVariance( osg::Object::DYNAMIC);
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

double
OceanSurfaceNode::getMaxRange() const
{
    return _maxRange;
}

void
OceanSurfaceNode::setMaxRange(double maxRange)
{
    _maxRange = maxRange;
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
    _enabled = enabled;
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
                double latitude, longitude;
                em->convertXYZToLatLongHeight(x, y, z, latitude, longitude, _currentElevation);
            }
        }

        bool enableEffect = _currentElevation < _maxRange;
        getOrCreateStateSet()->getOrCreateUniform("osgEarth_oceanEnabled", osg::Uniform::BOOL)->set(enableEffect & _enabled);
        //OE_NOTICE << "Current elevation " << _currentElevation << ":  Enabled=" << enableEffect << std::endl;

        osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);
        if (cv)
        {
            StateSetStack statesetStack;

            osgUtil::StateGraph* sg = cv->getCurrentStateGraph();
            while(sg)
            {
                const osg::StateSet* stateset = sg->getStateSet();
                if (stateset)
                {
                    statesetStack.push_front(stateset);
                }                
                sg = sg->_parent;
            }

            osg::StateAttribute::GLModeValue lightingEnabled = getModeValue(statesetStack, GL_LIGHTING);     
            osg::Uniform* lightingEnabledUniform = getOrCreateStateSet()->getOrCreateUniform("osgEarth_lightingEnabled", osg::Uniform::BOOL);
            lightingEnabledUniform->set((lightingEnabled & osg::StateAttribute::ON)!=0);

            const unsigned int numLights = 8;
            for (unsigned int i = 0; i < numLights; ++i)
            {
                osg::StateAttribute::GLModeValue lightEnabled = getModeValue(statesetStack, GL_LIGHT0 + i);     
                std::stringstream ss;
                ss << "osgEarth_light" << i << "Enabled";
                osg::Uniform* lightEnabledUniform = getOrCreateStateSet()->getOrCreateUniform(ss.str(), osg::Uniform::BOOL);
                lightEnabledUniform->set((lightEnabled & osg::StateAttribute::ON)!=0);
            }
        }
    }
    osg::Group::traverse(nv);
}


void
OceanSurfaceNode::rebuildShaders()
{
    OE_DEBUG << "Rebuilding shaders..." << std::endl;
    std::string vertShaderSource = std::string(fnormal_source) + 
        std::string(directionalLight_source) + 
        std::string(xyz_to_lat_lon_source) +
        std::string(vert_shader_source);

    {
        unsigned int unit = _oceanMaskTextureUnit >= 0 ? _oceanMaskTextureUnit : 0;        
        std::string str = vertShaderSource;
        std::string toreplace = std::string("OCEAN_MASK_UNIT");
        std::size_t start = str.find(toreplace);
        std::stringstream ss;
        ss << unit;
        str.replace(start, toreplace.size(), ss.str());
        vertShaderSource = str;
        OE_DEBUG << "Shader " << str << std::endl;
    }

    _vertShader->setShaderSource( vertShaderSource );

    unsigned int numUnits = 4;
    std::stringstream ss;
    for (unsigned int i = 0; i < numUnits; ++i)
    {
        if (i != _oceanMaskTextureUnit)
        {
            ss << "varying vec2 texCoord" << i << ";" << std::endl;
            ss << "uniform sampler2D osgEarth_Layer" << i << "_unit;" << std::endl;
        }
    }

    ss << "void main ( void ) " << std::endl
        << "{" << std::endl;


    for (unsigned int i = 0; i < numUnits; ++i)
    {
        ss << "vec4 tex" << i << " = vec4(0.0,0.0,0.0,0.0);\n";
        if (i != _oceanMaskTextureUnit)
        {
            ss << "tex" << i << " = texture2D(osgEarth_Layer" << i << "_unit, texCoord" << i << ");" << std::endl;
        }
    }

    ss << "//Interpolate the color between the first layer and second" << std::endl
        << "vec3 c = mix(tex0.rgb, tex1.rgb, tex1.a);" << std::endl
        << "c = mix(c, tex2.rgb, tex2.a);" << std::endl
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

