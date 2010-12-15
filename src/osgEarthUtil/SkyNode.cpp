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
#include <osgEarthUtil/SkyNode>

#include <osgEarth/ShaderComposition>
#include <osgEarth/FindNode>
#include <osgEarth/MapNode>

#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/BlendFunc>
#include <osg/FrontFace>
#include <osg/Program>
#include <osg/Shape>
#include <osg/Depth>

#include <sstream>

#define LC "[SkyNode] "

using namespace osgEarth;
using namespace osgEarth::Util;


//---------------------------------------------------------------------------

namespace
{
    // a cull callback that prevents objects from being included in the near/fear clip
    // plane calculates that OSG does. This is useful for including "distant objects"
    struct DoNotIncludeInNearFarComputationCallback : public osg::NodeCallback
    {
        virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
        {
            osgUtil::CullVisitor *cv = dynamic_cast< osgUtil::CullVisitor*>( nv );

            // Default value
            osg::CullSettings::ComputeNearFarMode oldMode =
                osg::CullSettings::COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES;

            if( cv )
            {
                oldMode = cv->getComputeNearFarMode();
                cv->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );
            }

            traverse(node, nv);

            if( cv )
            {
                cv->setComputeNearFarMode(oldMode);
            }
        }
    };

    struct OverrideNearFarValuesCallback : public osg::Drawable::DrawCallback
    {
        OverrideNearFarValuesCallback(double radius)
            : radius_(radius) {}

        virtual void drawImplementation(osg::RenderInfo& renderInfo,
            const osg::Drawable* drawable) const
        {
            osg::Camera* currentCamera = renderInfo.getCurrentCamera();
            if (currentCamera)
            {
                // Get the current camera position.
                osg::Vec3 eye, center, up;
                renderInfo.getCurrentCamera()->getViewMatrixAsLookAt( eye, center, up);

                // Get the max distance we need the far plane to be at,
                // which is the distance between the eye and the origin
                // plus the distant from the origin to the object (star sphere
                // radius, sun distance etc).
                double distance = eye.length() + radius_;

                // Save old values.
                osg::ref_ptr<osg::RefMatrixd> oldProjectionMatrix = new osg::RefMatrix;
                oldProjectionMatrix->set( renderInfo.getState()->getProjectionMatrix());

                // Get the individual values
                double left, right, bottom, top, zNear, zFar;
                oldProjectionMatrix->getFrustum( left, right, bottom, top, zNear, zFar);

                // Build a new projection matrix with a modified far plane
                osg::ref_ptr<osg::RefMatrixd> projectionMatrix = new osg::RefMatrix;
                //projectionMatrix->makeFrustum( left, right, bottom, top, zNear, distance);
                //OE_INFO << "zNear=" << zNear << ", zFar=" << zFar << std::endl;
                projectionMatrix->makeFrustum( left, right, bottom, top, zNear, distance );
                renderInfo.getState()->applyProjectionMatrix( projectionMatrix.get());

                // Draw the drawable
                drawable->drawImplementation(renderInfo);

                // Reset the far plane to the old value.
                renderInfo.getState()->applyProjectionMatrix( oldProjectionMatrix.get() );
            }
            else
            {
                drawable->drawImplementation(renderInfo);
            }
        }

        double radius_;
    };

    struct AddCallbackToDrawablesVisitor : public osg::NodeVisitor
    {
        AddCallbackToDrawablesVisitor(double radius)
            : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
            radius_(radius) {}

        virtual void apply(osg::Geode& node)
        {
            for (unsigned int i = 0; i < node.getNumDrawables(); i++)
            {
                node.getDrawable(i)->setDrawCallback(
                    new OverrideNearFarValuesCallback(radius_));
                // Do not use display lists otherwise the callback will only
                // be called once on initial compile.
                node.getDrawable(i)->setUseDisplayList(false);
            }
        }

        double radius_;
    };
}

//---------------------------------------------------------------------------

namespace
{
    // Atmospheric Scattering and Sun Shaders
    // Adapted from code that is
    // Copyright (c) 2004 Sean O'Neil

    static char s_atmosphereVertexSource[] =
        "#version 110 \n"

        "uniform mat4 osg_ViewMatrixInverse;     // camera position \n"
        "uniform vec3 atmos_v3LightPos;        // The direction vector to the light source \n"
        "uniform vec3 atmos_v3InvWavelength;   // 1 / pow(wavelength,4) for the rgb channels \n"
        "uniform float atmos_fOuterRadius;     // Outer atmosphere radius \n"
        "uniform float atmos_fOuterRadius2;    // fOuterRadius^2 \n"		
        "uniform float atmos_fInnerRadius;     // Inner planetary radius \n"
        "uniform float atmos_fInnerRadius2;    // fInnerRadius^2 \n"
        "uniform float atmos_fKrESun;          // Kr * ESun \n"	
        "uniform float atmos_fKmESun;          // Km * ESun \n"		
        "uniform float atmos_fKr4PI;           // Kr * 4 * PI \n"	
        "uniform float atmos_fKm4PI;           // Km * 4 * PI \n"		
        "uniform float atmos_fScale;           // 1 / (fOuterRadius - fInnerRadius) \n"	
        "uniform float atmos_fScaleDepth;      // The scale depth \n"
        "uniform float atmos_fScaleOverScaleDepth;     // fScale / fScaleDepth \n"	
        "uniform int atmos_nSamples; \n"	
        "uniform float atmos_fSamples; \n"				

        "varying vec3 atmos_v3Direction; \n"

        "vec3 vVec; \n"
        "float atmos_fCameraHeight;    // The camera's current height \n"		
        "float atmos_fCameraHeight2;   // fCameraHeight^2 \n"

        "float atmos_scale(float fCos) \n"	
        "{ \n"
        "    float x = 1.0 - fCos; \n"
        "    return atmos_fScaleDepth * exp(-0.00287 + x*(0.459 + x*(3.83 + x*(-6.80 + x*5.25)))); \n"
        "} \n"

        "void SkyFromSpace(void) \n"
        "{ \n"
        "    // Get the ray from the camera to the vertex and its length (which is the far point of the ray passing through the atmosphere) \n"
        "    vec3 v3Pos = gl_Vertex.xyz; \n"
        "    vec3 v3Ray = v3Pos - vVec; \n"
        "    float fFar = length(v3Ray); \n"
        "    v3Ray /= fFar; \n"

        "    // Calculate the closest intersection of the ray with the outer atmosphere \n"
        "    // (which is the near point of the ray passing through the atmosphere) \n"
        "    float B = 2.0 * dot(vVec, v3Ray); \n"
        "    float C = atmos_fCameraHeight2 - atmos_fOuterRadius2; \n"
        "    float fDet = max(0.0, B*B - 4.0 * C); \n"	
        "    float fNear = 0.5 * (-B - sqrt(fDet)); \n"		

        "    // Calculate the ray's starting position, then calculate its atmos_ing offset \n"
        "    vec3 v3Start = vVec + v3Ray * fNear; \n"			
        "    fFar -= fNear; \n"	
        "    float fStartAngle = dot(v3Ray, v3Start) / atmos_fOuterRadius; \n"			
        "    float fStartDepth = exp(-1.0 / atmos_fScaleDepth); \n"
        "    float fStartOffset = fStartDepth*atmos_scale(fStartAngle); \n"		

        "    // Initialize the atmos_ing loop variables \n"	
        "    float fSampleLength = fFar / atmos_fSamples; \n"		
        "    float fScaledLength = fSampleLength * atmos_fScale; \n"					
        "    vec3 v3SampleRay = v3Ray * fSampleLength; \n"	
        "    vec3 v3SamplePoint = v3Start + v3SampleRay * 0.5; \n"	

        "    // Now loop through the sample rays \n"
        "    vec3 v3FrontColor = vec3(0.0, 0.0, 0.0); \n"
        "    vec3 v3Attenuate; \n"  
        "    for(int i=0; i<atmos_nSamples; i++) \n"		
        "    { \n"
        "        float fHeight = length(v3SamplePoint); \n"			
        "        float fDepth = exp(atmos_fScaleOverScaleDepth * (atmos_fInnerRadius - fHeight)); \n"
        "        float fLightAngle = dot(atmos_v3LightPos, v3SamplePoint) / fHeight; \n"		
        "        float fCameraAngle = dot(v3Ray, v3SamplePoint) / fHeight; \n"			
        "        float fscatter = (fStartOffset + fDepth*(atmos_scale(fLightAngle) - atmos_scale(fCameraAngle))); \n"	
        "        v3Attenuate = exp(-fscatter * (atmos_v3InvWavelength * atmos_fKr4PI + atmos_fKm4PI)); \n"	
        "        v3FrontColor += v3Attenuate * (fDepth * fScaledLength); \n"					
        "        v3SamplePoint += v3SampleRay; \n"		
        "    } \n"		

        "    // Finally, scale the Mie and Rayleigh colors and set up the varying \n"			
        "    // variables for the pixel shader \n"	
        "    gl_TexCoord[0].xyz   = v3FrontColor * atmos_fKmESun; \n"				
        "    gl_TexCoord[1].xyz = v3FrontColor * (atmos_v3InvWavelength * atmos_fKrESun); \n"							
        "    atmos_v3Direction = vVec  - v3Pos; \n"			
        "} \n"		

        "void SkyFromAtmosphere(void) \n"		
        "{ \n"
        "  // Get the ray from the camera to the vertex, and its length (which is the far \n"
        "  // point of the ray passing through the atmosphere) \n"		
        "  vec3 v3Pos = gl_Vertex.xyz; \n"	
        "  vec3 v3Ray = v3Pos - vVec; \n"			
        "  float fFar = length(v3Ray); \n"					
        "  v3Ray /= fFar; \n"				

        "  // Calculate the ray's starting position, then calculate its atmos_ing offset \n"
        "  vec3 v3Start = vVec; \n"
        "  float fHeight = length(v3Start); \n"		
        "  float fDepth = exp(atmos_fScaleOverScaleDepth * (atmos_fInnerRadius - atmos_fCameraHeight)); \n"
        "  float fStartAngle = dot(v3Ray, v3Start) / fHeight; \n"	
        "  float fStartOffset = fDepth*atmos_scale(fStartAngle); \n"

        "  // Initialize the atmos_ing loop variables \n"		
        "  float fSampleLength = fFar / atmos_fSamples; \n"			
        "  float fScaledLength = fSampleLength * atmos_fScale; \n"				
        "  vec3 v3SampleRay = v3Ray * fSampleLength; \n"		
        "  vec3 v3SamplePoint = v3Start + v3SampleRay * 0.5; \n"

        "  // Now loop through the sample rays \n"		
        "  vec3 v3FrontColor = vec3(0.0, 0.0, 0.0); \n"		
        "  vec3 v3Attenuate; \n"  
        "  for(int i=0; i<atmos_nSamples; i++) \n"			
        "  { \n"	
        "    float fHeight = length(v3SamplePoint); \n"	
        "    float fDepth = exp(atmos_fScaleOverScaleDepth * (atmos_fInnerRadius - fHeight)); \n"
        "    float fLightAngle = dot(atmos_v3LightPos, v3SamplePoint) / fHeight; \n"
        "    float fCameraAngle = dot(v3Ray, v3SamplePoint) / fHeight; \n"	
        "    float fscatter = (fStartOffset + fDepth*(atmos_scale(fLightAngle) - atmos_scale(fCameraAngle))); \n"	
        "    v3Attenuate = exp(-fscatter * (atmos_v3InvWavelength * atmos_fKr4PI + atmos_fKm4PI)); \n"	
        "    v3FrontColor += v3Attenuate * (fDepth * fScaledLength); \n"		
        "    v3SamplePoint += v3SampleRay; \n"		
        "  } \n"

        "  // Finally, scale the Mie and Rayleigh colors and set up the varying \n"
        "  // variables for the pixel shader \n"					
        "  gl_TexCoord[0].xyz  = v3FrontColor * atmos_fKmESun; \n"			
        "  gl_TexCoord[1].xyz  = v3FrontColor * (atmos_v3InvWavelength * atmos_fKrESun); \n"						
        "  atmos_v3Direction = vVec - v3Pos; \n"				
        "} \n"

        "void main(void) \n"
        "{ \n"
        "  // Get camera position and height \n"
        "  vVec = osg_ViewMatrixInverse[3].xyz; \n"
        "  atmos_fCameraHeight = length(vVec); \n"
        "  atmos_fCameraHeight2 = atmos_fCameraHeight*atmos_fCameraHeight; \n"
        "  gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex; \n"
        "  if(atmos_fCameraHeight >= atmos_fOuterRadius) { \n"
        "    SkyFromSpace(); \n"
        "  } \n"
        "  else { \n"
        "    SkyFromAtmosphere(); \n"
        "  } \n"
        "} \n";
        
    static char s_atmosphereFragmentSource[] =
        "#version 110 \n"

        "uniform vec3 atmos_v3LightPos; \n"							
        "uniform float atmos_g; \n"				
        "uniform float atmos_g2; \n"
        "uniform float atmos_fWeather; \n"

        "varying vec3 atmos_v3Direction; \n"	
        "const float fExposure = 4.0; \n"

        "void main(void) \n"			
        "{ \n"				
        "    float fCos = dot(atmos_v3LightPos, atmos_v3Direction) / length(atmos_v3Direction); \n"
        "    //float fRayleighPhase = 0.75 * (1.0 + fCos*fCos); \n"
        "    float fMiePhase = 1.5 * ((1.0 - atmos_g2) / (2.0 + atmos_g2)) * (1.0 + fCos*fCos) / pow(1.0 + atmos_g2 - 2.0*atmos_g*fCos, 1.5); \n"
        "    vec3 f4Color = gl_TexCoord[1].xyz + fMiePhase * gl_TexCoord[0].xyz; \n"
        "    vec3 color = 1.0 - exp(f4Color * -fExposure); \n"
        "    gl_FragColor.rgb = color.rgb*atmos_fWeather; \n"
        "    gl_FragColor.a = (color.r+color.g+color.b) * 2.0; \n"
        "} \n";

    static char s_sunVertexSource[] = 
        "#version 110 \n"
        "varying vec3 atmos_v3Direction; \n"

        "void main() \n"
        "{ \n"
        "    vec3 v3Pos = gl_Vertex.xyz; \n"
        "    gl_Position = ftransform(); \n"
        "    atmos_v3Direction = vec3(0.0,0.0,1.0) - v3Pos; \n"
        "    atmos_v3Direction = atmos_v3Direction/length(atmos_v3Direction); \n"
        "} \n";

    static char s_sunFragmentSource[] =
        "#version 110 \n"
        "uniform float sunAlpha; \n"
        "varying vec3 atmos_v3Direction; \n"

        "void main( void ) \n"
        "{ \n"
        "   float fCos = -atmos_v3Direction[2]; \n"         
        "   float fMiePhase = 0.050387596899224826 * (1.0 + fCos*fCos) / pow(1.9024999999999999 - -1.8999999999999999*fCos, 1.5); \n"
        "   gl_FragColor.rgb = fMiePhase*vec3(.3,.3,.2); \n"
        "   gl_FragColor.a = sunAlpha*gl_FragColor.r; \n"
        "} \n";
}

//---------------------------------------------------------------------------

SkyNode::SkyNode( Map* map ) :
_lightPos( osg::Vec3f(0.0f, 1.0f, 0.0f) )
{
    _innerRadius = map->getProfile()->getSRS()->getGeographicSRS()->getEllipsoid()->getRadiusPolar();
    _outerRadius = _innerRadius * 1.025f;

    makeAtmosphere();
    makeSun();
}

void
SkyNode::attach( osg::View* view, int lightNum )
{
    if ( !view ) return;

    _light = new osg::Light( lightNum );
    _light->setLightNum(0);
    _light->setPosition( osg::Vec4( _lightPos, 0 ) );
    _light->setAmbient( osg::Vec4(.2,.2,.2,1) );
    _light->setDiffuse( osg::Vec4(1,1,1,1) );
    _light->setSpecular( osg::Vec4(0,0,0,1) );

    view->setLightingMode( osg::View::SKY_LIGHT );
    view->setLight( _light.get() );
    view->getCamera()->setClearColor( osg::Vec4(0,0,0,1) );
}

void
SkyNode::setSunPosition( const osg::Vec3& pos )
{
    _lightPos = pos;

    if ( _light.valid() )
        _light->setPosition( osg::Vec4( _lightPos, 0 ) );

    if ( _lightPosUniform.valid() )
        _lightPosUniform->set( _lightPos / _lightPos.length() );

    if ( _sunXform.valid() )
        _sunXform->setMatrix( osg::Matrix::translate( _sunDistance * _lightPos.x(), _sunDistance * _lightPos.y(), _sunDistance * _lightPos.z() ) );
}

void
SkyNode::makeAtmosphere()
{
    // create some skeleton geometry to shade:
    osg::Sphere* sphere = new osg::Sphere( osg::Vec3(0,0,0), _outerRadius );

    osg::ref_ptr<osg::TessellationHints> hints = new osg::TessellationHints;
    hints->setDetailRatio(5.0f);    
    osg::Drawable* drawable = new osg::ShapeDrawable( sphere, hints.get() );

    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( drawable );

    osg::StateSet* set = geode->getOrCreateStateSet();

    // configure the state set:
    set->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    set->setMode( GL_CULL_FACE, osg::StateAttribute::ON );
    //set->setMode( GL_DEPTH, osg::StateAttribute::OFF );
    set->setMode( GL_DEPTH_TEST, osg::StateAttribute::ON );
    set->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );
    //set->setBinNumber( 65 ); // todo, what?
    set->setBinNumber( -4 );
    set->setAttributeAndModes( new osg::Depth( osg::Depth::LESS, 0, 1, false ) ); // no depth write
    osg::BlendFunc* blend = new osg::BlendFunc( GL_ONE, GL_ONE );
    set->setAttributeAndModes( blend, osg::StateAttribute::ON );
    set->setAttributeAndModes( new osg::FrontFace( osg::FrontFace::CLOCKWISE ), osg::StateAttribute::ON );

    // next, create and add the shaders:
    osg::Program* program = new osg::Program();
    osg::Shader* vs = new osg::Shader( osg::Shader::VERTEX, s_atmosphereVertexSource );
    program->addShader( vs );
    osg::Shader* fs = new osg::Shader( osg::Shader::FRAGMENT, s_atmosphereFragmentSource );
    program->addShader( fs );
    set->setAttributeAndModes( program, osg::StateAttribute::ON );

    // apply the uniforms:
    float r_wl   = ::powf( .65f, 4.0f );
    float g_wl = ::powf( .57f, 4.0f );
    float b_wl  = ::powf( .475f, 4.0f );
    osg::Vec3 RGB_wl( 1.0f/r_wl, 1.0f/g_wl, 1.0f/b_wl );
    float Kr = 0.0025f;
    float Kr4PI = Kr * 4.0f * osg::PI;
    float Km = 0.0015f;
    float Km4PI = Km * 4.0f * osg::PI;
    float ESun = 15.0f;
    float MPhase = -.095f;
    float RayleighScaleDepth = 0.25f;
    int   Samples = 2;
    float Weather = 1.0f;
    
    float Scale = 1.0f / (_outerRadius - _innerRadius);

    _lightPosUniform = set->getOrCreateUniform( "atmos_v3LightPos", osg::Uniform::FLOAT_VEC3 );
    _lightPosUniform->set( _lightPos / _lightPos.length() );

    set->getOrCreateUniform( "atmos_v3InvWavelength", osg::Uniform::FLOAT_VEC3 )->set( RGB_wl );
    set->getOrCreateUniform( "atmos_fInnerRadius",    osg::Uniform::FLOAT )->set( _innerRadius );
    set->getOrCreateUniform( "atmos_fInnerRadius2",   osg::Uniform::FLOAT )->set( _innerRadius * _innerRadius );
    set->getOrCreateUniform( "atmos_fOuterRadius",    osg::Uniform::FLOAT )->set( _outerRadius );
    set->getOrCreateUniform( "atmos_fOuterRadius2",   osg::Uniform::FLOAT )->set( _outerRadius * _outerRadius );
    set->getOrCreateUniform( "atmos_fKrESun",         osg::Uniform::FLOAT )->set( Kr * ESun );
    set->getOrCreateUniform( "atmos_fKmESun",         osg::Uniform::FLOAT )->set( Km * ESun );
    set->getOrCreateUniform( "atmos_fKr4PI",          osg::Uniform::FLOAT )->set( Kr4PI );
    set->getOrCreateUniform( "atmos_fKm4PI",          osg::Uniform::FLOAT )->set( Km4PI );
    set->getOrCreateUniform( "atmos_fScale",          osg::Uniform::FLOAT )->set( Scale );
    set->getOrCreateUniform( "atmos_fScaleDepth",     osg::Uniform::FLOAT )->set( RayleighScaleDepth );
    set->getOrCreateUniform( "atmos_fScaleOverScaleDepth", osg::Uniform::FLOAT )->set( Scale / RayleighScaleDepth );
    set->getOrCreateUniform( "atmos_g",               osg::Uniform::FLOAT )->set( MPhase );
    set->getOrCreateUniform( "atmos_g2",              osg::Uniform::FLOAT )->set( MPhase * MPhase );
    set->getOrCreateUniform( "atmos_nSamples",        osg::Uniform::INT )->set( Samples );
    set->getOrCreateUniform( "atmos_fSamples",        osg::Uniform::FLOAT )->set( (float)Samples );
    set->getOrCreateUniform( "atmos_fWeather",        osg::Uniform::FLOAT )->set( Weather );
    
    //geode->setCullCallback( new DoNotIncludeInNearFarComputationCallback() );
    AddCallbackToDrawablesVisitor visitor( _innerRadius );
    geode->accept( visitor );
    //this->addChild( geode );

    // add an intermediate group for the clip plane callback (won't work on the geode itself)
    osg::Group* g = new osg::Group;
    g->setCullCallback( new DoNotIncludeInNearFarComputationCallback() );
    g->addChild( geode );
    this->addChild( g );
}

void
SkyNode::makeSun()
{
    osg::Billboard* sun = new osg::Billboard();
    sun->setMode( osg::Billboard::POINT_ROT_EYE );
    sun->setNormal( osg::Vec3(0, 0, 1) );

    float sunRadius = _innerRadius * 100.0f;
    _sunDistance = _innerRadius * 12000.0f;

    sun->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(0,0,0), sunRadius*40.0f, 0.0)));

    osg::StateSet* set = sun->getOrCreateStateSet();

    set->getOrCreateUniform( "sunAlpha", osg::Uniform::FLOAT )->set( 1.0f );

    // configure the stateset
    set->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    set->setMode( GL_CULL_FACE, osg::StateAttribute::OFF );
    set->setMode( GL_DEPTH, osg::StateAttribute::OFF );
    set->setMode( GL_DEPTH_TEST, osg::StateAttribute::OFF );
    set->setMode( GL_BLEND, osg::StateAttribute::ON );
    set->setRenderBinDetails( -5, "RenderBin" );
    set->setAttributeAndModes( new osg::Depth(osg::Depth::LEQUAL, 1.0, 1.0), osg::StateAttribute::ON );
    set->setAttributeAndModes( new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA), osg::StateAttribute::ON );

    // create shaders
    osg::Program* program = new osg::Program();
    osg::Shader* vs = new osg::Shader( osg::Shader::VERTEX, s_sunVertexSource );
    program->addShader( vs );
    osg::Shader* fs = new osg::Shader( osg::Shader::FRAGMENT, s_sunFragmentSource );
    program->addShader( fs );
    set->setAttributeAndModes( program, osg::StateAttribute::ON );

    // make the sun's transform:
    _sunXform = new osg::MatrixTransform();
    _sunXform->setMatrix( osg::Matrix::translate( _sunDistance * _lightPos.x(), _sunDistance * _lightPos.y(), _sunDistance * _lightPos.z() ) );
    _sunXform->addChild( sun );
    
    AddCallbackToDrawablesVisitor visitor( _sunDistance );
    _sunXform->accept( visitor );

    // add an intermediate group for the clip plane callback (won't work on the node itself)
    osg::Group* g = new osg::Group;
    g->setCullCallback( new DoNotIncludeInNearFarComputationCallback() );
    g->addChild( _sunXform );
    this->addChild( g );
}
