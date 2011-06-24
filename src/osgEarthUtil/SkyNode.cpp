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
#include <osgEarthUtil/StarData>

#include <osgEarth/ShaderComposition>
#include <osgEarth/FindNode>
#include <osgEarth/MapNode>

#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/BlendFunc>
#include <osg/FrontFace>
#include <osg/CullFace>
#include <osg/Program>
#include <osg/Point>
#include <osg/Shape>
#include <osg/Depth>
#include <osg/Quat>

#include <sstream>
#include <time.h>

#define LC "[SkyNode] "

using namespace osgEarth;
using namespace osgEarth::Util;

//---------------------------------------------------------------------------

#define BIN_STARS      -10
#define BIN_SUN         -9
#define BIN_ATMOSPHERE  -8

//---------------------------------------------------------------------------

namespace
{
    // a cull callback that prevents objects from being included in the near/fear clip
    // plane calculates that OSG does. This is useful for including "distant objects"
    struct DoNotIncludeInNearFarComputationCallback : public osg::NodeCallback
    {
        virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
        {
            osgUtil::CullVisitor* cv = dynamic_cast< osgUtil::CullVisitor*>( nv );

            // Default value
            osg::CullSettings::ComputeNearFarMode oldMode;

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
            : _radius(radius) {}

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
                // radius, sun distance etc), and then some.
                double distance = eye.length() + _radius*2;

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

        double _radius;
    };

    struct AddCallbackToDrawablesVisitor : public osg::NodeVisitor
    {
        AddCallbackToDrawablesVisitor(double radius)
            : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
            _radius(radius) {}

        virtual void apply(osg::Geode& node)
        {
            for (unsigned int i = 0; i < node.getNumDrawables(); i++)
            {
                node.getDrawable(i)->setDrawCallback( new OverrideNearFarValuesCallback(_radius) );

                // Do not use display lists otherwise the callback will only
                // be called once on initial compile.
                node.getDrawable(i)->setUseDisplayList(false);
            }
        }

        double _radius;
    };

    osg::Geometry*
    s_makeEllipsoidGeometry( const osg::EllipsoidModel* ellipsoid, double outerRadius )
    {
        double hae = outerRadius - ellipsoid->getRadiusEquator();

        osg::Geometry* geom = new osg::Geometry();

        //geom->setUseVertexBufferObjects( true );
        geom->setUseDisplayList( false );

        int latSegments = 100;
        int lonSegments = 2 * latSegments;

        double segmentSize = 180.0/(double)latSegments; // degrees

        osg::Vec3Array* verts = new osg::Vec3Array();
        verts->reserve( latSegments * lonSegments );

        osg::DrawElementsUShort* el = new osg::DrawElementsUShort( GL_TRIANGLES );
        el->reserve( latSegments * lonSegments * 6 );

        for( int y = 0; y <= latSegments; ++y )
        {
            double lat = -90.0 + segmentSize * (double)y;
            for( int x = 0; x < lonSegments; ++x )
            {
                double lon = -180.0 + segmentSize * (double)x;
                double gx, gy, gz;
                ellipsoid->convertLatLongHeightToXYZ( osg::DegreesToRadians(lat), osg::DegreesToRadians(lon), hae, gx, gy, gz );
                verts->push_back( osg::Vec3(gx, gy, gz) );

                if ( y < latSegments )
                {
                    int x_plus_1 = x < lonSegments-1 ? x+1 : 0;
                    int y_plus_1 = y+1;
                    el->push_back( y*lonSegments + x );
                    el->push_back( y*lonSegments + x_plus_1 );
                    el->push_back( y_plus_1*lonSegments + x );
                    el->push_back( y*lonSegments + x_plus_1 );
                    el->push_back( y_plus_1*lonSegments + x_plus_1 );
                    el->push_back( y_plus_1*lonSegments + x );
                }
            }
        }

        geom->setVertexArray( verts );
        geom->addPrimitiveSet( el );

        return geom;
    }

    osg::Geometry*
    s_makeDiscGeometry( double radius )
    {
        int segments = 48;
        float deltaAngle = 360.0/(float)segments;

        osg::Geometry* geom = new osg::Geometry();

        //geom->setUseVertexBufferObjects( true );
        geom->setUseDisplayList( false );

        osg::Vec3Array* verts = new osg::Vec3Array();
        verts->reserve( 1 + segments );
        geom->setVertexArray( verts );

        osg::DrawElementsUShort* el = new osg::DrawElementsUShort( GL_TRIANGLES );
        el->reserve( 1 + 2*segments );
        geom->addPrimitiveSet( el );

        verts->push_back( osg::Vec3(0,0,0) ); // center point

        for( int i=0; i<segments; ++i )
        {
            double angle = osg::DegreesToRadians( deltaAngle * (float)i );
            double x = radius * cos( angle );
            double y = radius * sin( angle );
            verts->push_back( osg::Vec3(x, y, 0.0) );

            int i_plus_1 = i < segments-1? i+1 : 0;
            el->push_back( 0 );
            el->push_back( 1 + i_plus_1 );
            el->push_back( 1 + i );
        }

        return geom;
    }
}

//---------------------------------------------------------------------------

// Astronomical Math
// http://www.stjarnhimlen.se/comp/ppcomp.html
namespace
{
#define d2r(X) osg::DegreesToRadians(X)
#define r2d(X) osg::RadiansToDegrees(X)
#define nrad(X) { while( X > TWO_PI ) X -= TWO_PI; while( X < 0.0 ) X += TWO_PI; }
#define nrad2(X) { while( X <= -osg::PI ) X += TWO_PI; while( X > osg::PI ) X -= TWO_PI; }

    static const double TWO_PI = (2.0*osg::PI);
    static const double JD2000 = 2451545.0;

    //double getTimeScale( int year, int month, int date, double hoursUT )
    //{
    //    int a = 367*year - 7 * ( year + (month+9)/12 ) / 4 + 275*month/9 + date - 730530;
    //    return (double)a + hoursUT/24.0;
    //}

    double getJulianDate( int year, int month, int date )
    {
        if ( month <= 2 )
        {
            month += 12;
            year -= 1;
        }

        int A = int(year/100);
        int B = 2-A+(A/4);
        int C = int(365.25*(year+4716));
        int D = int(30.6001*(month+1));
        return B + C + D + date - 1524.5;
    }

    struct Sun
    {
        Sun() { }

        // https://www.cfa.harvard.edu/~wsoon/JuanRamirez09-d/Chang09-OptimalTiltAngleforSolarCollector.pdf
        osg::Vec3d getPosition(int year, int month, int date, double hoursUTC ) const
        {
            double JD = getJulianDate(year, month, date);
            double JD1 = (JD - JD2000);                         // julian time since JD2000 epoch
            double JC = JD1/36525.0;                            // julian century

            double mu = 282.937348 + 0.00004707624*JD1 + 0.0004569*(JC*JC);

            double epsilon = 280.466457 + 0.985647358*JD1 + 0.000304*(JC*JC);

            // orbit eccentricity:
            double E = 0.01670862 - 0.00004204 * JC;

            // mean anomaly of the perihelion
            double M = epsilon - mu;

            // perihelion anomaly:
            double v =
                M + 
                360.0*E*sin(d2r(M))/osg::PI + 
                900.0*(E*E)*sin(d2r(2*M))/4*osg::PI - 
                180.0*(E*E*E)*sin(d2r(M))/4.0*osg::PI;

            // longitude of the sun in ecliptic coordinates:
            double sun_lon = d2r(v - 360.0 + mu); // lambda
            nrad2(sun_lon);

            // angle between the ecliptic plane and the equatorial plane
            double zeta = d2r(23.4392); // zeta

            // latitude of the sun on the ecliptic plane:
//            double omega = d2r(0.0);

            // latitude of the sun with respect to the equatorial plane (solar declination):
            double sun_lat = asin( sin(sun_lon)*sin(zeta) );
            nrad2(sun_lat);

            // finally, adjust for the time of day (rotation of the earth)
            double time_r = hoursUTC/24.0; // 0..1
            nrad(sun_lon); // clamp to 0..TWO_PI
            double sun_r = sun_lon/TWO_PI; // convert to 0..1

            // rotational difference between UTC and current time
            double diff_r = sun_r - time_r;
            double diff_lon = TWO_PI * diff_r;

            // apparent sun longitude.
            double app_sun_lon = sun_lon - diff_lon + osg::PI;
            nrad2(app_sun_lon);

#if 0
            OE_INFO
                << "sun lat = " << r2d(sun_lat) 
                << ", sun lon = " << r2d(sun_lon)
                << ", time delta_lon = " << r2d(diff_lon)
                << ", app sun lon = " << r2d(app_sun_lon)
                << std::endl;
#endif

            return osg::Vec3d(
                cos(sun_lat) * cos(-app_sun_lon),
                cos(sun_lat) * sin(-app_sun_lon),
                sin(sun_lat) );
        }
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
        "varying vec3 atmos_mieColor; \n"
        "varying vec3 atmos_rayleighColor; \n"

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
        "    atmos_mieColor      = v3FrontColor * atmos_fKmESun; \n"				
        "    atmos_rayleighColor = v3FrontColor * (atmos_v3InvWavelength * atmos_fKrESun); \n"						
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
        "  atmos_mieColor      = v3FrontColor * atmos_fKmESun; \n"			
        "  atmos_rayleighColor = v3FrontColor * (atmos_v3InvWavelength * atmos_fKrESun); \n"				
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
        "      SkyFromSpace(); \n"
        "  } \n"
        "  else { \n"
        "      SkyFromAtmosphere(); \n"
        "  } \n"
        "} \n";
        
    static char s_atmosphereFragmentSource[] =
        "#version 110 \n"
        
        "float fastpow( in float x, in float y ) \n"
        "{ \n"
        "    return x/(x+y-y*x); \n"
        "} \n"

        "uniform vec3 atmos_v3LightPos; \n"							
        "uniform float atmos_g; \n"				
        "uniform float atmos_g2; \n"
        "uniform float atmos_fWeather; \n"

        "varying vec3 atmos_v3Direction; \n"	
        "varying vec3 atmos_mieColor; \n"
        "varying vec3 atmos_rayleighColor; \n"

        "const float fExposure = 4.0; \n"

        "void main(void) \n"			
        "{ \n"				
        "    float fCos = dot(atmos_v3LightPos, atmos_v3Direction) / length(atmos_v3Direction); \n"
        "    float fRayleighPhase = 1.0; \n" // 0.75 * (1.0 + fCos*fCos); \n"
        "    float fMiePhase = 1.5 * ((1.0 - atmos_g2) / (2.0 + atmos_g2)) * (1.0 + fCos*fCos) / fastpow(1.0 + atmos_g2 - 2.0*atmos_g*fCos, 1.5); \n"
        "    vec3 f4Color = fRayleighPhase * atmos_rayleighColor + fMiePhase * atmos_mieColor; \n"
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
        "    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex; \n"
        "    atmos_v3Direction = vec3(0.0,0.0,1.0) - v3Pos; \n"
        "    atmos_v3Direction = atmos_v3Direction/length(atmos_v3Direction); \n"
        "} \n";

    static char s_sunFragmentSource[] =
        "#version 110 \n"
        
        "float fastpow( in float x, in float y ) \n"
        "{ \n"
        "    return x/(x+y-y*x); \n"
        "} \n"

        "uniform float sunAlpha; \n"
        "varying vec3 atmos_v3Direction; \n"

        "void main( void ) \n"
        "{ \n"
        "   float fCos = -atmos_v3Direction[2]; \n"         
        "   float fMiePhase = 0.050387596899224826 * (1.0 + fCos*fCos) / fastpow(1.9024999999999999 - -1.8999999999999999*fCos, 1.5); \n"
        "   gl_FragColor.rgb = fMiePhase*vec3(.3,.3,.2); \n"
        "   gl_FragColor.a = sunAlpha*gl_FragColor.r; \n"
        "} \n";
}

//---------------------------------------------------------------------------

namespace
{
    static const char s_starVertexSource[] = 
        "void main() \n"
        "{ \n"
        "    gl_FrontColor = vec4(1,1,1,1); \n"
        "    gl_PointSize = gl_Color.r + 1.0f; \n"
        "    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex; \n"
        "} \n";

    static const char s_starFragmentSource[] = 
        "void main( void ) \n"
        "{ \n"
        "    gl_FragColor = gl_Color; \n"
        "} \n";
}

//---------------------------------------------------------------------------

SkyNode::SkyNode( Map* map, const std::string& starFile )
{
    // intialize the default settings:
    _defaultPerViewData._lightPos.set( osg::Vec3f(0.0f, 1.0f, 0.0f) );
    _defaultPerViewData._light = new osg::Light( 0 );  
    _defaultPerViewData._light->setPosition( osg::Vec4( _defaultPerViewData._lightPos, 0 ) );
    _defaultPerViewData._light->setAmbient( osg::Vec4(0.4f, 0.4f, 0.4f ,1.0) );
    _defaultPerViewData._light->setDiffuse( osg::Vec4(1,1,1,1) );
    _defaultPerViewData._light->setSpecular( osg::Vec4(0,0,0,1) );
    _defaultPerViewData._starsVisible = true;
    
    // set up the astronomical parameters:
    _ellipsoidModel =  map->getProfile()->getSRS()->getGeographicSRS()->getEllipsoid();
    _innerRadius = _ellipsoidModel->getRadiusPolar();
    _outerRadius = _innerRadius * 1.025f;
    _sunDistance = _innerRadius * 12000.0f;

    // make the ephemeris (note: order is important here)
    makeAtmosphere( _ellipsoidModel );
    makeSun();
    makeStars(starFile);
}

osg::BoundingSphere
SkyNode::computeBound() const
{
    return osg::BoundingSphere();
}

void
SkyNode::traverse( osg::NodeVisitor& nv )
{
    osg::CullSettings::ComputeNearFarMode saveMode;

    osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>( &nv );
    if ( cv )
    {
        saveMode = cv->getComputeNearFarMode();
        cv->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );

        osg::View* view = cv->getCurrentCamera()->getView();
        PerViewDataMap::iterator i = _perViewData.find( view );
        if ( i != _perViewData.end() )
        {
            i->second._cullContainer->accept( nv );
        }
    }

    osg::Group::traverse( nv );

    if ( cv )
    {
        cv->setComputeNearFarMode( saveMode );
    }
}

void
SkyNode::attach( osg::View* view, int lightNum )
{
    if ( !view ) return;

    // creates the new per-view if it does not already exist
    PerViewData& data = _perViewData[view];

    data._light = osg::clone( _defaultPerViewData._light.get() );
    data._light->setLightNum( lightNum );
    data._light->setAmbient( _defaultPerViewData._light->getAmbient() );
    data._lightPos = _defaultPerViewData._lightPos;

    // the cull callback has to be on a parent group-- won't work on the xforms themselves.
    data._cullContainer = new osg::Group();

    data._sunXform = new osg::MatrixTransform();
    data._sunMatrix = osg::Matrixd::translate(
        _sunDistance * data._lightPos.x(),
        _sunDistance * data._lightPos.y(),
        _sunDistance * data._lightPos.z() );
    data._sunXform->setMatrix( data._sunMatrix );
    data._sunXform->addChild( _sun.get() );
    data._cullContainer->addChild( data._sunXform.get() );

    data._starsXform = new osg::MatrixTransform();
    data._starsMatrix = _defaultPerViewData._starsMatrix;
    data._starsXform->setMatrix( _defaultPerViewData._starsMatrix );
    data._starsXform->addChild( _stars.get() );
    data._cullContainer->addChild( data._starsXform.get() );

    data._starsVisible = true;

    data._cullContainer->addChild( _atmosphere.get() );
    data._lightPosUniform = osg::clone( _defaultPerViewData._lightPosUniform.get() );

    view->setLightingMode( osg::View::SKY_LIGHT );
    view->setLight( data._light.get() );
    view->getCamera()->setClearColor( osg::Vec4(0,0,0,1) );
}

void
SkyNode::setAmbientBrightness( float value, osg::View* view )
{
    if ( !view )
    {
        setAmbientBrightness( _defaultPerViewData, value );

        for( PerViewDataMap::iterator i = _perViewData.begin(); i != _perViewData.end(); ++i )
            setAmbientBrightness( i->second, value );
    }
    else if ( _perViewData.find(view) != _perViewData.end() )
    {
        setAmbientBrightness( _perViewData[view], value );
    }
}

float
SkyNode::getAmbientBrightness( osg::View* view ) const
{
    if ( view )
    {
        PerViewDataMap::const_iterator i = _perViewData.find(view);
        if ( i != _perViewData.end() )
            return i->second._light->getAmbient().r();
    }
    return _defaultPerViewData._light->getAmbient().r();
}

void 
SkyNode::setAmbientBrightness( PerViewData& data, float value )
{
    value = osg::clampBetween( value, 0.0f, 1.0f );
    data._light->setAmbient( osg::Vec4f(value, value, value, 1.0f) );
}

void
SkyNode::setSunPosition( const osg::Vec3& pos, osg::View* view )
{
    if ( !view )
    {
        setSunPosition( _defaultPerViewData, pos );
        for( PerViewDataMap::iterator i = _perViewData.begin(); i != _perViewData.end(); ++i )
            setSunPosition( i->second, pos );
    }
    else if ( _perViewData.find(view) != _perViewData.end() )
    {
        setSunPosition( _perViewData[view], pos );
    }
}

void
SkyNode::setSunPosition( PerViewData& data, const osg::Vec3& pos )
{
    data._lightPos = pos;

    if ( data._light.valid() )
        data._light->setPosition( osg::Vec4( data._lightPos, 0 ) );

    if ( data._lightPosUniform.valid() )
        data._lightPosUniform->set( data._lightPos / data._lightPos.length() );

    if ( data._sunXform.valid() )
    {
        data._sunXform->setMatrix( osg::Matrix::translate( 
            _sunDistance * data._lightPos.x(), 
            _sunDistance * data._lightPos.y(),
            _sunDistance * data._lightPos.z() ) );
    }
}

void
SkyNode::setSunPosition( double lat_degrees, double long_degrees, osg::View* view )
{
    if (_ellipsoidModel.valid())
    {
        double x, y, z;
        _ellipsoidModel->convertLatLongHeightToXYZ(
            osg::RadiansToDegrees(lat_degrees),
            osg::RadiansToDegrees(long_degrees),
            0, 
            x, y, z);
        osg::Vec3d up  = _ellipsoidModel->computeLocalUpVector(x, y, z);
        setSunPosition( up, view );
    }
}

void
SkyNode::setDateTime( int year, int month, int date, double hoursUTC, osg::View* view )
{
    if ( _ellipsoidModel.valid() )
    {
        // position the sun:
        Sun sun;
        osg::Vec3d pos = sun.getPosition( year, month, date, hoursUTC );
        pos.normalize();
        setSunPosition( pos, view );

        // position the stars:
        double time_r = hoursUTC/24.0; // 0..1
        double rot_z = -osg::PI + TWO_PI*time_r;

        osg::Matrixd starsMatrix = osg::Matrixd::rotate( -rot_z, 0, 0, 1 );
        if ( !view )
        {
            _defaultPerViewData._starsMatrix = starsMatrix;
            for( PerViewDataMap::iterator i = _perViewData.begin(); i != _perViewData.end(); ++i )
            {
                i->second._starsMatrix = starsMatrix;
                i->second._starsXform->setMatrix( starsMatrix );
            }
        }
        else if ( _perViewData.find(view) != _perViewData.end() )
        {
            PerViewData& data = _perViewData[view];
            data._starsMatrix = starsMatrix;
            data._starsXform->setMatrix( starsMatrix );
        }
    }
}

void
SkyNode::setStarsVisible( bool value, osg::View* view )
{
    if ( !view )
    {
        _defaultPerViewData._starsVisible = value;
        for( PerViewDataMap::iterator i = _perViewData.begin(); i != _perViewData.end(); ++i )
        {
            i->second._starsVisible = value;
            i->second._starsXform->setNodeMask( value ? ~0 : 0 );
        }
    }
    else if ( _perViewData.find(view) != _perViewData.end() )
    {
        _perViewData[view]._starsVisible = value;
        _perViewData[view]._starsXform->setNodeMask( value ? ~0 : 0 );
    }
}

bool
SkyNode::getStarsVisible( osg::View* view ) const
{
    PerViewDataMap::const_iterator i = _perViewData.find(view);

    if ( !view || i == _perViewData.end() )
    {
        return _defaultPerViewData._starsVisible;
    }
    else
    {
        return i->second._starsVisible;
    }
}

void
SkyNode::makeAtmosphere( const osg::EllipsoidModel* em )
{
    // create some skeleton geometry to shade:
    osg::Geometry* drawable = s_makeEllipsoidGeometry( em, _outerRadius );

    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( drawable );

    osg::StateSet* set = geode->getOrCreateStateSet();

    // configure the state set:
    set->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    set->setMode( GL_CULL_FACE, osg::StateAttribute::ON );
    set->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );
    //set->setBinNumber( 65 ); // todo, what?
    set->setBinNumber( BIN_ATMOSPHERE );
    set->setAttributeAndModes( new osg::Depth( osg::Depth::LESS, 0, 1, false ) ); // no depth write
    set->setAttributeAndModes( new osg::BlendFunc( GL_ONE, GL_ONE ), osg::StateAttribute::ON );
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

    _defaultPerViewData._lightPosUniform = set->getOrCreateUniform( "atmos_v3LightPos", osg::Uniform::FLOAT_VEC3 );
    _defaultPerViewData._lightPosUniform->set( _defaultPerViewData._lightPos / _defaultPerViewData._lightPos.length() );

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

    _atmosphere = geode;
}

void
SkyNode::makeSun()
{
    osg::Billboard* sun = new osg::Billboard();
    sun->setMode( osg::Billboard::POINT_ROT_EYE );
    sun->setNormal( osg::Vec3(0, 0, 1) );

    float sunRadius = _innerRadius * 100.0f;

    sun->addDrawable( s_makeDiscGeometry( sunRadius*80.0f ) ); 

    osg::StateSet* set = sun->getOrCreateStateSet();

    set->getOrCreateUniform( "sunAlpha", osg::Uniform::FLOAT )->set( 1.0f );

    // configure the stateset
    set->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    set->setMode( GL_CULL_FACE, osg::StateAttribute::OFF );
    set->setRenderBinDetails( BIN_SUN, "RenderBin" );
    set->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS, 0, 1, false), osg::StateAttribute::ON );
    set->setAttributeAndModes( new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA), osg::StateAttribute::ON );

    // create shaders
    osg::Program* program = new osg::Program();
    osg::Shader* vs = new osg::Shader( osg::Shader::VERTEX, s_sunVertexSource );
    program->addShader( vs );
    osg::Shader* fs = new osg::Shader( osg::Shader::FRAGMENT, s_sunFragmentSource );
    program->addShader( fs );
    set->setAttributeAndModes( program, osg::StateAttribute::ON );

    // make the sun's transform:
    // todo: move this?
    _defaultPerViewData._sunXform = new osg::MatrixTransform();
    _defaultPerViewData._sunXform->setMatrix( osg::Matrix::translate( 
        _sunDistance * _defaultPerViewData._lightPos.x(), 
        _sunDistance * _defaultPerViewData._lightPos.y(), 
        _sunDistance * _defaultPerViewData._lightPos.z() ) );
    _defaultPerViewData._sunXform->addChild( sun );
    
    AddCallbackToDrawablesVisitor visitor( _sunDistance );
    sun->accept( visitor );

    _sun = sun;
}

SkyNode::StarData::StarData(std::stringstream &ss)
{
  std::getline( ss, name, ',' );
  std::string buff;
  std::getline( ss, buff, ',' );
  std::stringstream(buff) >> right_ascension;
  std::getline( ss, buff, ',' );
  std::stringstream(buff) >> declination;
  std::getline( ss, buff, '\n' );
  std::stringstream(buff) >> magnitude;
}

void
SkyNode::makeStars(const std::string& starFile)
{
  _starRadius = 20000.0 * (_sunDistance > 0.0 ? _sunDistance : _outerRadius);

  std::vector<StarData> stars;

  if( starFile.empty() || parseStarFile(starFile, stars) == false )
  {
    if( !starFile.empty() )
      OE_WARN << "Warning: Unable to use star field defined in file \"" << starFile << "\", using default star data." << std::endl;

    getDefaultStars(stars);
  }

  osg::Node* starNode = buildStarGeometry(stars);

  AddCallbackToDrawablesVisitor visitor(_starRadius);
  starNode->accept(visitor);

  _stars = starNode;
}

osg::Geode*
SkyNode::buildStarGeometry(const std::vector<StarData>& stars)
{
  double minMag = DBL_MAX, maxMag = DBL_MIN;

  osg::Vec3Array* coords = new osg::Vec3Array();
  std::vector<StarData>::const_iterator p;
  for( p = stars.begin(); p != stars.end(); p++ )
  {
    osg::Vec3 v = osg::Vec3(0,_starRadius,0) * 
      osg::Matrix::rotate( p->declination, 1, 0, 0 ) * 
      osg::Matrix::rotate( p->right_ascension, 0, 0, 1 );

    coords->push_back( v );

    if ( p->magnitude < minMag ) minMag = p->magnitude;
    if ( p->magnitude > maxMag ) maxMag = p->magnitude;
  }

  osg::Vec4Array* colors = new osg::Vec4Array();
  for( p = stars.begin(); p != stars.end(); p++ )
  {
      //float c = 0.5f + 0.5f * ( (p->magnitude-minMag) / (maxMag-minMag) );
      float c = ( (p->magnitude-minMag) / (maxMag-minMag) );
      colors->push_back( osg::Vec4(c,c,c,1.0f) );
  }

  osg::Geometry* geometry = new osg::Geometry;
  geometry->setUseVertexBufferObjects( true );
  geometry->setVertexArray( coords );
  geometry->setColorArray( colors );
  geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
  geometry->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, coords->size()));

  osg::StateSet* sset = new osg::StateSet;

  sset->setMode( GL_VERTEX_PROGRAM_POINT_SIZE, osg::StateAttribute::ON );
  osg::Program* program = new osg::Program;
  program->addShader( new osg::Shader(osg::Shader::VERTEX, s_starVertexSource) );
  program->addShader( new osg::Shader(osg::Shader::FRAGMENT, s_starFragmentSource) );
  sset->setAttributeAndModes( program, osg::StateAttribute::ON );

  sset->setRenderBinDetails( BIN_STARS, "RenderBin");
  sset->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS, 0, 1, false), osg::StateAttribute::ON );
  geometry->setStateSet( sset );

  osg::Geode* starGeode = new osg::Geode;
  starGeode->addDrawable( geometry );

  return starGeode;
}

void
SkyNode::getDefaultStars(std::vector<StarData>& out_stars)
{
  out_stars.clear();

  for(const char **sptr = s_defaultStarData; *sptr; sptr++)
  {
    std::stringstream ss(*sptr);
    out_stars.push_back(StarData(ss));
  }
}

bool
SkyNode::parseStarFile(const std::string& starFile, std::vector<StarData>& out_stars)
{
  out_stars.clear();

  std::fstream in(starFile.c_str());
  if (!in)
  {
    OE_WARN <<  "Warning: Unable to open file star file \"" << starFile << "\"" << std::endl;
    return false ;
  }

  while (!in.eof())
  {
    std::string line;

    std::getline(in, line);
    if (in.eof())
      break;

    if (line.empty() || line[0] == '#') 
      continue;

    std::stringstream ss(line);
    out_stars.push_back(StarData(ss));
  }

  in.close();

  return true;
}
