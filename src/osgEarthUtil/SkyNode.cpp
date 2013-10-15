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
#include <osgEarthUtil/SkyNode>
#include <osgEarthUtil/StarData>
#include <osgEarthUtil/LatLongFormatter>

#include <osgEarth/VirtualProgram>
#include <osgEarth/NodeUtils>
#include <osgEarth/MapNode>
#include <osgEarth/Utils>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/CullingUtils>

#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/PointSprite>
#include <osg/BlendFunc>
#include <osg/FrontFace>
#include <osg/CullFace>
#include <osg/Program>
#include <osg/Camera>
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

#define BIN_STARS       -100003
#define BIN_SUN         -100002
#define BIN_MOON        -100001
#define BIN_ATMOSPHERE  -100000

//---------------------------------------------------------------------------

namespace
{
    // constucts an ellipsoidal mesh that we will use to draw the atmosphere
    osg::Geometry*
    s_makeEllipsoidGeometry( const osg::EllipsoidModel* ellipsoid, double outerRadius, bool genTexCoords = false )
    {
        double hae = outerRadius - ellipsoid->getRadiusEquator();

        osg::Geometry* geom = new osg::Geometry();
        geom->setUseVertexBufferObjects(true);

        int latSegments = 100;
        int lonSegments = 2 * latSegments;

        double segmentSize = 180.0/(double)latSegments; // degrees

        osg::Vec3Array* verts = new osg::Vec3Array();
        verts->reserve( latSegments * lonSegments );

        osg::Vec2Array* texCoords = 0;
        osg::Vec3Array* normals = 0;
        if (genTexCoords)
        {
            texCoords = new osg::Vec2Array();
            texCoords->reserve( latSegments * lonSegments );
            geom->setTexCoordArray( 0, texCoords );

            normals = new osg::Vec3Array();
            normals->reserve( latSegments * lonSegments );
            geom->setNormalArray( normals );
            geom->setNormalBinding(osg::Geometry::BIND_PER_VERTEX );
        }

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

                if (genTexCoords)
                {
                    double s = (lon + 180) / 360.0;
                    double t = (lat + 90.0) / 180.0;
                    texCoords->push_back( osg::Vec2(s, t ) );
                }

                if (normals)
                {
                    osg::Vec3 normal( gx, gy, gz);
                    normal.normalize();
                    normals->push_back( normal );
                }


                if ( y < latSegments )
                {
                    int x_plus_1 = x < lonSegments-1 ? x+1 : 0;
                    int y_plus_1 = y+1;
                    el->push_back( y*lonSegments + x );
                    el->push_back( y_plus_1*lonSegments + x );
                    el->push_back( y*lonSegments + x_plus_1 );
                    el->push_back( y*lonSegments + x_plus_1 );
                    el->push_back( y_plus_1*lonSegments + x );
                    el->push_back( y_plus_1*lonSegments + x_plus_1 );
                }
            }
        }

        geom->setVertexArray( verts );
        geom->addPrimitiveSet( el );

//        OSG_ALWAYS << "s_makeEllipsoidGeometry Bounds: " << geom->computeBound().radius() << " outerRadius: " << outerRadius << std::endl;
        
        return geom;
    }

    // makes a disc geometry that we'll use to render the sun/moon
    osg::Geometry*
    s_makeDiscGeometry( double radius )
    {
        int segments = 48;
        float deltaAngle = 360.0/(float)segments;

        osg::Geometry* geom = new osg::Geometry();
        geom->setUseVertexBufferObjects(true);

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


    double sgCalcEccAnom(double M, double e)
    {
        double eccAnom, E0, E1, diff;

        double epsilon = osg::DegreesToRadians(0.001);
        
        eccAnom = M + e * sin(M) * (1.0 + e * cos (M));
        // iterate to achieve a greater precision for larger eccentricities 
        if (e > 0.05)
        {
            E0 = eccAnom;
            do
            {
                 E1 = E0 - (E0 - e * sin(E0) - M) / (1 - e *cos(E0));
                 diff = fabs(E0 - E1);
                 E0 = E1;
            } while (diff > epsilon );
            return E0;
        }
        return eccAnom;
    }

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
            double omega = d2r(0.0);

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

    struct Moon
    {
        Moon() { }

        static std::string radiansToHoursMinutesSeconds(double ra)
        {
            while (ra < 0) ra += (osg::PI * 2.0);
            //Get the total number of hours
            double hours = (ra / (osg::PI * 2.0) ) * 24.0;
            double minutes = hours - (int)hours;
            hours -= minutes;
            minutes *= 60.0;
            double seconds = minutes - (int)minutes;
            seconds *= 60.0;
            std::stringstream buf;
            buf << (int)hours << ":" << (int)minutes << ":" << (int)seconds;
            return buf.str();
        }

        // From http://www.stjarnhimlen.se/comp/ppcomp.html
        osg::Vec3d getPosition(int year, int month, int date, double hoursUTC ) const
        {
            //double julianDate = getJulianDate( year, month, date );
            //julianDate += hoursUTC /24.0;
            double d = 367*year - 7 * ( year + (month+9)/12 ) / 4 + 275*month/9 + date - 730530;
            d += (hoursUTC / 24.0);                     

            double ecl = osg::DegreesToRadians(23.4393 - 3.563E-7 * d);

            double N = osg::DegreesToRadians(125.1228 - 0.0529538083 * d);
            double i = osg::DegreesToRadians(5.1454);
            double w = osg::DegreesToRadians(318.0634 + 0.1643573223 * d);
            double a = 60.2666;//  (Earth radii)
            double e = 0.054900;
            double M = osg::DegreesToRadians(115.3654 + 13.0649929509 * d);

            double E = M + e*(180.0/osg::PI) * sin(M) * ( 1.0 + e * cos(M) );
            
            double xv = a * ( cos(E) - e );
            double yv = a * ( sqrt(1.0 - e*e) * sin(E) );

            double v = atan2( yv, xv );
            double r = sqrt( xv*xv + yv*yv );

            //Compute the geocentric (Earth-centered) position of the moon in the ecliptic coordinate system
            double xh = r * ( cos(N) * cos(v+w) - sin(N) * sin(v+w) * cos(i) );
            double yh = r * ( sin(N) * cos(v+w) + cos(N) * sin(v+w) * cos(i) );
            double zh = r * ( sin(v+w) * sin(i) );

            // calculate the ecliptic latitude and longitude here
            double lonEcl = atan2 (yh, xh);
            double latEcl = atan2(zh, sqrt(xh*xh + yh*yh));

            double xg = r * cos(lonEcl) * cos(latEcl);
            double yg = r * sin(lonEcl) * cos(latEcl);
            double zg = r * sin(latEcl);

            double xe = xg;
            double ye = yg * cos(ecl) -zg * sin(ecl);
            double ze = yg * sin(ecl) +zg * cos(ecl);

            double RA    = atan2(ye, xe);
            double Dec = atan2(ze, sqrt(xe*xe + ye*ye));

            //Just use the average distance from the earth            
            double rg = 6378137.0 + 384400000.0;
            
            // finally, adjust for the time of day (rotation of the earth)
            double time_r = hoursUTC/24.0; // 0..1            
            double moon_r = RA/TWO_PI; // convert to 0..1

            // rotational difference between UTC and current time
            double diff_r = moon_r - time_r;
            double diff_lon = TWO_PI * diff_r;

            RA -= diff_lon;

            nrad2(RA);

            return SkyNode::getPositionFromRADecl( RA, Dec, rg );
        }
    };
}

//---------------------------------------------------------------------------

namespace
{
    // Atmospheric Scattering and Sun Shaders
    // Adapted from code that is
    // Copyright (c) 2004 Sean O'Neil

    static char s_versionString[] =
#ifdef OSG_GLES2_AVAILABLE
        "#version 100 \n";
#else
        "#version 110 \n";
#endif

    static char s_mathUtils[] =
        "float fastpow( in float x, in float y ) \n"
        "{ \n"
        "    return x/(x+y-y*x); \n"
        "} \n";

    static char s_atmosphereVertexDeclarations[] =
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
        "float atmos_fCameraHeight2;   // fCameraHeight^2 \n";

    static char s_atmosphereVertexShared[] =
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
        "} \n";


    static char s_atmosphereVertexMain[] =
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

    static char s_atmosphereFragmentDeclarations[] =
        "uniform vec3 atmos_v3LightPos; \n"							
        "uniform float atmos_g; \n"				
        "uniform float atmos_g2; \n"
        "uniform float atmos_fWeather; \n"

        "varying vec3 atmos_v3Direction; \n"	
        "varying vec3 atmos_mieColor; \n"
        "varying vec3 atmos_rayleighColor; \n"

        "const float fExposure = 4.0; \n";
        
    static char s_atmosphereFragmentMain[] =
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
        "varying vec3 atmos_v3Direction; \n"

        "void main() \n"
        "{ \n"
        "    vec3 v3Pos = gl_Vertex.xyz; \n"
        "    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex; \n"
        "    atmos_v3Direction = vec3(0.0,0.0,1.0) - v3Pos; \n"
        "    atmos_v3Direction = atmos_v3Direction/length(atmos_v3Direction); \n"
        "} \n";

    static char s_sunFragmentSource[] =
        "uniform float sunAlpha; \n"
        "varying vec3 atmos_v3Direction; \n"

        "void main( void ) \n"
        "{ \n"
        "   float fCos = -atmos_v3Direction[2]; \n"         
        "   float fMiePhase = 0.050387596899224826 * (1.0 + fCos*fCos) / fastpow(1.9024999999999999 - -1.8999999999999999*fCos, 1.5); \n"
        "   gl_FragColor.rgb = fMiePhase*vec3(.3,.3,.2); \n"
        "   gl_FragColor.a = sunAlpha*gl_FragColor.r; \n"
        "} \n";

    static char s_moonVertexSource[] = 
        "uniform mat4 osg_ModelViewProjectionMatrix;"
        "varying vec4 moon_TexCoord;\n"
        "void main() \n"
        "{ \n"
        "    moon_TexCoord = gl_MultiTexCoord0; \n"
        "    gl_Position = osg_ModelViewProjectionMatrix * gl_Vertex; \n"
        "} \n";

    static char s_moonFragmentSource[] =
        "varying vec4 moon_TexCoord;\n"
        "uniform sampler2D moonTex;\n"
        "void main( void ) \n"
        "{ \n"
        "   gl_FragColor = texture2D(moonTex, moon_TexCoord.st);\n"
        "} \n";
}

//---------------------------------------------------------------------------

namespace
{
    static std::string s_createStarVertexSource()
    {
        float glslVersion = Registry::instance()->getCapabilities().getGLSLVersion();

        return Stringify()
            << "#version " << (glslVersion < 1.2f ? GLSL_VERSION_STR : "120") << "\n"

            << "float remap( float val, float vmin, float vmax, float r0, float r1 ) \n"
            << "{ \n"
            << "    float vr = (clamp(val, vmin, vmax)-vmin)/(vmax-vmin); \n"
            << "    return r0 + vr * (r1-r0); \n"
            << "} \n"

            << "uniform vec3 atmos_v3LightPos; \n"
            << "uniform mat4 osg_ViewMatrixInverse; \n"
            << "varying float visibility; \n"
            << "varying vec4 osg_FrontColor; \n"
            << "void main() \n"
            << "{ \n"
            << "    osg_FrontColor = gl_Color; \n"
            << "    gl_PointSize = gl_Color.r * " << (glslVersion < 1.2f ? "2.0" : "14.0") << ";\n"
            << "    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex; \n"

            << "    vec3 eye = osg_ViewMatrixInverse[3].xyz; \n"
            << "    float hae = length(eye) - 6378137.0; \n"
            // "highness": visibility increases with altitude
            << "    float highness = remap( hae, 25000.0, 150000.0, 0.0, 1.0 ); \n"
            << "    eye = normalize(eye); \n"
            // "darkness": visibility increase as the sun goes around the other side of the earth
            << "    float darkness = 1.0-remap(dot(eye,atmos_v3LightPos), -0.25, 0.0, 0.0, 1.0); \n"
            << "    visibility = clamp(highness + darkness, 0.0, 1.0); \n"
            << "} \n";
    }

    static std::string s_createStarFragmentSource()
    {
        float glslVersion = Registry::instance()->getCapabilities().getGLSLVersion();

        if ( glslVersion < 1.2f )
        {
            return Stringify()
                << "#version " << GLSL_VERSION_STR << "\n"
#ifdef OSG_GLES2_AVAILABLE
                << "precision highp float;\n"
#endif  
                << "varying float visibility; \n"
                << "varying vec4 osg_FrontColor; \n"
                << "void main( void ) \n"
                << "{ \n"
                << "    gl_FragColor = osg_FrontColor * visibility; \n"
                << "} \n";
        }
        else
        {
            return Stringify()
                << "#version 120 \n"
#ifdef OSG_GLES2_AVAILABLE
                << "precision highp float;\n"
#endif
                << "varying float visibility; \n"
                << "varying vec4 osg_FrontColor; \n"
                << "void main( void ) \n"
                << "{ \n"
                << "    float b1 = 1.0-(2.0*abs(gl_PointCoord.s-0.5)); \n"
                << "    float b2 = 1.0-(2.0*abs(gl_PointCoord.t-0.5)); \n"
                << "    float i = b1*b1 * b2*b2; \n"
                << "    gl_FragColor = osg_FrontColor * i * visibility; \n"
                << "} \n";
        }
    }
}


//---------------------------------------------------------------------------

osg::Vec3d
DefaultEphemerisProvider::getSunPosition(const DateTime& date)
{
    Sun sun;
    return sun.getPosition( date.year(), date.month(), date.day(), date.hours() );
}

osg::Vec3d
DefaultEphemerisProvider::getMoonPosition(const DateTime& date)
{
    Moon moon;
    return moon.getPosition( date.year(), date.month(), date.day(), date.hours() );
}

//---------------------------------------------------------------------------

SkyNode::SkyNode( Map* map, const std::string& starFile, float minStarMagnitude ) : 
_minStarMagnitude(minStarMagnitude)
{
    initialize(map, starFile);
}

SkyNode::SkyNode( Map *map, float minStarMagnitude) : 
_minStarMagnitude(minStarMagnitude)
{
    initialize(map);
}

void
SkyNode::initialize( Map *map, const std::string& starFile )
{
    _ephemerisProvider = new DefaultEphemerisProvider();

    // intialize the default settings:
    _defaultPerViewData._lightPos.set( osg::Vec3f(0.0f, 1.0f, 0.0f) );
    _defaultPerViewData._light = new osg::Light( 0 );  
    _defaultPerViewData._light->setPosition( osg::Vec4( _defaultPerViewData._lightPos, 0 ) );
    _defaultPerViewData._light->setAmbient( osg::Vec4(0.2f, 0.2f, 0.2f, 2.0) );
    _defaultPerViewData._light->setDiffuse( osg::Vec4(1,1,1,1) );
    _defaultPerViewData._light->setSpecular( osg::Vec4(0,0,0,1) );
    _defaultPerViewData._starsVisible = true;
    _defaultPerViewData._moonVisible = true;
    
    // set up the uniform that conveys the normalized light position in world space
    _defaultPerViewData._lightPosUniform = new osg::Uniform( osg::Uniform::FLOAT_VEC3, "atmos_v3LightPos" );
    _defaultPerViewData._lightPosUniform->set( _defaultPerViewData._lightPos / _defaultPerViewData._lightPos.length() );
    
    // set up the astronomical parameters:
    _ellipsoidModel = map->getProfile()->getSRS()->getGeographicSRS()->getEllipsoid();
    _innerRadius = _ellipsoidModel->getRadiusPolar();
    _outerRadius = _innerRadius * 1.025f;
    _sunDistance = _innerRadius * 12000.0f;

    // make the sky elements (don't change the order here)
    makeAtmosphere( _ellipsoidModel.get() );

    makeSun();

    makeMoon();

    if (_minStarMagnitude < 0)
    {
      const char* magEnv = ::getenv("OSGEARTH_MIN_STAR_MAGNITUDE");
      if (magEnv)
        _minStarMagnitude = as<float>(std::string(magEnv), -1.0f);
    }

    makeStars(starFile);

    // automatically compute ambient lighting based on the eyepoint
    _autoAmbience = false;

    //Set a default time
    setDateTime( DateTime(2011, 3, 6, 18.) );
}

osg::BoundingSphere
SkyNode::computeBound() const
{
    return osg::BoundingSphere();
}

void
SkyNode::traverse( osg::NodeVisitor& nv )
{    
    osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);
    if ( cv )
    {

        // If there's a custom projection matrix clamper installed, remove it temporarily.
        // We dont' want it mucking with our sky elements.
        osg::ref_ptr<osg::CullSettings::ClampProjectionMatrixCallback> cb = cv->getClampProjectionMatrixCallback();
        cv->setClampProjectionMatrixCallback( 0L );

        osg::View* view = cv->getCurrentCamera()->getView();

                
        //Try to find the per view data for camera's view if there is one.
        PerViewDataMap::iterator itr = _perViewData.find( view );
        
        if ( itr == _perViewData.end() )
        {
            // If we don't find any per view data, just use the first one that is stored.
            // This needs to be reworked to be per camera and also to automatically create a 
            // new data structure on demand since camera's can be added/removed on the fly.
            itr = _perViewData.begin();
        }
        

        if ( _autoAmbience )
        {
            const float minAmb = 0.2f;
            const float maxAmb = 0.92f;
            const float minDev = -0.2f;
            const float maxDev = 0.75f;
            osg::Vec3 eye = cv->getViewPoint(); eye.normalize();
            osg::Vec3 sun = itr->second._lightPos; sun.normalize();
            float dev = osg::clampBetween(eye*sun, minDev, maxDev);
            float r   = (dev-minDev)/(maxDev-minDev);
            float amb = minAmb + r*(maxAmb-minAmb);
            itr->second._light->setAmbient( osg::Vec4(amb,amb,amb,1.0) );
            //OE_INFO << "dev=" << dev << ", amb=" << amb << std::endl;
        }

        itr->second._cullContainer->accept( nv );

        // restore a custom clamper.
        if ( cb.valid() ) cv->setClampProjectionMatrixCallback( cb.get() );
    }

    else
    {
        osg::Group::traverse( nv );
    }
}

EphemerisProvider*
SkyNode::getEphemerisProvider() const
{
    return _ephemerisProvider;
}

void
SkyNode::setEphemerisProvider(EphemerisProvider* ephemerisProvider )
{
    if (_ephemerisProvider != ephemerisProvider)
    {
        _ephemerisProvider = ephemerisProvider;

        //Update the positions of the planets
        for( PerViewDataMap::iterator i = _perViewData.begin(); i != _perViewData.end(); ++i )
        {
            setDateTime(i->second._date, i->first);
        }
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

    data._moonXform = new osg::MatrixTransform();
    data._moonMatrix = _defaultPerViewData._moonMatrix;
    data._moonXform->setMatrix( data._moonMatrix );
    data._moonXform->addChild( _moon.get() );
    data._cullContainer->addChild( data._moonXform.get() );
    data._moonVisible = _defaultPerViewData._moonVisible;
    data._moonXform->setNodeMask( data._moonVisible ? ~0 : 0 );
    
    data._starsXform = new osg::MatrixTransform();
    data._starsMatrix = _defaultPerViewData._starsMatrix;
    data._starsXform->setMatrix( _defaultPerViewData._starsMatrix );
    data._starsXform->addChild( _stars.get() );
    data._cullContainer->addChild( data._starsXform.get() );
    data._starsVisible = _defaultPerViewData._starsVisible;
    data._starsXform->setNodeMask( data._starsVisible ? ~0 : 0 );

    data._cullContainer->addChild( _atmosphere.get() );
    data._lightPosUniform = osg::clone( _defaultPerViewData._lightPosUniform.get() );
    data._cullContainer->getOrCreateStateSet()->addUniform( data._lightPosUniform.get() );

    // node to traverse the child nodes
    data._cullContainer->addChild( new TraverseNode<osg::Group>(this) );

    view->setLightingMode( osg::View::SKY_LIGHT );
    view->setLight( data._light.get() );
    view->getCamera()->setClearColor( osg::Vec4(0,0,0,1) );

    data._date = _defaultPerViewData._date;
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
SkyNode::setAutoAmbience( bool value )
{
    _autoAmbience = value;
}

bool
SkyNode::getAutoAmbience() const
{
    return _autoAmbience;
}

void 
SkyNode::setAmbientBrightness( PerViewData& data, float value )
{
    value = osg::clampBetween( value, 0.0f, 1.0f );
    data._light->setAmbient( osg::Vec4f(value, value, value, 1.0f) );
    _autoAmbience = false;
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
SkyNode::setMoonPosition( const osg::Vec3d& pos, osg::View* view )
{
    _moonPosition = pos;
    if ( !view )
    {
        setMoonPosition( _defaultPerViewData, pos );
        for( PerViewDataMap::iterator i = _perViewData.begin(); i != _perViewData.end(); ++i )
            setMoonPosition( i->second, pos );
    }
    else if ( _perViewData.find(view) != _perViewData.end() )
    {
        setMoonPosition( _perViewData[view], pos );
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
SkyNode::setMoonPosition( PerViewData& data, const osg::Vec3d& pos )
{
    if ( data._moonXform.valid() )
    {
        data._moonMatrix = osg::Matrixd::translate( pos.x(), pos.y(), pos.z() );
        data._moonXform->setMatrix( data._moonMatrix );
    }
}


void
SkyNode::getDateTime( DateTime& out, osg::View* view ) const
{    
    if ( view )
    {
        PerViewDataMap::const_iterator i = _perViewData.find(view);
        if (i != _perViewData.end() )
        {
            out = i->second._date;
            return;
        }
    }
    out = _defaultPerViewData._date;
}

void
SkyNode::getDateTime(int& year, int& month, int& date, double& hoursUTC, osg::View* view)
{
    DateTime temp;
    getDateTime(temp, view);

    year = temp.year();
    month = temp.month();
    date = temp.day();
    hoursUTC = temp.hours();

    OE_WARN << LC <<
        "The method getDateTime(int&,int&,int&,double&,View*) is deprecated; "
        "please use getDateTime(DateTime&, View*) instead" << std::endl;
}


void
SkyNode::setDateTime(const DateTime& dt, osg::View* view)
{    
    if ( _ellipsoidModel.valid() )
    {
        osg::Vec3d sunPosition;
        osg::Vec3d moonPosition;

        if (_ephemerisProvider)
        {
            sunPosition = _ephemerisProvider->getSunPosition( dt );
            moonPosition = _ephemerisProvider->getMoonPosition( dt );
        }
        else
        {
            OE_NOTICE << "You must provide an EphemerisProvider" << std::endl;
        }

        sunPosition.normalize();
        setSunPosition( sunPosition, view );
        setMoonPosition( moonPosition, view );

        // position the stars:
        double time_r = dt.hours()/24.0; // 0..1
        double rot_z = -osg::PI + TWO_PI*time_r;

        osg::Matrixd starsMatrix = osg::Matrixd::rotate( -rot_z, 0, 0, 1 );
        if ( !view )
        {
            _defaultPerViewData._starsMatrix = starsMatrix;
            _defaultPerViewData._date = dt;

            for( PerViewDataMap::iterator i = _perViewData.begin(); i != _perViewData.end(); ++i )
            {
                i->second._starsMatrix = starsMatrix;
                i->second._starsXform->setMatrix( starsMatrix );
                i->second._date = dt;
            }
        }
        else if ( _perViewData.find(view) != _perViewData.end() )
        {
            PerViewData& data = _perViewData[view];
            data._starsMatrix = starsMatrix;
            data._starsXform->setMatrix( starsMatrix );
            data._date = dt;
        }
    }
}


void
SkyNode::setDateTime( int year, int month, int date, double hoursUTC, osg::View* view )
{
    // backwards compatibility
    setDateTime( DateTime(year, month, date, hoursUTC), view );

    OE_WARN << LC << 
        "The method setDateTime(int,int,int,double,View*) is deprecated; "
        "please use setDateTime(const DateTime&, View*) instead"
        << std::endl;
}


void
SkyNode::setStarsVisible( bool value, osg::View* view )
{
    if ( !view )
    {
        _defaultPerViewData._starsVisible = value;
        _defaultPerViewData._starsXform->setNodeMask( value ? ~0 : 0 );
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

void
SkyNode::setMoonVisible( bool value, osg::View* view )
{
    if ( !view )
    {
        _defaultPerViewData._moonVisible = value;
        _defaultPerViewData._moonXform->setNodeMask( value ? ~0 : 0 );
        for( PerViewDataMap::iterator i = _perViewData.begin(); i != _perViewData.end(); ++i )
        {
            i->second._moonVisible = value;
            i->second._moonXform->setNodeMask( value ? ~0 : 0 );
        }
    }
    else if ( _perViewData.find(view) != _perViewData.end() )
    {
        _perViewData[view]._moonVisible = value;
        _perViewData[view]._moonXform->setNodeMask( value ? ~0 : 0 );
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

bool
SkyNode::getMoonVisible( osg::View* view ) const
{
    PerViewDataMap::const_iterator i = _perViewData.find(view);

    if ( !view || i == _perViewData.end() )
    {
        return _defaultPerViewData._moonVisible;
    }
    else
    {
        return i->second._moonVisible;
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
    set->setAttributeAndModes( new osg::CullFace(osg::CullFace::BACK), osg::StateAttribute::ON );
    //set->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );
    set->setAttributeAndModes( new osg::Depth( osg::Depth::LESS, 0, 1, false ) ); // no depth write
    set->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS, 0, 1, false) ); // no zbuffer
    set->setAttributeAndModes( new osg::BlendFunc( GL_ONE, GL_ONE ), osg::StateAttribute::ON );

    if ( Registry::capabilities().supportsGLSL() )
    {
        // next, create and add the shaders:
        osg::Program* program = new osg::Program();
        osg::Shader* vs = new osg::Shader( osg::Shader::VERTEX, Stringify()
            << s_versionString
            << s_mathUtils
            << s_atmosphereVertexDeclarations
            << s_atmosphereVertexShared
            << s_atmosphereVertexMain );
        program->addShader( vs );

        osg::Shader* fs = new osg::Shader( osg::Shader::FRAGMENT, Stringify()
            << s_versionString
#ifdef OSG_GLES2_AVAILABLE
            << "precision highp float;\n"
#endif
            << s_mathUtils
            << s_atmosphereFragmentDeclarations
            //<< s_atmosphereFragmentShared
            << s_atmosphereFragmentMain );
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

        // TODO: replace this with a UBO.

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
    }
    
    // A nested camera isolates the projection matrix calculations so the node won't 
    // affect the clip planes in the rest of the scene.
    osg::Camera* cam = new osg::Camera();
    cam->getOrCreateStateSet()->setRenderBinDetails( BIN_ATMOSPHERE, "RenderBin" );
    cam->setRenderOrder( osg::Camera::NESTED_RENDER );
    cam->setComputeNearFarMode( osg::CullSettings::COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES );
    cam->addChild( geode );

    _atmosphere = cam;
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
    set->setMode( GL_BLEND, 1 );

    set->getOrCreateUniform( "sunAlpha", osg::Uniform::FLOAT )->set( 1.0f );

    // configure the stateset
    set->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    set->setMode( GL_CULL_FACE, osg::StateAttribute::OFF );
    set->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS, 0, 1, false), osg::StateAttribute::ON );
   // set->setAttributeAndModes( new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA), osg::StateAttribute::ON );

    // create shaders
    if ( Registry::capabilities().supportsGLSL() )
    {
        osg::Program* program = new osg::Program();
        osg::Shader* vs = new osg::Shader( osg::Shader::VERTEX, Stringify()
            << s_versionString
            << s_sunVertexSource );
        program->addShader( vs );
        osg::Shader* fs = new osg::Shader( osg::Shader::FRAGMENT, Stringify()
            << s_versionString
#ifdef OSG_GLES2_AVAILABLE
            << "precision highp float;\n"
#endif
            << s_mathUtils
            << s_sunFragmentSource );
        program->addShader( fs );
        set->setAttributeAndModes( program, osg::StateAttribute::ON );
    }

    // make the sun's transform:
    // todo: move this?
    _defaultPerViewData._sunXform = new osg::MatrixTransform();
    _defaultPerViewData._sunXform->setMatrix( osg::Matrix::translate( 
        _sunDistance * _defaultPerViewData._lightPos.x(), 
        _sunDistance * _defaultPerViewData._lightPos.y(), 
        _sunDistance * _defaultPerViewData._lightPos.z() ) );
    _defaultPerViewData._sunXform->addChild( sun );

    // A nested camera isolates the projection matrix calculations so the node won't 
    // affect the clip planes in the rest of the scene.
    osg::Camera* cam = new osg::Camera();
    cam->getOrCreateStateSet()->setRenderBinDetails( BIN_SUN, "RenderBin" );
    cam->setRenderOrder( osg::Camera::NESTED_RENDER );
    cam->setComputeNearFarMode( osg::CullSettings::COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES );
    cam->addChild( sun );

    _sun = cam;
}

void
SkyNode::makeMoon()
{
    osg::ref_ptr< osg::EllipsoidModel > em = new osg::EllipsoidModel( 1738140.0, 1735970.0 );   
    osg::Geode* moon = new osg::Geode;
    moon->getOrCreateStateSet()->setAttributeAndModes( new osg::Program(), osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );
    osg::Geometry* geom = s_makeEllipsoidGeometry( em.get(), em->getRadiusEquator(), true );    
    //TODO:  Embed this texture in code or provide a way to have a default resource directory for osgEarth.
    //       Right now just need to have this file somewhere in your OSG_FILE_PATH
    osg::Image* image = osgDB::readImageFile( "moon_1024x512.jpg" );
    osg::Texture2D * texture = new osg::Texture2D( image );
    texture->setFilter(osg::Texture::MIN_FILTER,osg::Texture::LINEAR);
    texture->setFilter(osg::Texture::MAG_FILTER,osg::Texture::LINEAR);
    texture->setResizeNonPowerOfTwoHint(false);
    geom->getOrCreateStateSet()->setTextureAttributeAndModes( 0, texture, osg::StateAttribute::ON | osg::StateAttribute::PROTECTED);
    
    osg::Vec4Array* colors = new osg::Vec4Array(1);    
    geom->setColorArray( colors );
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    (*colors)[0] = osg::Vec4(1, 1, 1, 1 );
    moon->addDrawable( geom  ); 
    
    osg::StateSet* set = moon->getOrCreateStateSet();
    // configure the stateset
    set->setMode( GL_LIGHTING, osg::StateAttribute::ON );
    set->setAttributeAndModes( new osg::CullFace( osg::CullFace::BACK ), osg::StateAttribute::ON);
    set->setRenderBinDetails( BIN_MOON, "RenderBin" );
    set->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS, 0, 1, false), osg::StateAttribute::ON );
    set->setAttributeAndModes( new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA), osg::StateAttribute::ON );

#ifdef OSG_GLES2_AVAILABLE
    
    if ( Registry::capabilities().supportsGLSL() )
    {
        set->addUniform(new osg::Uniform("moonTex", 0));
        
        // create shaders
        osg::Program* program = new osg::Program();
        osg::Shader* vs = new osg::Shader( osg::Shader::VERTEX, Stringify()
                                          << s_versionString
                                          << "precision highp float;\n"
                                          << s_moonVertexSource );
        program->addShader( vs );
        osg::Shader* fs = new osg::Shader( osg::Shader::FRAGMENT, Stringify()
                                          << s_versionString
                                          << "precision highp float;\n"
                                          << s_moonFragmentSource );
        program->addShader( fs );
        set->setAttributeAndModes( program, osg::StateAttribute::ON | osg::StateAttribute::PROTECTED );
    }
#endif
    
    // make the moon's transform:
    // todo: move this?
    _defaultPerViewData._moonXform = new osg::MatrixTransform();    

    Moon moonModel;
    //Get some default value of the moon
    osg::Vec3d pos = moonModel.getPosition( 2011, 2, 1, 0 );            
    _defaultPerViewData._moonXform->setMatrix( osg::Matrix::translate( pos ) ); 
    _defaultPerViewData._moonXform->addChild( moon );

    //If we couldn't load the moon texture, turn the moon off
    if (!image)
    {
        OE_INFO << LC << "Couldn't load moon texture, add osgEarth's data directory your OSG_FILE_PATH" << std::endl;
        _defaultPerViewData._moonXform->setNodeMask( 0 );
        _defaultPerViewData._moonVisible = false;
    }

    // A nested camera isolates the projection matrix calculations so the node won't 
    // affect the clip planes in the rest of the scene.
    osg::Camera* cam = new osg::Camera();
    cam->getOrCreateStateSet()->setRenderBinDetails( BIN_MOON, "RenderBin" );
    cam->setRenderOrder( osg::Camera::NESTED_RENDER );
    cam->setComputeNearFarMode( osg::CullSettings::COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES );
    cam->addChild( moon );

    _moon = cam;
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

  _stars = starNode;
}

osg::Node*
SkyNode::buildStarGeometry(const std::vector<StarData>& stars)
{
  double minMag = DBL_MAX, maxMag = DBL_MIN;

  osg::Vec3Array* coords = new osg::Vec3Array();
  std::vector<StarData>::const_iterator p;
  for( p = stars.begin(); p != stars.end(); p++ )
  {
    
    osg::Vec3d v = getPositionFromRADecl( p->right_ascension, p->declination, _starRadius );
    coords->push_back( v );

    if ( p->magnitude < minMag ) minMag = p->magnitude;
    if ( p->magnitude > maxMag ) maxMag = p->magnitude;
  }

  osg::Vec4Array* colors = new osg::Vec4Array();
  for( p = stars.begin(); p != stars.end(); p++ )
  {
      float c = ( (p->magnitude-minMag) / (maxMag-minMag) );
      colors->push_back( osg::Vec4(c,c,c,1.0f) );
  }

  osg::Geometry* geometry = new osg::Geometry;
  geometry->setUseVertexBufferObjects(true);

  geometry->setVertexArray( coords );
  geometry->setColorArray( colors );
  geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
  geometry->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, coords->size()));

  osg::StateSet* sset = geometry->getOrCreateStateSet();

  if ( Registry::capabilities().supportsGLSL() )
  {
    sset->setTextureAttributeAndModes( 0, new osg::PointSprite(), osg::StateAttribute::ON );
    sset->setMode( GL_VERTEX_PROGRAM_POINT_SIZE, osg::StateAttribute::ON );

    osg::Program* program = new osg::Program;
    program->addShader( new osg::Shader(osg::Shader::VERTEX, s_createStarVertexSource()) );
    program->addShader( new osg::Shader(osg::Shader::FRAGMENT, s_createStarFragmentSource()) );
    sset->setAttributeAndModes( program, osg::StateAttribute::ON );
  }

  sset->setRenderBinDetails( BIN_STARS, "RenderBin");
  sset->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS, 0, 1, false), osg::StateAttribute::ON );
  sset->setMode(GL_BLEND, 1);

  osg::Geode* starGeode = new osg::Geode;
  starGeode->addDrawable( geometry );

  // A separate camera isolates the projection matrix calculations.
  osg::Camera* cam = new osg::Camera();
  cam->getOrCreateStateSet()->setRenderBinDetails( BIN_STARS, "RenderBin" );
  cam->setRenderOrder( osg::Camera::NESTED_RENDER );
  cam->setComputeNearFarMode( osg::CullSettings::COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES );
  cam->addChild( starGeode );

  return cam;
  //return starGeode;
}

void
SkyNode::getDefaultStars(std::vector<StarData>& out_stars)
{
  out_stars.clear();

  for(const char **sptr = s_defaultStarData; *sptr; sptr++)
  {
    std::stringstream ss(*sptr);
    out_stars.push_back(StarData(ss));

    if (out_stars[out_stars.size() - 1].magnitude < _minStarMagnitude)
      out_stars.pop_back();
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

    if (out_stars[out_stars.size() - 1].magnitude < _minStarMagnitude)
      out_stars.pop_back();
  }

  in.close();

  return true;
}

osg::Vec3d
SkyNode::getPositionFromRADecl( double ra, double decl, double range )
{
    return osg::Vec3(0,range,0) * 
           osg::Matrix::rotate( decl, 1, 0, 0 ) * 
           osg::Matrix::rotate( ra - osg::PI_2, 0, 0, 1 );
}
