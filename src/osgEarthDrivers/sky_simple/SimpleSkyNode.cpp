/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2019 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include "SimpleSkyNode"
#include "SimpleSkyShaders"

#include <osgEarthUtil/StarData>
#include <osgEarthUtil/Ephemeris>

#include <osgEarth/VirtualProgram>
#include <osgEarth/NodeUtils>
#include <osgEarth/Map>
#include <osgEarth/Utils>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/CullingUtils>
#include <osgEarth/ShaderFactory>
#include <osgEarth/ShaderGenerator>
#include <osgEarth/Shaders>
#include <osgEarth/GLUtils>
#include <osgEarth/Lighting>
#include <osgEarth/PointDrawable>

#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/PointSprite>
#include <osg/PolygonMode>
#include <osg/Texture2D>
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

#define LC "[SimpleSkyNode] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::SimpleSky;

//---------------------------------------------------------------------------

#define BIN_STARS       -100003
#define BIN_SUN         -100002
#define BIN_MOON        -100001
#define BIN_ATMOSPHERE  -100000

#define TWO_PI 6.283185307179586476925286766559

//---------------------------------------------------------------------------

namespace
{
    // constucts an ellipsoidal mesh that we will use to draw the atmosphere
    osg::Geometry* s_makeEllipsoidGeometry(const osg::EllipsoidModel* ellipsoid, 
                                           double                     outerRadius, 
                                           bool                       genTexCoords)
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

            normals = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
            normals->reserve( latSegments * lonSegments );
            geom->setNormalArray( normals );
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
                    osg::Vec3d normal(gx, gy, gz);
                    normal.normalize();
                    normals->push_back( osg::Vec3f(normal) );
                }


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

    // makes a disc geometry that we'll use to render the sun/moon
    osg::Geometry* s_makeDiscGeometry(double radius)
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
            el->push_back( 1 + i );
            el->push_back( 1 + i_plus_1 );
        }

        return geom;
    }
}

//---------------------------------------------------------------------------

SimpleSkyNode::SimpleSkyNode(const SimpleSkyOptions& options) :
SkyNode ( options ),
_options( options )
{
    construct();
}

void
SimpleSkyNode::construct()
{
    // protect us from the ShaderGenerator.
    ShaderGenerator::setIgnoreHint(this, true);

    osg::Vec3f lightPos(0.0f, 1.0f, 0.0f);

    _light = new LightGL3( 0 );
    _light->setPosition( osg::Vec4f(0.0f, 0.0f, 1.0f, 0.0f) );
    _light->setAmbient ( osg::Vec4f(0.1f, 0.1f, 0.1f, 1.0f) );
    _light->setDiffuse ( osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f) );
    _light->setSpecular( osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f) );

    // install the Sun as a lightsource.
    osg::LightSource* lightSource = new osg::LightSource();
    lightSource->setLight(_light.get());
    lightSource->setCullingActive(false);
    this->addChild( lightSource );
    lightSource->addCullCallback(new LightSourceGL3UniformGenerator());

    if ( _options.ambient().isSet() )
    {
        float a = osg::clampBetween(_options.ambient().get(), 0.0f, 1.0f);
        _light->setAmbient(osg::Vec4(a, a, a, 1.0f));
    }

    // only supports geocentric for now.
    if (getReferencePoint().isValid())
    {
        OE_WARN << LC << "Found an ephemeris reference point, but SimpleSky does not support projected maps" << std::endl;
        return;
    }

    // containers for sky elements.
    _cullContainer = new osg::Group();
    
    // set up the astronomical parameters:
    osg::ref_ptr<const SpatialReference> wgs84 = SpatialReference::get("wgs84");
    _ellipsoidModel = wgs84->getEllipsoid();
    _innerRadius = osg::minimum(
        _ellipsoidModel->getRadiusPolar(),
        _ellipsoidModel->getRadiusEquator() );
    _outerRadius = _innerRadius * 1.025f;

    CelestialBody sun = getEphemeris()->getSunPosition(DateTime());
    _sunDistance = sun.altitude.as(Units::METERS);
    
    if ( Registry::capabilities().supportsGLSL() )
    {
        osg::StateSet* stateset = this->getOrCreateStateSet();

        _lightPosUniform = new osg::Uniform(osg::Uniform::FLOAT_VEC3, "atmos_v3LightDir");
        _lightPosUniform->set( lightPos / lightPos.length() );
        stateset->addUniform( _lightPosUniform.get() );

        stateset->setDefine(OE_LIGHTING_DEFINE, osg::StateAttribute::ON);

        // make the uniforms and the terrain lighting shaders.
        makeSceneLighting();

        // make the sky elements (don't change the order here)
        makeAtmosphere( _ellipsoidModel.get() );

        makeSun();

        makeMoon();

        makeStars();

        // Decorations are shown by default, so hide them as needed
        if (_options.sunVisible() == false) setSunVisible(false);
        if (_options.moonVisible() == false) setMoonVisible(false);
        if (_options.starsVisible() == false) setStarsVisible(false);
        if (_options.atmosphereVisible() == false) setAtmosphereVisible(false);
    }

    // Update everything based on the date/time.
    onSetDateTime();
}

osg::BoundingSphere
SimpleSkyNode::computeBound() const
{
    return osg::BoundingSphere();
}

void 
SimpleSkyNode::traverse( osg::NodeVisitor& nv ) 
{ 
    if ( nv.getVisitorType() == nv.CULL_VISITOR && _cullContainer.valid() ) 
    { 
        osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv); 

        bool needToRestoreInheritanceMask =
            (cv->getInheritanceMask() & osg::CullSettings::CLAMP_PROJECTION_MATRIX_CALLBACK) > 0; 

        // If there's a custom projection matrix clamper installed, remove it temporarily. 
        // We dont' want it mucking with our sky elements. 
        osg::ref_ptr<osg::CullSettings::ClampProjectionMatrixCallback> cb = 
            cv->getClampProjectionMatrixCallback(); 

        cv->setClampProjectionMatrixCallback( 0L ); 

        _cullContainer->accept( nv ); 

        // restore a custom clamper. 
        if ( cb.valid() ) 
        { 
            cv->setClampProjectionMatrixCallback( cb.get() ); 
        } 

        if (needToRestoreInheritanceMask) 
        { 
            cv->setInheritanceMask(
                cv->getInheritanceMask() | osg::CullSettings::CLAMP_PROJECTION_MATRIX_CALLBACK); 
        } 
    } 

    SkyNode::traverse( nv ); 
}

void
SimpleSkyNode::releaseGLObjects(osg::State* state) const
{
    SkyNode::releaseGLObjects(state);
    if (_cullContainer.valid())
        _cullContainer->releaseGLObjects(state);
}

void
SimpleSkyNode::resizeGLObjectBuffers(unsigned maxSize)
{
    SkyNode::resizeGLObjectBuffers(maxSize);
    if (_cullContainer.valid())
        _cullContainer->resizeGLObjectBuffers(maxSize);
}

void
SimpleSkyNode::onSetEphemeris()
{
    // trigger the date/time update.
    onSetDateTime();
}

void
SimpleSkyNode::onSetDateTime()
{
    if ( _ellipsoidModel.valid() )
    {
        osg::View* view = 0L;
        const DateTime& dt = getDateTime();

        CelestialBody sun = getEphemeris()->getSunPosition(dt);
        setSunPosition( sun.geocentric );

        CelestialBody moon = getEphemeris()->getMoonPosition(dt);
        setMoonPosition( moon.geocentric );

        // position the stars:
        double time_r = dt.hours()/24.0; // 0..1
        double rot_z = -osg::PI + TWO_PI*time_r;

        if ( _starsXform.valid() )
            _starsXform->setMatrix( osg::Matrixd::rotate(-rot_z, 0, 0, 1) );
    }
}

void
SimpleSkyNode::attach( osg::View* view, int lightNum )
{
    if ( !view || !_light.valid() )
        return;

    _light->setLightNum( lightNum );

    // black background
    view->getCamera()->setClearColor( osg::Vec4(0,0,0,1) );
    
    // install the light in the view (so other modules can access it, like shadowing)
    view->setLight(_light.get());

    // Tell the view not to automatically include a light.
    view->setLightingMode( osg::View::NO_LIGHT );


    onSetDateTime();
}

void
SimpleSkyNode::setSunPosition(const osg::Vec3d& pos)
{
    osg::Vec3d npos = pos;
    _light->setPosition(osg::Vec4(npos, 0.0f)); // directional light
    npos.normalize();

    //OE_NOTICE << pos.x() << ", " << pos.y() << ", " << pos.z() << std::endl;
    
    if ( _lightPosUniform.valid() )
    {
        _lightPosUniform->set( osg::Vec3(npos) );
    }

    if ( _sunXform.valid() )
    {
        _sunXform->setMatrix( osg::Matrix::translate(pos) );

        if (_moonXform.valid())
        {
            osg::Vec3d moonToSun = _sunXform->getMatrix().getTrans() - _moonXform->getMatrix().getTrans();
            moonToSun.normalize();
            _moonXform->getOrCreateStateSet()->getOrCreateUniform("moonToSun", osg::Uniform::FLOAT_VEC3)->set(osg::Vec3f(moonToSun));
        }
    }
}

void
SimpleSkyNode::setMoonPosition(const osg::Vec3d& pos)
{
    if (_moonXform.valid())
    {
        _moonXform->setMatrix(osg::Matrixd::translate(pos));
        
        if (_sunXform.valid())
        {
            osg::Vec3d moonToSun = _sunXform->getMatrix().getTrans() - _moonXform->getMatrix().getTrans();
            moonToSun.normalize();
            _moonXform->getOrCreateStateSet()->getOrCreateUniform("moonToSun", osg::Uniform::FLOAT_VEC3)->set(osg::Vec3f(moonToSun));
        }
    }
}

void
SimpleSkyNode::onSetStarsVisible()
{
    if ( _starsXform.valid() )
        _starsXform->setNodeMask( getStarsVisible() ? ~0 : 0 );
}

void
SimpleSkyNode::onSetMoonVisible()
{
    if ( _moonXform.valid() )
        _moonXform->setNodeMask( getMoonVisible() ? ~0 : 0 );
}

void
SimpleSkyNode::onSetSunVisible()
{
    if ( _sunXform.valid() )
        _sunXform->setNodeMask( getSunVisible() ? ~0 : 0 );
}

void
SimpleSkyNode::onSetAtmosphereVisible()
{
    if (_atmosphere.valid())
        _atmosphere->setNodeMask( getAtmosphereVisible() ? ~0 : 0 );
}

void
SimpleSkyNode::makeSceneLighting()
{
    // installs the main uniforms and the shaders that will light the subgraph (terrain).
    osg::StateSet* stateset = this->getOrCreateStateSet();

    VirtualProgram* vp = VirtualProgram::getOrCreate( stateset );
    vp->setName( "SimpleSky Scene Lighting" );

    if (_options.atmosphericLighting() == true)
    {
        Shaders pkg;
        pkg.load( vp, pkg.Ground_ONeil_Vert );
        pkg.load( vp, pkg.Ground_ONeil_Frag );
        OE_INFO << LC << "Using O'Neil atmospheric lighting\n";
    }

    else
    {
        _phong = new PhongLightingEffect();
        _phong->attach( stateset );
        OE_INFO << LC << "Using Phong lighting\n";
    }

    // calculate and apply the uniforms:
    // TODO: perhaps we can just hard-code most of these as GLSL consts.
    float r_wl = ::powf( .65f, 4.0f );
    float g_wl = ::powf( .57f, 4.0f );
    float b_wl = ::powf( .475f, 4.0f );
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

    //TODO: make all these constants. -gw
    stateset->getOrCreateUniform( "atmos_v3InvWavelength", osg::Uniform::FLOAT_VEC3 )->set( RGB_wl );
    stateset->getOrCreateUniform( "atmos_fInnerRadius",    osg::Uniform::FLOAT )->set( _innerRadius );
    stateset->getOrCreateUniform( "atmos_fInnerRadius2",   osg::Uniform::FLOAT )->set( _innerRadius * _innerRadius );
    stateset->getOrCreateUniform( "atmos_fOuterRadius",    osg::Uniform::FLOAT )->set( _outerRadius );
    stateset->getOrCreateUniform( "atmos_fOuterRadius2",   osg::Uniform::FLOAT )->set( _outerRadius * _outerRadius );
    stateset->getOrCreateUniform( "atmos_fKrESun",         osg::Uniform::FLOAT )->set( Kr * ESun );
    stateset->getOrCreateUniform( "atmos_fKmESun",         osg::Uniform::FLOAT )->set( Km * ESun );
    stateset->getOrCreateUniform( "atmos_fKr4PI",          osg::Uniform::FLOAT )->set( Kr4PI );
    stateset->getOrCreateUniform( "atmos_fKm4PI",          osg::Uniform::FLOAT )->set( Km4PI );
    stateset->getOrCreateUniform( "atmos_fScale",          osg::Uniform::FLOAT )->set( Scale );
    stateset->getOrCreateUniform( "atmos_fScaleDepth",     osg::Uniform::FLOAT )->set( RayleighScaleDepth );
    stateset->getOrCreateUniform( "atmos_fScaleOverScaleDepth", osg::Uniform::FLOAT )->set( Scale / RayleighScaleDepth );
    stateset->getOrCreateUniform( "atmos_g",               osg::Uniform::FLOAT )->set( MPhase );
    stateset->getOrCreateUniform( "atmos_g2",              osg::Uniform::FLOAT )->set( MPhase * MPhase );
    stateset->getOrCreateUniform( "atmos_nSamples",        osg::Uniform::INT )->set( Samples );
    stateset->getOrCreateUniform( "atmos_fSamples",        osg::Uniform::FLOAT )->set( (float)Samples );
    stateset->getOrCreateUniform( "atmos_fWeather",        osg::Uniform::FLOAT )->set( Weather );

    // options:
    stateset->getOrCreateUniform("oe_sky_exposure",           osg::Uniform::FLOAT )->set( _options.exposure().value() );
    stateset->getOrCreateUniform("oe_sky_ambientBoostFactor", osg::Uniform::FLOAT)->set(_options.daytimeAmbientBoost().get());
}

void
SimpleSkyNode::makeAtmosphere(const osg::EllipsoidModel* em)
{
    // create some skeleton geometry to shade:
    osg::Geometry* drawable = s_makeEllipsoidGeometry( em, _outerRadius, false );

    // disable wireframe/point rendering on the atmosphere, since it is distracting.
    if ( _options.allowWireframe() == false )
    {
        drawable->getOrCreateStateSet()->setAttributeAndModes(
            new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL),
            osg::StateAttribute::PROTECTED);
    }

    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( drawable );
    
    // configure the state set:
    osg::StateSet* atmosSet = drawable->getOrCreateStateSet();
    GLUtils::setLighting(atmosSet, osg::StateAttribute::OFF);
    atmosSet->setAttributeAndModes( new osg::CullFace(osg::CullFace::FRONT), osg::StateAttribute::ON );
    atmosSet->setAttributeAndModes( new osg::Depth( osg::Depth::LESS, 0, 1, false ) ); // no depth write
    atmosSet->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS, 0, 1, false) ); // no zbuffer
    atmosSet->setAttributeAndModes( new osg::BlendFunc( GL_ONE, GL_ONE ), osg::StateAttribute::ON );

    // first install the atmosphere rendering shaders.
    if ( Registry::capabilities().supportsGLSL() )
    {
        VirtualProgram* vp = VirtualProgram::getOrCreate( atmosSet );
        vp->setName( "SimpleSky Atmosphere" );
        vp->setInheritShaders( false );

        Shaders pkg;
        pkg.load( vp, pkg.Atmosphere_Vert );
        pkg.load( vp, pkg.Atmosphere_Frag );
    }

    // A nested camera isolates the projection matrix calculations so the node won't 
    // affect the clip planes in the rest of the scene.
    osg::Camera* cam = new osg::Camera();
    cam->getOrCreateStateSet()->setRenderBinDetails( BIN_ATMOSPHERE, "RenderBin" );
    cam->setRenderOrder( osg::Camera::NESTED_RENDER );
    cam->setComputeNearFarMode( osg::CullSettings::COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES );
    cam->addChild( geode );

    _atmosphere = cam;

    _cullContainer->addChild( _atmosphere.get() );
}

void
SimpleSkyNode::makeSun()
{
    osg::Billboard* sun = new osg::Billboard();
    sun->setMode( osg::Billboard::POINT_ROT_EYE );
    sun->setNormal( osg::Vec3(0, 0, 1) );

    //float sunRadius = _innerRadius * 100.0f;
    //sun->addDrawable( s_makeDiscGeometry( sunRadius*80.0f ) ); 

    const double zoomFactor = 80.0; // to account for the solare glare
    const double sunRadius = 695508000.0; // radius of the run in meters
    sun->addDrawable(s_makeDiscGeometry(sunRadius*zoomFactor));

    osg::StateSet* set = sun->getOrCreateStateSet();
    set->setMode( GL_BLEND, 1 );

    // configure the stateset
    set->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS, 0, 1, false), osg::StateAttribute::ON );

    // create shaders
    Shaders pkg;
    osg::Program* program = new osg::Program();

    osg::Shader* vs = new osg::Shader(
        osg::Shader::VERTEX,
        ShaderLoader::load(pkg.Sun_Vert, pkg) );
    program->addShader( vs );

    osg::Shader* fs = new osg::Shader(
        osg::Shader::FRAGMENT,
        ShaderLoader::load(pkg.Sun_Frag, pkg) );
    program->addShader( fs );

    set->setAttributeAndModes( program, osg::StateAttribute::ON | osg::StateAttribute::PROTECTED );

    // A nested camera isolates the projection matrix calculations so the node won't 
    // affect the clip planes in the rest of the scene.
    osg::Camera* cam = new osg::Camera();
    cam->getOrCreateStateSet()->setRenderBinDetails( BIN_SUN, "RenderBin" );
    cam->setRenderOrder( osg::Camera::NESTED_RENDER );
    cam->setComputeNearFarMode( osg::CullSettings::COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES );
    cam->addChild( sun );

    _sun = cam;

    // make the sun's transform:
    _sunXform = new osg::MatrixTransform();
    _sunXform->setMatrix( osg::Matrix::translate( 
        _sunDistance * _light->getPosition().x(),
        _sunDistance * _light->getPosition().y(),
        _sunDistance * _light->getPosition().z() ) );
    _sunXform->addChild( _sun.get() );

    _cullContainer->addChild( _sunXform.get() );
}

void
SimpleSkyNode::makeMoon()
{
    osg::ref_ptr< osg::EllipsoidModel > em = new osg::EllipsoidModel( 1738140.0, 1735970.0 );   
    
    osg::Geometry* moonDrawable = s_makeEllipsoidGeometry( em.get(), em->getRadiusEquator()*_options.moonScale().get(), true );    
    osg::StateSet* stateSet = moonDrawable->getOrCreateStateSet();

    //TODO:  Embed this texture in code or provide a way to have a default resource directory for osgEarth.
    //       Right now just need to have this file somewhere in your OSG_FILE_PATH
    //stateSet->setAttributeAndModes( new osg::Program(), osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );
    osg::ref_ptr<osg::Image> image = _options.moonImageURI()->getImage();
    if (!image.valid())
    {
        OE_WARN << LC << "Failed to load moon texture from " << _options.moonImageURI()->full() << std::endl;
    }

    osg::Texture2D* texture = new osg::Texture2D( image.get() );
    texture->setFilter(osg::Texture::MIN_FILTER,osg::Texture::LINEAR);
    texture->setFilter(osg::Texture::MAG_FILTER,osg::Texture::LINEAR);
    texture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
    texture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
    texture->setResizeNonPowerOfTwoHint(false);
    stateSet->setTextureAttributeAndModes( 0, texture, osg::StateAttribute::ON | osg::StateAttribute::PROTECTED);
#ifdef OSG_GL3_AVAILABLE
    // Adjust for loss of GL_LUMINANCE in glTexture2D's format parameter.  OSG handles the texture's internal format,
    // but the format parameter comes from the image's pixel format field.
    if (image.valid() && image->getPixelFormat() == GL_LUMINANCE)
    {
      image->setPixelFormat(GL_RED);
      // Swizzle the RGB all to RED in order to match previous GL_LUMINANCE behavior
      texture->setSwizzle(osg::Vec4i(GL_RED, GL_RED, GL_RED, GL_ONE));
    }
#endif

    osg::Vec4Array* colors = new osg::Vec4Array(osg::Array::BIND_OVERALL, 1);    
    moonDrawable->setColorArray( colors );
    (*colors)[0] = osg::Vec4(1, 1, 1, 1 );

    // configure the stateset
    //GLUtils::setLighting(stateSet, osg::StateAttribute::ON );

    stateSet->setAttributeAndModes( new osg::CullFace( osg::CullFace::BACK ), osg::StateAttribute::ON);
    stateSet->setRenderBinDetails( BIN_MOON, "RenderBin" );
    stateSet->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS, 0, 1, false), osg::StateAttribute::ON );
    stateSet->setAttributeAndModes( new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA), osg::StateAttribute::ON );
    
#if 1
    stateSet->addUniform(new osg::Uniform("moonTex", 0));

    // create shaders
    Shaders pkg;
    osg::Program* program = new osg::Program();
    osg::Shader* vs = new osg::Shader(
        osg::Shader::VERTEX,
        ShaderLoader::load(pkg.Moon_Vert, pkg) );
    program->addShader( vs );
    osg::Shader* fs = new osg::Shader(
        osg::Shader::FRAGMENT,
        ShaderLoader::load(pkg.Moon_Frag, pkg) );
    program->addShader( fs );
    stateSet->setAttributeAndModes( program, osg::StateAttribute::ON | osg::StateAttribute::PROTECTED );

#else

    VirtualProgram* vp = VirtualProgram::getOrCreate(stateSet);

    const char* VS =
        "#version 330\n"
        "#pragma import_defines(OE_NUM_LIGHTS) \n"
        "out vec2 oe_SimpleSky_moonTexCoord;\n"
        "out float oe_SimpleSky_lighting;\n"
        "struct osg_LightSourceParameters  \n"
        "{    \n"
           "vec4 ambient;              // Aclarri    \n"
           "vec4 diffuse;              // Dcli    \n"
           "vec4 specular;             // Scli    \n"
           "vec4 position;             // Ppli    \n"
           "//vec4 halfVector;           // Derived: Hi    \n"
           "vec3 spotDirection;        // Sdli    \n"
           "float spotExponent;        // Srli    \n"
           "float spotCutoff;          // Crli                               \n"
                                      "// (range: [0.0,90.0], 180.0)    \n"
           "float spotCosCutoff;       // Derived: cos(Crli)                  \n"
                                      "// (range: [1.0,0.0],-1.0)    \n"
           "float constantAttenuation; // K0    \n"
           "float linearAttenuation;   // K1    \n"
           "float quadraticAttenuation;// K2   \n"

           "bool enabled; \n"
        "};   \n"
        "uniform osg_LightSourceParameters osg_LightSource[OE_NUM_LIGHTS]; \n"
        "void oe_SimpleSky_moonVS(inout vec4 vertexModel) { \n"
        "    oe_SimpleSky_moonTexCoord = gl_MultiTexCoord0.st; \n"
        "    oe_SimpleSky_lighting = 1.0; \n"
        "} \n";

    const char* FS =
        "#version 330\n"
        "in vec2 oe_SimpleSky_moonTexCoord;\n"
        "in float oe_SimpleSky_lighting;\n"
        "uniform sampler2D oe_SimpleSky_moonTex; \n"
        "void oe_SimpleSky_moonFS(inout vec4 color) { \n"
        "    color = texture(oe_SimpleSky_moonTex, oe_SimpleSky_moonTexCoord); \n"
        "    color.rgb *= oe_SimpleSky_lighting; \n"
        "} \n";


    vp->setFunction("oe_SimpleSky_moonVS", VS, ShaderComp::LOCATION_VERTEX_MODEL);
    vp->setFunction("oe_SimpleSky_moonFS", FS, ShaderComp::LOCATION_FRAGMENT_COLORING);

    stateSet->addUniform(new osg::Uniform("oe_SimpleSky_moonTex", 0));

#endif

    // A nested camera isolates the projection matrix calculations so the node won't 
    // affect the clip planes in the rest of the scene.
    osg::Camera* cam = new osg::Camera();
    cam->getOrCreateStateSet()->setRenderBinDetails( BIN_MOON, "RenderBin" );
    cam->setRenderOrder( osg::Camera::NESTED_RENDER );
    cam->setComputeNearFarMode( osg::CullSettings::COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES );
    cam->addChild( moonDrawable );

    _moon = cam;

    // make the moon's transform:
    CelestialBody moon = getEphemeris()->getMoonPosition(getDateTime());

    _moonXform = new osg::MatrixTransform();   
    _moonXform->setMatrix( osg::Matrix::translate( moon.geocentric ) ); 
    _moonXform->addChild( _moon.get() );

    //_moonXform->getOrCreateStateSet()->addUniform(new osg::Uniform("moonToSun", osg::Vec3f(0,0,1)));

    _cullContainer->addChild( _moonXform.get() );

    //If we couldn't load the moon texture, turn the moon off
    if (!image)
    {
        OE_INFO << LC << "Couldn't load moon texture, add osgEarth's data directory your OSG_FILE_PATH" << std::endl;
        //_moonXform->setNodeMask( 0 );
        setMoonVisible(false);
    }
}

SimpleSkyNode::StarData::StarData(std::stringstream &ss) :
right_ascension(0.0),
declination(0.0),
magnitude(0.0)
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
SimpleSkyNode::makeStars()
{
    const char* magEnv = ::getenv("OSGEARTH_MIN_STAR_MAGNITUDE");
    if (magEnv)
        _minStarMagnitude = as<float>(std::string(magEnv), -1.0f);
    else
        _minStarMagnitude = -1.0f;

    _starRadius = 20000.0 * (_sunDistance > 0.0 ? _sunDistance : _outerRadius);

    std::vector<StarData> stars;

    if( _options.starFile().isSet() )
    {
        if ( parseStarFile(*_options.starFile(), stars) == false )
        {
            OE_WARN << LC 
                << "Unable to use star field defined in \"" << *_options.starFile()
                << "\", using default star data instead." << std::endl;
        }
    }

    if ( stars.empty() )
    {
        getDefaultStars( stars );
    }

    _stars = buildStarGeometry(stars);

    // make the stars' transform:
    _starsXform = new osg::MatrixTransform();
    _starsXform->addChild( _stars.get() );

    _cullContainer->addChild( _starsXform.get() );
}

osg::Node*
SimpleSkyNode::buildStarGeometry(const std::vector<StarData>& stars)
{
    double minMag = DBL_MAX, maxMag = DBL_MIN;

    PointDrawable* drawable = new PointDrawable();
    drawable->setPointSize(_options.starSize().get());
    drawable->allocate(stars.size());

    for(unsigned p=0; p<stars.size(); ++p)
    {
        const StarData& star = stars[p];

        osg::Vec3d v = getEphemeris()->getECEFfromRADecl(
            star.right_ascension, 
            star.declination, 
            _starRadius );

        drawable->setVertex(p, v);

        if ( star.magnitude < minMag ) minMag = star.magnitude;
        if ( star.magnitude > maxMag ) maxMag = star.magnitude;
    }

    for(unsigned p=0; p<stars.size(); ++p)
    {
        const StarData& star = stars[p];
        float c = (star.magnitude-minMag) / (maxMag-minMag);
        drawable->setColor(p, osg::Vec4(c,c,c,1.0f));
    }

    drawable->finish();

    osg::StateSet* sset = drawable->getOrCreateStateSet();

    const osgEarth::Capabilities& caps = osgEarth::Registry::capabilities();

    VirtualProgram* vp = VirtualProgram::getOrCreate(drawable->getOrCreateStateSet());
    vp->setName("SimpleSky Stars");
    Shaders shaders;
    shaders.load(vp, shaders.Stars_Vert);
    shaders.load(vp, shaders.Stars_Frag);
    vp->setInheritShaders(false);

    sset->setRenderBinDetails( BIN_STARS, "RenderBin");
    sset->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS, 0, 1, false), osg::StateAttribute::ON );
    sset->setMode(GL_BLEND, 1);

    // A separate camera isolates the projection matrix calculations.
    osg::Camera* cam = new osg::Camera();
    cam->getOrCreateStateSet()->setRenderBinDetails( BIN_STARS, "RenderBin" );
    cam->setRenderOrder( osg::Camera::NESTED_RENDER );

    cam->addChild( drawable );
    return cam;
}

void
SimpleSkyNode::getDefaultStars(std::vector<StarData>& out_stars)
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
SimpleSkyNode::parseStarFile(const std::string& starFile, std::vector<StarData>& out_stars)
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
