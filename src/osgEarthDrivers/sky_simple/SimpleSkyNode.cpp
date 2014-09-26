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

#define LC "[SimpleSkyNode] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Drivers::SimpleSky;

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
            el->push_back( 1 + i_plus_1 );
            el->push_back( 1 + i );
        }

        return geom;
    }
}

//---------------------------------------------------------------------------

SimpleSkyNode::SimpleSkyNode(const SpatialReference* srs) :
SkyNode()
{
    initialize(srs);
}

SimpleSkyNode::SimpleSkyNode(const SpatialReference* srs,
                             const SimpleSkyOptions& options) :
SkyNode ( options ),
_options( options )
{
    initialize(srs);
}

void
SimpleSkyNode::initialize(const SpatialReference* srs)
{
    // protect us from the ShaderGenerator.
    ShaderGenerator::setIgnoreHint(this, true);

    osg::Vec3f lightPos(0.0f, 1.0f, 0.0f);

    _light = new osg::Light( 0 );
    _light->setPosition( osg::Vec4f(0.0f, 0.0f, 1.0, 0.0f) );
    _light->setAmbient ( osg::Vec4f(0.03f, 0.03f, 0.03f, 1.0f) );
    _light->setDiffuse ( osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f) );
    _light->setSpecular( osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f) );

    if ( _options.ambient().isSet() )
    {
        float a = osg::clampBetween(_options.ambient().get(), 0.0f, 1.0f);
        _light->setAmbient(osg::Vec4(a, a, a, 1.0f));
    }

    // only supports geocentric for now.
    if ( srs && !srs->isGeographic() )
    {
        OE_WARN << LC << "Sorry, SimpleSky only supports geocentric maps." << std::endl;
        return;
    }

    // containers for sky elements.
    _cullContainer = new osg::Group();
    
    // set up the astronomical parameters:
    _ellipsoidModel = srs->getEllipsoid();
    _innerRadius = osg::minimum(
        _ellipsoidModel->getRadiusPolar(),
        _ellipsoidModel->getRadiusEquator() );
    _outerRadius = _innerRadius * 1.025f;
    _sunDistance = _innerRadius * 12000.0f;
    
    if ( Registry::capabilities().supportsGLSL() )
    {
        _lightPosUniform = new osg::Uniform(osg::Uniform::FLOAT_VEC3, "atmos_v3LightDir");
        _lightPosUniform->set( lightPos / lightPos.length() );
        this->getOrCreateStateSet()->addUniform( _lightPosUniform.get() );

        // default GL_LIGHTING uniform setting
        this->getOrCreateStateSet()->addUniform(
            Registry::shaderFactory()->createUniformForGLMode(GL_LIGHTING, 1) );

        // make the uniforms and the terrain lighting shaders.
        makeSceneLighting();

        // make the sky elements (don't change the order here)
        makeAtmosphere( _ellipsoidModel.get() );

        makeSun();

        makeMoon();

        makeStars();
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

        osg::Vec3d sunPos = getEphemeris()->getSunPositionECEF( dt );
        osg::Vec3d moonPos = getEphemeris()->getMoonPositionECEF( dt );

        sunPos.normalize();
        setSunPosition( sunPos );
        setMoonPosition( moonPos );

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
    view->setLight( _light.get() );
    view->setLightingMode( osg::View::SKY_LIGHT );
    view->getCamera()->setClearColor( osg::Vec4(0,0,0,1) );

    onSetDateTime();
}

void
SimpleSkyNode::setSunPosition(const osg::Vec3& pos)
{
    _light->setPosition( osg::Vec4(pos, 0.0f) );
    
    if ( _lightPosUniform.valid() )
    {
        _lightPosUniform->set( pos/pos.length() );
    }

    if ( _sunXform.valid() )
    {
        _sunXform->setMatrix( osg::Matrix::translate( 
            _sunDistance * pos.x(), 
            _sunDistance * pos.y(),
            _sunDistance * pos.z() ) );
    }
}

void
SimpleSkyNode::setMoonPosition(const osg::Vec3d& pos)
{
    if ( _moonXform.valid() )
        _moonXform->setMatrix( osg::Matrixd::translate(pos.x(), pos.y(), pos.z()) );
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
SimpleSkyNode::makeSceneLighting()
{
    // installs the main uniforms and the shaders that will light the subgraph (terrain).
    osg::StateSet* stateset = this->getOrCreateStateSet();

    VirtualProgram* vp = VirtualProgram::getOrCreate( stateset );
    vp->setName( "SimpleSky Scene Lighting" );

    if ( _options.atmosphericLighting() == true )
    {
        vp->setFunction(
            "atmos_vertex_main",
            Ground_Scattering_Vertex,
            ShaderComp::LOCATION_VERTEX_VIEW);

        vp->setFunction(
            "atmos_fragment_main", 
            Ground_Scattering_Fragment,
            ShaderComp::LOCATION_FRAGMENT_LIGHTING);
    }

    else
    {
        _phong = new PhongLightingEffect();
        _phong->setCreateLightingUniform( false );
        _phong->attach( stateset );
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
    stateset->getOrCreateUniform( "atmos_exposure",        osg::Uniform::FLOAT )->set( _options.exposure().value() );
}

void
SimpleSkyNode::makeAtmosphere(const osg::EllipsoidModel* em)
{
    // create some skeleton geometry to shade:
    osg::Geometry* drawable = s_makeEllipsoidGeometry( em, _outerRadius, false );

    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( drawable );
    
    // configure the state set:
    osg::StateSet* atmosSet = drawable->getOrCreateStateSet();
    atmosSet->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    atmosSet->setAttributeAndModes( new osg::CullFace(osg::CullFace::BACK), osg::StateAttribute::ON );
    atmosSet->setAttributeAndModes( new osg::Depth( osg::Depth::LESS, 0, 1, false ) ); // no depth write
    atmosSet->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS, 0, 1, false) ); // no zbuffer
    atmosSet->setAttributeAndModes( new osg::BlendFunc( GL_ONE, GL_ONE ), osg::StateAttribute::ON );

    // first install the atmosphere rendering shaders.
    if ( Registry::capabilities().supportsGLSL() )
    {
        VirtualProgram* vp = VirtualProgram::getOrCreate( atmosSet );
        vp->setName( "SimpleSky Atmosphere" );
        vp->setInheritShaders( false );

        vp->setFunction(
            "atmos_vertex_main",
            Atmosphere_Vertex,
            ShaderComp::LOCATION_VERTEX_VIEW);

        vp->setFunction(
            "atmos_fragment_main",
            Atmosphere_Fragment,
            ShaderComp::LOCATION_FRAGMENT_LIGHTING);
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

    float sunRadius = _innerRadius * 100.0f;

    sun->addDrawable( s_makeDiscGeometry( sunRadius*80.0f ) ); 

    osg::StateSet* set = sun->getOrCreateStateSet();
    set->setMode( GL_BLEND, 1 );

    set->getOrCreateUniform( "atmos_sunAlpha", osg::Uniform::FLOAT )->set( 1.0f );

    // configure the stateset
    set->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    set->setMode( GL_CULL_FACE, osg::StateAttribute::OFF );
    set->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS, 0, 1, false), osg::StateAttribute::ON );

    // create shaders
    osg::Program* program = new osg::Program();
    osg::Shader* vs = new osg::Shader( osg::Shader::VERTEX, Sun_Vertex );
    program->addShader( vs );
    osg::Shader* fs = new osg::Shader( osg::Shader::FRAGMENT, Sun_Fragment );
    program->addShader( fs );
    set->setAttributeAndModes( program, osg::StateAttribute::ON );

    // A nested camera isolates the projection matrix calculations so the node won't 
    // affect the clip planes in the rest of the scene.
    osg::Camera* cam = new osg::Camera();
    cam->getOrCreateStateSet()->setRenderBinDetails( BIN_SUN, "RenderBin" );
    cam->setRenderOrder( osg::Camera::NESTED_RENDER );
    cam->setComputeNearFarMode( osg::CullSettings::COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES );
    cam->addChild( sun );

    _sun = cam;

    // make the sun's transform:
    // todo: move this?
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

    set->addUniform(new osg::Uniform("moonTex", 0));

    // create shaders
    osg::Program* program = new osg::Program();
    osg::Shader* vs = new osg::Shader( osg::Shader::VERTEX, Moon_Vertex );
    program->addShader( vs );
    osg::Shader* fs = new osg::Shader( osg::Shader::FRAGMENT, Moon_Fragment );
    program->addShader( fs );
    set->setAttributeAndModes( program, osg::StateAttribute::ON | osg::StateAttribute::PROTECTED );
#endif

    // A nested camera isolates the projection matrix calculations so the node won't 
    // affect the clip planes in the rest of the scene.
    osg::Camera* cam = new osg::Camera();
    cam->getOrCreateStateSet()->setRenderBinDetails( BIN_MOON, "RenderBin" );
    cam->setRenderOrder( osg::Camera::NESTED_RENDER );
    cam->setComputeNearFarMode( osg::CullSettings::COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES );
    cam->addChild( moon );

    _moon = cam;

    // make the moon's transform:
    _moonXform = new osg::MatrixTransform();    
    osg::Vec3d moonPosECEF = getEphemeris()->getMoonPositionECEF( getDateTime() );
    _moonXform->setMatrix( osg::Matrix::translate( moonPosECEF ) ); 
    _moonXform->addChild( _moon.get() );

    _cullContainer->addChild( _moonXform.get() );

    //If we couldn't load the moon texture, turn the moon off
    if (!image)
    {
        OE_INFO << LC << "Couldn't load moon texture, add osgEarth's data directory your OSG_FILE_PATH" << std::endl;
        //_moonXform->setNodeMask( 0 );
        setMoonVisible(false);
    }
}

SimpleSkyNode::StarData::StarData(std::stringstream &ss)
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

    // make the moon's transform:
    _starsXform = new osg::MatrixTransform();
    _starsXform->addChild( _stars.get() );

    _cullContainer->addChild( _starsXform.get() );
}

osg::Node*
SimpleSkyNode::buildStarGeometry(const std::vector<StarData>& stars)
{
    double minMag = DBL_MAX, maxMag = DBL_MIN;

    osg::Vec3Array* coords = new osg::Vec3Array();
    std::vector<StarData>::const_iterator p;
    for( p = stars.begin(); p != stars.end(); p++ )
    {
        osg::Vec3d v = getEphemeris()->getECEFfromRADecl(
            p->right_ascension, 
            p->declination, 
            _starRadius );

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

    sset->setTextureAttributeAndModes( 0, new osg::PointSprite(), osg::StateAttribute::ON );
    sset->setMode( GL_VERTEX_PROGRAM_POINT_SIZE, osg::StateAttribute::ON );

    std::string starVertSource, starFragSource;
    if ( Registry::capabilities().getGLSLVersion() < 1.2f )
    {
        starVertSource = Stars_Vertex_110;
        starFragSource = Stars_Fragment_110;
    }
    else
    {
        starVertSource = Stars_Vertex_120;
        starFragSource = Stars_Fragment_120;
    }

    osg::Program* program = new osg::Program;
    program->addShader( new osg::Shader(osg::Shader::VERTEX, starVertSource) );
    program->addShader( new osg::Shader(osg::Shader::FRAGMENT, starFragSource) );
    sset->setAttributeAndModes( program, osg::StateAttribute::ON );

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
