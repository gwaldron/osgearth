/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2015 Pelican Mapping
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
#include <SilverLining.h> // SilverLinking SDK
#include "SilverLiningContext"
#include <osg/Light>
#include <osgEarth/SpatialReference>

#define LC "[SilverLiningContext] "

using namespace osgEarth::SilverLining;


SilverLiningContext::SilverLiningContext(const SilverLiningOptions& options) :
_options              ( options ),
_initAttempted        ( false ),
_initFailed           ( false ),
_maxAmbientLightingAlt( -1.0 ),
_atmosphere           ( 0L ),
_clouds               ( 0L ),
_minAmbient           ( 0,0,0,0 )
{
    // Create a SL atmosphere (the main SL object).
    // TODO: plug in the username + license key.
    _atmosphere = new ::SilverLining::Atmosphere(
        options.user()->c_str(),
        options.licenseCode()->c_str() );
}

SilverLiningContext::~SilverLiningContext()
{
    if ( _atmosphere )
        delete _atmosphere;

    OE_INFO << LC << "Destroyed\n";
}

void
SilverLiningContext::setLight(osg::Light* light)
{
    _light = light;
}

void
SilverLiningContext::setSRS(const osgEarth::SpatialReference* srs)
{
    _srs = srs;
}

void
SilverLiningContext::setMinimumAmbient(const osg::Vec4f& value)
{
    _minAmbient = value;
}

void
SilverLiningContext::initialize(osg::RenderInfo& renderInfo)
{
    if ( !_initAttempted && !_initFailed )
    {
        // lock/double-check:
        Threading::ScopedMutexLock excl(_initMutex);
        if ( !_initAttempted && !_initFailed )
        {
            _initAttempted = true;

            // constant random seed ensures consistent clouds across windows
            // TODO: replace this with something else since this is global! -gw
            ::srand(1234);

            int result = _atmosphere->Initialize(
                ::SilverLining::Atmosphere::OPENGL,
                _options.resourcePath()->c_str(),
                true,
                0 );

            if ( result != ::SilverLining::Atmosphere::E_NOERROR )
            {
                _initFailed = true;
                OE_WARN << LC << "SilverLining failed to initialize: " << result << std::endl;
            }
            else
            {
                OE_INFO << LC << "SilverLining initialized OK!" << std::endl;

                // Defaults for a projected terrain. ECEF terrain vectors are set
                // in updateLocation().
                _atmosphere->SetUpVector( 0.0, 0.0, 1.0 );
                _atmosphere->SetRightVector( 1.0, 0.0, 0.0 );

#if 0 // todo: review this
                _maxAmbientLightingAlt = 
                    _atmosphere->GetConfigOptionDouble("atmosphere-height");
#endif

                if ( _options.drawClouds() == true )
                {
                    OE_INFO << LC << "Initializing clouds\n";
                    setupClouds();
                }
            }
        }
    }
}

void
SilverLiningContext::setupClouds()
{
    _clouds = ::SilverLining::CloudLayerFactory::Create( CUMULUS_CONGESTUS );
    _clouds->SetIsInfinite( true );
    _clouds->SetFadeTowardEdges(true);
    _clouds->SetBaseAltitude( 2000 );
    _clouds->SetThickness( 200 );
    _clouds->SetBaseLength( 100000 );
    _clouds->SetBaseWidth( 100000 );
    _clouds->SetDensity( 0.6 );
    _clouds->SetAlpha( 0.8 );

    _clouds->SeedClouds( *_atmosphere );
    _clouds->GenerateShadowMaps( false );
    
    _clouds->SetLayerPosition(0, 0);

    _atmosphere->GetConditions()->AddCloudLayer( _clouds );
}

void
SilverLiningContext::updateLight()
{
    if ( !ready() || !_light.valid() || !_srs.valid() )
        return;

    float ra, ga, ba, rd, gd, bd, x, y, z;

    // Clamp the camera's altitude while fetching the colors so the
    // lighting's ambient component doesn't fade to black at high altitude.
    ::SilverLining::Location savedLoc = _atmosphere->GetConditions()->GetLocation();
    ::SilverLining::Location clampedLoc = savedLoc;
    if ( _maxAmbientLightingAlt > 0.0 )
    {
        clampedLoc.SetAltitude( std::min(clampedLoc.GetAltitude(), _maxAmbientLightingAlt) );
        _atmosphere->GetConditions()->SetLocation( clampedLoc );
    }

    _atmosphere->GetAmbientColor( &ra, &ga, &ba );
    _atmosphere->GetSunColor( &rd, &gd, &bd );

    // Restore the actual altitude.
    if ( _maxAmbientLightingAlt > 0.0 )
    {
        _atmosphere->GetConditions()->SetLocation( savedLoc );
    }

    if ( _srs->isGeographic() )
    {
        _atmosphere->GetSunPositionGeographic( &x, &y, &z );
    }
    else
    {
        _atmosphere->GetSunPosition(&x, &y, &z);
    }

    osg::Vec3 direction(x, y, z);
    direction.normalize();

    osg::Vec4 ambient(
        osg::clampAbove(ra, _minAmbient.r()),
        osg::clampAbove(ba, _minAmbient.g()),
        osg::clampAbove(ga, _minAmbient.b()),
        1.0);

    _light->setAmbient( ambient );
    _light->setDiffuse( osg::Vec4(rd, gd, bd, 1.0f) );
    _light->setPosition( osg::Vec4(direction, 0.0f) ); //w=0 means "at infinity"
}

void
SilverLiningContext::updateLocation()
{
    if ( !ready() || !_srs.valid() )
        return;

    if ( _srs->isGeographic() )
    {
        // Get new local orientation
        osg::Vec3d up = _cameraPos;
        up.normalize();
        osg::Vec3d north = osg::Vec3d(0, 1, 0);
        osg::Vec3d east = north ^ up;

        // Check for edge case of north or south pole
        if (east.length2() == 0)
        {
            east = osg::Vec3d(1, 0, 0);
        }

        east.normalize();

        _atmosphere->SetUpVector(up.x(), up.y(), up.z());
        _atmosphere->SetRightVector(east.x(), east.y(), east.z());

        // Get new lat / lon / altitude
        osg::Vec3d latLonAlt;
        _srs->transformFromWorld(_cameraPos, latLonAlt);

        ::SilverLining::Location loc;
        loc.SetAltitude ( latLonAlt.z() );
        loc.SetLongitude( latLonAlt.x() ); //osg::DegreesToRadians(latLonAlt.x()) );
        loc.SetLatitude ( latLonAlt.y() ); //osg::DegreesToRadians(latLonAlt.y()) );

        _atmosphere->GetConditions()->SetLocation( loc );

#if 0
        if ( _clouds )
        {
#if 1 //TODO: figure out why we need to call this a couple times before
      //      it takes effect. -gw
            static int c = 2;
            if ( c > 0 ) {
                --c;
                _clouds->SetLayerPosition(0, 0);
            }
        }
#endif
#endif
    }
}
