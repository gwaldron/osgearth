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
#include "TritonContext"
#include <Triton.h>
#include <osg/GLExtensions>
#include <osgEarth/SpatialReference>

#define LC "[TritonContext] "

using namespace osgEarth;
using namespace osgEarth::Drivers::Triton;


TritonContext::TritonContext(const TritonOptions& options) :
_options              ( options ),
_initAttempted        ( false ),
_initFailed           ( false ),
_resourceLoader       ( 0L ),
_environment          ( 0L ),
_ocean                ( 0L )
{
    //nop
}

void
TritonContext::setSRS(const SpatialReference* srs)
{
    _srs = srs;
}

void
TritonContext::initialize(osg::RenderInfo& renderInfo)
{
    if ( !_initAttempted && !_initFailed )
    {
        // lock/double-check:
        Threading::ScopedMutexLock excl(_initMutex);
        if ( !_initAttempted && !_initFailed )
        {
            _initAttempted = true;

            _resourceLoader = new ::Triton::ResourceLoader(
                _options.resourcePath()->c_str() );

            _environment = new ::Triton::Environment();

            _environment->SetLicenseCode(
                _options.user()->c_str(),
                _options.licenseCode()->c_str() );

            // "WGS84" is used to represent any ellipsoid.
            ::Triton::CoordinateSystem cs =
                _srs->isGeographic() ? ::Triton::WGS84_ZUP :
                ::Triton::FLAT_ZUP;

            // Set the ellipsoid to match the one in our map's SRS.
            if ( _srs->isGeographic() )
            {
                const osg::EllipsoidModel* ellipsoid = _srs->getEllipsoid();
                
                std::string eqRadius = Stringify() << ellipsoid->getRadiusEquator();
                std::string poRadius = Stringify() << ellipsoid->getRadiusPolar();

                _environment->SetConfigOption( "equatorial-earth-radius-meters", eqRadius.c_str() );
                _environment->SetConfigOption( "polar-earth-radius-meters",      poRadius.c_str() );
            }

            float openGLVersion = osg::getGLVersionNumber();
            enum ::Triton::Renderer tritonOpenGlVersion = ::Triton::OPENGL_2_0;
            if( openGLVersion == 4.1 )
                tritonOpenGlVersion = ::Triton::OPENGL_4_1;
            else if( openGLVersion == 4.0 )
                tritonOpenGlVersion = ::Triton::OPENGL_4_0;
            else if( openGLVersion == 3.2 )
                tritonOpenGlVersion = ::Triton::OPENGL_3_2;

            ::Triton::EnvironmentError err = _environment->Initialize(
                cs,
                tritonOpenGlVersion,
                _resourceLoader );

            if ( err == ::Triton::SUCCEEDED )
            {
                ::Triton::WindFetch wf;
                wf.SetWind( 10.0, 0.0 );
                _environment->AddWindFetch( wf );

                _ocean = ::Triton::Ocean::Create(
                    _environment, 
                    ::Triton::JONSWAP );
            }

            if ( _ocean )
            {
                OE_INFO << LC << "Triton initialized OK!" << std::endl;
            }
            else
            {
                _initFailed = true;
                OE_WARN << LC << "Triton initialization failed" << std::endl;
            }
        }
    }
}

void
TritonContext::update(double simTime)
{
    if ( _ocean )
    {
        _ocean->UpdateSimulation( simTime );
    }
}

TritonContext::~TritonContext()
{
    if ( _ocean )
        delete _ocean;

    if ( _environment )
        delete _environment;

    if ( _resourceLoader )
        delete _resourceLoader;
}
