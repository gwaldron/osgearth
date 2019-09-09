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
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include "TritonContext"
#include <osg/GLExtensions>
#include <osg/Math>
#include <osgDB/FileNameUtils>
#include <osgEarth/SpatialReference>
#include <cstdlib>

#define LC "[TritonContext] "

using namespace osgEarth::Triton;


TritonContext::TritonContext(const TritonOptions& options) :
_options              ( options ),
_initAttempted        ( false ),
_initFailed           ( false ),
_resourceLoader       ( 0L ),
_environment          ( 0L ),
_environmentWrapper   ( 0L ),
_ocean                ( 0L ),
_oceanWrapper         ( 0L )
{    
    //nop
}

TritonContext::~TritonContext()
{
    if (_oceanWrapper)
        delete _oceanWrapper;

    if (_environmentWrapper)
        delete _environmentWrapper;
}

void
TritonContext::setSRS(const osgEarth::SpatialReference* srs)
{
    _srs = srs;
}

void
TritonContext::setCallback(Callback* callback)
{
    _callback = callback;
}

bool
TritonContext::passHeightMapToTriton() const
{
    return _options.useHeightMap() == true;
}

int
TritonContext::getHeightMapSize() const
{
    return osg::clampBetween(_options.heightMapSize().get(), 64, 2048);
}

const std::string&
TritonContext::getMaskLayerName() const
{
    return _options.maskLayer().get();
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

            std::string resourcePath = _options.resourcePath().get();
            if (resourcePath.empty() && ::getenv("TRITON_PATH"))
            {
                resourcePath = osgDB::concatPaths(::getenv("TRITON_PATH"), "Resources");
            }

            _resourceLoader = new ::Triton::ResourceLoader(resourcePath.c_str());

            _environment = new ::Triton::Environment();

            _environmentWrapper = new Environment((uintptr_t)_environment);

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

            //_environment->SetConfigOption("avoid-opengl-stalls", "yes");
            //_environment->SetConfigOption("fft-texture-update-frame-delayed", "yes");

            float openGLVersion = osg::getGLVersionNumber();
            enum ::Triton::Renderer tritonOpenGlVersion = ::Triton::OPENGL_2_0;
#ifndef OSG_GL_FIXED_FUNCTION_AVAILABLE
            if( openGLVersion >= 4.1 )
                tritonOpenGlVersion = ::Triton::OPENGL_4_1;
            else if( openGLVersion >= 4.0 )
                tritonOpenGlVersion = ::Triton::OPENGL_4_0;
            else if( openGLVersion >= 3.2 )
                tritonOpenGlVersion = ::Triton::OPENGL_3_2;
#endif

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
                    ::Triton::JONSWAP,
                    true );                 // enableHeightTests - activated GetHeight for intersections
            }

            if ( _ocean )
            {
                _oceanWrapper = new Ocean((uintptr_t)_ocean);

                // fire init callback if available
                if (_callback.valid())
                {
                    _callback->onInitialize(getEnvironmentWrapper(), getOceanWrapper());
                }

                OE_INFO << LC << "Triton initialized OK!" << std::endl;                
            }
            else
            {
                _initFailed = true;
                OE_WARN << LC << "Triton initialization failed- err=" << err << std::endl;
            }
        }
    }
}

bool
TritonContext::intersect(const osg::Vec3d& start, const osg::Vec3d& dir, float& out_height, osg::Vec3f& out_normal) const
{
    ::Triton::Vector3 p(start.ptr());
    ::Triton::Vector3 d(dir.ptr());
    ::Triton::Vector3 normal;
    bool ok = _ocean->GetHeight(p, d, out_height, normal);
    out_normal.set(normal.x, normal.y, normal.z);
    return ok;
}

void
TritonContext::resizeGLObjectBuffers(unsigned maxSize)
{
    osg::Object::resizeGLObjectBuffers(maxSize);
}

void
TritonContext::releaseGLObjects(osg::State* state) const
{
    osg::Object::releaseGLObjects(state);

    OE_INFO << LC << "Triton shutting down - releasing GL resources\n";
    if (state)
    {
        if ( _ocean )
        {
            delete _ocean;
            _ocean = 0L;
        }

        if ( _environment )
        {
            delete _environment;
            _environment = 0L;
        }

        if ( _resourceLoader )
        {
            delete _resourceLoader;
            _resourceLoader = 0L;
        }
    }
}
