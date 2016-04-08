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
#include "TritonAPIWrapper"
#include <Triton.h>

#define LC "[TritonAPI] "

using namespace osgEarth::Triton;

#define SETGET(NS, FUNC, TYPE) \
    void NS :: Set##FUNC (TYPE value) { HANDLE-> Set##FUNC (value); } \
    TYPE NS :: Get##FUNC () const     { return HANDLE -> Get##FUNC (); }

#define TOVEC3(X)   ::Triton::Vector3(X.x(),X.y(),X.z())
#define FROMVEC3(X) osg::Vec3(X.x,X.y,X.z)

//................................
#undef  HANDLE
#define HANDLE ((::Triton::BreakingWavesParameters*)_handle)

SETGET(BreakingWavesParameters, Steepness, float);
SETGET(BreakingWavesParameters, Wavelength, float);

//................................
#undef  HANDLE
#define HANDLE ((::Triton::Environment*)_handle)

BreakingWavesParameters Environment::GetBreakingWavesParameters() const {
    return BreakingWavesParameters((uintptr_t)&HANDLE->GetBreakingWavesParameters());
}
void Environment::SetDirectionalLight(const osg::Vec3& dir, const osg::Vec3& color) {
    HANDLE->SetDirectionalLight(TOVEC3(dir), TOVEC3(color));
}
osg::Vec3 Environment::GetLightDirection() const {
    const ::Triton::Vector3& v = HANDLE->GetLightDirection();
    return FROMVEC3(v);
}
osg::Vec3 Environment::GetDirectionalLightColor() const {
    const ::Triton::Vector3& v = HANDLE->GetDirectionalLightColor();
    return FROMVEC3(v);
}
void Environment::SetAmbientLight(const osg::Vec3& color) {
    HANDLE->SetAmbientLight(TOVEC3(color));
}
osg::Vec3 Environment::GetAmbientLightColor() const {
    const ::Triton::Vector3& v = HANDLE->GetAmbientLightColor();
    return FROMVEC3(v);
}
void Environment::SimulateSeaState(double bscale, double winddir) {
    HANDLE->SimulateSeaState(bscale, winddir);
}

//................................
#undef  HANDLE
#define HANDLE ((::Triton::Ocean*)_handle)

SETGET(Ocean, Choppiness, float);
//SETGET(Ocean, MaximumWavePeriod, double);

void Ocean::SetQuality(OceanQuality value) { HANDLE->SetQuality((::Triton::OceanQuality)value); }
OceanQuality Ocean::GetQuality() const { return (OceanQuality)HANDLE->GetQuality(); }

