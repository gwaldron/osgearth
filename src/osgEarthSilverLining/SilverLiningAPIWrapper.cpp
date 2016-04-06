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
#include "SilverLiningAPIWrapper"
#include <SilverLining.h>

#define LC "[SilverLiningAPI] "

using namespace osgEarth::SilverLining;

#define SETGET(NS, FUNC, TYPE) \
    void NS :: Set##FUNC (TYPE value) { HANDLE-> Set##FUNC (value); } \
    TYPE NS :: Get##FUNC () const     { return HANDLE -> Get##FUNC (); }

//................................
#undef  HANDLE
#define HANDLE ((::SilverLining::Atmosphere*)_handle)
void Atmosphere::EnableLensFlare(bool value) { HANDLE->EnableLensFlare(value); }
bool Atmosphere::GetLensFlareEnabled() const  { return HANDLE->GetLensFlareEnabled(); }
AtmosphericConditions Atmosphere::GetConditions() const { return AtmosphericConditions((uintptr_t)HANDLE->GetConditions()); }

//................................
#undef  HANDLE
#define HANDLE ((::SilverLining::AtmosphericConditions*)_handle)
void AtmosphericConditions::AddCloudLayer(CloudLayer& layer) { HANDLE->AddCloudLayer((::SilverLining::CloudLayer*)layer._handle); }

//................................
#undef  HANDLE
#define HANDLE ((::SilverLining::CloudLayer*)_handle)
SETGET(CloudLayer, Enabled, bool)
SETGET(CloudLayer, BaseWidth, double)
SETGET(CloudLayer, BaseLength, double)
SETGET(CloudLayer, BaseAltitude, double)
SETGET(CloudLayer, Thickness, double)
SETGET(CloudLayer, Density, double)
SETGET(CloudLayer, IsInfinite, bool)
SETGET(CloudLayer, CloudWrapping, bool)
SETGET(CloudLayer, FadeTowardEdges, bool)
SETGET(CloudLayer, Alpha, double)

void CloudLayer::SetLayerPosition(double e, double n) { HANDLE->SetLayerPosition(e, n); }
void CloudLayer::GetLayerPosition(double& e, double& n) const { HANDLE->GetLayerPosition(e, n); }

void CloudLayer::SetCloudAnimationEffects(double a, bool b, int c, int d) { HANDLE->SetCloudAnimationEffects(a, b, c, d); }
void CloudLayer::GetCloudAnimationEffects(double& a, bool& b, int& c, int& d) { HANDLE->SetCloudAnimationEffects(a, b, c, d);
}
void CloudLayer::SeedClouds(Atmosphere& a) { HANDLE->SeedClouds(*(::SilverLining::Atmosphere*)a._handle); }

//................................
CloudLayer CloudLayerFactory::Create(int kind) {
    return CloudLayer((uintptr_t)::SilverLining::CloudLayerFactory::Create((::CloudTypes)kind));
}