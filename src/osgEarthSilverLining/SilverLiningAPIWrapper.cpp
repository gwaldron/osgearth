/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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

#define SETGET_EXPLICIT(NS, SETTER, GETTER, TYPE) \
    void NS :: SETTER (TYPE value) { HANDLE-> SETTER (value); } \
    TYPE NS :: GETTER () const     { return HANDLE -> GETTER (); }

#define SETGET(NS, FUNC, TYPE) \
    SETGET_EXPLICIT(NS, Set##FUNC, Get##FUNC, TYPE)

//................................
#undef  HANDLE
#define HANDLE ((::SilverLining::Atmosphere*)_handle)

SETGET_EXPLICIT(Atmosphere, EnableLensFlare, GetLensFlareEnabled, bool);
void Atmosphere::SetSkyModel(SkyModel skyModel) { HANDLE->SetSkyModel(static_cast< ::SkyModel >(skyModel)); }
Atmosphere::SkyModel Atmosphere::GetSkyModel() const { return static_cast<SkyModel>(HANDLE->GetSkyModel()); }
AtmosphericConditions Atmosphere::GetConditions() const { return AtmosphericConditions((uintptr_t)HANDLE->GetConditions()); }
SETGET(Atmosphere, Gamma, double);
SETGET(Atmosphere, InfraRedMode, bool);

//................................
#undef  HANDLE
#define HANDLE ((::SilverLining::AtmosphericConditions*)_handle)

SETGET(AtmosphericConditions, Visibility, double);
SETGET(AtmosphericConditions, Turbidity, double);
SETGET(AtmosphericConditions, LightPollution, double);

void AtmosphericConditions::SetPrecipitation(int type, double rate) {
    HANDLE->SetPrecipitation((::CloudLayer::PrecipitationTypes)type, rate);
}

int AtmosphericConditions::AddCloudLayer(CloudLayer& layer) { return HANDLE->AddCloudLayer((::SilverLining::CloudLayer*)layer._handle); }
bool AtmosphericConditions::RemoveCloudLayer(int layerHandle) { return HANDLE->RemoveCloudLayer(layerHandle); }
void AtmosphericConditions::RemoveAllCloudLayers() { HANDLE->RemoveAllCloudLayers(); }

int AtmosphericConditions::SetWind(const WindVolume& windVolume)
{
    ::SilverLining::WindVolume slVolume;
    slVolume.SetMinAltitude(windVolume.GetMinAltitude());
    slVolume.SetMaxAltitude(windVolume.GetMaxAltitude());
    slVolume.SetWindSpeed(windVolume.GetWindSpeed());
    slVolume.SetDirection(windVolume.GetDirection());
    return HANDLE->SetWind(slVolume);
}
int AtmosphericConditions::SetWind(double metersPerSecond, double degreesFromNorth) { return SetWind(WindVolume(metersPerSecond, degreesFromNorth)); }
bool AtmosphericConditions::RemoveWindVolume(int layerHandle) { return HANDLE->RemoveWindVolume(layerHandle); }
void AtmosphericConditions::ClearWindVolumes() { HANDLE->ClearWindVolumes(); }
void AtmosphericConditions::SetPresetConditions(ConditionPresets preset, Atmosphere& atm) { HANDLE->SetPresetConditions(static_cast< ::SilverLining::AtmosphericConditions::ConditionPresets >(preset), *(::SilverLining::Atmosphere*)atm._handle); }
void AtmosphericConditions::EnableTimePassage(bool enabled, long relightFrequencyMS) { HANDLE->EnableTimePassage(enabled, relightFrequencyMS); }

//................................
WindVolume::WindVolume() :
_minAltitude( 0.0 ),
_maxAltitude( 100000.0 ),
_windSpeed  ( 0.0 ),
_direction  ( 0.0 )
{
}
WindVolume::WindVolume(double windSpeed, double direction) :
_minAltitude( 0.0 ),
_maxAltitude( 100000.0 ),
_windSpeed  ( windSpeed ),
_direction  ( direction )
{
}
void WindVolume::SetMinAltitude(double metersMSL) { _minAltitude = metersMSL; }
double WindVolume::GetMinAltitude() const { return _minAltitude; }
void WindVolume::SetMaxAltitude(double metersMSL) { _maxAltitude = metersMSL; }
double WindVolume::GetMaxAltitude() const { return _maxAltitude; }
void WindVolume::SetWindSpeed(double metersPerSecond) { _windSpeed = metersPerSecond; }
double WindVolume::GetWindSpeed() const { return _windSpeed; }
void WindVolume::SetDirection(double degreesFromNorth) { _direction = degreesFromNorth; }
double WindVolume::GetDirection() const { return _direction; }

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

void CloudLayer::SetWind(double windEast, double windSouth) { HANDLE->SetWind(windEast, windSouth); }
void CloudLayer::GetWind(double& windEast, double& windSouth) const { HANDLE->GetWind(windEast, windSouth); }

void CloudLayer::SetCloudAnimationEffects(double a, bool b, int c, int d) { HANDLE->SetCloudAnimationEffects(a, b, c, d); }
void CloudLayer::GetCloudAnimationEffects(double& a, bool& b, int& c, int& d) { HANDLE->SetCloudAnimationEffects(a, b, c, d); }

void CloudLayer::SeedClouds(Atmosphere& a) { HANDLE->SeedClouds(*(::SilverLining::Atmosphere*)a._handle); }

//................................
CloudLayer CloudLayerFactory::Create(int kind) {
    return CloudLayer((uintptr_t)::SilverLining::CloudLayerFactory::Create((::CloudTypes)kind));
}
