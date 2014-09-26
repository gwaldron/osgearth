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

#include <osgEarthAnnotation/AnnotationSettings>

using namespace osgEarth::Annotation;

//---------------------------------------------------------------------------

// static defaults
bool AnnotationSettings::_continuousClamping = true;
bool AnnotationSettings::_autoDepthOffset = true;
double AnnotationSettings::_occlusionCullingMaxAltitude = 200000.0;
double AnnotationSettings::_occlusionCullingHeightAdjustment = 5.0;
