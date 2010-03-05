/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osgEarthSymbology/Common>
#include <osgEarthSymbology/FeatureDataSetAdapter>
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/Query>

using namespace osgEarth::Symbology;


FeatureDataSetAdapter::FeatureDataSetAdapter(osgEarth::Features::FeatureSource* source) : _features(source)
{
    _features->initialize("");

}

int FeatureDataSetAdapter::getRevision() const { return 0; }

osgEarth::Features::FeatureCursor* FeatureDataSetAdapter::createCursor()
{
    return _features->createFeatureCursor(osgEarth::Features::Query());
}

