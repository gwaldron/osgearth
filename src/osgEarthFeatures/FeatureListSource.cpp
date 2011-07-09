/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osgEarthFeatures/FeatureListSource>

using namespace osgEarth::Features;

FeatureListSource::FeatureListSource():
FeatureSource()
{
    _profile = new FeatureProfile(GeoExtent(osgEarth::SpatialReference::create("epsg:4326"), -180, -90, 180, 90));
}

FeatureCursor*
FeatureListSource::createFeatureCursor( const Symbology::Query& query )
{
    //Create a copy of all of the features before returning the cursor.
    //The processing filters in osgEarth can modify the features as they are operating and we don't want our original data destroyed.
    FeatureList cursorFeatures;
    for (FeatureList::iterator itr = _features.begin(); itr != _features.end(); ++itr)
    {
        Feature* feature = new osgEarth::Features::Feature(*(itr->get()), osg::CopyOp::DEEP_COPY_ALL);        
        cursorFeatures.push_back( feature );
    }    
    return new FeatureListCursor( cursorFeatures );
}

void
FeatureListSource::initialize( const std::string& referenceURI )
{
}

const FeatureProfile*
FeatureListSource::createFeatureProfile()
{    
    return _profile.get();
}

bool
FeatureListSource::deleteFeature(FeatureID fid)
{
    for (FeatureList::iterator itr = _features.begin(); itr != _features.end(); ++itr) 
    {
        if (itr->get()->getFID() == fid)
        {
            _features.erase( itr );
            dirty();
            return true;
        }
    }
    return false;
}

Feature*
FeatureListSource::getFeature( FeatureID fid )
{
    for (FeatureList::iterator itr = _features.begin(); itr != _features.end(); ++itr) 
    {
        if (itr->get()->getFID() == fid)
        {
            return itr->get();
        }
    }
    return NULL;
}

bool FeatureListSource::insertFeature(Feature* feature)
{
    _features.push_back( feature );
    dirty();
    return true;
}
