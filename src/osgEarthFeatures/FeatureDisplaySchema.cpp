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
#include <osgEarthFeatures/FeatureDisplaySchema>
#include <list>
#include <deque>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

//------------------------------------------------------------------------

FeatureLevel::FeatureLevel( float minRange, float maxRange, const Query& query ) :
_minRange( minRange ),
_maxRange( maxRange ),
_query   ( query )
{
    //nop
}

//------------------------------------------------------------------------

FeatureDisplaySchema::FeatureDisplaySchema() :
_radiusFactor( 10.0f )
{
    //nop
}

void 
FeatureDisplaySchema::addLevel( const FeatureLevel& level )
{
    _levels.push_back( level );
}

unsigned 
FeatureDisplaySchema::getNumLevels() const
{
    return _levels.size();
}

const FeatureLevel* 
FeatureDisplaySchema::getLevel( unsigned i ) const
{
    return i >= 0 && i < _levels.size() ? &_levels[i] : 0L;
}

const FeatureLevel* 
FeatureDisplaySchema::getLevelForLOD( unsigned lod, const GeoExtent& featureExtent ) const
{
    double radius = featureExtent.bounds().radius();
    for( unsigned i=0; i<lod; ++i )
        radius *= 0.5;

    double lodRange = radius * _radiusFactor;

    for( unsigned j=0; j<_levels.size(); ++j )
    {
        const FeatureLevel* level = getLevel(j);
        if ( lodRange > level->getMinRange() && lodRange <= level->getMaxRange() )
        {
            return level;
        }
    }

    return 0L;
}
