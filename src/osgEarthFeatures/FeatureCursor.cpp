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
#include <osgEarthFeatures/FeatureCursor>
#include <osgEarthFeatures/Filter>
#include <osgEarth/Progress>

using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace OpenThreads;

//---------------------------------------------------------------------------

FeatureCursor::FeatureCursor(ProgressCallback* progress) :
_progress(progress)
{
    //nop
}

FeatureCursor::~FeatureCursor()
{
    //nop
}

void
FeatureCursor::fill(FeatureList& list)
{
    while( hasMore() )
    {
        list.push_back( nextFeature() );
    }
}

//---------------------------------------------------------------------------

FeatureListCursor::FeatureListCursor(const FeatureList& features) :
FeatureCursor(0L),
_features( features ),
_clone   ( false )
{
    _iter = _features.begin();
}

FeatureListCursor::~FeatureListCursor()
{
    //nop
}

bool
FeatureListCursor::hasMore() const
{
    return _iter != _features.end();
}

Feature*
FeatureListCursor::nextFeature()
{
    Feature* r = _iter->get();
    _iter++;
    return _clone ? osg::clone(r, osg::CopyOp::DEEP_COPY_ALL) : r;
}

//---------------------------------------------------------------------------

GeometryFeatureCursor::GeometryFeatureCursor(Geometry* geom) :
FeatureCursor(NULL),
_geom( geom )
{
    //nop
}

GeometryFeatureCursor::GeometryFeatureCursor(Geometry* geom,
                                             const FeatureProfile* fp,
                                             const FeatureFilterChain* filters) :
FeatureCursor(NULL),
_geom          ( geom ),
_featureProfile( fp ),
_filterChain   ( filters )
{
    //nop
}

GeometryFeatureCursor::~GeometryFeatureCursor()
{
    //nop
}

bool
GeometryFeatureCursor::hasMore() const
{
    return _geom.valid();
}

Feature*
GeometryFeatureCursor::nextFeature()
{
    if ( hasMore() )
    {        
        _lastFeature = new Feature( _geom.get(), _featureProfile.valid() ? _featureProfile->getSRS() : 0L );

        if ( _featureProfile && _featureProfile->geoInterp().isSet() )
            _lastFeature->geoInterp() = _featureProfile->geoInterp().get();

        FilterContext cx;
        cx.setProfile( _featureProfile.get() );

        FeatureList list;
        list.push_back( _lastFeature.get() );

        if (_filterChain.valid())
        {
            for( FeatureFilterChain::const_iterator i = _filterChain->begin(); i != _filterChain->end(); ++i )
            {
                cx = i->get()->push( list, cx );
            }
        }

        if ( list.empty() )
        {
            _lastFeature = 0L;
        }

        _geom = 0L;
    }

    return _lastFeature.get();
}
