/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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

using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace OpenThreads;

//---------------------------------------------------------------------------

void
FeatureCursor::fill( FeatureList& list )
{
    while( hasMore() )
    {
        list.push_back( nextFeature() );
    }
}

//---------------------------------------------------------------------------

FeatureListCursor::FeatureListCursor( const FeatureList& features, bool clone ) :
_features( features ),
_clone   ( clone )
{
    _iter = _features.begin();
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

GeometryFeatureCursor::GeometryFeatureCursor( Geometry* geom ) :
_geom( geom )
{
    //nop
}

GeometryFeatureCursor::GeometryFeatureCursor(Geometry* geom,
                                             const FeatureProfile* fp,
                                             const FeatureFilterList& filters ) :
_geom( geom ),
_featureProfile( fp ),
_filters( filters )
{
    //nop
}

bool
GeometryFeatureCursor::hasMore() const {
    return _geom.valid();
}

Feature*
GeometryFeatureCursor::nextFeature()
{
    if ( hasMore() )
    {        
        _lastFeature = new Feature( _geom.get(), _featureProfile.valid() ? _featureProfile->getSRS() : 0L );
        FilterContext cx;
        cx.profile() = _featureProfile.get();
        FeatureList list;
        list.push_back( _lastFeature.get() );
        for( FeatureFilterList::const_iterator i = _filters.begin(); i != _filters.end(); ++i ) {
            cx = i->get()->push( list, cx );
        }
        _geom = 0L;
    }
    return _lastFeature.get();
}
