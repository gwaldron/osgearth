/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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
#include <iterator>
#include <osgEarth/FeatureCursor>
#include <osgEarth/Filter>
#include <osgEarth/Progress>

using namespace osgEarth;

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

void
FeatureCursor::fill(
    FeatureList& list,
    std::function<bool(const Feature*)> predicate)
{
    while (hasMore())
    {
        osg::ref_ptr<Feature> f = nextFeature();
        if (predicate(f.get()))
            list.push_back(f);
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

//---------------------------------------------------------------------------

FilteredFeatureCursor::FilteredFeatureCursor(
    FeatureCursor* cursor,
    FeatureFilterChain* chain) :

    FeatureCursor(cursor ? cursor->getProgress() : nullptr),
    _cursor(cursor),
    _chain(chain),
    _user_cx(nullptr)
{
    //nop
}
FilteredFeatureCursor::FilteredFeatureCursor(
    FeatureCursor* cursor,
    FeatureFilterChain* chain,
    FilterContext* context,
    bool ownsContext
    ) :

    FeatureCursor(cursor->getProgress()),
    _cursor(cursor),
    _chain(chain),
    _user_cx(context),
    _ownsContext(ownsContext)
{
    //nop
}

FilteredFeatureCursor::~FilteredFeatureCursor()
{
    if (_user_cx && _ownsContext)
    {
        delete _user_cx;
    }
}

bool
FilteredFeatureCursor::hasMore() const
{
    if (!_cache.empty())
        return true;

    const int chunkSize = 500;

    FilterContext temp_cx;
    FilterContext& cx = _user_cx == nullptr ? temp_cx : *_user_cx;

    while(_cursor->hasMore() && _cache.size() < chunkSize)
    {
        FeatureList local;

        while(_cursor->hasMore() && local.size() < chunkSize)
        {
            local.push_back(_cursor->nextFeature());
        }

        for(FeatureFilterChain::const_iterator filter = _chain->begin();
            filter != _chain->end();
            ++filter)
        {
            cx = filter->get()->push(local, cx);
        }

        std::copy(local.begin(), local.end(), std::back_inserter(_cache));
    }

    return !_cache.empty();
}

Feature*
FilteredFeatureCursor::nextFeature()
{
    Feature* feature = _cache.front().release();
    _cache.pop_front();
    return feature;
}
