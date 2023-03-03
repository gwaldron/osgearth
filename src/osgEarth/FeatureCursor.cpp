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

FeatureCursor::FeatureCursor()
{
    // initialize to an empty set
    set(new FeatureListCursorImpl(FeatureList()));
}

FeatureCursor::FeatureCursor(FeatureCursorImplementation* impl)
{
    set(impl);
}

FeatureCursor::FeatureCursor(const FeatureList& list)
{
    _impl = std::make_shared<FeatureListCursorImpl>(list);
}

void
FeatureCursor::set(FeatureCursorImplementation* impl)
{
    if (impl)
        _impl = std::shared_ptr<FeatureCursorImplementation>(impl);
    else
        _impl = std::make_shared<FeatureListCursorImpl>(FeatureList()); // empty set
}

void
FeatureCursor::fill(FeatureList& list)
{
    while (hasMore())
    {
        list.push_back(nextFeature());
    }
}

void
FeatureCursor::fill(
    FeatureList& list,
    std::function<bool(const Feature*)> predicate)
{
    while (hasMore())
    {
        osg::ref_ptr<const Feature> f = nextFeature();
        if (predicate(f.get()))
            list.push_back(f);
    }
}

bool
FeatureCursor::hasMore() const
{
    return _impl ? _impl->hasMore() : false;
}

osg::ref_ptr<const Feature>
FeatureCursor::nextFeature()
{
    return _impl ? _impl->nextFeature() : nullptr;
}

//---------------------------------------------------------------------------

FeatureListCursorImpl::FeatureListCursorImpl(const FeatureList& features) :
    _features(features)
{
    _iter = _features.begin();
}

bool
FeatureListCursorImpl::hasMore() const
{
    return _iter != _features.end();
}

osg::ref_ptr<const Feature>
FeatureListCursorImpl::nextFeature()
{
    auto feature = *_iter;
    _iter++;
    return feature;
}

//---------------------------------------------------------------------------

FilteredFeatureCursorImpl::FilteredFeatureCursorImpl(
    FeatureCursor& cursor,
    const FeatureFilterChain& chain,
    FilterContext* cx) :
    _cursor(cursor),
    _chain(chain),
    _user_cx(cx)
{
    //nop
}

bool
FilteredFeatureCursorImpl::hasMore() const
{
    if (!_cache.empty())
        return true;

    const int chunkSize = 500;

    FilterContext temp_cx;
    FilterContext& cx = _user_cx == nullptr ? temp_cx : *_user_cx;

    while(_cursor->hasMore() && _cache.size() < chunkSize)
    {
        FeatureList features;

        while(_cursor->hasMore() && features.size() < chunkSize)
        {
            features.push_back(_cursor->nextFeature());
        }

        for (auto& filter : _chain)
        {
            cx = filter->push(features, cx);
        }

        std::copy(features.begin(), features.end(), std::back_inserter(_cache));
    }

    return !_cache.empty();
}

osg::ref_ptr<const Feature>
FilteredFeatureCursorImpl::nextFeature()
{
    auto feature = _cache.front();
    _cache.pop_front();
    return feature;
}
