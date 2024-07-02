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

unsigned
FeatureCursor::fill(FeatureList& list)
{
    unsigned count = 0;
    while( hasMore() )
    {
        auto f = nextFeature();
        if (f)
        {
            list.push_back(f);
            ++count;
        }
    }
    return count;
}

unsigned
FeatureCursor::fill(FeatureList& list, std::function<bool(const Feature*)> predicate)
{
    unsigned count = 0;
    while (hasMore())
    {
        osg::ref_ptr<Feature> f = nextFeature();
        if (f.valid() && predicate(f.get()))
        {
            list.push_back(f);
            ++count;
        }
    }
    return count;
}

//---------------------------------------------------------------------------

GeometryFeatureCursor::GeometryFeatureCursor(Geometry* geom) :
    FeatureCursor(NULL),
    _geom(geom)
{
    //nop
}

GeometryFeatureCursor::GeometryFeatureCursor(Geometry* geom, const FeatureProfile* fp, const FeatureFilterChain& filters) :
    FeatureCursor(NULL),
    _geom(geom),
    _featureProfile(fp),
    _filterChain(filters)
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
    if (hasMore())
    {
        _lastFeature = new Feature(_geom.get(), _featureProfile.valid() ? _featureProfile->getSRS() : nullptr);

        if (_featureProfile && _featureProfile->geoInterp().isSet())
            _lastFeature->geoInterp() = _featureProfile->geoInterp().get();

        FilterContext cx;
        cx.setProfile(_featureProfile.get());

        FeatureList one = { _lastFeature };

        cx = _filterChain.push(one, cx);

        if (one.empty())
        {
            _lastFeature = nullptr;
        }

        _geom = 0L;
    }

    return _lastFeature.get();
}
