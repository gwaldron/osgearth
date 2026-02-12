/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/FilterContext>
#include <osgEarth/Session>
#include <osgEarth/ResourceCache>
#include <osgEarth/FeatureSource>

using namespace osgEarth;

FilterContext::FilterContext(Session* session, const GeoExtent& workingExtent, FeatureIndexBuilder* index) :
    _session(session),
    _workingExtent(workingExtent),
    _index(index)
{
    //OE_SOFT_ASSERT(session, "ILLEGAL - session cannot be nullptr");

    _workingExtent = workingExtent;

    if (session)
    {
        if (session->getResourceCache())
        {
            _resourceCache = session->getResourceCache();
        }
        else
        {
            _resourceCache = new ResourceCache();
        }

        if (session->getFeatureSource())
        {
            _featureProfile = session->getFeatureSource()->getFeatureProfile();
        }
    }

    // attempt to establish a working extent if we don't have one:

    if (!_workingExtent->isValid() &&
        _featureProfile &&
        _featureProfile->getExtent().isValid())
    {
        _workingExtent = _featureProfile->getExtent();
    }

    if (!_workingExtent->isValid() &&
        session &&
        session->getMapProfile())
    {
        _workingExtent = session->getMapProfile()->getExtent();
    }
}

FilterContext::FilterContext(Session* session, const FeatureProfile* profile, const GeoExtent& workingExtent, FeatureIndexBuilder* index) :
    _session(session),
    _featureProfile(profile),
    _workingExtent(workingExtent),
    _index(index)
{
    OE_SOFT_ASSERT(session, "ILLEGAL - session cannot be nullptr");

    _workingExtent = workingExtent;

    if (session)
    {
        if ( session->getResourceCache() )
        {
            _resourceCache = session->getResourceCache();
        }
        else
        {
            _resourceCache = new ResourceCache();
        }
    }

    // attempt to establish a working extent if we don't have one:

    if (!_workingExtent->isValid() &&
        profile &&
        profile->getExtent().isValid() )
    {
        _workingExtent = profile->getExtent();
    }

    if (!_workingExtent->isValid() &&
        session && 
        session->getMapProfile() )
    {
        _workingExtent = session->getMapProfile()->getExtent();
    }
}

FilterContext::FilterContext(const FeatureProfile* profile, const Query& query)
{
    _featureProfile = profile;

    if (query.tileKey().isSet())
        _workingExtent = query.tileKey()->getExtent();
    else if (query.bounds().isSet() && profile)
        _workingExtent = GeoExtent(profile->getSRS(), query.bounds().get());
    else if (profile)
        _workingExtent = profile->getExtent();
}

bool
FilterContext::isGeoreferenced() const
{ 
    return _session.valid() && _featureProfile.valid();
}

const SpatialReference*
FilterContext::outputSRS() const
{
    if (_outputSRS.valid())
        return _outputSRS.get();

    if (_session.valid() && _session->getMapSRS())
        return _session->getMapSRS();

    if (_featureProfile.valid() && _featureProfile->getSRS())
        return _featureProfile->getSRS();

    if (_workingExtent.isSet())
        return _workingExtent->getSRS();

    return SpatialReference::get("wgs84");
}

const osgDB::Options*
FilterContext::getDBOptions() const
{
    return _session.valid() ? _session->getDBOptions() : 0L;
}
