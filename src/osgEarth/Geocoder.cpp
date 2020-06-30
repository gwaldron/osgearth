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
#include <osgEarth/Geocoder>
#include <osgEarth/OGRFeatureSource>
#include <osgEarth/Metrics>
#include <osgEarth/Utils>
#include <osgEarth/Containers>
#include "ogr_geocoding.h"

using namespace osgEarth;
using namespace osgEarth::Util;

namespace
{
    class OGRGeocodeResultCursor : public OGR::OGRFeatureCursor
    {
    public:
        OGRGeocodeResultCursor(OGRLayerH layerHandle, const FeatureProfile* profile) :
            OGR::OGRFeatureCursor(layerHandle, profile)
        {
            _geocodeResultHandle = layerHandle;
        }

        virtual ~OGRGeocodeResultCursor()
        {
            if (_geocodeResultHandle)
                OGRGeocodeFreeResult(_geocodeResultHandle);
        }

        OGRLayerH _geocodeResultHandle;
    };

    class OGRGeocodeImplementation : public Geocoder::Implementation
    {
    public:
        OGRGeocodeImplementation();
        virtual ~OGRGeocodeImplementation();

    public: // GeocoderImplementation
        osgEarth::Status search(const std::string& input, osg::ref_ptr<FeatureCursor>& output);

        void setServiceOption(const std::string& key, const std::string& value);

    private:
        OGRGeocodingSessionH _session;
        UnorderedMap<std::string,std::string> _options;
        void reset();
    };

    OGRGeocodeImplementation::OGRGeocodeImplementation() :
        _session(NULL)
    {
        reset();
    }

    OGRGeocodeImplementation::~OGRGeocodeImplementation()
    {
        if (_session)
            OGRGeocodeDestroySession(_session);
    }

    void OGRGeocodeImplementation::reset()
    {
        if (_session)
            OGRGeocodeDestroySession(_session);

        if (_options.empty() == false)
        {
            std::vector<std::string> buffers;
            char** str = new char*[_options.size()+1];
            int c = 0;
            for(UnorderedMap<std::string, std::string>::const_iterator i = _options.begin(); i != _options.end(); ++i)
            {                
                buffers.push_back(i->first + "=" + i->second);
                str[c++] = (char*)(buffers.back().c_str());
                //str[c++] = (char*)i->second.c_str();
            }
            str[c] = 0L;

            _session = OGRGeocodeCreateSession(str);

            delete [] str;
        }
        else
        {
            _session = OGRGeocodeCreateSession(NULL);
        }
    }

    void OGRGeocodeImplementation::setServiceOption(const std::string& name, const std::string& value)
    {
        _options[name] = value;
        reset();
    }

    Status OGRGeocodeImplementation::search(const std::string& input,  osg::ref_ptr<FeatureCursor>& output)
    {
        OGRLayerH layerHandle = OGRGeocode(_session, input.c_str(), NULL, NULL);
        if (!layerHandle)
        {
            return Status(Status::ServiceUnavailable);
        }

        // results are always in lat/long
        const osg::ref_ptr<SpatialReference> srs = SpatialReference::get("wgs84");
        GeoExtent extent(srs.get(), -180, -90, 180, 90);

        osg::ref_ptr<FeatureProfile> profile = new FeatureProfile(extent);
        output = new OGRGeocodeResultCursor(layerHandle, profile.get());

        return Status::OK();
    }

    // Operation that runs a geocode search in a thread pool
    struct GeocodeAsyncOperation : public osg::Operation
    {
        std::string _input;
        Promise<Geocoder::OutputData> _promise;
        osg::ref_ptr<Geocoder::Implementation> _impl;

        GeocodeAsyncOperation(const std::string& input, Promise<Geocoder::OutputData> promise, Geocoder::Implementation* impl) :
            _input(input),
            _promise(promise),
            _impl(impl)
        {
            //NOP
        }

        void operator()(osg::Object*)
        {
            OE_PROFILING_ZONE_NAMED("Geocode");
            if (!_promise.isAbandoned())
            {
                osg::ref_ptr<FeatureCursor> cursor;
                Status status = _impl->search(_input, cursor);
                _promise.resolve(new Geocoder::OutputData(status, cursor.get()));
            }
        }
    };
}

Geocoder::Geocoder()
{
    setImplementation( new OGRGeocodeImplementation() );
}

void
Geocoder::setServiceOption(const std::string& key, const std::string& value)
{
    if (_impl)
        _impl->setServiceOption(key, value);
}

Geocoder::Results
Geocoder::search(const std::string& input, const osgDB::Options* io_options)
{
    if (_impl)
    {
        osg::ref_ptr<ThreadPool> pool = ThreadPool::get(io_options);
        if (pool.valid())
        {
            Promise<Geocoder::OutputData> promise;
            GeocodeAsyncOperation* op = new GeocodeAsyncOperation(input, promise, _impl.get());
            pool->run(op);
            return Geocoder::Results(promise.getFuture());
        }
        else
        {
            osg::ref_ptr<FeatureCursor> cursor;
            Status status = _impl->search(input, cursor);
            return Geocoder::Results(status, cursor.get());
        }
    }
    else
    {
        return Geocoder::Results(
            Status(Status::ServiceUnavailable, "No geocoder implementation installed"),
            NULL);
    }
}

void
Geocoder::setImplementation(Geocoder::Implementation* impl)
{
    if (impl != _impl)
    {
        _impl = impl;
    }
}

Geocoder::Results::Results(const Status& status, FeatureCursor* cursor) :
    FutureResult(new Geocoder::OutputData(status, cursor))
{
    //NOP - error status
}

Geocoder::Results::Results(Future<Geocoder::OutputData> data) :
    FutureResult<Geocoder::OutputData>(data)
{
    //NOP
}

Status
Geocoder::Results::getStatus()
{
    return _future.get() ? _future.get()->_status : Status(Status::ServiceUnavailable);
}

FeatureCursor*
Geocoder::Results::getFeatures()
{
    return _future.get() ? _future.get()->_cursor.get() : NULL;
}
