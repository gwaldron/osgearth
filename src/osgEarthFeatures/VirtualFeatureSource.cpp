/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include <osgEarthFeatures/VirtualFeatureSource>

#define LC "[VirtualFeatureSource] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Features;

//------------------------------------------------------------------------

namespace
{
    /**
     * Cursor that will iterate over multiple feature sources with predicate filtering
     */
    struct VirtualFeatureCursor : public FeatureCursor
    {
        VirtualFeatureCursor( const FeatureSourceMappingVector& sources, const Query& query ) :
          _sources(sources), _query(query)
        {
            _si = _sources.begin();
            advance();
        }

        bool hasMore() const
        {
            return _nextFeature.valid();
        }

        Feature* nextFeature()
        {
            _lastFeatureReturned = _nextFeature.get();
            _nextFeature = 0L;
            if ( _lastFeatureReturned.valid() )
                advance();
            return _lastFeatureReturned.get();
        }

    private:
        // pulls the next feature (in advance) in preparation for the next
        // call to nextFeature.
        void advance()
        {
            _nextFeature = 0L;

            while ( !_nextFeature.valid() )
            {
                // check to see if we are completely done:
                if ( _si == _sources.end() )
                    return;

                // if we're at the beginning, create the first cursor:
                if ( _si == _sources.begin() && !_si_cursor.valid() )
                {
                    _si_cursor = _si->_source->createFeatureCursor( _query );
                }

                while ( !_si_cursor.valid() || !_si_cursor->hasMore() )
                {
                    // if the current cursor is done, advance to the next source.
                    // if there is no next source, we are done.
                    if ( ++_si == _sources.end() )
                        return;

                    // make a cursor for the next source
                    _si_cursor = _si->_source->createFeatureCursor( _query );
                }

                // here, we have a valid cursor with pending data:
                Feature* f = _si_cursor->nextFeature();

                // test against the predicate. (a NULL predicate always accepts the feature)
                if ( !_si->_predicate.valid() || _si->_predicate->acceptFeature( f ) )
                    _nextFeature = f;
            }
        }

    private:
        FeatureSourceMappingVector           _sources;
        Query                                _query;
        FeatureSourceMappingVector::iterator _si;        // points to current source
        osg::ref_ptr<FeatureCursor>          _si_cursor; // cursor into current source
        osg::ref_ptr<Feature>                _nextFeature;
        osg::ref_ptr<Feature>                _lastFeatureReturned; // to manage references during iteration
    };
}

//------------------------------------------------------------------------

void
VirtualFeatureSource::add( FeatureSource* source, FeaturePredicate* predicate )
{
    _sources.push_back( FeatureSourceMapping(source, predicate) );
    dirty();

    if (_sources.size() == 1)
        setFeatureProfile(createFeatureProfile());
}

FeatureCursor* 
VirtualFeatureSource::createFeatureCursor( const Query& query )
{
    return new VirtualFeatureCursor( _sources, query );
}

Status 
VirtualFeatureSource::initialize( const osgDB::Options* readOptions )
{
    //FeatureSource::initialize( dbOptions );

    for( FeatureSourceMappingVector::iterator i = _sources.begin(); i != _sources.end(); ++i )
    {
        const Status& sourceStatus = i->_source->open(readOptions);
        if (sourceStatus.isError())
            return sourceStatus;
        //i->_source->initialize( dbOptions );
    }

    return Status::OK();
}

const FeatureProfile* 
VirtualFeatureSource::createFeatureProfile()
{
    if ( _sources.size() > 0 )
        return _sources.front()._source->getFeatureProfile();
    else
        return 0L;
}

const FeatureSchema&
VirtualFeatureSource::getSchema() const
{
  static FeatureSchema s_emptySchema;

  return _sources.size() > 0 ?_sources.front()._source->getSchema() : s_emptySchema;
}
