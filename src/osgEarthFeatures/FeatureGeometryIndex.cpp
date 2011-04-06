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
#include <osgEarthFeatures/FeatureGeometryIndex>

using namespace osgEarth;
using namespace osgEarth::Features;

//---------------------------------------------------------------------------

namespace
{
    struct FindVisitor : public osg::NodeVisitor
    {
        FindVisitor( FeatureID id )
            : _id(id), 
              _rec(0L),
              osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) { }

        void apply( osg::Node& node )
        {
            if ( _rec )
                return;

            osg::Referenced* ref = node.getUserData();
            if ( ref )
            {
                FeatureGeometryIndex* index = dynamic_cast<FeatureGeometryIndex*>( ref );
                if ( index )
                {
                    _rec = index->get( _id );
                    if ( _rec )
                        return;
                }
            }
            traverse( node );
        }

        FeatureID _id;
        const FeatureGeometryRecord* _rec;
    };
}

FeatureGeometryQuery::FeatureGeometryQuery( osg::Node* graph ) :
_graph( graph )
{
    //nop
}

bool
FeatureGeometryQuery::find( FeatureID id, FeatureGeometryRecord& record ) const
{
    if ( _graph.valid() )
    {
        FindVisitor visitor( id );
        _graph->accept( visitor );
        if ( visitor._rec )
        {
            record = *(visitor._rec);
            return true;
        }
    }
    return false;
}

//---------------------------------------------------------------------------

FeatureGeometryIndex::FeatureGeometryIndex()
{
    //nop
}

const FeatureGeometryRecord*
FeatureGeometryIndex::get( FeatureID fid ) const
{
    FeatureRecords::const_iterator i = _records.find( fid );
    return i != _records.end () ? &(i->second) : 0L;
}

//---------------------------------------------------------------------------

namespace
{
    struct Collector : public osg::NodeVisitor
    {
        typedef FeatureGeometryIndexBuilder::PrimSetFeatureIdMap IdMap;

        Collector(const IdMap& ids, FeatureGeometryIndex::FeatureRecords& recs )
            : osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ),
              _recs( recs ),
              _ids( ids ) { }

        void apply( osg::Geode& geode )
        {
            for( unsigned i=0; i<geode.getNumDrawables(); ++i )
            {
                osg::Geometry* geom = geode.getDrawable(i)->asGeometry();
                if ( geom )
                {
                    for( unsigned j=0; j<geom->getNumPrimitiveSets(); ++j )
                    {
                        osg::PrimitiveSet* primSet = geom->getPrimitiveSet(j);

                        IdMap::const_iterator k = _ids.find( primSet );
                        if ( k != _ids.end() )
                        {
                            FeatureID fid = k->second;
                            FeatureGeometryRecord& rec = _recs[fid];
                            rec._geode = &geode;
                            rec._primSetsByGeometry[geom].push_back( primSet );
                        }
                    }
                }
            }
            traverse( geode );
        }

        FeatureGeometryIndex::FeatureRecords& _recs;
        const FeatureGeometryIndexBuilder::PrimSetFeatureIdMap& _ids;
    };
}

FeatureGeometryIndexBuilder::FeatureGeometryIndexBuilder()
{
    //nop
}

void
FeatureGeometryIndexBuilder::add( FeatureID id, osg::PrimitiveSet* primSet )
{
    _primSetIds[primSet] = id;
}

FeatureGeometryIndex*
FeatureGeometryIndexBuilder::createIndex( osg::Node* node )
{
    FeatureGeometryIndex* index = new FeatureGeometryIndex();
    Collector collector( _primSetIds, index->_records );
    node->accept( collector );
    return index;
}
