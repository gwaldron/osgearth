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
#include <osgEarthFeatures/Filter>
#include <osgEarthSymbology/LineSymbol>
#include <osgEarthSymbology/PointSymbol>
#include <osgEarth/ECEF>
#include <osg/MatrixTransform>
#include <osg/Point>
#include <osg/LineWidth>
#include <osg/LineStipple>

using namespace osgEarth;
using namespace osgEarth::Features;

/********************************************************************************/
Filter::~Filter()
{
}

/********************************************************************************/
FeatureFilter::~FeatureFilter()
{
}

/********************************************************************************/
        
FeatureFilterRegistry::FeatureFilterRegistry()
{
}

FeatureFilterRegistry*
FeatureFilterRegistry::instance()
{
    // OK to be in the local scope since this gets called at static init time
    // by the OSGEARTH_REGISTER_ANNOTATION macro
    static FeatureFilterRegistry* s_singleton =0L;
    static Threading::Mutex    s_singletonMutex;

    if ( !s_singleton )
    {
        Threading::ScopedMutexLock lock(s_singletonMutex);
        if ( !s_singleton )
        {
            s_singleton = new FeatureFilterRegistry();
        }
    }
    return s_singleton;
}

void
FeatureFilterRegistry::add( FeatureFilterFactory* factory )
{
    _factories.push_back( factory );
}

FeatureFilter*
FeatureFilterRegistry::create( const Config& conf )
{
    for (FeatureFilterFactoryList::iterator itr = _factories.begin(); itr != _factories.end(); itr++)
    {
        FeatureFilter* filter = itr->get()->create( conf );
        if (filter) return filter;
    }
    return 0;
} 

/********************************************************************************/

FeaturesToNodeFilter::~FeaturesToNodeFilter()
{
}

void
FeaturesToNodeFilter::computeLocalizers( const FilterContext& context )
{
    if ( context.isGeoreferenced() )
    {
        if ( context.getSession()->getMapInfo().isGeocentric() )
        {
            const SpatialReference* geogSRS = context.profile()->getSRS()->getGeographicSRS();
            GeoExtent geodExtent = context.extent()->transform( geogSRS );
            if ( geodExtent.width() < 180.0 )
            {
                osg::Vec3d centroid, centroidECEF;
                geodExtent.getCentroid( centroid.x(), centroid.y() );
                geogSRS->transform( centroid, geogSRS->getECEF(), centroidECEF );
                geogSRS->getECEF()->createLocalToWorld( centroidECEF, _local2world );
                _world2local.invert( _local2world );
            }
        }

        else // projected
        {
            if ( context.extent().isSet() )
            {
                osg::Vec3d centroid;
                context.extent()->getCentroid(centroid.x(), centroid.y());

                context.extent()->getSRS()->transform(
                    centroid,
                    context.getSession()->getMapInfo().getProfile()->getSRS(),
                    centroid );

                _world2local.makeTranslate( -centroid );
                _local2world.invert( _world2local );
            }
        }
    }
}

void
FeaturesToNodeFilter::transformAndLocalize(const std::vector<osg::Vec3d>& input,
                                           const SpatialReference*        inputSRS,
                                           osg::Vec3Array*                output,
                                           const SpatialReference*        outputSRS,
                                           const osg::Matrixd&            world2local,
                                           bool                           toECEF )
{
    output->reserve( output->size() + input.size() );

    if ( toECEF )
    {
        ECEF::transformAndLocalize( input, inputSRS, output, outputSRS, world2local );
    }
    else if ( inputSRS )
    {
        std::vector<osg::Vec3d> temp( input );
        inputSRS->transform( temp, outputSRS );

        for( std::vector<osg::Vec3d>::const_iterator i = temp.begin(); i != temp.end(); ++i )
        {
            output->push_back( (*i) * world2local );
        }
    }
    else
    {
        for( std::vector<osg::Vec3d>::const_iterator i = input.begin(); i != input.end(); ++i )
        {
            output->push_back( (*i) * world2local );
        }
    }
}



void
FeaturesToNodeFilter::transformAndLocalize(const std::vector<osg::Vec3d>& input,
                                           const SpatialReference*        inputSRS,
                                           osg::Vec3Array*                output_verts,
                                           osg::Vec3Array*                output_normals,
                                           const SpatialReference*        outputSRS,
                                           const osg::Matrixd&            world2local,
                                           bool                           toECEF )
{
    // pre-allocate enough space (performance)
    output_verts->reserve( output_verts->size() + input.size() );

    if ( output_normals )
        output_normals->reserve( output_verts->size() );

    if ( toECEF )
    {
        ECEF::transformAndLocalize( input, inputSRS, output_verts, output_normals, outputSRS, world2local );
    }
    else if ( inputSRS )
    {
        std::vector<osg::Vec3d> temp( input );
        inputSRS->transform( temp, outputSRS );

        for( std::vector<osg::Vec3d>::const_iterator i = temp.begin(); i != temp.end(); ++i )
        {
            output_verts->push_back( (*i) * world2local );
            if ( output_normals )
                output_normals->push_back( osg::Vec3(0,0,1) );
        }
    }
    else
    {
        for( std::vector<osg::Vec3d>::const_iterator i = input.begin(); i != input.end(); ++i )
        {
            output_verts->push_back( (*i) * world2local );
            if ( output_normals )
                output_normals->push_back( osg::Vec3(0,0,1) );
        }
    }
}




void
FeaturesToNodeFilter::transformAndLocalize(const osg::Vec3d&              input,
                                           const SpatialReference*        inputSRS,
                                           osg::Vec3d&                    output,
                                           const SpatialReference*        outputSRS,
                                           const osg::Matrixd&            world2local,
                                           bool                           toECEF )
{
    if ( toECEF )
    {
        ECEF::transformAndLocalize( input, inputSRS, output, outputSRS, world2local );
    }
    else if ( inputSRS )
    {
        inputSRS->transform( input, outputSRS, output );
        output = output * world2local;
    }
    else
    {
        output = input * world2local;
    }
}


osg::Node*
FeaturesToNodeFilter::delocalize( osg::Node* node ) const
{
    if ( !_local2world.isIdentity() ) 
        return delocalizeAsGroup( node );
    else
        return node;
}

osg::Group*
FeaturesToNodeFilter::delocalizeAsGroup( osg::Node* node ) const
{
    osg::Group* group = createDelocalizeGroup();
    if ( node )
        group->addChild( node );
    return group;
}

osg::Group*
FeaturesToNodeFilter::createDelocalizeGroup() const
{
    osg::Group* group = _local2world.isIdentity() ?
        new osg::Group() :
        new osg::MatrixTransform( _local2world );

    return group;
}


void 
FeaturesToNodeFilter::applyLineSymbology(osg::StateSet*    stateset, 
                                         const LineSymbol* line)
{
    if ( line && line->stroke().isSet() )
    {
        if ( line->stroke()->width().isSet() )
        {
            float width = std::max( 1.0f, *line->stroke()->width() );
            if ( width != 1.0f )
            {
                stateset->setAttributeAndModes(new osg::LineWidth(width), 1);
            }
        }

        if ( line->stroke()->stipplePattern().isSet() )
        {
            stateset->setAttributeAndModes( new osg::LineStipple(
                line->stroke()->stippleFactor().value(),
                line->stroke()->stipplePattern().value() ) );
        }
    }
}

void 
FeaturesToNodeFilter::applyPointSymbology(osg::StateSet*     stateset, 
                                          const PointSymbol* point)
{
    if ( point )
    {
        float size = std::max( 0.1f, *point->size() );
        stateset->setAttributeAndModes(new osg::Point(size), 1);
    }
}
