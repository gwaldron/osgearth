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
#include <osgEarthFeatures/Filter>
#include <osgEarthSymbology/LineSymbol>
#include <osgEarthSymbology/PointSymbol>
#include <osgEarth/ECEF>
#include <osgEarth/Registry>
#include <osg/MatrixTransform>
#include <osg/Point>
#include <osg/LineWidth>
#include <osg/LineStipple>
#include <osgEarth/VirtualProgram>

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
        
#undef  LC
#define LC "[FeatureFilterRegistry] "

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

#define FEATURE_FILTER_OPTIONS_TAG "__osgEarth::FeatureFilterOptions"

FeatureFilter*
FeatureFilterRegistry::create(const Config& conf, const osgDB::Options* dbo)
{
    std::string driver = conf.key();

    osg::ref_ptr<FeatureFilter> result;

    for (FeatureFilterFactoryList::iterator itr = _factories.begin(); result == 0L && itr != _factories.end(); itr++)
    {
        result = itr->get()->create( conf );
    }

    if ( !result.valid() )
    {
        // not found; try to load from plugin.
        if ( driver.empty() )
        {
            OE_WARN << LC << "ILLEGAL- no driver set for feature filter" << std::endl;
            return 0L;
        }

        ConfigOptions options(conf);

        osg::ref_ptr<osgDB::Options> dbopt = Registry::instance()->cloneOrCreateOptions(dbo);
        dbopt->setPluginData( FEATURE_FILTER_OPTIONS_TAG, (void*)&options );

        std::string driverExt = std::string( ".osgearth_featurefilter_" ) + driver;
        result = dynamic_cast<FeatureFilter*>( osgDB::readObjectFile( driverExt, dbopt.get() ) );
    }

    if ( !result.valid() )
    {
        OE_WARN << LC << "Failed to load FeatureFilter driver \"" << driver << "\"" << std::endl;
    }

    return result.release();
} 

const ConfigOptions&
FeatureFilterDriver::getConfigOptions(const osgDB::Options* options) const
{
    static ConfigOptions s_default;
    const void* data = options->getPluginData(FEATURE_FILTER_OPTIONS_TAG);
    return data ? *static_cast<const ConfigOptions*>(data) : s_default;
}

/********************************************************************************/

#undef  LC
#define LC "[FeaturesToNodeFilter] "

FeaturesToNodeFilter::~FeaturesToNodeFilter()
{
    //nop
}

void
FeaturesToNodeFilter::computeLocalizers( const FilterContext& context )
{
    computeLocalizers(context, context.extent().get(), _world2local, _local2world);
}

void
FeaturesToNodeFilter::computeLocalizers( const FilterContext& context, const osgEarth::GeoExtent &extent, osg::Matrixd &out_w2l, osg::Matrixd &out_l2w )
{
    if ( context.isGeoreferenced() )
    {
        if ( context.getSession()->getMapInfo().isGeocentric() )
        {
            const SpatialReference* geogSRS = context.profile()->getSRS()->getGeographicSRS();
            GeoExtent geodExtent = extent.transform( geogSRS );
            if ( geodExtent.width() < 180.0 )
            {
                osg::Vec3d centroid, centroidECEF;
                geodExtent.getCentroid( centroid.x(), centroid.y() );
                geogSRS->transform( centroid, geogSRS->getECEF(), centroidECEF );
                geogSRS->getECEF()->createLocalToWorld( centroidECEF, out_l2w );
                out_w2l.invert( out_l2w );
            }
        }

        else // projected
        {
            if ( extent.isValid() )
            {
                osg::Vec3d centroid;
                extent.getCentroid(centroid.x(), centroid.y());

                extent.getSRS()->transform(
                    centroid,
                    context.getSession()->getMapInfo().getProfile()->getSRS(),
                    centroid );

                out_w2l.makeTranslate( -centroid );
                out_l2w.invert( out_w2l );
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
    return delocalize(node, _local2world);
}

osg::Node*
FeaturesToNodeFilter::delocalize( osg::Node* node, const osg::Matrixd &local2world) const
{
    if ( !local2world.isIdentity() ) 
        return delocalizeAsGroup( node, local2world );
    else
        return node;
}

osg::Group*
FeaturesToNodeFilter::delocalizeAsGroup( osg::Node* node ) const
{
    return delocalizeAsGroup( node, _local2world );
}

osg::Group*
FeaturesToNodeFilter::delocalizeAsGroup( osg::Node* node, const osg::Matrixd &local2world ) const
{
    osg::Group* group = createDelocalizeGroup(local2world);
    if ( node )
        group->addChild( node );
    return group;
}

osg::Group*
FeaturesToNodeFilter::createDelocalizeGroup() const
{
    return createDelocalizeGroup( _local2world );
}

osg::Group*
FeaturesToNodeFilter::createDelocalizeGroup( const osg::Matrixd &local2world ) const
{
    osg::Group* group = local2world.isIdentity() ?
        new osg::Group() :
        new osg::MatrixTransform( local2world );

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
            stateset->setAttributeAndModes(
                new osg::LineStipple(
                    line->stroke()->stippleFactor().value(),
                    line->stroke()->stipplePattern().value()),
                osg::StateAttribute::ON );
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
