/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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

#include <osgEarth/Registry>
#include <osgEarth/Map>
#include <osgEarthFeatures/FeatureModelSource>
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/BufferFilter>
#include <osgEarthFeatures/TransformFilter>
#include <osgEarthFeatures/ResampleFilter>
#include <osgEarthFeatures/ConvertTypeFilter>
#include <osgEarthFeatures/FeatureGridder>
#include <osgEarthFeatures/Styling>
#include <osg/Notify>
#include <osg/MatrixTransform>
#include <osg/ClusterCullingCallback>
#include <osgDB/FileNameUtils>
#include <OpenThreads/Mutex>
#include <OpenThreads/ScopedLock>
#include "StencilUtils.h"

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace OpenThreads;

#define PROP_EXTRUSION_DISTANCE     "extrusion_distance"
#define PROP_DENSIFICATION_THRESH   "densification_threshold"

#define DEFAULT_EXTRUSION_DISTANCE      300000.0
#define DEFAULT_DENSIFICATION_THRESHOLD 1000000.0
#define RENDER_BIN_START                80000

class FeatureStencilModelSource : public FeatureModelSource
{
public:
    FeatureStencilModelSource( const PluginOptions* options, int renderBinStart, int sourceId ) :
        FeatureModelSource( options ),
        _sourceId( sourceId ),
        _renderBinStart( renderBinStart ),
        _showVolumes( false ),
        _extrusionDistance( DEFAULT_EXTRUSION_DISTANCE ),
        _densificationThresh( DEFAULT_DENSIFICATION_THRESHOLD )
    {
        if ( osg::DisplaySettings::instance()->getMinimumNumStencilBits() < 8 )
        {
            osg::DisplaySettings::instance()->setMinimumNumStencilBits( 8 );
        }

        const Config& conf = getOptions()->config();

        // overrides the default stencil volume extrusion size
        if ( conf.hasValue( PROP_EXTRUSION_DISTANCE ) )
            _extrusionDistance = conf.value<double>( PROP_EXTRUSION_DISTANCE, DEFAULT_EXTRUSION_DISTANCE );

        // overrides the default segment densification threshold.
        _densificationThresh = conf.value<double>( PROP_DENSIFICATION_THRESH, DEFAULT_DENSIFICATION_THRESHOLD );

        // debugging:
        _showVolumes = conf.child( "debug" ).attr( "show_volumes" ) == "true";
        if ( _showVolumes )
            _lit = false;
    }

    //override
    void initialize( const std::string& referenceURI, const Map* map )
    {
        _map = map;

        if ( !_extrusionDistance.isSet() )
        {
            // figure out a "reasonable" default extrusion size
            if ( _map->isGeocentric() )
                _extrusionDistance = 300000.0; // meters geocentric
            else if ( _map->getProfile()->getSRS()->isGeographic() )
                _extrusionDistance = 5.0; // degrees-as-distance
            else
                _extrusionDistance = 12000.0; // meters
        }
    }

    // implementation-specific data to pass to buildNodeForStyle:
    struct BuildData : public osg::Referenced {
        BuildData( int renderBinStart ) : _renderBin( renderBinStart ) { }
        int _renderBin;
    };

    //override
    osg::Referenced* createBuildData()
    {
        return new BuildData( _renderBinStart );
    }    
    
    //override
    osg::Node* renderFeaturesForStyle( const Style& style, FeatureList& features, osg::Referenced* data )
    {
        BuildData* buildData = static_cast<BuildData*>(data);

        // Scan the geometry to see if it includes line data, since that will require 
        // buffering:
        bool hasLines = false;
        for( FeatureList::iterator i = features.begin(); i != features.end(); ++i )
        {
            Feature* f = i->get();
            if ( f->getGeometry() && f->getGeometry()->getComponentType() == Geometry::TYPE_LINESTRING )
            {
                hasLines = true;
                break;
            }
        }

        bool isGeocentric = _map->isGeocentric();

        // A processing context to use with the filters:
        FilterContext context;
        context.profile() = getFeatureSource()->getFeatureProfile();

        // If the geometry is lines, we need to buffer them before they will work with stenciling
        if ( hasLines )
        {
            BufferFilter buffer;
            buffer.distance() = 0.5 * style.lineSymbolizer().stroke().width();
            buffer.capStyle() = style.lineSymbolizer().stroke().lineCap();
            context = buffer.push( features, context );
        }

        // Transform them into the map's SRS:
        TransformFilter xform( _map->getProfile()->getSRS(), isGeocentric );
        context = xform.push( features, context );

        if ( isGeocentric )
        {
            // We need to make sure that on a round globe, the points are sampled such that
            // long segments follow the curvature of the earth. By the way, if a Buffer was
            // applied, that will also remove colinear segment points. Resample the points to 
            // achieve a usable tesselation.
            ResampleFilter resample;
            resample.minLength() = 0.0;
            resample.maxLength() = _densificationThresh;
            resample.perturbationThreshold() = 0.1;
            context = resample.push( features, context );
        }

        // Extrude and cap the geometry in both directions to build a stencil volume:
        osg::ref_ptr<osg::Group> volumes = new osg::Group();

        for( FeatureList::iterator i = features.begin(); i != features.end(); ++i )
        {
            osg::Node* volume = StencilUtils::createVolume(
                i->get()->getGeometry(),
                -_extrusionDistance.get(),
                _extrusionDistance.get() * 2.0,
                context );

            if ( volume )
                volumes->addChild( volume );
        }

        osg::Group* result = 0L;

        // Resolve the localizing reference frame if necessary:
        if ( context.hasReferenceFrame() )
            result = new osg::MatrixTransform( context.inverseReferenceFrame() );
        else
            result = new osg::Group();

        if ( _showVolumes )
        {
            result->addChild( volumes.get() );
        }
        else
        {
            //// render the volumes to the stencil buffer:
            osg::Node* geomPass = StencilUtils::createGeometryPass( volumes.get(), buildData->_renderBin );
            result->addChild( geomPass );

            // render a full screen quad to write to the masked pixels:
            osg::Vec4ub maskColor = style.getColor( hasLines ? Geometry::TYPE_LINESTRING : Geometry::TYPE_POLYGON );
            osg::Node* maskPass = StencilUtils::createMaskPass( maskColor, buildData->_renderBin );
            result->addChild( maskPass );
        }

        return result;
    }

    //override
    //osg::Node* renderStyle( const Style& style, FeatureCursor* cursor, osg::Referenced* data )
    //{
    //    BuildData* buildData = static_cast<BuildData*>(data);

    //    bool isGeocentric = _map->isGeocentric();

    //    // read all features into a list, transforming geometry types and calculating
    //    // the extent.
    //    FeatureList features;
    //    bool hasLines = false;
    //    Bounds featuresBounds;

    //    while( cursor->hasMore() )
    //    {
    //        Feature* feature = cursor->nextFeature();
    //        Geometry* geom = feature->getGeometry();
    //        if ( geom )
    //        {
    //            // apply a type override if requested:
    //            if ( _geomTypeOverride.isSet() && _geomTypeOverride.get() != geom->getComponentType() )
    //            {
    //                geom = geom->cloneAs( _geomTypeOverride.get() );
    //                if ( geom )
    //                    feature->setGeometry( geom );
    //            }
    //        }
    //        if ( geom )
    //        {
    //            features.push_back( feature );
    //            if ( !hasLines && geom->getType() == Geometry::TYPE_LINESTRING )
    //                hasLines = true;

    //            featuresBounds.expandToInclude( geom->getBounds() );
    //        }
    //    }

    //    // Grid the features:
    //    FeatureGridder gridder( 
    //        features, 
    //        featuresBounds,
    //        gridCellSize().value(),
    //        gridCellSize().value() );
    //    
    //    osg::Group* classGroup = new osg::Group();
    //    classGroup->setName( style.name() );

    //    for( int cell=0; cell < gridder.getNumCells(); ++cell )
    //    {
    //        // Fetch the next cell
    //        FeatureList cellFeatures;
    //        gridder.getCell( cell, cellFeatures );

    //        // A processing context to use with the filters:
    //        FilterContext context;
    //        context.profile() = getFeatureSource()->getFeatureProfile();

    //        // If the geometry is lines, we need to buffer them before they will work with stenciling
    //        if ( hasLines )
    //        {
    //            BufferFilter buffer;
    //            buffer.distance() = 0.5 * style.lineSymbolizer().stroke().width();
    //            buffer.capStyle() = style.lineSymbolizer().stroke().lineCap();
    //            context = buffer.push( cellFeatures, context );
    //        }

    //        // Transform them into the map's SRS:
    //        TransformFilter xform( _map->getProfile()->getSRS(), isGeocentric );
    //        context = xform.push( cellFeatures, context );

    //        if ( isGeocentric )
    //        {
    //            // We need to make sure that on a round globe, the points are sampled such that
    //            // long segments follow the curvature of the earth. By the way, if a Buffer was
    //            // applied, that will also remove colinear segment points. Resample the points to 
    //            // achieve a usable tesselation.
    //            ResampleFilter resample;
    //            resample.minLength() = 0.0;
    //            resample.maxLength() = _densificationThresh;
    //            resample.perturbationThreshold() = 0.1;
    //            context = resample.push( cellFeatures, context );
    //        }

    //        // Extrude and cap the geometry in both directions to build a stencil volume:
    //        osg::ref_ptr<osg::Group> volumes = new osg::Group();

    //        for( FeatureList::iterator i = cellFeatures.begin(); i != cellFeatures.end(); ++i )
    //        {
    //            osg::Node* volume = StencilUtils::createVolume(
    //                i->get()->getGeometry(),
    //                -_extrusionDistance.get(),
    //                _extrusionDistance.get() * 2.0,
    //                context );

    //            if ( volume )
    //                volumes->addChild( volume );
    //        }

    //        osg::Group* cellGroup = 0L;

    //        // Resolve the localizing reference frame if necessary:
    //        if ( context.hasReferenceFrame() )
    //            cellGroup = new osg::MatrixTransform( context.inverseReferenceFrame() );
    //        else
    //            cellGroup = new osg::Group();

    //        if ( _showVolumes )
    //        {
    //            cellGroup->addChild( volumes.get() );
    //        }
    //        else
    //        {
    //            //// render the volumes to the stencil buffer:
    //            osg::Node* geomPass = StencilUtils::createGeometryPass( volumes.get(), buildData->_renderBin );
    //            cellGroup->addChild( geomPass );

    //            // render a full screen quad to write to the masked pixels:
    //            osg::Vec4ub maskColor = style.getColor( hasLines ? Geometry::TYPE_LINESTRING : Geometry::TYPE_POLYGON );
    //            osg::Node* maskPass = StencilUtils::createMaskPass( maskColor, buildData->_renderBin );
    //            cellGroup->addChild( maskPass );
    //        }

    //        osg::ClusterCullingCallback* ccc = new osg::ClusterCullingCallback();
    //        ccc->

    //        classGroup->addChild( cellGroup );
    //    }

    //    // lock out the optimizer:
    //    classGroup->setDataVariance( osg::Object::DYNAMIC );

    //    return classGroup;
    //}



private:
    int _sourceId;
    int _renderBinStart;
    osg::ref_ptr<const Map> _map;
    optional<double> _extrusionDistance;
    double _densificationThresh;
    bool _showVolumes;
};


class FeatureStencilModelSourceFactory : public osgDB::ReaderWriter
{
public:
    FeatureStencilModelSourceFactory() :
      _renderBinStart( RENDER_BIN_START )
    {
        supportsExtension( "osgearth_model_feature_stencil", "osgEarth feature stencil plugin" );
    }

    virtual const char* className()
    {
        return "osgEarth Feature Stencil Model Plugin";
    }

    FeatureStencilModelSource* create( const PluginOptions* options )
    {
        ScopedLock<Mutex> lock( _sourceIdMutex );
        FeatureStencilModelSource* obj = new FeatureStencilModelSource( options, _renderBinStart, _sourceId );
        _renderBinStart += 1000;
        if ( obj ) _sourceMap[_sourceId++] = obj;
        return obj;
    }

    FeatureStencilModelSource* get( int sourceId )
    {
        ScopedLock<Mutex> lock( _sourceIdMutex );
        return _sourceMap[sourceId].get();
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        FeatureStencilModelSourceFactory* nonConstThis = const_cast<FeatureStencilModelSourceFactory*>(this);
        return nonConstThis->create( static_cast<const PluginOptions*>(options) );
    }

    // NOTE: this doesn't do anything, yet. it's a template for recursing into the
    // plugin during pagedlod traversals. 
    virtual ReadResult readNode(const std::string& fileName, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( fileName )))
            return ReadResult::FILE_NOT_HANDLED;
    
        std::string stripped = osgDB::getNameLessExtension( fileName );
        int sourceId = 0;
        sscanf( stripped.c_str(), "%d", &sourceId );

        FeatureStencilModelSourceFactory* nonConstThis = const_cast<FeatureStencilModelSourceFactory*>(this);
        return ReadResult( nonConstThis->get( sourceId ) );
    }

protected:
    Mutex _sourceIdMutex;
    int _renderBinStart;
    int _sourceId;
    std::map<int, osg::ref_ptr<FeatureStencilModelSource> > _sourceMap;
};

REGISTER_OSGPLUGIN(osgearth_model_feature_stencil, FeatureStencilModelSourceFactory)

