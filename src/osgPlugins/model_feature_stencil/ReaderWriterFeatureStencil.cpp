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

#include <osgEarth/ModelSource>
#include <osgEarth/Registry>
#include <osgEarth/Map>
#include <osgEarthFeatures/Styling>
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/BufferFilter>
#include <osgEarthFeatures/TransformFilter>
#include <osgEarthFeatures/ResampleFilter>
#include <osgEarthFeatures/ConvertTypeFilter>
#include <osgEarthFeatures/FeatureGridder>
#include <osg/Notify>
#include <osg/ClearNode>
#include <osgDB/FileNameUtils>
#include <OpenThreads/Mutex>
#include <OpenThreads/ScopedLock>
#include "StencilUtils.h"

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace OpenThreads;

#define PROP_FEATURES           "features"
#define PROP_EXTRUSION_DISTANCE "extrusion_distance"
#define PROP_GEOMETRY_TYPE      "geometry_type"

class FeatureStencilModelSource : public ModelSource
{
public:
    FeatureStencilModelSource( const PluginOptions* options, int renderBinStart, int sourceId ) : ModelSource( options ),
        _sourceId( sourceId ),
        _renderBinStart( renderBinStart ),
        _showVolumes( false ),
        _extrusionDistance( 300000.0 )
    {
        if ( osg::DisplaySettings::instance()->getMinimumNumStencilBits() < 8 )
        {
            osg::DisplaySettings::instance()->setMinimumNumStencilBits( 8 );
        }
    }

    void initialize( const std::string& referenceURI, const Map* map )
    {
        _map = map;

        // figure out a "reasonable" default extrusion size
        if ( _map->isGeocentric() )
            _extrusionDistance = 300000.0; // meters geocentric
        else if ( _map->getProfile()->getSRS()->isGeographic() )
            _extrusionDistance = 5.0; // degrees-as-distance
        else
            _extrusionDistance = 12000.0; // meters

        const Config& conf = getOptions()->config();

        _features = FeatureSourceFactory::create( conf.child( PROP_FEATURES ) );
        if ( !_features.valid() )
        {
            osg::notify( osg::WARN ) << "[osgEarth] Feature Stencil driver - no valid feature source provided" << std::endl;
        }

        // overrides the default stencil volume extrusion size
        _extrusionDistance = conf.value<double>( PROP_EXTRUSION_DISTANCE, _extrusionDistance );

        // load up the style catalog.
        Styling::StyleReader::readLayerStyles( getName(), conf, _styles );

        // force a particular geometry type
        if ( conf.hasValue( PROP_GEOMETRY_TYPE ) )
        {
            // geometry type override: the config can ask that input geometry
            // be interpreted as a particular geometry type
            std::string gt = conf.value( PROP_GEOMETRY_TYPE );
            if ( gt == "line" || gt == "lines" || gt == "linestrip" )
                _geomTypeOverride = Geometry::TYPE_LINESTRING;
            else if ( gt == "point" || gt == "points" || gt == "pointset" )
                _geomTypeOverride = Geometry::TYPE_POINTSET;
            else if ( gt == "polygon" || gt == "polygons" )
                _geomTypeOverride = Geometry::TYPE_POLYGON;
        }

        // debugging:
        _showVolumes = conf.child( "debug" ).attr( "show_volumes" ) == "true";
    }


    osg::Node* buildClass( const Styling::StyleClass& style, int& ref_renderBin )
    {
        bool isGeocentric = _map->isGeocentric();

        // read all features into a list:
        FeatureList features;
        bool hasLines = false;
        osg::ref_ptr<FeatureCursor> c = _features->createCursor( style.query() );

        while( c->hasMore() )
        {
            Feature* feature = c->nextFeature();
            Geometry* geom = feature->getGeometry();
            if ( geom )
            {
                // apply a type override if requested:
                if ( _geomTypeOverride.isSet() && _geomTypeOverride.get() != geom->getComponentType() )
                {
                    geom = geom->cloneAs( _geomTypeOverride.get() );
                    if ( geom )
                        feature->setGeometry( geom );
                }
            }
            if ( geom )
            {
                features.push_back( feature );
                if ( !hasLines && geom->getType() == Geometry::TYPE_LINESTRING )
                    hasLines = true;
            }
        }

        //FilterContext cx;
        //if ( _geomTypeOverride.isSet() )
        //{
        //    ConvertTypeFilter changeType( _geomTypeOverride.get() );
        //    cx = changeType.push( features, cx );
        //
        //    if ( _geomTypeOverride.get() == Geometry::TYPE_LINESTRING )
        //        hasLines = true;
        //}

        // Grid the features:
        // TODO: use the actual input extent...
        //GeoExtent ex = osgEarth::Registry::instance()->getGlobalGeodeticProfile()->getExtent();
        //FeatureGridder gridder( features, ex, 360, 180 );
        
        osg::Group* classGroup = new osg::Group();
        classGroup->setName( style.name() );

        //for( int cell=0; cell < gridder.getNumCells(); ++cell )
        {
            FeatureList& cellFeatures = features;
            //FeatureList cellFeatures;
            //gridder.getCell( cell, cellFeatures );

            // A processing context to use with the filters:
            FilterContext context;
            context.profile() = _features->getFeatureProfile();

            // If the geometry is lines, we need to buffer them before they will work with stenciling
            if ( hasLines )
            {
                BufferFilter buffer;
                buffer.distance() = 0.5 * style.lineSymbolizer().stroke().width();
                buffer.capStyle() = style.lineSymbolizer().stroke().lineCap();
                context = buffer.push( cellFeatures, context );
            }

            // Transform them into the map's SRS:
            TransformFilter xform( _map->getProfile()->getSRS(), isGeocentric );
            context = xform.push( cellFeatures, context );

            if ( isGeocentric )
            {
                // We need to make sure that on a round globe, the points are sampled such that
                // long segments follow the curvature of the earth. By the way, if a Buffer was
                // applied, that will also remove colinear segment points. Resample the points to 
                // achieve a usable tesselation.
                ResampleFilter simplify;
                simplify.minLength() = 1.0; // need?
                simplify.maxLength() = 100000.0;
                context = simplify.push( cellFeatures, context );
            }

            // Extrude and cap the geometry in both directions to build a stencil volume:
            osg::ref_ptr<osg::Group> volumes = new osg::Group();

            for( FeatureList::iterator i = cellFeatures.begin(); i != cellFeatures.end(); ++i )
            {
                osg::Node* volume = StencilUtils::createVolume(
                    i->get()->getGeometry(),
                    -_extrusionDistance,
                    _extrusionDistance * 2.0,
                    context );

                if ( volume )
                    volumes->addChild( volume );
            }

            //ExtrudeGeometryFilter extrude( -_extrusionDistance, _extrusionDistance*2.0 );
            //context = extrude.push( cellFeatures, volumes, context );

            if ( _showVolumes )
            {
                classGroup->addChild( volumes.get() );
            }
            else
            {
                // render the volumes to the stencil buffer:
                osg::Node* geomPass = StencilUtils::createGeometryPass( volumes.get(), ref_renderBin );
                classGroup->addChild( geomPass );

                // render a full screen quad to write to the masked pixels:
                osg::Vec4ub maskColor = style.getColor( hasLines ? Geometry::TYPE_LINESTRING : Geometry::TYPE_POLYGON );
                osg::Node* maskPass = StencilUtils::createMaskPass( maskColor, ref_renderBin );
                classGroup->addChild( maskPass );
            }
        }

        // lock out the optimizer:
        classGroup->setDataVariance( osg::Object::DYNAMIC );
        return classGroup;
    }

    osg::Node* createOrInstallNode( MapNode* mapNode, ProgressCallback* progress =0L )
    {
        if ( !_features.valid() ) return 0L;

        // figure out which rule to use to style the geometry.
        Styling::NamedLayer styleLayer;
        bool hasStyle = _styles.getNamedLayer( getName(), styleLayer );

        osg::Group* group = new osg::Group();

        int renderBin = _renderBinStart;
        for( Styling::StyleClasses::iterator i = styleLayer.styleClasses().begin(); i != styleLayer.styleClasses().end(); ++i )
        {
            const Styling::StyleClass& style = *i;

            osg::Node* node = buildClass( style, renderBin );
            if ( node )
                group->addChild( node );
        }

        osg::StateSet* ss = group->getOrCreateStateSet();
        if ( !_showVolumes )
            ss->setMode( GL_LIGHTING, 0 );

        // install the node.

        group->setNodeMask( 0xfffffffe );

        return group;
    }

private:
    osg::ref_ptr<FeatureSource> _features;
    int _sourceId;
    int _renderBinStart;
    osg::ref_ptr<const Map> _map;
    Styling::StyleCatalog _styles;
    bool _showVolumes;
    double _extrusionDistance;
    optional<Geometry::Type> _geomTypeOverride;
};


class FeatureStencilModelSourceFactory : public osgDB::ReaderWriter
{
public:
    FeatureStencilModelSourceFactory() :
      _renderBinStart( 80000 )
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

