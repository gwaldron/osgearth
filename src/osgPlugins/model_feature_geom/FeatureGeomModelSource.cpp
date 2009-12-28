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
#include <osgEarthFeatures/TransformFilter>
#include <osgEarthFeatures/BuildGeometryFilter>
#include <osg/Notify>
#include <osg/LineWidth>
#include <osgDB/FileNameUtils>
#include <osgUtil/Tessellator>
#include <osgSim/OverlayNode>
#include <OpenThreads/Mutex>
#include <OpenThreads/ScopedLock>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace OpenThreads;

#define PROP_FEATURES      "features"
#define PROP_GEOMETRY_TYPE "geometry_type"
#define PROP_HEIGHT_OFFSET "height_offset"

class FeatureGeomModelSource : public ModelSource
{
public:
    FeatureGeomModelSource( const PluginOptions* options, int sourceId ) : ModelSource( options ),
        _sourceId( sourceId ),
        _geomTypeOverride( Geometry::TYPE_UNKNOWN ),
        _heightOffset( 0.0 )
    {
        //nop
    }

    void initialize( const std::string& referenceURI, const osgEarth::Map* map )
    {
        _map = map;

        const Config& conf = getOptions()->config();

        _features = FeatureSourceFactory::create( conf.child( PROP_FEATURES ) );
        if ( !_features.valid() )
        {
            osg::notify( osg::WARN ) << "[osgEarth] Feature Overlay driver - no valid feature source provided" << std::endl;
        }

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

        // vertical offset.
        _heightOffset = conf.value<double>( PROP_HEIGHT_OFFSET, _heightOffset );

        // load up the style catalog.
        Styling::StyleReader::readLayerStyles( getName(), conf, _styles );
    }

    osg::Node* buildClass( const Styling::StyleClass& style )
    {
        bool isGeocentric = _map->isGeocentric();

        // read all features into a list:
        FeatureList features;
        osg::ref_ptr<FeatureCursor> c = _features->createCursor( style.query() );
        while( c->hasMore() )
            features.push_back( c->nextFeature() );

        // A processing context to use with the filters:
        FilterContext context;
        context.profile() = _features->getFeatureProfile();

        // Transform them into the map's SRS:
        TransformFilter xform( _map->getProfile()->getSRS(), isGeocentric );
        xform.heightOffset() = _heightOffset;
        context = xform.push( features, context );

        // Build geometry:
        BuildGeometryFilter build;    
        if ( _geomTypeOverride.isSet() )
            build.geomTypeOverride() = _geomTypeOverride.get();

        // apply the style rule if we have one:
        osg::ref_ptr<osg::Node> result;
        build.styleClass() = style;
        context = build.push( features, result, context );

        return result.release();
    }

    osg::Node* createNode( ProgressCallback* progress =0L )
    {
        if ( !_features.valid() ) return 0L;

        // figure out which rule to use to style the geometry.
        Styling::NamedLayer styleLayer;
        bool hasStyle = _styles.getNamedLayer( getName(), styleLayer );

        osg::Group* group = new osg::Group();

        for( Styling::StyleClasses::iterator i = styleLayer.styleClasses().begin(); i != styleLayer.styleClasses().end(); ++i )
        {
            const Styling::StyleClass& style = *i;

            osg::Node* node = buildClass( style );
            if ( node )
                group->addChild( node );
        }

        osg::StateSet* ss = group->getOrCreateStateSet();
        ss->setMode( GL_LIGHTING, 0 );

        return group;
    }

private:
    osg::ref_ptr<FeatureSource> _features;
    int _sourceId;
    osg::ref_ptr<const osgEarth::Map> _map;
    Styling::StyleCatalog _styles;
    optional<Geometry::Type> _geomTypeOverride;
    double _heightOffset;
};


class FeatureGeomModelSourceFactory : public osgDB::ReaderWriter
{
public:
    FeatureGeomModelSourceFactory()
    {
        supportsExtension( "osgearth_model_feature_geom", "osgEarth feature geom plugin" );
    }

    virtual const char* className()
    {
        return "osgEarth Feature Geom Model Plugin";
    }

    FeatureGeomModelSource* create( const PluginOptions* options )
    {
        ScopedLock<Mutex> lock( _sourceIdMutex );
        FeatureGeomModelSource* obj = new FeatureGeomModelSource( options, _sourceId );
        if ( obj ) _sourceMap[_sourceId++] = obj;
        return obj;
    }

    FeatureGeomModelSource* get( int sourceId )
    {
        ScopedLock<Mutex> lock( _sourceIdMutex );
        return _sourceMap[sourceId].get();
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        FeatureGeomModelSourceFactory* nonConstThis = const_cast<FeatureGeomModelSourceFactory*>(this);
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

        FeatureGeomModelSourceFactory* nonConstThis = const_cast<FeatureGeomModelSourceFactory*>(this);
        return ReadResult( nonConstThis->get( sourceId ) );
    }

protected:
    Mutex _sourceIdMutex;
    int _sourceId;
    std::map<int, osg::ref_ptr<FeatureGeomModelSource> > _sourceMap;
};

REGISTER_OSGPLUGIN(osgearth_model_feature_geom, FeatureGeomModelSourceFactory) 
