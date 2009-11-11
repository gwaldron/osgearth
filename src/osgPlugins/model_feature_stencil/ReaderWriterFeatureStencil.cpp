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
#include <osgEarthFeatures/ExtrudeGeometryFilter>
#include <osg/Notify>
#include <osg/ClearNode>
#include <osgDB/FileNameUtils>
#include <OpenThreads/Mutex>
#include <OpenThreads/ScopedLock>
#include "StencilUtils.h"

using namespace osgEarth;
using namespace osgEarthFeatures;
using namespace OpenThreads;

#define PROP_FEATURES     "features"


class FeatureStencilSource : public ModelSource
{
public:
    FeatureStencilSource( const PluginOptions* options, int sourceId ) : ModelSource( options ),
        _sourceId( sourceId ),
        _renderBinStart( 8000 )
    {
        osg::DisplaySettings::instance()->setMinimumNumStencilBits( 8 );
    }

    void initialize( const std::string& referenceURI, const Map* map )
    {
        _map = map;

        const Config& conf = getOptions()->config();

        _features = FeatureSourceFactory::create( conf.child( PROP_FEATURES ) );
        if ( !_features.valid() )
        {
            osg::notify( osg::WARN ) << "[osgEarth] Feature Stencil driver - no valid feature source provided" << std::endl;
        }

        // load up the style catalog.
        Styling::StyleReader::readLayerStyles( getName(), conf, _styles );
    }

    osg::Node* buildClass( const Styling::StyleClass& style, int& ref_renderBin )
    {
        bool isGeocentric = _map->getCoordinateSystemType() == Map::CSTYPE_GEOCENTRIC;

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
        xform.push( features, context );
        context.isGeocentric() = isGeocentric;

        // Extrude the geometry in both directions to build a stencil volume:
        ExtrudeGeometryFilter extrude( -250000, 500000 );
        extrude.push( features, context );

        //TESTING:
        //return extrude.takeOutput( context );

        // build some capped volumes:
        osg::Node* volumes = extrude.getOutput( context );

        // render the volumes to the stencil buffer:
        osg::Node* geomPass = StencilUtils::createGeometryPass( volumes, ref_renderBin );

        // render a full screen quad to write to the masked pixels:
        osg::Vec4ub maskColor = style.getColor( context.profile()->getGeometryType() );
        osg::Node* maskPass = StencilUtils::createMaskPass( maskColor, ref_renderBin );

        osg::Group* classGroup = new osg::Group();
        classGroup->setName( style.name() );
        classGroup->addChild( geomPass );
        classGroup->addChild( maskPass );
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
        ss->setMode( GL_LIGHTING, 0 );

        // install the node.
        return group;

        //return 0L; // we installed it ourselves :)
    }

private:
    osg::ref_ptr<FeatureSource> _features;
    int _sourceId;
    int _renderBinStart;
    osg::ref_ptr<const Map> _map;
    Styling::StyleCatalog _styles;
};


class ReaderWriterFeatureStencil : public osgDB::ReaderWriter
{
public:
    ReaderWriterFeatureStencil()
    {
        supportsExtension( "osgearth_model_feature_stencil", "osgEarth feature stencil plugin" );
    }

    virtual const char* className()
    {
        return "osgEarth Feature Stencil Model Plugin";
    }

    FeatureStencilSource* create( const PluginOptions* options )
    {
        ScopedLock<Mutex> lock( _sourceIdMutex );
        FeatureStencilSource* obj = new FeatureStencilSource( options, _sourceId );
        if ( obj ) _sourceMap[_sourceId++] = obj;
        return obj;
    }

    FeatureStencilSource* get( int sourceId )
    {
        ScopedLock<Mutex> lock( _sourceIdMutex );
        return _sourceMap[sourceId].get();
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        ReaderWriterFeatureStencil* nonConstThis = const_cast<ReaderWriterFeatureStencil*>(this);
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

        ReaderWriterFeatureStencil* nonConstThis = const_cast<ReaderWriterFeatureStencil*>(this);
        return ReadResult( nonConstThis->get( sourceId ) );
    }

protected:
    Mutex _sourceIdMutex;
    int _sourceId;
    std::map<int, osg::ref_ptr<FeatureStencilSource> > _sourceMap;
};

REGISTER_OSGPLUGIN(osgearth_model_feature_stencil, ReaderWriterFeatureStencil)

