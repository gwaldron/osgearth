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
#include <osgEarthFeatures/CropFilter>
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

#define PROP_FEATURES     "features"
#define PROP_TEXTURE_UNIT "texture_unit"
#define PROP_TEXTURE_SIZE "texture_size"
#define PROP_OVERLAY_TECH "overlay_technique"
#define PROP_BASE_HEIGHT  "base_height"

#define VAL_OT_ODWOO      "object_dependent_with_orthographic_overlay"
#define VAL_OT_VDWOO      "view_dependent_with_orthographic_overlay"
#define VAL_OT_VDWPO      "view_dependent_with_perspective_overlay"

class FeatureOverlaySource : public ModelSource
{
public:
    FeatureOverlaySource( const PluginOptions* options, int sourceId ) : ModelSource( options ),
        _sourceId( sourceId ),
        _textureUnit( 0 ),
        _textureSize( 1024 ),
        _baseHeight( 0.0 ),
        _overlayTech( osgSim::OverlayNode::VIEW_DEPENDENT_WITH_PERSPECTIVE_OVERLAY )
    {
        //TODO
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

        // omitting the texture unit implies "AUTO" mode - MapNode will set one automatically
        if ( conf.hasValue( PROP_TEXTURE_UNIT ) )
            _textureUnit = conf.value<int>( PROP_TEXTURE_UNIT, _textureUnit );
        else
            _textureUnit = 0; // AUTO

        _textureSize = conf.value<int>( PROP_TEXTURE_SIZE, _textureSize );

        _baseHeight = conf.value<double>( PROP_BASE_HEIGHT, _baseHeight );

        if ( conf.hasValue( PROP_OVERLAY_TECH ) )
        {
            if ( conf.value( PROP_OVERLAY_TECH ) == VAL_OT_ODWOO )
                _overlayTech = osgSim::OverlayNode::OBJECT_DEPENDENT_WITH_ORTHOGRAPHIC_OVERLAY;
            else if ( conf.value( PROP_OVERLAY_TECH ) == VAL_OT_VDWOO )
                _overlayTech = osgSim::OverlayNode::VIEW_DEPENDENT_WITH_ORTHOGRAPHIC_OVERLAY;
            else if ( conf.value( PROP_OVERLAY_TECH ) == VAL_OT_VDWPO )
                _overlayTech = osgSim::OverlayNode::VIEW_DEPENDENT_WITH_PERSPECTIVE_OVERLAY;
        }

        // load up the style catalog.
        Styling::StyleReader::readLayerStyles( getName(), conf, _styles );
    }

    osg::Node* buildClass( const Styling::StyleClass& style )
    {
        bool isGeocentric = _map->getCoordinateSystemType() == osgEarth::Map::CSTYPE_GEOCENTRIC;

        //osg::notify(osg::NOTICE)
        //    << "Building class " << style.name() << ", SQL = " << style.query().expression() << std::endl;

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
        context = xform.push( features, context );

        // Build geometry:
        BuildGeometryFilter buildGeom;

        // apply the style rule if we have one:
        osg::ref_ptr<osg::Node> result;
        buildGeom.setStyleClass( style );
        context = buildGeom.push( features, result, context );

        return result.release();
    }

    osg::Node* createOrInstallNode( MapNode* mapNode, ProgressCallback* progress =0L )
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

        // finally, build the overlay node.
        osgSim::OverlayNode* overlayNode = new osgSim::OverlayNode();
        overlayNode->setName( this->getName() );
        overlayNode->setOverlayTechnique( _overlayTech );
        overlayNode->setOverlayBaseHeight( _baseHeight );
        overlayNode->setOverlayTextureSizeHint( _textureSize );
        overlayNode->setOverlayTextureUnit( _textureUnit );
        overlayNode->setContinuousUpdate( false );
        overlayNode->setOverlaySubgraph( group );

        return overlayNode;
    }

private:
    osg::ref_ptr<FeatureSource> _features;
    int _sourceId;
    int _textureSize;
    int _textureUnit;
    double _baseHeight;
    osgSim::OverlayNode::OverlayTechnique _overlayTech;
    osg::ref_ptr<const osgEarth::Map> _map;
    Styling::StyleCatalog _styles;
};


class ReaderWriterFeatureOverlay : public osgDB::ReaderWriter
{
public:
    ReaderWriterFeatureOverlay()
    {
        supportsExtension( "osgearth_model_feature_overlay", "osgEarth feature overlay plugin" );
    }

    virtual const char* className()
    {
        return "osgEarth Feature Overlay Model Plugin";
    }

    FeatureOverlaySource* create( const PluginOptions* options )
    {
        ScopedLock<Mutex> lock( _sourceIdMutex );
        FeatureOverlaySource* obj = new FeatureOverlaySource( options, _sourceId );
        if ( obj ) _sourceMap[_sourceId++] = obj;
        return obj;
    }

    FeatureOverlaySource* get( int sourceId )
    {
        ScopedLock<Mutex> lock( _sourceIdMutex );
        return _sourceMap[sourceId].get();
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        ReaderWriterFeatureOverlay* nonConstThis = const_cast<ReaderWriterFeatureOverlay*>(this);
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

        ReaderWriterFeatureOverlay* nonConstThis = const_cast<ReaderWriterFeatureOverlay*>(this);
        return ReadResult( nonConstThis->get( sourceId ) );
    }

protected:
    Mutex _sourceIdMutex;
    int _sourceId;
    std::map<int, osg::ref_ptr<FeatureOverlaySource> > _sourceMap;
};

REGISTER_OSGPLUGIN(osgearth_model_feature_overlay, ReaderWriterFeatureOverlay) 