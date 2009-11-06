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
using namespace osgEarthFeatures;
using namespace OpenThreads;

#define PROP_FEATURES     "features"
#define PROP_TEXTURE_UNIT "texture_unit"
#define PROP_TEXTURE_SIZE "texture_size"
#define PROP_STYLE        "style"

class FeatureOverlaySource : public ModelSource
{
public:
    FeatureOverlaySource( const PluginOptions* options, int sourceId ) : ModelSource( options ),
        _sourceId( sourceId ),
        _textureUnit( 0 ),
        _textureSize( 1024 )
    {
        //TODO
    }

    void initialize( const std::string& referenceURI, const Map* map )
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

        if ( conf.hasChild( PROP_STYLE ) )
        {
            Styling::StyleReader::read( conf.child(PROP_STYLE), _styles );
        }
    }


    osg::Node* getNode( ProgressCallback* progress =0L )
    {
        if ( !_features.valid() ) return 0L;

        // figure out which rule to use to style the geometry.
        Styling::NamedLayer styleLayer;
        bool hasStyle = _styles.getNamedLayer( _features->getName(), styleLayer );

        bool isGeocentric = _map->getCoordinateSystemType() == Map::CSTYPE_GEOCENTRIC;

        // read all features into a list:
        FeatureList features;
        osg::ref_ptr<FeatureCursor> c = _features->createCursor( FeatureQuery() );
        while( c->hasMore() )
            features.push_back( c->nextFeature() );

        // A processing context to use with the filters:
        FilterContext context;
        context._profile = _features->getFeatureProfile();

        // Transform them into the map's SRS:
        TransformFilter xform( _map->getProfile()->getSRS(), isGeocentric );
        xform.push( features, context );

        // Build geometry:
        BuildGeometryFilter buildGeom;

        // apply the style rule if we have one:
        if ( hasStyle && !styleLayer.empty() )
            buildGeom.setStyleRule( *styleLayer.userStyle().featureTypeStyle().rules().begin() );

        buildGeom.push( features, context );

        osg::Node* result = buildGeom.getOutput( context );
        if ( result )
        {
            // set up the appearance: (Later the symbolizer engine will do this)
            osg::StateSet* ss = result->getOrCreateStateSet();
            ss->setMode( GL_LIGHTING, 0 );
            ss->setAttributeAndModes( new osg::LineWidth( 2 ) );

            // finally, build the overlay node.
            osgSim::OverlayNode* overlayNode = new osgSim::OverlayNode();
            overlayNode->setOverlayTechnique( osgSim::OverlayNode::VIEW_DEPENDENT_WITH_PERSPECTIVE_OVERLAY );
            overlayNode->setContinuousUpdate( false );
            overlayNode->setOverlaySubgraph( result );
            overlayNode->setOverlayBaseHeight( 0.0 );
            overlayNode->setOverlayTextureSizeHint( _textureSize );
            overlayNode->setOverlayTextureUnit( _textureUnit ); 

            result = overlayNode;
        }

        return result;
    }

private:
    osg::ref_ptr<FeatureSource> _features;
    int _sourceId;
    int _textureSize;
    int _textureUnit;
    osg::ref_ptr<const Map> _map;
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

