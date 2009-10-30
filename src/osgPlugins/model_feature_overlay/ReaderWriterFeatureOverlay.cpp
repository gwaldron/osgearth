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
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/TransformFilter>
#include <osg/Notify>
#include <osg/Geode>
#include <osg/Geometry>
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

class FeatureOverlaySource : public ModelSource
{
public:
    FeatureOverlaySource( const PluginOptions* options, int sourceId ) : ModelSource( options ),
        _sourceId( sourceId ),
        _textureUnit( 1 ),
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

        _textureUnit = conf.value<int>( PROP_TEXTURE_UNIT, _textureUnit );
        _textureSize = conf.value<int>( PROP_TEXTURE_SIZE, _textureSize );
    }


    osg::Node* getNode( ProgressCallback* progress =0L )
    {
        if ( !_features.valid() ) return 0L;

        bool isGeocentric =  _map->getCoordinateSystemType() == Map::CSTYPE_GEOCENTRIC;
        FeatureProfile::GeometryType geomType = _features->getFeatureProfile()->getGeometryType();
        GLenum prim =
            geomType == FeatureProfile::GEOM_LINE ? GL_LINE_STRIP :
            geomType == FeatureProfile::GEOM_POINT ? GL_POINTS :
            geomType == FeatureProfile::GEOM_POLYGON ? GL_LINE_LOOP : // for later tessellation
            GL_POINTS;

        osg::Geode* geode = new osg::Geode();

        FilterContext context;
        context._profile = _features->getFeatureProfile();

        FilterChain chain( context );
        chain.push_back( new TransformFilter( _map->getProfile()->getSRS(), isGeocentric ) );

        osg::ref_ptr<FeatureCursor> c = _features->createCursor( FeatureQuery() );
        while( c->hasMore() )
        {
            Feature* feature = c->nextFeature();
            chain.process( feature );

            osg::Geometry* geom = new osg::Geometry();
            osg::Vec4Array* colors = new osg::Vec4Array(1);
            (*colors)[0].set( 1, 1, 1, 1 );
            geom->setColorArray( colors );
            geom->setColorBinding( osg::Geometry::BIND_OVERALL );

            osg::Vec3Array* allverts = new osg::Vec3Array();
            geom->setVertexArray( allverts );

            for( int p=0, partPtr=0; p<feature->getNumParts(); p++ )
            {                
                osg::Vec3dArray* part = feature->getPart( p );
                allverts->reserve( allverts->size() + part->size() );
                for( int v=0; v<part->size(); v++ )
                    allverts->push_back( (*part)[v] );
                geom->addPrimitiveSet( new osg::DrawArrays( prim, partPtr, part->size() ) );
                partPtr += part->size();
            }
            
            // tessellate all polygon geometries. Tessellating each geometry separately
            // with TESS_TYPE_GEOMETRY is much faster than doing the whole bunch together
            // using TESS_TYPE_DRAWABLE.
            if ( geomType == FeatureProfile::GEOM_POLYGON )
            {
                osgUtil::Tessellator tess;
                tess.setTessellationType( osgUtil::Tessellator::TESS_TYPE_GEOMETRY );
                tess.setWindingType( osgUtil::Tessellator::TESS_WINDING_POSITIVE );
                tess.retessellatePolygons( *geom );
            }

            geode->addDrawable( geom );
        }

        osg::StateSet* ss = geode->getOrCreateStateSet();
        ss->setMode( GL_LIGHTING, 0 );

        // finally, build the overlay node.
        osgSim::OverlayNode* overlayNode = new osgSim::OverlayNode();
        overlayNode->setOverlayTechnique( osgSim::OverlayNode::VIEW_DEPENDENT_WITH_PERSPECTIVE_OVERLAY );
        overlayNode->setContinuousUpdate( false );
        overlayNode->setOverlaySubgraph( geode );
        overlayNode->setOverlayBaseHeight( 0.0 );
        overlayNode->setOverlayTextureSizeHint( _textureSize );
        overlayNode->setOverlayTextureUnit( _textureUnit ); 

        return overlayNode;
    }

private:
    osg::ref_ptr<FeatureSource> _features;
    int _sourceId;
    int _textureSize;
    int _textureUnit;
    osg::ref_ptr<const Map> _map;
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

