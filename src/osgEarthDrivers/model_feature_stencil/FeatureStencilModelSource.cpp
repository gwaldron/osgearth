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
#include <osgEarthFeatures/StencilVolumeNode>
#include <osg/Notify>
#include <osg/MatrixTransform>
#include <osg/ClusterCullingCallback>
#include <osg/Geode>
#include <osg/Projection>
#include <osg/MatrixTransform>
#include <osgDB/FileNameUtils>
#include <OpenThreads/Mutex>
#include <OpenThreads/ScopedLock>

#include "FeatureStencilModelOptions"

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers;
using namespace OpenThreads;

#define RENDER_BIN_START 100
#define MAX_NUM_STYLES   100

#define OFF_PROTECTED osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED

/** Creates a full-screen quad to fill in the colors on the stencil volume. */
static
osg::Node* createColorNode( const osg::Vec4f& color )
{
    // make a full screen quad:
    osg::Geometry* quad = new osg::Geometry();
    osg::Vec3Array* verts = new osg::Vec3Array(4);
    (*verts)[0].set( 0, 1, 0 );
    (*verts)[1].set( 0, 0, 0 );
    (*verts)[2].set( 1, 0, 0 );
    (*verts)[3].set( 1, 1, 0 );
    quad->setVertexArray( verts );
    quad->addPrimitiveSet( new osg::DrawArrays( osg::PrimitiveSet::QUADS, 0, 4 ) );
    osg::Vec4Array* colors = new osg::Vec4Array(1);
    (*colors)[0] = color;
    quad->setColorArray( colors );
    quad->setColorBinding( osg::Geometry::BIND_OVERALL );
    osg::Geode* quad_geode = new osg::Geode();
    quad_geode->addDrawable( quad );

    osg::StateSet* quad_ss = quad->getOrCreateStateSet();
    quad_ss->setMode( GL_CULL_FACE, OFF_PROTECTED );
    quad_ss->setMode( GL_DEPTH_TEST, OFF_PROTECTED );
    quad_ss->setMode( GL_LIGHTING, OFF_PROTECTED );
    osg::MatrixTransform* abs = new osg::MatrixTransform();
    abs->setReferenceFrame( osg::Transform::ABSOLUTE_RF );
    abs->setMatrix( osg::Matrix::identity() );
    abs->addChild( quad_geode );

    osg::Projection* proj = new osg::Projection();
    proj->setMatrix( osg::Matrix::ortho(0, 1, 0, 1, 0, -1) );
    proj->addChild( abs );

    proj->getOrCreateStateSet()->setMode( GL_BLEND, 1 );    

    return proj;
}


class FeatureStencilModelSource : public FeatureModelSource
{
public:
    FeatureStencilModelSource( const PluginOptions* options, int renderBinStart ) :
        FeatureModelSource( options ),
        _renderBinStart( renderBinStart )
    {
        _options = dynamic_cast<const FeatureStencilModelOptions*>( options );
        if ( !_options )
            _options = new FeatureStencilModelOptions( options );

        // make sure we have stencil bits. Note, this only works before
        // a viewer gets created. You may need to allocate stencil bits
        // yourself if you make this object after realizing a viewer.
        if ( osg::DisplaySettings::instance()->getMinimumNumStencilBits() < 8 )
        {
            osg::DisplaySettings::instance()->setMinimumNumStencilBits( 8 );
        }
    }

    //override
    void initialize( const std::string& referenceURI, const Map* map )
    {
        FeatureModelSource::initialize( referenceURI, map );

        _map = map;

        if ( _options->extrusionDistance().isSet() )
        {
            _extrusionDistance = _options->extrusionDistance().value();
        }
        else
        {
            if ( _map->isGeocentric() )
                _extrusionDistance = 300000.0; // meters geocentric
            else if ( _map->getProfile()->getSRS()->isGeographic() )
                _extrusionDistance = 5.0; // degrees-as-meters
            else
                _extrusionDistance = 12000.0; // meters
        }
    }

    // implementation-specific data to pass to buildNodeForStyle:
    struct BuildData : public osg::Referenced
    {
        BuildData( int renderBinStart ) : _renderBin( renderBinStart ) { }
        int _renderBin;

        // pairs a style name with a starting render bin.
        typedef std::pair<std::string,StencilVolumeNode*> StyleGroup;
        std::vector<StyleGroup> _styleGroups;

        bool getStyleNode( const std::string& styleName, StencilVolumeNode*& out_svn ) {
            for(std::vector<StyleGroup>::iterator i = _styleGroups.begin(); i != _styleGroups.end(); ++i ) {
                if( i->first == styleName ) {
                    out_svn = i->second;
                    return true;
                }
            }
            return false;
        }
    };

    //override
    osg::Referenced* createBuildData()
    {
        return new BuildData( _renderBinStart );
    }    
    
    //override
    osg::Node* renderFeaturesForStyle(const Style& style, FeatureList& features, osg::Referenced* data, osg::Node** out_createdNode )
    {
        BuildData* buildData = static_cast<BuildData*>(data);

        StencilVolumeNode* styleNode = 0L;
        bool styleNodeAlreadyCreated = buildData->getStyleNode( style.name(), styleNode );

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
            buffer.distance() = 0.5 * style.lineSymbolizer()->stroke()->width().value();
            buffer.capStyle() = style.lineSymbolizer()->stroke()->lineCap().value();
            context = buffer.push( features, context );
        }

        // Transform them into the map's SRS, localizing the verts along the way:
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
            resample.maxLength() = _options->densificationThreshold().value();
            resample.perturbationThreshold() = 0.1;
            context = resample.push( features, context );
        }

        // Extrude and cap the geometry in both directions to build a stencil volume:
        osg::Group* volumes = 0L;

        for( FeatureList::iterator i = features.begin(); i != features.end(); ++i )
        {
            osg::Node* volume = StencilVolumeFactory::createVolume(
                i->get()->getGeometry(),
                -_extrusionDistance,
                _extrusionDistance * 2.0,
                context );

            if ( volume )
            {
                if ( !volumes )
                    volumes = new osg::Group();
                volumes->addChild( volume );
            }
        }

        osg::Node* result = 0L;

        if ( volumes )
        {
            // Resolve the localizing reference frame if necessary:
            if ( context.hasReferenceFrame() )
            {
                osg::MatrixTransform* xform = new osg::MatrixTransform( context.inverseReferenceFrame() );
                xform->addChild( volumes );
                volumes = xform;
            }

            // Apply an LOD if required:
            if ( getOptions()->minRange().isSet() || getOptions()->maxRange().isSet() )
            {
                osg::LOD* lod = new osg::LOD();
                lod->addChild( volumes, getOptions()->minRange().value(), getOptions()->maxRange().value() );
                volumes = lod;
            }

            if ( _options->showVolumes() == true )
            {
                result = volumes;
            }
            else
            {
                if ( !styleNodeAlreadyCreated )
                {
                    osg::notify(osg::NOTICE) << "[osgEarth] Creating new style group for '" << style.name() << "'" << std::endl;
                    styleNode = new StencilVolumeNode( _options->mask().value(), _options->inverted().value() );
                    if ( _options->mask() == false )
                    {
                        osg::Vec4f maskColor = style.getColor( hasLines ? Geometry::TYPE_LINESTRING : Geometry::TYPE_POLYGON );
                        styleNode->addChild( createColorNode(maskColor) );
                    }
                    buildData->_renderBin = styleNode->setBaseRenderBin( buildData->_renderBin );
                    buildData->_styleGroups.push_back( BuildData::StyleGroup( style.name(), styleNode ) );
                }
            
                styleNode->addVolumes( volumes );
                result = styleNodeAlreadyCreated ? 0L : styleNode;
                if ( out_createdNode ) *out_createdNode = volumes;
            }
        }

        return result;
    }

private:
    int _sourceId;
    int _renderBinStart;
    osg::ref_ptr<const Map> _map;
    double _extrusionDistance;
    
    osg::ref_ptr<const FeatureStencilModelOptions> _options;
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
        ScopedLock<Mutex> lock( _createMutex );
        FeatureStencilModelSource* obj = new FeatureStencilModelSource( options, _renderBinStart );
        _renderBinStart += MAX_NUM_STYLES*4;
        return obj;
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        FeatureStencilModelSourceFactory* nonConstThis = const_cast<FeatureStencilModelSourceFactory*>(this);
        return nonConstThis->create( static_cast<const PluginOptions*>(options) );
    }

protected:
    Mutex _createMutex;
    int _renderBinStart;
};

REGISTER_OSGPLUGIN(osgearth_model_feature_stencil, FeatureStencilModelSourceFactory)

