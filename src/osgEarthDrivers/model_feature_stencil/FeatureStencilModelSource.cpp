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

#include <osgEarth/Registry>
#include <osgEarth/Map>
#include <osgEarthFeatures/FeatureModelSource>
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/AltitudeFilter>
#include <osgEarthFeatures/BufferFilter>
#include <osgEarthFeatures/TransformFilter>
#include <osgEarthFeatures/ResampleFilter>
#include <osgEarthFeatures/ConvertTypeFilter>
#include <osgEarthFeatures/ExtrudeGeometryFilter>
#include <osgEarthSymbology/StencilVolumeNode>
#include <osgEarthSymbology/Style>
#include <osg/Notify>
#include <osg/MatrixTransform>
#include <osg/ClusterCullingCallback>
#include <osg/Geode>
#include <osg/Projection>
#include <osgUtil/Tessellator>
#include <osg/MatrixTransform>
#include <osgDB/FileNameUtils>
#include <OpenThreads/Mutex>
#include <OpenThreads/ScopedLock>

#include "FeatureStencilModelOptions"

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace osgEarth::Drivers;
using namespace OpenThreads;

#define LC "[FeatureStencilModelSource] "

#define RENDER_BIN_START 100
#define MAX_NUM_STYLES   100
#define OFF_PROTECTED osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED

namespace
{
    /** Creates a full-screen quad to fill in the colors on the stencil volume. */
    osg::Node* createColorNode( const osg::Vec4f& color )
    {
        // make a full screen quad:
        osg::Geometry* quad = new osg::Geometry();
        quad->setUseVertexBufferObjects(true);

        osg::Vec3Array* verts = new osg::Vec3Array(4);
        (*verts)[0].set( 0, 1, 0 );
        (*verts)[1].set( 0, 0, 0 );
        (*verts)[2].set( 1, 0, 0 );
        (*verts)[3].set( 1, 1, 0 );
        quad->setVertexArray( verts );
        if ( verts->getVertexBufferObject() )
            verts->getVertexBufferObject()->setUsage(GL_STATIC_DRAW_ARB);

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

    struct BuildData // : public osg::Referenced
    {
        //BuildData() { }
        BuildData( int renderBinStart ) : _renderBin( renderBinStart ) { }

        typedef std::pair<std::string, osg::ref_ptr<StencilVolumeNode> > StyleGroup;
        int                       _renderBin;
        Threading::ReadWriteMutex _mutex;
        std::vector<StyleGroup>   _styleGroups;  // NOTE: DO NOT ACCESS without a mutex!


        bool getStyleNode( const std::string& styleName, StencilVolumeNode*& out_svn, bool useLock )
        {
            if ( useLock )
            {
                Threading::ScopedReadLock lock( _mutex );
                return getStyleNodeWithoutLocking( styleName, out_svn );
            }
            else
            {
                return getStyleNodeWithoutLocking( styleName, out_svn );
            }
        }

    private:
        bool getStyleNodeWithoutLocking( const std::string& styleName, StencilVolumeNode*& out_svn )
        {
            for(std::vector<StyleGroup>::iterator i = _styleGroups.begin(); i != _styleGroups.end(); ++i )
            {
                if( i->first == styleName )
                {
                    out_svn = i->second.get();
                    return true;
                }
            }
            return false;
        }
    };

    class StencilVolumeNodeFactory : public FeatureNodeFactory
    {
    protected:
        const FeatureStencilModelOptions _options;
        int                              _renderBinStart;
        BuildData                        _buildData;

    public:
        StencilVolumeNodeFactory( const FeatureStencilModelOptions& options, int renderBinStart )
            : _options(options),
              _buildData( renderBinStart )
        { }

        //override
        bool createOrUpdateNode(
            FeatureCursor*            cursor,
            const Style&              style,
            const FilterContext&      context,
            osg::ref_ptr<osg::Node>&  node )
        {
            const MapInfo& mi = context.getSession()->getMapInfo();
            
            // A processing context to use locally
            FilterContext cx = context;

            // make a working copy of the feature data.
            FeatureList featureList;
            cursor->fill( featureList );

            // establish the extrusion distance for the stencil volumes
            double extrusionDistance = 1;
            double densificationThreshold = 1.0;
            if ( _options.extrusionDistance().isSet() )
            {
                extrusionDistance = *_options.extrusionDistance();
            }
            else
            {
                if ( mi.isGeocentric() )
                    extrusionDistance = 300000.0; // meters geocentric
                else if ( mi.getProfile()->getSRS()->isGeographic() )
                    extrusionDistance = 5.0; // degrees-as-meters
                else
                    extrusionDistance = 12000.0; // meters
            }

            densificationThreshold = *_options.densificationThreshold();

            // Scan the geometry to see if it includes line data, since that will require buffering:
            bool hasLines = style.has<LineSymbol>() && !style.has<PolygonSymbol>();

            // If the geometry is lines, we need to buffer them before they will work with stenciling
            if ( hasLines )
            {
                const LineSymbol* line = style.getSymbol<LineSymbol>();
                if (line)
                {
                    BufferFilter buffer;
                    buffer.distance() = 0.5 * line->stroke()->width().value();
                    buffer.capStyle() = line->stroke()->lineCap().value();
                    cx = buffer.push( featureList, cx );
                }
            }

            // Extrude and cap the geometry in both directions to build a stencil volume:

            Style bs;
            bs.getOrCreate<AltitudeSymbol>()->verticalOffset() = -extrusionDistance;
            bs.getOrCreate<ExtrusionSymbol>()->height() = extrusionDistance * 2.0;
            
            AltitudeFilter alt;
            alt.setPropertiesFromStyle( bs );
            cx = alt.push( featureList, cx );

            ExtrudeGeometryFilter extrude;
            extrude.setStyle( bs );
            extrude.setMakeStencilVolume( true );
            osg::Node* volumes = extrude.push( featureList, cx );           

            if ( volumes )
            {
                // Apply an LOD if required:
                if ( _options.minRange().isSet() || _options.maxRange().isSet() )
                {
                    osg::LOD* lod = new osg::LOD();
                    lod->addChild( volumes, _options.minRange().value(), _options.maxRange().value() );
                    volumes = lod;
                }

                // Add the volumes to the appropriate style group.
                osg::Group* styleGroup = getOrCreateStyleGroup( style, cx.getSession() );
                StencilVolumeNode* svNode = dynamic_cast<StencilVolumeNode*>( styleGroup );
                if ( svNode )
                    svNode->addVolumes( volumes );
                else
                    styleGroup->addChild( volumes );
            }

            node = 0L; // always return null, since we added our geom to the style group.
            return volumes != 0L;
        }

        //override
        osg::Group* getOrCreateStyleGroup( const Style& style, Session* session )
        {
            if ( _options.showVolumes() == true )
            {
                return new osg::Group();
            }
            else
            {
                StencilVolumeNode* styleNode = 0L;
                if ( !_buildData.getStyleNode(style.getName(), styleNode, true) )
                {
                    // did not find; write-lock it and try again (double-check pattern)
                    Threading::ScopedWriteLock exclusiveLock( _buildData._mutex );

                    if ( !_buildData.getStyleNode(style.getName(), styleNode, false) )
                    {
                        OE_INFO << LC << "Create style group \"" << style.getName() << "\"" << std::endl;

                        styleNode = new StencilVolumeNode( *_options.mask(), *_options.inverted() );

                        if ( _options.mask() == false )
                        {
                            osg::Vec4f maskColor = osg::Vec4(1,1,0,1);

                            if (/*hasLines &&*/ style.getSymbol<LineSymbol>())
                            {
                                const LineSymbol* line = style.getSymbol<LineSymbol>();
                                maskColor = line->stroke()->color();
                            } 
                            else
                            {
                                const PolygonSymbol* poly = style.getSymbol<PolygonSymbol>();
                                if (poly)
                                    maskColor = poly->fill()->color();
                            }
                            styleNode->addChild( createColorNode(maskColor) );
                            
                            osg::StateSet* ss = styleNode->getOrCreateStateSet();

                            ss->setMode( GL_LIGHTING, _options.enableLighting() == true?
                                 osg::StateAttribute::ON | osg::StateAttribute::PROTECTED :
                                 osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );
                        }

                        _buildData._renderBin = styleNode->setBaseRenderBin( _buildData._renderBin );
                        _buildData._styleGroups.push_back( BuildData::StyleGroup(style.getName(), styleNode) );
                    }
                }

                return styleNode;
            }
        }
    };


    class FeatureStencilModelSource : public FeatureModelSource
    {
    public:
        FeatureStencilModelSource( const ModelSourceOptions& options, int renderBinStart ) :
            FeatureModelSource( options ),
            _options( options ),
            _renderBinStart( renderBinStart )
        {
            // make sure we have stencil bits. Note, this only works before
            // a viewer gets created. You may need to allocate stencil bits
            // yourself if you make this object after realizing a viewer.
            if ( osg::DisplaySettings::instance()->getMinimumNumStencilBits() < 8 )
            {
                osg::DisplaySettings::instance()->setMinimumNumStencilBits( 8 );
            }
        }
        
    public: // FeatureModelSource

        virtual const FeatureModelSourceOptions& getFeatureModelOptions() const
        {
            return _options;
        }

        void initialize( const osgDB::Options* dbOptions )
        {
            FeatureModelSource::initialize( dbOptions );
        }

        FeatureNodeFactory* createFeatureNodeFactory()
        {
            return new StencilVolumeNodeFactory( _options, _renderBinStart );
        }

    protected:
        int _renderBinStart;
        const FeatureStencilModelOptions _options;
    };
}


class FeatureStencilModelSourceDriver : public ModelSourceDriver
{
public:
    FeatureStencilModelSourceDriver() :
        _renderBinStart( RENDER_BIN_START )
    {
        supportsExtension( "osgearth_model_feature_stencil", "osgEarth feature stencil plugin" );
    }

    virtual const char* className()
    {
        return "osgEarth Feature Stencil Model Plugin";
    }

    FeatureStencilModelSource* create( const Options* options )
    {
        ScopedLock<Mutex> lock( _createMutex );

        FeatureStencilModelSource* obj = new FeatureStencilModelSource( 
            getModelSourceOptions(options), 
            _renderBinStart );

        _renderBinStart += MAX_NUM_STYLES*4;

        return obj;
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        FeatureStencilModelSourceDriver* nonConstThis = const_cast<FeatureStencilModelSourceDriver*>(this);
        return nonConstThis->create( options );
    }

protected:
    Mutex _createMutex;
    int _renderBinStart;
};

REGISTER_OSGPLUGIN(osgearth_model_feature_stencil, FeatureStencilModelSourceDriver)
