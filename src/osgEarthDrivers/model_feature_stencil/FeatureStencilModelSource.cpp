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
#include <osgUtil/Tessellator>
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



//#define USE_SYMBOLOGY
#ifdef USE_SYMBOLOGY
#include <osgEarthSymbology/Style>
#include <osgEarthSymbology/GeometrySymbol>
#include <osgEarthSymbology/StencilVolumeNode>
#include <osgEarthSymbology/GeometryInput>
#include <osgEarthSymbology/SymbolicNode>


static
void tessellate( osg::Geometry* geom )
{
    osgUtil::Tessellator tess;
    tess.setTessellationType( osgUtil::Tessellator::TESS_TYPE_GEOMETRY );
    tess.setWindingType( osgUtil::Tessellator::TESS_WINDING_ODD );
//    tess.setWindingType( osgUtil::Tessellator::TESS_WINDING_POSITIVE );
    tess.retessellatePolygons( *geom );
}

osg::Geode*
createVolume(osgEarth::Symbology::Geometry* geom,
             double               offset,
             double               height,
             const FilterContext& context )
{
    if ( !geom ) return 0L;

    int numRings = 0;

    // start by offsetting the input data and counting the number of rings
    {
        osgEarth::Symbology::GeometryIterator i( geom );
        i.traverseMultiGeometry() = true;
        i.traversePolygonHoles() = true;
        while( i.hasMore() )
        {
            osgEarth::Symbology::Geometry* part = i.next();

            if (offset != 0.0)
            {
                for( osg::Vec3dArray::iterator j = part->begin(); j != part->end(); j++ )
                {
                    if ( context.isGeocentric() )
                    {
                        osg::Vec3d world = context.toWorld( *j );
                        // TODO: get the proper up vector; this is spherical.. or does it really matter for
                        // stencil volumes?
                        osg::Vec3d offset_vec = world;
                        offset_vec.normalize();
                        *j = context.toLocal( world + offset_vec * offset ); //(*j) += offset_vec * offset;
                    }
                    else
                    {
                        (*j).z() += offset;
                    }
                }
            }

            // in the meantime, count the # of closed geoms. We will need to know this in 
            // order to pre-allocate the proper # of verts.
            if ( part->getType() == osgEarth::Symbology::Geometry::TYPE_POLYGON || part->getType() == osgEarth::Symbology::Geometry::TYPE_RING )
            {
                numRings++;
            }
        }
    }

    // now, go thru and remove any coplanar segments from the geometry. The tesselator will
    // not work include a vert connecting two colinear segments in the tesselation, and this
    // will break the stenciling logic.
#define PARALLEL_EPSILON 0.01
    osgEarth::Symbology::GeometryIterator i( geom );
    i.traverseMultiGeometry() = true;
    i.traversePolygonHoles() = true;
    while( i.hasMore() )
    {
        osgEarth::Symbology::Geometry* part = i.next();
        if ( part->size() >= 3 )
        {
            osg::Vec3d prevVec = part->front() - part->back();
            prevVec.normalize();

            for( osg::Vec3dArray::iterator j = part->begin(); part->size() >= 3 && j != part->end(); )
            {
                osg::Vec3d& p0 = *j;
                osg::Vec3d& p1 = j+1 != part->end() ? *(j+1) : part->front();
                osg::Vec3d vec = p1-p0; vec.normalize();

                // if the vectors are essentially parallel, remove the extraneous vertex.
                if ( (prevVec ^ vec).length() < PARALLEL_EPSILON )
                {
                    j = part->erase( j );
                    //OE_NOTICE << "removed colinear segment" << std::endl;
                }
                else
                {
                    ++j;
                    prevVec = vec;
                }
            }
        }
    }


    bool made_geom = true;
    const SpatialReference* srs = context.profile()->getSRS();

    // total up all the points so we can pre-allocate the vertex arrays.
    int num_cap_verts = geom->getTotalPointCount();
    int num_wall_verts = 2 * (num_cap_verts + numRings); // add in numRings b/c we need to close each wall

    osg::Geometry* walls = new osg::Geometry();
    osg::Vec3Array* verts = new osg::Vec3Array( num_wall_verts );
    walls->setVertexArray( verts );

    osg::Geometry* top_cap = new osg::Geometry();
    osg::Vec3Array* top_verts = new osg::Vec3Array( num_cap_verts );
    top_cap->setVertexArray( top_verts );

    osg::Geometry* bottom_cap = new osg::Geometry();
    osg::Vec3Array* bottom_verts = new osg::Vec3Array( num_cap_verts );
    bottom_cap->setVertexArray( bottom_verts );

    int wall_vert_ptr = 0;
    int top_vert_ptr = 0;
    int bottom_vert_ptr = 0;

    //double target_len = height;

    // now generate the extruded geometry.
    osgEarth::Symbology::GeometryIterator k( geom );
    while( k.hasMore() )
    {
        osgEarth::Symbology::Geometry* part = k.next();

        unsigned int wall_part_ptr = wall_vert_ptr;
        unsigned int top_part_ptr = top_vert_ptr;
        unsigned int bottom_part_ptr = bottom_vert_ptr;
        double part_len = 0.0;

        GLenum prim_type = part->getType() == osgEarth::Symbology::Geometry::TYPE_POINTSET ? GL_LINES : GL_TRIANGLE_STRIP;

        for( osg::Vec3dArray::const_iterator m = part->begin(); m != part->end(); ++m )
        {
            osg::Vec3d extrude_vec;

            if ( srs )
            {
                osg::Vec3d m_world = context.toWorld( *m ); //*m * context.inverseReferenceFrame();
                if ( context.isGeocentric() )
                {
                    osg::Vec3d p_vec = m_world; // todo: not exactly right; spherical

                    osg::Vec3d unit_vec = p_vec; 
                    unit_vec.normalize();
                    p_vec = p_vec + unit_vec*height;

                    extrude_vec = context.toLocal( p_vec ); //p_vec * context.referenceFrame();
                }
                else
                {
                    extrude_vec.set( m_world.x(), m_world.y(), height );
                    extrude_vec = context.toLocal( extrude_vec ); //extrude_vec * context.referenceFrame();
                }
            }
            else
            {
                extrude_vec.set( m->x(), m->y(), height );
            }

            (*top_verts)[top_vert_ptr++] = extrude_vec;
            (*bottom_verts)[bottom_vert_ptr++] = *m;
             
            part_len += wall_vert_ptr > wall_part_ptr?
                (extrude_vec - (*verts)[wall_vert_ptr-2]).length() :
                0.0;

            int p;

            p = wall_vert_ptr++;
            (*verts)[p] = extrude_vec;

            p = wall_vert_ptr++;
            (*verts)[p] = *m;
        }

        // close the wall if it's a ring/poly:
        if ( part->getType() == osgEarth::Symbology::Geometry::TYPE_RING || part->getType() == osgEarth::Symbology::Geometry::TYPE_POLYGON )
        {
            part_len += wall_vert_ptr > wall_part_ptr?
                ((*verts)[wall_part_ptr] - (*verts)[wall_vert_ptr-2]).length() :
                0.0;

            int p;

            p = wall_vert_ptr++;
            (*verts)[p] = (*verts)[wall_part_ptr];

            p = wall_vert_ptr++;
            (*verts)[p] = (*verts)[wall_part_ptr+1];
        }

        walls->addPrimitiveSet( new osg::DrawArrays(
            prim_type,
            wall_part_ptr, wall_vert_ptr - wall_part_ptr ) );

        top_cap->addPrimitiveSet( new osg::DrawArrays(
            osg::PrimitiveSet::LINE_LOOP,
            top_part_ptr, top_vert_ptr - top_part_ptr ) );

        // reverse the bottom verts so the front face is down:
        std::reverse( bottom_verts->begin()+bottom_part_ptr, bottom_verts->begin()+bottom_vert_ptr );

        bottom_cap->addPrimitiveSet( new osg::DrawArrays(
            osg::PrimitiveSet::LINE_LOOP,
            bottom_part_ptr, bottom_vert_ptr - bottom_part_ptr ) );
    }

    // build solid surfaces for the caps:
    tessellate( top_cap );
    tessellate( bottom_cap );

    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( walls );
    geode->addDrawable( top_cap );
    geode->addDrawable( bottom_cap );

    return geode;
}


    // implementation-specific data to pass to buildNodeForStyle:
    struct BuildData : public osg::Referenced
    {
        BuildData( int renderBinStart ) : _renderBin( renderBinStart ) { }
        int _renderBin;

        // pairs a style name with a starting render bin.
        typedef std::pair<std::string,osg::ref_ptr<osgEarth::Symbology::StencilVolumeNode> > StyleGroup;
        std::vector<StyleGroup> _styleGroups;

        bool getStyleNode( const std::string& styleName, osgEarth::Symbology::StencilVolumeNode*& out_svn ) {
            for(std::vector<StyleGroup>::iterator i = _styleGroups.begin(); i != _styleGroups.end(); ++i ) {
                if( i->first == styleName ) {
                    out_svn = i->second.get();
                    return true;
                }
            }
            return false;
        }
    };

class VolumeSymbol : public osgEarth::Symbology::Symbol
{
public:

    VolumeSymbol() :_extrusionDistance( 300000 ),
                    _densificationThreshold( 1000000 ){}

    optional<double>& densificationThreshold() { return _densificationThreshold; }
    const optional<double>& densificationThreshold() const { return _densificationThreshold; }

    optional<double>& extrusionDistance() { return _extrusionDistance; }
    const optional<double>& extrusionDistance() const { return _extrusionDistance; }

    optional<float>& minRange() { return _minRange; }
    const optional<float>& minRange() const { return _minRange; }

    optional<float>& maxRange() { return _maxRange; }
    const optional<float>& maxRange() const { return _maxRange; }

    optional<bool>& showVolumes() { return _showVolume; }
    const optional<bool>& showVolumes() const { return _showVolume; }

    optional<bool>& mask() { return _mask; }
    const optional<bool>& mask() const { return _mask; }

    optional<bool>& inverted() { return _inverted; }
    const optional<bool>& inverted() const { return _inverted; }
    
protected:
    optional<double> _extrusionDistance;
    optional<double> _offset;
    optional<double> _densificationThreshold;
    optional<bool> _inverted;
    optional<bool> _mask;
    optional<bool> _showVolume;
    optional<float> _minRange, _maxRange;
};


class VolumeSymbolizerContext : public osgEarth::Symbology::SymbolizerContext
{
public:
    VolumeSymbolizerContext(const Map* map, const FeatureProfile* p) : _map(map), _featureProfile(p) {}
    const FeatureProfile* getFeatureProfile() const { return _featureProfile.get(); }
    const Map* getMap() const { return _map.get(); }
protected:
    osg::ref_ptr<const FeatureProfile> _featureProfile;
    osg::ref_ptr<const Map> _map;
};


class FeatureInput : public osgEarth::Symbology::SymbolizerInput
{
public:
    FeatureInput(const FeatureList& f) : _features(f) {}
    FeatureList& getFeatures() { return _features; }
    const FeatureList& getFeatures() const { return _features; }
protected:
    FeatureList _features;
};

class VolumeSymbolizer : public osgEarth::Symbology::Symbolizer
{
public:
    VolumeSymbolizer(BuildData* bd) : _buildData(bd) {}

    bool update(
        osgEarth::Symbology::SymbolizerInput* dataInput,
        const osgEarth::Symbology::Style* style,
        osg::Group* attachPoint,
        osgEarth::Symbology::SymbolizerContext* symbolizerContext )
    {

        if (!dataInput || !symbolizerContext)
            return false;

        VolumeSymbolizerContext* volumeContext = dynamic_cast<VolumeSymbolizerContext*>(symbolizerContext);


        FeatureInput* featureInput = dynamic_cast<FeatureInput*>(dataInput);
        if (!featureInput)
            return false;


        double extrusionDistance = 1;
        double densificationThreshold = 0.0;
        const VolumeSymbol* vsym = style->getSymbol<VolumeSymbol>();
        if (vsym)
        {
            densificationThreshold = vsym->densificationThreshold().value();
            extrusionDistance = vsym->extrusionDistance().value();
        }

        // Scan the geometry to see if it includes line data, since that will require 
        // buffering:
        FeatureList& featureList = featureInput->getFeatures();
        bool hasLines = false;
        for( FeatureList::const_iterator i = featureList.begin(); i != featureList.end(); ++i )
        {
            Feature* feature = *i;
            // later should be a osgEarth::Symbology::Geometry
            Geometry* geom = feature->getGeometry();
            if ( geom && geom->getComponentType() == Geometry::TYPE_LINESTRING )
            {
                hasLines = true;
                break;
            }
        }

        bool isGeocentric = volumeContext->getMap()->isGeocentric();

        // A processing context to use with the filters:
        FilterContext context;
        context.profile() = volumeContext->getFeatureProfile();

        // If the geometry is lines, we need to buffer them before they will work with stenciling
        if ( hasLines )
        {
            const osgEarth::Symbology::LineSymbol* line = style->getSymbol<osgEarth::Symbology::LineSymbol>();
            if (line) {
                BufferFilter buffer;
                buffer.distance() = 0.5 * line->stroke()->width().value();
                // should be natif from Symbology
                buffer.capStyle() = (osgEarth::Features::Stroke::LineCapStyle)line->stroke()->lineCap().value();
                context = buffer.push( featureList, context );
            }
        }

        // Transform them into the map's SRS, localizing the verts along the way:
        TransformFilter xform( volumeContext->getMap()->getProfile()->getSRS(), isGeocentric );
        context = xform.push( featureList, context );

        if ( isGeocentric )
        {
            // We need to make sure that on a round globe, the points are sampled such that
            // long segments follow the curvature of the earth. By the way, if a Buffer was
            // applied, that will also remove colinear segment points. Resample the points to 
            // achieve a usable tesselation.
            ResampleFilter resample;
            resample.maxLength() = densificationThreshold;
            resample.minLength() = 0.0;
            resample.perturbationThreshold() = 0.1;
            context = resample.push( featureList, context );
        }

        // Extrude and cap the geometry in both directions to build a stencil volume:
        osg::Group* volumes = 0L;

        for( FeatureList::iterator i = featureList.begin(); i != featureList.end(); ++i )
        {
            Feature* feature = *i;
            // later will be a native osgEarth::Symbology::Geometry
            Geometry* geom = feature->getGeometry();
            osg::ref_ptr<osgEarth::Symbology::Geometry> symGeom = osgEarth::Symbology::Geometry::create((osgEarth::Symbology::Geometry::Type)(geom->getType()), geom);
            osg::Node* volume = createVolume(
                symGeom,
                -extrusionDistance,
                extrusionDistance * 2.0,
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
            const VolumeSymbol* vsym = style->getSymbol<VolumeSymbol>();
            if (vsym)
            {
                if ( vsym->minRange().isSet() || vsym->maxRange().isSet() )
                {
                    osg::LOD* lod = new osg::LOD();
                    lod->addChild( volumes, vsym->minRange().value(), vsym->maxRange().value() );
                    volumes = lod;
                }

                osgEarth::Symbology::StencilVolumeNode* styleNode = 0L;
                bool styleNodeAlreadyCreated = _buildData->getStyleNode(style->getName(), styleNode);
                if ( vsym->showVolumes() == true )
                {
                    result = volumes;
                }
                else
                {
                    if ( !styleNodeAlreadyCreated )
                    {
                        OE_NOTICE << "Creating new style group for '" << style->getName() << "'" << std::endl;
                        styleNode = new osgEarth::Symbology::StencilVolumeNode( vsym->mask().value(), vsym->inverted().value() );
                        if ( vsym->mask() == false )
                        {
                            osg::Vec4f maskColor = osg::Vec4(1,1,0,1); //style.getColor( hasLines ? osgEarth::Symbology::Geometry::TYPE_LINESTRING : osgEarth::Symbology::Geometry::TYPE_POLYGON );
                            styleNode->addChild( createColorNode(maskColor) );
                        }
                        _buildData->_renderBin = styleNode->setBaseRenderBin( _buildData->_renderBin );
                        _buildData->_styleGroups.push_back( BuildData::StyleGroup( style->getName(), styleNode ) );
                    }
            
                    styleNode->addVolumes( volumes );
                    result = styleNodeAlreadyCreated ? 0L : styleNode;
                }

                attachPoint->removeChildren(0, attachPoint->getNumChildren());
                if (result)
                    attachPoint->addChild(result);
            }
        }

        return (attachPoint->getNumChildren() > 0 );
    }

    osg::ref_ptr<BuildData> _buildData;
};


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

    //override
    osg::Referenced* createBuildData()
    {
        return new BuildData( _renderBinStart );
    }
    
    //override
    osg::Node* renderFeaturesForStyle(const Style& style, FeatureList& features, osg::Referenced* data, osg::Node** out_createdNode )
    {
        BuildData* buildData = static_cast<BuildData*>(data);

        FeatureInput* input = new FeatureInput(features);
        VolumeSymbolizer* symbolizer = new VolumeSymbolizer(buildData);
        VolumeSymbolizerContext* ctx = new VolumeSymbolizerContext(_map.get(), getFeatureSource()->getFeatureProfile());
        osgEarth::Symbology::SymbolicNode* symb = new osgEarth::Symbology::SymbolicNode;
        osgEarth::Symbology::Style* vstyle = new osgEarth::Symbology::Style;
        vstyle->setName(style.name());
        vstyle->addSymbol(new osgEarth::Symbology::LineSymbol);
        VolumeSymbol* symbol = new VolumeSymbol;
        vstyle->addSymbol(symbol);

        symb->setDataSet(input);
        symb->setContext(ctx);
        symb->setSymbolizer(symbolizer);
        symb->setStyle(vstyle);
        // if (out_createdNode)
        //     *out_createdNode = symb;
        return symb;
    }

protected:
    int _sourceId;
    int _renderBinStart;
    osg::ref_ptr<const Map> _map;
    double _extrusionDistance;
    
    osg::ref_ptr<const FeatureStencilModelOptions> _options;
};

#else

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
                    OE_NOTICE << "Creating new style group for '" << style.name() << "'" << std::endl;
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

#endif

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

