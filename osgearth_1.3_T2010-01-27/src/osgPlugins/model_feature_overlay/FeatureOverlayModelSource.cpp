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
#include <osgEarthFeatures/Styling>
#include <osgEarthFeatures/FeatureModelSource>
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/TransformFilter>
#include <osgEarthFeatures/BuildGeometryFilter>
#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgSim/OverlayNode>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace OpenThreads;

#define PROP_TEXTURE_UNIT  "texture_unit"
#define PROP_TEXTURE_SIZE  "texture_size"
#define PROP_OVERLAY_TECH  "overlay_technique"
#define PROP_BASE_HEIGHT   "base_height"

#define VAL_OT_ODWOO      "object_dependent_with_orthographic_overlay"
#define VAL_OT_VDWOO      "view_dependent_with_orthographic_overlay"
#define VAL_OT_VDWPO      "view_dependent_with_perspective_overlay"

class FeatureOverlayModelSource : public FeatureModelSource
{
public:
    FeatureOverlayModelSource( const PluginOptions* options ) :
        FeatureModelSource( options ),
        _textureUnit( 0 ),
        _textureSize( 1024 ),
        _baseHeight( 0.0 ),
        _overlayTech( osgSim::OverlayNode::VIEW_DEPENDENT_WITH_PERSPECTIVE_OVERLAY )
    {
        const Config& conf = getOptions()->config();

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
    }

    void initialize( const std::string& referenceURI, const osgEarth::Map* map )
    {
        FeatureModelSource::initialize( referenceURI, map );

        _mapSRS = map->getProfile()->getSRS();
        _mapIsGeocentric = map->isGeocentric();
    }

    //override
    osg::Node* renderFeaturesForStyle( const Style& style, FeatureList& features, osg::Referenced* data, osg::Node** out_newNode )
    {
        // A processing context to use with the filters:
        FilterContext context;
        context.profile() = getFeatureSource()->getFeatureProfile();

        // Transform them into the map's SRS:
        TransformFilter xform( _mapSRS.get(), _mapIsGeocentric );
        context = xform.push( features, context );

        // Build geometry:
        BuildGeometryFilter build;    
        if ( _geomTypeOverride.isSet() )
            build.geomTypeOverride() = _geomTypeOverride.get();

        // apply the style rule if we have one:
        osg::ref_ptr<osg::Node> result;
        build.style() = style;
        context = build.push( features, result, context );

        if ( out_newNode ) *out_newNode = result.get();
        return result.release();
    }

    //override
    osg::Node* createNode( ProgressCallback* progress =0L )
    {
        osg::Node* node = FeatureModelSource::createNode( progress );

        // build an overlay node around the geometry
        osgSim::OverlayNode* overlayNode = new osgSim::OverlayNode();
        overlayNode->setName( this->getName() );
        overlayNode->setOverlayTechnique( _overlayTech );
        overlayNode->setOverlayBaseHeight( _baseHeight );
        overlayNode->setOverlayTextureSizeHint( _textureSize );
        overlayNode->setOverlayTextureUnit( _textureUnit );
        overlayNode->setContinuousUpdate( false );
        overlayNode->setOverlaySubgraph( node );

        return overlayNode;
    }

private:
    int _textureSize;
    int _textureUnit;
    double _baseHeight;
    osgSim::OverlayNode::OverlayTechnique _overlayTech;
    osg::ref_ptr<const SpatialReference> _mapSRS;
    bool _mapIsGeocentric;
};


class FeatureOverlayModelSourceFactory : public osgDB::ReaderWriter
{
public:
    FeatureOverlayModelSourceFactory()
    {
        supportsExtension( "osgearth_model_feature_overlay", "osgEarth feature overlay plugin" );
    }

    virtual const char* className()
    {
        return "osgEarth Feature Overlay Model Plugin";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return ReadResult( new FeatureOverlayModelSource( dynamic_cast<const PluginOptions*>(options) ) );
    }
};

REGISTER_OSGPLUGIN(osgearth_model_feature_overlay, FeatureOverlayModelSourceFactory) 
