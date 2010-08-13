/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osgEarthSymbology/Style>
#include <osgEarthFeatures/FeatureModelSource>
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/TransformFilter>
#include <osgEarthFeatures/FeatureSymbolizer>
#include <osgEarthFeatures/BuildGeometryFilter>
#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgSim/OverlayNode>

#include "FeatureOverlayModelOptions"

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace osgEarth::Drivers;


class FactoryLayerSymbolizer : public SymbolizerFactory
{
protected:
    osg::ref_ptr<FeatureModelSource> _model;

public:
    FactoryLayerSymbolizer(FeatureModelSource* model) : _model(model) {}
    FeatureModelSource* getFeatureModelSource() { return _model.get(); }
    //override

    virtual osg::Node* createNodeForStyle(
        const Symbology::Style* style,
        const FeatureList& features,
        FeatureSymbolizerContext* context,
        osg::Node** out_newNode)
    {
        // A processing context to use with the filters:
        FilterContext contextFilter;
        contextFilter.profile() = context->getModelSource()->getFeatureSource()->getFeatureProfile();
        const FeatureOverlayModelOptions* options = dynamic_cast<const FeatureOverlayModelOptions*>(context->getModelSource()->getFeatureModelOptions());

        FeatureList featureList;
        for (FeatureList::const_iterator it = features.begin(); it != features.end(); ++it)
            featureList.push_back(osg::clone((*it).get(),osg::CopyOp::DEEP_COPY_ALL));

        // Transform them into the map's SRS:
        TransformFilter xform( context->getModelSource()->getMap()->getProfile()->getSRS() );
        xform.setMakeGeocentric( context->getModelSource()->getMap()->isGeocentric() );
        xform.setLocalizeCoordinates( true );

        contextFilter = xform.push( featureList, contextFilter );

        // Build geometry:
        BuildGeometryFilter build;    
        if ( options->geometryTypeOverride().isSet() )
            build.geomTypeOverride() = options->geometryTypeOverride().value();

        // apply the style rule if we have one:
        osg::ref_ptr<osg::Node> result;
        build.setStyle(style);
        contextFilter = build.push( featureList, result, contextFilter );

        if ( out_newNode ) *out_newNode = result.get();
        return result.release();
    }
};


class FeatureOverlayModelSource : public FeatureModelSource
{
public:
    FeatureOverlayModelSource( const PluginOptions* options ) : FeatureModelSource( options )
    {
        _options = dynamic_cast<const FeatureOverlayModelOptions*>( options );
        if ( !_options )
            _options = new FeatureOverlayModelOptions( options );
    }

    void initialize( const std::string& referenceURI, const osgEarth::Map* map )
    {
        FeatureModelSource::initialize( referenceURI, map );
    }

#if 0
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
        if ( _options->geometryTypeOverride().isSet() )
            build.geomTypeOverride() = _options->geometryTypeOverride().value();

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
        overlayNode->setOverlayTechnique( _options->overlayTechnique().value() );
        overlayNode->setOverlayBaseHeight( _options->baseHeight().value() );
        overlayNode->setOverlayTextureSizeHint( _options->textureSize().value() );
        overlayNode->setOverlayTextureUnit( _options->textureUnit().value() );
        overlayNode->setContinuousUpdate( false );
        overlayNode->setOverlaySubgraph( node );

        return overlayNode;
    }
#endif

    osg::Node* createNode( ProgressCallback* progress )
    {
        osg::Node* node = new FeatureSymbolizerGraph(new FactoryLayerSymbolizer(this));
        const FeatureOverlayModelOptions* options = dynamic_cast<const FeatureOverlayModelOptions*>( _options.get() );
        
        // build an overlay node around the geometry
        osgSim::OverlayNode* overlayNode = new osgSim::OverlayNode();
        overlayNode->setName( this->getName() );
        overlayNode->setOverlayTechnique( options->overlayTechnique().value() );
        overlayNode->setOverlayBaseHeight( options->baseHeight().value() );
        overlayNode->setOverlayTextureSizeHint( options->textureSize().value() );
        overlayNode->setOverlayTextureUnit( options->textureUnit().value() );
        overlayNode->setContinuousUpdate( false );
        overlayNode->setOverlaySubgraph( node );
        return overlayNode;
    }

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
