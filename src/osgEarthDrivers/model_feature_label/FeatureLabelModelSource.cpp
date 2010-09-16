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

#include <osgEarth/ModelSource>
#include <osgEarth/Registry>
#include <osgEarth/Map>
#include <osgEarthFeatures/FeatureSymbolizer>
#include <osgEarthFeatures/FeatureModelSource>
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/TransformFilter>
#include <osgEarthFeatures/BuildGeometryFilter>
#include <osg/Notify>
#include <osg/MatrixTransform>
#include <osgDB/FileNameUtils>
#include <OpenThreads/Mutex>
#include <OpenThreads/ScopedLock>
#include <osgEarthSymbology/Style>
#include <osgEarthSymbology/GeometrySymbol>
#include <osgEarthSymbology/GeometrySymbolizer>
#include <osgEarthSymbology/SymbolicNode>
#include <osgEarthFeatures/BuildTextOperator>

#include "FeatureLabelModelOptions"

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace osgEarth::Drivers;
using namespace OpenThreads;

#define PROP_HEIGHT_OFFSET "height_offset"


class FactoryLabelSymbolizer : public SymbolizerFactory
{
protected:
    osg::ref_ptr<FeatureModelSource> _model;
    const FeatureLabelModelOptions _options;

public:
    FactoryLabelSymbolizer(FeatureModelSource* model, const FeatureLabelModelOptions& options) 
        : _model(model), _options(options) { }

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
        contextFilter.profile() = _model->getFeatureSource()->getFeatureProfile();

        // Transform them into the map's SRS:
        TransformFilter xform( _model->getMap()->getProfile()->getSRS() );
        xform.setMakeGeocentric( _model->getMap()->isGeocentric() );
        xform.setLocalizeCoordinates( true );

        //const FeatureLabelModelOptions* options = dynamic_cast<const FeatureLabelModelOptions*>(
        //    context->getModelSource()->getFeatureModelOptions());

        FeatureList featureList;
        for (FeatureList::const_iterator it = features.begin(); it != features.end(); ++it)
            featureList.push_back(osg::clone((*it).get(),osg::CopyOp::DEEP_COPY_ALL));

        xform.setHeightOffset( _options.heightOffset().value() );
        contextFilter = xform.push( featureList, contextFilter );        
        
        //Make some labels
        osg::ref_ptr<const TextSymbol> textSymbol = style->getSymbol<TextSymbol>();
        //Use a default symbol if we have no text symbol
        if (!textSymbol) textSymbol = new TextSymbol();
        osg::Node* labels = NULL;
        if (textSymbol.valid())
        {
            BuildTextOperator textOperator;
            labels = textOperator(featureList, textSymbol.get(), contextFilter);
        }

        osg::Node* result = labels;

        // If the context specifies a reference frame, apply it to the resulting model.
        // Q: should this be here, or should the reference frame matrix be passed to the Symbolizer?
        // ...probably the latter.
        if ( contextFilter.hasReferenceFrame() )
        {
            osg::MatrixTransform* delocalizer = new osg::MatrixTransform(
                contextFilter.inverseReferenceFrame() );
            delocalizer->addChild( labels );
            result = delocalizer;
        }

        // Apply an LOD if required:
        if ( _options.minRange().isSet() || _options.maxRange().isSet() )
        {
            osg::LOD* lod = new osg::LOD();
            lod->addChild( result, _options.minRange().value(), _options.maxRange().value() );
            result = lod;
        }

        // set the output node if necessary:
        if ( out_newNode )
            *out_newNode = result;

        return result;
    }
};

//------------------------------------------------------------------------

class FeatureLabelModelSource : public FeatureModelSource
{
public:
    FeatureLabelModelSource( const ModelSourceOptions& options ) : FeatureModelSource( options ),
        _options( options ) { }

    //override
    void initialize( const std::string& referenceURI, const osgEarth::Map* map )
    {
        FeatureModelSource::initialize( referenceURI, map );
    }

    osg::Node* createNode( ProgressCallback* progress )
    {
        return new FeatureSymbolizerGraph(new FactoryLabelSymbolizer(this, _options));
    }


protected:
    int _sourceId;
    ModelSourceOptions _options;
};

//------------------------------------------------------------------------

class FeatureLabelModelSourceFactory : public ModelSourceDriver
{
public:
    FeatureLabelModelSourceFactory()
    {
        supportsExtension( "osgearth_model_feature_label", "osgEarth feature label plugin" );
    }

    virtual const char* className()
    {
        return "osgEarth Feature Label Model Plugin";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return new FeatureLabelModelSource( getModelSourceOptions(options) );
    }
};

REGISTER_OSGPLUGIN(osgearth_model_feature_label, FeatureLabelModelSourceFactory) 
