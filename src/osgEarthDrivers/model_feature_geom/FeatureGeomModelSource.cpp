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
#include <osgEarthFeatures/BuildTextOperator>

#include <osgEarthSymbology/Style>
#include <osgEarthSymbology/GeometrySymbol>
#include <osgEarthSymbology/GeometrySymbolizer>
#include <osgEarthSymbology/SymbolicNode>

#include <osg/Notify>
#include <osg/MatrixTransform>
#include <osgDB/FileNameUtils>
#include <OpenThreads/Mutex>
#include <OpenThreads/ScopedLock>

#include "FeatureGeomModelOptions"

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace osgEarth::Drivers;
using namespace OpenThreads;

#define PROP_HEIGHT_OFFSET "height_offset"

class FeatureGeomModelSource;

//------------------------------------------------------------------------

class FactoryGeomSymbolizer : public SymbolizerFactory
{
protected:
    osg::ref_ptr<FeatureModelSource> _modelSource;
    const FeatureGeomModelOptions _options;

public:
    FactoryGeomSymbolizer( FeatureModelSource* modelSource, const FeatureGeomModelOptions& options ) :
      _modelSource( modelSource ),
      _options( options ) { }

    //override
    virtual FeatureModelSource* getFeatureModelSource() { return _modelSource.get(); }

    //override
    virtual osg::Node* createNodeForStyle(
        const Symbology::Style* style,
        const FeatureList& features,
        FeatureSymbolizerContext* context,
        osg::Node** out_newNode)
    {
        // break the features out into separate lists for geometries and text annotations:
        FeatureList geomFeatureList, textAnnoList;

        for (FeatureList::const_iterator it = features.begin(); it != features.end(); ++it)
        {
            Feature* f = osg::clone((*it).get(),osg::CopyOp::DEEP_COPY_ALL);
            if ( dynamic_cast<TextAnnotation*>(f) )
                textAnnoList.push_back( f );
            else
                geomFeatureList.push_back( f );
        }

        // a single group to hold the results:
        osg::Group* root = new osg::Group;

        // compile the geometry features:
        if ( geomFeatureList.size() > 0 )
        {
            osg::Node* node = compileGeometries( geomFeatureList, style );
            if ( node ) root->addChild( node );
        }

        // compile the text annotation features:
        if ( textAnnoList.size() > 0 )
        {
            osg::Node* node = compileTextAnnotations( textAnnoList, style );
            if ( node ) root->addChild( node );
        }
        
        // Apply an LOD if required:
        if ( _options.minRange().isSet() || _options.maxRange().isSet() )
        {
            osg::LOD* lod = new osg::LOD();
            lod->addChild( root, _options.minRange().value(), _options.maxRange().value() );
            root = lod;
        }

        // set the output node if necessary:
        if ( out_newNode )
            *out_newNode = root;

        return root;
    }

    osg::Node*
    compileGeometries( FeatureList& features, const Style* style )
    {
        // A processing context to use with the filters:
        FilterContext cx;
        cx.profile() = _modelSource->getFeatureSource()->getFeatureProfile();

        // Transform them into the map's SRS:
        TransformFilter xform( _modelSource->getMap()->getProfile()->getSRS() );
        xform.setMakeGeocentric( _modelSource->getMap()->isGeocentric() );
        xform.setLocalizeCoordinates( true );

        // Apply the height offset if necessary:
        if ( _options.heightOffset().isSet() )
            xform.setHeightOffset( _options.heightOffset().value() );

        cx = xform.push( features, cx );

        // Build geometry:
        BuildGeometryFilter build;
        if ( _options.geometryTypeOverride().isSet() )
            build.geomTypeOverride() = *_options.geometryTypeOverride();

        osg::ref_ptr<osg::Node> result;
        build.setStyle( style );
        cx = build.push( features, result, cx );

        // Localize it.
        if ( cx.hasReferenceFrame() )
        {
            osg::MatrixTransform* delocalizer = new osg::MatrixTransform( cx.inverseReferenceFrame() );
            delocalizer->addChild( result );
            result = delocalizer;
        }

        return result.release();
    }

    osg::Node*
    compileTextAnnotations( FeatureList& features, const Style* style )
    {
        // A processing context to use with the filters:
        FilterContext contextFilter;
        contextFilter.profile() = _modelSource->getFeatureSource()->getFeatureProfile();

        // Transform them into the map's SRS:
        TransformFilter xform( _modelSource->getMap()->getProfile()->getSRS() );
        xform.setMakeGeocentric( _modelSource->getMap()->isGeocentric() );
        xform.setLocalizeCoordinates( true );

        // Apply the height offset if necessary:
        xform.setHeightOffset( _options.heightOffset().value() );
        contextFilter = xform.push( features, contextFilter );

        osg::ref_ptr<const TextSymbol> textSymbol = style->getSymbol<TextSymbol>();
        //Use a default symbol if we have no text symbol
        if (! textSymbol)
        {
            TextSymbol* ts = new TextSymbol();
            ts->rotateToScreen() = true;
            textSymbol = ts;
        }

        // build the text.
        BuildTextOperator textOperator;
        osg::Node* result = textOperator( features, textSymbol.get(), contextFilter );
        
        // install the localization transform if necessary.
        if ( contextFilter.hasReferenceFrame() )
        {
            osg::MatrixTransform* delocalizer = new osg::MatrixTransform( contextFilter.inverseReferenceFrame() );
            delocalizer->addChild( result );
            result = delocalizer;
        }

        return result;
    }
};

//------------------------------------------------------------------------

/** The model source implementation for feature_geom */
class FeatureGeomModelSource : public FeatureModelSource
{
public:
    FeatureGeomModelSource( const ModelSourceOptions& options )
        : FeatureModelSource( options ), _options( options )
    {
        //nop
    }

    const FeatureGeomModelOptions& getOptions() const { return _options; }

    //override
    void initialize( const std::string& referenceURI, const osgEarth::Map* map )
    {
        FeatureModelSource::initialize( referenceURI, map );
    }

    osg::Node* createNode( ProgressCallback* progress )
    {
        if ( _features.valid() && _features->getFeatureProfile() )
        {
            //OE_NOTICE << _options.getConfig().toString() << std::endl;
            return new FeatureSymbolizerGraph( new FactoryGeomSymbolizer(this, _options) );
        }
        else
        {
            return 0L;
        }
    }

private:
    const FeatureGeomModelOptions _options;
};

//------------------------------------------------------------------------

/** The plugin factory object */
class FeatureGeomModelSourceFactory : public ModelSourceDriver
{
public:
    FeatureGeomModelSourceFactory()
    {
        supportsExtension( "osgearth_model_feature_geom", "osgEarth feature geom plugin" );
    }

    virtual const char* className()
    {
        return "osgEarth Feature Geom Model Plugin";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return new FeatureGeomModelSource( getModelSourceOptions(options) );
    }
};

REGISTER_OSGPLUGIN(osgearth_model_feature_geom, FeatureGeomModelSourceFactory) 
