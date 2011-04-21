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
#include <osgEarthFeatures/FeatureModelSource>
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/TransformFilter>
#include <osgEarthFeatures/BuildGeometryFilter>
#include <osg/Notify>
#include <osg/MatrixTransform>
#include <osgDB/FileNameUtils>
#include <osgEarthSymbology/Style>
#include <osgEarthFeatures/BuildTextOperator>

#include "FeatureLabelModelOptions"

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace osgEarth::Drivers;

namespace
{
    class LabelNodeFactory : public FeatureNodeFactory
    {
    protected:
        const FeatureLabelModelOptions _options;

    public:
        LabelNodeFactory(const FeatureLabelModelOptions& options) 
            : _options(options) { }

        //override
        bool createOrUpdateNode(
            FeatureCursor*            cursor,
            const Style&              style,
            const FilterContext&      context,
            osg::ref_ptr<osg::Node>&  node )
        {
            const MapInfo& mi = context.getSession()->getMapInfo();

            // A processing context to use with the filters:
            FilterContext cx = context;

            // Make a working copy of the features:
            FeatureList featureList;
            cursor->fill( featureList );
            //for (FeatureList::const_iterator it = features.begin(); it != features.end(); ++it)
            //    featureList.push_back(osg::clone((*it).get(),osg::CopyOp::DEEP_COPY_ALL));

            // Transform them into the map's SRS:
            TransformFilter xform( mi.getProfile()->getSRS(), mi.isGeocentric() );
            xform.setLocalizeCoordinates( true );
            xform.setMatrix( osg::Matrixd::translate( 0, 0, *_options.heightOffset() ) );
            //xform.setHeightOffset( *_options.heightOffset() );
            cx = xform.push( featureList, cx );        
            
            // Make some labels
            osg::ref_ptr<const TextSymbol> textSymbol = style.getSymbol<TextSymbol>();
            //Use a default symbol if we have no text symbol
            if (!textSymbol) textSymbol = new TextSymbol();
            osg::Node* labels = NULL;
            if (textSymbol.valid())
            {
                BuildTextOperator textOperator;
                labels = textOperator(featureList, textSymbol.get(), cx);
            }

            osg::Node* result = labels;

            // If the context specifies a reference frame, apply it to the resulting model.
            if ( cx.hasReferenceFrame() )
            {
                osg::MatrixTransform* delocalizer = new osg::MatrixTransform( cx.inverseReferenceFrame() );
                delocalizer->addChild( labels );
                result = delocalizer;
            }

            // Apply an LOD if required:
            if ( _options.minRange().isSet() || _options.maxRange().isSet() )
            {
                osg::LOD* lod = new osg::LOD();
                lod->addChild( result, *_options.minRange(), *_options.maxRange() );
                result = lod;
            }

            node = result;
            return true;
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

        //override
        FeatureNodeFactory* createFeatureNodeFactory()
        {
            return new LabelNodeFactory( _options );
        }

    protected:
        int _sourceId;
        ModelSourceOptions _options;
    };
}

//------------------------------------------------------------------------

class FeatureLabelModelSourceDriver : public ModelSourceDriver
{
public:
    FeatureLabelModelSourceDriver()
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

REGISTER_OSGPLUGIN(osgearth_model_feature_label, FeatureLabelModelSourceDriver) 
