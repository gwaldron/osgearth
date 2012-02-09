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
#include <osgEarthFeatures/LabelSource>
#include <osgEarth/DepthOffset>
#include <osgEarthAnnotation/LabelNode>
#include <osgEarthAnnotation/Decluttering>
#include <osgDB/FileNameUtils>
#include <osgUtil/Optimizer>

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;

class AnnotationLabelSource : public LabelSource
{
public:
    AnnotationLabelSource( const LabelSourceOptions& options )
        : LabelSource( options )
    {
        //nop
    }

    /**
     * Creates a simple label. The caller is responsible for placing it in the scene.
     */
    osg::Node* createNode(
        const std::string& text,
        const Style&       style )
    {
        return 0L; // no support
    }

    /**
     * Creates a complete set of positioned label nodes from a feature list.
     */
    osg::Node* createNode(
        const FeatureList&   input,
        const Style&         style,
        const FilterContext& context )
    {
        const TextSymbol* text = style.get<TextSymbol>();
        if ( !text )
            return 0L;

        osg::Group* group = new osg::Group();
        Decluttering::setEnabled( group->getOrCreateStateSet(), true );
        if ( text->priority().isSet() )
        {
            DeclutteringOptions dco = Decluttering::getOptions();
            dco.sortByPriority() = text->priority().isSet();
            Decluttering::setOptions( dco );
        }    
        
        StringExpression  contentExpr ( *text->content() );
        NumericExpression priorityExpr( *text->priority() );

        if ( text->removeDuplicateLabels() == true )
        {
            // in remove-duplicates mode, make a list of unique features, selecting
            // the one with the largest area as the one we'll use for labeling.

            typedef std::pair<double, osg::ref_ptr<const Feature> > Entry;
            typedef std::map<std::string, Entry>                    EntryMap;

            EntryMap used;
    
            for( FeatureList::const_iterator i = input.begin(); i != input.end(); ++i )
            {
                Feature* feature = i->get();
                if ( feature && feature->getGeometry() )
                {
                    const std::string& value = feature->eval( contentExpr );
                    if ( !value.empty() )
                    {
                        double area = feature->getGeometry()->getBounds().area2d();
                        if ( used.find(value) == used.end() )
                        {
                            used[value] = Entry(area, feature);
                        }
                        else 
                        {
                            Entry& biggest = used[value];
                            if ( area > biggest.first )
                            {
                                biggest.first = area;
                                biggest.second = feature;
                            }
                        }
                    }
                }
            }

            for( EntryMap::iterator i = used.begin(); i != used.end(); ++i )
            {
                const std::string& value = i->first;
                const Feature* feature = i->second.second.get();
                group->addChild( makeLabelNode(context, feature, value, text, priorityExpr) );
            }
        }

        else
        {
            for( FeatureList::const_iterator i = input.begin(); i != input.end(); ++i )
            {
                const Feature* feature = i->get();
                if ( !feature )
                    continue;

                const Geometry* geom = feature->getGeometry();
                if ( !geom )
                    continue;

                const std::string& value = feature->eval( contentExpr, &context );
                if ( value.empty() )
                    continue;

                group->addChild( makeLabelNode(context, feature, value, text, priorityExpr) );
            }
        }

#if 0 // good idea but needs work.
        DepthOffsetGroup* dog = new DepthOffsetGroup();
        dog->setMinimumOffset( 500.0 );
        dog->addChild( group );
        return dog;
#endif
        return group;
    }

        
    osg::Node* makeLabelNode(const FilterContext& context, 
                             const Feature*       feature, 
                             const std::string&   value, 
                             const TextSymbol*    text, 
                             NumericExpression&   priorityExpr )
    {
        LabelNode* labelNode = new LabelNode(
            context.getSession()->getMapInfo().getProfile()->getSRS(),
            GeoPoint(feature->getSRS(), feature->getGeometry()->getBounds().center()),
            value,
            text );

        if ( text->priority().isSet() )
        {
            AnnotationData* data = new AnnotationData();
            data->setPriority( feature->eval(priorityExpr, &context) );
            labelNode->setAnnotationData( data );
        }

        return labelNode;
    }

};

//------------------------------------------------------------------------

class AnnotationLabelSourceDriver : public LabelSourceDriver
{
public:
    AnnotationLabelSourceDriver()
    {
        supportsExtension( "osgearth_label_annotation", "osgEarth annotation label plugin" );
    }

    virtual const char* className()
    {
        return "osgEarth Annotation Label Plugin";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return new AnnotationLabelSource( getLabelSourceOptions(options) );
    }
};

REGISTER_OSGPLUGIN(osgearth_label_annotation, AnnotationLabelSourceDriver)