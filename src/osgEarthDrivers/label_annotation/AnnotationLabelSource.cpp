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
#include <osgEarthFeatures/LabelSource>
#include <osgEarthFeatures/FeatureSourceIndexNode>
#include <osgEarthAnnotation/LabelNode>
#include <osgEarthAnnotation/PlaceNode>
#include <osgEarth/DepthOffset>
#include <osgDB/FileNameUtils>
#include <osgUtil/Optimizer>

#define LC "[AnnoLabelSource] "

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
     * Creates a complete set of positioned label nodes from a feature list.
     */
    osg::Node* createNode(
        const FeatureList&   input,
        const Style&         style,
        const FilterContext& context )
    {
        if ( style.get<TextSymbol>() == 0L && style.get<IconSymbol>() == 0L )
            return 0L;

        // copy the style so we can (potentially) modify the text symbol.
        Style styleCopy = style;
        TextSymbol* text = styleCopy.get<TextSymbol>();
        IconSymbol* icon = styleCopy.get<IconSymbol>();

        osg::Group* group = new osg::Group();
        
        StringExpression  textContentExpr ( text ? *text->content()  : StringExpression() );
        NumericExpression textPriorityExpr( text ? *text->priority() : NumericExpression() );
        StringExpression  iconUrlExpr     ( icon ? *icon->url()      : StringExpression() );

#if 0        
        if ( text && text->removeDuplicateLabels() == true )
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
                    const std::string& value = feature->eval( textContentExpr, &context );
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

                group->addChild( makePlaceNode(
                    context,
                    feature,
                    styleCopy,
                    textPriorityExpr) );

                //if ( text )
                //    group->addChild( makeLabelNode(context, feature, value, styleCopy, textPriorityExpr) );
                //if ( icon )
                //    group->addChild( makeIconNode(context, feature, value, styleCopy) );
            }
        }

        else
#endif
        {
            for( FeatureList::const_iterator i = input.begin(); i != input.end(); ++i )
            {
                const Feature* feature = i->get();
                if ( !feature )
                    continue;

                const Geometry* geom = feature->getGeometry();
                if ( !geom )
                    continue;

                Style tempStyle = styleCopy;

                if ( text )
                {
                    std::string labelText = feature->eval( textContentExpr, &context );
                    tempStyle.get<TextSymbol>()->content()->setLiteral( labelText );
                }
                if ( icon )
                {
                    std::string urlText = feature->eval( iconUrlExpr, &context );
                    tempStyle.get<IconSymbol>()->url()->setLiteral( urlText );
                }
                
                osg::Node* node = makePlaceNode(
                    context,
                    feature,
                    tempStyle,
                    textPriorityExpr);

                if ( node )
                {
                    if ( context.featureIndex() )
                    {
                        context.featureIndex()->tagNode(node, const_cast<Feature*>(feature));
                    }

                    group->addChild( node );
                }
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


    osg::Node* makePlaceNode(const FilterContext& context,
                             const Feature*       feature, 
                             const Style&         style, 
                             NumericExpression&   priorityExpr )
    {
        osg::Vec3d center = feature->getGeometry()->getBounds().center();
        GeoPoint point(feature->getSRS(), center.x(), center.y());

        PlaceNode* placeNode = new PlaceNode(0L, point, style, context.getDBOptions());

        if ( !priorityExpr.empty() )
        {
            AnnotationData* data = new AnnotationData();
            data->setPriority( feature->eval(priorityExpr, &context) );
            placeNode->setAnnotationData( data );
        }

        return placeNode;
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
