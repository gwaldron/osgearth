/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include <osgEarth/VirtualProgram>
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
        FilterContext&       context )
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
        NumericExpression textSizeExpr    ( text ? *text->size()     : NumericExpression() );
        NumericExpression textRotationExpr( text ? *text->onScreenRotation() : NumericExpression() );
        NumericExpression textCourseExpr  ( text ? *text->geographicCourse() : NumericExpression() );
        StringExpression  iconUrlExpr     ( icon ? *icon->url()      : StringExpression() );
        NumericExpression iconScaleExpr   ( icon ? *icon->scale()    : NumericExpression() );
        NumericExpression iconHeadingExpr ( icon ? *icon->heading()  : NumericExpression() );

        for( FeatureList::const_iterator i = input.begin(); i != input.end(); ++i )
        {
            Feature* feature = i->get();
            if ( !feature )
                continue;
            
            // run a symbol script if present.
            if ( text && text->script().isSet() )
            {
                StringExpression temp( text->script().get() );
                feature->eval( temp, &context );
            }
            
            // run a symbol script if present.
            if ( icon && icon->script().isSet() )
            {
                StringExpression temp( icon->script().get() );
                feature->eval( temp, &context );
            }

            const Geometry* geom = feature->getGeometry();
            if ( !geom )
                continue;

            Style tempStyle = styleCopy;

            // evaluate expressions into literals.
            // TODO: Later we could replace this with a generate "expression evaluator" type
            // that we could pass to PlaceNode in the DB options. -gw

            if ( text )
            {
                if ( text->content().isSet() )
                    tempStyle.get<TextSymbol>()->content()->setLiteral( feature->eval( textContentExpr, &context ) );

                if ( text->size().isSet() )
                    tempStyle.get<TextSymbol>()->size()->setLiteral( feature->eval(textSizeExpr, &context) );

                if ( text->onScreenRotation().isSet() )
                    tempStyle.get<TextSymbol>()->onScreenRotation()->setLiteral( feature->eval(textRotationExpr, &context) );

                if ( text->geographicCourse().isSet() )
                    tempStyle.get<TextSymbol>()->geographicCourse()->setLiteral( feature->eval(textCourseExpr, &context) );
            }

            if ( icon )
            {
                if ( icon->url().isSet() )
                    tempStyle.get<IconSymbol>()->url()->setLiteral( feature->eval(iconUrlExpr, &context) );

                if ( icon->scale().isSet() )
                    tempStyle.get<IconSymbol>()->scale()->setLiteral( feature->eval(iconScaleExpr, &context) );

                if ( icon->heading().isSet() )
                    tempStyle.get<IconSymbol>()->heading()->setLiteral( feature->eval(iconHeadingExpr, &context) );
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
                    context.featureIndex()->tagNode(node, feature);
                }

                group->addChild( node );
            }
        }

        return group;
    }


    osg::Node* makePlaceNode(FilterContext&     context,
                             Feature*           feature, 
                             const Style&       style, 
                             NumericExpression& priorityExpr )
    {
        osg::Vec3d center = feature->getGeometry()->getBounds().center();

        AltitudeMode mode = ALTMODE_ABSOLUTE;        

        const AltitudeSymbol* alt = style.getSymbol<AltitudeSymbol>();
        if (alt &&
           (alt->clamping() == AltitudeSymbol::CLAMP_TO_TERRAIN || alt->clamping() == AltitudeSymbol::CLAMP_RELATIVE_TO_TERRAIN) &&
           alt->technique() == AltitudeSymbol::TECHNIQUE_SCENE)
        {
            mode = ALTMODE_RELATIVE;
        }                              

        GeoPoint point(feature->getSRS(), center.x(), center.y(), center.z(), mode);        

        PlaceNode* node = new PlaceNode(0L, point, style, context.getDBOptions());
        
        if ( !priorityExpr.empty() )
        {
            float val = feature->eval(priorityExpr, &context);
            node->setPriority( val >= 0.0f ? val : FLT_MAX );
        }

        return node;
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

    virtual const char* className() const
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
