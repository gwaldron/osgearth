/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#include <osgEarthFeatures/FilterContext>
#include <osgEarthAnnotation/LabelNode>
#include <osgEarthAnnotation/PlaceNode>
#include <osgEarthAnnotation/BarNode>
#include <osgEarth/DepthOffset>
#include <osgEarth/VirtualProgram>
#include <osgEarth/StateSetCache>
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
        if ( style.get<TextSymbol>() == 0L && style.get<IconSymbol>() == 0L && style.get<BarSymbol>() == 0L)
            return 0L;

        // copy the style so we can (potentially) modify the text symbol.
        Style styleCopy = style;
        TextSymbol* text = styleCopy.get<TextSymbol>();
        IconSymbol* icon = styleCopy.get<IconSymbol>();
        BarSymbol* bar = styleCopy.get<BarSymbol>();
        AltitudeSymbol* alt = styleCopy.get<AltitudeSymbol>();

        osg::Group* group = new osg::Group();
        
        StringExpression  textContentExpr ( text ? *text->content()  : StringExpression() );
        NumericExpression textPriorityExpr( text ? *text->priority() : NumericExpression() );
        NumericExpression textSizeExpr    ( text ? *text->size()     : NumericExpression() );
        NumericExpression textRotationExpr( text ? *text->onScreenRotation() : NumericExpression() );
        NumericExpression textCourseExpr  ( text ? *text->geographicCourse() : NumericExpression() );
        StringExpression  iconUrlExpr     ( icon ? *icon->url()      : StringExpression() );
        NumericExpression iconScaleExpr   ( icon ? *icon->scale()    : NumericExpression() );
        NumericExpression iconHeadingExpr ( icon ? *icon->heading()  : NumericExpression() );
        NumericExpression vertOffsetExpr  ( alt  ? *alt->verticalOffset() : NumericExpression() );

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

            if( bar )
            {
                if (bar->width().isSet())
                {
                    NumericExpression expr(*bar->width());
                    tempStyle.get<BarSymbol>()->width()->setLiteral(feature->eval(expr, &context));
                }
                if(!bar->values().empty())
                {
                    BarSymbol::ValueList values(bar->values().size());
                    unsigned index = 0;
                    for(BarSymbol::ValueList::const_iterator it = bar->values().begin(); it != bar->values().end(); ++it, ++index)
                    {
                        const BarSymbol::Value& entry = *it;
                        BarSymbol::Value& out = values[index];
                        out = entry;
                        NumericExpression barValueExpr(*entry.value());
                        NumericExpression barValueScaleExpr(*entry.valueScale());
                        NumericExpression barMinValueExpr(*entry.minimumValue());
                        NumericExpression barMaxValueExpr(*entry.maximumValue());

                        if (entry.value().isSet())
                            out.value()->setLiteral(feature->eval(barValueExpr, &context));
                        if (entry.valueScale().isSet())
                            out.valueScale()->setLiteral(feature->eval(barValueScaleExpr, &context));
                        if (entry.minimumValue().isSet())
                            out.minimumValue()->setLiteral(feature->eval(barMinValueExpr, &context));
                        if (entry.maximumValue().isSet())
                            out.maximumValue()->setLiteral(feature->eval(barMaxValueExpr, &context));
                        
                    }
                    tempStyle.get<BarSymbol>()->values() = values;
                }
            }
            
            GeoPositionNode* node = 0L;
            GeoPositionNode* barNode = 0L;
            if (bar)
            {
                barNode = makeBarNode(
                    context,
                    feature,
                    tempStyle);
            }
            if (text || icon)
            {
                node = makePlaceNode(
                    context,
                    feature,
                    tempStyle);
            }

            if (node)
            {
                if (!textPriorityExpr.empty())
                {
                    float val = feature->eval(textPriorityExpr, &context);
                    node->setPriority( val >= 0.0f ? val : FLT_MAX );
                }

                if (alt && alt->technique() == alt->TECHNIQUE_SCENE && !vertOffsetExpr.empty())
                {
                    float val = feature->eval(vertOffsetExpr, &context);
                    const osg::Vec3d& off = node->getLocalOffset();
                    node->setLocalOffset(osg::Vec3d(off.x(), off.y(), val));
                }

                if ( context.featureIndex() )
                {
                    context.featureIndex()->tagNode(node, feature);
                }

                group->addChild(node);
            }
            if(barNode)
            {
                if (!textPriorityExpr.empty())
                {
                    float val = feature->eval(textPriorityExpr, &context);
                    barNode->setPriority(val >= 0.0f ? val : FLT_MAX);
                }

                if (alt && alt->technique() == alt->TECHNIQUE_SCENE && !vertOffsetExpr.empty())
                {
                    float val = feature->eval(vertOffsetExpr, &context);
                    const osg::Vec3d& off = barNode->getLocalOffset();
                    barNode->setLocalOffset(osg::Vec3d(off.x(), off.y(), val));
                }

                if (context.featureIndex())
                {
                    context.featureIndex()->tagNode(barNode, feature);
                }
                group->addChild(barNode);
            }
        }

        return group;
    }


    PlaceNode* makePlaceNode(FilterContext&     context,
                             Feature*           feature, 
                             const Style&       style )
    {
        osg::Vec3d center = feature->getGeometry()->getBounds().center();

        AltitudeMode mode = ALTMODE_ABSOLUTE;        
        osg::Vec3d localOffset;

        GeoPoint point;

        const AltitudeSymbol* alt = style.get<AltitudeSymbol>();

        // If the symbol asks for map-clamping, disable any auto-scene-clamping on the annotation:
        if (alt != NULL &&
            alt->clamping() != alt->CLAMP_NONE &&
            alt->technique().isSetTo(alt->TECHNIQUE_MAP))
        {
            point.set(feature->getSRS(), center.x(), center.y(), center.z(), ALTMODE_ABSOLUTE);
        }

        // If the symbol says clamp to terrain (but not using the map), zero out the 
        // Z value and dynamically clamp to the surface:
        else if (
            alt != NULL &&
            alt->clamping() == alt->CLAMP_TO_TERRAIN &&
            !alt->technique().isSetTo(alt->TECHNIQUE_MAP))
        {
            point.set(feature->getSRS(), center.x(), center.y(), 0.0, ALTMODE_RELATIVE);
        }

        // By default, use terrain-relative scene clamping Zm above the terrain.
        else
        {
            point.set(feature->getSRS(), center.x(), center.y(), center.z(), ALTMODE_RELATIVE);
        }

        PlaceNode* node = new PlaceNode();
        node->setStyle(style, context.getDBOptions());
        node->setPosition(point);

        return node;
    }

    BarNode* makeBarNode(FilterContext& context,
        Feature* feature,
        const Style& style)
    {
        osg::Vec3d center = feature->getGeometry()->getBounds().center();

        //AltitudeMode mode = ALTMODE_ABSOLUTE;        
        osg::Vec3d localOffset;

        GeoPoint point;

        const AltitudeSymbol* alt = style.get<AltitudeSymbol>();

        // If the symbol asks for map-clamping, disable any auto-scene-clamping on the annotation:
        if (alt != NULL &&
            alt->clamping() != alt->CLAMP_NONE &&
            alt->technique().isSetTo(alt->TECHNIQUE_MAP))
        {
            point.set(feature->getSRS(), center.x(), center.y(), center.z(), ALTMODE_ABSOLUTE);
        }

        // If the symbol says clamp to terrain (but not using the map), zero out the 
        // Z value and dynamically clamp to the surface:
        else if (
            alt != NULL &&
            alt->clamping() == alt->CLAMP_TO_TERRAIN &&
            !alt->technique().isSetTo(alt->TECHNIQUE_MAP))
        {
            point.set(feature->getSRS(), center.x(), center.y(), 0.0, ALTMODE_RELATIVE);
        }

        // By default, use terrain-relative scene clamping Zm above the terrain.
        else
        {
            point.set(feature->getSRS(), center.x(), center.y(), center.z(), ALTMODE_RELATIVE);
        }

        BarNode* node = new BarNode();
        node->setStyle(style);
        node->setPosition(point);

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
