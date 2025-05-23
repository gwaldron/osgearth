/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/BuildTextFilter>
#include <osgEarth/FilterContext>
#include <osgEarth/TextSymbol>
#include <osgEarth/PlaceNode>
#include <osgEarth/FeatureIndex>
#include <osgEarth/Notify>

#define LC "[BuildTextFilter] "

using namespace osgEarth;
using namespace osgEarth::Util;

namespace
{
    PlaceNode* makePlaceNode(
        FilterContext&     context,
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
        //LabelNode* node = new LabelNode();
        node->setStyle(style, context.getDBOptions());
        node->setPosition(point);

        return node;
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
                    tempStyle.get<TextSymbol>()->content().mutable_value().setLiteral( feature->eval( textContentExpr, &context ) );

                if ( text->size().isSet() )
                    tempStyle.get<TextSymbol>()->size().mutable_value().setLiteral( feature->eval(textSizeExpr, &context) );

                if ( text->onScreenRotation().isSet() )
                    tempStyle.get<TextSymbol>()->onScreenRotation().mutable_value().setLiteral( feature->eval(textRotationExpr, &context) );

                if ( text->geographicCourse().isSet() )
                    tempStyle.get<TextSymbol>()->geographicCourse().mutable_value().setLiteral( feature->eval(textCourseExpr, &context) );
            }

            if ( icon )
            {
                if ( icon->url().isSet() )
                    tempStyle.get<IconSymbol>()->url().mutable_value().setLiteral( feature->eval(iconUrlExpr, &context) );

                if ( icon->scale().isSet() )
                    tempStyle.get<IconSymbol>()->scale().mutable_value().setLiteral( feature->eval(iconScaleExpr, &context) );

                if ( icon->heading().isSet() )
                    tempStyle.get<IconSymbol>()->heading().mutable_value().setLiteral( feature->eval(iconHeadingExpr, &context) );
            }

            PlaceNode* node = makePlaceNode(
                context,
                feature,
                tempStyle);

            if ( node )
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

                group->addChild( node );
            }
        }

        return group;
    }
}

BuildTextFilter::BuildTextFilter( const Style& style ) :
_style( style )
{
    //nop
}

osg::Node*
BuildTextFilter::push( FeatureList& input, FilterContext& context )
{
    const TextSymbol* text = _style.get<TextSymbol>();
    const IconSymbol* icon = _style.get<IconSymbol>();

    if ( !text && !icon )
    {
        OE_WARN << LC << "Insufficient symbology (no TextSymbol/IconSymbol)" << std::endl;
        return 0L;
    }

    return createNode( input, _style, context );
}
