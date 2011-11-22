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
#include "KML_Placemark"
#include "KML_Geometry"
#include "KML_Style"
#include <osgEarthFeatures/FeatureNode>
#include <osgEarthAnnotation/PlaceNode>
#include <osgEarthAnnotation/Decluttering>

using namespace osgEarth::Features;
using namespace osgEarth::Annotation;

void 
KML_Placemark::build( const Config& conf, KMLContext& cx )
{
    Style style;
    if ( conf.hasValue("styleurl") )
    {
        // process a "stylesheet" style
        const Style* ref_style = cx._sheet->getStyle( conf.value("styleurl"), false );
        if ( ref_style )
            style = *ref_style;
    }
    else if ( conf.hasChild("style") )
    {
        // process an "inline" style
        KML_Style kmlStyle;
        kmlStyle.scan( conf.child("style"), cx );
        style = cx._activeStyle;
    }

    URI iconURI;
    MarkerSymbol* iconMarker = style.get<MarkerSymbol>();    
    if ( iconMarker && iconMarker->url().isSet() )
    {
        iconURI = URI( iconMarker->url()->expr(), iconMarker->url()->uriContext() );
    }

    std::string text = 
        conf.hasValue("name") ? conf.value("name") :
        conf.hasValue("description") ? conf.value("description") :
        "Unnamed";

    // read in the geometry:
    bool isPoly = false;
    bool isPoint = false;    

    osg::Vec3d position;
    KML_Geometry geometry;
    geometry.build(conf, cx, style);
    if ( geometry._geom.valid() && geometry._geom->getTotalPointCount() > 0 )
    {
        Geometry* geom = geometry._geom.get();
        position = geom->getBounds().center();
        isPoly = geom->getComponentType() == Geometry::TYPE_POLYGON;
        isPoint = geom->getComponentType() == Geometry::TYPE_POINTSET;
    }

    FeatureNode* fNode = 0L;
    PlaceNode*   pNode = 0L;

    MarkerSymbol* marker = style.get<MarkerSymbol>();    

    // if we have a non-single-point geometry or it's a marker, render it.
    if ( geometry._geom.valid() && geometry._geom->getTotalPointCount() > 1 || marker )    
    {
        const ExtrusionSymbol* ex = style.get<ExtrusionSymbol>();
        const AltitudeSymbol* alt = style.get<AltitudeSymbol>();        

        bool draped =            
            ((ex == 0L && alt == 0L && isPoly) || (ex == 0L && alt != 0L && alt->clamping() == AltitudeSymbol::CLAMP_TO_TERRAIN)) &&
            (!marker);

        // Make a feature node; drape if we're not extruding.
        GeometryCompilerOptions options;
        options.clustering() = false;
        fNode = new FeatureNode( cx._mapNode, new Feature(geometry._geom.get()), draped, options );
        fNode->setStyle( style );

        if ( draped )
            fNode->getOrCreateStateSet()->setMode(GL_LIGHTING, 1);
    }

    if ( isPoint && !marker )
    {
        osg::ref_ptr<osg::Image> image = iconURI.readImage().getImage();
        if ( !image.valid() )
        {
            image = cx._options->defaultIconImage().get();
            if ( !image.valid() )
            {
                image = cx._options->defaultIconURI()->readImage().getImage();
            }
        }

        // apply the default text symbol for labeling, if necessary:
        if ( !style.get<TextSymbol>() && cx._options->defaultTextSymbol().valid() )
        {
            style.addSymbol( cx._options->defaultTextSymbol().get() );
        }

        pNode = new PlaceNode( cx._mapNode, position, image, text, style );
    }

    osg::Group* parent =
        cx._options->iconAndLabelGroup().valid() ?
        cx._options->iconAndLabelGroup() :
        cx._groupStack.top();


    if ( fNode && pNode )
    {
        osg::Group* group = new osg::Group();
        group->addChild( fNode );
        group->addChild( pNode );
        parent->addChild( group );
        KML_Feature::build( conf, cx, group );
    }
    else if ( pNode )
    {
        parent->addChild( pNode );
        KML_Feature::build( conf, cx, pNode );
    }
    else if ( fNode )
    {
        parent->addChild( fNode );
        KML_Feature::build( conf, cx, fNode );
    }

    // apply decluttering if necessary
    if ( pNode && cx._options->declutter() == true && !cx._options->iconAndLabelGroup().valid() )
    {
        Decluttering::setEnabled( pNode->getOrCreateStateSet(), true );
    }
}
