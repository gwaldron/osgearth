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
#include <osgEarthFeatures/MarkerFactory>
#include <osgEarthUtil/Annotation>

using namespace osgEarth::Features;
using namespace osgEarth::Util::Annotation;

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
    MarkerSymbol* marker = style.get<MarkerSymbol>();
    if ( marker && marker->url().isSet() )
    {
        iconURI = marker->url()->expr();
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
    if ( geometry._geom.valid() && geometry._geom->size() > 0 )
    {
        Geometry* geom = geometry._geom.get();
        position = geom->getBounds().center();
        isPoly = geom->getComponentType() == Geometry::TYPE_POLYGON;
        isPoint = geom->getComponentType() == Geometry::TYPE_POINTSET;
    }

    FeatureNode*   fNode = 0L;
    PlacemarkNode* pNode = 0L;

    // if we have a non-single-point geometry, render it.
    if ( geometry._geom.valid() && geometry._geom->size() != 1 )
    {
        const ExtrusionSymbol* ex = style.get<ExtrusionSymbol>();
        const AltitudeSymbol* alt = style.get<AltitudeSymbol>();

        bool draped =
            (ex == 0L && alt == 0L && isPoly) ||
            (ex == 0L && alt != 0L && alt->clamping() == AltitudeSymbol::CLAMP_TO_TERRAIN);

        // Make a feautre node; drape if we're not extruding.
        fNode = new FeatureNode( cx._mapNode, new Feature(geometry._geom.get()), draped );
        fNode->setStyle( style );

        if ( draped )
            fNode->getOrCreateStateSet()->setMode(GL_LIGHTING, 1);
    }

    if ( isPoint )
    {
        osg::Image* image = iconURI.readImage();
        if ( !image )
        {
            image = cx._options->defaultIconImage().get();
            if ( !image )
            {
                image = cx._options->defaultIconURI()->readImage();
            }
        }

        pNode = new PlacemarkNode( cx._mapNode, position, image, text, style );
    }

    if ( fNode && pNode )
    {
        osg::Group* group = new osg::Group();
        group->addChild( fNode );
        group->addChild( pNode );
        cx._groupStack.top()->addChild( group );
        KML_Feature::build( conf, cx, group );
    }
    else if ( pNode )
    {
        cx._groupStack.top()->addChild( pNode );
        KML_Feature::build( conf, cx, pNode );
    }
    else if ( fNode )
    {
        cx._groupStack.top()->addChild( fNode );
        KML_Feature::build( conf, cx, fNode );
    }
}
