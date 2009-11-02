/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osgEarthFeatures/Styling>
#include <stack>

using namespace osgEarth;
using namespace osgEarthFeatures;
using namespace osgEarthFeatures::Styling;


void
StyleVisitor::apply( class Stroke& obj ) { 
    //nop
}

void
StyleVisitor::apply( class Fill& obj ) {
    //nop
}

void
StyleVisitor::apply( class LineSymbolizer& obj ) {
    obj.stroke().accept( *this );
}

void
StyleVisitor::apply( class PolygonSymbolizer& obj ) {
    obj.fill().accept( *this );
}

void
StyleVisitor::apply( class Rule& obj ) {
    obj.filter().accept( *this );
    obj.lineSymbolizer().accept( *this );
    obj.polygonSymbolizer().accept( *this );
}

void
StyleVisitor::apply( class RuleFilter& obj ) {
}

void
StyleVisitor::apply( class FeatureTypeStyle& obj ) {
    for(RuleList::iterator i = obj.rules().begin(); i != obj.rules().end(); i++ )
        i->accept( *this );
}

void
StyleVisitor::apply( class UserLayer& obj ) {
    obj.featureTypeStyle().accept( *this );
}

void
StyleVisitor::apply( class StyledLayerDescriptor& obj ) {
    for(UserLayerList::iterator i = obj.userLayers().begin(); i != obj.userLayers().end(); i++ )
        i->accept( *this );
}

/**************************************************************************/

