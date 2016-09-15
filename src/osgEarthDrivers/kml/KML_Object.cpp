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
#include "KML_Object"

using namespace osgEarth_kml;

void
KML_Object::build( xml_node<>* node, KMLContext& cx, osg::Node* working )
{
    //todo - read ID
}

AnnotationData*
KML_Object::getOrCreateAnnotationData( osg::Node* node )
{
    AnnotationData* data = dynamic_cast<AnnotationData*>( node->getUserData() );
    if ( !data ) {
        data = new AnnotationData();
        node->setUserData( data );
    }
    return data;
}

