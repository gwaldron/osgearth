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
#include "KML_Feature"
#include "KML_Style"
#include "KML_StyleMap"
#include <osg/UserDataContainer>
#include <osg/ValueObject>
#include <osgEarth/Viewpoint>

using namespace osgEarth_kml;
using namespace osgEarth;

void
KML_Feature::scan( const Config& conf, KMLContext& cx )
{
    KML_Object::scan(conf, cx);
    for_many( Style, scan, conf, cx );
    for_many( StyleMap, scan, conf, cx );
}

void
KML_Feature::scan2( const Config& conf, KMLContext& cx )
{
    KML_Object::scan2(conf, cx);
    for_many( Style, scan2, conf, cx );
    for_many( StyleMap, scan2, conf, cx );
}

void
KML_Feature::build( const Config& conf, KMLContext& cx, osg::Node* working )
{
    KML_Object::build(conf, cx, working);

    // subclass feature is built; now add feature level data if available
    if ( working )
    {
        // parse the visibility to show/hide the item by default:
        if ( conf.hasValue("visibility") )
            working->setNodeMask( conf.value<int>("visibility", 1) == 1 ? ~0 : 0 );

        // parse a "LookAt" element (stores a viewpoint)
        AnnotationData* anno = getOrCreateAnnotationData(working);
        
        anno->setName( conf.value("name") );
        anno->setDescription( conf.value("description") );

        const Config& lookat = conf.child("lookat");
        if ( !lookat.empty() )
        {
            Viewpoint vp(
                lookat.value<double>("longitude", 0.0),
                lookat.value<double>("latitude", 0.0),
                lookat.value<double>("altitude", 0.0),
                lookat.value<double>("heading", 0.0),
                -lookat.value<double>("tilt", 45.0),
                lookat.value<double>("range", 10000.0) );

            anno->setViewpoint( vp );
        }

        const Config& extdata = conf.child("extendeddata");
        if ( !extdata.empty() )
        {
            ConfigSet innerConfs = extdata.children("data");
            for( ConfigSet::const_iterator i = innerConfs.begin(); i != innerConfs.end(); ++i )
            {
                working->setUserValue(i->value("name"), i->value("value"));
            }
        }
    }
}
