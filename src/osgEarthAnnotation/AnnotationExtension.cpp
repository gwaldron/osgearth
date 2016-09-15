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
#include "AnnotationExtension"
#include <osgEarthAnnotation/AnnotationRegistry>
#include <osgEarth/Extension>

#include <osgEarth/MapNode>
#include <osgDB/FileNameUtils>
#include <osgDB/ReaderWriter>

using namespace osgEarth;
using namespace osgEarth::Annotation;

#define LC "[AnnotationExtension] "

//.........................................................................


AnnotationExtension::AnnotationExtension(const ConfigOptions& co) :
ConfigOptions( co )
{
    //nop
}

void
AnnotationExtension::setDBOptions(const osgDB::Options* dbOptions)
{
    _dbo = dbOptions;
}

bool
AnnotationExtension::connect(MapNode* mapNode)
{
    if ( !mapNode )
    {
        OE_WARN << LC << "Illegal: MapNode cannot be null." << std::endl;
        return false;
    }

    // decode the Config.
    osg::Group* annotations = 0L;

    AnnotationRegistry::instance()->create( mapNode, getConfig(), _dbo.get(), annotations );

    if ( annotations )
    {
        if ( annotations )
            annotations->setName("osgEarth::AnnotationExtension");

        mapNode->addChild( annotations );
    }

    return true;
}

bool
AnnotationExtension::disconnect(MapNode* mapNode)
{
    if ( mapNode )
    {
        for(unsigned i=0; i<mapNode->getNumChildren(); ++i)
        {
            if ( mapNode->getChild(i)->getName() == "osgEarth::AnnotationExtension")
            {
                mapNode->removeChild(i);
                break;
            }
        }
    }

    return true;
}

REGISTER_OSGEARTH_EXTENSION(osgearth_annotations, AnnotationExtension);
//REGISTER_OSGEARTH_EXTENSION(osgearth_annotation,  AnnotationExtension);
