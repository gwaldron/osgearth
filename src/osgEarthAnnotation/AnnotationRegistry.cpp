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
#include <osgEarthAnnotation/AnnotationRegistry>
#include <osgEarthAnnotation/Decluttering>

using namespace osgEarth;
using namespace osgEarth::Annotation;


AnnotationRegistry*
AnnotationRegistry::instance()
{
    static AnnotationRegistry* s_singleton =0L;
    static Threading::Mutex    s_singletonMutex;
    if ( !s_singleton )
    {
        Threading::ScopedMutexLock lock(s_singletonMutex);
        if ( !s_singleton )
        {
            s_singleton = new AnnotationRegistry();
        }
    }
    return s_singleton;
}


void
AnnotationRegistry::add( const std::string& type, AnnotationFactory* factory )
{
    if ( factory )
        _factories[type] = factory;
}


bool
AnnotationRegistry::create( MapNode* mapNode, const Config& conf, osg::Group*& results ) const
{
    bool createdAtLeastOne = false;

    bool declutter = conf.value<bool>("declutter",false) == true;

    // first try to parse the top-level config as an annotation:
    AnnotationNode* top = createOne(mapNode, conf, declutter);
    if ( top )
    {
        if ( results == 0L )
            results = new osg::Group();
        results->addChild( top );
        createdAtLeastOne = true;
    }

    // failing that, treat it like a group of annotations:
    else
    {
        for( ConfigSet::const_iterator i = conf.children().begin(); i != conf.children().end(); ++i )
        {
            AnnotationNode* anno = createOne( mapNode, *i, declutter );
            if ( anno )
            {
                if ( results == 0L )
                    results = new osg::Group();
                results->addChild( anno );
                createdAtLeastOne = true;
            }
        }
    }

    return createdAtLeastOne;
}



AnnotationNode*
AnnotationRegistry::createOne( MapNode* mapNode, const Config& conf, bool declutterOrthos ) const
{
    FactoryMap::const_iterator f = _factories.find( conf.key() );
    if ( f != _factories.end() && f->second != 0L )
    {
        AnnotationNode* anno = f->second->create(mapNode, conf);
        if ( anno )
        {
            if ( declutterOrthos && dynamic_cast<SupportsDecluttering*>(anno) )
            {
                Decluttering::setEnabled( anno->getOrCreateStateSet(), true );
            }

            return anno;
        }
    }
    return 0L;
}
