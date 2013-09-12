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
#include <osgEarthAnnotation/AnnotationRegistry>
#include <osgEarth/Decluttering>

using namespace osgEarth;
using namespace osgEarth::Annotation;

//-------------------------------------------------------------------

namespace
{
    struct CollectAnnotationNodes : public osg::NodeVisitor
    {
        CollectAnnotationNodes() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
        {
            _group.key() = "annotations";
            _declutter   = false;
        }

        void apply(osg::Node& node)
        {
            AnnotationNode* anno = dynamic_cast<AnnotationNode*>( &node );
            if ( anno )
            {
                Config conf = anno->getConfig();
                _group.add( conf );
            }

            if (!_declutter &&
                node.getStateSet() &&
                node.getStateSet()->getRenderBinMode() != osg::StateSet::INHERIT_RENDERBIN_DETAILS &&
                node.getStateSet()->getBinName() == OSGEARTH_DECLUTTER_BIN )
            {
                _declutter = true;
            }

            traverse(node);
        }

        Config _group;
        bool   _declutter;
    };
}

//-------------------------------------------------------------------

AnnotationRegistry*
AnnotationRegistry::instance()
{
    // OK to be in the local scope since this gets called at static init time
    // by the OSGEARTH_REGISTER_ANNOTATION macro
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
AnnotationRegistry::create(MapNode*              mapNode, 
                           const Config&         conf, 
                           const osgDB::Options* options,
                           osg::Group*&          results ) const
{
    bool createdAtLeastOne = false;

    bool declutter = conf.value<bool>("declutter",false) == true;

    // first try to parse the top-level config as an annotation:
    AnnotationNode* top = createOne(mapNode, conf, options, declutter);
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
            AnnotationNode* anno = createOne( mapNode, *i, options, declutter );
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
AnnotationRegistry::createOne(MapNode*              mapNode, 
                              const Config&         conf, 
                              const osgDB::Options* options, 
                              bool                  declutterOrthos ) const
{
    FactoryMap::const_iterator f = _factories.find( conf.key() );
    if ( f != _factories.end() && f->second != 0L )
    {
        AnnotationNode* anno = f->second->create(mapNode, conf, options);
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


Config
AnnotationRegistry::getConfig( osg::Node* graph ) const
{
    if ( graph )
    {
        CollectAnnotationNodes visitor;
        graph->accept( visitor );
        if ( visitor._declutter )
            visitor._group.set( "declutter", "true" );
        return visitor._group;
    }
    return Config();
}

