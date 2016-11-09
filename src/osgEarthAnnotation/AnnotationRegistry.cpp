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
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include <osgEarthAnnotation/AnnotationRegistry>
#include <osgEarth/ScreenSpaceLayout>
#include <osgEarth/Registry>
#include <osgEarth/ObjectIndex>

#define LC "[AnnotationRegistry] "

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
        }

        void apply(osg::Node& node)
        {
            AnnotationNode* anno = dynamic_cast<AnnotationNode*>( &node );
            if ( anno )
            {
                Config conf = anno->getConfig();
                _group.add( conf );
            }

            traverse(node);
        }

        Config _group;
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

    // first try to parse the top-level config as an annotation:
    AnnotationNode* top = createOne(mapNode, conf, options);
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
            AnnotationNode* anno = createOne( mapNode, *i, options );
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
                              const osgDB::Options* options) const
{
    FactoryMap::const_iterator f = _factories.find( conf.key() );
    if ( f != _factories.end() && f->second != 0L )
    {
        AnnotationNode* anno = f->second->create(mapNode, conf, options);
        if ( anno )
        {
            Registry::objectIndex()->tagNode( anno, anno );
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
        return visitor._group;
    }
    return Config();
}

