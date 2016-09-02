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
#include <SilverLining.h>

#include "SilverLiningNode"
#include "SilverLiningContextNode"
#include "SilverLiningContext"
#include "SilverLiningSkyDrawable"
#include "SilverLiningCloudsDrawable"

#include <osg/Light>
#include <osg/LightSource>
#include <osgEarth/CullingUtils>

#undef  LC
#define LC "[SilverLiningNode] "

using namespace osgEarth::SilverLining;

SilverLiningNode::SilverLiningNode(const osgEarth::SpatialReference*    mapSRS,
                                   const SilverLiningOptions& options,
                                   Callback*                  callback) :
_options(options),
_mapSRS(mapSRS),
_callback(callback)
{
    // Create a new Light for the Sun.
    _light = new osg::Light();
    _light->setLightNum( 0 );
    _light->setDiffuse( osg::Vec4(1,1,1,1) );
    _light->setAmbient( osg::Vec4(0.2f, 0.2f, 0.2f, 1) );
    _light->setPosition( osg::Vec4(1, 0, 0, 0) ); // w=0 means infinity
    _light->setDirection( osg::Vec3(-1,0,0) );

    _lightSource = new osg::LightSource();
    _lightSource->setLight( _light.get() );
    _lightSource->setReferenceFrame(osg::LightSource::RELATIVE_RF);

    // scene lighting
    osg::StateSet* stateset = this->getOrCreateStateSet();
    _lighting = new PhongLightingEffect();
    _lighting->setCreateLightingUniform( false );
    _lighting->attach( stateset );

    // need update traversal.
    ADJUST_UPDATE_TRAV_COUNT(this, +1);
}


SilverLiningNode::~SilverLiningNode()
{
    if ( _lighting.valid() )
        _lighting->detach();
}

void
SilverLiningNode::attach(osg::View* view, int lightNum)
{
    _light->setLightNum( lightNum );
    view->setLight( _light.get() );
    view->setLightingMode( osg::View::SKY_LIGHT );
}

unsigned
SilverLiningNode::getNumContexts() const
{
    return static_cast<unsigned>(_contextList.size());
}

osg::StateSet*
SilverLiningNode::getCloudsStateSet(unsigned index) const
{
    if (index < getNumContexts())
    {
        const SilverLiningContextNode* node = dynamic_cast<const SilverLiningContextNode*>(_contextList[index].get());
        if ( node )
            return node->getCloudsStateSet();
    }
    return 0L;
}

osg::StateSet*
SilverLiningNode::getSkyStateSet(unsigned index) const
{
    if (index < getNumContexts())
    {
        const SilverLiningContextNode* node = dynamic_cast<const SilverLiningContextNode*>(_contextList[index].get());
        if ( node )
            return node->getSkyStateSet();
    }
    return 0L;
}

void
SilverLiningNode::onSetDateTime()
{
    for (CameraContextMap::const_iterator itr = _contexts.begin();
        itr != _contexts.end();
        ++itr)
    {
        SilverLiningContextNode* node = dynamic_cast<SilverLiningContextNode* > ((*itr).second.get());
        if(node)
            node->onSetDateTime();
    }
}

void
SilverLiningNode::onSetMinimumAmbient()
{
    for (CameraContextMap::const_iterator itr = _contexts.begin();
        itr != _contexts.end();
        ++itr)
    {
        SilverLiningContextNode* node = dynamic_cast<SilverLiningContextNode* > ((*itr).second.get());
        if(node)
            node->onSetMinimumAmbient();
    }
}

void
SilverLiningNode::traverse(osg::NodeVisitor& nv)
{
    static Threading::Mutex s_mutex;

    if ( nv.getVisitorType() == nv.CULL_VISITOR )
    {
        osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);
        osg::Camera* camera = cv->getCurrentCamera();
        if ( camera )
        {
            Threading::ScopedMutexLock lock(s_mutex);

            CameraContextMap::const_iterator i = _contexts.find(camera);
            if (i == _contexts.end())
            {
                _camerasToAdd.insert(camera);
            }

            else
            {
                i->second->accept(nv);
            }
        }
    }

    else if (nv.getVisitorType() == nv.UPDATE_VISITOR)
    {
        {
            Threading::ScopedMutexLock lock(s_mutex);
            if (!_camerasToAdd.empty())
            {
                for (CameraSet::const_iterator i = _camerasToAdd.begin(); i != _camerasToAdd.end(); ++i)
                {
                    SilverLiningContextNode* newNode = new SilverLiningContextNode(this, i->get(), _light, _mapSRS, _options, _callback);
                    _contexts[i->get()] = newNode;
                    _contextList.push_back(newNode);
                }
                _camerasToAdd.clear();
            }
        }

        for (CameraContextMap::const_iterator i = _contexts.begin(); i != _contexts.end(); ++i)
        {
            i->second->accept(nv);
        }
    }

    else
    {
        Threading::ScopedMutexLock lock(s_mutex);
        for (CameraContextMap::const_iterator i = _contexts.begin(); i != _contexts.end(); ++i)
        {
            i->second->accept(nv);
        }
    }

    if ( _lightSource.valid() )
    {
        _lightSource->accept(nv);
    }

    osgEarth::Util::SkyNode::traverse(nv);
}
