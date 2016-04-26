/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2015 Pelican Mapping
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

SilverLiningNode::SilverLiningNode(const osgEarth::Map*       map,
                                   const SilverLiningOptions& options,
                                   Callback*                  callback) :
_options     (options),
_map(map)
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

void
SilverLiningNode::onSetDateTime()
{
  for (osg::NodeList::iterator itr = _children.begin();
		itr != _children.end();
		++itr)
	{
		SilverLiningContextNode* node = dynamic_cast<SilverLiningContextNode* > ((*itr).get());
		if(node)
			node->onSetDateTime(); 
	}
}

void
SilverLiningNode::onSetMinimumAmbient()
{
  for (osg::NodeList::iterator itr = _children.begin();
		itr != _children.end();
		++itr)
	{
		SilverLiningContextNode* node = dynamic_cast<SilverLiningContextNode* > ((*itr).get());
		if(node)
			node->onSetMinimumAmbient(); 
	}
}

void
SilverLiningNode::traverse(osg::NodeVisitor& nv)
{
	if ( nv.getVisitorType() == nv.CULL_VISITOR )
	{
		osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);
		osg::Camera* camera  = cv->getCurrentCamera();
		if ( camera )
		{
			SilverLiningContextNode *slContextNode = dynamic_cast<SilverLiningContextNode *>(camera->getUserData());
			if (!slContextNode) 
			{
				slContextNode = new SilverLiningContextNode(this, _light, _map, _options);

//Use camera cull mask that will remove the need for context check 
//in SilverLiningContextNode, SilverLiningSkyDrawable and SilverLiningCloudsDrawable
#ifdef SL_USE_CULL_MASK 
				static int nodeMask = 0x1;
				slContextNode->getSLGeode()->setNodeMask(nodeMask);
				slContextNode->setNodeMask(nodeMask);

				int inheritanceMask = 
					(osg::CullSettings::VariablesMask::ALL_VARIABLES &
					~osg::CullSettings::VariablesMask::CULL_MASK);

				camera->setInheritanceMask(inheritanceMask);
				camera->setCullMask(nodeMask);
				nodeMask = nodeMask << 1;
#endif
				camera->setUserData(slContextNode);
				addChild(slContextNode);
			}
		}
	}

    osgEarth::Util::SkyNode::traverse( nv );

    if ( _lightSource.valid() )
    {
        _lightSource->accept(nv);
    }
}
