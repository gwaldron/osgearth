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
#include <osgEarthFeatures/FeatureNode>
#include <algorithm>

using namespace osgEarth;
using namespace osgEarth::Features;

#define LC "[FeatureNode] "

FeatureNode::FeatureNode(FeatureSource * featureSource, FeatureID fid)
	: _featureSource(featureSource), _fid(fid)
{
}

FeatureMultiNode::FeatureMultiNode(FeatureSource * featureSource)
	: FeatureNode(featureSource)
{
}

void FeatureMultiNode::addDrawable(osg::Drawable * drawable, FeatureID fid)
{
	//OE_DEBUG << LC << "addDrawable " << drawable << " fid=" << fid << std::endl;
	_features.insert(DrawableFeatureIDMap::value_type(drawable, fid));
}

void FeatureMultiNode::removeDrawable(osg::Drawable * drawable)
{
	DrawableFeatureIDMap::iterator it = _features.find(drawable);
	if(it != _features.end())
	{
		//OE_DEBUG << LC << "FeatureMultiNode removeDrawable " << drawable << std::endl;
		_features.erase(it);
	}
	else
	{
		//OE_DEBUG << LC << "FeatureMultiNode removeDrawable " << drawable  << " not found" << std::endl;
	}
}

void FeatureMultiNode::clearDrawables()
{
	_features.clear();
}

unsigned FeatureMultiNode::getNumDrawables() const
{
	return _features.size();
}

FeatureID FeatureMultiNode::getFID(osg::Drawable * drawable) const
{
	DrawableFeatureIDMap::const_iterator it = _features.find(drawable);
	FeatureID ret = (it != _features.end())?it->second:-1;
	//OE_DEBUG << LC << "FeatureMultiNode getFID " << drawable << " fid=" << ret << std::endl;
	return ret;
}
