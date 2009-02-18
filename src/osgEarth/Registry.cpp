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


#include <osgEarth/Registry>
#include <osg/Notify>

using namespace osgEarth;

Registry::Registry():
Referenced(true)
{
}

Registry::~Registry()
{
    osg::notify(osg::NOTICE) << "Destroying osgEarth::Registry " << std::endl;
}

Registry* Registry::instance(bool erase)
{
    static osg::ref_ptr<Registry> s_registry = new Registry;
    if (erase) 
    {   
        s_registry->destruct();
        s_registry = 0;
    }
    return s_registry.get(); // will return NULL on erase
}

void Registry::destruct()
{
    //Clean up the overriden cache config
    _cacheConfigOverride = 0;
}