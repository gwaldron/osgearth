/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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
#include <osgEarth/ColorFilter>
#include <osgEarth/ThreadingUtils>

using namespace osgEarth;

ColorFilterRegistry*
ColorFilterRegistry::instance()
{
    // OK to be in the local scope since this gets called at static init time
    // by the OSGEARTH_REGISTER_COLORFILTER macro
    static ColorFilterRegistry* s_singleton =0L;
    static Threading::Mutex     s_singletonMutex;

    if ( !s_singleton )
    {
        Threading::ScopedMutexLock lock(s_singletonMutex);
        if ( !s_singleton )
        {
            s_singleton = new ColorFilterRegistry();
        }
    }
    return s_singleton;
}

bool
ColorFilterRegistry::readChain(const Config& conf, ColorFilterChain& out_chain)
{
    bool createdAtLeastOne = false;

    // first try to parse the top-level config:
    ColorFilter* top = createOne( conf );
    if ( top )
    {
        out_chain.push_back( top );
        createdAtLeastOne = true;
    }

    // failing that, treat it like a chain:
    else
    {
        for( ConfigSet::const_iterator i = conf.children().begin(); i != conf.children().end(); ++i )
        {
            ColorFilter* object = createOne( *i );
            if ( object )
            {
                out_chain.push_back( object );
                createdAtLeastOne = true;
            }
        }
    }

    return createdAtLeastOne;
}


bool
ColorFilterRegistry::writeChain(const ColorFilterChain& chain, Config& out_config)
{
    bool wroteAtLeastOne = false;

    for( ColorFilterChain::const_iterator i = chain.begin(); i != chain.end(); ++i )
    {
        Config conf = i->get()->getConfig();
        if ( !conf.empty() )
        {
            out_config.add( conf );
            wroteAtLeastOne = true;
        }
    }

    return wroteAtLeastOne;
}


void
ColorFilterRegistry::add( const std::string& key, class ColorFilterFactory* factory )
{
    if ( factory )
        _factories[key] = factory;
}

ColorFilter*
ColorFilterRegistry::createOne(const Config& conf) const
{
    FactoryMap::const_iterator f = _factories.find( conf.key() );
    if ( f != _factories.end() && f->second != 0L )
    {
        ColorFilter* object = f->second->create(conf);
        if ( object )
        {
            return object;
        }
    }
    return 0L;
}
