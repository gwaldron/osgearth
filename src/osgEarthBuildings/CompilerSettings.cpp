/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2016 Pelican Mapping
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
#include "CompilerSettings"

using namespace osgEarth;
using namespace osgEarth;
using namespace osgEarth::Buildings;

CompilerSettings::CompilerSettings() :
_rangeFactor  ( 6.0f ),
_useClustering( false )
{
    //nop
}

CompilerSettings::CompilerSettings(const CompilerSettings& rhs) :
_rangeFactor( rhs._rangeFactor ),
_useClustering( rhs._useClustering ),
_lodBins( rhs._lodBins )
{
    //nop
}

CompilerSettings::LODBin&
CompilerSettings::addLODBin()
{
    _lodBins.push_back(LODBin());
    return _lodBins.back();
}

const CompilerSettings::LODBin*
CompilerSettings::getLODBin(const std::string& tag) const
{
    for(LODBins::const_iterator bin = _lodBins.begin(); bin != _lodBins.end(); ++bin)
    {
        if ( tag == bin->tag )
        {
            return &(*bin);
        }
    }
    return 0L;
}

const CompilerSettings::LODBin*
CompilerSettings::getLODBin(const TagSet& tags) const
{
    for(LODBins::const_iterator bin = _lodBins.begin(); bin != _lodBins.end(); ++bin)
    {
        if ( tags.find(bin->tag) != tags.end() )
        {
            return &(*bin);
        }
    }
    return 0L;
}


CompilerSettings::CompilerSettings(const Config& conf) :
_rangeFactor( 6.0f )
{
    const Config* bins = conf.child_ptr("bins");
    if ( bins )
    {
        for(ConfigSet::const_iterator b = bins->children().begin(); b != bins->children().end(); ++b )
        {
            LODBin& bin = addLODBin();
            bin.tag = b->value("tag");
            bin.lodScale = b->value("lod_scale", 1.0f);
            bin.minLodScale = b->value("min_lod_scale", 0.0f);
        }
    }
    conf.get("range_factor", _rangeFactor);
    conf.get("clustering", _useClustering);
    conf.get("max_verts_per_cluster", _maxVertsPerCluster);
}

Config
CompilerSettings::getConfig() const
{
    Config conf("settings");
    if (!_lodBins.empty())
    {
        Config bins("bins");
        conf.set(bins);

        for(LODBins::const_iterator b = _lodBins.begin(); b != _lodBins.end(); ++b)
        {
            Config bin("bin");
            bins.add(bin);
            
            if (!b->tag.empty())
                bin.set("tag", b->tag);
            
            bin.set("lodscale", b->lodScale);
        }
    }
    
    conf.set("range_factor", _rangeFactor);
    conf.set("clustering", _useClustering);
    conf.set("max_verts_per_cluster", _maxVertsPerCluster);

    return conf;
}
