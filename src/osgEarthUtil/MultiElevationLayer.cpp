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
#include <osgEarthUtil/MultiElevationLayer>

using namespace osgEarth;
using namespace osgEarth::Util;

#define LC "[MultiElevationLayer] "

#define OE_TEST OE_DEBUG

REGISTER_OSGEARTH_LAYER(multi_elevation, MultiElevationLayer);

//............................................................................

MultiElevationLayerOptions::MultiElevationLayerOptions(const ConfigOptions& co) :
ElevationLayerOptions(co)
{
    fromConfig(_conf);
}

void
MultiElevationLayerOptions::fromConfig(const Config& conf)
{

    const ConfigSet layers = conf.child("layers").children();
    for (ConfigSet::const_iterator i = layers.begin(); i != layers.end(); ++i)
        _layers.push_back(*i);
}

Config
MultiElevationLayerOptions::getConfig() const
{
    Config conf = ElevationLayerOptions::getConfig();
    conf.key() = "multi_elevation";
    Config layers("layers");
    for (std::vector<ConfigOptions>::const_iterator i = _layers.begin(); i != _layers.end(); ++i)
        layers.add(i->getConfig());
    return conf;
}

//............................................................................

MultiElevationLayer::MultiElevationLayer() :
ElevationLayer(&_optionsConcrete),
_options(&_optionsConcrete)
{
    init();
}

MultiElevationLayer::MultiElevationLayer(const MultiElevationLayerOptions& options) :
ElevationLayer(&_optionsConcrete),
_options(&_optionsConcrete),
_optionsConcrete(options)
{
    init();
}

MultiElevationLayer::~MultiElevationLayer()
{
    //todo
}

void
MultiElevationLayer::init()
{
    setTileSourceExpected(false);
    ElevationLayer::init();
}

const Status&
MultiElevationLayer::open()
{
    for (std::vector<ConfigOptions>::const_iterator i = options().layers().begin();
        i != options().layers().end();
        ++i)
    {
        const ConfigOptions& co = *i;
        osg::ref_ptr<Layer> layer = Layer::create(co);
        if (layer.valid())
        {
            OE_INFO << LC << "Adding layer \"" << layer->getName() << "\"...\n";
            ElevationLayer* elayer = dynamic_cast<ElevationLayer*>(layer.get());
            if (elayer)
            {
                elayer->setReadOptions(getReadOptions());
                const Status& s = elayer->open();
                if (!s.isOK())
                {
                    // fail.
                    return setStatus(s);
                }

                // Take profile from the first successfully opened layer.
                if (!getProfile())
                {
                    setProfile(elayer->getProfile());
                }

                _layers.push_back(elayer);
            }
            else
            {
                OE_WARN << LC << "Illegal to add a non-elevation layer\n";
                return setStatus(Status::Error(Status::ConfigurationError, "Only elevation layers are allowed"));
            }
        }
    }

    return ElevationLayer::open();
}

void
MultiElevationLayer::addedToMap(const Map* map)
{
    for (ElevationLayerVector::iterator i = _layers.begin(); i != _layers.end(); ++i)
    {
        i->get()->addedToMap(map);
    }
}

void
MultiElevationLayer::removedFromMap(const Map* map)
{
    for (ElevationLayerVector::iterator i = _layers.begin(); i != _layers.end(); ++i)
    {
        i->get()->removedFromMap(map);
    }
}

osg::HeightField*
MultiElevationLayer::createHeightFieldImplementation(const TileKey& key, ProgressCallback* progress)
{
    osg::ref_ptr<osg::HeightField> heightField = new osg::HeightField();
    heightField->allocate(257, 257);

    // Initialize the heightfield to nodata
    heightField->getFloatArray()->assign(heightField->getFloatArray()->size(), NO_DATA_VALUE);

    // Populate the heightfield and return it if it's valid
    if (_layers.populateHeightField(heightField.get(), key, 0, INTERP_BILINEAR, progress))
    {             
        return heightField.release();
    }
    else
    {        
        return 0L;
    }
}
