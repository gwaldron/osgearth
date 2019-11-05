/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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

#include "AerodromeCatalog"
#include "AerodromeFeatureOptions"
#include <osgEarth/Config>
#include <osgEarth/XmlUtils>

using namespace osgEarth;
using namespace osgEarth::Aerodrome;

#define LC "[AerodromeCatalog] "

#define AERODROME_CATALOG_CURRENT_VERSION 1


AerodromeCatalog::AerodromeCatalog()
{
    _version = AERODROME_CATALOG_CURRENT_VERSION;
}

void
AerodromeCatalog::fromConfig(const Config& conf)
{
    conf.get("version", _version);

    ConfigSet boundaries = conf.children("boundaries");
    for (ConfigSet::const_iterator i = boundaries.begin(); i != boundaries.end(); i++)
        _boundaryOptions.push_back(BoundaryFeatureOptions(*i));

    ConfigSet lightBeacons = conf.children("light_beacons");
    for (ConfigSet::const_iterator i = lightBeacons.begin(); i != lightBeacons.end(); i++)
        _lightBeaconOptions.push_back(AerodromeFeatureOptions(*i));

    ConfigSet lightIndicators = conf.children("light_indicators");
    for (ConfigSet::const_iterator i = lightIndicators.begin(); i != lightIndicators.end(); i++)
        _lightIndicatorOptions.push_back(AerodromeFeatureOptions(*i));

    ConfigSet linearFeatures = conf.children("linear_features");
    for (ConfigSet::const_iterator i = linearFeatures.begin(); i != linearFeatures.end(); i++)
        _linearFeatureOptions.push_back(AerodromeFeatureOptions(*i));

    ConfigSet pavements = conf.children("pavement");
    for (ConfigSet::const_iterator i = pavements.begin(); i != pavements.end(); i++)
        _pavementOptions.push_back(AerodromeFeatureOptions(*i));

    ConfigSet runways = conf.children("runways");
    for (ConfigSet::const_iterator i = runways.begin(); i != runways.end(); i++)
        _runwayOptions.push_back(AerodromeFeatureOptions(*i));

    ConfigSet runwayThresholds = conf.children("runway_thresholds");
    for (ConfigSet::const_iterator i = runwayThresholds.begin(); i != runwayThresholds.end(); i++)
        _runwayThresholdOptions.push_back(AerodromeFeatureOptions(*i));

    ConfigSet startupLocations = conf.children("startup_locations");
    for (ConfigSet::const_iterator i = startupLocations.begin(); i != startupLocations.end(); i++)
        _startupLocationOptions.push_back(AerodromeFeatureOptions(*i));

    ConfigSet stopways = conf.children("stopways");
    for (ConfigSet::const_iterator i = stopways.begin(); i != stopways.end(); i++)
        _stopwayOptions.push_back(AerodromeFeatureOptions(*i));

    ConfigSet taxiways = conf.children("taxiways");
    for (ConfigSet::const_iterator i = taxiways.begin(); i != taxiways.end(); i++)
        _taxiwayOptions.push_back(AerodromeFeatureOptions(*i));

    ConfigSet taxiwaySigns = conf.children("taxiway_signs");
    for (ConfigSet::const_iterator i = taxiwaySigns.begin(); i != taxiwaySigns.end(); i++)
        _taxiwaySignOptions.push_back(AerodromeFeatureOptions(*i));

    ConfigSet terminals = conf.children("terminals");
    for (ConfigSet::const_iterator i = terminals.begin(); i != terminals.end(); i++)
        _terminalOptions.push_back(TerminalFeatureOptions(*i));

    ConfigSet windsocks = conf.children("windsocks");
    for (ConfigSet::const_iterator i = windsocks.begin(); i != windsocks.end(); i++)
        _windsockOptions.push_back(AerodromeFeatureOptions(*i));
}

Config
AerodromeCatalog::getConfig() const
{
    Config conf;
    conf.set("version", _version);

    for(BoundaryOptionsSet::const_iterator i = _boundaryOptions.begin(); i != _boundaryOptions.end(); ++i)
        conf.add("boundaries", i->getConfig());

    for(AerodromeOptionsSet::const_iterator i = _lightBeaconOptions.begin(); i != _lightBeaconOptions.end(); ++i)
        conf.add("light_beacons", i->getConfig());

    for(AerodromeOptionsSet::const_iterator i = _lightIndicatorOptions.begin(); i != _lightIndicatorOptions.end(); ++i)
        conf.add("light_indicators", i->getConfig());

    for(AerodromeOptionsSet::const_iterator i = _linearFeatureOptions.begin(); i != _linearFeatureOptions.end(); ++i)
        conf.add("linear_features", i->getConfig());

    for(AerodromeOptionsSet::const_iterator i = _pavementOptions.begin(); i != _pavementOptions.end(); ++i)
        conf.add("pavement", i->getConfig());

    for(AerodromeOptionsSet::const_iterator i = _runwayOptions.begin(); i != _runwayOptions.end(); ++i)
        conf.add("runways", i->getConfig());

    for(AerodromeOptionsSet::const_iterator i = _runwayThresholdOptions.begin(); i != _runwayThresholdOptions.end(); ++i)
        conf.add("runway_thresholds", i->getConfig());

    for(AerodromeOptionsSet::const_iterator i = _startupLocationOptions.begin(); i != _startupLocationOptions.end(); ++i)
        conf.add("startup_locations", i->getConfig());

    for(AerodromeOptionsSet::const_iterator i = _stopwayOptions.begin(); i != _stopwayOptions.end(); ++i)
        conf.add("stopways", i->getConfig());

    for(AerodromeOptionsSet::const_iterator i = _taxiwayOptions.begin(); i != _taxiwayOptions.end(); ++i)
        conf.add("taxiways", i->getConfig());

    for(AerodromeOptionsSet::const_iterator i = _taxiwaySignOptions.begin(); i != _taxiwaySignOptions.end(); ++i)
        conf.add("taxiway_signs", i->getConfig());

    for(TerminalOptionsSet::const_iterator i = _terminalOptions.begin(); i != _terminalOptions.end(); ++i)
        conf.add("terminals", i->getConfig());

    for(AerodromeOptionsSet::const_iterator i = _windsockOptions.begin(); i != _windsockOptions.end(); ++i)
        conf.add("windsocks", i->getConfig());

    return conf;
}

AerodromeCatalog*
AerodromeCatalog::read(const URI& uri, const osgDB::Options* options)
{
    osg::ref_ptr<AerodromeCatalog> catalog;

    osg::ref_ptr<XmlDocument> doc = XmlDocument::load( uri, options );
    if ( doc.valid() )
    {
        catalog = new AerodromeCatalog();
        catalog->fromConfig( doc->getConfig().child("catalog") );
    }
    else
    {
        OE_WARN << LC << "Failed to read catalog from " << uri.full() << "\n";
    }

    return catalog.release();
}