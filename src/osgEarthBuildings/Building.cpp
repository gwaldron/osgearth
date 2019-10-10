/* -*-c++-*- */
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
#include "Building"
#include "BuildingVisitor"
#include "BuildContext"

#define LC "[Building] "

using namespace osgEarth::Symbology;
using namespace osgEarth::Buildings;

Building::Building() :
Taggable<osg::Object>(),
_zoning    ( Zoning::ZONING_UNKNOWN ),
_minHeight ( 0.0f ),
_maxHeight ( FLT_MAX ),
_minArea   ( 0.0f ),
_maxArea   ( FLT_MAX ),
_instanced ( false )
{
    //nop
}

Building::Building(const Building& rhs, const osg::CopyOp& copy) :
Taggable<osg::Object>( rhs, copy ),
_zoning    ( rhs._zoning ),
_minHeight ( rhs._minHeight ),
_maxHeight ( rhs._maxHeight ),
_minArea   ( rhs._minArea ),
_maxArea   ( rhs._maxArea ),
_instanced ( rhs._instanced ),
_externalModelURI      ( rhs._externalModelURI ),
_instancedModelSymbol  ( rhs._instancedModelSymbol),
_instancedModelResource( rhs._instancedModelResource)
{
    for(ElevationVector::const_iterator e = rhs.getElevations().begin(); e != rhs.getElevations().end(); ++e)
        _elevations.push_back( e->get()->clone() );
}

void
Building::setHeight(float height)
{
    for(ElevationVector::iterator e = _elevations.begin(); e != _elevations.end(); ++e)
    {
        e->get()->setHeight( height );
    }
}

bool
Building::build(const Polygon* footprint, BuildContext& bc)
{
    if ( !footprint || !footprint->isValid() )
        return false;

    // Resolve an instanced building model if available.
    resolveInstancedModel( bc );
    
    // In the absence of an instanced model, build parametric data.
    if ( getInstancedModelResource() == 0L )
    {
        for(ElevationVector::iterator e = _elevations.begin(); e != _elevations.end(); ++e)
        {
            e->get()->build( footprint, bc );
        }
    }

    // if we are using an instanced model, we still need the rotation/AABB from the first elevation:
    else if ( _elevations.size() > 0 )
    {
        _elevations.front()->calculateRotations(footprint);
    }

    return true;
}

void
Building::resolveInstancedModel(BuildContext& bc)
{
    if ( getInstancedModelSymbol() && bc.getResourceLibrary() )
    {        
        // resolve the resource.
        ModelResourceVector candidates;
        bc.getResourceLibrary()->getModels( getInstancedModelSymbol(), candidates, bc.getDBOptions() );
        if ( !candidates.empty() )
        {
            unsigned index = Random(bc.getSeed()).next( candidates.size() );
            setInstancedModelResource( candidates.at(index).get() );
        }
        else
        {
            OE_WARN << LC << "no matching instanced model:\n"
                << "SYMBOL= " << getInstancedModelSymbol()->getConfig().toJSON(true) << "\n";
        }
    }
}

void
Building::accept(BuildingVisitor& bv)
{
    bv.apply( this );
}

Config
Building::getConfig() const
{
    //TODO: incomplete.
    Config conf("building");
    conf.set("external_model_uri", _externalModelURI);
    if ( !getElevations().empty() )
    {
        Config evec("elevations");
        for(ElevationVector::const_iterator e = getElevations().begin(); e != getElevations().end(); ++e)
            evec.add("elevation", e->get()->getConfig());
        conf.set(evec);
    }
    return conf;
}
