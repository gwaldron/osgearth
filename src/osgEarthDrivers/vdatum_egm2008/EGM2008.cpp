/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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

#include <osgEarth/VerticalDatum>
#include <osgEarth/Geoid>
#include <osgEarth/Units>
#include <osgDB/ReaderWriter>
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>
#include "EGM2008Grid.h"

using namespace osgEarth;

//--------------------------------------------------------------------------

namespace
{
    class EGM2008VerticalDatum : public VerticalDatum
    {
    public:
        EGM2008VerticalDatum() : VerticalDatum(
            "EGM2008",                                  // readable name
            "egm2008" )                                 // initialization string
        {
            // build a heightfield from the data.

            unsigned cols = 1441, rows = 721;
            float colStep = 0.25f, rowStep = 0.25f;

            osg::HeightField* hf = new osg::HeightField();
            hf->allocate( cols, rows );
            osg::Vec3 origin(-180.f, -90.f, 0.f);
            hf->setOrigin( origin );
            hf->setXInterval( colStep );
            hf->setYInterval( rowStep );

            for( unsigned c=0; c<cols-1; ++c )
            {
                float inputLon = 0.0f + float(c) * colStep;
                if ( inputLon > 180.0 ) inputLon -= 360.0;

                for( unsigned r=0; r<rows; ++r )
                {
                    float inputLat = 90.0f - float(r) * rowStep;

                    unsigned outc = unsigned( (inputLon-origin.x())/colStep );
                    unsigned outr = unsigned( (inputLat-origin.y())/rowStep );

                    Linear h( (double)s_egm2008grid[r*cols+c], Units::CENTIMETERS );
                    hf->setHeight( outc, outr, float(h.as(Units::METERS)) );
                }
            }

            _geoid = new Geoid();
            _geoid->setHeightField( hf );
            _geoid->setUnits( Units::METERS );
            _geoid->setName( "EGM2008" );
        }
    };
}


class EGM2008VerticalDatumFactory : public osgDB::ReaderWriter
{
public:
    EGM2008VerticalDatumFactory()
    {
        supportsExtension( "osgearth_vdatum_egm2008", "osgEarth EGM2008 vertical datum" );
    }

    virtual const char* className()
    {
        return "osgEarth EGM2008 vertical datum";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return ReadResult( new EGM2008VerticalDatum() );
    }
};

REGISTER_OSGPLUGIN(osgearth_vdatum_egm2008, EGM2008VerticalDatumFactory) 
