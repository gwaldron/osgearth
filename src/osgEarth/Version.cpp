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

#include <osgEarth/Common>
#include <osgEarth/Version>
#include <string>
#include <stdio.h>

extern "C" {

const char* osgEarthGetVersion()
{
    static char osgearth_version[256];
    static int osgearth_version_init = 1;
    if (osgearth_version_init)
    {
        if (OSGEARTH_RC_VERSION == 0 )
        {
            sprintf(osgearth_version,"%d.%d.%d (%s)",
                OSGEARTH_MAJOR_VERSION,
                OSGEARTH_MINOR_VERSION,
                OSGEARTH_PATCH_VERSION,
                osgEarthGitSHA1() );
        }
        else
        {
            sprintf(osgearth_version,"%d.%d.%d RC%d (%s)",
                OSGEARTH_MAJOR_VERSION,
                OSGEARTH_MINOR_VERSION,
                OSGEARTH_PATCH_VERSION,
                OSGEARTH_RC_VERSION,
                osgEarthGitSHA1() );
        }

        osgearth_version_init = 0;
    }
    
    return osgearth_version;
}

const char* osgEarthGetSOVersion()
{
    static char osgearth_soversion[32];
    static int osgearth_soversion_init = 1;
    if (osgearth_soversion_init)
    {
        sprintf(osgearth_soversion,"%d",OSGEARTH_SOVERSION);
        osgearth_soversion_init = 0;
    }
    
    return osgearth_soversion;
}

const char* osgEarthGetLibraryName()
{
    return "osgEarth Library";
}

}
