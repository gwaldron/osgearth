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

#include <osgEarth/DirectReadTileSource>
#include <osgEarth/Compositing>
#include <osgEarth/ElevationManager>

using namespace osgEarth;

DirectReadTileSource::DirectReadTileSource(osgEarth::TileSource* tileSource,
                                           unsigned int tileSize,
                                           const osgDB::ReaderWriter::Options* options):
TileSource(options),
_tileSize(tileSize),
_tileSource(tileSource)
{
}

osg::Image*
DirectReadTileSource::createImage(const osgEarth::TileKey* key)
{
    //If the destination profile and the underlying TileSource profile are the same,
    // simply request the image and return.
    if ( key->getProfile()->isEquivalentTo( _tileSource->getProfile() ) )
    {
        return _tileSource->createImage( key );
    }

    //Mosaic the imagery together
    Compositor comp;
    osg::ref_ptr<GeoImage> mosaic = comp.mosaicImages( key , _tileSource.get() );

    if ( mosaic.valid() )
    {        
        osg::ref_ptr<GeoImage> image = mosaic->reproject(
            key->getProfile()->getSRS(),
            &key->getGeoExtent(),
            _tileSize,
            _tileSize );

        if (image.valid())
        {
            return image->takeImage();
        }
    }

    return NULL;
}

osg::HeightField*
DirectReadTileSource::createHeightField(const osgEarth::TileKey* key)
{
    osg::ref_ptr<ElevationManager> em = new ElevationManager();
    em->getElevationSources().push_back( _tileSource.get() );
    return em->createHeightField( key );
}

const Profile*
DirectReadTileSource::createProfile(const osgEarth::Profile* mapProfile, const std::string& configPath)
{
    //Just set our profile to the MapProfile
    _profile = mapProfile;

    const Profile* ts_profile = _tileSource.valid()? _tileSource->initProfile( mapProfile, configPath ) : 0;

    if (!ts_profile)
    {
        return NULL;
    }

    return _profile.get();
}

int 
DirectReadTileSource::getPixelsPerTile() const
{
    return _tileSize;
}

void DirectReadTileSource::setOverrideProfile( const Profile* profile )
{
	if (_tileSource.valid())
	{
		_tileSource->setOverrideProfile( profile );
	}
}

