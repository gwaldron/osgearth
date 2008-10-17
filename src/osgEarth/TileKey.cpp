#include <osgEarth/TileKey>
#include <osgEarth/PlateCarre>
#include <osgEarth/Mercator>

using namespace osgEarth;

TileKey::TileKey() :
profile( TileGridProfile( 0, 0, 256, 256, 256 ) )
{
    //NOP
}

TileKey::TileKey( const TileKey& rhs ) :
key( rhs.key ),
profile( rhs.profile )
{
    //NOP
}

TileKey::TileKey( const std::string& _key, const TileGridProfile& _profile ) :
key( _key ),
profile( _profile )
{
    //NOP
}

const std::string&
TileKey::str() const
{
    return key;
}

std::string
TileKey::getName() const
{
    return getTypeCode() + str();
}

const TileGridProfile&
TileKey::getProfile() const
{
    return profile;
}

int
TileKey::getMapSizePixels() const
{
    return getMapSizePixels( profile.pixelsPerTile(), getLevelOfDetail() );
}

/*static*/ int
TileKey::getMapSizePixels( int tile_size, int lod )
{
    return tile_size << lod;
}

int
TileKey::getMapSizeTiles() const
{
    return getMapSizePixels() / profile.pixelsPerTile();
}

/************************************************************************/

TileKey*
TileKeyFactory::createFromName( const std::string& input )
{
    TileKey* result = NULL;
    if ( input.length() >= 1 )
    {
        if ( input.substr( 0, 1 ) == PlateCarreTileKey::TYPE_CODE )
            result = new PlateCarreTileKey( input.substr( 1 ) );
        else if ( input.substr( 0, 1 ) == MercatorTileKey::TYPE_CODE )
            result = new MercatorTileKey( input.substr( 1 ) );
    }
    return result;
}