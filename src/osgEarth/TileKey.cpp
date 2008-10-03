#include <osgEarth/TileKey>
#include <osgEarth/PlateCarre>
#include <osgEarth/Mercator>

using namespace osgEarth;

TileKey::TileKey() :
profile( TileGridProfile( 0, 0, 0, 0 ) )
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