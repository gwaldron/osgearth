#include <osgEarth/CacheSeed>
#include <osgEarth/Mercator>
#include <osgEarth/PlateCarre>

using namespace osgEarth;

void CacheSeed::seed(MapConfig *map)
{
    osg::ref_ptr<TileKey> key;

    //Create the root key
    if ( map->getTileProjection() == MapConfig::PROJ_MERCATOR )
        key = new MercatorTileKey( "" );
    else
        key = new PlateCarreTileKey( "" );

    //Create a TileBuilder for the map
    osg::ref_ptr<TileBuilder> tile_builder = TileBuilder::create( map, map->getFilename() );

    processKey( tile_builder, key.get() );
}

void CacheSeed::processKey(TileBuilder* tile_builder, TileKey *key)
{
    if (_maxLevel >= key->getLevelOfDetail())
    {
        osg::notify(osg::NOTICE) << "Processing " << key->str() << std::endl;

        if (key->getLevelOfDetail() != 0)
        {
            //Assumes the the TileSource will perform the caching for us when we call createImage
            for (TileSourceList::iterator itr = tile_builder->getImageSources().begin(); itr != tile_builder->getImageSources().end(); ++itr)
            {
                osg::ref_ptr<osg::Image> image = itr->get()->createImage(key);
            }

            for (TileSourceList::iterator itr = tile_builder->getHeightFieldSources().begin(); itr != tile_builder->getHeightFieldSources().end(); ++itr)
            {
                osg::ref_ptr<osg::HeightField> heightField = itr->get()->createHeightField(key);
            }
        }

        osg::ref_ptr<TileKey> k0 = key->getSubkey(0);
        osg::ref_ptr<TileKey> k1 = key->getSubkey(1);
        osg::ref_ptr<TileKey> k2;
        osg::ref_ptr<TileKey> k3;

        if (key->getLevelOfDetail() > 0 || dynamic_cast<MercatorTileKey*>(key))
        {
            k2 = key->getSubkey(2);
            k3 = key->getSubkey(3);
        }

        //Check to see if the bounds intersects ANY of the tile's children.  If it does, then process all of the children
        //for this level
        if ((k0.valid() && _bounds.intersects(k0.get())) ||
            (k1.valid() && _bounds.intersects(k1.get())) ||
            (k2.valid() && _bounds.intersects(k2.get())) ||
            (k3.valid() && _bounds.intersects(k3.get())))
        {
            if (k0.valid()) processKey(tile_builder, k0.get()); 
            if (k1.valid()) processKey(tile_builder, k1.get()); 
            if (k2.valid()) processKey(tile_builder, k2.get()); 
            if (k3.valid()) processKey(tile_builder, k3.get()); 
        }
    }
}