#include "LandCover"
#include "SplatCatalog"

#include <osgEarthSymbology/BillboardSymbol>
#include <osgEarthSymbology/BillboardResource>

using namespace osgEarth;
using namespace osgEarth::Splat;
using namespace osgEarth::Symbology;

#define LC "[LandCover] "


bool
LandCover::configure(const ConfigOptions& conf, const osgDB::Options* dbo)
{
    LandCoverOptions in( conf );
    
    if ( in.library().isSet() )
    {
        _lib = new ResourceLibrary( "default", in.library().get() );
        if ( !_lib->initialize( dbo ) )
        {
            OE_WARN << LC << "Failed to load resource library \"" << in.library()->full() << "\"\n";
            return false;
        }
    }

    if ( in.layers().empty() )
    {
        OE_WARN << LC << "No land cover layers defined; no land cover to render\n";
    }
    else
    {
        for(int i=0; i<in.layers().size(); ++i)
        {
            osg::ref_ptr<LandCoverLayer> layer = new LandCoverLayer();
            if ( layer->configure( in.layers().at(i), dbo ) )
            {
                _layers.push_back( layer.get() );
                OE_INFO << LC << "Configured land cover layer \"" << layer->getName() << "\"\n";
            }
        }
    }

    return true;
}

bool
LandCoverLayer::configure(const ConfigOptions& conf, const osgDB::Options* dbo)
{
    LandCoverLayerOptions in( conf );

    if ( in.name().isSet() )
        setName( in.name().get() );
    if ( in.lod().isSet() )
        setLOD( in.lod().get() );
    if ( in.maxDistance().isSet() )
        setMaxDistance( in.maxDistance().get() );
    if ( in.density().isSet() )
        setDensity( in.density().get() );
    if ( in.wind().isSet() )
        setWind( in.wind().get() );

    for(int i=0; i<in.biomes().size(); ++i)
    {
        osg::ref_ptr<LandCoverBiome> biome = new LandCoverBiome();
        if ( biome->configure( in.biomes().at(i), dbo ) )
        {
            _biomes.push_back( biome.get() );
        }
    }

    return true;
}

bool
LandCoverBiome::configure(const ConfigOptions& conf, const osgDB::Options* dbo)
{
    LandCoverBiomeOptions in( conf );

    if ( in.biomeClass().isSet() )
        setClass( in.biomeClass().get() );

    for(SymbolVector::const_iterator i = in.symbols().begin(); i != in.symbols().end(); ++i)
    {
        const BillboardSymbol* bs = dynamic_cast<BillboardSymbol*>( i->get() );
        if ( bs )
        {
            osg::Image* image = const_cast<osg::Image*>( bs->getImage() );
            if ( !image )
            {
                image = URI(bs->url()->eval()).getImage(dbo);
            }

            if ( image )
            {
                getBillboards().push_back( LandCoverBillboard(image, bs->width().get(), bs->height().get()) );
            }
            else
            {
                OE_WARN << LC << "Failed to load billboard image from \"" << bs->url() << "\"\n";
            }
        } 
        else
        {
            OE_WARN << LC << "Unrecognized symbol in land cover biome\n";
        }
    }

    return true;
}

