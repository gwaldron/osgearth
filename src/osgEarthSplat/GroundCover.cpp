#include "GroundCover"
#include "Coverage"
#include "SplatCatalog"
#include "SplatCoverageLegend"

#include <osgEarth/Map>
#include <osgEarth/ImageLayer>
#include <osgEarth/ImageUtils>
#include <osgEarthSymbology/BillboardSymbol>
#include <osgEarthSymbology/BillboardResource>

#include <osg/Texture2DArray>

using namespace osgEarth;
using namespace osgEarth::Splat;
using namespace osgEarth::Symbology;

#define LC "[GroundCover] "

bool
GroundCover::configure(const ConfigOptions& conf, const Map* map, const osgDB::Options* dbo)
{
    GroundCoverOptions in( conf );
    
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
        OE_WARN << LC << "No land cover layers defined\n";
        return false;
    }
    else
    {
        for(int i=0; i<in.layers().size(); ++i)
        {
            osg::ref_ptr<GroundCoverLayer> layer = new GroundCoverLayer();

            if ( layer->configure( in.layers()[i], dbo ) )
            {
                _layers.push_back( layer.get() );
                OE_INFO << LC << "Configured land cover layer \"" << layer->getName() << "\"\n";
            }
            else
            {
                OE_WARN << LC << "Land cover layer \"" << layer->getName() << "\" is improperly configured\n";
                return false;
            }
        }
    }

    if ( in.maskLayerName().isSet() )
    {
        ImageLayer* layer = map->getLayerByName<ImageLayer>( in.maskLayerName().get() );
        if ( layer )
        {
            setMaskLayer( layer );
        }
        else
        {
            OE_WARN << LC << "Masking layer \"" << in.maskLayerName().get() << "\" not found in the map." << std::endl;
            return false;
        }
    }

    return true;
}


//............................................................................

bool
GroundCoverLayer::configure(const ConfigOptions& conf, const osgDB::Options* dbo)
{
    GroundCoverLayerOptions in( conf );

    if ( in.name().isSet() )
        setName( in.name().get() );
    if ( in.lod().isSet() )
        setLOD( in.lod().get() );
    if ( in.castShadows().isSet() )
        setCastShadows( in.castShadows().get() );
    if ( in.maxDistance().isSet() )
        setMaxDistance( in.maxDistance().get() );
    if ( in.density().isSet() )
        setDensity( in.density().get() );
    if ( in.fill().isSet() )
        setFill( in.fill().get() );
    if ( in.wind().isSet() )
        setWind( in.wind().get() );
    if ( in.brightness().isSet() )
        setBrightness( in.brightness().get() );
    if ( in.contrast().isSet() )
        setContrast( in.contrast().get() );

    if ( in.biomes().size() == 0 )
    {
        OE_WARN << LC << "No biomes defined in layer \"" << getName() << "\"\n";
        return false;
    }

    for(int i=0; i<in.biomes().size(); ++i)
    {
        osg::ref_ptr<GroundCoverBiome> biome = new GroundCoverBiome();

        if ( biome->configure( in.biomes()[i], dbo ) )
        {
            _biomes.push_back( biome.get() );
        }
        else
        {
            OE_WARN << LC << "One of the biomes in layer \"" << getName() << "\" is improperly configured\n";
            return false;
        }
    }

    return true;
}

int
GroundCoverLayer::getTotalNumBillboards() const
{
    int count = 0;
    for(int i=0; i<getBiomes().size(); ++i)
    {
        count += getBiomes()[i]->getBillboards().size();
    }
    return count;
}

osg::StateSet*
GroundCoverLayer::getOrCreateStateSet()
{
    if ( !_stateSet.valid() )
        _stateSet = new osg::StateSet();
    return _stateSet.get();
}

osg::Shader*
GroundCoverLayer::createShader() const
{
    std::stringstream biomeBuf;
    std::stringstream billboardBuf;

    int totalBillboards = getTotalNumBillboards();

    // encode all the biome data.
    biomeBuf << 
        "struct oe_GroundCover_Biome { \n"
        "    int firstBillboardIndex; \n"
        "    int numBillboards; \n"
        "    float density; \n"
        "    float fill; \n"
        "    vec2 maxWidthHeight; \n"
        "}; \n"

        "const oe_GroundCover_Biome oe_GroundCover_biomes[" << getBiomes().size() << "] = oe_GroundCover_Biome[" << getBiomes().size() << "] ( \n";

    billboardBuf <<
        "struct oe_GroundCover_Billboard { \n"
        "    int arrayIndex; \n"
        "    float width; \n"
        "    float height; \n"
        "}; \n"

        "const oe_GroundCover_Billboard oe_GroundCover_billboards[" << totalBillboards << "] = oe_GroundCover_Billboard[" << totalBillboards << "](\n";
    
    int index = 0;
    for(int i=0; i<getBiomes().size(); ++i)
    {
        const GroundCoverBiome* biome = getBiomes()[i].get();

        float maxWidth = 0.0f, maxHeight = 0.0f;
        
        int firstIndex = index;

        for(int j=0; j<biome->getBillboards().size(); ++j)
        {
            const GroundCoverBillboard& bb = biome->getBillboards()[j];

            billboardBuf
                << "    oe_GroundCover_Billboard("
                << index 
                << ", float(" << bb._width << ")"
                << ", float(" << bb._height << ")"
                << ")";
            
            ++index;
            if ( index < totalBillboards )
                billboardBuf << ",\n";

            maxWidth = std::max(maxWidth, bb._width);
            maxHeight = std::max(maxHeight, bb._height);
        }

        // We multiply the height x 2 below because billboards have their origin
        // at the bottom center of the image. The GPU culling code considers the
        // distance from this anchor point. For width, it's in the middle, but for
        // height, it's at the bottom so we need to double it. It doubles in both
        // directions, but that's OK since we are rarely if ever going to GPU-cull
        // a billboard at the top of the viewport. -gw

        biomeBuf << "    oe_GroundCover_Biome(" 
            << firstIndex << ", "
            << biome->getBillboards().size() 
            << ", float(" << getDensity() << ")"
            << ", float(" << getFill() << ")"
            << ", vec2(float(" << maxWidth << "),float(" << maxHeight*2.0f << ")))";

        if ( (i+1) < getBiomes().size() )
            biomeBuf << ",\n";
    }

    biomeBuf
        << "\n);\n";

    billboardBuf
        << "\n); \n";

    biomeBuf 
        << "void oe_GroundCover_getBiome(in int biomeIndex, out oe_GroundCover_Biome biome) { \n"
        << "    biome = oe_GroundCover_biomes[biomeIndex]; \n"
        << "} \n";
        
    billboardBuf
        << "void oe_GroundCover_getBillboard(in int billboardIndex, out oe_GroundCover_Billboard billboard) { \n"
        << "    billboard = oe_GroundCover_billboards[billboardIndex]; \n"
        << "} \n";
    
    osg::ref_ptr<ImageLayer> layer;

    osg::Shader* shader = new osg::Shader();
    shader->setName( "GroundCoverLayer" );
    shader->setShaderSource( Stringify() << "#version 330\n" << biomeBuf.str() << "\n" << billboardBuf.str() );
    return shader;
}

osg::Shader*
GroundCoverLayer::createPredicateShader(const Coverage* coverage) const
{
    const char* defaultCode = "int oe_GroundCover_getBiomeIndex(in vec4 coords) { return -1; }\n";

    std::stringstream buf;
    buf << "#version 330\n";
    
    osg::ref_ptr<ImageLayer> layer;

    if ( !coverage )
    {
        buf << defaultCode;
        OE_INFO << LC << "No coverage; generating default coverage predicate\n";
    }
    else if ( !coverage->getLegend() )
    {
        buf << defaultCode;
        OE_INFO << LC << "No legend; generating default coverage predicate\n";
    }
    else if ( !coverage->lockLayer(layer) )
    {
        buf << defaultCode;
        OE_INFO << LC << "No classification layer; generating default coverage predicate\n";
    }
    else
    {
        const std::string& sampler = layer->shareTexUniformName().get();
        const std::string& matrix  = layer->shareTexMatUniformName().get();

        buf << "uniform sampler2D " << sampler << ";\n"
            << "uniform mat4 " << matrix << ";\n"
            << "int oe_GroundCover_getBiomeIndex(in vec4 coords) { \n"
            << "    float value = textureLod(" << sampler << ", (" << matrix << " * coords).st, 0).r;\n";

        for(int biomeIndex=0; biomeIndex<getBiomes().size(); ++biomeIndex)
        {
            const GroundCoverBiome* biome = getBiomes()[biomeIndex].get();

            if ( !biome->getClasses().empty() )
            {
                StringVector classes;
                StringTokenizer(biome->getClasses(), classes, " ", "\"", false);

                for(int i=0; i<classes.size(); ++i)
                {
                    std::vector<const CoverageValuePredicate*> predicates;
                    if ( coverage->getLegend()->getPredicatesForClass(classes[i], predicates) )
                    {
                        for(std::vector<const CoverageValuePredicate*>::const_iterator p = predicates.begin();
                            p != predicates.end(); 
                            ++p)
                        {
                            const CoverageValuePredicate* predicate = *p;

                            if ( predicate->_exactValue.isSet() )
                            {
                                buf << "    if (value == " << predicate->_exactValue.get() << ") return " << biomeIndex << "; \n";
                            }
                            else if ( predicate->_minValue.isSet() && predicate->_maxValue.isSet() )
                            {
                                buf << "    if (value >= " << predicate->_minValue.get() << " && value <= " << predicate->_maxValue.get() << ") return " << biomeIndex << "; \n";
                            }
                            else if ( predicate->_minValue.isSet() )
                            {
                                buf << "    if (value >= " << predicate->_minValue.get() << ")  return " << biomeIndex << "; \n";
                            }
                            else if ( predicate->_maxValue.isSet() )
                            {
                                buf << "    if (value <= " << predicate->_maxValue.get() << ") return " << biomeIndex << "; \n";
                            }

                            else 
                            {
                                OE_WARN << LC << "Class \"" << classes[i] << "\" found, but no exact/min/max value was set in the legend\n";
                            }
                        }
                    }
                    else
                    {
                        OE_WARN << LC << "Class \"" << classes[i] << "\" not found in the legend!\n";
                    }
                }
            }
        }
        buf << "    return -1; \n";
        buf << "}\n";
    }
    
    osg::Shader* shader = new osg::Shader();
    shader->setName("oe GroundCover predicate function");
    shader->setShaderSource( buf.str() );

    return shader;
}

//............................................................................

bool
GroundCoverBiome::configure(const ConfigOptions& conf, const osgDB::Options* dbo)
{
    GroundCoverBiomeOptions in( conf );

    if ( in.biomeClasses().isSet() )
        setClasses( in.biomeClasses().get() );

    for(SymbolVector::const_iterator i = in.symbols().begin(); i != in.symbols().end(); ++i)
    {
        const BillboardSymbol* bs = dynamic_cast<BillboardSymbol*>( i->get() );
        if ( bs )
        {
            URI imageURI = bs->url()->evalURI();

            osg::Image* image = const_cast<osg::Image*>( bs->getImage() );
            if ( !image )
            {
                image = imageURI.getImage(dbo);
            }

            if ( image )
            {
                getBillboards().push_back( GroundCoverBillboard(image, bs->width().get(), bs->height().get()) );
            }
            else
            {
                OE_WARN << LC << "Failed to load billboard image from \"" << imageURI.full() << "\"\n";
            }
        } 
        else
        {
            OE_WARN << LC << "Unrecognized symbol in land cover biome\n";
        }
    }

    if ( getBillboards().size() == 0 )
    {
        OE_WARN << LC << "A biome failed to install any billboards.\n";
        return false;
    }

    return true;
}

osg::Texture*
GroundCoverLayer::createTexture() const
{
    osg::Texture2DArray* tex = new osg::Texture2DArray();

    int arrayIndex = 0;
    float s = -1.0f, t = -1.0f;

    for(int b=0; b<getBiomes().size(); ++b)
    {
        const GroundCoverBiome* biome = getBiomes()[b];

        for(int i=0; i<biome->getBillboards().size(); ++i, ++arrayIndex)
        {
            const GroundCoverBillboard& bb = biome->getBillboards()[i];

            osg::ref_ptr<osg::Image> im;

            if ( s < 0 )
            {
                s  = bb._image->s();
                t  = bb._image->t();
                im = bb._image.get();
                tex->setTextureSize(s, t, getTotalNumBillboards());                              
            }
            else
            {
                if ( bb._image->s() != s || bb._image->t() != t )
                {
                    ImageUtils::resizeImage( bb._image.get(), s, t, im );
                }
                else
                {
                    im = bb._image.get();
                }
            }

            tex->setImage( arrayIndex, im.get() );
        }
    }

    tex->setFilter(tex->MIN_FILTER, tex->NEAREST_MIPMAP_LINEAR);
    tex->setFilter(tex->MAG_FILTER, tex->LINEAR);
    tex->setWrap  (tex->WRAP_S, tex->CLAMP_TO_EDGE);
    tex->setWrap  (tex->WRAP_T, tex->CLAMP_TO_EDGE);
    tex->setUnRefImageDataAfterApply( true );
    tex->setMaxAnisotropy( 4.0 );
    tex->setResizeNonPowerOfTwoHint( false );

    return tex;
}
