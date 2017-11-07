#include "GroundCover"
#include "Coverage"
#include "SplatCatalog"
#include "SplatCoverageLegend"
#include "Zone"
#include "SplatShaders"

#include <osgEarth/Map>
#include <osgEarth/ImageLayer>
#include <osgEarth/ImageUtils>
#include <osgEarth/VirtualProgram>
#include <osgEarthSymbology/BillboardSymbol>

#include <osg/Texture2DArray>

using namespace osgEarth;
using namespace osgEarth::Splat;
using namespace osgEarth::Symbology;

#define LC "[GroundCover] "

//........................................................................

void
GroundCoverBiomeOptions::fromConfig(const Config& conf) 
{
    conf.getIfSet("classes", _biomeClasses);
    const ConfigSet& symbols = conf.children();
    for (ConfigSet::const_iterator i = symbols.begin(); i != symbols.end(); ++i) {
        Symbol* s = SymbolRegistry::instance()->create(*i);
        if (s) {
            _symbols.push_back(s);
        }
    }
}

Config
GroundCoverBiomeOptions::getConfig() const 
{
    Config conf("biome");
    conf.set("classes", _biomeClasses);
    for (int i = 0; i < _symbols.size(); ++i) {
        Config symbolConf = _symbols[i]->getConfig();
        if (!symbolConf.empty()) {
            conf.add(symbolConf);
        }
    }
    return conf;
}

//........................................................................

GroundCoverOptions::GroundCoverOptions(const ConfigOptions& co) :
ConfigOptions(co),
_lod(14),
_maxDistance(1000.0f),
_density(1.0f),
_fill(1.0f),
_wind(0.0f),
_brightness(1.0f),
_contrast(0.5f)
{
    fromConfig(_conf);
}

Config
GroundCoverOptions::getConfig() const
{
    Config conf = ConfigOptions::getConfig();
    conf.key() = "groundcover";
    conf.set("name", _name);
    conf.set("lod", _lod);
    conf.set("max_distance", _maxDistance);
    conf.set("density", _density);
    conf.set("fill", _fill);
    conf.set("wind", _wind);
    conf.set("brightness", _brightness);
    conf.set("contrast", _contrast);
    if (_biomes.size() > 0) {
        Config biomes("biomes");
        for (int i = 0; i < _biomes.size(); ++i) {
            biomes.add("biome", _biomes[i].getConfig());
        }
        conf.update(biomes);
    }
    return conf;
}

void
GroundCoverOptions::fromConfig(const Config& conf)
{
    conf.getIfSet("name", _name);
    conf.getIfSet("lod", _lod);
    conf.getIfSet("max_distance", _maxDistance);
    conf.getIfSet("density", _density);
    conf.getIfSet("fill", _fill);
    conf.getIfSet("wind", _wind);
    conf.getIfSet("brightness", _brightness);
    conf.getIfSet("contrast", _contrast);
    const Config* biomes = conf.child_ptr("biomes");
    if (biomes) {
        const ConfigSet& biomesVec = biomes->children();
        for (ConfigSet::const_iterator i = biomesVec.begin(); i != biomesVec.end(); ++i) {
            _biomes.push_back(GroundCoverBiomeOptions(*i));
        }
    }
}

//............................................................................

GroundCover::GroundCover(const GroundCoverOptions& in) :
_options(in)
{
    //nop
}

bool
GroundCover::configure(const osgDB::Options* readOptions)
{
    if ( options().biomes().size() == 0 )
    {
        OE_WARN << LC << "No biomes defined in layer \"" << getName() << "\"\n";
        return false;
    }

    for(int i=0; i<_options.biomes().size(); ++i)
    {
        osg::ref_ptr<GroundCoverBiome> biome = new GroundCoverBiome();
        _biomes.push_back( biome.get() );
    }

    for (int i = 0; i<_biomes.size(); ++i)
    {
        if ( _biomes[i]->configure( options().biomes()[i], readOptions ) == false )
        {
            OE_WARN << LC << "One of the biomes in layer \"" << getName() << "\" is improperly configured\n";
            return false;
        }
    }

    return true;
}

int
GroundCover::getTotalNumBillboards() const
{
    int count = 0;
    for(int i=0; i<_biomes.size(); ++i)
    {
        count += _biomes[i]->getBillboards().size();
    }
    return count;
}

osg::StateSet*
GroundCover::getOrCreateStateSet()
{
    if ( !_stateSet.valid() )
    {
        _stateSet = new osg::StateSet();

        _stateSet->addUniform(new osg::Uniform("oe_GroundCover_windFactor", options().wind().get()));
        _stateSet->addUniform(new osg::Uniform("oe_GroundCover_noise", 1.0f));
        _stateSet->addUniform(new osg::Uniform("oe_GroundCover_ao", 0.5f));
        _stateSet->addUniform(new osg::Uniform("oe_GroundCover_exposure", 1.0f));

        _stateSet->addUniform(new osg::Uniform("oe_GroundCover_density", options().density().get()));
        _stateSet->addUniform(new osg::Uniform("oe_GroundCover_fill", options().fill().get()));
        _stateSet->addUniform(new osg::Uniform("oe_GroundCover_maxDistance", options().maxDistance().get()));

        _stateSet->addUniform(new osg::Uniform("oe_GroundCover_brightness", options().brightness().get()));
        _stateSet->addUniform(new osg::Uniform("oe_GroundCover_contrast", options().contrast().get()));
    }

    return _stateSet.get();
}

#define SET_GET_UNIFORM(NAME, UNIFORM) \
    void GroundCover::set##NAME (float value) { getOrCreateStateSet()->getUniform(UNIFORM)->set(value); } \
    float GroundCover::get##NAME () const { float value = 0.0f; if (getStateSet()) getStateSet()->getUniform(UNIFORM)->get(value); return value; }

SET_GET_UNIFORM(Wind, "oe_GroundCover_windFactor")
SET_GET_UNIFORM(Density, "oe_GroundCover_density")
SET_GET_UNIFORM(Fill, "oe_GroundCover_fill")
SET_GET_UNIFORM(MaxDistance, "oe_GroundCover_maxDistance")
SET_GET_UNIFORM(Brightness, "oe_GroundCover_brightness")
SET_GET_UNIFORM(Contrast, "oe_GroundCover_contrast")


osg::Shader*
GroundCover::createShader() const
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
            << ", float(" << options().density().get() << ")"
            << ", float(" << options().fill().get() << ")"
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
GroundCover::createPredicateShader(LandCoverDictionary* landCoverDict, LandCoverLayer* layer) const
{
    const char* defaultCode = "int oe_GroundCover_getBiomeIndex(in vec4 coords) { return -1; }\n";

    std::stringstream buf;
    buf << "#version 330\n";

        if ( !landCoverDict )
    {
        buf << defaultCode;
        OE_WARN << LC << "No land cover dictionary; generating default coverage predicate\n";
    }
    else if ( !layer )
    {
        buf << defaultCode;
        OE_WARN << LC << "No classification layer; generating default coverage predicate\n";
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
                    const LandCoverClass* lcClass = landCoverDict->getClassByName(classes[i]);
                    if (lcClass)
                    {
                        buf << "    if (value == " << lcClass->getValue() << ") return " << biomeIndex << "; \n";
                    }
                    else
                    {
                        OE_WARN << LC << "Land cover class \"" << classes[i] << "\" was not found in the dictionary!\n";
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

osg::Texture*
GroundCover::createTexture() const
{
    osg::Texture2DArray* tex = new osg::Texture2DArray();

    int arrayIndex = 0;
    float s = -1.0f, t = -1.0f;

    for(int b=0; b<getBiomes().size(); ++b)
    {
        const GroundCoverBiome* biome = getBiomes()[b].get();

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
