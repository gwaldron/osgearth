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
#include <osgEarth/BillboardSymbol>
#include <osgEarth/Registry>
#include <osgEarth/Math>

#include <osg/Texture2DArray>

using namespace osgEarth;
using namespace osgEarth::Splat;

#define LC "[GroundCover] "

//........................................................................

void
GroundCoverBiomeOptions::fromConfig(const Config& conf) 
{
    conf.get("classes", _biomeClasses);
    const ConfigSet& symbols = conf.children();
    for (ConfigSet::const_iterator i = symbols.begin(); i != symbols.end(); ++i) {
        Symbol* s = SymbolRegistry::instance()->create(*i);
        if (s) {
            _symbols.push_back(s);
        }
    }
    conf.get("fill", fill());
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
    conf.set("fill", fill());
    return conf;
}

//........................................................................

GroundCoverOptions::GroundCoverOptions(const ConfigOptions& co) :
    ConfigOptions(co),
    //_lod(14),
    _maxDistance(FLT_MAX),//1000.0f),
    _density(1.0f),
    _spacing(25.0f),
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
    conf.set("max_distance", _maxDistance);
    conf.set("density", _density);
    conf.set("spacing", _spacing);
    conf.set("fill", _fill);
    conf.set("wind", _wind);
    conf.set("brightness", _brightness);
    conf.set("contrast", _contrast);
    if (_biomes.size() > 0) {
        Config biomes("biomes");
        for (int i = 0; i < _biomes.size(); ++i) {
            biomes.add("biome", _biomes[i].getConfig());
        }
        conf.set(biomes);
    }
    return conf;
}

void
GroundCoverOptions::fromConfig(const Config& conf)
{
    conf.get("name", _name);
    conf.get("max_distance", _maxDistance);
    conf.get("density", _density);
    conf.get("spacing", _spacing);
    conf.get("fill", _fill);
    conf.get("wind", _wind);
    conf.get("brightness", _brightness);
    conf.get("contrast", _contrast);
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

    GroundCoverBiome::ImageCache cache;

    for (int i = 0; i<_biomes.size(); ++i)
    {
        if ( _biomes[i]->configure( options().biomes()[i], readOptions, cache ) == false )
        {
            OE_WARN << LC << "One of the biomes in layer \"" << getName() << "\" is improperly configured\n";
            return false;
        }
    }

    return true;
}

int
GroundCover::getTotalNumObjects() const
{
    int count = 0;
    for(int i=0; i<_biomes.size(); ++i)
    {
        count += _biomes[i]->getObjects().size();
    }
    return count;
}
int
GroundCover::getTotalNumModels() const
{
    int count = 0;
    for(int i=0; i<_biomes.size(); ++i)
    {
        for(int j=0; j<_biomes[i]->getObjects().size(); ++j)
        {
            const GroundCoverObject* obj = _biomes[i]->getObjects()[j].get();
            if (obj->getType() == GroundCoverObject::TYPE_MODEL)
                ++count;
        }
    }
    return count;
}

osg::StateSet*
GroundCover::getOrCreateStateSet()
{
    if ( !_stateSet.valid() )
    {
        _stateSet = new osg::StateSet();

        _stateSet->addUniform(new osg::Uniform("oe_GroundCover_wind", options().wind().get()));
    }

    return _stateSet.get();
}

#define SET_GET_UNIFORM(NAME, PROP, UNIFORM) \
    void GroundCover::set##NAME (float value) { getOrCreateStateSet()->getUniform(UNIFORM)->set(value); options(). PROP () = value; } \
    float GroundCover::get##NAME () const { return options(). PROP() .get(); }

SET_GET_UNIFORM(Wind, wind, "oe_GroundCover_wind")
SET_GET_UNIFORM(MaxDistance, maxDistance, "oe_GroundCover_maxDistance")

void GroundCover::setSpacing(float value) { options().spacing() = value; }
float GroundCover::getSpacing() const { return options().spacing().get(); }

void GroundCover::setFill(float value) { options().fill() = value; }
float GroundCover::getFill() const { return options().fill().get(); }

osg::Shader*
GroundCover::createShader() const
{
    std::stringstream biomeBuf;
    std::stringstream objectsBuf;
    std::stringstream billboardsBuf;

    int totalObjects = getTotalNumObjects();

    unsigned numBillboards = 0;

    // encode all the biome data.
    biomeBuf << 
        "struct oe_GroundCover_Biome { \n"
        "    int firstObjectIndex; \n"
        "    int numObjects; \n"
        "    float density; \n"
        "    float fill; \n"
        "    vec2 maxWidthHeight; \n"
        "}; \n"
        "const oe_GroundCover_Biome oe_GroundCover_biomes[" << getBiomes().size() << "] = oe_GroundCover_Biome[" << getBiomes().size() << "] ( \n";

    billboardsBuf <<
        "struct oe_GroundCover_Billboard { \n"
        "    int atlasIndexSide; \n"
        "    int atlasIndexTop; \n"
        "    float width; \n"
        "    float height; \n"
        "    float sizeVariation; \n"
        "}; \n"
        "const oe_GroundCover_Billboard oe_GroundCover_billboards[%NUM_BILLBOARDS%] = oe_GroundCover_Billboard[%NUM_BILLBOARDS%](\n";

    objectsBuf <<
        "struct oe_GroundCover_Object { \n"
        "    int type; // 0=billboard \n"
        "    int objectArrayIndex; // index into the type's object array \n"
        "}; \n"
        "const oe_GroundCover_Object oe_GroundCover_objects[%NUM_OBJECTS%] = oe_GroundCover_Object[%NUM_OBJECTS%](\n";

    // Since the texture array containing the billboard images contains each
    // unique image only once, we need to maps unique image to array indexes
    // here as well. This table maps each unique image to its texture array index.
    typedef std::map<osg::Image*, int> ImageSet;
    ImageSet uniqueImages;

    int nextAtlasIndex = 0;
    unsigned totalNumObjectsInserted = 0;

    for(int i=0; i<getBiomes().size(); ++i)
    {
        const GroundCoverBiome* biome = getBiomes()[i].get();

        float maxWidth = 0.0f, maxHeight = 0.0f;

        int firstObjectIndexOfBiome = totalNumObjectsInserted;

        // This will be larger than biome->getObjects().size() IF any of the
        // objects have a weight greater than 1.
        unsigned numObjectsInsertedInBiome = 0;

        for(int j=0; j<biome->getObjects().size(); ++j)
        {
            const GroundCoverObject* object = biome->getObjects()[j].get();

            if (object->getType() == GroundCoverObject::TYPE_BILLBOARD)
            {
                const GroundCoverBillboard* bb = dynamic_cast<const GroundCoverBillboard*>(object);

                // If we already used this image, use its original tex array index; 
                // otherwise use the next available index.
                int atlasSideIndex = -1;
                if (bb->_sideImage.valid())
                {
                    ImageSet::iterator u = uniqueImages.find(bb->_sideImage.get());
                    if (u != uniqueImages.end())
                    {
                        atlasSideIndex = u->second;
                    }
                    else
                    {
                        atlasSideIndex = nextAtlasIndex++;
                        uniqueImages[bb->_sideImage.get()] = atlasSideIndex;
                    }
                }

                int atlasTopIndex = -1;
                if (bb->_topImage.valid())
                {
                    ImageSet::iterator u = uniqueImages.find(bb->_topImage.get());
                    if (u != uniqueImages.end())
                    {
                        atlasTopIndex = u->second;
                    }
                    else
                    {
                        atlasTopIndex = nextAtlasIndex++;
                        uniqueImages[bb->_topImage.get()] = atlasTopIndex;
                    }
                }

                if (numBillboards > 0)
                    billboardsBuf << ", \n";

                const BillboardSymbol* symbol = bb->_symbol.get();

                billboardsBuf
                    << "    oe_GroundCover_Billboard("
                    << atlasSideIndex << ", " << atlasTopIndex
                    << ", float(" << symbol->width().get() << ")"
                    << ", float(" << symbol->height().get() << ")"
                    << ", float(" << symbol->sizeVariation().get() << ")"
                    << ")";

                maxWidth = osg::maximum(
                    maxWidth, 
                    symbol->width().get() + (symbol->width().get()*symbol->sizeVariation().get()));

                maxHeight = osg::maximum(
                    maxHeight, 
                    symbol->height().get() + (symbol->height().get()*symbol->sizeVariation().get()));

                // apply the selection weight by adding the object multiple times
                unsigned weight = osg::maximum(symbol->selectionWeight().get(), 1u);
                for(unsigned w=0; w<weight; ++w)
                {
                    objectsBuf 
                        << (totalNumObjectsInserted>0?",\n":"")
                        << "   oe_GroundCover_Object("
                        << (int)object->getType() << ", "
                        << (int)numBillboards
                        << ")";
                    ++numObjectsInsertedInBiome;
                    ++totalNumObjectsInserted;
                }

                ++numBillboards;
            }
        }

        // We multiply the height x 2 below because billboards have their origin
        // at the bottom center of the image. The GPU culling code considers the
        // distance from this anchor point. For width, it's in the middle, but for
        // height, it's at the bottom so we need to double it. It doubles in both
        // directions, but that's OK since we are rarely if ever going to GPU-cull
        // a billboard at the top of the viewport. -gw

        float fill = biome->fill().isSet() ? biome->fill().get() : options().fill().get();

        biomeBuf << "    oe_GroundCover_Biome("
            << firstObjectIndexOfBiome << ", "
            << numObjectsInsertedInBiome //<< biome->getObjects().size() 
            << ", float(" << options().density().get() << ")"
            << ", float(" << fill << ")"
            << ", vec2(float(" << maxWidth << "),float(" << maxHeight*2.0f << ")))";

        if ( (i+1) < getBiomes().size() )
            biomeBuf << ",\n";
    }

    if (totalNumObjectsInserted == 0)
    {
        OE_WARN << LC << "Shader creation failed; no valid groundcover billboards" << std::endl;
        return NULL;
    }

    biomeBuf
        << "\n);\n";

    billboardsBuf
        << "\n); \n";

    objectsBuf
        << "\n); \n";

    biomeBuf 
        << "void oe_GroundCover_getBiome(in int index, out oe_GroundCover_Biome biome) { \n"
        << "    biome = oe_GroundCover_biomes[index]; \n"
        << "} \n";

    objectsBuf
        << "void oe_GroundCover_getObject(in int index, out oe_GroundCover_Object object) { \n"
        << "    object = oe_GroundCover_objects[index]; \n"
        << "} \n";

    billboardsBuf
        << "void oe_GroundCover_getBillboard(in int index, out oe_GroundCover_Billboard billboard) { \n"
        << "    billboard = oe_GroundCover_billboards[index]; \n"
        << "} \n";

    std::string biomeStr = biomeBuf.str();

    std::string billboardsStr = billboardsBuf.str();
    replaceIn(billboardsStr, "%NUM_BILLBOARDS%", Stringify()<<numBillboards);

    std::string objectsStr = objectsBuf.str();
    replaceIn(objectsStr, "%NUM_OBJECTS%", Stringify() << totalNumObjectsInserted); //getTotalNumObjects());

    osg::ref_ptr<ImageLayer> layer;

    osg::Shader* shader = new osg::Shader();
    shader->setName( "GroundCover lookup tables" );
    shader->setShaderSource( Stringify() 
        << "#version " GLSL_VERSION_STR "\n"
        << biomeStr << "\n"
        << objectsStr << "\n"
        << billboardsStr << "\n" 
    );
    return shader;
}

const GroundCoverBiome*
GroundCover::getBiome(const LandCoverClass* lcClass) const
{
    for (int biomeIndex = 0; biomeIndex < getBiomes().size(); ++biomeIndex)
    {
        const GroundCoverBiome* biome = getBiomes()[biomeIndex].get();
        if (!biome->getClasses().empty())
        {
            StringVector classes;
            StringTokenizer(biome->getClasses(), classes, " ", "\"", false);

            for (int i = 0; i < classes.size(); ++i)
            {
                if (classes[i] == lcClass->getName())
                {
                    return biome;
                }
            }
        }
    }
    return NULL;
}

osg::Shader*
GroundCover::createPredicateShader(LandCoverDictionary* landCoverDict, LandCoverLayer* layer) const
{
    const char* defaultCode = "int oe_GroundCover_getBiomeIndex(in vec4 coords) { return -1; }\n";

    std::stringstream buf;
    buf << "#version " GLSL_VERSION_STR "\n";

    if ( !landCoverDict )
    {
        buf << defaultCode;
        OE_WARN << LC << "No land cover dictionary; generating default coverage predicate\n";
    }
    //else if ( !layer )
    //{
    //    buf << defaultCode;
    //    OE_WARN << LC << "No classification layer; generating default coverage predicate\n";
    //}
    else
    {
        buf << "int oe_GroundCover_getBiomeIndex(in float code) { \n";

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
                        buf << "    if (code == " << lcClass->getValue() << ") return " << biomeIndex << "; \n";
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
    // Creates a texture array containing all the billboard images.
    // Each image is included only once.
    osg::Texture2DArray* tex = new osg::Texture2DArray();

    int arrayIndex = 0;
    float s = -1.0f, t = -1.0f;

    typedef std::set<osg::Image*> ImageSet;
    typedef std::vector<osg::Image*> ImageVector;
    ImageSet uniqueImages;
    ImageVector imagesToAdd;


    for(int b=0; b<getBiomes().size(); ++b)
    {
        const GroundCoverBiome* biome = getBiomes()[b].get();

        for(int i=0; i<biome->getObjects().size(); ++i)
        {
            const GroundCoverObject* obj = biome->getObjects()[i].get();
            if (obj->getType() == obj->TYPE_BILLBOARD)
            {
                const GroundCoverBillboard* bb = dynamic_cast<const GroundCoverBillboard*>(obj);

                if (bb->_sideImage.valid() && uniqueImages.find(bb->_sideImage.get()) == uniqueImages.end())
                {
                    imagesToAdd.push_back(bb->_sideImage.get());
                    uniqueImages.insert(bb->_sideImage.get());
                }

                if (bb->_topImage.valid() && uniqueImages.find(bb->_topImage.get()) == uniqueImages.end())
                {
                    imagesToAdd.push_back(bb->_topImage.get());
                    uniqueImages.insert(bb->_topImage.get());
                }
            }
        }
    }

    for(unsigned i=0; i<imagesToAdd.size(); ++i)
    {
        osg::Image* image = imagesToAdd[i];
        osg::ref_ptr<osg::Image> im;

        // make sure the texture array is POT - required now for mipmapping to work
        if ( s < 0 )
        {
            s  = nextPowerOf2(image->s());
            t  = nextPowerOf2(image->t());
            tex->setTextureSize(s, t, imagesToAdd.size());
        }

        if ( image->s() != s || image->t() != t )
        {
            ImageUtils::resizeImage( image, s, t, im );
        }
        else
        {
            im = image;
        }

        tex->setImage( i, im.get() );
    }

    OE_INFO << LC << "Created texture with " << imagesToAdd.size() << " unique images" << std::endl; 

    tex->setFilter(tex->MIN_FILTER, tex->NEAREST_MIPMAP_LINEAR);
    tex->setFilter(tex->MAG_FILTER, tex->LINEAR);
    tex->setWrap  (tex->WRAP_S, tex->CLAMP_TO_EDGE);
    tex->setWrap  (tex->WRAP_T, tex->CLAMP_TO_EDGE);
    tex->setUnRefImageDataAfterApply(Registry::instance()->unRefImageDataAfterApply().get());
    tex->setMaxAnisotropy( 4.0 );

    // Let the GPU do it since we only download this at startup
    //ImageUtils::generateMipmaps(tex);
    tex->setUseHardwareMipMapGeneration(true);

    return tex;
}

void
GroundCover::resizeGLObjectBuffers(unsigned maxSize)
{
    if (getStateSet())
        getStateSet()->resizeGLObjectBuffers(maxSize);
}

void
GroundCover::releaseGLObjects(osg::State* state) const
{
    if (getStateSet())
        getStateSet()->releaseGLObjects(state);
}

//............................................................................

bool
GroundCoverBiome::configure(const ConfigOptions& conf, const osgDB::Options* dbo, ImageCache& cache)
{
    GroundCoverBiomeOptions in( conf );

    if ( in.biomeClasses().isSet() )
        setClasses( in.biomeClasses().get() );

    if (in.fill().isSet())
        fill() = in.fill().get();

    for(SymbolVector::const_iterator i = in.symbols().begin(); i != in.symbols().end(); ++i)
    {
        const BillboardSymbol* bs = dynamic_cast<BillboardSymbol*>( i->get() );
        if ( bs )
        {
            // Start with the side image, which is mandatory:
            osg::ref_ptr<osg::Image> sideImage;

            // Symbol contains an image instance?
            if (bs->getSideImage())
            {
                sideImage = const_cast<osg::Image*>(bs->getSideImage());
            }
            else if (bs->url().isSet())
            {
                // Load the image from the URI:
                URI imageURI = bs->url()->evalURI();

                // If image was already loaded into a biome, share it:
                ImageCache::iterator ic = cache.find(imageURI);
                if (ic != cache.end())
                {
                    sideImage = ic->second.get();
                }

                if (!sideImage.valid())
                {
                    sideImage = imageURI.getImage(dbo);
                    if (sideImage.valid())
                    {
                        cache[imageURI] = sideImage.get();
                    }
                    else
                    {
                        OE_WARN << LC << "Failed to load billboard image from \"" << imageURI.full() << "\"" << std::endl;
                    }
                }
            }

            if (!sideImage.valid())
            {
                OE_WARN << LC << "A billboard is missing the mandatory image" << std::endl;
                //return false;
                sideImage = new osg::Image();
            }

            // Next process the top image (optional)
            // Check for an actual instance in the symbol:
            osg::ref_ptr<osg::Image> topImage;

            if (bs->getTopImage())
            {
                topImage = const_cast<osg::Image*>(bs->getTopImage());
            }
            else if (bs->topURL().isSet())
            {
                // Load the image from the URI:
                URI imageURI = bs->topURL()->evalURI();

                // If image was already loaded into a biome, share it:
                ImageCache::iterator ic = cache.find(imageURI);
                if (ic != cache.end())
                {
                    topImage = ic->second.get();
                }

                if (!topImage.valid())
                {
                    topImage = imageURI.getImage(dbo);
                    if (topImage.valid())
                    {
                        cache[imageURI] = topImage.get();
                    }
                    else
                    {
                        OE_WARN << LC << "Failed to load billboard top image from \"" << imageURI.full() << "\"\n";
                    }
                }
            }


            if ( sideImage.valid() )
            {
                getObjects().push_back( new GroundCoverBillboard(sideImage.get(), topImage.get(), bs) );
            }

            continue;
        } 

        const ModelSymbol* model = dynamic_cast<const ModelSymbol*>(i->get());
        if (model)
        {
            osg::ref_ptr<osg::Node> node = URI(model->url()->evalURI()).getNode(dbo);
            getObjects().push_back(new GroundCoverModel(node.release()));
        }
    }

    if ( getObjects().size() == 0 )
    {
        OE_WARN << LC << "A biome failed to install any billboards.\n";
        return false;
    }

    return true;
}
