#include "LandCover"
#include "Coverage"
#include "SplatCatalog"
#include "SplatCoverageLegend"

#include <osgEarth/ImageLayer>
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

//............................................................................

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

//............................................................................

bool
LandCoverBiome::configure(const ConfigOptions& conf, const osgDB::Options* dbo)
{
    LandCoverBiomeOptions in( conf );

    if ( in.biomeClasses().isSet() )
        setClasses( in.biomeClasses().get() );

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
                OE_WARN << LC << "Failed to load billboard image from \"" << bs->url()->eval() << "\"\n";
            }
        } 
        else
        {
            OE_WARN << LC << "Unrecognized symbol in land cover biome\n";
        }
    }

    return true;
}

osg::Shader*
LandCoverBiome::createPredicateShader(const Coverage* coverage, osg::Shader::Type type) const
{
    const char* defaultCode = "bool oe_landcover_passesCoverage(in vec4 coords) { return true; }\n";

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
            << "bool oe_landcover_passesCoverage(in vec4 coords) { \n";
    
        if ( !getClasses().empty() )
        {
            buf << "    float value = textureLod(" << sampler << ", (" << matrix << " * coords).st, 0).r;\n";

            StringVector classes;
            StringTokenizer(getClasses(), classes, " ", "\"", false);

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
                            buf << "    if (value == " << predicate->_exactValue.get() << ") return true;\n";
                        }
                        else if ( predicate->_minValue.isSet() && predicate->_maxValue.isSet() )
                        {
                            buf << "    if (value >= " << predicate->_minValue.get() << " && value <= " << predicate->_maxValue.get() << ") return true;\n";
                        }
                        else if ( predicate->_minValue.isSet() )
                        {
                            buf << "    if (value >= " << predicate->_minValue.get() << ") return true;\n";
                        }
                        else if ( predicate->_maxValue.isSet() )
                        {
                            buf << "    if (value <= " << predicate->_maxValue.get() << ") return true;\n";
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

            buf << "    return false; \n";
        }

        else
        {
            // no classes defined; accept all.
            buf << "    return true;\n";
        }

        buf << "}\n";
    }
    
    osg::Shader* shader = new osg::Shader(type);
    shader->setName("oe Landcover predicate function");
    shader->setShaderSource( buf.str() );

    return shader;
}
