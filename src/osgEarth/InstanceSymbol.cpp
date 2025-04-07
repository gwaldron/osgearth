/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/InstanceSymbol>
#include <osgEarth/IconSymbol>
#include <osgEarth/ModelSymbol>

using namespace osgEarth;

InstanceSymbol::InstanceSymbol( const Config& conf ) :
TaggableWithConfig<Symbol>(conf),
_placement ( PLACEMENT_VERTEX ),
_density   ( 25.0f ),
_randomSeed( 0 ),
_scale     ( NumericExpression(1.0) )
{
    mergeConfig( conf );
}

InstanceSymbol::InstanceSymbol(const InstanceSymbol& rhs,const osg::CopyOp& copyop):
TaggableWithConfig<Symbol>(rhs, copyop),
_url(rhs._url),
_library(rhs._library),
_scale(rhs._scale),
_placement(rhs._placement),
_density(rhs._density),
_randomSeed(rhs._randomSeed),
_uriAliasMap(rhs._uriAliasMap),
_script(rhs._script)
{
}

Config 
InstanceSymbol::getConfig() const
{
    Config conf = TaggableWithConfig<Symbol>::getConfig();
    conf.key() = "instance";
    conf.set( "url", _url );
    conf.set( "library", _library );
    conf.set( "scale", _scale );
    conf.set( "script", _script );
    conf.set( "placement", "vertex",    _placement, PLACEMENT_VERTEX );
    conf.set( "placement", "interval",  _placement, PLACEMENT_INTERVAL );
    conf.set( "placement", "random",    _placement, PLACEMENT_RANDOM );
    conf.set( "placement", "centroid",  _placement, PLACEMENT_CENTROID );
    conf.set( "density", _density );
    conf.set( "random_seed", _randomSeed );

    std::string tagstring = this->tagString();
    if ( !tagstring.empty() )
        conf.set("tags", tagstring);

    return conf;
}

void 
InstanceSymbol::mergeConfig( const Config& conf )
{
    conf.get( "url", _url );
    conf.get( "library", _library );
    conf.get( "scale", _scale );
    conf.get( "script", _script );
    conf.get( "placement", "vertex",   _placement, PLACEMENT_VERTEX );
    conf.get( "placement", "interval", _placement, PLACEMENT_INTERVAL );
    conf.get( "placement", "random",   _placement, PLACEMENT_RANDOM );
    conf.get( "placement", "centroid", _placement, PLACEMENT_CENTROID );
    conf.get( "density", _density );
    conf.get( "random_seed", _randomSeed );
    
    addTags( conf.value("tags") );
}


const IconSymbol*
InstanceSymbol::asIcon() const
{
    return dynamic_cast<const IconSymbol*>( this );
}

const ModelSymbol*
InstanceSymbol::asModel() const
{
    return dynamic_cast<const ModelSymbol*>( this );
}
