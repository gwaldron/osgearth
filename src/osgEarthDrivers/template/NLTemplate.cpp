#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "NLTemplate.h"

#include <osgEarth/FileUtils>


using namespace std;
using namespace NL::Template;



enum {
    TOKEN_END,
    TOKEN_TEXT,
    TOKEN_BLOCK,
    TOKEN_ENDBLOCK,
    TOKEN_INCLUDE,
    TOKEN_VAR
};


static inline bool alphanum( const char c ) {
    return
    ( c >= 'a' && c <= 'z' ) ||
    ( c >= 'A' && c <= 'Z' ) ||
    ( c >= '0' && c <= '9' ) ||    
    ( c == '_' ) ||
    ( c == '.' ) ||

    // JB:  Added these to enable matching of paths as well
    ( c == ' ' ) ||
    ( c == ':' ) ||
    ( c == '-' ) ||
    ( c == '@' ) ||
    ( c == '/' ) ||
    ( c == '\\' );
}


static inline long match_var( const char *text, string & result ) {
    if (text[ 0 ] != '{' ||
        text[ 1 ] != '{' ||
        text[ 2 ] != ' ' )
    {
        return -1;
    }
    
    const char *var = text + 3;
    const char *cursor = var;
    
    while ( *cursor ) {
        if (cursor[ 0 ] == ' ' &&
            cursor[ 1 ] == '}' &&
            cursor[ 2 ] == '}' )
        {
            result = string( var, cursor - var );
            return cursor + 3 - text;
        }
        
        if ( !alphanum( *cursor ) ) {
            return -1;
        }
        
        cursor++;
    }
    
    return -1;
}


static inline long match_tag_with_param( const char *tag, const char *text, string & result ) {
    if (text[ 0 ] != '{' ||
        text[ 1 ] != '%' ||
        text[ 2 ] != ' ')
    {
        return -1;
    }

    long taglen = strlen( tag );
    if ( strncmp( text + 3, tag, taglen ) ) {
        return -1;
    }

    const char *param = text + 3 + taglen;
    
    if ( *param != ' ' ) {
        return -1;
    }
    
    param++;

    const char *cursor = param;

    while ( *cursor ) {
        if (cursor[ 0 ] == ' ' &&
            cursor[ 1 ] == '%' &&
            cursor[ 2 ] == '}' )
        {
            result = string( param, cursor - param );
            return cursor + 3 - text;
        }

        if ( !alphanum( *cursor ) ) {
            return -1;
        }
        
        cursor++;
    }
    
    return -1;
}


Tokenizer::Tokenizer( const char *text ) :
text( text ),
len( strlen( text ) ),
pos( 0 ),
peeking( false )
{
}


Tokenizer::~Tokenizer() {
    free( (void*) text );
}


Token Tokenizer::next() {
    static const char * s_endblock = "{% endblock %}";
    static const char * s_block = "block";
    static const char * s_include = "include";
    static const long s_endblock_len = strlen( s_endblock );
    
    if ( peeking ) {
        peeking = false;
        return peek;
    }
    
    Token token;
    token.value.clear();
    peek.value.clear();
    token.type = TOKEN_END;
    peek.type = TOKEN_END;
    
    long textpos = pos;
    long textlen = 0;
    
a:
    if ( pos < len ) {
        long m = match_tag_with_param( s_block, text + pos, peek.value );
        if ( m > 0 ) {
            peek.type = TOKEN_BLOCK;
            pos += m;
        } else if ( !strncmp( s_endblock, text + pos, s_endblock_len ) ) {
            peek.type = TOKEN_ENDBLOCK;
            pos += s_endblock_len;
        } else if ( ( m = match_tag_with_param( s_include, text + pos, peek.value ) ) > 0 ) {
            peek.type = TOKEN_INCLUDE;
            pos += m;
        } else if ( ( m = match_var( text + pos, peek.value ) ) > 0 ) {
            peek.type = TOKEN_VAR;
            pos += m;
        } else {
            textlen ++;
            pos ++;
            peeking = true;
            goto a;
        }
    }

    if ( peeking ) {
        token.type = TOKEN_TEXT;
        token.value = string( text + textpos, textlen );
        return token;
    }

    return peek;
}


const string Dictionary::find( const string & name ) const {
    for ( size_t i=0; i < properties.size(); i++ ) {
        if ( properties[ i ].first == name ) {
            return properties[ i ].second;
        }
    }
    return "";
}


void Dictionary::set( const string & name, const string & value ) {
    for ( size_t i=0; i < properties.size(); i++ ) {
        if ( properties[ i ].first == name ) {
            properties[ i ].second = value;
            return;
        }
    }
    properties.push_back( pair<string, string>( name, value ) );
}


Fragment::~Fragment() {
}


bool Fragment::isBlockNamed( const string & ) const {
    return false;
}



Text::Text( const string & text ) : text( text ) {
}


void Text::render( Output & output, const Dictionary & ) const {
    output.print( text );
}


Fragment *Text::copy() const {
    return new Text( text );
}


Property::Property( const string & name ) : name( name ) {
}


void Property::render( Output & output, const Dictionary & dictionary ) const {
    output.print( dictionary.find( name ) );
}


Fragment *Property::copy() const {
    return new Property( name );
}


Node::~Node() {
    for ( size_t i=0; i < fragments.size(); i++ ) {
        delete fragments[ i ];
    }
}


Fragment *Node::copy() const {
    Node *node = new Node();
    node->properties = properties;
    for ( size_t i=0; i < fragments.size(); i++ ) {
        node->fragments.push_back( fragments[ i ]->copy() );
    }
    return node;
}


void Node::render( Output & output, const Dictionary & ) const {
    for ( size_t i=0; i < fragments.size(); i++ ) {
        fragments[ i ]->render( output, *this );
    }
}



Block & Node::block( const string & name ) const {
    for ( size_t i=0; i < fragments.size(); i++ ) {
        if ( fragments[ i ]->isBlockNamed( name ) ) {
            return *dynamic_cast<Block*>( fragments[ i ] );
        }
    }
    throw 0;
}


Block::Block( const string & name ) : name( name ), enabled( true ), resized( false ) {
}


Fragment *Block::copy() const {
    Block *block = new Block( name );
    block->properties = properties;
    for ( size_t i=0; i < fragments.size(); i++ ) {
        block->fragments.push_back( fragments[ i ]->copy() );
    }
    return block;
}


Block::~Block() {
    for ( size_t i=0; i < nodes.size(); i++ ) {
        delete nodes[ i ];
    }
}


bool Block::isBlockNamed( const string & name ) const {
    return this->name == name;
}


void Block::enable() {
    enabled = true;
}


void Block::disable() {
    enabled = false;
}

void Block::repeat( size_t n ) {
    resized = true;
    for ( size_t i=0; i < nodes.size(); i++ ) {
        delete nodes[ i ];
    }
    nodes.clear();
    for ( size_t i=0; i < n; i++ ) {
        nodes.push_back( static_cast<Node*>( copy() ) );
    }
}


Node & Block::operator[]( size_t index ) {
    return *nodes.at( index );
}


void Block::render( Output & output, const Dictionary & ) const {
    if ( enabled ) {
        if ( resized ) {
            for ( size_t i=0; i < nodes.size(); i++ ) {
                nodes[ i ]->render( output, *nodes[ i ] );
            }
        } else {
            Node::render( output, *this );
        }
    }
}


Output::~Output() {
}


void OutputString::print( const string & text ) {
    buf << text;
}


void OutputStdout::print( const std::string &text ) {
    cout << text;
}


Loader::~Loader() {
}


const std::string& Loader::getReferrer() const
{
    return _referrer;
}

void Loader::setReferrer(const std::string& referrer)
{
    _referrer = referrer; 
}


const char * LoaderFile::load( const char *name ) {

    std::string fullPath = osgEarth::getFullPath(_referrer, std::string(name));

    FILE *f = fopen( fullPath.c_str(), "rb" );
    if (!f) {
        std::cout << "NLTemplate error:  Failed to open " << fullPath << std::endl;
        return 0;
    }
    fseek( f, 0, SEEK_END );
    long len = ftell( f );
    if (len > 0)
    {
        fseek( f, 0, SEEK_SET );
        char *buffer = (char*) malloc( len + 1 );
        fread( (void*) buffer, (size_t)len, 1, f );
        fclose( f );
        buffer[ len ] = 0;
        return buffer;
    }
    else
    {
        fclose(f);
        return 0L;
    }
}


Template::Template( Loader & loader ) : Block( "main" ), loader( loader ) {
}


void Template::load_recursive( const char *name, vector<Tokenizer*> & files, vector<Node*> & nodes ) {
        
    const char* loaded = loader.load( name );
    if (!loaded)
    {
        return;
    }

    Tokenizer *tokenizer = new Tokenizer( loaded );
    files.push_back( tokenizer );

    std::string referrer = osgEarth::getFullPath(loader.getReferrer(), std::string(name));
    _referrerStack.push_back( referrer );
    loader.setReferrer( referrer );      

    bool done = false;
    while( !done ) {
        Token token = files.back()->next();
        switch ( token.type ) {
            case TOKEN_END:
                done = true;
                break;
            case TOKEN_BLOCK: {
                Block *block = new Block( token.value );
                nodes.back()->fragments.push_back( block );
                nodes.push_back( block );
            }
                break;
            case TOKEN_ENDBLOCK:
                nodes.pop_back();
                break;
            case TOKEN_VAR:
                nodes.back()->fragments.push_back( new Property( token.value ) );
                break;
            case TOKEN_TEXT:
                nodes.back()->fragments.push_back( new Text( token.value ) );
                break;
            case TOKEN_INCLUDE:
                load_recursive( token.value.c_str(), files, nodes );
                break;
        }
    }

    _referrerStack.pop_back();
    if (!_referrerStack.empty())
    {
        loader.setReferrer( _referrerStack.back() );
    }
    else
    {
        loader.setReferrer("");
    }
    
    delete files.back();
    files.pop_back();
}


void Template::clear() {
    for ( size_t i=0; i < fragments.size(); i++ ) {
        delete fragments[ i ];
    }
    for ( size_t i=0; i < nodes.size(); i++ ) {
        delete nodes[ i ];
    }
    nodes.clear();
    fragments.clear();
    properties.clear();
}


void Template::load( const char *name ) {
    clear();
    
    vector<Node*> stack;
    stack.push_back( this );
    
    vector<Tokenizer*> file_stack;
   
    load_recursive( name, file_stack, stack );

}


void Template::render( Output & output ) const {
    Node::render( output, *this );
}

