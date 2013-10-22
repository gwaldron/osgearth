/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include <osgEarthSymbology/Expression>
#include <osgEarth/StringUtils>
#include <algorithm>

using namespace osgEarth;
using namespace osgEarth::Symbology;

#define LC "[Expression] "

NumericExpression::NumericExpression( const std::string& expr ) : 
_src  ( expr ),
_value( 0.0 ),
_dirty( true )
{
    init();
}

NumericExpression::NumericExpression( const NumericExpression& rhs ) :
_src  ( rhs._src ),
_rpn  ( rhs._rpn ),
_vars ( rhs._vars ),
_value( rhs._value ),
_dirty( rhs._dirty )
{
    //nop
}

NumericExpression::NumericExpression( double staticValue ) :
_value( staticValue ),
_dirty( false )
{
    _src = Stringify() << staticValue;
    init();
}

NumericExpression::NumericExpression( const Config& conf )
{
    mergeConfig( conf );
    init();
}

void
NumericExpression::mergeConfig( const Config& conf )
{
    _src = conf.value();
    init();
    _dirty = true;
}

Config
NumericExpression::getConfig() const
{
    return Config( "numeric_expression", _src );
}

#define IS_OPERATOR(a) ( a .first == ADD || a .first == SUB || a .first == MULT || a .first == DIV || a .first == MOD )

void
NumericExpression::init()
{
    _vars.clear();
    _rpn.clear();

    StringTokenizer variablesTokenizer( "", "" );
    variablesTokenizer.addDelims( "[]", true );
    variablesTokenizer.addQuotes( "'\"", true );
    variablesTokenizer.keepEmpties() = false;

    StringTokenizer operandTokenizer( "", "" );
    operandTokenizer.addDelims( ",()%*/+-", true );
    operandTokenizer.addQuotes( "'\"", true );
    operandTokenizer.keepEmpties() = false;

    StringVector variablesTokens;
    variablesTokenizer.tokenize( _src, variablesTokens );

    StringVector t;
    bool invar = false;
    for( unsigned i=0; i<variablesTokens.size(); ++i )
    {
        if ( variablesTokens[i] == "[" && !invar )
        {
            // Start variable, add "[" token
            invar = true;
            t.push_back(variablesTokens[i]);
        }
        else if ( variablesTokens[i] == "]" && invar )
        {
            // End variable, add "]" token
            invar = false;
            t.push_back(variablesTokens[i]);
        }
        else if ( invar )
        {
            // Variable, add variable token
            t.push_back(variablesTokens[i]);
        }
        else
        {
            // Operands, tokenize it and add tokens
            StringVector operandTokens;
            operandTokenizer.tokenize( variablesTokens[i], operandTokens );
            t.insert(t.end(), operandTokens.begin(), operandTokens.end());
        }
    }

    // identify tokens:
    AtomVector infix;
    invar = false;
    for( unsigned i=0; i<t.size(); ++i ) {
        if ( t[i] == "[" && !invar ) {
            invar = true;
        }
        else if ( t[i] == "]" && invar ) {
            invar = false;
            infix.push_back( Atom(VARIABLE,0.0) );
            _vars.push_back( Variable(t[i-1],0) );
        }
        else if ( t[i] == "(" ) infix.push_back( Atom(LPAREN,0.0) );
        else if ( t[i] == ")" ) infix.push_back( Atom(RPAREN,0.0) );
        else if ( t[i] == "," ) infix.push_back( Atom(COMMA,0.0) );
        else if ( t[i] == "%" ) infix.push_back( Atom(MOD,0.0) );
        else if ( t[i] == "*" ) infix.push_back( Atom(MULT,0.0) );
        else if ( t[i] == "/" ) infix.push_back( Atom(DIV,0.0) );
        else if ( t[i] == "+" ) infix.push_back( Atom(ADD,0.0) );
        else if ( t[i] == "-" ) infix.push_back( Atom(SUB,0.0) );
        else if ( t[i] == "min" ) infix.push_back( Atom(MIN,0.0) );
        else if ( t[i] == "max" ) infix.push_back( Atom(MAX,0.0) );
        else if ( (t[i][0] >= '0' && t[i][0] <= '9') || t[i][0] == '.' )
            infix.push_back( Atom(OPERAND,as<double>(t[i],0.0)) );
        else if ( t[i] != "," && (i == 0 || t[i-1] != "["))
        {
          std::string var = t[i];
          // If this is a call to a script function, store the entire function
          // call in the variable string
          if (i < t.size() - 1 && t[i+1] == "(")
          {
            int parenCount = 0;
            do
            {
              ++i;
              var += t[i];

              if (t[i] == "(")
                parenCount++;
              else if (t[i] == ")")
                parenCount--;

            } while (i < t.size() - 1 && parenCount > 0);
          }

          infix.push_back( Atom(VARIABLE, 0.0) );
          _vars.push_back( Variable(var, 0) );
        }

        // note: do nothing for a comma
    }

    // convert to RPN:
    // http://en.wikipedia.org/wiki/Shunting-yard_algorithm
    AtomStack s;
    unsigned var_i = 0;

    for( unsigned i=0; i<infix.size(); ++i )
    {
        Atom& a = infix[i];

        if ( a.first == LPAREN )
        {
            s.push( a );
        }
        else if ( a.first == RPAREN )
        {
            while( s.size() > 0 )
            {
                Atom top = s.top();
                s.pop();
                if ( top.first == LPAREN )
                    break;
                else
                    _rpn.push_back( top );
            }
        }
        else if ( a.first == COMMA )
        {
            while( s.size() > 0 && s.top().first != LPAREN )
            {
                _rpn.push_back( s.top() );
                s.pop();
            }
        }
        else if ( IS_OPERATOR(a) )
        {
            if ( s.empty() || a.first > s.top().first )
            {
                s.push( a );
            }
            else 
            {
                while( s.size() > 0 && a.first < s.top().first && IS_OPERATOR(s.top()) )
                {
                    _rpn.push_back( s.top() );
                    s.pop();
                }
                s.push( a );
            }
        }
        else if ( a.first == MIN || a.first == MAX )
        {
            s.push( a );
        }
        else if ( a.first == OPERAND )
        {
            _rpn.push_back( a );
        }
        else if ( a.first == VARIABLE )
        {
            _rpn.push_back( a );
            _vars[var_i++].second = _rpn.size()-1; // store the index
        }
    }

    while( s.size() > 0 )
    {
        _rpn.push_back( s.top() );
        s.pop();
    }
}

void 
NumericExpression::set( const Variable& var, double value )
{
    Atom& a = _rpn[var.second];
    if ( a.second != value )
    {
        a.second = value;
        _dirty = true;
    }
}

double
NumericExpression::eval() const
{
    if ( _dirty )
    {
        std::stack<double> s;

        for( unsigned i=0; i<_rpn.size(); ++i )
        {
            const Atom& a = _rpn[i];

            if ( a.first == ADD )
            {
                if ( s.size() >= 2 )
                {
                    double op2 = s.top(); s.pop();
                    double op1 = s.top(); s.pop();
                    s.push( op1 + op2 );
                }
            }
            else if ( a.first == SUB )
            {
                if ( s.size() >= 2 )
                {
                    double op2 = s.top(); s.pop();
                    double op1 = s.top(); s.pop();
                    s.push( op1 - op2 );
                }
            }
            else if ( a.first == MULT )
            {
                if ( s.size() >= 2 )
                {
                    double op2 = s.top(); s.pop();
                    double op1 = s.top(); s.pop();
                    s.push( op1 * op2 );
                }
            }
            else if ( a.first == DIV )
            {
                if ( s.size() >= 2 )
                {
                    double op2 = s.top(); s.pop();
                    double op1 = s.top(); s.pop();
                    s.push( op1 / op2 );
                }
            }
            else if ( a.first == MOD )
            {
                if ( s.size() >= 2 )
                {
                    double op2 = s.top(); s.pop();
                    double op1 = s.top(); s.pop();
                    s.push( fmod(op1, op2) );
                }
            }
            else if ( a.first == MIN )
            {
                if ( s.size() >= 2 )
                {
                    double op2 = s.top(); s.pop();
                    double op1 = s.top(); s.pop();
                    s.push( std::min(op1, op2) );
                }
            }
            else if ( a.first == MAX )
            {
                if ( s.size() >= 2 )
                {
                    double op2 = s.top(); s.pop();
                    double op1 = s.top(); s.pop();
                    s.push( std::max(op1, op2) );
                }
            }
            else // OPERAND or VARIABLE
            {
                s.push( a.second );
            }
        }

        const_cast<NumericExpression*>(this)->_value = s.size() > 0 ? s.top() : 0.0;
        const_cast<NumericExpression*>(this)->_dirty = false;
    }

    return !osg::isNaN( _value ) ? _value : 0.0;
}

//------------------------------------------------------------------------

StringExpression::StringExpression( const std::string& expr ) : 
_src( expr ),
_dirty( true )
{
    init();
}

StringExpression::StringExpression(const std::string& expr,
                                   const URIContext&  uriContext) :
_src       ( expr ),
_uriContext( uriContext ),
_dirty     ( true )
{
    init();
}

StringExpression::StringExpression( const StringExpression& rhs ) :
_src( rhs._src ),
_vars( rhs._vars ),
_value( rhs._value ),
_infix( rhs._infix ),
_dirty( rhs._dirty ),
_uriContext( rhs._uriContext )
{
    //nop
}

void
StringExpression::setInfix( const std::string& expr )
{
    _src = expr;
    _dirty = true;
    init();
}

void
StringExpression::setLiteral( const std::string& expr )
{
    _src = "\"" + expr + "\"";
    _value = expr;
    _dirty = false;
}

StringExpression::StringExpression( const Config& conf )
{
    mergeConfig( conf );
    init();
}

void
StringExpression::mergeConfig( const Config& conf )
{
    _src        = conf.value();
    _uriContext = conf.referrer();
    _dirty      = true;
}

Config
StringExpression::getConfig() const
{
    Config conf( "string_expression", _src );
    conf.setReferrer( uriContext().referrer() );
    return conf;
}

void
StringExpression::init()
{
    bool inQuotes = false;
    int inVar = 0;
    int startPos = 0;
    for (int i=0; i < (int)_src.length(); i++)
    {
      if (_src[i] == '"')
      {
        if (inQuotes)
        {
          int length = i - startPos;
          if (length > 0)
            _infix.push_back( Atom(OPERAND, _src.substr(startPos, length)) );

          inQuotes = false;
        }
        else if (!inVar)
        {
          inQuotes = true;
          startPos = i + 1;
        }
      }
      else if (_src[i] == '+' || _src[i] == ' ')
      {
        if (inVar == 1)
        {
          int length = i - startPos;

          //Check for feature attribute access
          if (length > 2 && _src[startPos] == '[' && _src[i - 1] == ']')
          {
            startPos++;
            length -= 2;
          }

          if (length > 0)
          {
            std::string val = _src.substr(startPos, length);
            _vars.push_back( Variable(val, _infix.size()) );
            _infix.push_back( Atom(VARIABLE,val) );
          }

          inVar = 0;
        }
      }
      else if ((_src[i] == '(' || _src[i] == '[') && inVar)
      {
        inVar++;
      }
      else if ((_src[i] == ')' || _src[i] == ']') && inVar > 1)
      {
        inVar--;
      }
      else
      {
        if (!inQuotes && !inVar)
        {
          inVar = 1;
          startPos = i;
        }
      }
    }

    if (inVar == 1)
    {
      int length = _src.length() - startPos;

      //Check for feature attribute access
      if (length > 2 && _src[startPos] == '[' && _src[_src.length() - 1] == ']')
      {
        startPos++;
        length -= 2;
      }

      if (length > 0)
      {
        std::string val = _src.substr(startPos, length);
        _vars.push_back( Variable(val,_infix.size()) );
        _infix.push_back( Atom(VARIABLE,val) );
      }
    }
}

void 
StringExpression::set( const Variable& var, const std::string& value )
{
    Atom& a = _infix[var.second];
    if ( a.second != value )
    {
        a.second = value;
        _dirty = true;
    }
}

void
StringExpression::set( const std::string& varName, const std::string& value )
{
    for( Variables::const_iterator v = _vars.begin(); v != _vars.end(); ++v )
    {
        if ( v->first == varName )
        {
            set( *v, value );
        }
    }
}

const std::string&
StringExpression::eval() const
{
    if ( _dirty )
    {
        std::stringstream buf;
        for( AtomVector::const_iterator i = _infix.begin(); i != _infix.end(); ++i )
            buf << i->second;

        const_cast<StringExpression*>(this)->_value = buf.str();
        const_cast<StringExpression*>(this)->_dirty = false;
    }

    return _value;
}
