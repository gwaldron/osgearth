/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2010 Pelican Mapping
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

using namespace osgEarth;
using namespace osgEarth::Symbology;

Expression::Expression( const std::string& expr )
{
    StringVector t;
    tokenize(expr, t, "()%*/+-", "'\"", false, true);

    // identify tokens:
    AtomVector infix;
    for( unsigned i=0; i<t.size(); ++i ) {
        if      ( t[i] == "(" ) infix.push_back( Atom(LPAREN,0.0) );
        else if ( t[i] == ")" ) infix.push_back( Atom(RPAREN,0.0) );
        else if ( t[i] == "%" ) infix.push_back( Atom(MOD,0.0) );
        else if ( t[i] == "*" ) infix.push_back( Atom(MULT,0.0) );
        else if ( t[i] == "/" ) infix.push_back( Atom(DIV,0.0) );
        else if ( t[i] == "+" ) infix.push_back( Atom(ADD,0.0) );
        else if ( t[i] == "-" ) infix.push_back( Atom(SUB,0.0) );
        else if ( t[i][0] >= '0' && t[i][0] <= '9' ) infix.push_back( Atom(OPERAND,as<double>(t[i],0.0)) );
        else {
            infix.push_back( Atom(VARIABLE,0.0) );
            _vars.push_back( Variable(t[i],0) );
        }
    }

    // convert to RPN:
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
        else if ( a.first == ADD || a.first == SUB || a.first == MULT || a.first == DIV || a.first == MOD )
        {
            if ( s.empty() || a.first > s.top().first )
            {
                s.push( a );
            }
            else 
            {
                while( s.size() > 0 && a.first < s.top().first )
                {
                    _rpn.push_back( s.top() );
                    s.pop();
                }
                s.push( a );
            }
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

Expression::Expression( const Expression& rhs ) :
_rpn( rhs._rpn ),
_vars( rhs._vars )
{
    //nop
}

void 
Expression::set( const Variable& var, double value )
{
    _rpn[var.second].second = value;
}

double
Expression::eval() const
{
    std::stack<double> s;

    for( unsigned i=0; i<_rpn.size(); ++i )
    {
        const Atom& a = _rpn[i];

        if ( a.first == ADD )
        {
            double op2 = s.top(); s.pop();
            double op1 = s.top(); s.pop();
            s.push( op1 + op2 );
        }
        else if ( a.first == SUB )
        {
            double op2 = s.top(); s.pop();
            double op1 = s.top(); s.pop();
            s.push( op1 - op2 );
        }
        else if ( a.first == MULT )
        {
            double op2 = s.top(); s.pop();
            double op1 = s.top(); s.pop();
            s.push( op1 * op2 );
        }
        else if ( a.first == DIV )
        {
            double op2 = s.top(); s.pop();
            double op1 = s.top(); s.pop();
            s.push( op1 / op2 );
        }
        else if ( a.first == MOD )
        {
            double op2 = s.top(); s.pop();
            double op1 = s.top(); s.pop();
            s.push( fmod(op1, op2) );
        }
        else // OPERAND or VARIABLE
        {
            s.push( a.second );
        }
    }

    return s.size() > 0 ? s.top() : 0.0;
}
