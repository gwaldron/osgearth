/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osgEarthSymbology/SymbolicNode>
#include <osg/NodeVisitor>

using namespace osgEarth::Symbology;

SymbolicNode::SymbolicNode()
{
    _symGroup = new osg::Group();
    _symGroup->setDataVariance( osg::Object::DYNAMIC );
    this->addChild( _symGroup.get() );
    setNumChildrenRequiringUpdateTraversal(1);
}

SymbolicNode::SymbolicNode( const SymbolicNode& rhs, const osg::CopyOp& op ) :
osg::Group( rhs, op ),
_style( rhs._style ),
_symbolizer( rhs._symbolizer ),
_dataSet( rhs._dataSet ),
//_dataSetRevision( rhs._dataSetRevision ),
//_styleRevision( rhs._styleRevision ),
_symGroup( rhs._symGroup )
{
    _state = _symbolizer ? _symbolizer->createState() : 0L;
}

void
SymbolicNode::setSymbolizer( Symbolizer* sym )
{
    _symbolizer = sym;
    _state = _symbolizer ? _symbolizer->createState() : 0L;
}

void
SymbolicNode::traverse( osg::NodeVisitor& nv )
{
    setNumChildrenRequiringUpdateTraversal(1);
    if ( _symbolizer.valid() && _state.valid() && nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
    {   
        if ( _dataSet->outOfSync( _state->_dataSetRevision ) || _style->outOfSync( _state->_styleRevision ) )
        {
            _symbolizer->update(
                _dataSet.get(),
                _style.get(),
                _symGroup.get(),
                _context.get(),
                _state.get() );

            if ( _dataSet.valid() )
                _dataSet->sync( _state->_dataSetRevision );

            if ( _style.valid() )
                _style->sync( _state->_styleRevision );

            this->dirtyBound();
        }
    }

    osg::Group::traverse( nv );
}

