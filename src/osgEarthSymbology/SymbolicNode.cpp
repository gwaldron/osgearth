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

SymbolicNode::SymbolicNode() :
    _dataSetRevision( -1 ),
    _styleRevision( -1 )
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
_dataSetRevision( rhs._dataSetRevision ),
_styleRevision( rhs._styleRevision ),
_symGroup( rhs._symGroup )
{
}

void
SymbolicNode::traverse( osg::NodeVisitor& nv )
{
    setNumChildrenRequiringUpdateTraversal(1);
    if ( _symbolizer.valid() )
    {
        if ( nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR)
        {
            // if our symbology is out of revision, update it!
            if (_dataSet.valid() && (_dataSetRevision != _dataSet->getRevision()) ||
                _styleRevision != _style->getRevision())
            {
                _symbolizer->update( _dataSet.get(), _style, _symGroup.get(), _context.get() );
                if (_dataSet.valid())
                    _dataSetRevision = _dataSet->getRevision();

                _styleRevision = _style->getRevision();
                this->dirtyBound();
            }
        }
    }

    osg::Group::traverse( nv );
}

