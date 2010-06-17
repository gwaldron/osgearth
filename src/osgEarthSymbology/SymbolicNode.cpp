///* -*-c++-*- */
///* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
// * Copyright 2008-2009 Pelican Ventures, Inc.
// * http://osgearth.org
// *
// * osgEarth is free software; you can redistribute it and/or modify
// * it under the terms of the GNU Lesser General Public License as published by
// * the Free Software Foundation; either version 2 of the License, or
// * (at your option) any later version.
// *
// * This program is distributed in the hope that it will be useful,
// * but WITHOUT ANY WARRANTY; without even the implied warranty of
// * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// * GNU Lesser General Public License for more details.
// *
// * You should have received a copy of the GNU Lesser General Public License
// * along with this program.  If not, see <http://www.gnu.org/licenses/>
// */
//#include <osgEarthSymbology/SymbolicNode>
//#include <osg/NodeVisitor>
//
//using namespace osgEarth::Symbology;
//
//struct SymUpdateCallback : public osg::NodeCallback
//{
//    virtual void operator ()(osg::Node* node, osg::NodeVisitor* nv)
//    {
//        static_cast<SymbolicNodeBase*>(node)->updateSymbology();
//        traverse( node, nv );
//    }
//};
//
//template<typename S, typename C>
//SymbolicNode<S,C>::SymbolicNode()
//{
//    // create and incorporate the attach group.
//    _symGroup = new osg::Group();
//    _symGroup->setDataVariance( osg::Object::DYNAMIC );
//    this->addChild( _symGroup.get() );
//
//    // install an update callback for managing content and style changes.
//    this->addUpdateCallback( new SymUpdateCallback() );
//}
//
//template<typename S, typename C>
//SymbolicNode<S,C>::SymbolicNode( const SymbolicNode& rhs, const osg::CopyOp& op ) :
//osg::Group( rhs, op ),
//_contentPending( rhs._contentPending ),
//_stylePending( rhs._stylePending ),
//_symGroup( rhs._symGroup )
//{
//    // this will cause a new state to form and correctly copy the state values
//    setSymbolizer( rhs._symbolizer.get() );
//}
//
//template<typename S, typename C>
//void
//SymbolicNode<S,C>::setContent( const C* content )
//{
//    _contentPending = content;
//    if ( getOrCreateState() )
//        _state->setContent( content );
//}
//
//template<typename S, typename C>
//const C*
//SymbolicNode<S,C>::getContent() const 
//{
//    return _state.valid() ? _state->getContent() : _contentPending.get();
//}
//
//template<typename S, typename C>
//void
//SymbolicNode<S,C>::setStyle( const Style* style )
//{
//    _stylePending = style;
//    if ( getOrCreateState() )
//        _state->setStyle( style );
//}
//
//template<typename S, typename C>
//const Style*
//SymbolicNode<S,C>::getStyle() const 
//{
//    return _state.valid() ? _state->getStyle() : _stylePending.get();
//}
//
//template<typename S, typename C>
//void
//SymbolicNode<S,C>::setSymbolizer( Symbolizer<S>* sym )
//{
//    // copy the old state over to the new state.
//    osg::ref_ptr<STATE_TYPE> oldState = _state.get();
//
//    _symbolizer = sym;
//    _state = 0L;
//
//    if ( _symbolizer.valid() )
//    {
//        _state = _symbolizer->createState();
//        if ( oldState.valid() )
//        {
//            _state->setContent( _oldState->getContent() );
//            _state->setStyle( _oldState->getStyle() );
//            _state->setContext( _oldState->getContext() );
//        }
//        else
//        {
//            _state->setContent( _contentPending.get() );
//            _state->setStyle( _stylePending.get() );
//        }
//    }
//}
//
//template<typename S, typename C>
//void
//SymbolicNode<S,C>::updateSymbology()
//{
//    if ( _symbolizer.valid() && _state.valid() )
//    {   
//        if ( _symbolizer->getAlwaysUpdate() ||
//            ( _state->getContent() && _state->getContent()->outOfSyncWith( _state->_contentRevision ) ) ||
//            ( _state->getStyle() && _state->getStyle()->outOfSyncWith( _state->_styleRevision ) ) )
//        {
//            _symbolizer->update( _state.get(), _symGroup.get(), _context.get() );
//
//            //_symbolizer->update(
//            //    _dataSet.get(),
//            //    _style.get(),
//            //    _symGroup.get(),
//            //    _context.get(),
//            //    _state.get() );
//
//            if ( _state->getContent() )
//                _state->getContent()->sync( _state->_contentRevision );
//
//            if ( _state->getStyle() )
//                _state->getStyle()->sync( _state->_styleRevision );
//
//            this->dirtyBound();
//        }
//    }
//}
//
//template<typename S, typename C>
//S*
//SymbolicNode<S,C>::getOrCreateState()
//{
//    if ( !_state.valid() && _symbolizer.valid() )
//    {
//        _state = new S();
//    }
//
//    return _state.get();
//}
//
