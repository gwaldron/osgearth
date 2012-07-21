//
//  ViewController.h
//  osgEarthViewerIOS
//
//  Created by Thomas Hogarth on 14/07/2012.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#import <UIKit/UIKit.h>

#include <osgViewer/Viewer>

@interface ViewController : UIViewController{

    osg::ref_ptr<osgViewer::Viewer> _viewer;
    CADisplayLink* _displayLink;
    bool _isAnimating;
}

- (void)startAnimation;
- (void)stopAnimation;

@end
