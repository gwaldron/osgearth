//
//  ViewController.h
//  osgEarthViewerIOS
//
//  Created by Thomas Hogarth on 14/07/2012.
//

#import <UIKit/UIKit.h>
#import <QuartzCore/QuartzCore.h>

#include <osgViewer/Viewer>

@interface ViewController : UIViewController{

    osg::ref_ptr<osgViewer::Viewer> _viewer;
    CADisplayLink* _displayLink;
    bool _isAnimating;
    
    //the file to load
    std::string _file;
}

- (id)intWithFileName:(NSString*)file;

- (void)startAnimation;
- (void)stopAnimation;

@end
