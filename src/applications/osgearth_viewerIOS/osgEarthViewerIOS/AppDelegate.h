//
//  AppDelegate.h
//  osgEarthViewerIOS
//
//  Created by Thomas Hogarth on 14/07/2012.
//

#import <UIKit/UIKit.h>

#include <osgViewer/Viewer>

@class StartViewerController;

@interface AppDelegate : UIResponder <UIApplicationDelegate>

@property (strong, nonatomic) UIWindow *window;

@property (strong, nonatomic) StartViewerController *startViewerController;

@end
