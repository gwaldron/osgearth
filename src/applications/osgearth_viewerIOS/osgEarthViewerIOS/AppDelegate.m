//
//  AppDelegate.m
//  osgEarthViewerIOS
//
//  Created by Thomas Hogarth on 14/07/2012.
//

#import "AppDelegate.h"

#import "StartViewerController.h"

@implementation AppDelegate

@synthesize window = _window;
@synthesize startViewerController = _startViewerController;

- (void)dealloc
{
    [_window release];
    [_startViewerController release];
    [super dealloc];
}

- (BOOL)application:(UIApplication *)application didFinishLaunchingWithOptions:(NSDictionary *)launchOptions
{
    self.window = [[[UIWindow alloc] initWithFrame:[[UIScreen mainScreen] bounds]] autorelease];
    // Override point for customization after application launch.
    
    self.startViewerController = [[StartViewerController alloc] init];
    self.window.rootViewController = self.startViewerController;
    [self.window makeKeyAndVisible];
    return YES;
}

- (void)applicationWillResignActive:(UIApplication *)application
{
    // Sent when the application is about to move from active to inactive state. This can occur for certain types of temporary interruptions (such as an incoming phone call or SMS message) or when the user quits the application and it begins the transition to the background state.
    // Use this method to pause ongoing tasks, disable timers, and throttle down OpenGL ES frame rates. Games should use this method to pause the game.
}

- (void)applicationDidEnterBackground:(UIApplication *)application
{
    // Use this method to release shared resources, save user data, invalidate timers, and store enough application state information to restore your application to its current state in case it is terminated later. 
    // If your application supports background execution, this method is called instead of applicationWillTerminate: when the user quits.
    [self.startViewerController stopAnimation];
}

- (void)applicationWillEnterForeground:(UIApplication *)application
{
    // Called as part of the transition from the background to the inactive state; here you can undo many of the changes made on entering the background.
}

- (void)applicationDidBecomeActive:(UIApplication *)application
{
    // Restart any tasks that were paused (or not yet started) while the application was inactive. If the application was previously in the background, optionally refresh the user interface.
    [self.startViewerController startAnimation];
}

- (void)applicationWillTerminate:(UIApplication *)application
{
    OSG_ALWAYS << "applicationWillTerminate" <<std::endl;
    // Called when the application is about to terminate. Save data if appropriate. See also applicationDidEnterBackground:.
    self.window.rootViewController = nil;
    [self.startViewerController stopAnimation];
    //self.viewController = nil;
}

@end
