//
//  StartViewerController.h
//  osgEarthViewerIOS
//
//  Created by Thomas Hogarth on 31/07/2012.
//

#import <UIKit/UIKit.h>

@class ViewController;

@interface StartViewerController : UIViewController <UIPickerViewDelegate, UIPickerViewDataSource> {

    IBOutlet UIPickerView *pickerView;
    NSMutableArray *fileArray;
    NSInteger currentSelection;
}

@property(nonatomic, retain) IBOutlet UIPickerView *pickerView;
@property (strong, nonatomic) ViewController *osgEarthViewController;

-(IBAction)onStartViewer;

- (void)startAnimation;
- (void)stopAnimation;

@end
