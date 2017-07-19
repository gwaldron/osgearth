//
//  StartViewerController.m
//  osgEarthViewerIOS
//
//  Created by Thomas Hogarth on 31/07/2012.
//

#import "StartViewerController.h"
#import "ViewController.h"

#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

static NSString* s_autoloadFile = @""; //readymap_flat.earth";

@interface StartViewerController ()

@end

@implementation StartViewerController

@synthesize pickerView;
@synthesize osgEarthViewController = _osgEarthViewController;

- (id)initWithNibName:(NSString *)nibNameOrNil bundle:(NSBundle *)nibBundleOrNil
{
    self = [super initWithNibName:nibNameOrNil bundle:nibBundleOrNil];
    if (self) {
        // Custom initialization
    }
    return self;
}

- (void)dealloc
{
    [super dealloc];
    [_osgEarthViewController release];
}

- (void)viewDidLoad
{
    [super viewDidLoad];
    // Do any additional setup after loading the view from its nib.
    
    fileArray = [[NSMutableArray alloc] init];
    
    std::string fullPath = osgDB::findDataFile("tests/readymap.earth");
    if(fullPath.empty()) fullPath = osgDB::findDataFile("readymap.earth");

    osgDB::DirectoryContents dirContents = osgDB::getDirectoryContents(osgDB::getFilePath(fullPath));
    for(unsigned int i=0; i<dirContents.size(); i++){
        //OSG_ALWAYS << "Dir item: " << dirContents[i] << std::endl;
        if(osgDB::getFileExtensionIncludingDot(dirContents[i]) == ".earth"){ 
            NSString* nsFile = [NSString stringWithCString:dirContents[i].c_str() encoding:NSASCIIStringEncoding];
            [fileArray addObject:nsFile];
        }
    }
    
    if([fileArray count] > 0) {
        currentSelection = [fileArray count]-1;
        [pickerView selectRow:currentSelection inComponent:0 animated:NO];
    }

    if(s_autoloadFile != nil && [s_autoloadFile length] > 0)
        [self loadEarthView:s_autoloadFile];
}

- (void)viewDidUnload
{
    [super viewDidUnload];
    // Release any retained subviews of the main view.
    self.pickerView = nil;
    [self.osgEarthViewController stopAnimation]; 
}

- (BOOL)shouldAutorotateToInterfaceOrientation:(UIInterfaceOrientation)interfaceOrientation
{
    return (interfaceOrientation == UIInterfaceOrientationPortrait);
}

-(IBAction)onStartViewer
{
    [self loadEarthView:[fileArray objectAtIndex:currentSelection]];
}

-(void) loadEarthView:(NSString*)aFile
{
    if(self.osgEarthViewController != nil) {
        [self.osgEarthViewController release];
        self.osgEarthViewController = nil;
    }
    
    self.osgEarthViewController = [[ViewController alloc] intWithFileName:aFile];
    [self.osgEarthViewController startAnimation]; 
    [self.view addSubview:self.osgEarthViewController.view];
}

- (void)startAnimation
{
    if(self.osgEarthViewController){
        [self.osgEarthViewController startAnimation]; 
    }
}
- (void)stopAnimation
{
    if(self.osgEarthViewController){
        [self.osgEarthViewController stopAnimation]; 
    }
}

#pragma mark - PickerView Delegates

- (NSInteger)numberOfComponentsInPickerView:(UIPickerView *)pickerView;
{
    return 1;
}

- (void)pickerView:(UIPickerView *)pickerView didSelectRow:(NSInteger)row inComponent:(NSInteger)component
{
    //mlabel.text=    [arrayNo objectAtIndex:row];
    currentSelection = row;
}

- (NSInteger)pickerView:(UIPickerView *)pickerView numberOfRowsInComponent:(NSInteger)component;
{
    return [fileArray count];
}

- (NSString *)pickerView:(UIPickerView *)pickerView titleForRow:(NSInteger)row forComponent:(NSInteger)component;
{
    return [fileArray objectAtIndex:row];
}


@end
