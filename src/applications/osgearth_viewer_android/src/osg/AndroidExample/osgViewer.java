package osg.AndroidExample;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.res.Resources;
import android.graphics.Color;
import android.graphics.PointF;
import android.os.Bundle;
import android.util.FloatMath;
import android.util.Log;
import android.view.KeyEvent;
import android.view.LayoutInflater;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.WindowManager;
import android.view.View.OnClickListener;
import android.view.inputmethod.InputMethodManager;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ImageButton;

import java.io.File;

public class osgViewer extends Activity implements View.OnTouchListener, View.OnKeyListener
{
	enum moveTypes { NONE , DRAG, MDRAG, ZOOM ,ACTUALIZE}
	enum navType { PRINCIPAL , SECONDARY }
	enum lightType { ON , OFF }
		
	moveTypes mode=moveTypes.NONE;
	navType navMode = navType.PRINCIPAL;
	lightType lightMode = lightType.ON;
	
	PointF oneFingerOrigin = new PointF(0,0);
	long timeOneFinger=0;
	PointF twoFingerOrigin = new PointF(0,0);
	long timeTwoFinger=0;
	float distanceOrigin;
	
	private static final String TAG = "OSG Activity";
	//Ui elements
    EGLview mView;
    Button uiCenterViewButton;
    
    //Dialogs
    AlertDialog removeLayerDialog;
    AlertDialog loadLayerAddress;

    //Main Android Activity life cycle
    @Override protected void onCreate(Bundle icicle)
    {
        super.onCreate(icicle);
        setContentView(R.layout.ui_layout_gles);
        
        // get the gl view and attach touch listeners
		mView= (EGLview) findViewById(R.id.surfaceGLES);
		mView.setOnTouchListener(this);
		mView.setOnKeyListener(this);
	        
	    uiCenterViewButton = (Button) findViewById(R.id.uiButtonCenter);
	    uiCenterViewButton.setOnClickListener(uiListenerCenterView);
    }
    
    @Override protected void onPause() {
        super.onPause();
        mView.onPause();
    }
    @Override protected void onResume() {
        super.onResume();
        mView.onResume();
    }
    
    //Main view event processing
    @Override
	public boolean onKey(View v, int keyCode, KeyEvent event) {
		
		return true;
	}
    @Override
    public boolean onKeyDown(int keyCode, KeyEvent event){
    	//DO NOTHING this will render useless every menu key except Home
    	int keyChar= event.getUnicodeChar();
    	osgNativeLib.keyboardDown(keyChar);
    	return true;
    }
    @Override
    public boolean onKeyUp(int keyCode, KeyEvent event){
    	switch (keyCode){
    	case KeyEvent.KEYCODE_BACK:
    		super.onDestroy();
    		this.finish();
    		break;
    	case KeyEvent.KEYCODE_SEARCH:
    		break;
    	case KeyEvent.KEYCODE_MENU:
    		this.openOptionsMenu();
    		break;
    	default:
    		int keyChar= event.getUnicodeChar();
    		osgNativeLib.keyboardUp(keyChar);    		
    	}
    	
    	return true;
    }
    @Override
    public boolean onTouch(View v, MotionEvent event) {
    	
    	//dumpEvent(event);
    	
    	long time_arrival = event.getEventTime();
    	int n_points = event.getPointerCount();
    	int action = event.getAction() & MotionEvent.ACTION_MASK;
    	
    	switch(n_points){
    	case 1:
    		switch(action){
    		case MotionEvent.ACTION_DOWN:
	    		mode = moveTypes.DRAG;
	    		
	    		osgNativeLib.mouseMoveEvent(event.getX(0), event.getY(0));
	    		if(navMode==navType.PRINCIPAL)
	    			osgNativeLib.mouseButtonPressEvent(event.getX(0), event.getY(0), 2);
	    		else
	    			osgNativeLib.mouseButtonPressEvent(event.getX(0), event.getY(0), 1);
	    		
	    		oneFingerOrigin.x=event.getX(0);
	    		oneFingerOrigin.y=event.getY(0);
    			break;
    		case MotionEvent.ACTION_CANCEL:
    			switch(mode){
    			case DRAG:
    				osgNativeLib.mouseMoveEvent(event.getX(0), event.getY(0));
    				if(navMode==navType.PRINCIPAL)
    					osgNativeLib.mouseButtonReleaseEvent(event.getX(0), event.getY(0), 2);
    				else
    					osgNativeLib.mouseButtonReleaseEvent(event.getX(0), event.getY(0), 1);
    				break;
    			default :
    				Log.e(TAG,"There has been an anomaly in touch input 1point/action");
    			}
    			mode = moveTypes.NONE;
    			break;
    		case MotionEvent.ACTION_MOVE:
    			
    			osgNativeLib.mouseMoveEvent(event.getX(0), event.getY(0));
    			
    			oneFingerOrigin.x=event.getX(0);
	    		oneFingerOrigin.y=event.getY(0);
	    		
    			break;
    		case MotionEvent.ACTION_UP:
    			switch(mode){
    			case DRAG:
    				if(navMode==navType.PRINCIPAL)
    					osgNativeLib.mouseButtonReleaseEvent(event.getX(0), event.getY(0), 2);
    				else
    					osgNativeLib.mouseButtonReleaseEvent(event.getX(0), event.getY(0), 1);
    				break;
    			default :
    				Log.e(TAG,"There has been an anomaly in touch input 1 point/action");
    			}
    			mode = moveTypes.NONE;
    			break;
    		default :
    			Log.e(TAG,"1 point Action not captured");	
    		}
    		break;
    	case 2:
    		switch (action){
    		case MotionEvent.ACTION_POINTER_DOWN:
    			//Free previous Action
    			switch(mode){
    			case DRAG:
    				if(navMode==navType.PRINCIPAL)
    					osgNativeLib.mouseButtonReleaseEvent(event.getX(0), event.getY(0), 2);
    				else
    					osgNativeLib.mouseButtonReleaseEvent(event.getX(0), event.getY(0), 1);
    				break;
    			}
    			mode = moveTypes.ZOOM;
    			distanceOrigin = sqrDistance(event);
    			twoFingerOrigin.x=event.getX(1);
    			twoFingerOrigin.y=event.getY(1);
    			oneFingerOrigin.x=event.getX(0);
	    		oneFingerOrigin.y=event.getY(0);
    			
    			osgNativeLib.mouseMoveEvent(oneFingerOrigin.x,oneFingerOrigin.y);
    			osgNativeLib.mouseButtonPressEvent(oneFingerOrigin.x,oneFingerOrigin.y, 3);
    			osgNativeLib.mouseMoveEvent(oneFingerOrigin.x,oneFingerOrigin.y);
    			
    		case MotionEvent.ACTION_MOVE:
    			float distance = sqrDistance(event);
    			float result = distance-distanceOrigin;
    			distanceOrigin=distance;
    			
    			if(result>1||result<-1){
    	    		oneFingerOrigin.y=oneFingerOrigin.y+result;
    				osgNativeLib.mouseMoveEvent(oneFingerOrigin.x,oneFingerOrigin.y);
    			}
    			
    			break;
    		case MotionEvent.ACTION_POINTER_UP:
    			mode =moveTypes.NONE;
    			osgNativeLib.mouseButtonReleaseEvent(oneFingerOrigin.x,oneFingerOrigin.y, 3);
    			break;
    		case MotionEvent.ACTION_UP:
    			mode =moveTypes.NONE;
    			osgNativeLib.mouseButtonReleaseEvent(oneFingerOrigin.x,oneFingerOrigin.y, 3);
    			break;
    		default :
    			Log.e(TAG,"2 point Action not captured");
    		}
    		break;    		
    	}
			
		return true;
	}

    //Ui Listeners
    OnClickListener uiListenerCenterView = new OnClickListener() {
        public void onClick(View v) {
        	//Log.d(TAG, "Center View");
        	osgNativeLib.keyboardDown(32);
        	osgNativeLib.keyboardUp(32);
        }
    };
    
    //Menu
    
    //Android Life Cycle Menu
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.appmenu, menu);
        return super.onCreateOptionsMenu(menu);
    }
    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle item selection
        switch (item.getItemId()) {
        case R.id.menuShowKeyboard:
        	Log.d(TAG,"Keyboard");
        	InputMethodManager mgr= (InputMethodManager)this.getSystemService(Context.INPUT_METHOD_SERVICE);
    		mgr.toggleSoftInput(InputMethodManager.SHOW_IMPLICIT, 0);
            return true;
        default:
            return super.onOptionsItemSelected(item);
        }
    }
    
    //Utilities
    /** Show an event in the LogCat view, for debugging */
    private void dumpEvent(MotionEvent event) {
       String names[] = { "DOWN", "UP", "MOVE", "CANCEL", "OUTSIDE",
             "POINTER_DOWN", "POINTER_UP", "7?", "8?", "9?" };
       StringBuilder sb = new StringBuilder();
       int action = event.getAction();
       int actionCode = action & MotionEvent.ACTION_MASK;
       sb.append("event ACTION_").append(names[actionCode]);
       if (actionCode == MotionEvent.ACTION_POINTER_DOWN
             || actionCode == MotionEvent.ACTION_POINTER_UP) {
          sb.append("(pid ").append(
                action >> MotionEvent.ACTION_POINTER_ID_SHIFT);
          sb.append(")");
       }
       sb.append("[");
       for (int i = 0; i < event.getPointerCount(); i++) {
          sb.append("#").append(i);
          sb.append("(pid ").append(event.getPointerId(i));
          sb.append(")=").append((int) event.getX(i));
          sb.append(",").append((int) event.getY(i));
          if (i + 1 < event.getPointerCount())
             sb.append(";");
       }
       sb.append("]");
       //Log.d(TAG, sb.toString());
    }
    private float sqrDistance(MotionEvent event) {
        float x = event.getX(0) - event.getX(1);
        float y = event.getY(0) - event.getY(1);
        return (float)(Math.sqrt(x * x + y * y));
     }
	
}