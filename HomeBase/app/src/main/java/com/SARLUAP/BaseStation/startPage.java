package com.sarLuap.baseStation;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import android.Manifest;
import android.annotation.SuppressLint;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.graphics.drawable.Animatable2;
import android.graphics.drawable.AnimatedVectorDrawable;
import android.graphics.drawable.Drawable;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.Toast;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicBoolean;

import dji.common.error.DJIError;
import dji.common.error.DJISDKError;
import dji.common.remotecontroller.HardwareState;
import dji.log.DJILog;
import dji.sdk.base.BaseComponent;
import dji.sdk.base.BaseProduct;
import dji.sdk.battery.Battery;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.sdkmanager.DJISDKInitEvent;
import dji.sdk.sdkmanager.DJISDKManager;

public class startPage extends AppCompatActivity implements View.OnClickListener{

    private static final String TAG = startPage.class.getName();

    private static final String[] REQUIRED_PERMISSION_LIST = new String[]{
            Manifest.permission.VIBRATE,
            Manifest.permission.INTERNET,
            Manifest.permission.ACCESS_WIFI_STATE,
            Manifest.permission.WAKE_LOCK,
            Manifest.permission.ACCESS_COARSE_LOCATION,
            Manifest.permission.ACCESS_NETWORK_STATE,
            Manifest.permission.ACCESS_FINE_LOCATION,
            Manifest.permission.ACCESS_COARSE_LOCATION,
            Manifest.permission.CHANGE_WIFI_STATE,
            Manifest.permission.WRITE_EXTERNAL_STORAGE,
            Manifest.permission.BLUETOOTH,
            Manifest.permission.BLUETOOTH_ADMIN,
            Manifest.permission.READ_EXTERNAL_STORAGE,
            Manifest.permission.READ_PHONE_STATE,
    };

    private final List<String> missingPermission = new ArrayList<>();
    private final AtomicBoolean isRegistrationInProgress = new AtomicBoolean(false);
    private static final int REQUEST_PERMISSION_CODE = 12345;

    private Button mBtnOpen;
    private ImageView mDottedLine1;
    private ImageView mDottedLine2;

    private AnimatedVectorDrawable aDottedLine1;
    private AnimatedVectorDrawable aDottedLine2;

    private boolean needAnimationDottedLine1;
    private boolean needAnimationDottedLine2;

    private int OSDKReady;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        checkAndRequestPermissions();

        setContentView(R.layout.activity_start_page);

        IntentFilter filter = new IntentFilter();
        filter.addAction(DroneConnection.FLAG_CONNECTION_CHANGE);
        registerReceiver(mReceiver, filter);

        initUi();

        CustomFun.HideNavBar(this);
    }

    /**
     * Checks if there is any missing permissions, and
     * requests runtime permission if needed.
     */
    private void checkAndRequestPermissions() {
        // Check for permissions
        for (String eachPermission : REQUIRED_PERMISSION_LIST) {
            if (ContextCompat.checkSelfPermission(this, eachPermission) != PackageManager.PERMISSION_GRANTED) {
                missingPermission.add(eachPermission);
            }
        }
        // Request for missing permissions
        if (!missingPermission.isEmpty()) {
            CustomFun.showToast("Need to grant the permissions!", getApplicationContext());
            ActivityCompat.requestPermissions(this,
                    missingPermission.toArray(new String[missingPermission.size()]),
                    REQUEST_PERMISSION_CODE);
        }

    }

    /**
     * Result of runtime permission request
     */
    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        // Check for granted permission and remove from missing list
        if (requestCode == REQUEST_PERMISSION_CODE) {
            for (int i = grantResults.length - 1; i >= 0; i--) {
                if (grantResults[i] == PackageManager.PERMISSION_GRANTED) {
                    missingPermission.remove(permissions[i]);
                }
            }
        }
        // If there is enough permission, we will start the registration
        if (missingPermission.isEmpty()) {
            startSDKRegistration();
        } else {
            CustomFun.showToast("Missing permissions!!!", getApplicationContext());
        }
    }

    private void startSDKRegistration() {
        if (isRegistrationInProgress.compareAndSet(false, true)) {
            AsyncTask.execute(new Runnable() {
                @Override
                public void run() {
//                    showToast( "registering, pls wait...");
                    DJISDKManager.getInstance().registerApp(getApplicationContext(), new DJISDKManager.SDKManagerCallback() {
                        @Override
                        public void onRegister(DJIError djiError) {
                            if (djiError == DJISDKError.REGISTRATION_SUCCESS) {
                                DJILog.e("App registration", DJISDKError.REGISTRATION_SUCCESS.getDescription());
                                DJISDKManager.getInstance().startConnectionToProduct();

                            } else {
                                CustomFun.showToast( "Register sdk fails, check network is available", getApplicationContext());
                            }
                            Log.v(TAG, djiError.getDescription());
                        }

                        @Override
                        public void onProductDisconnect() {
                            Log.d(TAG, "onProductDisconnect");

                        }
                        @Override
                        public void onProductConnect(BaseProduct baseProduct) {
                            Log.d(TAG, String.format("onProductConnect newProduct:%s", baseProduct));

                        }

                        @Override
                        public void onProductChanged(BaseProduct baseProduct) {

                        }

                        @Override
                        public void onComponentChange(BaseProduct.ComponentKey componentKey, BaseComponent oldComponent,
                                                      BaseComponent newComponent) {

                            if (newComponent != null) {
                                newComponent.setComponentListener(new BaseComponent.ComponentListener() {

                                    @Override
                                    public void onConnectivityChange(boolean isConnected) {
                                        Log.d(TAG, "onComponentConnectivityChanged: " + isConnected);
                                    }
                                });
                            }
                            Log.d(TAG,
                                    String.format("onComponentChange key:%s, oldComponent:%s, newComponent:%s",
                                            componentKey,
                                            oldComponent,
                                            newComponent));

                        }

                        @Override
                        public void onInitProcess(DJISDKInitEvent djisdkInitEvent, int i) {

                        }

                        @Override
                        public void onDatabaseDownloadProgress(long l, long l1) {

                        }
                    });
                }
            });
        }
    }

    @Override
    public void onResume() {
        CustomFun.HideNavBar(this);
        super.onResume();
    }

    @Override
    protected void onDestroy() {
        unregisterReceiver(mReceiver);
        super.onDestroy();
    }

    private void initUi(){
        mBtnOpen = findViewById(R.id.act_start_continue);
        mBtnOpen.setOnClickListener(this);
//        mBtnOpen.setEnabled(false);

        mDottedLine1 = findViewById(R.id.connection_img_dot1);
        mDottedLine2 = findViewById(R.id.connection_img_dot2);

        needAnimationDottedLine1 = true;
        needAnimationDottedLine2 = true;

        OSDKReady = 0;

        aDottedLine1 = (AnimatedVectorDrawable) mDottedLine1.getDrawable();
        aDottedLine1.registerAnimationCallback(new Animatable2.AnimationCallback() {
            @Override
            public void onAnimationEnd(Drawable drawable) {
                super.onAnimationEnd(drawable);
                if(needAnimationDottedLine1){
                    aDottedLine1.start();
                }
            }
        });
        aDottedLine1.start();

        aDottedLine2 = (AnimatedVectorDrawable) mDottedLine2.getDrawable();
        aDottedLine2.registerAnimationCallback(new Animatable2.AnimationCallback() {
            @Override
            public void onAnimationEnd(Drawable drawable) {
                super.onAnimationEnd(drawable);
                if(needAnimationDottedLine2){
                    aDottedLine2.start();
                }
                else{
                    mBtnOpen.setEnabled(true);
                }
            }
        });
        aDottedLine2.start();

        final Handler handler = new Handler(Looper.getMainLooper());
        handler.post(new Runnable() {
            @Override
            public void run() {
                if (DroneConnection.isControllerConnected()){
                    needAnimationDottedLine1 = false;
                }
                else{
                    handler.postDelayed(this, 100);
                }

            }
        });

        handler.post(new Runnable() {
            @Override
            public void run() {
                switch (OSDKReady){
                    case 0: {
                        if (DroneConnection.isDroneConnected() && DroneConnection.getFlightConInstance() != null) {
                            DroneConnection.getFlightConInstance().setOnboardSDKDeviceDataCallback(bytes -> {
                                if (bytes.length == 0) {
                                    CustomFun.showToast("ERROR: received nothing", getApplicationContext());
                                } else if (Byte.toUnsignedInt(bytes[0]) == 200 && Byte.toUnsignedInt(bytes[1]) == 0) {
                                    OSDKReady = 2;
                                }
                            });
                            OSDKReady = 1;
                        }
                    }

                    case 1:{
                        if (DroneConnection.isDroneConnected() && DroneConnection.getFlightConInstance() != null) {
                            byte[] sendData = {(byte) 0xC8};
                            DroneConnection.getFlightConInstance().sendDataToOnboardSDKDevice(sendData, djiError -> {
                                if (djiError != null) {
                                    CustomFun.showToast(djiError.getDescription(), getApplicationContext());
                                }
                            });
                        }
                        handler.postDelayed(this, 500);
                        break;
                    }

                    case 2:{
                        needAnimationDottedLine2 = false;
                        break;
                    }

                    default:{
                        CustomFun.showToast("What happened?", getApplicationContext());
                    }
                }
            }
        });

    }

    protected BroadcastReceiver mReceiver = new BroadcastReceiver() {

        @Override
        public void onReceive(Context context, Intent intent) {
            initUi();
        }
    };

    @Override
    public void onClick(View v) {
        if (v.getId() == R.id.act_start_continue){
//            Intent intent = new Intent(this, MainActivity.class);
            Intent intent = new Intent(this, LiveMapActivity.class);
            startActivity(intent);
            initUi();
        }
    }

}