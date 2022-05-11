package com.sarLuap.baseStation;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.media.MediaPlayer;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.Parcelable;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;

import java.io.Serializable;
import java.util.List;
import java.util.Objects;
import java.util.Timer;
import java.util.TimerTask;

import dji.common.flightcontroller.LocationCoordinate3D;
import dji.common.remotecontroller.HardwareState;
import dji.sdk.base.BaseProduct;
import dji.sdk.battery.Battery;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.remotecontroller.RemoteController;

public class MainActivity extends AppCompatActivity implements View.OnClickListener {
    private static final String TAG = MainActivity.class.getName();

    private boolean gps;
    public static boolean inSearch;

    private Handler handler;

    private BaseProduct mProduct;
    private RemoteController mRemoteCon;
    private FlightController mFlightCon;

    protected TextView productTV;
    protected TextView droneBatteryTV;
    protected TextView conBatteryTV;

    protected Button btnBack;
    protected Button btnMap;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.main_activity);
        inSearch = false;
        handler = new Handler();

        initValues();
        initUi();

        IntentFilter filter = new IntentFilter();
        filter.addAction(DroneConnection.FLAG_CONNECTION_CHANGE);
        registerReceiver(mReceiver, filter);

        CustomFun.HideNavBar(this);

    }

    @Override
    protected void onDestroy() {
        unregisterReceiver(mReceiver);
        super.onDestroy();
    }

    @Override
    public void onResume() {
        CustomFun.HideNavBar(this);
        refreshSDKRelativeUI();
        super.onResume();
    }
    private void initValues(){
        mProduct = DroneConnection.getProductInstance();
        mRemoteCon = DroneConnection.getRemoteConInstance();
        mFlightCon = DroneConnection.getFlightConInstance();
    }

    private void initUi(){
        productTV = (TextView) findViewById(R.id.tv_product_information);
        droneBatteryTV = (TextView) findViewById(R.id.tv_drone_battery_information);
        conBatteryTV = (TextView) findViewById(R.id.tv_controller_battery_information);

        btnBack = (Button) findViewById(R.id.btn_back);
        btnMap = (Button) findViewById(R.id.btn_map);

        btnBack.setOnClickListener(this);
        btnMap.setOnClickListener(this);

        gps = false;

        refreshSDKRelativeUI();
    }

    protected BroadcastReceiver mReceiver = new BroadcastReceiver() {

        @Override
        public void onReceive(Context context, Intent intent) {
            refreshSDKRelativeUI();
        }
    };

    @SuppressLint("SetTextI18n")
    private void refreshSDKRelativeUI() {
        if (mProduct != null && mProduct.isConnected() && mFlightCon != null && mFlightCon.isConnected()) {
            Log.v(TAG, "refreshSDK: True");

            if (null != mProduct.getModel()) {
                productTV.setText("" + mProduct.getModel().getDisplayName());
            } else {
                productTV.setText(R.string.connection_drone_loose);
            }

            if (null != mProduct.getBatteries()){
                List<Battery> batteries = mProduct.getBatteries();
                if(batteries.size() == 1){
                    Battery battery = batteries.get(0);
                    battery.setStateCallback(batteryState ->
                            droneBatteryTV.setText("Remaining charge: " + batteryState.getChargeRemaining() +
                                          "mAh (" + batteryState.getChargeRemainingInPercent() + "%)\n" +
                                          "Current draw: " + batteryState.getCurrent() * - 1 + "mA\n" +
                                          "Temperature: " + batteryState.getTemperature() + "°C"));
                }
                else{
                    droneBatteryTV.setText("Multiple batteries detected");
                }
            }

        } else {
            Log.v(TAG, "refreshSDK: False");
            finish();
        }

        if (mRemoteCon != null && mRemoteCon.isConnected()){
            mRemoteCon.setChargeRemainingCallback(batteryState ->
                    conBatteryTV.setText("remaining charge: " + batteryState.getRemainingChargeInmAh() +
                                         "mAh (" + batteryState.getRemainingChargeInPercent() + "%)"));
            mRemoteCon.setHardwareStateCallback(new HardwareState.HardwareStateCallback() {
                @Override
                public void onUpdate(@NonNull HardwareState hardwareState) {
                    if(inSearch && (Objects.requireNonNull(hardwareState.getLeftStick()).getVerticalPosition() != 0 || hardwareState.getLeftStick().getHorizontalPosition() != 0 || Objects.requireNonNull(hardwareState.getRightStick()).getHorizontalPosition() != 0 || hardwareState.getRightStick().getVerticalPosition() != 0)){
                        byte[] sendData = {(byte) 0x02};
                        mFlightCon.sendDataToOnboardSDKDevice(sendData, djiError -> {
                            if(djiError != null){
                                CustomFun.showToast(djiError.getDescription(), getApplicationContext());
                            }
                        });
                        CustomFun.showToast("manual override", getApplicationContext());
                        inSearch = false;
                    }
                }
            });
        }
        else {
            conBatteryTV.setText(R.string.connection_controller_loose);
        }
    }

    @SuppressLint({"NonConstantResourceId", "SetTextI18n"})
    @Override
    public void onClick(View v) {
        switch (v.getId()) {

            case R.id.btn_back: {
                finish();
                break;
            }

            case R.id.btn_map: {
                Intent intent = new Intent(this, LiveMapActivity.class);
                startActivity(intent);
                refreshSDKRelativeUI();

                break;
            }

            default:
                break;
        }
    }
}
