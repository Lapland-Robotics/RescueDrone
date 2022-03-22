package com.SARLUAP.BaseStation;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;

import java.util.List;
import java.util.Objects;

import dji.common.error.DJIError;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.flightcontroller.LEDsSettings;
import dji.common.flightcontroller.LocationCoordinate3D;
import dji.common.util.CommonCallbacks;
import dji.sdk.base.BaseProduct;
import dji.sdk.battery.Battery;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.remotecontroller.RemoteController;

public class MainActivity extends Activity implements View.OnClickListener {
    private static final String TAG = MainActivity.class.getName();

    private boolean gps;

    private Handler handler;

    private BaseProduct mProduct;
    private RemoteController mRemoteCon;
    private FlightController mFlightCon;

    protected TextView ProductTV;
    protected TextView DroneBatteryTV;
    protected TextView ConBatteryTV;
    protected TextView GPSTV;

    protected Button btn_back;
    protected Button btn_esc_beep;
    protected Button btn_map;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.main_activity);

        handler = new Handler();

        initValues();
        initUi();

        IntentFilter filter = new IntentFilter();
        filter.addAction(DroneConnection.FLAG_CONNECTION_CHANGE);
        registerReceiver(mReceiver, filter);

        custom_fun.HideNavBar(this);
    }

    @Override
    protected void onDestroy() {
        unregisterReceiver(mReceiver);
        super.onDestroy();
    }
/*
    public void onReturn(View view){
        Log.e(TAG, "onReturn");
        this.finish();
    }
*/
    @Override
    public void onResume() {
        custom_fun.HideNavBar(this);
        super.onResume();
    }
    private void initValues(){
        mProduct = DroneConnection.getProductInstance();
        mRemoteCon = DroneConnection.getRemoteConInstance();
        mFlightCon = DroneConnection.getFlightConInstance();
    }

    private void initUi(){
        ProductTV = (TextView) findViewById(R.id.tv_product_information);
        DroneBatteryTV = (TextView) findViewById(R.id.tv_drone_battery_information);
        ConBatteryTV = (TextView) findViewById(R.id.tv_controller_battery_information);
        GPSTV = (TextView) findViewById(R.id.tv_gps_information);

        btn_esc_beep = (Button) findViewById(R.id.btn_ESCBeep);
        btn_back = (Button) findViewById(R.id.btn_back);
        btn_map = (Button) findViewById(R.id.btn_map);

        btn_esc_beep.setOnClickListener(this);
        btn_back.setOnClickListener(this);
        btn_map.setOnClickListener(this);

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
        if (null != mProduct && mProduct.isConnected()) {
            Log.v(TAG, "refreshSDK: True");

            if (null != mProduct.getModel()) {
                ProductTV.setText("" + mProduct.getModel().getDisplayName());
            } else {
                ProductTV.setText(R.string.connection_drone_loose);
            }

            if (null != mProduct.getBatteries()){
                List<Battery> batteries = mProduct.getBatteries();
                if(batteries.size() == 1){
                    Battery battery = batteries.get(0);
                    battery.setStateCallback(batteryState ->
                            DroneBatteryTV.setText("Remaining charge: " + batteryState.getChargeRemaining() +
                                          "mAh (" + batteryState.getChargeRemainingInPercent() + "%)\n" +
                                          "Current draw: " + batteryState.getCurrent() * - 1 + "mA\n" +
                                          "Temperature: " + batteryState.getTemperature() + "°C"));
                }
                else{
                    DroneBatteryTV.setText("Multiple batteries detected");
                }
            }

        } else {
            Log.v(TAG, "refreshSDK: False");
            finish();
        }

        if (null != mRemoteCon && mRemoteCon.isConnected()){
            mRemoteCon.setChargeRemainingCallback(batteryState ->
                    ConBatteryTV.setText("remaining charge: " + batteryState.getRemainingChargeInmAh() +
                                         "mAh (" + batteryState.getRemainingChargeInPercent() + "%)"));
        }
        else {
            ConBatteryTV.setText(R.string.connection_controller_loose);
        }

        if(!gps){
            GPSTV.setText(R.string.gps_disables);
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

            case R.id.btn_ESCBeep: {
                break;
            }

            case R.id.btn_map: {
                gps = true;
                GPSTV.setText(R.string.gps_enabled);
                mFlightCon.setStateCallback(flightControllerState -> {
                    LocationCoordinate3D location = flightControllerState.getAircraftLocation();
                    GPSTV.setText("lat: " + location.getLatitude() + " long: " + location.getLongitude() + " alt: " + location.getAltitude() +
                                  "\nSatCount: " + flightControllerState.getSatelliteCount() + " SignalLevel: " + flightControllerState.getGPSSignalLevel());
                });
                break;
            }

            default:
                break;
        }
    }

    private void showToast(final String toastMsg) {

        Handler handler = new Handler(Looper.getMainLooper());
        handler.post(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(getApplicationContext(), toastMsg, Toast.LENGTH_LONG).show();
            }
        });
    }
}
