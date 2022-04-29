package com.sarLuap.baseStation;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.media.MediaPlayer;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;

import org.bouncycastle.asn1.eac.UnsignedInteger;

import java.nio.Buffer;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import dji.common.error.DJIError;
import dji.common.flightcontroller.LocationCoordinate3D;
import dji.common.util.CommonCallbacks;
import dji.sdk.base.BaseProduct;
import dji.sdk.battery.Battery;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.remotecontroller.RemoteController;


public class LiveMapActivity extends AppCompatActivity implements View.OnClickListener{

    private LiveMapClass liveMapClass;

    private Button btnBack;
    private Button btnTrack;
    private Button btnStartSearch;
    private Button btnForceStop;
    private Button btnStartCoordinates;

    private TextView tvDroneBat;
    private TextView tvConBat;

    private double lastLong;
    private double lastLat;
    private double lastHead;

    private FlightController mFlightCon;

    private sound_timer beep;
    private sound_timer battery;
    private sound_timer found_person;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.live_map_view);

        initUi();

        // IntentFilter filter = new IntentFilter();
        // filter.addAction(DroneConnection.FLAG_CONNECTION_CHANGE);
        // registerReceiver(mReceiver, filter);

        CustomFun.HideNavBar(this);

        beep = new sound_timer();
        battery = new sound_timer();
        found_person = new sound_timer();

        beep.initSound(this, R.raw.one_beep);
        battery.initSound(this, R.raw.one_battery_warning);
        found_person.initSound(this, R.raw.one_found_person);
    }

    @Override
    protected void onStart () {
        super.onStart();
    }

    @Override
    public void onResume() {
        CustomFun.HideNavBar(this);
        super.onResume();
    }

    @Override
    protected void onStop () {
        super.onStop();
    }

    @Override
    public void onLowMemory() {
        super.onLowMemory();
    }

    @Override
    protected void onDestroy () {
        super.onDestroy();
        liveMapClass.onDestroy();
    }

    @SuppressLint("SetTextI18n")
    private void initUi(){
        tvConBat = findViewById(R.id.tv_controller_battery_information);
        tvDroneBat = findViewById(R.id.tv_drone_battery_information);

        btnBack = findViewById(R.id.btn_back);
        btnTrack = findViewById(R.id.btn_position_tracking);
        btnStartSearch = findViewById(R.id.btn_start_search);
        btnForceStop = findViewById(R.id.btn_force_Stop);
        btnStartCoordinates = findViewById(R.id.btn_start_coordinates);

        btnBack.setOnClickListener(this);
        btnTrack.setOnClickListener(this);
        btnStartSearch.setOnClickListener(this);
        btnForceStop.setOnClickListener(this);
        btnStartCoordinates.setOnClickListener(this);

        liveMapClass = new LiveMapClass();
        liveMapClass.mapInit(this, findViewById(R.id.map_view), btnTrack);

        refreshSDKRelativeUI();
    }
    @SuppressLint("SetTextI18n")
    private void refreshSDKRelativeUI() {
        BaseProduct mProduct = DroneConnection.getProductInstance();
        RemoteController mRemoteCon = DroneConnection.getRemoteConInstance();
        mFlightCon = DroneConnection.getFlightConInstance();

        if (mProduct != null && mProduct.isConnected() && mFlightCon != null && mFlightCon.isConnected()) {

            if (null != mProduct.getBatteries()){
                List<Battery> batteries = mProduct.getBatteries();
                if(batteries.size() == 1){
                    Battery battery = batteries.get(0);
                    battery.setStateCallback(batteryState ->
                            tvDroneBat.setText("battery Drone "  + batteryState.getChargeRemainingInPercent() + "%"));
                }
                else{
                    tvDroneBat.setText("Multiple batteries detected");
                }
            }
            mFlightCon.setOnboardSDKDeviceDataCallback(bytes -> {
                if(bytes.length == 0){
                    showToast("ERROR: received nothing");
                }
                else {
                    //showToast("length: " + bytes.length + " data: " + Arrays.toString(bytes));
                    switch (Byte.toUnsignedInt(bytes[1])){
                        case 0:
                            switch (Byte.toUnsignedInt(bytes[0])) {
                                case 0:
                                    showToast("IDK");
                                    break;

                                case 2:
                                    showToast("Successfully handed over controls");
                                    break;

                                case 10:
                                    showToast("Successfully takeoff");
                                    beep.stopTimer();
                                    break;

                                case 11:
                                    showToast("Successfully land");
                                    beep.stopTimer();
                                    break;

                                case 20:
                                    showToast("successfully start Search");
                                    break;
                            }
                            break;

                        case 2:
                            showToast("Can't give controls");
                            break;

                        case 3:
                            showToast("landing in progress");
                            if(beep.isPlaying()){
                                beep.stopTimer();
                            }
                            beep.startTimer(200);
                            break;

                        case 10:
                            showToast("error with takeoff");
                            beep.stopTimer();
                            break;

                        case 11:
                            showToast("error with land");
                            beep.stopTimer();
                            break;

                        case 12:
                            showToast("no need to tell again im already there ");
                            beep.stopTimer();
                            break;

                        case 13:
                            showToast("chill i'm already on the ground");
                            beep.stopTimer();
                            break;

                        case 20:
                            showToast("maybe give me some instructions???");
                            break;
                    }
                }
            });

        } else {
//            finish();
        }

        if (null != mRemoteCon && mRemoteCon.isConnected()){
            mRemoteCon.setChargeRemainingCallback(batteryState ->
                    tvConBat.setText("battery Controller "  + batteryState.getRemainingChargeInPercent() + "%"));
        }
        else {
            tvConBat.setText(R.string.connection_controller_loose);
        }

        assert mFlightCon != null;
        mFlightCon.setStateCallback(flightControllerState -> {
            LocationCoordinate3D location = flightControllerState.getAircraftLocation();
            //GPSTV.setText("lat: " + location.getLatitude() + " long: " + location.getLongitude() + " alt: " + location.getAltitude() +
                    //"\nSatCount: " + flightControllerState.getSatelliteCount() + " SignalLevel: " + flightControllerState.getGPSSignalLevel());
            double headTmp = mFlightCon.getCompass().getHeading();
            if(Double.isNaN(location.getLongitude()) || Double.isNaN(location.getLatitude()) || Double.isNaN(headTmp)){
                if(lastLat != 0 && lastLat != 0){
                    liveMapClass.updateDrone(lastLong,lastLat,0,true);
                }
                else {
                    liveMapClass.updateDrone(0,0,0,false);
                }
            }
            else{
                lastHead = headTmp;
                lastLat = location.getLatitude();
                lastLong = location.getLongitude();
                liveMapClass.updateDrone(lastLong, lastLat, 0, true);
            }
        });
    }

    public static double toDouble(byte[] bytes) {
        byte[] ba = new byte[8];
        for(int i = 0; i < 8 ; i++){
            ba[i] = bytes[7-i];
        }
        return ByteBuffer.wrap(ba).getDouble();
    }

    @SuppressLint({"NonConstantResourceId", "SetTextI18n"})
    @Override
    public void onClick(View v) {
        switch (v.getId()) {

            case R.id.btn_back: {
                finish();
                break;
            }

            case R.id.btn_position_tracking:{
                if (liveMapClass.getPosTrackingON()) {
                    liveMapClass.dismissedCameraTracking();
                } else {
                    liveMapClass.activateCameraTrakingActivate();
                }
                break;
            }

            case R.id.btn_start_search:{
                byte[] sendData = {(byte) 0x14};
                mFlightCon.sendDataToOnboardSDKDevice(sendData, djiError -> {
                    if(djiError != null){
                        showToast(djiError.getDescription());
                    }
                });
                MainActivity.inSearch = true;
                if(!beep.isPlaying()) {
                    beep.startTimer(1000);
                }

                break;
            }

            case R.id.btn_force_Stop:{
                byte[] sendData = {(byte) 0x15};
                mFlightCon.sendDataToOnboardSDKDevice(sendData, djiError -> {
                    if(djiError != null){
                        showToast(djiError.getDescription());
                    }
                });
                break;
            }

            case R.id.btn_start_coordinates:{
                byte[] lat = doubleToByte(66.48211996577223);
                byte[] lon = doubleToByte(25.72159975108791);
                byte[] sendData = new byte[lat.length + lon.length + 2];
                sendData[0] = (byte) 22;
                for(int i = 0; i < sendData.length - 2; i ++){
                    sendData[i + 2] = i < lat.length? lat[i] : lon[i - lat.length];
                }
                mFlightCon.sendDataToOnboardSDKDevice(sendData, djiError -> {
                    if(djiError != null){
                        showToast(djiError.getDescription());
                    }
                });
            }

            default:
                break;
        }
    }

    @NonNull
    private byte[] doubleToByte(double input){
        byte[] output = new byte[8];
        long lng = Double.doubleToLongBits(input);
        for(int i = 0; i < 8; i++){
            output[7 - i] = (byte) ((lng >> ((7 - i) * 8)) & 0xff);
        }
        return output;
    }

    private void showToast(final String toastMsg) {

        Handler handler = new Handler(Looper.getMainLooper());
        handler.post(() -> Toast.makeText(getApplicationContext(), toastMsg, Toast.LENGTH_SHORT).show());
    }
}
