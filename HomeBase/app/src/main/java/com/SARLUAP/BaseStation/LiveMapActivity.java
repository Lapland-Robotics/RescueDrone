package com.sarLuap.baseStation;

import static java.lang.Math.abs;

import android.annotation.SuppressLint;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;

import com.mapbox.geojson.Point;

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

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
    private Button btnSendArea;
    private Button btnRemoveArea;

    private TextView tvDroneBat;
    private TextView tvConBat;

    private double lastLong;
    private double lastLat;
    private double lastHead;

    private FlightController mFlightCon;

    private sound_timer beep;
    private sound_timer battery;
    private sound_timer found_person;

    private long updateTime = 0;
    private static final long updateFrequency = 40;
    private static boolean updatingDrone = false;

    private AtomicBoolean error = new AtomicBoolean(false);
    private AtomicBoolean nextAreaCoordinate = new AtomicBoolean(false);

    private List<Point> area;
    private int counter;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.live_map_view);

        initUi();

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
        btnStartCoordinates = findViewById(R.id.btn_send_area);
        btnSendArea = findViewById(R.id.btn_remove_last);
        btnRemoveArea = findViewById(R.id.btn_remove_area);

        btnBack.setOnClickListener(this);
        btnTrack.setOnClickListener(this);
        btnStartSearch.setOnClickListener(this);
        btnForceStop.setOnClickListener(this);
        btnStartCoordinates.setOnClickListener(this);
        btnSendArea.setOnClickListener(this);
        btnRemoveArea.setOnClickListener(this);

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
                    CustomFun.showToast("ERROR: received nothing", getApplicationContext());
                }
                else {
                    //showToast("length: " + bytes.length + " data: " + Arrays.toString(bytes));
                    switch (Byte.toUnsignedInt(bytes[1])){
                        case 0:
                            switch (Byte.toUnsignedInt(bytes[0])) {
                                case 0:
                                    CustomFun.showToast("IDK", getApplicationContext());
                                    break;

                                case 2:
                                    CustomFun.showToast("Successfully handed over controls", getApplicationContext());
                                    break;

                                case 10:
                                    CustomFun.showToast("Successfully takeoff", getApplicationContext());
                                    beep.stopTimer();
                                    break;

                                case 11:
                                    CustomFun.showToast("Successfully land", getApplicationContext());
                                    beep.stopTimer();
                                    break;

                                case 20:
                                    CustomFun.showToast("successfully start Search", getApplicationContext());
                                    break;

                                case 22:
//                                    CustomFun.showToast("yes" + counter, getApplicationContext());
                                    if(abs(toDouble(Arrays.copyOfRange(bytes, 2, 10)) - area.get(counter).latitude()) < 0.002 && abs(toDouble(Arrays.copyOfRange(bytes, 10, 18)) - area.get(counter).longitude()) < 0.002){
                                        counter ++;
                                        nextAreaCoordinate.set(true);
                                    }
                                    else{
                                        error.set(true);
                                        CustomFun.showToast("got wrong coordinate callback", getApplicationContext());
                                    }
                                    break;

                                case 112:
                                    String tmp = "Found Person at lat: " + toDouble(Arrays.copyOfRange(bytes, 2, 10)) + " long: " + toDouble(Arrays.copyOfRange(bytes, 10, 18));
                                    CustomFun.showToast(tmp, getApplicationContext());
                                    break;
                            }
                            break;

                        case 2:
                            CustomFun.showToast("Can't give controls", getApplicationContext());
                            break;

                        case 3:
                            CustomFun.showToast("landing in progress", getApplicationContext());
                            if(beep.isPlaying()){
                                beep.stopTimer();
                            }
                            beep.startTimer(200);
                            break;

                        case 10:
                            CustomFun.showToast("error with takeoff", getApplicationContext());
                            beep.stopTimer();
                            break;

                        case 11:
                            CustomFun.showToast("error with land", getApplicationContext());
                            beep.stopTimer();
                            break;

                        case 12:
                            CustomFun.showToast("no need to tell again im already there ", getApplicationContext());
                            beep.stopTimer();
                            break;

                        case 13:
                            CustomFun.showToast("chill i'm already on the ground", getApplicationContext());
                            beep.stopTimer();
                            break;

                        case 20:
                            CustomFun.showToast("maybe give me some instructions???", getApplicationContext());
                            break;

                        case 22:
                            CustomFun.showToast("Onboard computer has some problems with the area coordinate", getApplicationContext());
                            break;

                    }
                }

                byte[] sendData = {(byte) 255};
                mFlightCon.sendDataToOnboardSDKDevice(sendData, tmp);
            });

            mFlightCon.setStateCallback(flightControllerState -> {
                if(!updatingDrone && (System.currentTimeMillis() - updateTime) > updateFrequency ) {
                    updatingDrone = true;
                    LocationCoordinate3D location = flightControllerState.getAircraftLocation();
                    float headTmp = mFlightCon.getCompass().getHeading();
                    if (Double.isNaN(location.getLongitude()) || Double.isNaN(location.getLatitude()) || Double.isNaN(headTmp)) {
                        if (lastLat != 0 && lastLong != 0) {
                            liveMapClass.updateDrone(lastLong, lastLat, headTmp);
                        } else {
                            liveMapClass.removeDrone();
                        }
                    } else {
                        lastHead = headTmp;
                        lastLat = location.getLatitude();
                        lastLong = location.getLongitude();
                        liveMapClass.updateDrone(lastLong, lastLat, headTmp);
                    }
                    updateTime = System.currentTimeMillis();
                    updatingDrone = false;
                }
            });

        } else {
            finish();
        }

        if (null != mRemoteCon && mRemoteCon.isConnected()){
            mRemoteCon.setChargeRemainingCallback(batteryState ->
                    tvConBat.setText("battery \nCon "  + batteryState.getRemainingChargeInPercent() + "%"));
        }
        else {
            tvConBat.setText("no controller");
        }
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
                    liveMapClass.activateCameraTrackingActivate();
                }
                break;
            }

            case R.id.btn_start_search:{
                byte[] sendData = {(byte) 20};
                mFlightCon.sendDataToOnboardSDKDevice(sendData, tmp);
                MainActivity.inSearch = true;
                if(!beep.isPlaying()) {
                    beep.startTimer(1000);
                }

                break;
            }

            case R.id.btn_force_Stop:{
                byte[] sendData = {(byte) 21};
                mFlightCon.sendDataToOnboardSDKDevice(sendData, tmp);
                break;
            }

            case R.id.btn_remove_last:{
                liveMapClass.removeLastPoint();
                break;
            }

            case R.id.btn_send_area:{
                counter = 0;
                area = liveMapClass.getArea();

                error.set(false);
                nextAreaCoordinate.set(true);

                while(!error.get() && counter < area.size()){
                    if(nextAreaCoordinate.get()) {
                        nextAreaCoordinate.set(false);
                        mFlightCon.sendDataToOnboardSDKDevice(encodeData(area.get(counter).latitude(), area.get(counter).longitude(), (byte) 22), tmp);
                    }
                }

                byte[] sendData = {(byte) 23};
                mFlightCon.sendDataToOnboardSDKDevice(sendData, tmp);

                CustomFun.showToast(error.get() ? "error with sending area" : "area successfully send", getApplicationContext());
                break;
            }

            case R.id.btn_remove_area:{
                liveMapClass.removePoints();
            }

            default:
                break;
        }
    }

    @NonNull
    private byte[] encodeData(double lat, double lon, byte command){
        byte[] Blat = doubleToByte(lat);
        byte[] Blon = doubleToByte(lon);
        byte[] sendData = new byte[Blat.length + Blon.length + 1];

        sendData[0] = command;
        for(int i = 0; i < sendData.length - 1; i ++){
            sendData[i + 1] = i < Blat.length? Blat[i] : Blon[i - Blat.length];
        }
        return sendData;
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

    CommonCallbacks.CompletionCallback tmp = djiError -> {
        if(djiError != null){
            CustomFun.showToast(djiError.getDescription(), getApplicationContext());
        }
    };
}
