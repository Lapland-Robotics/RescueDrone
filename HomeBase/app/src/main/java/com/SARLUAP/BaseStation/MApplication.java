package com.sarLuap.baseStation;

import android.app.Application;
import android.content.Context;

import com.secneo.sdk.Helper;

public class MApplication extends Application {
    private DroneConnection droneConnection;

    @Override
    protected void attachBaseContext(Context paramContext) {
        super.attachBaseContext(paramContext);
        Helper.install(MApplication.this);
        if (droneConnection == null){
            droneConnection = new DroneConnection();
            droneConnection.setContext(this);
        }
    }

    @Override
    public void onCreate() {
        super.onCreate();
        droneConnection.onCreate();
    }
}
