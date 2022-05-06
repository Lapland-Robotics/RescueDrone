package com.sarLuap.baseStation;

import android.app.Activity;
import android.content.Context;
import android.media.MediaPlayer;

import java.util.Timer;
import java.util.TimerTask;

public class sound_timer {

    private Timer tm;
    private MediaPlayer mp;
    private boolean playing;

    protected boolean isPlaying(){
        return playing;
    }

    protected sound_timer() {
        playing = false;
    }

    public void initSound(Activity ctx, int sound){
        mp = MediaPlayer.create(ctx, sound);
    }

    public void startTimer(long period){
        if(!playing) {
            tm = new Timer();
            tm.schedule(new TimerTask() {
                @Override
                public void run() {
                    mp.start();
                }
            }, 0, period);
            playing = true;
        }
    }

    public void stopTimer(){
        if(playing){
            tm.cancel();
            playing = false;
        }
    }
}
