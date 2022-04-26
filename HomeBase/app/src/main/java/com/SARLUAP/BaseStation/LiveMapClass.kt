package com.sarLuap.baseStation

import android.annotation.SuppressLint
import android.content.Context
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.drawable.BitmapDrawable
import android.graphics.drawable.Drawable
import android.os.Handler
import android.os.Looper
import android.widget.Button
import android.widget.Toast
import androidx.annotation.DrawableRes
import androidx.appcompat.content.res.AppCompatResources
import com.mapbox.android.gestures.MoveGestureDetector
import com.mapbox.geojson.Point
import com.mapbox.maps.CameraOptions
import com.mapbox.maps.MapView
import com.mapbox.maps.Style
import com.mapbox.maps.extension.style.expressions.dsl.generated.interpolate
import com.mapbox.maps.extension.style.image.image
import com.mapbox.maps.extension.style.layers.generated.SymbolLayer
import com.mapbox.maps.extension.style.layers.generated.symbolLayer
import com.mapbox.maps.extension.style.layers.getLayerAs
import com.mapbox.maps.extension.style.layers.properties.generated.IconRotationAlignment
import com.mapbox.maps.extension.style.sources.generated.GeoJsonSource
import com.mapbox.maps.extension.style.sources.generated.geoJsonSource
import com.mapbox.maps.extension.style.sources.getSourceAs
import com.mapbox.maps.extension.style.style
import com.mapbox.maps.plugin.LocationPuck2D
import com.mapbox.maps.plugin.compass.CompassViewImpl
import com.mapbox.maps.plugin.compass.compass
import com.mapbox.maps.plugin.gestures.OnMoveListener
import com.mapbox.maps.plugin.gestures.gestures
import com.mapbox.maps.plugin.locationcomponent.OnIndicatorBearingChangedListener
import com.mapbox.maps.plugin.locationcomponent.OnIndicatorPositionChangedListener
import com.mapbox.maps.plugin.locationcomponent.location

class LiveMapClass {
    private val onIndicatorBearingChangedListener = OnIndicatorBearingChangedListener {
        mapView.getMapboxMap().setCamera(CameraOptions.Builder().bearing(it).build())
    }

    private val onIndicatorPositionChangedListener = OnIndicatorPositionChangedListener {
        mapView.getMapboxMap().setCamera(CameraOptions.Builder().center(it).build())
        mapView.gestures.focalPoint = mapView.getMapboxMap().pixelForCoordinate(it)
    }

    private val onMoveListener = object : OnMoveListener {
        override fun onMoveBegin(detector: MoveGestureDetector) {
            dismissedCameraTracking()
        }

        override fun onMove(detector: MoveGestureDetector): Boolean {
            return false
        }

        override fun onMoveEnd(detector: MoveGestureDetector) {}
    }

    private lateinit var liveMapActivityIntern: LiveMapActivity
    private lateinit var mapView: MapView
    private lateinit var btnTrack: Button

    private var posTrackingON: Boolean = false

    // getters
    fun getPosTrackingON(): Boolean {return posTrackingON}


    //public fun
    @SuppressLint("SetTextI18n")
    fun mapInit(liveMapActivityTmp: LiveMapActivity, mapViewTmp: MapView, trackingButton: Button ) {
        liveMapActivityIntern = liveMapActivityTmp
        mapView = mapViewTmp
        btnTrack = trackingButton

        mapView.getMapboxMap().setCamera(
            CameraOptions.Builder()
                .zoom(14.0)
                .build()
        )
        mapView.getMapboxMap().loadStyle(
            styleExtension = style(Style.OUTDOORS){
                +image(DRONE) {
                    bitmapFromDrawableRes(liveMapActivityIntern, R.drawable.ic_drone_icon2)?.let {
                        bitmap(it)
                    }
                }

                +geoJsonSource(SOURCE_DRONE){
                    geometry(Point.fromLngLat(LONGITUDE, LATITUDE))
                }

                +symbolLayer(LAYER_DRONE, SOURCE_DRONE){
                    iconImage(DRONE)
                    iconSize(0.05)
                    iconRotationAlignment(IconRotationAlignment.MAP)
                    iconOpacity(0.0)
                }

            }
        ) {
            initLocationComponent()
            setupGesturesListener()
            mapView.location.updateSettings {
                enabled = true
                pulsingEnabled = true
            }
        }

        Style.OnStyleLoaded { mapView.compass.updateSettings { enabled = true; fadeWhenFacingNorth = false; clickable = true } }

        posTrackingON = false
        btnTrack.text = "" + liveMapActivityIntern.getText(R.string.button_position_tracking) + " OFF"
    }

    fun updateDrone(long: Double, lat: Double, rot: Double, show: Boolean){

        mapView.getMapboxMap().getStyle{ style ->
            val layer = style.getLayerAs<SymbolLayer>(LAYER_DRONE)
            val source = style.getSourceAs<GeoJsonSource>(SOURCE_DRONE)

            source?.geometry(Point.fromLngLat(if(show) long else LONGITUDE, if(show) lat else LATITUDE))
            layer?.iconRotate(rot)
            layer?.iconOpacity(if(show) 1.0 else 0.0)
        }
    }

    @SuppressLint("SetTextI18n")
    fun dismissedCameraTracking() {
        mapView.location.removeOnIndicatorPositionChangedListener(onIndicatorPositionChangedListener)
        mapView.location.removeOnIndicatorBearingChangedListener(onIndicatorBearingChangedListener)
        mapView.gestures.removeOnMoveListener(onMoveListener)

        posTrackingON = false
        btnTrack.text = "" + liveMapActivityIntern.getText(R.string.button_position_tracking) + " OFF"
    }

    @SuppressLint("SetTextI18n")
    fun activateCameraTrakingActivate(){
        mapView.location.addOnIndicatorPositionChangedListener(onIndicatorPositionChangedListener)
        mapView.location.addOnIndicatorBearingChangedListener(onIndicatorBearingChangedListener)
        mapView.gestures.addOnMoveListener(onMoveListener)

        posTrackingON = true
        btnTrack.text = "" + liveMapActivityIntern.getText(R.string.button_position_tracking) + " ON"
    }

    fun onDestroy() {
        mapView.location.removeOnIndicatorBearingChangedListener(onIndicatorBearingChangedListener)
        mapView.location.removeOnIndicatorPositionChangedListener(onIndicatorPositionChangedListener)
        mapView.gestures.removeOnMoveListener(onMoveListener)
    }

    //private fun
    private fun setupGesturesListener() {
        mapView.gestures.addOnMoveListener(onMoveListener)
    }

    private fun initLocationComponent() {
        val locationComponentPlugin = mapView.location
        locationComponentPlugin.updateSettings {
            this.enabled = true
            this.locationPuck = LocationPuck2D(
                bearingImage = AppCompatResources.getDrawable(
                    liveMapActivityIntern,
                    //R.drawable.ic_drone_icon2,
                    R.drawable.mapbox_user_puck_icon
                ),
                shadowImage = AppCompatResources.getDrawable(
                    liveMapActivityIntern,
                    R.drawable.mapbox_user_icon_shadow,
                ),
                scaleExpression = interpolate {
                    linear()
                    zoom()
                    stop {
                        literal(0.0)
                        //literal(0.03)
                        literal(0.6)
                    }
                    stop {
                        literal(20.0)
                        //literal(0.05)
                        literal(1.0)
                    }
                }.toJson()
            )
        }
        locationComponentPlugin.addOnIndicatorPositionChangedListener(onIndicatorPositionChangedListener)
        //locationComponentPlugin.addOnIndicatorBearingChangedListener(onIndicatorBearingChangedListener)

    }
    private fun bitmapFromDrawableRes(context: Context, @DrawableRes resourceId: Int) =
        convertDrawableToBitmap(AppCompatResources.getDrawable(context, resourceId))

    private fun convertDrawableToBitmap(sourceDrawable: Drawable?): Bitmap? {
        if (sourceDrawable == null) {
            return null
        }
        return if (sourceDrawable is BitmapDrawable) {
            sourceDrawable.bitmap
        } else {
            // copying drawable object to not manipulate on the same reference
            val constantState = sourceDrawable.constantState ?: return null
            val drawable = constantState.newDrawable().mutate()
            val bitmap: Bitmap = Bitmap.createBitmap(
                drawable.intrinsicWidth, drawable.intrinsicHeight,
                Bitmap.Config.ARGB_8888
            )
            val canvas = Canvas(bitmap)
            drawable.setBounds(0, 0, canvas.width, canvas.height)
            drawable.draw(canvas)
            bitmap
        }
    }

    private fun showToast(toastMsg: String) {
        val handler = Handler(Looper.getMainLooper())
        handler.post {
            Toast.makeText(liveMapActivityIntern, toastMsg, Toast.LENGTH_SHORT).show()
        }
    }

    companion object {
        private const val DRONE = "drone"
        private const val SOURCE_DRONE = "source_drone"
        private const val LAYER_DRONE = "layer_drone"
        private const val LATITUDE = 0.0
        private const val LONGITUDE = 0.0
    }
}
