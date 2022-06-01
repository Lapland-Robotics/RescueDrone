package com.sarLuap.baseStation

import android.annotation.SuppressLint
import android.content.Context
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.drawable.BitmapDrawable
import android.graphics.drawable.Drawable
import android.widget.Button
import androidx.annotation.DrawableRes
import androidx.appcompat.content.res.AppCompatResources
import com.mapbox.android.gestures.MoveGestureDetector
import com.mapbox.geojson.MultiPoint
import com.mapbox.geojson.Point
import com.mapbox.geojson.Polygon
import com.mapbox.maps.CameraOptions
import com.mapbox.maps.MapView
import com.mapbox.maps.Style
import com.mapbox.maps.extension.style.expressions.dsl.generated.interpolate
import com.mapbox.maps.extension.style.image.image
import com.mapbox.maps.extension.style.layers.addLayer
import com.mapbox.maps.extension.style.layers.addLayerBelow
import com.mapbox.maps.extension.style.layers.generated.FillLayer
import com.mapbox.maps.extension.style.layers.generated.SymbolLayer
import com.mapbox.maps.extension.style.layers.generated.fillLayer
import com.mapbox.maps.extension.style.layers.generated.symbolLayer
import com.mapbox.maps.extension.style.layers.getLayerAs
import com.mapbox.maps.extension.style.layers.properties.generated.IconAnchor
import com.mapbox.maps.extension.style.layers.properties.generated.IconRotationAlignment
import com.mapbox.maps.extension.style.sources.addSource
import com.mapbox.maps.extension.style.sources.generated.GeoJsonSource
import com.mapbox.maps.extension.style.sources.generated.geoJsonSource
import com.mapbox.maps.extension.style.sources.getSourceAs
import com.mapbox.maps.extension.style.style
import com.mapbox.maps.plugin.LocationPuck2D
import com.mapbox.maps.plugin.compass.compass
import com.mapbox.maps.plugin.gestures.OnMapLongClickListener
import com.mapbox.maps.plugin.gestures.OnMoveListener
import com.mapbox.maps.plugin.gestures.gestures
import com.mapbox.maps.plugin.locationcomponent.OnIndicatorBearingChangedListener
import com.mapbox.maps.plugin.locationcomponent.OnIndicatorPositionChangedListener
import com.mapbox.maps.plugin.locationcomponent.location
import kotlin.math.abs


class LiveMapClass {
    private val onIndicatorBearingChangedListener = OnIndicatorBearingChangedListener {
        mapView.getMapboxMap().setCamera(CameraOptions.Builder().bearing(it).build())
    }

    private val onIndicatorPositionChangedListener = OnIndicatorPositionChangedListener {
        mapView.getMapboxMap().setCamera(CameraOptions.Builder().center(it).build())
        mapView.gestures.focalPoint = mapView.getMapboxMap().pixelForCoordinate(it)
    }

    private val onMapLongClickListener = OnMapLongClickListener { point ->
            area.add(point)
            updatePoints()
//            CustomFun.showToast("" + area.size, liveMapActivityIntern)
            false
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
    private var area: MutableList<Point> = ArrayList()

    private var lastRot: Double = 0.0
    private var lastLong: Double = 0.0
    private var lastLat: Double = 0.0
    private var posTrackingON: Boolean = false

    // getters
    fun getPosTrackingON(): Boolean {return posTrackingON}
    fun getArea(): MutableList<Point>{return area}

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

                +image(POINTS){
                    bitmapFromDrawableRes(liveMapActivityIntern, R.drawable.ic_map_corner_point)?.let {
                        bitmap(it)
                    }
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

//        area.clear()

        posTrackingON = false
        btnTrack.text = "" + liveMapActivityIntern.getText(R.string.button_position_tracking) + " OFF"
    }

    fun updateDrone(long: Double, lat: Double, rot: Double){

        val needRotUpdated = (abs(rot - lastRot) > 10)
        val needPointUpdated = (abs(long - lastLong) > 0.000015 || abs(lat - lastLat) > 0.000015)

        mapView.getMapboxMap().getStyle{ style ->
            val layer = style.getLayerAs<SymbolLayer>(LAYER_DRONE)
            val source = style.getSourceAs<GeoJsonSource>(SOURCE_DRONE)

            if(source == null){
                style.addSource(geoJsonSource(SOURCE_DRONE).geometry(Point.fromLngLat(long, lat)))
            }
            else if (needPointUpdated){
                source.geometry(Point.fromLngLat(long, lat))
                lastLong = long
                lastLat = lat
            }

            if(layer == null){
                style.addLayer(symbolLayer(LAYER_DRONE, SOURCE_DRONE){
                    iconImage(DRONE)
                    iconSize(0.05)
                    iconRotationAlignment(IconRotationAlignment.MAP)
                    iconRotate(rot)
                    iconAllowOverlap(true)
                })
            }
            else if(needRotUpdated){
                layer.iconRotate(rot)
                lastRot = rot
            }
        }
    }

    private fun updatePoints() {
        mapView.getMapboxMap().getStyle{ style ->
            val layer = style.getLayerAs<SymbolLayer>(LAYER_POINTS)
            val source = style.getSourceAs<GeoJsonSource>(SOURCE_POINTS)

            if(source == null){
                style.addSource(geoJsonSource(SOURCE_POINTS).geometry(MultiPoint.fromLngLats(area)))
            }
            else{
                source.geometry(MultiPoint.fromLngLats(area))
            }

            if(layer == null) {
                style.addLayer(symbolLayer(LAYER_POINTS, SOURCE_POINTS) {
                    iconImage(POINTS)
                    iconSize(0.05)
                    iconOpacity(1.0)
                    iconAllowOverlap(true)
                    iconAnchor(IconAnchor.BOTTOM)
                })
            }
        }

        if(area.size > 3){
            updateArea()
        }
        else if(area.size == 3){
            area.add(area.first())
            removeArea()
            updateArea()
            area.removeLast()
        }
        else{
            removeArea()
        }
    }

    private fun updateArea(){

        mapView.getMapboxMap().getStyle{ style ->
            val source = style.getSourceAs<GeoJsonSource>(SOURCE_AREA)
            val layer = style.getLayerAs<FillLayer>(LAYER_AREA)

            val areaTmp: MutableList<MutableList<Point>> = ArrayList()
            areaTmp.add(area)

            if(source == null){
                style.addSource(geoJsonSource(SOURCE_AREA).geometry(Polygon.fromLngLats(areaTmp)))
            }
            else{
                source.geometry(Polygon.fromLngLats(areaTmp))
            }

            if(layer == null){
                style.addLayerBelow(fillLayer(LAYER_AREA, SOURCE_AREA){
                    fillColor(liveMapActivityIntern.getColor(R.color.colorPrimary))
                    fillOpacity(0.5)
                }, LAYER_POINTS)
            }
        }
    }

    fun removeDrone(){
        mapView.getMapboxMap().getStyle { style ->
            style.removeStyleLayer(LAYER_DRONE)
            style.removeStyleSource(SOURCE_DRONE)
        }
    }

    fun removeLastPoint(){
        if(area.size == 0){
            return
        }
        area.removeLast()
        updatePoints()
    }

    fun removePoints(){
        mapView.getMapboxMap().getStyle{ style ->
            style.removeStyleLayer(LAYER_POINTS)
            style.removeStyleSource(SOURCE_POINTS)
        }
        removeArea()
        area.clear()
    }

    private fun removeArea(){
        mapView.getMapboxMap().getStyle { style ->
            style.removeStyleLayer(LAYER_AREA)
            style.removeStyleSource(SOURCE_AREA)
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
    fun activateCameraTrackingActivate(){
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
        mapView.gestures.removeOnMapLongClickListener(onMapLongClickListener)
    }

    //private fun
    private fun setupGesturesListener() {
        mapView.gestures.addOnMoveListener(onMoveListener)
        mapView.gestures.addOnMapLongClickListener(onMapLongClickListener)
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
        }
        else {
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

    companion object {
        private const val POINTS = "points"
        private const val SOURCE_POINTS = "source_points"
        private const val LAYER_POINTS = "layer_points"

        private const val SOURCE_AREA = "source_area"
        private const val LAYER_AREA = "layer_area"

        private const val DRONE = "drone"
        private const val SOURCE_DRONE = "source_drone"
        private const val LAYER_DRONE = "layer_drone"
    }
}
