void preLaunch(){ //This code works great
  t4 = millis();
    
  for (int i = 0; i<=99; i++){
    currentPressure = barometricSensor.getPressure();
    sensors_event_t event;
    accel.getEvent(&event);
    rocketAccel = ((event.acceleration.y/9.81+0.01)-(event.acceleration.x/9.81-0.04))/sqrt(2);
    accelerations.push(rocketAccel);
    altitudeDelta = barometricSensor.altitudeChange(currentPressure, startingPressure)+0.6;
    altitudes.push(altitudeDelta);
    FilteredAltitudes.push(kalmanFilter(altitudeDelta));
    times.push(millis()-t4);
  }  
}
