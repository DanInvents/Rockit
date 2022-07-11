void batteryStatus(){
  if ((2*analogRead(29)*3.3/(pow(2,12)-1)) < 3.8){
    digitalWrite(batLED, HIGH);
  }
  else if ((2*analogRead(29)*3.3/(pow(2,12)-1)) > 3.8){
    digitalWrite(batLED, LOW);
  }
}
