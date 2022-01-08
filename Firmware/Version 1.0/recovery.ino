void recovery(){
  if (timer == true && t1 >= (1000*EEPROM.read(1)+908)){ //EEPROM.read(1);
    digitalWrite(statusLED, HIGH); 
    servo1.write(EEPROM.read(3)); //Move servo1 to the final position EEPROM.read(3);
    if (t1 >= (1000*EEPROM.read(1) + 500*EEPROM.read(6) + 908 + 100)){
      servo2.write(EEPROM.read(5));
      timer = false;
    }
  }
  
  else if (automatic == true){
    if ((filteredAltitudeDelta - altold) < -0.01){
      n++;
      if (n > 4 & deploy == false){
        deploy = true;
        tconfig = t1;
      }
      if (deploy == true & t1-tconfig >= 500*EEPROM.read(0)){
        digitalWrite(statusLED, HIGH);
        servo1.write(EEPROM.read(3));
         if (t1 >= (1000*EEPROM.read(1) + 500*EEPROM.read(6) + 908 + 100)){
          servo2.write(EEPROM.read(5));
         }
      }
    }
    
    else if ((filteredAltitudeDelta - altold) >= 0){
      n = 0;
    }
   altold = filteredAltitudeDelta;
 }
}
