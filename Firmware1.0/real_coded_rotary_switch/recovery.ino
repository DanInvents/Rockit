void recovery(){
  if (timer == true && t1 >= (1000*EEPROM.read(1)+908)){; //Here the 908 ms correspond to the time covered by the circular buffer
    servo1.write(EEPROM.read(3)); //Move servo1 to the final position EEPROM.read(3);
    if (t1 >= (1000*EEPROM.read(1) + 500*EEPROM.read(6) + 908 + 100)){ //The additional 100 ms is to prevent both servos from moving simultaneously.
      servo2.write(EEPROM.read(5));
      timer = false;
    }
  }
  
  else if (automatic == true){
    if ((filteredAltitudeDelta - altold) < -0.01){
      n++;      
      if (n == 4 && deploy == false){
        deploy = true;
        tconfig = t1;
      }
    }
      
    else if ((filteredAltitudeDelta - altold) >= 0 && deploy == false){
      n = 0;
    }
    
    if (deploy == true && (t1-tconfig) >= 500*EEPROM.read(0)){
      servo1.write(EEPROM.read(3));
    }
        
    if (deploy == true && (t1-tconfig) >= (500*(EEPROM.read(0)+EEPROM.read(6)))){
     servo2.write(EEPROM.read(5));
     deploy = false;
    }
    altold = filteredAltitudeDelta;
  }
}
