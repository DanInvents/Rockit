void recovery(){

  if (timer == true && t1 >= (1000*EEPROM.read(1)+908)){ //Here the 908 ms correspond to the time covered by the circular buffer
    servo1.write(EEPROM.read(3)); //Move servo1 to the final position EEPROM.read(3);
    servo1.attach(servo1pin);
    if (timer == true && t1 >= (1000*EEPROM.read(1)+908+2000)){ //We disable the servo after 2 seconds to save power
      servo1.detach();
    }

    if (t1 >= (1000*EEPROM.read(1) + 500*EEPROM.read(6) + 908 + 100)){ //The additional 100 ms is to prevent both servos from moving simultaneously.
      servo2.write(EEPROM.read(5));
      servo2.attach(servo2pin);
      timer = false;
    }

    if (t1 >= (1000*EEPROM.read(1) + 500*EEPROM.read(6) + 908 + 100 + 2000)){ //We disable the servo after 2 seconds to save power
      servo2.detach();
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
      servo1.attach(servo1pin);
        if (deploy == true && (t1-tconfig) >= (500*EEPROM.read(0) + 2000)){ //We disable the servo after 2 seconds to save power
          servo1.detach();
        }
    }
        
    if (deploy == true && (t1-tconfig) >= (500*(EEPROM.read(0) + EEPROM.read(6)))){
     servo2.write(EEPROM.read(5));
     servo2.attach(servo2pin);
     deploy = false;
       if (deploy == true && (t1-tconfig) >= (500*(EEPROM.read(0) + EEPROM.read(6)) + 2000)){ //We disable the servo after 2 seconds to save power
         servo2.detach();
        }
    }
  }
}
