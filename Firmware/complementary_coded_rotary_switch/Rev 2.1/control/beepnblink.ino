void beepnblink(){
  if (initVar == true && overtime == false){

    if (p<30){
      digitalWrite(statusLED, HIGH);
      if (piezoEnable == true){
        analogWrite(piezo, 50); //Turn the piezo on for 300ms
      }
    }
  
    else if (p == 30){
      digitalWrite(statusLED, LOW);
      if (piezoEnable == true){
        analogWrite(piezo, 0); //Turn the piezo off
      }
    }

    else if (p == 400){
      p = 0;
    }
  p++;
  }

  else if (initVar == false && overtime == false){ //This is necesary because if p<30 after launch is detected the LED and the piezo will be stuck in ON mode.
    digitalWrite(statusLED, LOW);
    if (piezoEnable == false){
      analogWrite(piezo, 0);
    }
  }

  else if (initVar == false && overtime == true){ //After timeout blink and beep the maximum altitude in meters
    altToDigits(); //Convert the maximum altitude to digits
    while(1){
      for (int i=3; i>-1; i--){
        blinknbeep(altMaxDig[i]); //Beep and blink the altitude digits
        delay(700); //0.5s pause in between the beeps and blinks
        if (i == 0){
          sleep_ms(10000); //Sleep for 10 seconds to save battery
        }
      }
    }
  }
}

void altToDigits(){ //Convert altitude to digits
  for (int i=0; i<4; i++){
    rmnd = altMax % 10; //This is the remainder
    altMaxDig[i] = rmnd;
    altMax = altMax / 10;
  }
}

void blinkLED(int n){ //Blinks the blue LED every 200 ms
    for (int i=0; i<=n; i++){
      digitalWrite(statusLED, HIGH);
      delay(200);
      digitalWrite(statusLED, LOW);
      delay(200);
    }
}

void blinknbeep(int n){ //Blinks the blue LED every 300 ms and makes the buzzer beep
    for (int i=0; i<n; i++){
      digitalWrite(statusLED, HIGH);
      if (piezoEnable == true){
        analogWrite(piezo, 50);
      }
      delay(250);
      
      digitalWrite(statusLED, LOW);
      if (piezoEnable == true){
        analogWrite(piezo, 0);
      }
      delay(250);
    }
}
