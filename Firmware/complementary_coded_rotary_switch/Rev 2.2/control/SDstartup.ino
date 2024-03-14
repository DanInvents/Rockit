void SDstartup(){ 
  // This program checks if the card is present and can be initialized:
  if (!SD.begin(17)) {
      digitalWrite(statusLED, HIGH); //The blue LED turns on if the card cannot be initialized
    while(1);
  }

  char filename[] = "00.CSV"; //File name
  for (uint8_t i = 0; i < 100; i++) { //The SD card can store up to 100 files
    filename[0] = i/10 + '0';
    filename[1] = i%10 + '0';
    if (! SD.exists(filename)) {
      dataFile = SD.open(filename, O_CREAT | O_WRITE); //Only open a new file if it doesn't exist
      break;
    }
    else if (SD.exists(F("99.CSV"))){
      while(1){
        digitalWrite(statusLED, HIGH); //If there are 100 files, the blue LED turns on
      }
    }
  }
  dataFile.println(F("Time (ms), Altitude (m), Filtered altitude (m), Acceleration (g), Perpendicular acceleration (g), Temperature (C)")); //File header
  dataFile.flush(); //Writes data to the SD card
}
