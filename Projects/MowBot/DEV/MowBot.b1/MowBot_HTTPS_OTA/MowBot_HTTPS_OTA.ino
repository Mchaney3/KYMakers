/**
   esp32 firmware OTA
   
   Purpose: Perform an OTA update from a bin located on a webserver (HTTPS) without having a root cert

   Setup:
   Step 1 : Set your WiFi (ssid & password)
   Step 2 : set esp32fota()
   
   Upload:
   Step 1 : Menu > Sketch > Export Compiled Library. The bin file will be saved in the sketch folder (Menu > Sketch > Show Sketch folder)
   Step 2 : Upload it to your webserver
   Step 3 : Update your firmware JSON file ( see firwmareupdate )

   THIS IS WORKING AS OF 4/2/22. DON'T TOUCH IT!

*/

#include <Arduino.h>

#include <WiFi.h>

#include <FS.h>
#include <SPIFFS.h>
#include <esp32fota.h>


// Change to your WiFi credentials
const char *ssid = "chlabs_bot";
const char *password = "chlabsrobotseverywhere";

// esp32fota esp32fota("<Type of Firmware for this device>", <this version>, <validate signature>, <allow insecure https>);
esp32FOTA esp32FOTA("s3-arnold", 4, false, true);

void setup_wifi()
{
  delay(10);
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println(WiFi.localIP());
}

void setup()
{
  
  esp32FOTA.checkURL = "http://mowbotfirmware.axstechnology.com/fota.json";
  Serial.begin(115200);
  setup_wifi();
}

void loop()
{

  bool updatedNeeded = esp32FOTA.execHTTPcheck();
  if (updatedNeeded)
  {
    Serial.println("Update Available");
    esp32FOTA.execOTA();
  }
  else {
    Serial.println("No Update Necessary");
  }
  delay(2000);
}
