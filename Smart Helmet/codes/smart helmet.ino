#include <DHT.h>
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1 //Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C //See datasheet for Address
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define led1 5
#define mq2pin 33                  //defining pins connected to sensors
#define piezo1 32                  //defining pins connected to sensors  
//#define piezo2 13                  //defining pins connected to sensors 
//#define piezo3 14                  //defining pins connected to sensors 
#define DHTPIN 18                  //defining pins connected to sensors 
// Define the RX and TX pins for the GPS module
#define GPS_RX 16   // Use the RX2 pin on ESP32
#define GPS_TX 17   // Use the TX2 pin on ESP32
const int buzzerPin = 19; // Define the pin connected to the buzzer
HardwareSerial neogps(1);


TinyGPSPlus gps;

#define DHTTYPE DHT11
DHT dht(DHTPIN,DHTTYPE);

#define WLAN_SSID       ""
#define WLAN_PASS       ""

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    ""
#define AIO_KEY         "" 
WiFiClient client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish Gas = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Gas");
Adafruit_MQTT_Publish Temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Temperature");
Adafruit_MQTT_Publish Hit = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Hit");
Adafruit_MQTT_Publish latitudeFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Latitude");
Adafruit_MQTT_Publish longitudeFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Longitude");
Adafruit_MQTT_Subscribe Light1 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME"/feeds/Light");

#define thresh 150                //defining threshold value if something hits

void setup() {
  Serial.begin(9600);


  pinMode(buzzerPin, OUTPUT); // Set the buzzer pin as an output
  pinMode(led1, OUTPUT);

  Serial.println("Heating up the MQ-2 Sensor");
  delay(5000);
  dht.begin();
  Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS Module Test");

  Serial.print(F("Connecting to "));
  Serial.println(WLAN_SSID);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());

   if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }



  display.clearDisplay();
  display.display();
  delay(10000);
  mqtt.subscribe(&Light1);
  // connect to adafruit io
 connect(); 

}


void loop() {

boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (neogps.available())
    {
      if (gps.encode(neogps.read()))
      {
        newData = true;
      }
    }
  }
  //If newData is true
  if(newData == true)
  {
    newData = false;
    // Serial.println(gps.satellites.value());
    print_speed();
  }
  else
  {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.setTextSize(3);
    display.print("No Data");
    display.display();
  }  
  

  if(! mqtt.ping(3)) {
    // reconnect to adafruit io
    if(! mqtt.connected())
      connect();
  }
  delay(5000);

    while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
  }

  // Check if new data is available
  if (gps.location.isUpdated()) {
    Serial.println("GPS Fix Acquired!");

    // Print latitude, longitude, and altitude to the serial monitor
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.println(" ");
    

    Serial.print("Number of satelites connected: ");
    Serial.println(gps.satellites.value());


    // Publish data to Adafruit IO
    publishToAdafruitIO(gps.location.lat(), gps.location.lng());

    Serial.println(); // Add a blank line for better readability
  } else {
    Serial.println("Waiting for GPS Fix...");
  }

  
  float mq2Value = analogRead(mq2pin);  //reading values from sensor pins
  float piezo1Value = analogRead(piezo1); //reading values from sensor pins  
  if (mq2Value>2000 || piezo1Value>=thresh){
      digitalWrite(buzzerPin, HIGH);
      delay(500);
  }
  else
  {
      digitalWrite(buzzerPin, LOW);

  }
  //piezo2Value = analogRead(piezo2); //reading values from sensor pins 
  //piezo3Value = analogRead(piezo3); //reading values from sensor pins 
  float temp = dht.readTemperature();     //reading values from sensor pins 

  Serial.println("New:____________________________________________________________________________________________________________________________");
  Serial.print("Gas: "); Serial.println(mq2Value);
  Serial.print("Temperature in Celcius: "); Serial.println(temp);            //dh11 temp value 
  Serial.print("Hit :");Serial.println(piezo1Value);
  delay(3000);
  
  if (! Temperature.publish(temp)) {                     //Publish to Adafruit
      Serial.println(F("Failed"));
    } 
  if (!Gas.publish(mq2Value)) {                     //Publish to Adafruit
      Serial.println(F("Failed"));
    }

  if (! Hit.publish(piezo1Value)) {                     //Publish to Adafruit
      Serial.println(F("Failed"));
    }
  else {
      Serial.println(F("Sent!"));
    }

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(20000))) {
    if (subscription == &Light1) {
      Serial.print(F("Got: "));
      Serial.println((char *)Light1.lastread);
      int Light1_State = atoi((char *)Light1.lastread);
      digitalWrite(led1, Light1_State);
      
    }
  
  }  

  delay(3000);

}


void connect(){
Serial.print(F("Connecting to Adafruit IO... "));
  int8_t ret;
  while ((ret = mqtt.connect()) != 0) {
    switch (ret) {
      case 1: Serial.println(F("Wrong protocol")); break;
      case 2: Serial.println(F("ID rejected")); break;
      case 3: Serial.println(F("Server unavail")); break;
      case 4: Serial.println(F("Bad user/pass")); break;
      case 5: Serial.println(F("Not authed")); break;
      case 6: Serial.println(F("Failed to subscribe")); break;
      default: Serial.println(F("Connection failed")); break;
    }

    if(ret >= 0)
      mqtt.disconnect();

    Serial.println(F("Retrying connection..."));
    delay(10000);
  }
  Serial.println(F("Adafruit IO Connected!"));
}


void publishToAdafruitIO(float latitude, float longitude) {
  // Publish latitude and longitude to Adafruit IO feeds
  if (latitudeFeed.publish(latitude) && longitudeFeed.publish(longitude)) {
    Serial.println("Data sent to Adafruit IO feeds");
  } else {
    Serial.println("Failed to send data to Adafruit IO feeds");
  }
}

void print_speed()
{
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
       
  if (gps.location.isValid() == 1)
  {
   //String gps_speed = String(gps.speed.kmph());
    display.setTextSize(1);
    
    display.setCursor(25, 5);
    display.print("Lat: ");
    display.setCursor(50, 5);
    display.print(gps.location.lat(),6);

    display.setCursor(25, 20);
    display.print("Lng: ");
    display.setCursor(50, 20);
    display.print(gps.location.lng(),6);

    display.setCursor(25, 35);
    display.print("Speed: ");
    display.setCursor(65, 35);
    display.print(gps.speed.kmph());
    
    display.setTextSize(1);
    display.setCursor(0, 50);
    display.print("SAT:");
    display.setCursor(25, 50);
    display.print(gps.satellites.value());

    display.setTextSize(1);
    display.setCursor(70, 50);
    display.print("ALT:");
    display.setCursor(95, 50);
    display.print(gps.altitude.meters(), 0);

    display.display();
    
  }
  else
  {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.setTextSize(3);
    display.print("No Data");
    display.display();
  }  

}