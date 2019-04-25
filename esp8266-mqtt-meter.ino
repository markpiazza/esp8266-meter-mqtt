/*****************************************************************************************
* Simple pulse counter which sends data over MQTT.
* 
* It sends out data every 60 seconds by default.
* It keeps adding pulses until succesful connection.
* 
* EPS8266MOD       - https://github.com/nodemcu/nodemcu-devkit-v1.0
* PubSubClient     - https://github.com/knolleary/pubsubclient
*****************************************************************************************/
#include <Arduino.h>
#include <ESP8266WiFi.h>  // ESP8266 Wifi and http server 
#include <PubSubClient.h> // mqtt service
#include <WiFiUdp.h>    // for ntp

/**
 * D0 - GPIO16 - MCU pin 16 - red led
 * D1 - GPIO05 - MCU pin 5 
 * D2 - GPIO04 - MCU pin 4 - boot fail if pulled low
 * D3 - GPIO00 - MCU pin 0 - boot fail if pulled low
 * D4 - GPIO02 - MCU pin 2 - blue led
 * D5 - GPIO14 - MCU pin 14
 * D6 - GPIO12 - MCU pin 12
 * D7 - GPIO13 - MCU pin 13
 * D8 - GPIO15 - MCU pin 15
 * RX - GPIO03 - MCU pin 3
 * TX - GPIO01 - MCU pin 1
 */

#define PULSE_PIN                     0           // D3 - GPIO0
#define WIFI_LED_PIN                  2           // D4 - GPIO2 - esp8266 blue led
#define RED_LED_PIN                   16          // D0 - GPIO16 - dev board red led
#define SAMPLE_MINUTES                1           // time between updates
#define DEBOUNCE_MS                   500         // debouncing for interrupts
#define SENSOR_FAMILY                 "ESP8266a" 
#define SENSOR_NAME                   "energy"    
#define SENSOR_ID                     "1"
#define TOPIC_PREFIX                  "meter"
#define SERIAL_SPEED                  115200      // serial port speed in kbps
#define KH                            3.6         // kh constant 
#define KS                            KH/3600     // ks constant

const char *compileDate               = __DATE__ " " __TIME__;
const char *sensorString              = SENSOR_FAMILY "-" SENSOR_NAME "-" SENSOR_ID;
const char *ntpServerName             = "time.nist.gov";
const char *ssid                      = "Colt45";          
const char *pass                      = "zoeythecat2012";
const char *mqttServer                = "192.168.1.177";
const char *mqttTopic                 = TOPIC_PREFIX "/" SENSOR_NAME;

const unsigned int mqttPort           = 1883;     // MQTT port number. default is 1883
const unsigned int httpPort           = 80;       // HTTP port number. 
unsigned int ntpPort                  = 2390;     // NTP port number

const int NTP_PACKET_SIZE             = 48;       // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[NTP_PACKET_SIZE];               // Buffer to hold incoming and outgoing packets

unsigned int mqttFailures             = 0;

volatile unsigned long lastMicros;

unsigned long sysTime;

volatile unsigned int pulses          = 0;
volatile unsigned int pulsesLast      = 0;
volatile unsigned int pulsesKept      = 0;
volatile unsigned int pulsesPeriods   = 0;

WiFiUDP udp;
WiFiServer server(httpPort);
WiFiClient wifiClient;
os_timer_t myTimer;

String lastPayload;
bool tickOccured;


/**
 * mqtt callback
 */
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.println("mqtt callback invoked. not implemented.");
}

PubSubClient mqttClient(mqttServer, mqttPort, mqttCallback, wifiClient);


/**
 * timer callback
 */
void timerCallback(void *pArg) {
  pulsesLast = pulses;
  pulsesKept += pulses;
  pulses = 0;
  pulsesPeriods++;
  tickOccured = true;
}


/**
 * timer init
 */
void timerInit(void) {
  os_timer_setfn(&myTimer, timerCallback, NULL);
  os_timer_arm(&myTimer, 1000 * 60 * SAMPLE_MINUTES, true);
}


/**
 * pulse handler
 */
void pulseHandler() {
  if(digitalRead(PULSE_PIN)==HIGH) 
    return;
    
  if((long)(micros() - lastMicros) >= DEBOUNCE_MS * 1000) {
    pulses = pulses + 1;
    lastMicros = micros();
    Serial.println ("pulse: " + String(pulses));
  }
}


/**
 * get time
 */
unsigned long inline ntpUnixTime (UDP &udp) {
  static int udpInited = udp.begin(123); // open socket on arbitrary port
  const long ntpFirstFourBytes = 0xEC0600E3; // NTP request header
  
  if (!udpInited)
    return 0;
  
  udp.flush();
  
  if (!(udp.beginPacket(ntpServerName, 123) // 123 is the NTP port
   && udp.write((byte *)&ntpFirstFourBytes, 48) == 48
   && udp.endPacket()))
    return 0;       // sending request failed

  const int pollIntv = 150;   // poll every this many ms
  const byte maxPoll = 15;    // poll up to this many times
  int pktLen;       // received packet length  
  for (byte i=0; i<maxPoll; i++) {
    if ((pktLen = udp.parsePacket()) == 48)
      break;
    delay(pollIntv);
  }
  
  if (pktLen != 48)
    return 0;       // no correct packet received
  
  const byte useless = 40;
  for (byte i = 0; i < useless; ++i)
    udp.read();
  
  unsigned long time = udp.read();  // NTP time
  for (byte i = 1; i < 4; i++)
    time = time << 8 | udp.read();
  
  time += (udp.read() > 115 - pollIntv/8);

  // Discard the rest of the packet
  udp.flush();
  
  // convert NTP time to Unix time
  return time - 2208988800ul;   
}


/**
 * connect wifi
 */
void wifiConnect() {
  Serial.println("Connecting to: " + String(ssid));
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED ) {
    blinkLed();
  }
  Serial.println("WiFi connected");  
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}


/**
 * mqttConnect
 */
void mqttConnect(){
  if (WiFi.status() == WL_CONNECTED) {
    if (!mqttClient.connected()) {
      yield();
      if (mqttClient.connect(sensorString)) {
        yield();
        mqttClient.publish("welcome", sensorString);
        Serial.println("mqtt connected");
      }
    }

    if (mqttClient.connected())
      mqttClient.loop();
  }
}


/**
 * http serve
 */
void httpServe(){
  WiFiClient client = server.available();
  if ( client ) {
    String req = client.readStringUntil( '\r' );
    client.flush();
    if ( req.indexOf( "GET" ) != -1 ) {
      client.print( lastPayload  ); 
    }
    client.flush();
  }
}


/**
 * processTick
 */
void processTick(){
  if (tickOccured == true) {
    Serial.print("Tick occured. Pulses kept so far: ");
    String payload = getPayload();
    lastPayload = payload;
    Serial.println(payload);

    if (mqttClient.publish(mqttTopic, (char*) payload.c_str())) {
      Serial.println("mqtt publish ok");
      blinkLed();  
      lastPayload = payload;
      pulsesPeriods = 0;
      pulsesKept = 0;
    } else {
      Serial.println("mqtt publish fail");
      if (mqttClient.connected()) {
        Serial.println("mqtt connection still up");
      } else {
        Serial.println("mqtt connection down");
        if (mqttClient.connect(sensorString)) {
          yield();
        }
      }
    }
    tickOccured = false;
  }
}


/**
 * blinkLed 
 */
void blinkLed(){
  digitalWrite( WIFI_LED_PIN, LOW ); // on
  delay(100);
  digitalWrite( WIFI_LED_PIN, HIGH ); // off
  delay( 100 );
}


/**
 * printStartup
 */
void printStartup() {
  Serial.println();
  Serial.print("Compile Date: ");
  Serial.println(compileDate);
  Serial.print("Broker: ");
  Serial.print(mqttServer);
  Serial.print(" Topic: "); 
  Serial.println(mqttTopic);
}


/**
 * getPayload
 */
String getPayload(){
  int seconds = pulsesPeriods * 60 * SAMPLE_MINUTES;
  int wattHours = pulsesKept * KH;
  double watts = 3600 * wattHours / seconds;
  unsigned long upTime = (unsigned long) millis()/1000;
  String payload = "{";
  payload += "\"pulses\":"; 
  payload += pulsesKept;
  payload += ", \"seconds\":";
  payload += seconds;
  payload += ", \"pulsePeriods\":"; 
  payload += pulsesPeriods;
  payload += ", \"watts\":";
  payload += watts;
  payload += ", \"wattHours\":";
  payload += wattHours;
  payload += ", \"upTime\":";
  payload +=  upTime;
  payload += ", \"sysTime\":";
  payload += (sysTime+upTime); 
  payload += "}";
  return payload;
}


/**
 * setup
 */
void setup() {
  Serial.begin(SERIAL_SPEED);
  delay(10);
  printStartup();

  pinMode(WIFI_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(PULSE_PIN, INPUT);

  digitalWrite(WIFI_LED_PIN, HIGH); // turn off 
  digitalWrite(RED_LED_PIN, HIGH); // turn off
      
  attachInterrupt(digitalPinToInterrupt(PULSE_PIN), pulseHandler, CHANGE);
  lastPayload = "Waiting for first publish";
  tickOccured = false;

  timerInit();
  wifiConnect();
  server.begin();
  udp.begin(ntpPort);

  sysTime = ntpUnixTime(udp);
  Serial.print("NTP Update: ");
  Serial.println(sysTime);

  Serial.println("READY!");
  Serial.println();
}


/**
 * loop
 */
void loop() {
  if (WiFi.status() != WL_CONNECTED) {  
    wifiConnect(); 
  }
  httpServe(); // process http connections
  processTick(); // process tick
  mqttConnect(); // reconnect to mqtt if needed
  
  yield();
}
