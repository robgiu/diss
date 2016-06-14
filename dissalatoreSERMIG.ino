/*
  remote control for "Dissalatore"
 (C) Roberto Giungato
16/04/2016
 */

#include <SPI.h>
#include <Ethernet.h>
#include <stdlib.h>
#include <PString.h>
#include <Time.h>

#include <PubSubClient.h>     // MQTT

#define OUTPUTLEVEL 0    // 0 = invia solo i delta letture, 1 = invia tutto

#define INPUT0 7    // led su digital input 7
#define INPUT1 8    // led su digital input 8
#define INPUT2 9    // led su digital input 9
#define INPUT3 9    // led su digital input 9
#define OUTPUT0 2    // pulsante su digital 2
#define OUTPUT1 3    // pulsante su digital 3
#define OUTPUT2 5    // pulsante su digital 5
#define OUTPUT3 6    // pulsante su digital 6

int reading0 = 0;           // the current reading from the input pin
int reading1 = 0; 
int reading2 = 0; 
int reading3 = 0; 
int previous0 = 2;    // the previous reading from the input pin
int previous1 = 2; 
int previous2 = 2; 
int previous3 = 2; 
long time = 0;         // the last time the output pin was toggled
long debounce = 200;   // the debounce time, increase if the output flickers
static char stringToBeSent[15];

//Creao un array di byte per specificare il mac address
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
//creo un array di byte per specificare l'indirizzo ip
// rete Arsenale: 192.168.0.36, 192.168.0.254, 192.168.0.254, 255.255.255.0 in sequenza
byte ip[] = {
  192, 168, 3, 33}; // modificate questo valore in base alla vostra rete - valido per FONERA
byte nameserver[] = {
  8,8,8,8};
byte gateway[] = {
  192,168,3,1 }; // indirizzo FONERA, porta COMPUTER, modificate questo valore in base alla vostra rete
byte subnet[] = {
  255,255,255,0 };

IPAddress timeServer(132, 163, 4, 101); // time-a.timefreq.bldrdoc.gov NTP server
const int NTP_PACKET_SIZE= 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
const int timeZone = 1;     // Central European Time

// A UDP instance to let us send and receive packets over UDP
// EthernetUDP Udp;
// unsigned int localPort = 8888;      // local port to listen for UDP packets

// Callback function header
void callback(char* topic, byte* payload, unsigned int length);
// time_t getNtpTime();

EthernetClient ethClient;
String mqttServer = "xxx.yyy.com";    
PubSubClient mqttClient("xxx.yyy.com", 1883, callback, ethClient); // oggetto per MQTT

unsigned long previousTime = 0;          // last time you connected to the server, in milliseconds
const unsigned long sendingInterval = 1500;  // 
unsigned long previousTimeL = 0;          // last time you connected to the server, in milliseconds
const unsigned long loopingInterval = 1500;

int input0 = 0;
int input1 = 0;
int input2 = 0;
int input3 = 0;

char buffer[30];
PString datetime(buffer, sizeof(buffer));

//creao un oggetto server che rimane in ascolto sulla porta specificata
// EthernetServer WebServer(80);
EthernetClient pc_client;

void setup()
{
  // analogReference(EXTERNAL);
  Serial.begin(9600);
  delay(8000);

  //inizializza lo shield con il mac e l'ip
  Ethernet.begin(mac, ip, nameserver,gateway,subnet);
  // Ethernet.begin(mac);  // per utilizzare DHCP

//  Udp.begin(localPort);
//  Serial.println("waiting for sync");
//  setSyncProvider(getNtpTime);
//  setSyncInterval(3600);

  pinMode(INPUT0, OUTPUT);
  pinMode(INPUT1, OUTPUT);
  pinMode(INPUT2, OUTPUT);
  pinMode(INPUT3, OUTPUT);
  pinMode(OUTPUT0, INPUT);
  pinMode(OUTPUT1, INPUT);
  pinMode(OUTPUT2, INPUT);
  pinMode(OUTPUT3, INPUT);    
  digitalWrite(INPUT0, LOW);
  digitalWrite(INPUT1, LOW);
  digitalWrite(INPUT2, LOW);
  digitalWrite(INPUT3, LOW);  

    if (mqttClient.connect("clientDISS", "username", "password")) {
      Serial.println("si è connesso");
      mqttClient.publish("diss/dissalatore","Connected");
      mqttClient.subscribe("diss/input0");
      mqttClient.subscribe("diss/input1");
      mqttClient.subscribe("diss/input2");
      mqttClient.subscribe("diss/input3");
    } 
  
  Serial.print("setup terminato");
}

void loop()
{
  unsigned long currentTime = millis();

  // intervallo breve per leggere i valori sottoscritti
  if (currentTime - previousTimeL > loopingInterval) {
    mqttClient.loop();   // loop MQTT
    previousTimeL = currentTime;
  }

  // intervallo lungo per pubblicare
  if (currentTime - previousTime > sendingInterval) {
    previousTime = currentTime;

    reading0 = digitalRead(OUTPUT0);
    reading1 = digitalRead(OUTPUT1);
    reading2 = digitalRead(OUTPUT2);        
    reading3 = digitalRead(OUTPUT3);

    dtostrf(input0, 1,0, stringToBeSent);
    mqttClient.publish("diss/i0status",stringToBeSent);
    dtostrf(input1, 1,0, stringToBeSent);
    mqttClient.publish("diss/i1status",stringToBeSent);
    dtostrf(input2, 1,0, stringToBeSent);
    mqttClient.publish("diss/i2status",stringToBeSent);
    dtostrf(input3, 1,0, stringToBeSent);
    mqttClient.publish("diss/i3status",stringToBeSent);        
    
    if (OUTPUTLEVEL || previous0 != reading0) {
      dtostrf(reading0, 1,0, stringToBeSent);
      mqttClient.publish("diss/output0",stringToBeSent);
      delay(50);
      previous0 = reading0;
      Serial.println("INVIO DATI A MQTT");
      do_send_mqtt(); // invio a MQTT
    }  
    if (OUTPUTLEVEL || previous1 != reading1) {
      dtostrf(reading1, 1,0, stringToBeSent);
      mqttClient.publish("diss/output1",stringToBeSent);
      delay(50);
      previous1 = reading1;
      Serial.println("INVIO DATI A MQTT");
      do_send_mqtt(); // invio a MQTT
    }  
  if (OUTPUTLEVEL || previous2 != reading2) {
      dtostrf(reading2, 1,0, stringToBeSent);
      mqttClient.publish("diss/output2",stringToBeSent);
      delay(50);
      previous2 = reading2;
      Serial.println("INVIO DATI A MQTT");
      do_send_mqtt(); // invio a MQTT
    }  
  if (OUTPUTLEVEL || previous3 != reading3) {
      dtostrf(reading3, 1,0, stringToBeSent);
      mqttClient.publish("diss/output3",stringToBeSent);
      delay(50);
      previous3 = reading3;
      Serial.println("INVIO DATI A MQTT");
      do_send_mqtt(); // invio a MQTT
    }  

    mqttClient.loop();  // loop MQTT
    // delay(10000);
  }
}


void do_send_mqtt() {
  // if you're not connected
  //  then connect again and send data:
  static char stringToBeSent[15];
  
  Serial.println("pronto per inviare");

  if (mqttClient.connected()) {
    Serial.println("risulta connesso");
    
    // dtostrf(temperatureD, 8,2, stringToBeSent);
    // mqttClient.publish("/ics/temp",stringToBeSent);
    // delay(100);

//    static char datetimeA[18];
//    strcpy(datetimeA, datetime);
//    mqttClient.publish("datetime",datetimeA);
//    delay(50);
//    Serial.println("appena eseguito publish");
  } 
  else {
    Serial.println("risulta NON connesso");
    if (mqttClient.connect("clientDISS", "username", "password")) {
      Serial.println("si è connesso");
      mqttClient.publish("diss/dissalatore","REconnected");
      mqttClient.subscribe("diss/input0");
      mqttClient.subscribe("diss/input1");
      mqttClient.subscribe("diss/input2");
      mqttClient.subscribe("diss/input3");
    } 
    else {
      Serial.println("Connessione fallita NUOVAMENTE");
    }
  }
}


void callback(char* topic, byte* payload, unsigned int length) {
  // accendo o spengo led
  if (strcmp(topic,"diss/input0")==0) { 
        Serial.println("RICEVUTO INPUT0");
    if (payload[0] == '0') {
      digitalWrite(INPUT0, LOW);
      input0 = 0;
    } 
    else if (payload[0] == '1') {
      digitalWrite(INPUT0, HIGH);
      input0 = 1;
    }
  } 
  if (strcmp(topic,"diss/input1")==0) { 
        Serial.println("RICEVUTO INPUT1");
    if (payload[0] == '0') {
      digitalWrite(INPUT1, LOW);
      input1 = 0;
    } 
    else if (payload[0] == '1') {
      digitalWrite(INPUT1, HIGH);
      input1 = 1;
    }
  }
  if (strcmp(topic,"diss/input2")==0) { 
        Serial.println("RICEVUTO INPUT2");
    if (payload[0] == '0') {
      digitalWrite(INPUT2, LOW);
      input2 = 0;
    } 
    else if (payload[0] == '1') {
      digitalWrite(INPUT2, HIGH);
      input2 = 1;
    }
  } 
  if (strcmp(topic,"diss/input3")==0) { 
        Serial.println("RICEVUTO INPUT3");
    if (payload[0] == '0') {
      digitalWrite(INPUT3, LOW);
      input3 = 0;
    } 
    else if (payload[0] == '1') {
      digitalWrite(INPUT3, HIGH);
      input3 = 1;
    }
  }  
}


/*-------- NTP code ----------*/
/*
time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:                 
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}
*/

