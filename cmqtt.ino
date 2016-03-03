// based on: https://github.com/Protoneer/MQTT-ESP8266-CLOUDMQTT.COM

#include <ESP8266WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PubSubClient.h>
#include <vector>
#include <functional>

#define ONE_WIRE_BUS 2
#define RELAY_GPIO 5
#define TEMPERATURE_PRECISION 9
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

String addrToString(DeviceAddress deviceAddress);
void printAddress(DeviceAddress deviceAddress);
void sendTemp();
void sendRelay();

struct DA
{
  DA(const DeviceAddress &address)
  {
    memcpy(addr, address, sizeof(DeviceAddress));
    strAddress = addrToString(addr); 
  }
  DeviceAddress addr;
  String strAddress;
};

using VofDA = std::vector<DA>;
VofDA gDevices;

const char *ssid =  "#YOUR_WIFI_SSID#";
const char *pass =  "#YOUR_WIFI_KEY#";

const char *mqtt_server = "#YOUR_MQQT_BROKER_SERVER#";
const int mqtt_port = 1111; // your MQTT BROKER PORT
const char *mqtt_user = "#YOUR_MQTT_USER#";
const char *mqtt_pass = "#YOUR_MQTT_PASS";
const char *mqtt_client_name = "#RANDOM_CLIENT_NAME#"; // Client connections cant have the same connection name

unsigned long previousMillis = 0;
const long interval = 10000;   

WiFiClient wclient;
PubSubClient client(wclient, mqtt_server, mqtt_port);
using MQTTCB = std::function<void(const MQTT::Publish&)>;

class Subscription
{
public:
  Subscription(String topic, MQTTCB cb): m_topic(topic), m_callback(cb) {};
  void subscribe(PubSubClient &client) const
  {
    client.set_callback(m_callback);
    client.subscribe(m_topic);
  }
private:
  String m_topic;
  MQTTCB m_callback;
};

class SubManager
{
public:
  SubManager(PubSubClient& c): m_client(c) {};
  void addSubscription(Subscription &sub)
  {
    m_subs.push_back(sub);
  }
  void doSubscriptions() const
  {
    for (const Subscription& s : m_subs)
      s.subscribe(m_client);
  }
private:
  PubSubClient& m_client;
  std::vector<Subscription> m_subs;
};

SubManager gSubMgr(client);

Subscription relay("home/esp8266/relay", [](const MQTT::Publish &pub) { 
  int val = pub.payload_string().toInt() ? 1 : 0;
  digitalWrite(RELAY_GPIO, val);  
});
  
void setup() 
{
  // Setup console
  Serial.begin(115200);
  delay(10);
  Serial.println();
  Serial.println();

  pinMode(RELAY_GPIO, OUTPUT);
  
  sensors.begin();
  int numberOfDevices = sensors.getDeviceCount();
  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(numberOfDevices, DEC);
  Serial.println(" devices.");

  for (int i=0;i<numberOfDevices;++i)
  {
    DeviceAddress addr;
    if(sensors.getAddress(addr, i))
    {
      gDevices.emplace_back(addr);
      Serial.print("Found device with address: ");
      Serial.print(gDevices.back().strAddress);
      Serial.println();
      sensors.setResolution(addr, TEMPERATURE_PRECISION);
    }
  }
  gSubMgr.addSubscription(relay);
}

void loop() 
{
  if (WiFi.status() != WL_CONNECTED) 
  {
    Serial.print("Connecting to ");
    Serial.print(ssid);
    Serial.println("...");
    WiFi.begin(ssid, pass);

    if (WiFi.waitForConnectResult() != WL_CONNECTED)
      return;
    Serial.println("WiFi connected");
  }

  if (WiFi.status() == WL_CONNECTED) 
  {
    if (!client.connected()) 
    {
      Serial.println("Connecting to MQTT server");
      if (client.connect(MQTT::Connect(mqtt_client_name)
                         .set_auth(mqtt_user, mqtt_pass))) 
      {
        Serial.println("Connected to MQTT server");
        gSubMgr.doSubscriptions();
      } 
      else 
        Serial.println("Could not connect to MQTT server");   
    }

    if (client.connected())
      client.loop();
  }
  static unsigned long lastSent = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - lastSent >= interval)
  {
    lastSent = currentMillis;
    sendTemp();
    sendRelay();
  }
  delay(100);
}

void sendRelay()
{
  int p = digitalRead(RELAY_GPIO) == HIGH ? 1 : 0;
  client.publish(String("home/esp8266/relay/status"),String(p)); 
}

void sendTemp()
{
  sensors.requestTemperatures(); 
  for(const DA &dev : gDevices)
  {
    float t = sensors.getTempC(dev.addr);
    Serial.print("Temperature: "); 
    Serial.print(t);
    Serial.println(" *C");
    client.publish(String("home/esp8266/sensors/")+dev.strAddress,String(t)+"C" );
  }
}

String addrToString(DeviceAddress deviceAddress)
{
  String ret;
  for (uint8_t i = 0; i < 8; i++)
    ret += String(deviceAddress[i], HEX);
  return ret;
}

void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
