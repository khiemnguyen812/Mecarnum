#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <IRremote.h>

const char* ssid = "Thiện";
const char* password = "minhhieu";

int D0 = 32;
int D1 = 33;
int D2 = 25;
int D3 = 26;
int D5 = 27;
int D6 = 14;
int D7 = 12;
int D8 = 13;

int trig_pin_front = 4;
int echo_pin_front = 0; 


int trig_pin_behind = 19;
int echo_pin_behind = 36; 

int ir_left = 18;
int ir_right = 5;

int led_turn_dark = 22;
int led_find_car=23;
int photo=34;
int buzzer=21;
int MQ135 = 39; 
//***Set server***
const char* mqttServer = "test.mosquitto.org"; 
int port = 1883;

int currentState = -1;
int oldState = -1;
int receivedState;

int receiveCloud=0;
int startRun=0,endRun=0;

//IFTTT cho thông báo tai nạn
const char* host = "maker.ifttt.com";
const int http_port = 80;
const char* requestDanger = "/trigger/thien/json/with/key/lkGCvNZ7zGBlsnxvde5iQqiWRotKnZ82WdlR98gJk9J";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

//Kiểm tra tình trạng của ultra trước và sau
bool lastStateDistanceFront = false;
bool currStateDistanceFront = false;
bool lastStateDistanceBehind = false;
bool currStateDistanceBehind = false;

//Kiểm tra tình trạng không khí
bool lastStateAir = false;
bool currStateAir = false;

//Gửi request thông báo nguy hiểm về KHOẢNG CÁCH
void sendRequestWhenDanger() {
  WiFiClient client;
  while(!client.connect(host, http_port)) {
    Serial.println("connection fail");
    delay(500);
  }
  client.print(String("GET ") + requestDanger + " HTTP/1.1\r\n"
              + "Host: " + host + "\r\n"
              + "Connection: close\r\n\r\n");
  delay(500);

  while(client.available()) {
    String line = client.readStringUntil('\r');
    Serial.print(line);
  }
}

long getDistance(int trig_pin, int echo_pin) {
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);
  long duration = pulseIn(echo_pin, HIGH);
  long distanceCm = duration * 0.034 / 2;
  return distanceCm;
}

void wifiConnect() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Connected!");
}

void mqttConnect() {
  while(!mqttClient.connected()) {
    Serial.println("Attemping MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if(mqttClient.connect(clientId.c_str())) {
      Serial.println("connected");

      //***Subscribe all topic you need***
      mqttClient.subscribe("OneDirection");
      Serial.println("subscibed");
    }
    else {
      Serial.println("try again in 5 seconds");
      delay(5000);
    }
  }
}

//MQTT Receiver
void callback(char* topic, byte* message, unsigned int length) {
  Serial.println(topic);
  String strMsg;
  for(int i=0; i<length; i++) {
    strMsg += (char)message[i];
  }
  Serial.println(strMsg);
  if(strMsg.length()>2) {
    const size_t capacity = JSON_OBJECT_SIZE(2) + 50; // Tăng giá trị 50 hoặc thay đổi theo nhu cầu

    // Tạo đối tượng JSON
    StaticJsonDocument<capacity> doc;

    // Phân tích chuỗi JSON
    DeserializationError error = deserializeJson(doc, strMsg);

    // Kiểm tra lỗi phân tích JSON
    if (error) {
      Serial.print("Failed to parse JSON: ");
      Serial.println(error.c_str());
      return;
    }

    // Lấy giá trị từ đối tượng JSON
    const char* distance = doc["time"];
    const char* date = doc["date"];

    // In kết quả
    Serial.println("Parsed JSON:");
    Serial.print("Time: ");
    Serial.println(distance);
    Serial.print("Date: ");
    Serial.println(date);

    receiveCloud=1;
  }
  else {
    receivedState = strMsg.toInt(); // Chuyển đổi strMsg thành số nguyên

    if(receivedState == 0) {
      ledPingpong_buzzer(led_find_car);
    }

    
    if(receivedState == currentState || receivedState==-1) {
      stopRobot();
      oldState=currentState;
      currentState=-1;
    } else {
        if (receivedState == Fmq132 && getDistance(trig_pin_front,echo_pin_front)>20){
          forward();
        }
        else if (receivedState == 7 && getDistance(trig_pin_behind,echo_pin_behind)>20 ){
          back();
        }
        else if (receivedState == 5 && digitalRead(ir_right)){
          right();
        }
        else if (receivedState == 4 && digitalRead(ir_left)){
          left();
        }
        else if (receivedState == 1 && !(getDistance(trig_pin_front,echo_pin_front) <= 20 || digitalRead(ir_left) == 0)){
          forwardleft();
        }
        else if (receivedState == 3 && !(getDistance(trig_pin_front,echo_pin_front) <= 20 || digitalRead(ir_right) == 0)){
          forwardright();
        }
        else if (receivedState == 6 && !(getDistance(trig_pin_behind,echo_pin_behind) <= 20 || digitalRead(ir_left) == 0)){
          backleft();
        }
        else if (receivedState == 8 && !(getDistance(trig_pin_behind,echo_pin_behind) <= 20 || digitalRead(ir_right) == 0)){
          backright();
        }
        else if (receivedState==9) {
          rotateleft();
        }
        else if (receivedState==10) {
          rotateright();
        }
        currentState = receivedState; // Cập nhật currentState với giá trị mới nhận được
    }
  }
  // Code xử lý gói tin nhận được ở đây
}

void ledPingpong_buzzer(int led) {
  for(int i=0;i<7q;i++) {
    digitalWrite(led,HIGH);
    buzzerRing();
    delay(500);
    digitalWrite(led,LOW);
    delay(500);
  }
}

void setup() {
  Serial.begin(9600);
  Serial.print("Connecting to WiFi");

  wifiConnect();
  mqttClient.setServer(mqttServer, port);
  mqttClient.setCallback(callback);
  mqttClient.setKeepAlive( 90 );

  
  pinMode(trig_pin_front, OUTPUT);
  pinMode(echo_pin_front, INPUT);

  pinMode(trig_pin_behind, OUTPUT);
  pinMode(echo_pin_behind, INPUT);

  pinMode(D0, OUTPUT);
  digitalWrite(D0,LOW);
  pinMode(D1, OUTPUT);
  digitalWrite(D1,LOW);
  pinMode(D2, OUTPUT);
  digitalWrite(D2,LOW);
  pinMode(D3, OUTPUT);
  digitalWrite(D3,LOW);
  pinMode(D5, OUTPUT);
  digitalWrite(D5,LOW);
  pinMode(D6, OUTPUT);
  digitalWrite(D6,LOW);
  pinMode(D7, OUTPUT);
  digitalWrite(D7,LOW);
  pinMode(D8, OUTPUT);
  digitalWrite(D8,LOW);
  
  
  pinMode(MQ135, INPUT);
  pinMode(ir_left, INPUT); 
  pinMode(ir_right, INPUT); 

  pinMode(led_turn_dark, OUTPUT);
  pinMode(led_find_car, OUTPUT);
  pinMode(photo,INPUT);
  pinMode(buzzer,OUTPUT);
  digitalWrite(led_turn_dark,LOW);

}

void upDistanceToWeb_Front(int dis) {
  char buffer[50];
  sprintf(buffer,"%d cm",dis);
  mqttClient.publish("Group_One_Receive_Distance_Front",buffer);
}

void upDistanceToWeb_Behind(int dis) {
  char buffer[50];
  sprintf(buffer,"%d cm",dis);
  mqttClient.publish("Group_One_Receive_Distance_Behind",buffer);
}

void upAirToWeb(int CO2) {
  char buffer[50];
  sprintf(buffer,"%d",CO2);
  mqttClient.publish("Group_One_Receive_Air",buffer);
}

void buzzerRing() {
 tone(buzzer,200,50);
}

//Kiểm tra nghi hiểm về khoảng cách
//dir: true -> front
//dir: false -> behind
void checkDanger(int distance, bool dir) {
  //front
  if (dir){
    if (distance <= 5)
      currStateDistanceFront = true;
    else currStateDistanceFront = false;

    if (!lastStateDistanceFront && currStateDistanceFront){
      Serial.println("DANGER!!!");
      sendRequestWhenDanger();
    }    
    lastStateDistanceFront = currStateDistanceFront;
  }
  //behind
  else{
    if (distance <= 10)
      currStateDistanceBehind = true;
    else currStateDistanceBehind = false;

    if (!lastStateDistanceBehind && currStateDistanceBehind){
      Serial.println("DANGER!!!");
      sendRequestWhenDanger();
    }
    lastStateDistanceBehind = currStateDistanceBehind;
  }
}

//Kiểm tra nguy hiểm về chất lượng không khí
void checkAir(int CO2) {
  if (CO2 >= 3500){
    currStateAir = true;
    buzzerRing();
  }
  else currStateAir = false;

  if (!lastStateAir && currStateAir){
    Serial.println("POLLUTED AIR!!!");
  }    
  lastStateAir = currStateAir;
}

int step = 0;

void sendTimeWeb() {
  if(currentState==-1 && oldState!=-1 && oldState!=-2) {
    int temp = abs(endRun-startRun)/1000;
    Serial.print("Day la thoi gian: ");
    Serial.println(temp);
    char buffer[50];
    delay(200);
    sprintf(buffer,"%d",temp);
    if(step==0) {
      mqttClient.publish("Group_One_Receive_Time_1",buffer);
      step=1;
    }
    if(step==1) {
      mqttClient.publish("Group_One_Receive_Time_2",buffer);
      step=0;
    }
    oldState=-2;
  }

  if(currentState!=-1) {
    endRun=millis();
  } else startRun=millis();

}

void checkLight() {
  if(digitalRead(photo) == 1) digitalWrite(led_turn_dark,HIGH);
  else digitalWrite(led_turn_dark,LOW);
}

void checkSafe(int distance_front, int distance_behind, int irStatus_left, int irStatus_right) {
  if ((distance_front <= 25 && currentState == 2) || (distance_behind <= 25 && currentState == 7) || (irStatus_left == 0 && currentState == 4) || (irStatus_right == 0 && currentState == 5)) {
    stopRobot();
    oldState = currentState;
    currentState = -1;
  }

  // Kiểm tra khi robot đang điều hướng forwardleft()
  if (currentState == 1 && (distance_front <= 25 || irStatus_left == 0)) {
    stopRobot();
    oldState = currentState;
    currentState = -1;
  }

  if (currentState == 3 && (distance_front <= 25 || irStatus_right == 0)) {
    stopRobot();
    oldState = currentState;
    currentState = -1;
  }

  if (currentState == 6 && (distance_behind <= 25 || irStatus_left == 0)) {
    stopRobot();
    oldState = currentState;
    currentState = -1;
  }

  if (currentState == 8 && (distance_behind <= 25 || irStatus_right == 0)) {
    stopRobot();
    oldState = currentState;
    currentState = -1;
  }
  sendTimeWeb();
}


 
void loop() {
  int CO2 = analogRead(MQ135);

 if(!mqttClient.connected()) {
    mqttConnect();
  }
  mqttClient.loop();
  
  while(receiveCloud==0) {
    Serial.println("Waiting for data from cloud!!!");
    mqttClient.loop();
    buzzerRing();
  }
  
  delay(100);

  int distance_front=getDistance(trig_pin_front,echo_pin_front);  
  int distance_behind=getDistance(trig_pin_behind,echo_pin_behind);  
  int irStatus_left = digitalRead(ir_left); 
  int irStatus_right = digitalRead(ir_right); 

  checkSafe(distance_front,distance_behind,irStatus_left,irStatus_right);
  checkDanger(distance_front, true);
  checkDanger(distance_behind, false);
  checkLight();
  checkAir(CO2);
  upDistanceToWeb_Front(distance_front);
  upDistanceToWeb_Behind(distance_behind);
  upAirToWeb(CO2);
  
  sendTimeWeb();

  delay(100);
}

void stopRobot() {
  digitalWrite(D0, LOW);
  digitalWrite(D1, LOW);

  digitalWrite(D2, LOW);
  digitalWrite(D3, LOW);
  
  digitalWrite(D5, LOW);
  digitalWrite(D6, LOW);
  
  digitalWrite(D7, LOW);
  digitalWrite(D8, LOW);
  
}

void forward() {
  digitalWrite(D0, HIGH);
  digitalWrite(D1, LOW);
  
  digitalWrite(D2, HIGH);
  digitalWrite(D3, LOW);
  
  digitalWrite(D8, LOW);
  digitalWrite(D7, HIGH);
  
  digitalWrite(D6, HIGH);
  digitalWrite(D5, LOW);
  
}

void back() {
  digitalWrite(D0, LOW);
  digitalWrite(D1, HIGH);
  
  digitalWrite(D2, LOW);
  digitalWrite(D3, HIGH);

  digitalWrite(D8, HIGH);
  digitalWrite(D7, LOW);
 
  digitalWrite(D6, LOW);
  digitalWrite(D5, HIGH);
  
}

void right() {
  digitalWrite(D0, LOW);
  digitalWrite(D1, HIGH);

  digitalWrite(D2, HIGH);
  digitalWrite(D3, LOW);
  
  digitalWrite(D5, LOW);
  digitalWrite(D6, HIGH);
 
  digitalWrite(D7, LOW);
  digitalWrite(D8, HIGH);
}

void left () {
  digitalWrite(D0, HIGH);
  digitalWrite(D1, LOW);

  digitalWrite(D2, LOW);
  digitalWrite(D3, HIGH);
 
  digitalWrite(D5, HIGH);
  digitalWrite(D6, LOW);
  
  digitalWrite(D7, HIGH);
  digitalWrite(D8, LOW);
}

void forwardright () {
  digitalWrite(D0, LOW);
  digitalWrite(D1, LOW);
  
  digitalWrite(D2, HIGH);
  digitalWrite(D3, LOW);

  digitalWrite(D8, LOW);
  digitalWrite(D7, LOW);

  digitalWrite(D6, HIGH);
  digitalWrite(D5, LOW);
  
}

void forwardleft () {
  digitalWrite(D0, HIGH);
  digitalWrite(D1, LOW);

  digitalWrite(D2, LOW);
  digitalWrite(D3, LOW);

  digitalWrite(D8, LOW);
  digitalWrite(D7, HIGH);

  digitalWrite(D6, LOW);
  digitalWrite(D5, LOW);
}

void backleft () {
  digitalWrite(D0, LOW);
  digitalWrite(D1, LOW);
  
  digitalWrite(D2, LOW);
  digitalWrite(D3, HIGH);

  digitalWrite(D8, LOW);
  digitalWrite(D7, LOW);

  digitalWrite(D6, LOW);
  digitalWrite(D5, HIGH);  
}

void backright () {
  digitalWrite(D0, LOW);
  digitalWrite(D1, HIGH);

  digitalWrite(D2, LOW);
  digitalWrite(D3, LOW);

  digitalWrite(D8, HIGH);
  digitalWrite(D7, LOW);

  digitalWrite(D6, LOW);
  digitalWrite(D5, LOW);
}

void rotateright () {
  digitalWrite(D0, HIGH);
  digitalWrite(D1, LOW);
  
  digitalWrite(D2, LOW);
  digitalWrite(D3, HIGH);

  digitalWrite(D8, HIGH);
  digitalWrite(D7, LOW);

  digitalWrite(D6, HIGH);
  digitalWrite(D5, LOW);
}

void rotateleft () {
  digitalWrite(D0, LOW);
  digitalWrite(D1, HIGH);
  
  digitalWrite(D2, HIGH);
  digitalWrite(D3, LOW);

  digitalWrite(D8, LOW);
  digitalWrite(D7, HIGH);

  digitalWrite(D6, LOW);
  digitalWrite(D5, HIGH);
}
