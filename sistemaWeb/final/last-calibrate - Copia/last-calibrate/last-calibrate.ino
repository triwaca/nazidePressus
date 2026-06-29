
#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>

// Configuração dos pinos para os motores
#define STEP_PIN_X 33
#define DIR_PIN_X 25
#define STEP_PIN_Y 32
#define DIR_PIN_Y 27

// Configuração dos pinos para os sensores
#define SENSOR1 18
#define SENSOR2 19

// Configuração da rede Wi-Fi
const char* ssid = "ESP32_Motor_Controller";
const char* password = ""; 

// Servidor web e DNS
AsyncWebServer server(80);
DNSServer dnsServer;

int stepsPerRevolution = 6400; 
int currentPositionX = 0;      
int currentPositionY = 0;      
String sensor1Status = "Inativo";
String sensor2Status = "Inativo";

// Configuração inicial
const int pulseDelay = 100; 

// Função para mover um motor
void moveStep(int stepPin, int dirPin, int direction) {
  digitalWrite(dirPin, direction);
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(pulseDelay);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(pulseDelay);
}

// Função de calibração dos motores
void calibraMotores(int stepPin, int dirPin, int sensorPin, int initialHoming, const char* motorName) {
  while (digitalRead(sensorPin)) {
    moveStep(stepPin, dirPin, initialHoming > 0 ? HIGH : LOW);
  }
  initialHoming = -initialHoming; 
  while (!digitalRead(sensorPin)) {
    moveStep(stepPin, dirPin, initialHoming > 0 ? HIGH : LOW);
  }
}

void initializeMotors() {
  for(int i =0;i<6;i++){
    calibraMotores(STEP_PIN_X, DIR_PIN_X, SENSOR1, -1, "X");
    calibraMotores(STEP_PIN_Y, DIR_PIN_Y, SENSOR2, -1, "Y");
    delay(1000);
  }

//  currentPositionX = 0;
//  currentPositionY = 0;

    moveMotorsWithSlider(51);// adicionei pra ele ja ir pro angulo
//   currentPositionX = 0;// talvez mudar aqui para 359
//   currentPositionY = 0;// ---

  currentPositionX = map(359, 0, 360, 0, stepsPerRevolution);
  currentPositionY = map(359, 0, 360, 0, stepsPerRevolution); 
//  Serial.println("Motores inicializados e zerados.");
}

void moveMotorsWithSlider(int angle) {
  int targetSteps = map(angle, 0, 360, 0, stepsPerRevolution);
  int stepsToMoveX = targetSteps - currentPositionX;
  int stepsToMoveY = targetSteps - currentPositionY;

  digitalWrite(DIR_PIN_X, stepsToMoveX >= 0 ? LOW : HIGH);
  digitalWrite(DIR_PIN_Y, stepsToMoveY >= 0 ? LOW : HIGH);

  stepsToMoveX = abs(stepsToMoveX);
  stepsToMoveY = abs(stepsToMoveY);

  int maxSteps = max(stepsToMoveX, stepsToMoveY);

  for (int i = 0; i < maxSteps; i++) {
    if (i < stepsToMoveX) {
      digitalWrite(STEP_PIN_X, HIGH);
    }
    if (i < stepsToMoveY) {
      digitalWrite(STEP_PIN_Y, HIGH);
    }
    delayMicroseconds(200);
    digitalWrite(STEP_PIN_X, LOW);
    digitalWrite(STEP_PIN_Y, LOW);
    delayMicroseconds(200);
  }

  currentPositionX = targetSteps;
  currentPositionY = targetSteps;
}

// Configuração do servidor web
void webServerSetup() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", R"rawliteral(
      <!DOCTYPE html>
      <html lang="pt-BR">
      <head>
          <meta charset="UTF-8">
          <meta name="viewport" content="width=device-width, initial-scale=1.0">
          <title>Controle de Motores</title>
          <style>
              body { font-family: Arial, sans-serif; text-align: center; margin-top: 20px; }
              .slider, input[type="number"] { width: 80%; }
              .container { max-width: 600px; margin: auto; }
              .sensor-status { margin-top: 20px; }
              .calibration { margin-top: 30px; }
              button { margin-top: 10px; padding: 10px 20px; font-size: 16px; border: none; border-radius: 5px; background-color: #007BFF; color: white; cursor: pointer; }
              button:hover { background-color: #0056b3; }
              input[type="number"] { padding: 8px; margin-top: 10px; border-radius: 5px; border: 1px solid #ccc; }
          </style>
      </head>
      <body>
          <div class="container">
              <h2>Controle de Motores</h2>
              <label for="angle">Ângulo (0-360): <span id="angleValue">0</span>°</label>
              <input type="range" id="angle" min="0" max="360" value="0" class="slider" oninput="updateAngle(this.value)">
              <button onclick="setAngle()">Definir Ângulo</button>
              
              <input type="number" id="manualAngle" min="0" max="360" placeholder="Digite o ângulo (0-360)">
              <button onclick="setManualAngle()">Definir Ângulo Manualmente</button>

              <div class="sensor-status">
                  <p>Estado do Sensor 1: <span id="sensor1Status">Inativo</span></p>
                  <p>Estado do Sensor 2: <span id="sensor2Status">Inativo</span></p>
              </div>
              <div class="calibration">
                  <button onclick="calibrateMotors()">Calibrar Motores</button>
                  <p id="calibrationStatus"></p>
              </div>
          </div>
          <script>
              function updateAngle(value) {
                  document.getElementById('angleValue').innerText = value;
              }
              function setAngle() {
                  const angle = document.getElementById('angle').value;
                  fetch(`/setAngle?value=${angle}`);
              }
              function setManualAngle() {
                  const angle = document.getElementById('manualAngle').value;
                  fetch(`/setAngle?value=${angle}`);
              }
              function calibrateMotors() {
                  fetch('/calibrate').then(response => {
                      if (response.ok) {
                          document.getElementById('calibrationStatus').innerText = 'Calibração concluída!';
                      }
                  });
              }
              setInterval(() => {
                  fetch('/getSensorStatus')
                      .then(response => response.json())
                      .then(data => {
                          document.getElementById('sensor1Status').innerText = data.sensor1;
                          document.getElementById('sensor2Status').innerText = data.sensor2;
                      });
              }, 1000);
          </script>
      </body>
      </html>
    )rawliteral");
  });

  server.on("/setAngle", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("value")) {
      int angle = request->getParam("value")->value().toInt();
      moveMotorsWithSlider(angle);
      request->send(200, "text/plain", "Ângulo definido");
    } else {
      request->send(400, "text/plain", "Parâmetro 'value' ausente");
    }
  });

  server.on("/calibrate", HTTP_GET, [](AsyncWebServerRequest *request) {
    initializeMotors();
    request->send(200, "text/plain", "Calibração concluída");
  });

  server.on("/getSensorStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
    sensor1Status = digitalRead(SENSOR1) == LOW ? "Ativo" : "Inativo";
    sensor2Status = digitalRead(SENSOR2) == LOW ? "Ativo" : "Inativo";
    String json = "{\"sensor1\":\"" + sensor1Status + "\",\"sensor2\":\"" + sensor2Status + "\"}";
    request->send(200, "application/json", json);
  });

  server.onNotFound([](AsyncWebServerRequest *request) {
    request->redirect("/");
  });

  server.begin();
}

void setup() {
  Serial.begin(115200);

  pinMode(STEP_PIN_X, OUTPUT);
  pinMode(DIR_PIN_X, OUTPUT);
  pinMode(STEP_PIN_Y, OUTPUT);
  pinMode(DIR_PIN_Y, OUTPUT);

  pinMode(SENSOR1, INPUT_PULLUP);
  pinMode(SENSOR2, INPUT_PULLUP);

  WiFi.softAP(ssid, password);

  dnsServer.start(53, "*", WiFi.softAPIP());

  initializeMotors();
  webServerSetup();
}

void loop() {
  dnsServer.processNextRequest();
}
