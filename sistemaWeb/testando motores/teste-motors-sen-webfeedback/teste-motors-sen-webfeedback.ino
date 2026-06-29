//#include <Arduino.h>
//#include <AsyncTCP.h>
//#include <ESPAsyncWebServer.h>
//#include <DNSServer.h>
//
//// Configuração dos pinos para os dois motores
//int PUL1 = 33; // Pino de Pulso do Motor 1 (Pulse +)
//int DIR1 = 25; // Pino de Direção do Motor 1 (Dir +)
//int PUL2 = 32; // Pino de Pulso do Motor 2 (Pulse +)
//int DIR2 = 27; // Pino de Direção do Motor 2 (Dir +)
//
//// Configuração dos pinos para os sensores infravermelhos
//int sensor1 = 18; // Sensor do Motor Esquerdo
//int sensor2 = 19; // Sensor do Motor Direito
//
//// Configuração da rede Wi-Fi
//const char* ssid = "ESP32_Motor_Controller"; // Nome da rede Wi-Fi
//const char* password = "";                   // Senha (em branco)
//
//// Servidor web e DNS
//AsyncWebServer server(80);
//DNSServer dnsServer;
//
//int stepsPerRevolution = 6400; // Passos para uma revolução completa
//int currentPosition1 = 0; // Posição atual do Motor 1 em passos
//int currentPosition2 = 0; // Posição atual do Motor 2 em passos
//
//// Função de calibração
//void initializeMotors() {
//  bool sensor1Active = false;
//  bool sensor2Active = false;
//
//  while (!sensor1Active || !sensor2Active) {
//    if (!sensor1Active) {
//      digitalWrite(DIR1, HIGH); // Rotação no sentido desejado
//      digitalWrite(PUL1, HIGH);
//      delayMicroseconds(50);
//      digitalWrite(PUL1, LOW);
//      delayMicroseconds(50);
//      sensor1Active = digitalRead(sensor1) == LOW; // Estado invertido
//    }
//    if (!sensor2Active) {
//      digitalWrite(DIR2, LOW); // Rotação no sentido oposto
//      digitalWrite(PUL2, HIGH);
//      delayMicroseconds(50);
//      digitalWrite(PUL2, LOW);
//      delayMicroseconds(50);
//      sensor2Active = digitalRead(sensor2) == LOW; // Estado invertido
//    }
//  }
//
//  // Zera as posições ao terminar
//  currentPosition1 = 0;
//  currentPosition2 = 0;
//  Serial.println("Motores inicializados e zerados.");
//}
//
//// Função para mover ambos os motores com um slider único
//void moveMotorsWithSlider(int angle) {
//  int targetSteps = map(angle, 0, 360, 0, stepsPerRevolution);
//
//  int stepsToMove1 = targetSteps - currentPosition1;
//  int stepsToMove2 = targetSteps - currentPosition2;
//
//  digitalWrite(DIR1, stepsToMove1 >= 0 ? LOW : HIGH); // Define direção do Motor 1
//  digitalWrite(DIR2, stepsToMove2 >= 0 ? LOW : HIGH); // Define direção do Motor 2
//
//  stepsToMove1 = abs(stepsToMove1);
//  stepsToMove2 = abs(stepsToMove2);
//
//  int maxSteps = max(stepsToMove1, stepsToMove2);
//
//  for (int i = 0; i < maxSteps; i++) {
//    if (i < stepsToMove1) {
//      digitalWrite(PUL1, HIGH);
//    }
//    if (i < stepsToMove2) {
//      digitalWrite(PUL2, HIGH);
//    }
//    delayMicroseconds(50);
//    digitalWrite(PUL1, LOW);
//    digitalWrite(PUL2, LOW);
//    delayMicroseconds(50);
//  }
//
//  currentPosition1 = targetSteps;
//  currentPosition2 = targetSteps;
//}
//
//// Configuração do servidor web
//void webServerSetup() {
//  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
//    request->send(200, "text/html", R"rawliteral(
//      <!DOCTYPE html>
//      <html lang="pt-BR">
//      <head>
//          <meta charset="UTF-8">
//          <meta name="viewport" content="width=device-width, initial-scale=1.0">
//          <title>Controle de Motores</title>
//          <style>
//              body { font-family: Arial, sans-serif; text-align: center; margin-top: 50px; }
//              .slider { width: 80%; }
//              .container { max-width: 600px; margin: auto; }
//              #notification { color: green; font-weight: bold; display: none; margin-top: 20px; }
//          </style>
//      </head>
//      <body>
//          <div class="container">
//              <h2>Controle de Motores</h2>
//              <label for="angle">Ângulo (0-360): <span id="angleValue">0</span>°</label>
//              <input type="range" id="angle" min="0" max="360" value="0" class="slider" oninput="updateAngle(this.value)">
//              <button onclick="setAngle()">Definir Ângulo</button>
//              <button onclick="calibrateMotors()">Calibrar Motores</button>
//              <div id="notification">Calibração concluída!</div>
//          </div>
//          <script>
//              function updateAngle(value) {
//                  document.getElementById('angleValue').innerText = value;
//              }
//              function setAngle() {
//                  const angle = document.getElementById('angle').value;
//                  fetch(`/setAngle?value=${angle}`);
//              }
//              function calibrateMotors() {
//                  fetch('/calibrate')
//                      .then(response => {
//                          if (response.ok) {
//                              document.getElementById('notification').style.display = 'block';
//                          }
//                      });
//              }
//          </script>
//      </body>
//      </html>
//    )rawliteral");
//    Serial.println("Página de controle dos motores carregada");
//  });
//
//  // Rota para definir o ângulo com o slider
//  server.on("/setAngle", HTTP_GET, [](AsyncWebServerRequest *request) {
//    if (request->hasParam("value")) {
//      int angle = request->getParam("value")->value().toInt();
//      moveMotorsWithSlider(angle);
//      request->send(200, "text/plain", "Ângulo definido");
//      Serial.print("Motores movidos para ângulo: ");
//      Serial.println(angle);
//    } else {
//      request->send(400, "text/plain", "Parâmetro 'value' ausente");
//    }
//  });
//
//  // Rota para calibração
//  server.on("/calibrate", HTTP_GET, [](AsyncWebServerRequest *request) {
//    initializeMotors();
//    request->send(200, "text/plain", "Calibração concluída");
//    Serial.println("Calibração dos motores concluída");
//  });
//
//  // Redireciona qualquer outra rota para a página raiz
//  server.onNotFound([](AsyncWebServerRequest *request) {
//    request->redirect("/");
//  });
//
//  server.begin();
//  Serial.println("Servidor web iniciado");
//}
//
//void setup() {
//  Serial.begin(115200);
//
//  // Configura os pinos dos motores
//  pinMode(PUL1, OUTPUT);
//  pinMode(DIR1, OUTPUT);
//  pinMode(PUL2, OUTPUT);
//  pinMode(DIR2, OUTPUT);
//
//  // Configura os pinos dos sensores
//  pinMode(sensor1, INPUT);
//  pinMode(sensor2, INPUT);
//
//  // Configura o ESP32 como ponto de acesso
//  WiFi.softAP(ssid, password);
//  Serial.println("Wi-Fi iniciado");
//
//  // Exibe o IP do ponto de acesso
//  Serial.println(WiFi.softAPIP());
//
//  // Configura o DNS para redirecionar todas as requisições para o IP do ESP32
//  dnsServer.start(53, "*", WiFi.softAPIP());
//
//  // Inicializa os motores e o servidor web
//  webServerSetup();
//}
//
//void loop() {
//  dnsServer.processNextRequest();
//}









//====================================================================
//
//#include <Arduino.h>
//#include <AsyncTCP.h>
//#include <ESPAsyncWebServer.h>
//#include <DNSServer.h>
//
//// Configuração dos pinos para os dois motores
//int PUL1 = 33; // Pino de Pulso do Motor 1 (Pulse +)
//int DIR1 = 25; // Pino de Direção do Motor 1 (Dir +)
//int PUL2 = 32; // Pino de Pulso do Motor 2 (Pulse +)
//int DIR2 = 27; // Pino de Direção do Motor 2 (Dir +)
//
//// Configuração dos pinos para os sensores infravermelhos
//int sensor1 = 18; // Sensor do Motor Esquerdo
//int sensor2 = 19; // Sensor do Motor Direito
//
//// Configuração da rede Wi-Fi
//const char* ssid = "ESP32_Motor_Controller"; // Nome da rede Wi-Fi
//const char* password = "";                   // Senha (em branco)
//
//// Servidor web e DNS
//AsyncWebServer server(80);
//DNSServer dnsServer;
//
//int stepsPerRevolution = 6400; // Passos para uma revolução completa
//int currentPosition1 = 0; // Posição atual do Motor 1 em passos
//int currentPosition2 = 0; // Posição atual do Motor 2 em passos
//
//// Estado dos sensores
//String sensor1Status = "Inativo";
//String sensor2Status = "Inativo";
//
//void initializeMotors() {
//  bool sensor1Active = false;
//  bool sensor2Active = false;
//
//  while (!sensor1Active || !sensor2Active) {
//    if (!sensor1Active) {
//      digitalWrite(DIR1, HIGH); // Rotação no sentido desejado
//      digitalWrite(PUL1, HIGH);
//      delayMicroseconds(100); // Maior tempo aumenta a força
//      digitalWrite(PUL1, LOW);
//      delayMicroseconds(100);
//      sensor1Active = digitalRead(sensor1) == LOW; // Estado invertido
//    }
//    if (!sensor2Active) {
//      digitalWrite(DIR2, LOW); // Rotação no sentido oposto
//      digitalWrite(PUL2, HIGH);
//      delayMicroseconds(100); // Maior tempo aumenta a força
//      digitalWrite(PUL2, LOW);
//      delayMicroseconds(100);
//      sensor2Active = digitalRead(sensor2) == LOW; // Estado invertido
//    }
//  }
//
//  // Zera as posições ao terminar
//  currentPosition1 = 0;
//  currentPosition2 = 0;
//  Serial.println("Motores inicializados e zerados.");
//}
//
//void moveMotorsWithSlider(int angle) {
//  int targetSteps = map(angle, 0, 360, 0, stepsPerRevolution);
//
//  int stepsToMove1 = targetSteps - currentPosition1;
//  int stepsToMove2 = targetSteps - currentPosition2;
//
//  digitalWrite(DIR1, stepsToMove1 >= 0 ? LOW : HIGH); // Define direção do Motor 1
//  digitalWrite(DIR2, stepsToMove2 >= 0 ? LOW : HIGH); // Define direção do Motor 2
//
//  stepsToMove1 = abs(stepsToMove1);
//  stepsToMove2 = abs(stepsToMove2);
//
//  int maxSteps = max(stepsToMove1, stepsToMove2);
//
//  for (int i = 0; i < maxSteps; i++) {
//    if (i < stepsToMove1) {
//      digitalWrite(PUL1, HIGH);
//    }
//    if (i < stepsToMove2) {
//      digitalWrite(PUL2, HIGH);
//    }
//    delayMicroseconds(100); // Ajuste para maior força
//    digitalWrite(PUL1, LOW);
//    digitalWrite(PUL2, LOW);
//    delayMicroseconds(100);
//  }
//
//  currentPosition1 = targetSteps;
//  currentPosition2 = targetSteps;
//}
//
//// Configuração do servidor web
//void webServerSetup() {
//  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
//    request->send(200, "text/html", R"rawliteral(
//      <!DOCTYPE html>
//      <html lang="pt-BR">
//      <head>
//          <meta charset="UTF-8">
//          <meta name="viewport" content="width=device-width, initial-scale=1.0">
//          <title>Controle de Motores</title>
//          <style>
//              body { font-family: Arial, sans-serif; text-align: center; margin-top: 20px; }
//              .slider { width: 80%; }
//              .container { max-width: 600px; margin: auto; }
//              .sensor-status { margin-top: 20px; }
//              .calibration { margin-top: 30px; }
//          </style>
//      </head>
//      <body>
//          <div class="container">
//              <h2>Controle de Motores</h2>
//              <label for="angle">Ângulo (0-360): <span id="angleValue">0</span>°</label>
//              <input type="range" id="angle" min="0" max="360" value="0" class="slider" oninput="updateAngle(this.value)">
//              <button onclick="setAngle()">Definir Ângulo</button>
//              <div class="sensor-status">
//                  <p>Estado do Sensor 1: <span id="sensor1Status">Inativo</span></p>
//                  <p>Estado do Sensor 2: <span id="sensor2Status">Inativo</span></p>
//              </div>
//              <div class="calibration">
//                  <button onclick="calibrateMotors()">Calibrar Motores</button>
//                  <p id="calibrationStatus"></p>
//              </div>
//          </div>
//          <script>
//              function updateAngle(value) {
//                  document.getElementById('angleValue').innerText = value;
//              }
//              function setAngle() {
//                  const angle = document.getElementById('angle').value;
//                  fetch(`/setAngle?value=${angle}`);
//              }
//              function calibrateMotors() {
//                  fetch('/calibrate').then(response => {
//                      if (response.ok) {
//                          document.getElementById('calibrationStatus').innerText = 'Calibração concluída!';
//                      }
//                  });
//              }
//              setInterval(() => {
//                  fetch('/getSensorStatus')
//                      .then(response => response.json())
//                      .then(data => {
//                          document.getElementById('sensor1Status').innerText = data.sensor1;
//                          document.getElementById('sensor2Status').innerText = data.sensor2;
//                      });
//              }, 1000);
//          </script>
//      </body>
//      </html>
//    )rawliteral");
//  });
//
//  // Rota para definir o ângulo com o slider
//  server.on("/setAngle", HTTP_GET, [](AsyncWebServerRequest *request) {
//    if (request->hasParam("value")) {
//      int angle = request->getParam("value")->value().toInt();
//      moveMotorsWithSlider(angle);
//      request->send(200, "text/plain", "Ângulo definido");
//    } else {
//      request->send(400, "text/plain", "Parâmetro 'value' ausente");
//    }
//  });
//
//  // Rota para calibrar os motores
//  server.on("/calibrate", HTTP_GET, [](AsyncWebServerRequest *request) {
//    initializeMotors();
//    request->send(200, "text/plain", "Calibração concluída");
//  });
//
//  // Rota para obter o estado dos sensores
//  server.on("/getSensorStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
//    sensor1Status = digitalRead(sensor1) == LOW ? "Ativo" : "Inativo";
//    sensor2Status = digitalRead(sensor2) == LOW ? "Ativo" : "Inativo";
//    String json = "{\"sensor1\":\"" + sensor1Status + "\",\"sensor2\":\"" + sensor2Status + "\"}";
//    request->send(200, "application/json", json);
//  });
//
//  server.begin();
//}
//
//void setup() {
//  Serial.begin(115200);
//
//  // Configura os pinos dos motores
//  pinMode(PUL1, OUTPUT);
//  pinMode(DIR1, OUTPUT);
//  pinMode(PUL2, OUTPUT);
//  pinMode(DIR2, OUTPUT);
//
//  // Configura os pinos dos sensores
//  pinMode(sensor1, INPUT);
//  pinMode(sensor2, INPUT);
//
//  // Configura o ESP32 como ponto de acesso
//  WiFi.softAP(ssid, password);
//  Serial.println("Wi-Fi iniciado");
//  Serial.println(WiFi.softAPIP());
//
//  // Inicializa os motores e o servidor web
//  initializeMotors();
//  webServerSetup();
//}
//
//void loop() {
//  dnsServer.processNextRequest();
//}





//=========================================================================== ultimo

//
//#include <Arduino.h>
//#include <AsyncTCP.h>
//#include <ESPAsyncWebServer.h>
//#include <DNSServer.h>
//
//// Configuração dos pinos para os dois motores
//int PUL1 = 33; // Pino de Pulso do Motor 1 (Pulse +)
//int DIR1 = 25; // Pino de Direção do Motor 1 (Dir +)
//int PUL2 = 32; // Pino de Pulso do Motor 2 (Pulse +)
//int DIR2 = 27; // Pino de Direção do Motor 2 (Dir +)
//
//// Configuração dos pinos para os sensores infravermelhos
//int sensor1 = 18; // Sensor do Motor Esquerdo
//int sensor2 = 19; // Sensor do Motor Direito
//
//// Configuração da rede Wi-Fi
//const char* ssid = "ESP32_Motor_Controller"; // Nome da rede Wi-Fi
//const char* password = "";                   // Senha (em branco)
//
//// Servidor web e DNS
//AsyncWebServer server(80);
//DNSServer dnsServer;
//
//int stepsPerRevolution = 6400; // Passos para uma revolução completa
//int currentPosition1 = 0; // Posição atual do Motor 1 em passos
//int currentPosition2 = 0; // Posição atual do Motor 2 em passos
//
//// Estado dos sensores
//String sensor1Status = "Inativo";
//String sensor2Status = "Inativo";
//
//void initializeMotors() {
//  bool sensor1Active = false;
//  bool sensor2Active = false;
//
//  while (!sensor1Active || !sensor2Active) {
//    if (!sensor1Active) {
//      digitalWrite(DIR1, HIGH); // Rotação no sentido desejado
//      digitalWrite(PUL1, HIGH);
//      delayMicroseconds(100); // Maior tempo aumenta a força
//      digitalWrite(PUL1, LOW);
//      delayMicroseconds(100);
//      sensor1Active = digitalRead(sensor1) == LOW; // Estado invertido
//    }
//    if (!sensor2Active) {
//      digitalWrite(DIR2, LOW); // Rotação no sentido oposto
//      digitalWrite(PUL2, HIGH);
//      delayMicroseconds(100); // Maior tempo aumenta a força
//      digitalWrite(PUL2, LOW);
//      delayMicroseconds(100);
//      sensor2Active = digitalRead(sensor2) == LOW; // Estado invertido
//    }
//  }
//
//  // Zera as posições ao terminar
//  currentPosition1 = 0;
//  currentPosition2 = 0;
//  Serial.println("Motores inicializados e zerados.");
//}
//
//void moveMotorsWithSlider(int angle) {
//  int targetSteps = map(angle, 0, 360, 0, stepsPerRevolution);
//
//  int stepsToMove1 = targetSteps - currentPosition1;
//  int stepsToMove2 = targetSteps - currentPosition2;
//
//  digitalWrite(DIR1, stepsToMove1 >= 0 ? LOW : HIGH); // Define direção do Motor 1
//  digitalWrite(DIR2, stepsToMove2 >= 0 ? LOW : HIGH); // Define direção do Motor 2
//
//  stepsToMove1 = abs(stepsToMove1);
//  stepsToMove2 = abs(stepsToMove2);
//
//  int maxSteps = max(stepsToMove1, stepsToMove2);
//
//  for (int i = 0; i < maxSteps; i++) {
//    if (i < stepsToMove1) {
//      digitalWrite(PUL1, HIGH);
//    }
//    if (i < stepsToMove2) {
//      digitalWrite(PUL2, HIGH);
//    }
//    delayMicroseconds(100); // Ajuste para maior força
//    digitalWrite(PUL1, LOW);
//    digitalWrite(PUL2, LOW);
//    delayMicroseconds(100);
//  }
//
//  currentPosition1 = targetSteps;
//  currentPosition2 = targetSteps;
//}
//
//// Configuração do servidor web
//void webServerSetup() {
//  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
//    request->send(200, "text/html", R"rawliteral(
//      <!DOCTYPE html>
//      <html lang="pt-BR">
//      <head>
//          <meta charset="UTF-8">
//          <meta name="viewport" content="width=device-width, initial-scale=1.0">
//          <title>Controle de Motores</title>
//          <style>
//              body { font-family: Arial, sans-serif; text-align: center; margin-top: 20px; }
//              .slider { width: 80%; }
//              .container { max-width: 600px; margin: auto; }
//              .sensor-status { margin-top: 20px; }
//              .calibration { margin-top: 30px; }
//          </style>
//      </head>
//      <body>
//          <div class="container">
//              <h2>Controle de Motores</h2>
//              <label for="angle">Ângulo (0-360): <span id="angleValue">0</span>°</label>
//              <input type="range" id="angle" min="0" max="360" value="0" class="slider" oninput="updateAngle(this.value)">
//              <button onclick="setAngle()">Definir Ângulo</button>
//              <div class="sensor-status">
//                  <p>Estado do Sensor 1: <span id="sensor1Status">Inativo</span></p>
//                  <p>Estado do Sensor 2: <span id="sensor2Status">Inativo</span></p>
//              </div>
//              <div class="calibration">
//                  <button onclick="calibrateMotors()">Calibrar Motores</button>
//                  <p id="calibrationStatus"></p>
//              </div>
//          </div>
//          <script>
//              function updateAngle(value) {
//                  document.getElementById('angleValue').innerText = value;
//              }
//              function setAngle() {
//                  const angle = document.getElementById('angle').value;
//                  fetch(`/setAngle?value=${angle}`);
//              }
//              function calibrateMotors() {
//                  fetch('/calibrate').then(response => {
//                      if (response.ok) {
//                          document.getElementById('calibrationStatus').innerText = 'Calibração concluída!';
//                      }
//                  });
//              }
//              setInterval(() => {
//                  fetch('/getSensorStatus')
//                      .then(response => response.json())
//                      .then(data => {
//                          document.getElementById('sensor1Status').innerText = data.sensor1;
//                          document.getElementById('sensor2Status').innerText = data.sensor2;
//                      });
//              }, 1000);
//          </script>
//      </body>
//      </html>
//    )rawliteral");
//  });
//
//  // Rota para definir o ângulo com o slider
//  server.on("/setAngle", HTTP_GET, [](AsyncWebServerRequest *request) {
//    if (request->hasParam("value")) {
//      int angle = request->getParam("value")->value().toInt();
//      moveMotorsWithSlider(angle);
//      request->send(200, "text/plain", "Ângulo definido");
//    } else {
//      request->send(400, "text/plain", "Parâmetro 'value' ausente");
//    }
//  });
//
//  // Rota para calibrar os motores
//  server.on("/calibrate", HTTP_GET, [](AsyncWebServerRequest *request) {
//    initializeMotors();
//    request->send(200, "text/plain", "Calibração concluída");
//  });
//
//  // Rota para obter o estado dos sensores
//  server.on("/getSensorStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
//    sensor1Status = digitalRead(sensor1) == LOW ? "Ativo" : "Inativo";
//    sensor2Status = digitalRead(sensor2) == LOW ? "Ativo" : "Inativo";
//    String json = "{\"sensor1\":\"" + sensor1Status + "\",\"sensor2\":\"" + sensor2Status + "\"}";
//    request->send(200, "application/json", json);
//  });
//
//  // Redireciona qualquer outra rota para a página raiz
//  server.onNotFound([](AsyncWebServerRequest *request) {
//    request->redirect("/");
//  });
//  
//  server.begin();
//}
//
//void setup() {
//  Serial.begin(115200);
//
//  // Configura os pinos dos motores
//  pinMode(PUL1, OUTPUT);
//  pinMode(DIR1, OUTPUT);
//  pinMode(PUL2, OUTPUT);
//  pinMode(DIR2, OUTPUT);
//
//  // Configura os pinos dos sensores
//  pinMode(sensor1, INPUT);
//  pinMode(sensor2, INPUT);
//
//  // Configura o ESP32 como ponto de acesso
//  WiFi.softAP(ssid, password);
//  Serial.println("Wi-Fi iniciado");
//  Serial.println(WiFi.softAPIP());
//
//    // Configura o DNS para redirecionar todas as requisições para o IP do ESP32
//  dnsServer.start(53, "*", WiFi.softAPIP());
//
//  // Inicializa os motores e o servidor web
//  initializeMotors();
//  webServerSetup();
//}
//
//void loop() {
//  dnsServer.processNextRequest();
//}









//
//#include <Arduino.h>
//#include <AsyncTCP.h>
//#include <ESPAsyncWebServer.h>
//#include <DNSServer.h>
//
//// Configuração dos pinos para os motores
//#define STEP_PIN_X 33
//#define DIR_PIN_X 25
//#define STEP_PIN_Y 32
//#define DIR_PIN_Y 27
//
//// Configuração dos pinos para os sensores
//#define SENSOR1 18
//#define SENSOR2 19
//
//// Configuração da rede Wi-Fi
//const char* ssid = "ESP32_Motor_Controller"; // Nome da rede Wi-Fi
//const char* password = "";                   // Senha (em branco)
//
//// Servidor web e DNS
//AsyncWebServer server(80);
//DNSServer dnsServer;
//
//int stepsPerRevolution = 6400; // Passos para uma revolução completa
//int currentPositionX = 0;      // Posição atual do Motor X
//int currentPositionY = 0;      // Posição atual do Motor Y
//String sensor1Status = "Inativo";
//String sensor2Status = "Inativo";
//
//// Configuração inicial
//const int pulseDelay = 100; // Microsegundos entre pulsos (controla velocidade)
//
//// Função para mover um motor
//void moveStep(int stepPin, int dirPin, int direction) {
//  digitalWrite(dirPin, direction);
//  digitalWrite(stepPin, HIGH);
//  delayMicroseconds(pulseDelay);
//  digitalWrite(stepPin, LOW);
//  delayMicroseconds(pulseDelay);
//}
//
//// Função de calibração dos motores
//void calibraMotores(int stepPin, int dirPin, int sensorPin, int initialHoming, const char* motorName) {
//  Serial.print("Calibrando motor: ");
//  Serial.println(motorName);
//
//  // Fase 1: Mover até acionar o sensor
//  while (digitalRead(sensorPin)) {
//    moveStep(stepPin, dirPin, initialHoming > 0 ? HIGH : LOW);
//  }
//
//  // Fase 2: Mover até liberar o sensor
//  initialHoming = -initialHoming; // Inverter a direção
//  while (!digitalRead(sensorPin)) {
//    moveStep(stepPin, dirPin, initialHoming > 0 ? HIGH : LOW);
//  }
//
//  Serial.print("Motor ");
//  Serial.print(motorName);
//  Serial.println(": OK");
//}
//
//void initializeMotors() {
//  calibraMotores(STEP_PIN_X, DIR_PIN_X, SENSOR1, -1, "X");
//  delay(2000);
//  calibraMotores(STEP_PIN_Y, DIR_PIN_Y, SENSOR2, -1, "Y");
//  currentPositionX = 0;
//  currentPositionY = 0;
//  Serial.println("Motores inicializados e zerados.");
//}
//
//void moveMotorsWithSlider(int angle) {
//  int targetSteps = map(angle, 0, 360, 0, stepsPerRevolution);
//  int stepsToMoveX = targetSteps - currentPositionX;
//  int stepsToMoveY = targetSteps - currentPositionY;
//
//  digitalWrite(DIR_PIN_X, stepsToMoveX >= 0 ? LOW : HIGH);
//  digitalWrite(DIR_PIN_Y, stepsToMoveY >= 0 ? LOW : HIGH);
//
//  stepsToMoveX = abs(stepsToMoveX);
//  stepsToMoveY = abs(stepsToMoveY);
//
//  int maxSteps = max(stepsToMoveX, stepsToMoveY);
//
//  for (int i = 0; i < maxSteps; i++) {
//    if (i < stepsToMoveX) {
//      digitalWrite(STEP_PIN_X, HIGH);
//    }
//    if (i < stepsToMoveY) {
//      digitalWrite(STEP_PIN_Y, HIGH);
//    }
//    delayMicroseconds(100);
//    digitalWrite(STEP_PIN_X, LOW);
//    digitalWrite(STEP_PIN_Y, LOW);
//    delayMicroseconds(100);
//  }
//
//  currentPositionX = targetSteps;
//  currentPositionY = targetSteps;
//}
//
//// Configuração do servidor web
//void webServerSetup() {
//  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
//    request->send(200, "text/html", R"rawliteral(
//      <!DOCTYPE html>
//      <html lang="pt-BR">
//      <head>
//          <meta charset="UTF-8">
//          <meta name="viewport" content="width=device-width, initial-scale=1.0">
//          <title>Controle de Motores</title>
//          <style>
//              body { font-family: Arial, sans-serif; text-align: center; margin-top: 20px; }
//              .slider { width: 80%; }
//              .container { max-width: 600px; margin: auto; }
//              .sensor-status { margin-top: 20px; }
//              .calibration { margin-top: 30px; }
//          </style>
//      </head>
//      <body>
//          <div class="container">
//              <h2>Controle de Motores</h2>
//              <label for="angle">Ângulo (0-360): <span id="angleValue">0</span>°</label>
//              <input type="range" id="angle" min="0" max="360" value="0" class="slider" oninput="updateAngle(this.value)">
//              <button onclick="setAngle()">Definir Ângulo</button>
//              <div class="sensor-status">
//                  <p>Estado do Sensor 1: <span id="sensor1Status">Inativo</span></p>
//                  <p>Estado do Sensor 2: <span id="sensor2Status">Inativo</span></p>
//              </div>
//              <div class="calibration">
//                  <button onclick="calibrateMotors()">Calibrar Motores</button>
//                  <p id="calibrationStatus"></p>
//              </div>
//          </div>
//          <script>
//              function updateAngle(value) {
//                  document.getElementById('angleValue').innerText = value;
//              }
//              function setAngle() {
//                  const angle = document.getElementById('angle').value;
//                  fetch(`/setAngle?value=${angle}`);
//              }
//              function calibrateMotors() {
//                  fetch('/calibrate').then(response => {
//                      if (response.ok) {
//                          document.getElementById('calibrationStatus').innerText = 'Calibração concluída!';
//                      }
//                  });
//              }
//              setInterval(() => {
//                  fetch('/getSensorStatus')
//                      .then(response => response.json())
//                      .then(data => {
//                          document.getElementById('sensor1Status').innerText = data.sensor1;
//                          document.getElementById('sensor2Status').innerText = data.sensor2;
//                      });
//              }, 1000);
//          </script>
//      </body>
//      </html>
//    )rawliteral");
//  });
//
//  server.on("/setAngle", HTTP_GET, [](AsyncWebServerRequest *request) {
//    if (request->hasParam("value")) {
//      int angle = request->getParam("value")->value().toInt();
//      moveMotorsWithSlider(angle);
//      request->send(200, "text/plain", "Ângulo definido");
//    } else {
//      request->send(400, "text/plain", "Parâmetro 'value' ausente");
//    }
//  });
//
//  server.on("/calibrate", HTTP_GET, [](AsyncWebServerRequest *request) {
//    for(int i =0;i<5;i++){
//      initializeMotors();
//      delay(1000);
//    }
//    request->send(200, "text/plain", "Calibração concluída");
//  });
//
//  server.on("/getSensorStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
//    sensor1Status = digitalRead(SENSOR1) == LOW ? "Ativo" : "Inativo";
//    sensor2Status = digitalRead(SENSOR2) == LOW ? "Ativo" : "Inativo";
//    String json = "{\"sensor1\":\"" + sensor1Status + "\",\"sensor2\":\"" + sensor2Status + "\"}";
//    request->send(200, "application/json", json);
//  });
//
//  server.onNotFound([](AsyncWebServerRequest *request) {
//    request->redirect("/");
//  });
//
//  server.begin();
//}
//
//void setup() {
//  Serial.begin(115200);
//
//  pinMode(STEP_PIN_X, OUTPUT);
//  pinMode(DIR_PIN_X, OUTPUT);
//  pinMode(STEP_PIN_Y, OUTPUT);
//  pinMode(DIR_PIN_Y, OUTPUT);
//
//  pinMode(SENSOR1, INPUT_PULLUP);
//  pinMode(SENSOR2, INPUT_PULLUP);
//
//  WiFi.softAP(ssid, password);
//  Serial.println("Wi-Fi iniciado");
//  Serial.println(WiFi.softAPIP());
//
//  dnsServer.start(53, "*", WiFi.softAPIP());
//
//  for(int i =0;i<5;i++){
//    initializeMotors();
//    delay(1000);
//  }
//
//  
//  
//  webServerSetup();
//}
//
//void loop() {
//  dnsServer.processNextRequest();
//}
//
//





//ajustar =============================================================================================================== funcionando (ok)

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

//void initializeMotors() {
//  calibraMotores(STEP_PIN_X, DIR_PIN_X, SENSOR1, -1, "X");
//  calibraMotores(STEP_PIN_Y, DIR_PIN_Y, SENSOR2, -1, "Y");
//  currentPositionX = 0;
//  currentPositionY = 0;
//}

void initializeMotors() {
  for(int i =0;i<6;i++){
    calibraMotores(STEP_PIN_X, DIR_PIN_X, SENSOR1, -1, "X");
    calibraMotores(STEP_PIN_Y, DIR_PIN_Y, SENSOR2, -1, "Y");
    delay(1000);
  }

  currentPositionX = 0;
  currentPositionY = 0;
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
    delayMicroseconds(100);
    digitalWrite(STEP_PIN_X, LOW);
    digitalWrite(STEP_PIN_Y, LOW);
    delayMicroseconds(100);
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
