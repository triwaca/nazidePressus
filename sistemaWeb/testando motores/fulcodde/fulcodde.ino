////#include <Arduino.h>
////#include <AsyncTCP.h>
////#include <ESPAsyncWebServer.h>
////#include <DNSServer.h>
////
////// Configuração do motor
////int PUL = 25; // Pino de Pulso
////int DIR = 26; // Pino de Direção
////int ENA = 27; // Pino de Ativação
////
////// Configuração da rede Wi-Fi
////const char* ssid = "ESP32_Motor_Controller"; // Nome da rede Wi-Fi
////const char* password = "";                   // Senha (em branco)
////
////// Servidor web e DNS
////AsyncWebServer server(80);
////DNSServer dnsServer;
////
////int stepsPerRevolution = 6400; // Exemplo: quantidade de passos para uma revolução completa
////int currentPosition = 0; // Armazena a posição atual do motor em passos
////
////// Função para mover o motor para um ângulo específico
////void moveToAngle(int angle) {
////  int targetSteps = map(angle, 0, 360, 0, stepsPerRevolution);
////  int stepsToMove = targetSteps - currentPosition;
////  
////  digitalWrite(DIR, stepsToMove >= 0 ? HIGH : LOW); // Define a direção
////
////  // Converte para positivo se for negativo
////  stepsToMove = abs(stepsToMove);
////
////  for (int i = 0; i < stepsToMove; i++) {
////    digitalWrite(ENA, HIGH);
////    digitalWrite(PUL, HIGH);
////    delayMicroseconds(50);
////    digitalWrite(PUL, LOW);
////    delayMicroseconds(50);
////  }
////
////  // Atualiza a posição atual
////  currentPosition = targetSteps;
////}
////
////// Função para definir um novo marco inicial
////void resetPosition() {
////  currentPosition = 0;
////}
////
////void webServerSetup() {
////  // Página inicial com controle de ângulo e botão de reset
////  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
////    request->send(200, "text/html", R"rawliteral(
////      <!DOCTYPE html>
////      <html lang="pt-BR">
////      <head>
////          <meta charset="UTF-8">
////          <meta name="viewport" content="width=device-width, initial-scale=1.0">
////          <title>Controle de Motor</title>
////          <style>
////              body { font-family: Arial, sans-serif; text-align: center; margin-top: 50px; }
////              .slider { width: 80%; }
////              .container { max-width: 600px; margin: auto; }
////              button { padding: 10px 20px; font-size: 16px; margin-top: 20px; }
////          </style>
////      </head>
////      <body>
////          <div class="container">
////              <h2>Controle do Motor</h2>
////              <label for="angle">Ângulo (0-360): <span id="angleValue">0</span>°</label>
////              <input type="range" id="angle" min="0" max="360" value="0" class="slider" oninput="updateAngle(this.value)">
////              <button onclick="setAngle()">Definir Ângulo</button>
////              <button onclick="resetMotor()">Zerar Posição</button>
////          </div>
////          <script>
////              function updateAngle(value) {
////                  document.getElementById('angleValue').innerText = value;
////              }
////
////              function setAngle() {
////                  const angle = document.getElementById('angle').value;
////                  fetch(`/setAngle?value=${angle}`);
////              }
////
////              function resetMotor() {
////                  fetch(`/resetPosition`);
////              }
////          </script>
////      </body>
////      </html>
////    )rawliteral");
////    Serial.println("Página de controle do motor carregada");
////  });
////
////  // Rota para definir o ângulo
////  server.on("/setAngle", HTTP_GET, [](AsyncWebServerRequest *request){
////    if (request->hasParam("value")) {
////      int angle = request->getParam("value")->value().toInt();
////      moveToAngle(angle);
////      request->send(200, "text/plain", "Ângulo definido");
////      Serial.print("Ângulo movido para: ");
////      Serial.println(angle);
////    } else {
////      request->send(400, "text/plain", "Parâmetro 'value' ausente");
////    }
////  });
////
////  // Rota para redefinir a posição do motor
////  server.on("/resetPosition", HTTP_GET, [](AsyncWebServerRequest *request){
////    resetPosition();
////    request->send(200, "text/plain", "Posição redefinida para zero");
////    Serial.println("Posição redefinida para zero");
////  });
////
////  // Redireciona qualquer outra rota para a página raiz
////  server.onNotFound([](AsyncWebServerRequest *request){
////    request->redirect("/");
////  });
////
////  // Inicia o servidor
////  server.begin();
////  Serial.println("Servidor web iniciado");
////}
////
////void setup() {
////  Serial.begin(115200);
////
////  // Configura os pinos do motor
////  pinMode(PUL, OUTPUT);
////  pinMode(DIR, OUTPUT);
////  pinMode(ENA, OUTPUT);
////
////  // Configura o ESP32 como ponto de acesso
////  WiFi.softAP(ssid, password);
////  Serial.println("Wi-Fi iniciado");
////
////  // Exibe o IP do ponto de acesso
////  Serial.println(WiFi.softAPIP());
////
////  // Configura o DNS para redirecionar todas as requisições para o IP do ESP32
////  dnsServer.start(53, "*", WiFi.softAPIP());
////
////  // Configura as rotas do servidor web
////  webServerSetup();
////
////  Serial.println("Setup completo");
////}
////
////void loop() {
////  dnsServer.processNextRequest(); // Mantém o DNS ativo para o portal cativo
////}
//
//
//#include <Arduino.h>
//#include <AsyncTCP.h>
//#include <ESPAsyncWebServer.h>
//#include <DNSServer.h>
//
//// Configuração dos pinos para o driver TB6600
////int PUL = 25; // Pino de Pulso (Pulse +)
////int DIR = 26; // Pino de Direção (Dir +)
//
//int PUL = 33; // Pino de Pulso (Pulse +)
//int DIR = 25; // Pino de Direção (Dir +)
//
//// Configuração da rede Wi-Fi
//const char* ssid = "ESP32_Motor_Controller"; // Nome da rede Wi-Fi
//const char* password = "";                   // Senha (em branco)
//
//// Servidor web e DNS
//AsyncWebServer server(80);
//DNSServer dnsServer;
//
//int stepsPerRevolution = 6400; // Quantidade de passos para uma revolução completa
//int currentPosition = 0; // Posição atual do motor em passos
//
//// Função para mover o motor para um ângulo específico
//void moveToAngle(int angle) {
//  int targetSteps = map(angle, 0, 360, 0, stepsPerRevolution);
//  int stepsToMove = targetSteps - currentPosition;
//  
//  digitalWrite(DIR, stepsToMove >= 0 ? HIGH : LOW); // Define a direção
//
//  // Converte para positivo se for negativo
//  stepsToMove = abs(stepsToMove);
//
//  for (int i = 0; i < stepsToMove; i++) {
//    digitalWrite(PUL, HIGH);
//    delayMicroseconds(50);
//    digitalWrite(PUL, LOW);
//    delayMicroseconds(50);
//  }
//
//  // Atualiza a posição atual
//  currentPosition = targetSteps;
//}
//
//// Função para definir um novo marco inicial
//void resetPosition() {
//  currentPosition = 0;
//}
//
//void webServerSetup() {
//  // Página inicial com controle de ângulo e botão de reset
//  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
//    request->send(200, "text/html", R"rawliteral(
//      <!DOCTYPE html>
//      <html lang="pt-BR">
//      <head>
//          <meta charset="UTF-8">
//          <meta name="viewport" content="width=device-width, initial-scale=1.0">
//          <title>Controle de Motor</title>
//          <style>
//              body { font-family: Arial, sans-serif; text-align: center; margin-top: 50px; }
//              .slider { width: 80%; }
//              .container { max-width: 600px; margin: auto; }
//              button { padding: 10px 20px; font-size: 16px; margin-top: 20px; }
//          </style>
//      </head>
//      <body>
//          <div class="container">
//              <h2>Controle do Motor</h2>
//              <label for="angle">Ângulo (0-360): <span id="angleValue">0</span>°</label>
//              <input type="range" id="angle" min="0" max="360" value="0" class="slider" oninput="updateAngle(this.value)">
//              <button onclick="setAngle()">Definir Ângulo</button>
//              <button onclick="resetMotor()">Zerar Posição</button>
//          </div>
//          <script>
//              function updateAngle(value) {
//                  document.getElementById('angleValue').innerText = value;
//              }
//
//              function setAngle() {
//                  const angle = document.getElementById('angle').value;
//                  fetch(`/setAngle?value=${angle}`);
//              }
//
//              function resetMotor() {
//                  fetch(`/resetPosition`);
//              }
//          </script>
//      </body>
//      </html>
//    )rawliteral");
//    Serial.println("Página de controle do motor carregada");
//  });
//
//  // Rota para definir o ângulo
//  server.on("/setAngle", HTTP_GET, [](AsyncWebServerRequest *request){
//    if (request->hasParam("value")) {
//      int angle = request->getParam("value")->value().toInt();
//      moveToAngle(angle);
//      request->send(200, "text/plain", "Ângulo definido");
//      Serial.print("Ângulo movido para: ");
//      Serial.println(angle);
//    } else {
//      request->send(400, "text/plain", "Parâmetro 'value' ausente");
//    }
//  });
//
//  // Rota para redefinir a posição do motor
//  server.on("/resetPosition", HTTP_GET, [](AsyncWebServerRequest *request){
//    resetPosition();
//    request->send(200, "text/plain", "Posição redefinida para zero");
//    Serial.println("Posição redefinida para zero");
//  });
//
//  // Redireciona qualquer outra rota para a página raiz
//  server.onNotFound([](AsyncWebServerRequest *request){
//    request->redirect("/");
//  });
//
//  // Inicia o servidor
//  server.begin();
//  Serial.println("Servidor web iniciado");
//}
//
//void setup() {
//  Serial.begin(115200);
//
//  // Configura os pinos do motor
//  pinMode(PUL, OUTPUT);
//  pinMode(DIR, OUTPUT);
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
//  // Configura as rotas do servidor web
//  webServerSetup();
//
//  Serial.println("Setup completo");
//}
//
//void loop() {
//  dnsServer.processNextRequest(); // Mantém o DNS ativo para o portal cativo
//}






#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>

// Configuração dos pinos para os dois motores
int PUL1 = 33; // Pino de Pulso do Motor 1 (Pulse +)
int DIR1 = 25; // Pino de Direção do Motor 1 (Dir +)
int PUL2 = 32; // Pino de Pulso do Motor 2 (Pulse +)
int DIR2 = 27; // Pino de Direção do Motor 2 (Dir +


// Configuração da rede Wi-Fi
const char* ssid = "ESP32_Motor_Controller"; // Nome da rede Wi-Fi
const char* password = "";                   // Senha (em branco)

// Servidor web e DNS
AsyncWebServer server(80);
DNSServer dnsServer;

int stepsPerRevolution = 6400; // Passos para uma revolução completa
int currentPosition1 = 0; // Posição atual do Motor 1 em passos
int currentPosition2 = 0; // Posição atual do Motor 2 em passos

// Função para mover o Motor 1 para um ângulo específico
void moveToAngle1(int angle) {
  int targetSteps = map(angle, 0, 360, 0, stepsPerRevolution);
  int stepsToMove = targetSteps - currentPosition1;
  
  // Define a direção do motor
  digitalWrite(DIR1, stepsToMove >= 0 ? LOW : HIGH);
  stepsToMove = abs(stepsToMove); // Converte para positivo, caso seja negativo

  // Realiza os passos necessários para o ângulo especificado
  for (int i = 0; i < stepsToMove; i++) {
    digitalWrite(PUL1, HIGH);
    delayMicroseconds(50);
    digitalWrite(PUL1, LOW);
    delayMicroseconds(50);
  }

  // Atualiza a posição atual
  currentPosition1 = targetSteps;
}

// Função para mover o Motor 2 para um ângulo específico
void moveToAngle2(int angle) {
  int targetSteps = map(angle, 0, 360, 0, stepsPerRevolution);
  int stepsToMove = targetSteps - currentPosition2;

  // Define a direção do motor
  digitalWrite(DIR2, stepsToMove >= 0 ? LOW : HIGH);
  stepsToMove = abs(stepsToMove); // Converte para positivo, caso seja negativo

  // Realiza os passos necessários para o ângulo especificado
  for (int i = 0; i < stepsToMove; i++) {
    digitalWrite(PUL2, HIGH);
    delayMicroseconds(50);
    digitalWrite(PUL2, LOW);
    delayMicroseconds(50);
  }

  // Atualiza a posição atual
  currentPosition2 = targetSteps;
}

// Funções para redefinir a posição dos motores
void resetPosition1() { currentPosition1 = 0; }
void resetPosition2() { currentPosition2 = 0; }

void webServerSetup() {
  // Página inicial com controles para os dois motores
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", R"rawliteral(
      <!DOCTYPE html>
      <html lang="pt-BR">
      <head>
          <meta charset="UTF-8">
          <meta name="viewport" content="width=device-width, initial-scale=1.0">
          <title>Controle de Motores</title>
          <style>
              body { font-family: Arial, sans-serif; text-align: center; margin-top: 50px; }
              .slider { width: 80%; }
              .container { max-width: 600px; margin: auto; }
              button { padding: 10px 20px; font-size: 16px; margin-top: 20px; }
          </style>
      </head>
      <body>
          <div class="container">
              <h2>Controle do Motor 1</h2>
              <label for="angle1">Ângulo Motor 1 (0-360): <span id="angleValue1">0</span>°</label>
              <input type="range" id="angle1" min="0" max="360" value="0" class="slider" oninput="updateAngle1(this.value)">
              <button onclick="setAngle1()">Definir Ângulo Motor 1</button>
              <button onclick="resetMotor1()">Zerar Posição Motor 1</button>

              <h2>Controle do Motor 2</h2>
              <label for="angle2">Ângulo Motor 2 (0-360): <span id="angleValue2">0</span>°</label>
              <input type="range" id="angle2" min="0" max="360" value="0" class="slider" oninput="updateAngle2(this.value)">
              <button onclick="setAngle2()">Definir Ângulo Motor 2</button>
              <button onclick="resetMotor2()">Zerar Posição Motor 2</button>
          </div>
          <script>
              function updateAngle1(value) {
                  document.getElementById('angleValue1').innerText = value;
              }
              function setAngle1() {
                  const angle = document.getElementById('angle1').value;
                  fetch(`/setAngle1?value=${angle}`);
              }
              function resetMotor1() {
                  fetch(`/resetPosition1`);
              }

              function updateAngle2(value) {
                  document.getElementById('angleValue2').innerText = value;
              }
              function setAngle2() {
                  const angle = document.getElementById('angle2').value;
                  fetch(`/setAngle2?value=${angle}`);
              }
              function resetMotor2() {
                  fetch(`/resetPosition2`);
              }
          </script>
      </body>
      </html>
    )rawliteral");
    Serial.println("Página de controle dos motores carregada");
  });

  // Rota para definir o ângulo do Motor 1
  server.on("/setAngle1", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      int angle = request->getParam("value")->value().toInt();
      moveToAngle1(angle);
      request->send(200, "text/plain", "Ângulo do Motor 1 definido");
      Serial.print("Motor 1 movido para ângulo: ");
      Serial.println(angle);
    } else {
      request->send(400, "text/plain", "Parâmetro 'value' ausente");
    }
  });

  // Rota para definir o ângulo do Motor 2
  server.on("/setAngle2", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      int angle = request->getParam("value")->value().toInt();
      moveToAngle2(angle);
      request->send(200, "text/plain", "Ângulo do Motor 2 definido");
      Serial.print("Motor 2 movido para ângulo: ");
      Serial.println(angle);
    } else {
      request->send(400, "text/plain", "Parâmetro 'value' ausente");
    }
  });

  // Rota para redefinir a posição do Motor 1
  server.on("/resetPosition1", HTTP_GET, [](AsyncWebServerRequest *request){
    resetPosition1();
    request->send(200, "text/plain", "Posição do Motor 1 redefinida para zero");
    Serial.println("Posição do Motor 1 redefinida para zero");
  });

  // Rota para redefinir a posição do Motor 2
  server.on("/resetPosition2", HTTP_GET, [](AsyncWebServerRequest *request){
    resetPosition2();
    request->send(200, "text/plain", "Posição do Motor 2 redefinida para zero");
    Serial.println("Posição do Motor 2 redefinida para zero");
  });

  // Redireciona qualquer outra rota para a página raiz
  server.onNotFound([](AsyncWebServerRequest *request){
    request->redirect("/");
  });

  // Inicia o servidor
  server.begin();
  Serial.println("Servidor web iniciado");
}

void setup() {
  Serial.begin(115200);

  // Configura os pinos dos motores
  pinMode(PUL1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(PUL2, OUTPUT);
  pinMode(DIR2, OUTPUT);

  // Configura o ESP32 como ponto de acesso
  WiFi.softAP(ssid, password);
  Serial.println("Wi-Fi iniciado");

  // Exibe o IP do ponto de acesso
  Serial.println(WiFi.softAPIP());

  // Configura o DNS para redirecionar todas as requisições para o IP do ESP32
  dnsServer.start(53, "*", WiFi.softAPIP());

  // Configura as rotas do servidor web
  webServerSetup();

  Serial.println("Setup completo");
}

void loop() {
  dnsServer.processNextRequest(); // Mantém o DNS ativo para o portal cativo
}
