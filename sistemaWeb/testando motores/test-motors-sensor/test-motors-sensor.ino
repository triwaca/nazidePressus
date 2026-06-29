#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>

// Configuração dos pinos para os dois motores
int PUL1 = 33; // Pino de Pulso do Motor 1 (Pulse +)
int DIR1 = 25; // Pino de Direção do Motor 1 (Dir +)
int PUL2 = 32; // Pino de Pulso do Motor 2 (Pulse +)
int DIR2 = 27; // Pino de Direção do Motor 2 (Dir +)

// Configuração dos pinos para os sensores infravermelhos
int sensor1 = 18; // Sensor do Motor Esquerdo
int sensor2 = 19; // Sensor do Motor Direito

// Configuração da rede Wi-Fi
const char* ssid = "ESP32_Motor_Controller"; // Nome da rede Wi-Fi
const char* password = "";                   // Senha (em branco)

// Servidor web e DNS
AsyncWebServer server(80);
DNSServer dnsServer;

int stepsPerRevolution = 6400; // Passos para uma revolução completa
int currentPosition1 = 0; // Posição atual do Motor 1 em passos
int currentPosition2 = 0; // Posição atual do Motor 2 em passos

void initializeMotors() {
  bool sensor1Active = false;
  bool sensor2Active = false;

  while (!sensor1Active || !sensor2Active) {
    if (!sensor1Active) {
      digitalWrite(DIR1, HIGH); // Rotação no sentido desejado
      digitalWrite(PUL1, HIGH);
      delayMicroseconds(50);
      digitalWrite(PUL1, LOW);
      delayMicroseconds(50);
      sensor1Active = digitalRead(sensor1) == LOW; // Estado invertido
    }
    if (!sensor2Active) {
      digitalWrite(DIR2, LOW); // Rotação no sentido oposto
      digitalWrite(PUL2, HIGH);
      delayMicroseconds(50);
      digitalWrite(PUL2, LOW);
      delayMicroseconds(50);
      sensor2Active = digitalRead(sensor2) == LOW; // Estado invertido
    }
  }

  // Zera as posições ao terminar
  currentPosition1 = 0;
  currentPosition2 = 0;
  Serial.println("Motores inicializados e zerados.");
}


// Função para mover ambos os motores com um slider único
void moveMotorsWithSlider(int angle) {
  int targetSteps = map(angle, 0, 360, 0, stepsPerRevolution);

  int stepsToMove1 = targetSteps - currentPosition1;
  int stepsToMove2 = targetSteps - currentPosition2;

  digitalWrite(DIR1, stepsToMove1 >= 0 ? LOW : HIGH); // Define direção do Motor 1
//  digitalWrite(DIR2, stepsToMove2 >= 0 ? HIGH : LOW); // Define direção do Motor 2 (oposto)
digitalWrite(DIR2, stepsToMove2 >= 0 ? LOW :HIGH); // Define direção do Motor 2 (oposto)

  stepsToMove1 = abs(stepsToMove1);
  stepsToMove2 = abs(stepsToMove2);

  int maxSteps = max(stepsToMove1, stepsToMove2);

  for (int i = 0; i < maxSteps; i++) {
    if (i < stepsToMove1) {
      digitalWrite(PUL1, HIGH);
    }
    if (i < stepsToMove2) {
      digitalWrite(PUL2, HIGH);
    }
    delayMicroseconds(50);
    digitalWrite(PUL1, LOW);
    digitalWrite(PUL2, LOW);
    delayMicroseconds(50);
  }

  currentPosition1 = targetSteps;
  currentPosition2 = targetSteps;
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
              body { font-family: Arial, sans-serif; text-align: center; margin-top: 50px; }
              .slider { width: 80%; }
              .container { max-width: 600px; margin: auto; }
          </style>
      </head>
      <body>
          <div class="container">
              <h2>Controle de Motores</h2>
              <label for="angle">Ângulo (0-360): <span id="angleValue">0</span>°</label>
              <input type="range" id="angle" min="0" max="360" value="0" class="slider" oninput="updateAngle(this.value)">
              <button onclick="setAngle()">Definir Ângulo</button>
          </div>
          <script>
              function updateAngle(value) {
                  document.getElementById('angleValue').innerText = value;
              }
              function setAngle() {
                  const angle = document.getElementById('angle').value;
                  fetch(`/setAngle?value=${angle}`);
              }
          </script>
      </body>
      </html>
    )rawliteral");
    Serial.println("Página de controle dos motores carregada");
  });

  // Rota para definir o ângulo com o slider
  server.on("/setAngle", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("value")) {
      int angle = request->getParam("value")->value().toInt();
      moveMotorsWithSlider(angle);
      request->send(200, "text/plain", "Ângulo definido");
      Serial.print("Motores movidos para ângulo: ");
      Serial.println(angle);
    } else {
      request->send(400, "text/plain", "Parâmetro 'value' ausente");
    }
  });

  // Redireciona qualquer outra rota para a página raiz
  server.onNotFound([](AsyncWebServerRequest *request) {
    request->redirect("/");
  });

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

  // Configura os pinos dos sensores
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);

  // Configura o ESP32 como ponto de acesso
  WiFi.softAP(ssid, password);
  Serial.println("Wi-Fi iniciado");

  // Exibe o IP do ponto de acesso
  Serial.println(WiFi.softAPIP());

  // Configura o DNS para redirecionar todas as requisições para o IP do ESP32
  dnsServer.start(53, "*", WiFi.softAPIP());

  // Inicializa os motores e o servidor web
  initializeMotors();
  webServerSetup();
}

void loop() {
  dnsServer.processNextRequest();
}
