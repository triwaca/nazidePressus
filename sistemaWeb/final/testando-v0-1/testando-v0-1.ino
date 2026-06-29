
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

//pode apagar ===============
//// Variável global para o nível de pressão atual
//int nivelPressaoAtualAng = 0; 
//
//// Função para alterar o nível de pressão
//void nivelPressao(int nivel) {
//  if (nivel < 0 || nivel > 5) {
//    // Serial.println("Erro: Nível de pressão fora do intervalo permitido (0 a 5).");
//    return;
//  }
//  
//  // Valores dos angulos correspondentes aos níveis de pressão
//  const int valoresPressao[] = {50, 11, 294, 182, 108, 98};
//  nivelPressaoAtualAng = valoresPressao[nivel];
//  // Serial.print("Nível de pressão alterado para: ");
//  // Serial.println(nivelPressaoAtualAng);
//}
//==========================

 

// Função para alterar o nível de pressão
int nivelPressao(int nivel) {
  if (nivel < 0 || nivel > 5) {
    // Serial.println("Erro: Nível de pressão fora do intervalo permitido (0 a 5).");
    return -1;
  }
  
  // Valores dos angulos correspondentes aos níveis de pressão
  const int valoresPressao[] = {50, 11, 294, 182, 108, 98};
  return valoresPressao[nivel];
}

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

  currentPositionX = 0;
  currentPositionY = 0;
//  Serial.println("Motores inicializados e zerados.");
}

void moveMotorsToAngle(int angle) {
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

//declaração dos modos ====


// Função endurance
void endurance(int valor, int nivel) {
//    Serial.printf("Executando repetition com Valor: %d segundos, Nivel: %d\n", valor, nivel);
    int angle = nivelPressao(nivel);
    moveMotorsToAngle(angle);

    // Mantém o motor no ângulo pelo tempo especificado
    for (int i = 0; i < valor; i++) {
//        Serial.printf("Motor no ângulo %d... (%d/%d segundos)\n", angle, i + 1, valor);
        delay(1000); // Espera 1 segundo
    }

//    move para o nivel 0
    moveMotorsToAngle(nivelPressao(0));
//    Serial.println("Repetition concluído.");
}

// Função repetition
void repetition(int valor, int nivel) { //@talvez mudar
    int angle = nivelPressao(nivel); // Calcula o ângulo baseado no nível
    for (int i = 0; i < valor; i++) {
//        Serial.printf("Ciclo %d/%d: Movendo para o ângulo %d por 1 segundo\n", i + 1, valor, angle);
        moveMotorsToAngle(angle);    // Move para o ângulo calculado
        delay(1000);                // Espera 1 segundo

        moveMotorsToAngle(0);
        delay(4000);                // Espera 4 segundos

    }

//    Serial.println("Repetition concluído.");
}


// Função fast
void fast(int valor, int nivel) {
    int angle = nivelPressao(nivel); // Calcula o ângulo baseado no nível
    for (int i = 0; i < valor; i++) {
//        Serial.printf("Ciclo %d/%d: Movendo para o ângulo %d por 1 segundo\n", i + 1, valor, angle);
        moveMotorsToAngle(angle);    // Move para o ângulo calculado
        delay(2000);                // Espera 1 segundo

        moveMotorsToAngle(0);
        delay(1000);                // Espera 4 segundos

    }

//    Serial.println("Fast concluído.");
}

// Função para lidar com os modos
void functionHandle(String modo, String valorStr, String nivelStr) {
    int valor = valorStr.toInt(); // Converte valor para int
    int nivel = nivelStr.toInt(); // Converte nível para int

//    Serial.printf("Modo: %s, Valor (int): %d, Nivel (int): %d\n", modo.c_str(), valor, nivel);

    if (modo == "2") {
        endurance(valor, nivel);
    } else if (modo == "3") {
        repetition(valor, nivel);
    } else if (modo == "4") {
        fast(valor, nivel);
    } else {
//        Serial.printf("Modo inválido: %s\n", modo.c_str());
    }
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
            <title>Card de Botões</title>
            <style>
                body {
                    display: flex;
                    flex-direction: column;
                    align-items: center;
                    height: 100vh;
                    margin: 0;
                    background-color: #f0f0f0;
                }
        
                .navbar {
                    width: 100%;
                    height: 50px;
                    background-color: #F19ED2;
                    color: white;
                    padding: 10px;
                    box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);
                    display: flex;
                    justify-content: space-between;
                    align-items: center;
                }
        
                .navbar button {
                    background: none;
                    border: none;
                    color: white;
                    cursor: pointer;
                    font-size: 16px;
                    padding: 5px 10px;
                }
        
                .navbar button::before {
                    content: "";
                    display: inline-block;
                    width: 0;
                    height: 0;
                    margin-right: 5px;
                    border-top: 5px solid transparent;
                    border-bottom: 5px solid transparent;
                    border-right: 10px solid white;
                }
        
                .card {
                    display: flex;
                    flex-direction: column;
                    align-items: center;
                    background-color: white;
                    border-radius: 10px;
                    box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);
                    margin-bottom: 10px;
                    width: 75%;
                    max-width: 600px;
        
                    padding-bottom: 1rem;
                }
        
                .sub-card {
                    display: flex;
                    flex-direction: column;
                    align-items: center;
                    background-color: #E8C5E5;
                    padding: 10px;
                    border-radius: 5px;
                    margin: 5px;
                    width: 80%;
                }
        
                .sub-card input {
                    width: 40px;
                    margin: 0 10px;
                }
        
                button {
                    margin: 10px;
                    padding: 10px 20px;
                    border: none;
                    border-radius: 5px;
                    background-color: #F19ED2;
                    color: white;
                    cursor: pointer;
                    width: 80%;
                    box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);
                }
        
                button:disabled {
                    background-color: #ccc;
                    cursor: not-allowed;
                }
        
                .hidden {
                    display: none;
                }
        
                .container {
                    display: flex;
                    flex-direction: column;
                    align-items: center;
                    width: 100%;
                    margin-top: 20px;
                }
                
                #backButton {
                    width: auto;
                }
        
                .butons-group {
                    padding: 20px;
                    display: flex;
                    flex-direction: column;
                    align-items: center;
                    width: 100%;
                }
                
                .cardLabel {
                    width: 100%;
                    background: aquamarine;
                    /* background: #E8C5E5; */
                    align-items: center;
                    text-align: center;
                    height: 3rem;
                    border-radius: 10px 10px 0px 0px;
                    display: flex;
                    justify-content: center;
                }
        
                input {
                    padding: 10px;
                    border: 2px solid #ddd;
                    border-radius: 10px;
                    outline: none;
                    transition: border-color 0.3s ease, box-shadow 0.3s ease;
                }
        
                input:focus {
                    border-color: #4a90e2;
                    box-shadow: 0 0 10px rgba(74, 144, 226, 0.5);
                }
        
                #thirdCardLabel, .labelCard{
                    margin-top: 2rem;
                    padding: 10px;
                    margin-bottom: 1rem;
                    text-align: center;
                }
        
                #thirdCardTitle{
                    margin-top: 1rem;
                    padding: 10px;
                    font-size: 22px;
                }
                
        
                
            </style>
        </head>
        <body>
            <div class="navbar">
                <button id="backButton" class="hidden" onclick="goToInitialCard()"></button>
            </div>
            <div class="container">
                <div id="initialCard" class="card">
                    <label class="cardLabel">Nazide Pressus</label>
                    <div class="butons-group">
                        <button id="btn1" onclick="showSecondCard()">Força</button>
                        <button id="btn2" onclick="showThirdCard(2)" disabled>Resistência</button>
                        <button id="btn3" onclick="showThirdCard(3)" disabled>Repetições</button>
                        <button id="btn4" onclick="showThirdCard(4)" disabled>Contrações Rápidas</button>
                    </div>
                </div>
                <div id="secondCard" class="card hidden">
                    <label class="labelCard">Valores de 0 a 5 pela escala de oxford</label>
                    <button onclick="handleNumberClick(0)">Nivel 0 (ausente)</button>
                    <button onclick="handleNumberClick(1)">Nivel 1 (muito Fraco)</button>
                    <button onclick="handleNumberClick(2)">Nivel 2 (Fraco)</button>
                    <button onclick="handleNumberClick(3)">Nivel 3 (Moderado)</button>
                    <button onclick="handleNumberClick(4)">Nivel 4 (Bom)</button>
                    <button onclick="handleNumberClick(5)">Nivel 5 (Forte)</button>
                </div>
                <!-- <div id="thirdCard" class="card hidden">
                    <label id="thirdCardTitle">Card Externo</label>
                    <label id="thirdCardLabel">Card Externo</label>
                    <div class="sub-card">
                        <div>
                            <label>Nivel 1</label>
                            <input type="number" min="1" max="10" oninput="checkInputValue(this)">
                            
                        </div>
                        <button>Iniciar</button>
                    </div>
                    <div class="sub-card">
                        <div>
                            <label>Nivel 2</label>
                            <input type="number" min="1" max="10" oninput="checkInputValue(this)">
                            
                        </div>
                        <button>Iniciar</button>
                    </div>
                    <div class="sub-card">
                        <div>
                            <label>Nivel 3</label>
                            <input type="number" min="1" max="10" oninput="checkInputValue(this)">
                            
                        </div>
                        <button>Iniciar</button>
                    </div>
                    <div class="sub-card">
                        <div>
                            <label>Nivel 4</label>
                            <input type="number" min="1" max="10" oninput="checkInputValue(this)">
                            
                        </div>
                        <button>Iniciar</button>
                    </div>
                    <div class="sub-card">
                        <div>
                            <label>Nivel 5</label>
                            <input type="number" min="1" max="10" oninput="checkInputValue(this)">
                          
                        </div>
                        <button>Iniciar</button>
                    </div>
                </div> -->
                <div id="thirdCard" class="card hidden">
                    <label id="thirdCardTitle">Card Externo</label>
                    <label id="thirdCardLabel">Card Externo</label>
                    <div class="sub-card" id="subCard1">
                        <div>
                            <label>Nivel 1</label>
                            <input type="number" min="1" max="10" oninput="checkInputValue(this)">
                        </div>
                        <button onclick="onHandleServer(1)">Iniciar</button>
                    </div>
                    <div class="sub-card" id="subCard2">
                        <div>
                            <label>Nivel 2</label>
                            <input type="number" min="1" max="10" oninput="checkInputValue(this)">
                        </div>
                        <button onclick="onHandleServer(2)">Iniciar</button>
                    </div>
                    <div class="sub-card" id="subCard3">
                        <div>
                            <label>Nivel 3</label>
                            <input type="number" min="1" max="10" oninput="checkInputValue(this)">
                        </div>
                        <button onclick="onHandleServer(3)">Iniciar</button>
                    </div>
                    <div class="sub-card" id="subCard4">
                        <div>
                            <label>Nivel 4</label>
                            <input type="number" min="1" max="10" oninput="checkInputValue(this)">
                        </div>
                        <button onclick="onHandleServer(4)">Iniciar</button>
                    </div>
                    <div class="sub-card" id="subCard5">
                        <div>
                            <label>Nivel 5</label>
                            <input type="number" min="1" max="10" oninput="checkInputValue(this)">
                        </div>
                        <button onclick="onHandleServer(5)">Iniciar</button>
                    </div>
                </div>
            </div>
        
            <script>
                let estadoAtual = null;
        
                function showThirdCard(btnNumber) {
                    
                    document.getElementById('initialCard').classList.add('hidden');
                    document.getElementById('thirdCard').classList.remove('hidden');
                    document.getElementById('backButton').classList.remove('hidden');
        
                    estadoAtual = btnNumber; // alterar o estado
                    const title = document.getElementById('thirdCardTitle');
                    const label = document.getElementById('thirdCardLabel');
        
                    if(estadoAtual == 2){//resistência
                        title.innerText = `Resistência`;
                        label.innerText = `Tempo mantido de contração máxima sem perder a força`;
                    }
                    if(estadoAtual == 3){//repetições
                        title.innerText = `Repetições`;
                        label.innerText = `Número de repetições possíveis com descanso de 4 segundos`;
                    }
                    if(estadoAtual == 4){//contrações rápidas
                        title.innerText = `Contrações Rápidas`;
                        label.innerText = `Número de contrações rápidas após 1 minuto de descanso`;
                    }
        
        //label.innerText = `Card Externo - Botão ${btnNumber} selecionado`;// alterar o texto do terceiro card
                }
        
                function showSecondCard() {
                    document.getElementById('initialCard').classList.add('hidden');
                    document.getElementById('secondCard').classList.remove('hidden');
                    document.getElementById('backButton').classList.remove('hidden');
                }
        
                function handleNumberClick(number) {
                    document.getElementById('secondCard').classList.add('hidden');
                    if (number === 0) {
                        document.getElementById('btn2').disabled = true;
                        document.getElementById('btn3').disabled = true;
                        document.getElementById('btn4').disabled = true;
                        goToInitialCard();
                    } else {
                        document.getElementById('secondCard').classList.add('hidden');
                        document.getElementById('initialCard').classList.remove('hidden');
                        document.getElementById('btn2').disabled = false;
                        document.getElementById('btn3').disabled = false;
                        document.getElementById('btn4').disabled = false;
                        document.getElementById('backButton').classList.add('hidden');
                    }
                }
        
                function goToInitialCard() {
                    document.getElementById('initialCard').classList.remove('hidden');
                    document.getElementById('secondCard').classList.add('hidden');
                    document.getElementById('thirdCard').classList.add('hidden');
                    document.getElementById('backButton').classList.add('hidden');
                }
        
                function checkInputValue(input) {
                    if (input.value > 10) {
                        input.value = 10;
                    } else if (input.value < 1) {
                        input.value = 1;
                    }
                }
                
        
                function onHandleServer(buttonPosition) {
                    // Obtém o valor do input associado ao botão clicado
                    const subCard = document.getElementById(`subCard${buttonPosition}`);
                    const inputValue = subCard.querySelector('input').value;
        
                    // Verifica se o valor é válido
                    if (!inputValue || inputValue < 1 || inputValue > 10) {
                        alert('Por favor, insira um valor válido entre 1 e 10.');
                        return;
                    }
        
                    // Cria a URL para enviar os dados ao servidor
                    const url = `/handle?modo=${estadoAtual}&valor=${inputValue}&nivel=${buttonPosition}`;
        
                    // // Envia os dados via requisição GET
                    fetch(url)
                        .then(response => {
                           
                        })
                        .catch(error => {
                            console.error('Erro ao enviar os dados:', error);
//                            alert('Erro ao enviar os dados.');
                        });
        
                    // console.log(url);
                }
            </script>
        </body>
        </html>
    )rawliteral");
  });

  //rotas antigas === pode apagar !
  server.on("/setAngle", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("value")) {
      int angle = request->getParam("value")->value().toInt();
      moveMotorsToAngle(angle);
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

  //novo

  server.on("/handle", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("modo") && request->hasParam("valor") && request->hasParam("nivel")) {
        String modo = request->getParam("modo")->value();
        String valor = request->getParam("valor")->value();
        String nivel = request->getParam("nivel")->value();

//        Serial.printf("Modo: %s, Valor: %s, Nivel: %s\n", modo.c_str(), valor.c_str(), nivel.c_str());
        request->send(200, "text/plain", "Dados recebidos com sucesso");

        // Chama a função para tratar os modos
        functionHandle(modo, valor, nivel);

    } else {
        request->send(400, "text/plain", "Parâmetros faltando");
    }
  });


  server.onNotFound([](AsyncWebServerRequest *request) {
    request->redirect("/");
  });

  server.begin();
}

void setup() {
//  Serial.begin(115200);

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
