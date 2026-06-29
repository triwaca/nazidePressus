
#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>

#include <Wire.h> //Biblioteca utilizada gerenciar a comunicação entre dispositicos através do protocolo I2C
#include <LiquidCrystal_I2C.h> //Biblioteca controlar display 16x2 através do I2C

// Configuração dos pinos para os motores
#define STEP_PIN_X 33
#define DIR_PIN_X 25
#define STEP_PIN_Y 32
#define DIR_PIN_Y 27

// Configuração dos pinos para os sensores
#define SENSOR1 18
#define SENSOR2 19


#define col  16 //Define o número de colunas do display utilizado
#define lin   2 //Define o número de linhas do display utilizado
// #define ende  0x3F //Define o endereço do display
#define ende 0x27

LiquidCrystal_I2C lcd(ende,16,2); //Cria o objeto lcd passando como parâmetros o endereço, o nº de colunas e o nº de linhas

//  lcd.setCursor(0, 0); //Coloca o cursor do display na coluna 1 e linha 1
//  lcd.print("Calibrando"); //Exibe a mensagem na primeira linha do display
//  lcd.setCursor(0, 1); //Coloca o cursor do display na coluna 1 e linha 2
//  lcd.print("TUTORIAL DISPLAY");  //Exibe a mensagem na segunda linha do display


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
  lcd.setCursor(0, 0); //Coloca o cursor do display na coluna 1 e linha 1
  lcd.print("Calibrando"); //Exibe a mensagem na primeira linha do display

  for(int i =0;i<6;i++){
    calibraMotores(STEP_PIN_X, DIR_PIN_X, SENSOR1, -1, "X");
    calibraMotores(STEP_PIN_Y, DIR_PIN_Y, SENSOR2, -1, "Y");
    delay(1000);
  }

  currentPositionX = 0;
  currentPositionY = 0;
//  Serial.println("Motores inicializados e zerados.");
  lcd.setCursor(0, 0); //Coloca o cursor do display na coluna 1 e linha 1
  lcd.print("Motores inicializados"); //Exibe a mensagem na primeira linha do display
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

// ENDURANCE ==================================================================================================================================

// Endurance corigido  ========================================================================================================================

unsigned long timerStartEndurance = 0;
int angle = 0;        // Ângulo calculado
int modoGlobal = 0;   // Modo atual
bool enduranceInProgress = false; // Estado do modo endurance

int temEndurance;

// Função de configuração inicial para endurance
void endurance(int tempo, int nivelInicial) {
    angle = nivelPressao(nivelInicial); // Calcula o ângulo baseado no nível
    timerStartEndurance = millis();             // Inicia o temporizador
  
    enduranceInProgress = true;        // Marca que o modo está ativo
    moveMotorsToAngle(angle);          // Move para o ângulo inicial
    temEndurance = tempo*1000;
    modoGlobal = 2;                    // Define o modo como endurance
    
//    Serial.printf("Configuração inicial: Modo Endurance, Tempo: %d ms, Nível: %d\n", tempo, nivelInicial);

  lcd.clear(); //Limpa a tela do display
  lcd.setCursor(0, 0); //Coloca o cursor do display na coluna 1 e linha 1
  lcd.print("ENDURANCE: " + String(nivelInicial) + " ");
}

// Função principal de controle do endurance
void endurance() {
    if (!enduranceInProgress) return; // Retorna se o modo não está ativo

    unsigned long currentTime = millis();

    // Verifica se o temporizador expirou
    if (currentTime - timerStartEndurance >= temEndurance) {
        moveMotorsToAngle(0); // Retorna ao nível 0
        enduranceInProgress = false; // Finaliza o modo
        modoGlobal = 0;              // Reseta o modo
//        Serial.println("Endurance concluído.");
    }
}



// REPETITION ==================================================================================================================================

// Variáveis globais para repetition
unsigned long repetitionTimerStart = 0;
int repetitionCount = 0;
int repetitionAngle = 0;
bool repetitionInProgress = false;

int repetitionContrctTime = 10000;
int repetitionRealeaseTime = 4000;

// Função de configuração inicial para repetition
void repetition(int valorInicial, int nivelInicial) {
    repetitionCount = valorInicial; // Número de repetições
    repetitionAngle = nivelPressao(nivelInicial); // Calcula o ângulo baseado no nível
    repetitionInProgress = true; // Marca que o modo está ativo
    modoGlobal = 3; // Define o modo como repetition
//    Serial.printf("Configuração inicial: Modo Repetition, Valor: %d, Nível: %d\n", valorInicial, nivelInicial);

  lcd.clear(); //Limpa a tela do display
  lcd.setCursor(0, 0); //Coloca o cursor do display na coluna 1 e linha 1
  lcd.print("REPTITION: " + String(nivelInicial) + " ");
}

// Função principal para executar o modo repetition
void repetition() {
    if (!repetitionInProgress) return; // Retorna se o modo não está ativo

    unsigned long currentTime = millis();

    static enum { MOVING_TO_ANGLE, WAITING, RETURNING_TO_ZERO } repetitionState = MOVING_TO_ANGLE;

    switch (repetitionState) {
        case MOVING_TO_ANGLE:
            moveMotorsToAngle(repetitionAngle); // Move para o ângulo especificado
            repetitionTimerStart = currentTime; // Inicia o temporizador
            repetitionState = WAITING;
            break;

        case WAITING:
            if (currentTime - repetitionTimerStart >= repetitionContrctTime) { // Espera 1 segundo
                moveMotorsToAngle(0); // Retorna ao nível 0
                repetitionTimerStart = currentTime; // Inicia o temporizador
                repetitionState = RETURNING_TO_ZERO;
            }
            break;

        case RETURNING_TO_ZERO:
            if (currentTime - repetitionTimerStart >= repetitionRealeaseTime) { // Espera 4 segundos
                repetitionCount--; // Reduz o número de repetições restantes
                if (repetitionCount > 0) {
                    repetitionState = MOVING_TO_ANGLE; // Reinicia o ciclo
                } else {
                    repetitionInProgress = false; // Finaliza o modo
                    modoGlobal = 0; // Reseta o modo
//                    Serial.println("Repetition concluído.");
                }
            }
            break;
    }
}


// FAST ==================================================================================================================================



// Variáveis globais para fast
unsigned long fastTimerStart = 0;
int fastCount = 0;
int fastAngle = 0;
bool fastInProgress = false;

int fastContractTime = 2000;
int fastReleaseTime = 100;

// Função de configuração inicial para fast
void fast(int valorInicial, int nivelInicial) {
    fastCount = valorInicial;             // Número de ciclos rápidos
    fastAngle = nivelPressao(nivelInicial); // Calcula o ângulo baseado no nível
    fastInProgress = true;               // Marca que o modo está ativo
    modoGlobal = 4;                      // Define o modo como fast
//    Serial.printf("Configuração inicial: Modo Fast, Valor: %d, Nível: %d\n", valorInicial, nivelInicial);

  lcd.clear(); //Limpa a tela do display
  lcd.setCursor(0, 0); //Coloca o cursor do display na coluna 1 e linha 1
  lcd.print("FAST: " + String(nivelInicial) + " ");
}

// Função principal para executar o modo fast
void fast() {
    if (!fastInProgress) return; // Retorna se o modo não está ativo

    unsigned long currentTime = millis();

    static enum { MOVING_TO_ANGLE_FAST, WAITING_FAST, RETURNING_FAST } fastState = MOVING_TO_ANGLE_FAST;

    switch (fastState) {
        case MOVING_TO_ANGLE_FAST:
            moveMotorsToAngle(fastAngle); // Move para o ângulo especificado
            fastTimerStart = currentTime; // Inicia o temporizador
            fastState = WAITING_FAST;
            break;

        case WAITING_FAST:
            if (currentTime - fastTimerStart >= fastContractTime) { // Espera 0,5 segundo (tempo reduzido para fast)
                moveMotorsToAngle(0); // Retorna ao nível 0
                fastTimerStart = currentTime; // Inicia o temporizador
                fastState = RETURNING_FAST;
            }
            break;

        case RETURNING_FAST:
            if (currentTime - fastTimerStart >= fastReleaseTime) { // Espera 0,5 segundo antes de reiniciar
                fastCount--; // Reduz o número de ciclos restantes
                if (fastCount > 0) {
                    fastState = MOVING_TO_ANGLE_FAST; // Reinicia o ciclo
                } else {
                    fastInProgress = false; // Finaliza o modo
                    modoGlobal = 0;         // Reseta o modo
//                    Serial.println("Fast concluído.");
                }
            }
            break;
    }
}



void functionHandle(String modo, String valorStr, String nivelStr) {
    int valor = valorStr.toInt(); // Converte valor para int
    int nivel = nivelStr.toInt(); // Converte nível para int

    if (modo == "2") {
        endurance(valor, nivel); // Configura o modo endurance
    } else if (modo == "3") {
        repetition(valor, nivel); // Configura o modo repetition
    } else if (modo == "4") {
        fast(valor, nivel); // Configura o modo fast
    } else {
        Serial.printf("Modo inválido: %s\n", modo.c_str());
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
        
        select {
        /* Estilos para o elemento select em si */
        width: 200px; /* Largura do dropdown */
        padding: 10px;
        border: 1px solid #ccc;
        border-radius: 5px;
        appearance: none; /* Remove a seta padrão em alguns navegadores */
        background-image: url(seta.png); /* Adiciona uma imagem de seta personalizada */
        background-repeat: no-repeat;
        background-position: right center;
        }

        option {
        /* Estilos para as opções dentro do dropdown */
        padding: 10px;
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
                <button id="btn2" onclick="showThirdCard(2)" >Resistência</button>
                <button id="btn3" onclick="showThirdCard(3)" >Repetições</button>
                <button id="btn4" onclick="showThirdCard(4)" >Contrações Rápidas</button>
            </div>
        </div>
        <div id="thirdCard" class="card hidden">
            <label id="thirdCardTitle">Card Externo</label>
            <label id="thirdCardLabel">Card Externo</label>
            <div class="sub-card" id="subCard1">
                <div>
                    <label>Nivel 1</label>
                    <select id="nivel1Select">
                        <option value="1">1</option>
                        <option value="2">2</option>
                        <option value="3">3</option>
                        <option value="4">4</option>
                        <option value="5">5</option>
                        <option value="6">6</option>
                        <option value="7">7</option>
                        <option value="8">8</option>
                        <option value="9">9</option>
                        <option value="10">10</option>
                    </select>
                </div>
                <button onclick="onHandleServer(1)">Iniciar</button>
            </div>
            <div class="sub-card" id="subCard2">
                <div>
                    <label>Nivel 2</label>
                    <select id="nivel2Select">
                        <option value="1">1</option>
                        <option value="2">2</option>
                        <option value="3">3</option>
                        <option value="4">4</option>
                        <option value="5">5</option>
                        <option value="6">6</option>
                        <option value="7">7</option>
                        <option value="8">8</option>
                        <option value="9">9</option>
                        <option value="10">10</option>
                    </select>
                </div>
                <button onclick="onHandleServer(2)">Iniciar</button>
            </div>
            <div class="sub-card" id="subCard3">
                <div>
                    <label>Nivel 3</label>
                    <select id="nivel3Select">
                        <option value="1">1</option>
                        <option value="2">2</option>
                        <option value="3">3</option>
                        <option value="4">4</option>
                        <option value="5">5</option>
                        <option value="6">6</option>
                        <option value="7">7</option>
                        <option value="8">8</option>
                        <option value="9">9</option>
                        <option value="10">10</option>
                    </select>
                </div>
                <button onclick="onHandleServer(3)">Iniciar</button>
            </div>
            <div class="sub-card" id="subCard4">
                <div>
                    <label>Nivel 4</label>
                    <select id="nivel4Select">
                        <option value="1">1</option>
                        <option value="2">2</option>
                        <option value="3">3</option>
                        <option value="4">4</option>
                        <option value="5">5</option>
                        <option value="6">6</option>
                        <option value="7">7</option>
                        <option value="8">8</option>
                        <option value="9">9</option>
                        <option value="10">10</option>
                    </select>
                </div>
                <button onclick="onHandleServer(4)">Iniciar</button>
            </div>
            <div class="sub-card" id="subCard5">
                <div>
                    <label>Nivel 5</label>
                    <select id="nivel5Select">
                        <option value="1">1</option>
                        <option value="2">2</option>
                        <option value="3">3</option>
                        <option value="4">4</option>
                        <option value="5">5</option>
                        <option value="6">6</option>
                        <option value="7">7</option>
                        <option value="8">8</option>
                        <option value="9">9</option>
                        <option value="10">10</option>
                    </select>
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
            // document.getElementById('secondCard').classList.add('hidden');s
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

            const selectId = `nivel${buttonPosition}Select`;
            const selectElement = document.getElementById(selectId);
            const inputValue = selectElement.value;

            // Cria a URL para enviar os dados ao servidor
            const url = `/handle?modo=${estadoAtual}&valor=${inputValue}&nivel=${buttonPosition}`;

            // // Envia os dados via requisição GET
            fetch(url)
                .then(response => {
                    if (response.ok) {
                        alert('Dados enviados com sucesso!');
                    } else {
                        alert('Erro ao enviar os dados.');
                    }
                })
                .catch(error => {
                    console.error('Erro ao enviar os dados:', error);
                    alert('Erro ao enviar os dados.');
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

  
  lcd.init(); //Inicializa a comunicação com o display já conectado
  lcd.clear(); //Limpa a tela do display
  lcd.backlight(); //Aciona a luz de fundo do display

  initializeMotors();
  
  webServerSetup();
}

void loop() {
  dnsServer.processNextRequest();

//  switch (modo) {
//    case 2:
//      endurance(valorGlobal, nivelGlobal);
//      break;
//    case 3:
//      repetition(valorGlobal, nivelGlobal);
//      break;
//    case 4:
//      fast(valorGlobal, nivelGlobal);
//      break;
//    default:
//      // Modo inativo
//      break;
//    }


    switch (modoGlobal) {
      case 2:
        endurance(); // Chama a função de execução do modo endurance
        break;

      case 3:
        repetition(); // Implementar de forma similar ao endurance
        break;

      case 4:
        fast(); // Implementar de forma similar ao endurance
        break;

      default:
        // Estado padrão, nenhum modo ativo
        break;
    }



}
