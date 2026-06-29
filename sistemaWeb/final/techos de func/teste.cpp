
//pode apagar esse codigo =========
// Variáveis globais
int modo = 0; // Controla qual modo será executado na loop
int valorGlobal = 0; // Valor compartilhado para diferentes modos
int nivelGlobal = 0; // Nível compartilhado para diferentes modos

// Função endurance com configurações iniciais
void enduranceSetup(int valor, int nivel) {
    // Configurações iniciais
    valorGlobal = valor;
    nivelGlobal = nivel;
    modo = 2; // Define o modo endurance para execução
    Serial.printf("Configuração inicial para endurance: Valor = %d, Nível = %d\n", valor, nivel);
}

// Função endurance original
void endurance(int valor, int nivel) {
    static unsigned long timerStart = 0;
    static int remainingCycles = valor; // Número de ciclos restantes
    static int angle = 0;
    static bool initialized = false;

    unsigned long currentTime = millis();

    if (!initialized) {
        angle = nivelPressao(nivel);
        moveMotorsToAngle(angle);
        timerStart = currentTime;
        initialized = true;
    } else {
        if (currentTime - timerStart >= 1000) { // Mantém o motor ativo por 1 segundo
            moveMotorsToAngle(0);
            timerStart = currentTime;
            remainingCycles--;

            if (remainingCycles > 0) {
                moveMotorsToAngle(angle);
            } else {
                modo = 0; // Finaliza o modo endurance
                initialized = false; // Reinicia o estado
                Serial.println("Endurance concluído.");
            }
        }
    }
}

// Função repetition (exemplo para outros modos)
void repetitionSetup(int valor, int nivel) {
    valorGlobal = valor;
    nivelGlobal = nivel;
    modo = 3; // Define o modo repetition para execução
    Serial.printf("Configuração inicial para repetition: Valor = %d, Nível = %d\n", valor, nivel);
}

void repetition(int valor, int nivel) {
    // Similar à implementação original, adaptada para uso em modo global.
}

// Função fast (exemplo para outros modos)
void fastSetup(int valor, int nivel) {
    valorGlobal = valor;
    nivelGlobal = nivel;
    modo = 4; // Define o modo fast para execução
    Serial.printf("Configuração inicial para fast: Valor = %d, Nível = %d\n", valor, nivel);
}

void fast(int valor, int nivel) {
    // Similar à implementação original, adaptada para uso em modo global.
}

// Função para lidar com os modos
void functionHandle(String modoStr, String valorStr, String nivelStr) {
    int valor = valorStr.toInt();
    int nivel = nivelStr.toInt();

    if (modoStr == "2") {
        enduranceSetup(valor, nivel);
    } else if (modoStr == "3") {
        repetitionSetup(valor, nivel);
    } else if (modoStr == "4") {
        fastSetup(valor, nivel);
    } else {
        Serial.printf("Modo inválido: %s\n", modoStr.c_str());
    }
}

// Loop principal com switch-case
void loop() {
    switch (modo) {
        case 2:
            endurance(valorGlobal, nivelGlobal);
            break;
        case 3:
            repetition(valorGlobal, nivelGlobal);
            break;
        case 4:
            fast(valorGlobal, nivelGlobal);
            break;
        default:
            // Modo inativo
            break;
    }
}
