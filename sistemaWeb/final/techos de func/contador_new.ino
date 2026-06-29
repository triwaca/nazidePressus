unsigned long timer1Start, timer2Start;
int valor, angle;

void endurance(int nivel) {
    static bool firstTime = true; // Variável estática para verificar a primeira chamada

    angle = nivelPressao(nivel);

    if (firstTime) {
        moveMotorsToAngle(angle);
        timer1Start = millis(); // Inicia o primeiro temporizador
        firstTime = false;
    } else {
        unsigned long currentTime = millis();

        // Verifica se o primeiro temporizador expirou
        if (currentTime - timer1Start >= primeiroTempo) {
            moveMotorsToAngle(0);
            timer2Start = currentTime; // Inicia o segundo temporizador
        }

        // Verifica se o segundo temporizador expirou
        if (currentTime - timer2Start >= segundoTempo) {
            valor--;
            if (valor > 0) {
                moveMotorsToAngle(angle);
                timer1Start = currentTime; // Reinicia o primeiro temporizador
            }
        }
    }
}