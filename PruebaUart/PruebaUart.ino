void setup() {
  // Inicializar el puerto serial para la comunicación con la Tiva C
  Serial.begin(115200);

  // Inicializar el puerto serial 1 para la comunicación con la Tiva C
  Serial2.begin(115200, SERIAL_8N1, 5, 18);

  // Esperar a que el puerto serial 1 esté listo
  while (!Serial2);

  // Otros inicializaciones si es necesario
}

void loop() {
  // Verificar si hay datos disponibles en el puerto serial 1
  if (Serial2.available() > 0) {
    // Leer el carácter recibido
    char receivedChar = Serial2.read();

    // Imprimir el carácter en el monitor serial
    Serial.print("Dato recibido: ");
    Serial.println(receivedChar);
  }

  // Tu código principal aquí
}
