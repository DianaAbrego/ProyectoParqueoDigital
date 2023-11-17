#define A 32
#define B 33
#define C 25
#define D 26
#define E 27
#define F 14
#define G 12

//VARIABLES GLOBALES------------------------------------------------------
int PINESdisp[7]={A,B,C,D,E,F,G};
int DA[10]={0,1,0,0,1,0,0,0,0,0};
int DB[10]={0,0,0,0,0,1,1,0,0,0};
int DC[10]={0,0,1,0,0,0,0,0,0,0};
int DD[10]={0,1,0,0,1,0,0,1,0,0};
int DE[10]={0,1,0,1,1,1,0,1,0,1};
int DF[10]={0,1,1,1,0,0,0,1,0,0};
int DG[10]={1,1,0,0,0,0,0,1,0,0};

unsigned char Tiva1;
unsigned char Tiva2;
unsigned char Ocupancy = (Tiva1 << 4) | Tiva2; 
int p1status = (Ocupancy & (1 << 0)) != 0 ? 1 : 0;
int p2status = (Ocupancy & (1 << 1)) != 0 ? 1 : 0;
int p3status = (Ocupancy & (1 << 2)) != 0 ? 1 : 0;
int p4status = (Ocupancy & (1 << 3)) != 0 ? 1 : 0;
int p5status = (Ocupancy & (1 << 4)) != 0 ? 1 : 0;
int p6status = (Ocupancy & (1 << 5)) != 0 ? 1 : 0;
int p7status = (Ocupancy & (1 << 6)) != 0 ? 1 : 0;
int p8status = (Ocupancy & (1 << 7)) != 0 ? 1 : 0;


//------------------------------------------------------------------------
//Prototipos de funciones-------------------------------------------------
void Display7S(int N);
int contarBitsEncendidos(unsigned char valor);
//------------------------------------------------------------------------

void setup() {
  //Serial1.begin(115200,SERIAL_8N1,40,41);
  //Serial2.begin(115200, SERIAL_8N1,25,27);
  // Inicializar el puerto serial para la comunicación con la Tiva C
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 19, 21);  //Puerto TIVA1
  Serial2.begin(115200, SERIAL_8N1, 5, 18);   //Puerto TIVA2
  while (!Serial1);     //Esperar a que el puerto serial 1 esté listo
  while (!Serial2);     //Esperar a que el puerto serial 2 esté listo

  for(int i=0; i<=7; i++){    //definicion pines display
    pinMode(PINESdisp[i],OUTPUT);
  }                           //fin for

  for(int i=0; i<=7; i++){    //escritura valor inicial en display
    digitalWrite(PINESdisp[i],LOW);
  }                           //fin for
  delay(2000);
}

void loop() {
  if (Serial1.available() > 0) { //verificacion de recepcion
    Tiva1 = Serial1.read();   //Guardo UART1 en Tiva1
    Serial.print("Dato recibido: ");  //verificacion de recepcion de datos
    Serial.println(Tiva1);     //verficiacion 2
  }
  if (Serial2.available() > 0) {
    Tiva2 = Serial2.read();
    Serial.print("Dato 2 recibido: ");
    Serial.println(Tiva2);
  }

 Ocupancy = (Tiva1 << 4) | Tiva2;
 
 Display7S(contarBitsEncendidos(Ocupancy));
  
}

//Funciones---------------------------------------------------------------
void Display7S(int N){
  digitalWrite(A,DA[N]);
  digitalWrite(B,DB[N]);
  digitalWrite(C,DC[N]);
  digitalWrite(D,DD[N]);
  digitalWrite(E,DE[N]);
  digitalWrite(F,DF[N]);
  digitalWrite(G,DG[N]);
}

int contarBitsEncendidos(unsigned char valor){
    int contador = 0;
    for (int i = 0; i < 8; ++i) {       // Iterar sobre cada bit de la variable
        if ((valor & (1 << i)) != 0) {  // Verificar si el bit actual está encendido (1)
            contador++;
        }
    }
    return contador;
}
//------------------------------------------------------------------------
