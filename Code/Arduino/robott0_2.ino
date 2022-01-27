#include <SoftwareSerial.h> // Libreria per comunicazione Seriale
#include <math.h>
#include <Servo.h> // Libreria per i motori servo

#define HEADER '|' // dichiarazione header per riconoscere inizio messaggio
#define Xc 'N' // dichiarazione di tag per riconoscere valore X di coordinate
#define Yc 'M' // dichiarazione di tag per riconoscere valore Y di coordinate
#define MESSAGE_BYTES 7 // Il numero di byte totali che vengono mandati dal PC via seriale
#define trigPin 38
#define echoPin 39

SoftwareSerial mySerial(50, 48); // dichiarare pin per comunicazione bluetooth RX e TX del Mega non funzionano, usare RX=50 e TX=48
Servo semeServo;
Servo aratroServo;

// Parametri
const int driveDistance = 200;   // in cm
const int directionDistance = 20; //distanza del movimento per calcolare la direzione

const int motorPower = 80;      // 0-255
const int motor_offset = 1;       // Differenza per la regolazione dei motori
const int wheel_d = 55;           // diametro ruote(mm)
const float wheel_c = PI * wheel_d; // Circonferenza ruote (mm)
const int countsPerRev = 75;
const float anglePerTick = 1.148;
const float movementCorrection = 1.0;
const float tickPerCm = 4.340579207; // numero di segnali degli speed encoder ogni cm
const double r = 11.5; // raggio della circonferenza di rotazione
const float turn180 = r * 2 * 2 * M_PI * 0.5;
const int positionLoopN = 80;

// Pins
const int enc_l_pin = 2;
const int enc_r_pin = 3;
const int pwma_pin = 13;
const int ain1_pin = 12;
const int ain2_pin = 11;
const int pwmb_pin = 8;
const int bin1_pin = 10;
const int bin2_pin = 9;

const int aratroPin = 5;
const int semePin = 6;

// Variabili globali
volatile unsigned long enc_l = 0; // valore speed encoder
volatile unsigned long enc_r = 0;
int valY; //variabile coordinata Y
int valX; // variabile coordinata X
char checkBounds = 'y';
int destArrayX[] = {0, 20}; // coordinate x dei punti di arrivo
int destArrayY[] = {100, 80}; // coordinate y dei punti di arrivo
int destN = 0; // destination array index
float distance = 0; // distanza da percorrere dai motori
float lastDistance = 20; // salva l'ultima distanza percorsa

  // calcAngolo()
double direzioneX = 0;
double direzioneY = 0;

double deltaX = 0;
double deltaY = 0;

double deltaDestX = 0;
double deltaDestY = 0;

double xT = 0;
double yT = 0;

double x0 = 0;
double y0 = 0;

double alpha = 0;
double cross = 0;
double angle = 0;
int segno = 1;
String senso = "left";
double dx, dy, dd, a, b, t, x1, y1, x2, y2, xSin, xDest, ySin, yDest;
double arco;

//posizioni - getDirection()
int posI[2]; //[X,Y]
int posF[2];

bool left = true;
bool lostRobotBool = false;

void setup() {

  // Debug
  Serial.begin(9600);

  // Impostare i pin
  pinMode(enc_l_pin, INPUT);
  pinMode(enc_r_pin, INPUT);
  pinMode(pwma_pin, OUTPUT);
  pinMode(ain1_pin, OUTPUT);
  pinMode(ain2_pin, OUTPUT);
  pinMode(pwmb_pin, OUTPUT);
  pinMode(bin1_pin, OUTPUT);
  pinMode(bin2_pin, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.begin(9600); // inizializzare comunicazione seriale con console
  mySerial.begin(9600); // inizializzare comunicazione seriale con modulo bluetooth (Processing su PC)
  Serial.println("You`re connected via Bluetooth"); // messaggio per indicare connessione a BT

  // Interrupts per i pin degli speed encoder
  attachInterrupt(digitalPinToInterrupt(enc_l_pin), countLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_r_pin), countRight, CHANGE);

  delay(1000);

  // Iniziare il programma
  go();

  // TEST per verificare visivamente a quanto ammonta l'errore durante la rotazione
  // turnRobot(turn180, motorPower, false);
}

void loop() {
  // Non fare nulla
}

// ***************************************
//           GO
//  Imposta la prima funzione da eseguire
// ***************************************
void go() {
  getDirection();
}

// ***************************************
//           GET DIRECTION
//  movimento per calcolare direzione
// ***************************************
void getDirection() {
  Serial.println("Getting direction");

  // Alzare l'aratro
  if (aratroServo.read() < 10) {
    aratroServo.attach(aratroPin);
    aratroServo.write(180);
    delay(2000);
    aratroServo.detach();
  }

  lastDistance = directionDistance;

  positionLoop(positionLoopN);

  // Salvare coordinate della posizione iniziale
  posI[0] = valX;
  posI[1] = valY;

  driveStraight(directionDistance, motorPower);

  positionLoop(positionLoopN);

  // Salvare coordinate della posizione finale
  posF[0] = valX;
  posF[1] = valY;

  Serial.println("initial: " + String(posI[0]) + ", " + String(posI[1]));
  Serial.println("final: " + String(posF[0]) + ", " + String(posF[1]));

  calcAngolo();
}

// ***************************************
//           CALC ANGOLO
//    calcola angolo di rotazione
// ***************************************
void calcAngolo() {

  positionLoop(positionLoopN);

  double direzioneY = (posF[1] - posI[1]);
  double direzioneX = (posF[0] - posI[0]);

  alpha = (M_PI / 2.0) - atan2(abs(direzioneY), abs(direzioneX));

  // robot direction
  double a1 = direzioneX;
  double a2 = direzioneY;
  // robot to destination
  double b1 = destArrayX[destN] - valX;
  double b2 = destArrayY[destN] - valY;

  // CROSS
  cross = b1 * a2 - a1 * b2;

  if (cross < 0.0f) {
    segno = -1;
  } else {
    segno = 1;
  }

  // calcolare coordinate (x0 e y0) del centro della circonferenza
  if (direzioneX >= 0) {
    y0 = valY - (segno * abs(r * sin(alpha)));
  } else {
    y0 = valY + (segno * abs(r * sin(alpha)));
  }

  if (direzioneY >= 0) {
    x0 = valX + (segno * abs(r * cos(alpha)));
  } else {
    x0 = valX - (segno * abs(r * cos(alpha)));
  }


  // ===============================
  //  calcolo dei punti di tangenza
  // ===============================

  dx = x0 - destArrayX[destN];
  dy = y0 - destArrayY[destN];
  dd = sqrt(dx * dx + dy * dy);
  // Quando dd < r a non dara` un risultato, quidi si calcola l'angolo con un'approssimazione
  a = asin(r / dd);
  b = atan2(dy, dx);
  t = b - a;
  x1 = r * sin(t) + x0;
  y1 = -r * cos(t) + y0;
  t = b + a;
  x2 = -r * sin(t) + x0;
  y2 = r * cos(t) + y0;

  // Controllare quali coordinate sono piu` grandi
  if (x1 < x2) {
    xSin = x1;
    ySin = y1;

    xDest = x2;
    yDest = y2;
  } else {
    xSin = x2;
    ySin = y2;

    xDest = x1;
    yDest = y1;
  }

  if (cross > 0) {
    if (destArrayY[destN] > y0) {
      xT = xSin;
      yT = ySin;
    } else {
      xT = xDest;
      yT = yDest;
    }
  } else {
    if (destArrayY[destN] > y0) {
      xT = xDest;
      yT = yDest;
    } else {
      xT = xSin;
      yT = ySin;
    }
  }

  // ===============================
  //  calcolo arco
  // ===============================
  if (dd >= 2 * r) {
    deltaX = valX - x0;
    deltaY = valY - y0;

    deltaDestX = xT - x0;
    deltaDestY = yT - y0;
  } else { // se la distanza é piu` piccola del raggio si fa un'approssimazione
    deltaX = direzioneX;
    deltaY = direzioneY;

    deltaDestX = destArrayX[destN] - x0;
    deltaDestY = destArrayY[destN] - y0;
  }
  // viene usata la formula per calcolare l'angolo acuto tra due vettori
  angle = acos((deltaX * deltaDestX + deltaY * deltaDestY) /
               (sqrt(deltaX * deltaX + deltaY * deltaY) *
                sqrt(deltaDestX * deltaDestX + deltaDestY * deltaDestY))) * 57.2958;

  // calcolo della distanza che dovra` percorrere il robot
  arco = 2 * 2 * M_PI * r * angle / 360;

  if (cross < 0) {
    senso = "left";
  } else {
    senso = "right";
  }

  turnRobot(arco , motorPower, true);  // ruotare robot
}

// ***************************************
//           TURN ROBOT
// ruotare il robot per dirigersi verso
// la destinazione
// ***************************************
void turnRobot(float dist, int power, bool calcdestination) {
  // Alzare l'aratro
  if (aratroServo.read() < 150) {
    aratroServo.attach(aratroPin);
    aratroServo.write(180);
    delay(2000);
    aratroServo.detach();
  }

  unsigned long num_ticks_l;
  unsigned long num_ticks_r;

  // impostare potenza iniziale dei motori
  int power_l = motorPower;
  int power_r = motorPower;

  // usate per stabilire in quale senso si deve ruotare il motore per la correzione
  unsigned long diff_l;
  unsigned long diff_r;

  // resettare contatori encoder
  enc_l = 0;
  enc_r = 0;

  // salvare contatore encoder precedente
  unsigned long enc_l_prev = enc_l;
  unsigned long enc_r_prev = enc_r;

  // calcolare di quanto devono girare i motori
  unsigned long target_count;
  if (dd >= r) {
    target_count = dist * tickPerCm * 2 * movementCorrection;
  } else {
    target_count = dist * tickPerCm * movementCorrection;
  }

  // ruotare finche` uno dei motori non compie la distanza target
  while ( (enc_l < target_count) && (enc_r < target_count) ) {

    // Salvare numero di "ticks"
    num_ticks_l = enc_l;
    num_ticks_r = enc_r;

    // Se dd >= r 
    if (dd >= r) {
      if (senso == "left") {
        drive(110, 0);
      } else if (senso == "right") {
        drive(0, 85);
      }
    } else {
      //se motore sinistro piu` veloce, rallentarlo e velocizzare il destro
      if ( diff_l < diff_r ) {
        if (left == true) {
          power_l += motor_offset;
          power_r -= motor_offset;

        } else {
          power_l -= motor_offset;
          power_r += motor_offset;
        }
      }

      // se motore destro piu` veloce, rallentarlo e velocizzare il sinistro
      if ( diff_l > diff_r ) {

        if (left == true)
          power_l += motor_offset;
        power_r -= motor_offset;
      }
      if (senso == "left") {
        drive(110, -85);
      } else if (senso == "right") {
        drive(-110, 85);
      }
    }

    // numero di ticks dall'ultima volta
    diff_l = num_ticks_l - enc_l_prev;
    diff_r = num_ticks_r - enc_r_prev;

    // salvare numero di ticks per prossima volta
    enc_l_prev = num_ticks_l;
    enc_r_prev = num_ticks_r;

    // breve pausa per lasciare tempo ai motori per reagire
    delay(20);
  }

  // Frenare
  brake();

  delay(1000);
  if (calcdestination) {
    calcDestination();
  }
}

// ***************************************
//           CALC DESTINATION
// calcola il modulo delvettore destinazione
// ***************************************
void calcDestination() {
  positionLoop(positionLoopN);

  float a = valX - destArrayX[destN]; //differenza x
  float b = valY - destArrayY[destN];  //differenza y

  distance = sqrt((a * a) + (b * b)); //modulo vettore (a;b) in ticks
  Serial.println("distance to dest.: " + String(distance));

  destinationMovement(distance, true);
}

// ***************************************
//        DESTINATION MOVEMENT
//    movimento fino a destinazione
// ***************************************
void destinationMovement(float dist, bool controlBool) {
  aratroServo.attach(aratroPin);
  aratroServo.write(90);
  delay(1000);
  aratroServo.detach();

  lastDistance = dist;

  if (controlBool) {
    positionLoop(positionLoopN);
  }

  posI[0] = valX; //get initial position
  posI[1] = valY;

  driveStraight(dist * 2, motorPower); // movimento in avanti per "distance" metri

  delay(1000);

  if (controlBool) {
    positionLoop(positionLoopN);
  }

  posF[0] = valX; //get final position
  posF[1] = valY;

  Serial.println("initial: " + String(posI[0]) + ", " + String(posI[1]));
  Serial.println("final: " + String(posF[0]) + ", " + String(posF[1]));

  if (controlBool) {
    control();
  }
}
// ***************************************
//               CONTROL
// controllare che le coordinate siano
// entro un margine di errore
// ***************************************
void control() {

  positionLoop(positionLoopN);

  float a = valX - destArrayX[destN]; //differenza x
  float b = valY - destArrayY[destN];  //differenza y

  float distance = sqrt((a * a) + (b * b));  //modulo vettore (a;b) in ticks
  if (distance <= r) {

    Serial.println("==============================");
    Serial.println("destination reached");

    // Fare cadere i semi
    semeServo.attach(semePin);
    semeServo.write(180);
    delay(1000);
    semeServo.detach();

    destN++; //Prossima destinazione
  }

  if (destN <= 1) {
    calcAngolo(); // calcolare l'angolo di rotazione per la prossima destinazione
  } else {
    Serial.println("░░░░░░░▄▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▄░░░░░░");
    Serial.println("░░░░░░█░░▄▀▀▀▀▀▀▀▀▀▀▀▀▀▄░░█░░░░░");
    Serial.println("░░░░░░█░█░░░░░░░░▄▀▀▄░▀░█░█▄▀▀▄░");
    Serial.println("█▀▀█▄░█░█░░▀░░░░░█░░░▀▄▄█▄▀░░░█░");
    Serial.println("▀▄▄░▀██░█▄░▀░░░▄▄▀░░░░░░░░░░░░▀▄");
    Serial.println("░░▀█▄▄█░█░░░░▄░░█░░░▄█░░░▄░▄█░░█");
    Serial.println("░░░░░▀█░▀▄▀░░░░░█░██░▄░░▄░░▄░███");
    Serial.println("░░░░░▄█▄░░▀▀▀▀▀▀▀▀▄░░▀▀▀▀▀▀▀░▄▀░");
    Serial.println("░░░░█░░▄█▀█▀▀█▀▀▀▀▀▀█▀▀█▀█▀▀█░░░");
    Serial.println("░░░░▀▀▀▀░░▀▀▀░░░░░░░░▀▀▀░░▀▀░░░░");
    Serial.println("Tutte le destinazioni sono state raggiunte!");

    // Entrare in un ciclo infinito e non fare niente. Eventualmente si potrebbe
    // aggiungere un pulsante per far ripartire il robot
    while (1);
  }
}

// aggiornare dati posizione
void positionLoop(int n) {
  // aggiorna le coordinate
  serialFlush();
  delay(100);
  for (int i = 0; i <= n; ++i) {
    readCoordinates();
  }

  // se il robot e` fuori dal campo visivo del Kinect eseguire lostRobot
  if (checkBounds == 'x') {
    lostRobot(lastDistance);
  }
}
void driveStraight(float dist, int power) {
  // numero di "ticks"
  unsigned long num_ticks_l;
  unsigned long num_ticks_r;

  // impostare potenza iniziale dei motori
  int power_l = motorPower;
  int power_r = motorPower;

  // usate per stabilire in quale senso si deve ruotare il motore per la correzione
  unsigned long diff_l;
  unsigned long diff_r;

  // resettare contatori encoder
  enc_l = 0;
  enc_r = 0;

  // salvare contatore encoder precedente
  unsigned long enc_l_prev = enc_l;
  unsigned long enc_r_prev = enc_r;

  // calcolare di quanto devono girare i motori
  float num_rev = (dist * 10) / wheel_c;  // Convert to mm
  unsigned long target_count = num_rev * countsPerRev;

  // Debug
  Serial.print("Driving for ");
  Serial.print(dist);
  Serial.print(" cm (");
  Serial.print(target_count);
  Serial.print(" ticks) at ");
  Serial.print(power);
  Serial.println(" motor power");

  // se il robot e` fuori dal campo visivo tornare indietro fino a rientrarci
  if (lostRobotBool) {
    Serial.println("Moving until robot is in");

    // muoversi in avanti finche` non viene stabilito che il robot e` rientrato nel campo visivo
    while ( checkBounds == 'x') {

      // Salvare numero di "ticks"
      num_ticks_l = enc_l;
      num_ticks_r = enc_r;

      // Drive
      drive(power_l, power_r);

      // numero di ticks dall'ultima volta
      diff_l = num_ticks_l - enc_l_prev;
      diff_r = num_ticks_r - enc_r_prev;

      // salvare numero di ticks per prossima volta
      enc_l_prev = num_ticks_l;
      enc_r_prev = num_ticks_r;

      // se motore sinistro piu` veloce, rallentarlo e velocizzare il destro
      if ( diff_l < diff_r ) {
        power_l -= motor_offset;
        power_r += motor_offset;
      }

      // se motore destro piu` veloce, rallentarlo e velocizzare il sinistro
      if ( diff_l > diff_r ) {
        power_l += motor_offset;
        power_r -= motor_offset;
      }

      //Controllare che non ci sia un ostacolo
      obstacle();

      serialFlush();
      delay(100);
      for (int i = 0; i <= 50; ++i) {
        readCoordinates();
      }
    }
  } else {
    // ruotare finche` uno dei motori non compie la distanza target
    while ( (enc_l < target_count) && (enc_r < target_count) ) {

      // Salvare numero di "ticks"
      num_ticks_l = enc_l;
      num_ticks_r = enc_r;

      // Drive
      drive(power_l, power_r);

      // numero di ticks dall'ultima volta
      diff_l = num_ticks_l - enc_l_prev;
      diff_r = num_ticks_r - enc_r_prev;

      // salvare numero di ticks per prossima volta
      enc_l_prev = num_ticks_l;
      enc_r_prev = num_ticks_r;

      // se motore sinistro piu` veloce, rallentarlo e velocizzare il destro
      if ( diff_l < diff_r ) {
        power_l -= motor_offset;
        power_r += motor_offset;
      }

      // se motore destro piu` veloce, rallentarlo e velocizzare il sinistro
      if ( diff_l > diff_r ) {
        power_l += motor_offset;
        power_r -= motor_offset;
      }

      obstacle();
    }
  }

  lostRobotBool = false;

  // frenare
  brake();
}
void drive(int power_a, int power_b) {

  // Limitare power da a -255 and 255
  power_a = constrain(power_a, -255, 255);
  power_b = constrain(power_b, -255, 255);

  // Direzione motore sinistro
  if ( power_a < 0 ) {
    digitalWrite(ain1_pin, LOW);
    digitalWrite(ain2_pin, HIGH);
  } else if (power_a > 0 ) {
    digitalWrite(ain1_pin, HIGH);
    digitalWrite(ain2_pin, LOW);
  } else {
    digitalWrite(ain1_pin, LOW);
    digitalWrite(ain2_pin, LOW);
  }

  // Direzione motore destro
  if ( power_b < 0 ) {
    digitalWrite(bin1_pin, LOW);
    digitalWrite(bin2_pin, HIGH);
  } else if ( power_b > 0) {
    digitalWrite(bin1_pin, HIGH);
    digitalWrite(bin2_pin, LOW);
  } else {
    digitalWrite(bin1_pin, LOW);
    digitalWrite(bin2_pin, LOW);
  }

  // Impostare velocita`
  analogWrite(pwma_pin, abs(power_a));
  analogWrite(pwmb_pin, abs(power_b));
}
void brake() {
  digitalWrite(ain1_pin, LOW);  // Imposta tutti i pin a 0V
  digitalWrite(ain2_pin, LOW);
  digitalWrite(bin1_pin, LOW);
  digitalWrite(bin2_pin, LOW); // Imposta il segnale pwm a 0
  analogWrite(pwma_pin, 0);
  analogWrite(pwmb_pin, 0);
}
void serialFlush() {
  while (mySerial.available())
    mySerial.read();
}
void readCoordinates() {
  if ( mySerial.available() >= MESSAGE_BYTES) // quando porta seriale BT disponibile
  {
    if ( mySerial.read() == HEADER)      // se riconosce il header
    {
      char tag = mySerial.read();        // var. char usata per ricevere il bit
      int sign = 1;                      // int per rendere il numero positivo o negativo

      if (tag == Xc)                     // se riconosce che il valore appartiene alla coordinata X
      {

        valX = mySerial.read() * 256;    // ricompone il numero, muovendo il byte di una posizione
        valX = valX + mySerial.read();   // aggiunge il bit successivo

        int checkSign = mySerial.read(); // funzione per verificare il segno
        if (checkSign == 'n') {          // se negativo Processing manda una n
          sign = -1;
        }

        valY *= sign;              // moltiplicare il valore per il segno

      } else if (tag == Yc) {             // processo analogo ma con la coordinata Y

        valY = mySerial.read() * 256;
        valY = valY + mySerial.read();

        int checkSign = mySerial.read();
        if (checkSign == 'n') {
          sign = -1;
        }

        valY *= sign;

        // se riceve y vuol dire che il robot e` dentro il campo visivo, se riceve x non lo e`
        checkBounds = mySerial.read();

        Serial.println(String(valY));

      } else {
        Serial.print("got message with unknown tag ");
        Serial.println(tag);
      }
    }
  }
}
void lostRobot(float lastDistance) {
  Serial.println("Robot is lost, turning 180 degrees and going back for " + String(lastDistance));
  turnRobot(turn180, motorPower, false);
  lostRobotBool = true;
  destinationMovement(lastDistance, false);
  driveStraight(10, motorPower);

  getDirection();
  }
  void countLeft() {
    enc_l++;
  }
  void countRight() {
    enc_r++;
  }
  long getDistance() {
  // calcolo della distanza che rileva il sensore a ultrasuoni
  long duration, distance;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) / 29.1;

  return distance;
}
void obstacle(){
    int distanceToObject = getDistance();
      if ( distanceToObject <= 20.0 && distanceToObject > 2.0) {
        brake();
        while (distanceToObject <= 20.0 && distanceToObject > 2.0) {
          Serial.println("Distance to object: " + String(distanceToObject) );
          distanceToObject = getDistance();
        }
      }

      // breve pausa per lasciare tempo ai motori per reagire
      delay(20);
  }
