#include <HX711.h>
#include <AccelStepper.h>
#include <ezButton.h>

// Pin definitions
#define STP1 22
#define STP2 23
#define DIR1 3
#define DIR2 2
#define MS11 24
#define MS12 25
#define EN1 26
#define EN2 27
#define BEP 52

HX711 scale;

// Load cell pin configuration
int DOUT_pin = SDA;
int SCK_pin = SCL;
float tare = 0.0;

// Calibration settings
const long LOADCELL_OFFSET = 25900;
const long LOADCELL_DIVIDER = 2280.f;
const long calibration_factor = 7200.0;
const long zero_factor = 898226.4;

// Limit switches
ezButton limit2(4);
ezButton limit1(5);
ezButton limit4(6);
ezButton limit3(7);

// Step counters and motion parameters
int stepsA = 0;
int stepsB = 0;
int requestedA = 0;
int requestedB = 0;
int StepDelay = 1200;
int x = 1;
int y = 1;

// ============================================================
// CALIBRATION DATA STRUCTURE
// ============================================================
struct CalibrationData {
  float slope;           // coefficiente angolare (mN per unità sensore)
  float intercept;       // intercetta
  float perimeter;       // perimetro piatto Wilhelmy (mm)
  boolean isCalibrated;  // flag di calibrazione
};

CalibrationData calibration = {0.0, 0.0, 0.0, false};

// ============================================================
// REGRESSIONE LINEARE - calcola slope e intercept
// ============================================================
void linearRegression(float* x_values, float* y_values, int n, float& slope, float& intercept) {
  float sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
  
  for (int i = 0; i < n; i++) {
    sum_x += x_values[i];
    sum_y += y_values[i];
    sum_xy += x_values[i] * y_values[i];
    sum_x2 += x_values[i] * x_values[i];
  }
  
  float denominator = (n * sum_x2) - (sum_x * sum_x);
  
  if (denominator == 0) {
    Serial.println("ERROR: Cannot calculate regression (denominator is zero)");
    slope = 0;
    intercept = 0;
    return;
  }
  
  slope = ((n * sum_xy) - (sum_x * sum_y)) / denominator;
  intercept = (sum_y - slope * sum_x) / n;
}

// ============================================================
// CONVERTI LETTURA SENSORE A MILLINEWTON
// ============================================================
float sensorToMillinewton(float raw_reading) {
  if (!calibration.isCalibrated) {
    Serial.println("ERROR: Scale not calibrated!");
    return 0.0;
  }
  return (raw_reading * calibration.slope) + calibration.intercept;
}

// ============================================================
// NUOVA FUNZIONE CALIBRAZIONE - Comando 'T'
// ============================================================
void calibrateScaleInteractive() {
  Serial.println("\n========== CALIBRAZIONE BILANCIA ==========");
  Serial.println("Assicurati che il piatto sia VUOTO e stabile");
  Serial.println("Premere un tasto e inviare quando pronto...");
  
  // Attesa input
  while (!Serial.available()) {
    delay(10);
  }
  Serial.read(); // leggi il carattere
  
  // Inizializza la bilancia
  scale.power_up();
  scale.begin(DOUT_pin, SCK_pin);
  scale.set_scale(LOADCELL_DIVIDER);
  scale.tare();
  
  delay(1000);
  
  // Array per i dati di calibrazione
  float sensor_readings[10];  // max 10 punti
  float known_weights[10];    // max 10 punti
  int num_points = 0;
  
  Serial.println("\n--- INSERIMENTO PUNTI DI CALIBRAZIONE ---");
  
  // Ciclo principale per i punti
  boolean adding_points = true;
  
  while (adding_points && num_points < 10) {
    Serial.print("\nPunto #");
    Serial.println(num_points + 1);
    Serial.println("Posiziona il peso sul piatto e stabilizza");
    Serial.println("Digita il peso (milligrammi) misurato dalla bilancia di riferimento:");
    
    // Attendi input peso
    while (!Serial.available()) {
      delay(10);
    }
    float weight = Serial.parseFloat();
    
    if (Serial.available()) Serial.read(); // leggi newline
    
    // Leggi dalla bilancia HX711
    Serial.println("Acquisizione lettura dal sensore...");
    delay(500);
    
    float sensor_value = scale.get_value(10); // media di 10 letture
    
    sensor_readings[num_points] = sensor_value;
    known_weights[num_points] = weight;
    num_points++;
    
    Serial.print("Peso dichiarato: ");
    Serial.print(weight);
    Serial.print(" g | Lettura sensore: ");
    Serial.println(sensor_value);
    
    // Chiedi se continuare (dopo primo punto, sempre dopo il secondo)
    if (num_points >= 2) {
      Serial.println("\nVuoi aggiungere un altro punto? (si/no)");
      
      String response = "";
      while (response == "") {
        if (Serial.available()) {
          response = Serial.readStringUntil('\n');
          response.toLowerCase();
          response.trim();
        }
        delay(10);
      }
      
      if (response != "si" && response != "yes" && response != "s" && response != "y") {
        adding_points = false;
      }
    }
  }
  
  Serial.print("\nTotale punti acquisiti: ");
  Serial.println(num_points);
  
  if (num_points < 2) {
    Serial.println("ERROR: Minimo 2 punti richiesti per la calibrazione!");
    scale.power_down();
    return;
  }
  
  // Chiedi perimetro piatto Wilhelmy
  Serial.println("\n--- PERIMETRO PIATTO WILHELMY ---");
  Serial.println("Inserisci il perimetro del piatto (mm):");
  
  while (!Serial.available()) {
    delay(10);
  }
  float perimeter = Serial.parseFloat();
  if (Serial.available()) Serial.read();
  
  calibration.perimeter = perimeter;
  
  Serial.print("Perimetro impostato a: ");
  Serial.print(perimeter);
  Serial.println(" mm");
  
  // Calcola regressione lineare
  Serial.println("\n--- Linear regression calculation  ---");
  linearRegression(sensor_readings, known_weights, num_points, calibration.slope, calibration.intercept);
  
  calibration.isCalibrated = true;
  
  Serial.println("Calibrazione completata!");
  Serial.print("Slope (mN per unità sensore): ");
  Serial.println(calibration.slope, 6);
  Serial.print("Intercept: ");
  Serial.println(calibration.intercept, 6);
  Serial.print("Perimetro: ");
  Serial.println(calibration.perimeter);
  
  // Test conversione
  Serial.println("\n--- TEST CONVERSIONE ---");
  for (int i = 0; i < num_points; i++) {
    float converted = sensorToMillinewton(sensor_readings[i]);
    float weight_mg = converted * 0.10197;  // Conversione mN → mg (precisa)
    Serial.print("Lettura ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(sensor_readings[i]);
    Serial.print(" → ");
    Serial.print(converted);
    Serial.print(" mN (");
    Serial.print(weight_mg, 4);
    Serial.print(" mg) | Atteso: ");
    Serial.print(known_weights[i]);
    Serial.println(" mg");
  }
  
  scale.power_down();
  Serial.println("\nDone");
}

// ============================================================
// LEGGI PESO CALIBRATO - Comando 'P'
// ============================================================
void readCalibratedWeight(int num_readings) {
  if (!calibration.isCalibrated) {
    Serial.println("ERROR: Non calibrated scale! Use 'T' command!");
    return;
  }
  
  if (num_readings <= 0) num_readings = 5; // default
  
  Serial.println("Start load cell to evaluate the measurement");
  delay(800);
  scale.power_up();
  scale.begin(DOUT_pin, SCK_pin);
  Serial.print("Load cell measurement");
  delay(800);
  Serial.print(".");
  delay(800);
  Serial.print(".");
  delay(800);
  Serial.println(".");
  delay(800);
  
  Serial.print("Average of ");
  Serial.print(num_readings);
  Serial.print(" weighings: ");
  
  // Ottieni lettura grezza come media
  float raw_reading = scale.get_value(num_readings);
  
  // Converti in mN usando la regressione
  float weight_mN = sensorToMillinewton(raw_reading);
  
  // Dividi per il perimetro (in mm) per ottenere mN/m
  float surface_tension = weight_mN / calibration.perimeter;
  
  Serial.println(surface_tension, 4);  // 4 decimali
  
  Serial.print("Raw reading: ");
  Serial.print(raw_reading);
  Serial.print(" | Weight: ");
  Serial.print(weight_mN);
  Serial.print(" mN | Surface tension: ");
  Serial.print(surface_tension);
  Serial.println(" mN/m");
  
  Serial.print("Actual position, X axis: ");
  Serial.print(stepsA);
  Serial.print(", Y Axis: ");
  Serial.println(stepsB);
  delay(1000);
  Serial.println("Load Cell in standby");
  scale.power_down();
  delay(1000);
}

// ============================================================
// SETUP e LOOP ORIGINALI (con modifiche)
// ============================================================

void setup() {
  // Motor and driver setup
  pinMode(STP1, OUTPUT);
  pinMode(STP2, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(MS11, OUTPUT);
  pinMode(MS12, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  digitalWrite(MS11, HIGH);
  digitalWrite(MS12, HIGH);
  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);

  // Limit switch debouncing
  limit1.setDebounceTime(50);
  limit2.setDebounceTime(50);
  limit3.setDebounceTime(50);
  limit4.setDebounceTime(50);

  // Load cell setup
  scale.begin(DOUT_pin, SCK_pin);
  scale.set_scale(LOADCELL_DIVIDER);
  scale.set_offset(LOADCELL_OFFSET);

  // Serial interface
  Serial.begin(9600);
  Serial.println("Send a letter followed by a number:");
  Serial.println("'w' and 's' move Y axis");
  Serial.println("'a' and 'd' move X axis");
  Serial.println("'A' and 'D' move both axes");
  Serial.println("'p' performs weighing (old method)");
  Serial.println("'x' and 'y' set motion ratios");
  Serial.println("'z' sets step delay");
  Serial.println("'F' triggers beeper");
  Serial.println("'u' waits (value * 100 millisec)");
  Serial.println("'h' homes the system");
  Serial.println("'t' taring");
  Serial.println("'T' CALIBRAZIONE INTERATTIVA (NEW)");
  Serial.println("'P' lettura peso calibrato (NEW)");
  Serial.println("Ready");
}

void limits() {
  limit1.loop();
  limit2.loop();
  limit3.loop();
  limit4.loop();
}

// Axis movement
void go_up () {
  limits();
  digitalWrite(DIR2, HIGH);
  digitalWrite(STP2, HIGH);
  delayMicroseconds(StepDelay);
  digitalWrite(STP2, LOW);
  delayMicroseconds(StepDelay);
  stepsB++;
}

void go_down () {
  limits();
  digitalWrite(DIR2, LOW);
  digitalWrite(STP2, HIGH);
  delayMicroseconds(StepDelay);
  digitalWrite(STP2, LOW);
  delayMicroseconds(StepDelay);
  stepsB--;
}

void close() {
  limits();
  digitalWrite(DIR1, LOW);
  digitalWrite(STP1, HIGH);
  delayMicroseconds(StepDelay);
  digitalWrite(STP1, LOW);
  delayMicroseconds(StepDelay);
  stepsA++;
}

void open () {
  limits();
  digitalWrite(DIR1, HIGH);
  digitalWrite(STP1, HIGH);
  delayMicroseconds(StepDelay);
  digitalWrite(STP1, LOW);
  delayMicroseconds(StepDelay);
  stepsA--;
}

// Homing sequence
void home() {
  int tempD = StepDelay;
  StepDelay = 100;

  limits();

  digitalWrite(EN1, LOW);
  while (limit2.getState() == LOW) close();
  while (limit1.getState() == LOW) open();
  digitalWrite(EN1, HIGH);

  digitalWrite(EN2, LOW);
  while (limit3.getState() == LOW) go_down();
  while (limit4.getState() == LOW) go_up();
  digitalWrite(EN2, HIGH);

  stepsA = 0;
  stepsB = 0;

  StepDelay = tempD;
}

// Main loop
void loop() {
  limits();
  scale.set_scale(2280.f);

  if (Serial.available()) {
    char command = Serial.read();
    int param = Serial.parseInt();

    if (command == 'a') {
      digitalWrite(EN1, LOW);
      int state1 = limit1.getState();
      if (state1 == LOW) {
        for (int i = 0; i < param; i++) {
          int state1 = limit1.getState();
          if (state1 == LOW) {
            open();
          } else {
            Serial.println("LIMIT SWITCH X PRESSED");
            break;
          }
        }
      } 
      else {
        Serial.println("LIMIT SWITCH X PRESSED");
      }
      Serial.print("X Axis moved to position: ");
      Serial.println(stepsA);
      digitalWrite(EN1, HIGH);
      Serial.println("Done");
    }
    else if (command == 'd') {
      digitalWrite(EN1, LOW);
      int state2 = limit2.getState();
      if (state2 == LOW) {
        for (int i = 0; i < param; i++) {
          int state2 = limit2.getState();
          if (state2 == LOW) {
            close();
          } else {
            Serial.println("LIMIT SWITCH X PRESSED");
            break;
          }
        }
      } else {
        Serial.println("LIMIT SWITCH X PRESSED");
      }
      Serial.print("X Axis moved to position: ");
      Serial.println(stepsA);
      digitalWrite(EN1, HIGH);
      Serial.println("Done");
    }
    else if (command == 'w') {
      digitalWrite(EN2, LOW);
      int state4 = limit4.getState();
      if (state4 == LOW) {
        for (int i = 0; i < param; i++) {
          int state4 = limit4.getState();
          if (state4 == LOW) {
            go_up();
          } else {
            Serial.println("LIMIT SWITCH Y PRESSED");
            break;
          }
        }
      } else {
        Serial.println("LIMIT SWITCH Y PRESSED");
      }
      Serial.print("Y Axis moved to position: ");
      Serial.println(stepsB);
      digitalWrite(EN2, HIGH);
      Serial.println("Done");
    }
    else if (command == 's') {
      digitalWrite(EN2, LOW);
      int state3 = limit3.getState();
      if (state3 == LOW) {
        for (int i = 0; i < param; i++) {
          int state3 = limit3.getState();
          if (state3 == LOW) {
            go_down();
          } else {
            Serial.println("LIMIT SWITCH Y PRESSED");
            break;
          }
        }
      } else {
        Serial.println("LIMIT SWITCH Y PRESSED");
      }
      Serial.print("Y Axis moved to position: ");
      Serial.println(stepsB);
      digitalWrite(EN2, HIGH);
      Serial.println("Done");
    }
    else if (command == 'C') {
      digitalWrite(EN1, LOW);
      digitalWrite(EN2, LOW);
      int state3 = limit3.getState();
      int state1 = limit1.getState();
      if (state3 == LOW || state1 == LOW) {
        for (int i = 0; i < param; i++) {
          int state3 = limit3.getState();
          int state1 = limit1.getState();
          if (state3 == LOW && state1 == LOW) {
            for (int i = 0; i < x; i++) {
              close();
            }
            for (int i = 0; i < y; i++) {
              go_down();
            }
          } else {
            Serial.println("LIMIT SWITCH PRESSED");
            break;
          }
        }
      } else {
        Serial.println("LIMIT SWITCH PRESSED");
      }
      Serial.print("X Axis moved to position: ");
      Serial.println(stepsA);
      Serial.print("Y Axis moved to position: ");
      Serial.println(stepsB);
      digitalWrite(EN1, HIGH);
      digitalWrite(EN2, HIGH);
      Serial.println("Done");
    }
    else if (command == 'A') {
      digitalWrite(EN1, LOW);
      digitalWrite(EN2, LOW);
      int state4 = limit4.getState();
      int state2 = limit2.getState();
      if (state4 == LOW || state2 == LOW) {
        for (int i = 0; i < param; i++) {
          int state4 = limit4.getState();
          int state2 = limit2.getState();
          if (state4 == LOW && state2 == LOW) {
            for (int i = 0; i < x; i++) {
              open();
            }
            for (int i = 0; i < y; i++) {
              go_up();
            }
          } else {
            Serial.println("LIMIT SWITCH PRESSED");
            break;
          }
        }
      } else {
        Serial.println("LIMIT SWITCH PRESSED");
      }
      Serial.print("X Axis moved to position: ");
      Serial.println(stepsA);
      Serial.print("Y Axis moved to position: ");
      Serial.println(stepsB);
      digitalWrite(EN1, HIGH);
      digitalWrite(EN2, HIGH);
      Serial.println("Done");
    }
    else if (command == 'D') {
      digitalWrite(EN1, LOW);
      digitalWrite(EN2, LOW);
      int state1 = limit1.getState();
      int state4 = limit4.getState();
      if (state1 == LOW || state4 == LOW) {
        for (int i = 0; i < param; i++) {
          int state1 = limit1.getState();
          int state4 = limit4.getState();
          if (state1 == LOW && state4 == LOW) {
            for (int i = 0; i < x; i++) {
              close();
            }
            for (int i = 0; i < y; i++) {
              go_up();
            }
          } else {
            Serial.println("LIMIT SWITCH PRESSED");
            break;
          }
        }
      } else {
        Serial.println("LIMIT SWITCH PRESSED");
      }
      Serial.print("X Axis moved to position: ");
      Serial.println(stepsA);
      Serial.print("Y Axis moved to position: ");
      Serial.println(stepsB);
      digitalWrite(EN1, HIGH);
      digitalWrite(EN2, HIGH);
      Serial.println("Done");
    }
    else if (command == 'x') {
      if (param != 0) {
        x = param;
        Serial.print("x set as: ");
        Serial.println(x);
      } else {
        Serial.println("You DO NOT divide by zero.");
      }
      Serial.println("Done");
    }
    else if (command == 'y') {
      if (param != 0) {
        y = param;
        Serial.print("y set as: ");
        Serial.println(y);
      } else {
        Serial.println("You DO NOT divide by zero.");
      }
      Serial.println("Done");
    }
    else if (command == 'p') {
      Serial.println("Start load cell to evaluate the load of load cell");
      delay(800);
      scale.power_up();
      scale.begin(DOUT_pin, SCK_pin);
      Serial.print("Load cell measurement");
      delay(800);
      Serial.print(".");
      delay(800);
      Serial.print(".");
      delay(800);
      Serial.println(".");
      delay(800);
      Serial.print("Average of ");
      Serial.print(param);
      Serial.print(" weighings: ");
      Serial.println(scale.get_value(param) - tare);
      param = 0;
      Serial.print("Actual position, X axis: ");
      Serial.print(stepsA);
      Serial.print(", Y Axis: ");
      Serial.println(stepsB);
      delay(1000);
      Serial.println("Load Cell in standby");
      scale.power_down();
      delay(1000);
      Serial.println("Done");
    }
    else if (command == 'F') {
      digitalWrite(BEP, HIGH);
      delayMicroseconds(1000);
      digitalWrite(BEP, LOW);
      delay(2000);
      digitalWrite(BEP, HIGH);
      delayMicroseconds(1000);
      digitalWrite(BEP, LOW);
      delay(2000);
      digitalWrite(BEP, HIGH);
      delayMicroseconds(1000);
      digitalWrite(BEP, LOW);
      delay(2000);
      Serial.println("Done");
    }
    else if (command == 'u') {
      delay(param * 100);
      Serial.println("Done");
    }
    else if (command == 'z') {
      if (param > 99) {
        StepDelay = param;
        Serial.print("StepDelay set as: ");
        Serial.println(StepDelay);
      } else {
        Serial.println("Cannot go that fast");
      }
      Serial.println("Done");
    }
    else if (command == 'h') {
      home();
      Serial.println("homed");
      Serial.println("Done");
    }
    else if (command == 't') {
      scale.power_up();
      Serial.println("tare test");
      Serial.println(scale.get_value(param), 1);
      tare = (scale.get_value(param));
      Serial.println("tare set up");
      Serial.println(tare);
      Serial.println("Done");
    }
    else if (command == 'T') {
      calibrateScaleInteractive();
    }
    else if (command == 'P') {
      readCalibratedWeight(param > 0 ? param : 5);
      Serial.println("Done");
    }
  }
}
