/*
  Machine à cocktails - ATmega2560 + CNC Shield V3
  ------------------------------------------------
  Librairies : AccelStepper, Adafruit NeoPixel (Servo est incluse dans l'IDE)

  Protocole série (USB depuis le RPi à 115200, et Serial1 à 9600 pour le HC-06) :
    1..n | C3 | COCKTAIL:12 -> prépare le cocktail n° (n = nombre de recettes dans RECIPES)
    STOP                     -> arrêt de la préparation / du déplacement en cours
    HOME                     -> refait le homing (rapide)
    STATUS                   -> état / verre / position
    SENSOR                   -> valeur brute du RP-S40-ST (calibration)
    PUMP:n:ms                -> test de la pompe n (1..4) pendant ms
    PUMPML:n:ml              -> pompe n pendant la durée calculée pour ml (vérifie PUMP_MS_PER_ML)
    SERVO:angle              -> place le servo à l'angle 0..180 (réglage de SERVO_REST / SERVO_PUSH)
    DOSE:n                   -> n doses (1..5) à la position actuelle du chariot

  Commandes de TEST des positions (moniteur série, 115200, "Nouvelle ligne") :
    POS                      -> position actuelle depuis le HOME (cm et pas)
    GOTO:x                   -> va à x cm depuis le HOME (ex : GOTO:40)
    JOG:d                    -> déplacement relatif en cm (ex : JOG:0.5 ou JOG:-0.2)
    JUICE                    -> va à JUICE_POS
    DISP:n                   -> va au distributeur d'alcool n (1..6 = Hard 1..6)
    TOUR                     -> HOME puis jus + 6 distributeurs, pause 4 s à chaque arrêt

  Réponses : START:<nom>, DONE, ERR:<raison>, ...

  Les valeurs marquées "A AJUSTER" dépendent de ta mécanique.
*/

#include <Arduino.h>
#include <AccelStepper.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

// ====================== BROCHES ======================
const uint8_t PIN_STEP    = 2;    // CNC Shield axe Y : STEP
const uint8_t PIN_DIR     = 5;    // CNC Shield axe Y : DIR
const uint8_t PIN_ENABLE  = 4;    // CNC Shield : EN (actif à LOW)
const uint8_t PIN_ENDSTOP = 9;   // A AJUSTER : Y- du CNC Shield = D10 (pin non précisée)
const uint8_t PIN_PUMP[4] = {23, 25, 27, 29};
const uint8_t PIN_GLASS   = A9;   // RP-S40-ST
const uint8_t PIN_LED     = 22;   // WS2812B
const uint8_t PIN_SERVO   = A8;   // HS-422

// ====================== CONFIG ======================
// --- Moteur ---
const float    STEPS_PER_UNIT = 1600.0; // A AJUSTER : pas par cm RÉEL. Mesuré : à 400, JUICE (25) = 6 cm et DISP:1 (35) = 8,5 cm
                                        // de l'end-stop => le chariot faisait 4x trop peu => 400 x 4 = 1600.
                                        // Calibration précise : HOME, GOTO:80, mesure D (cm) -> STEPS_PER_UNIT = ancienne valeur x 80 / D
const float    MAX_SPEED      = 5000;   // pas/s
const float    ACCELERATION   = 3000;   // pas/s²
const float    MAX_TRAVEL     = 87;     // course utile x=0 -> x=90 (limite des déplacements GOTO/JOG)
const bool     INVERT_DIR     = true;  // A AJUSTER : true si le chariot part du mauvais côté au homing
const uint8_t  ENDSTOP_PRESSED = LOW;   // INPUT_PULLUP : contact NO vers GND -> LOW quand appuyé

// --- Homing (2 passes : approche RAPIDE puis approche LENTE de précision) ---
const float HOMING_FAST_SPEED    = 4500;  // pas/s  : approche rapide (anciennement 1500). Max conseillé = MAX_SPEED
const float HOMING_FAST_ACCEL    = 4000;  // pas/s² : accélération pendant l'approche rapide
const float HOMING_BACKOFF       = 0.5;   // cm     : recul après le 1er contact avant la passe lente
const float HOMING_SLOW_SPEED    = 600;   // pas/s  : approche lente (précision / répétabilité de x=0)
const float HOMING_RELEASE_SPEED = 800;   // pas/s  : dégagement de l'end-stop

// --- Démarrage ---
const bool AUTO_HOME_AT_BOOT = false;   // false : le chariot ne bouge pas à l'allumage, on lance HOME à la main

// --- Positions (x) ---
// Positions mesurées à la règle depuis l'end-stop (cm) : Soft 25, Hard 1..6 = 35, 45, 55, 65, 75, 85
// ENDSTOP_TO_GLASS : décalage entre le contact de l'end-stop et le centre du verre quand le chariot est à HOME.
// Laisse 0 ; si le verre est trop loin de X cm partout (même écart sur tous les distributeurs), mets X ici.
const float ENDSTOP_TO_GLASS = 4.5;
const float JUICE_POS = 25 - ENDSTOP_TO_GLASS;                       // distributeur 1 : Soft (jus)
const float ALCOHOL_POS[6] = {35 - ENDSTOP_TO_GLASS, 45 - ENDSTOP_TO_GLASS, 55 - ENDSTOP_TO_GLASS,
                              65 - ENDSTOP_TO_GLASS, 75 - ENDSTOP_TO_GLASS, 85 - ENDSTOP_TO_GLASS};  // Hard 1..6 (distributeurs 2..7)

// Ordre des alcools sur les distributeurs (index 0..5 = Hard 1..6, positions 35, 45, 55, 65, 75, 85 cm)
enum Alcohol { RHUM_BLANC, WHISKY, TEQUILA, VODKA, CURACAO_BLEU, AMARETTO };
// Ordre des pompes (jus) : 0 Orange, 1 Ananas, 2 Cranberry, 3 Pamplemousse

// --- Relais pompes (module 4 relais, souvent actif à LOW) ---
const uint8_t RELAY_ON  = LOW;
const uint8_t RELAY_OFF = HIGH;

// --- Détecteur de verre ---
const int           GLASS_THRESHOLD        = 500;  // A AJUSTER (commande SENSOR)
const bool          GLASS_HIGH_WHEN_PRESENT = true; // false si la valeur baisse avec un verre
const unsigned long GLASS_DEBOUNCE_MS      = 200;

// --- Servo (doseur 25 ml) ---
const uint8_t       SERVO_REST      = 0;     // A AJUSTER (degrés)
const uint8_t       SERVO_PUSH      = 60;    // A AJUSTER
const unsigned long SERVO_PUSH_MS   = 1200;  // temps de maintien (écoulement)
const unsigned long SERVO_REFILL_MS = 1500;  // temps de remplissage du doseur
const uint8_t       DOSE_ML         = 25;    // A AJUSTER : volume d'une dose du doseur (ml)

// --- Pompes (jus) ---
// Durée de pompe par ml. Calibration : PUMP:1:10000 puis mesure les ml versés -> 10000 / ml = ms par ml
const uint16_t      PUMP_MS_PER_ML[4] = {60, 60, 60, 60};   // A AJUSTER pour chaque pompe

// --- LED ---
const uint16_t LED_COUNT      = 30;          // A AJUSTER
const uint8_t  LED_BRIGHTNESS = 80;

// ====================== RECETTES ======================
// Quantités en ml. Alcools : un nombre de doses est calculé (arrondi à DOSE_ML). Jus : durée = ml x PUMP_MS_PER_ML.
struct Recipe {
  const char* name;
  uint8_t alcMl[6];     // ml : Rhum blanc, Whisky, Tequila, Vodka, Curaçao bleu, Amaretto
  uint8_t juiceMl[4];   // ml : Orange, Ananas, Cranberry, Pamplemousse
};

// Pour ajouter un cocktail : ajoute une ligne (le numéro = sa position dans la liste).
// Alcools : multiples de DOSE_ML (25 ml = 1 dose, 50 ml = 2 doses). Jus : multiples de 5 ml.
// Volume total de chaque cocktail = 240 ml (24 cl).
//                          Rhum  Whisk Teq  Vodka Cura  Amar      Oran Anan Cran Pamp
const Recipe RECIPES[] = {
  {"Blue Hawaii",       { 25,   0,   0,   0,  25,   0},   { 55, 135,   0,   0}},  //  1
  {"Rhum Tropical",     { 50,   0,   0,   0,   0,   0},   { 65, 125,   0,   0}},  //  2
  {"Pineapple Vodka",   {  0,   0,   0,  50,   0,   0},   { 40, 150,   0,   0}},  //  3
  {"Tequila Sunrise",   {  0,   0,  50,   0,   0,   0},   {150,   0,  40,   0}},  //  4
  {"Pink Paloma",       {  0,   0,  50,   0,   0,   0},   {  0,   0,  45, 145}},  //  5
  {"Amaretto Sunset",   {  0,   0,   0,   0,   0,  50},   { 95,  70,  25,   0}},  //  6
  {"Almond Kicker",     {  0,   0,   0,   0,   0,  50},   { 75,   0,   0, 115}},  //  7
  {"Whisky Orange",     {  0,  50,   0,   0,   0,   0},   {165,   0,  25,   0}},  //  8
  {"Whisky Paradise",   {  0,  50,   0,   0,   0,   0},   { 55, 135,   0,   0}},  //  9
  {"Blue Lagoon",       {  0,   0,   0,  50,  25,   0},   {110,  55,   0,   0}},  // 10
  {"Sea Breeze",        {  0,   0,   0,  50,   0,   0},   {  0,   0,  95,  95}},  // 11
  {"Pink Vodka",        {  0,   0,   0,  50,   0,   0},   { 80,   0, 110,   0}},  // 12
  {"Tequila Breeze",    {  0,   0,  50,   0,   0,   0},   { 55,   0,   0, 135}},  // 13
  {"Caribbean Red",     { 50,   0,   0,   0,   0,   0},   {  0,  95,  95,   0}},  // 14
  {"Citrus Dream",      {  0,   0,   0,  50,   0,   0},   { 95,   0,   0,  95}},  // 15
  {"Blue Paradise",     { 25,   0,   0,   0,  50,   0},   {  0, 110,  55,   0}},  // 16
  {"Golden Sunset",     {  0,  25,   0,   0,   0,  50},   {110,   0,  55,   0}},  // 17
  {"Mexican Paradise",  {  0,   0,  50,   0,   0,   0},   { 35,  90,   0,  65}},  // 18
  {"Pink Almond",       {  0,   0,   0,   0,   0,  50},   {  0,   0, 110,  80}},  // 19
  {"Blue Tropical",     {  0,   0,   0,  25,  25,   0},   {  0, 115,   0,  75}},  // 20
  {"Purple Storm",      {  0,   0,   0,  50,  25,   0},   {  0,   0,  95,  70}},  // 21
  {"Golden Caribbean",  { 25,   0,   0,   0,   0,  25},   { 55, 135,   0,   0}},  // 22
  {"Ruby Manhattan",    {  0,  50,   0,   0,   0,   0},   {  0,   0, 115,  75}},  // 23
  {"Blue Margarita",    {  0,   0,  50,   0,  25,   0},   {  0,  55,   0, 110}},  // 24
};
const uint8_t NB_RECIPES = sizeof(RECIPES) / sizeof(RECIPES[0]);

// Nombre de doses pour une quantité en ml (arrondi au plus proche ; 0 = trop peu, on saute)
uint8_t dosesFor(uint8_t ml) { return (ml + DOSE_ML / 2) / DOSE_ML; }

// ====================== OBJETS / ÉTAT ======================
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);
Servo doser;
Adafruit_NeoPixel strip(LED_COUNT, PIN_LED, NEO_GRB + NEO_KHZ800);

enum State { IDLE, BUSY, DONE, ERROR };
State state = IDLE;
bool glass = false;          // verre présent (après anti-rebond)
bool working = false;        // une opération est en cours (anti-réentrance)
bool abortFlag = false;
bool homed = false;
bool testMove = false;       // true pendant les commandes de test : ignore le détecteur de verre
uint32_t lastColor = 0x01000000;  // valeur impossible -> force le 1er affichage

Stream* ports[2] = {&Serial, &Serial1};
char lineBuf[2][40];
uint8_t lineLen[2] = {0, 0};

// ====================== UTILITAIRES ======================
void reply(const char* s) {
  Serial.println(s);
  Serial1.println(s);
}

bool endstopPressed() { return digitalRead(PIN_ENDSTOP) == ENDSTOP_PRESSED; }

void allPumpsOff() {
  for (uint8_t i = 0; i < 4; i++) digitalWrite(PIN_PUMP[i], RELAY_OFF);
}

void updateGlass() {
  static bool last = false;
  static unsigned long t = 0;
  int v = analogRead(PIN_GLASS);
  bool raw = GLASS_HIGH_WHEN_PRESENT ? (v > GLASS_THRESHOLD) : (v < GLASS_THRESHOLD);
  if (raw != last) { last = raw; t = millis(); }
  if (raw != glass && millis() - t > GLASS_DEBOUNCE_MS) glass = raw;
}

void updateLeds() {
  uint32_t c;
  unsigned long now = millis();
  switch (state) {
    case BUSY:  c = strip.Color(0, 0, 255); break;
    case DONE:  c = strip.Color(0, 255, 0); break;
    case ERROR: c = ((now / 250) % 2) ? strip.Color(255, 0, 0) : 0; break;
    default:
      if (glass) c = strip.Color(255, 0, 0);                                   // verre : rouge fixe
      else       c = ((now / 500) % 2) ? strip.Color(255, 255, 255) : 0;       // sans verre : blanc clignotant
  }
  if (c != lastColor) {        // show() seulement si changement (évite de perturber le moteur)
    strip.fill(c);
    strip.show();
    lastColor = c;
  }
}

void handleLine(char* line, uint8_t port);

void pollSerial() {
  for (uint8_t p = 0; p < 2; p++) {
    while (ports[p]->available()) {
      char c = ports[p]->read();
      if (c == '\n' || c == '\r') {
        if (lineLen[p] > 0) {
          char tmp[40];
          lineBuf[p][lineLen[p]] = 0;
          strcpy(tmp, lineBuf[p]);
          lineLen[p] = 0;           // libère le buffer avant traitement
          handleLine(tmp, p);
        }
      } else if (lineLen[p] < sizeof(lineBuf[p]) - 1) {
        lineBuf[p][lineLen[p]++] = toupper(c);
      }
    }
  }
}

// Tâches de fond, appelées pendant les attentes et mouvements (limitées à 1 fois / 20 ms)
void service() {
  static unsigned long last = 0;
  if (millis() - last < 20) return;
  last = millis();
  updateGlass();
  pollSerial();
  updateLeds();
}

// En mode test, l'absence de verre n'interrompt pas le déplacement
bool abortRequested() { return abortFlag || (!testMove && !glass); }

bool waitMs(unsigned long ms) {
  unsigned long t = millis();
  while (millis() - t < ms) {
    service();
    if (abortRequested()) return false;
  }
  return true;
}

// ====================== MOUVEMENT ======================
// Dégage l'end-stop lentement (si on est dessus)
void releaseEndstop() {
  stepper.setSpeed(HOMING_RELEASE_SPEED);   // sens positif = s'éloigne de x=0
  long n = 0;
  const long limit = (long)(10 * STEPS_PER_UNIT);
  while (endstopPressed() && n < limit) {
    if (stepper.runSpeed()) n++;
    service();
  }
}

bool homingFail(bool prev) {
  state = ERROR;
  reply("ERR:HOMING");
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(ACCELERATION);
  working = prev;
  return false;
}

bool homeAxis() {
  bool prev = working;
  working = true;
  homed = false;

  const long maxSteps = (long)((MAX_TRAVEL + 20) * STEPS_PER_UNIT);
  stepper.setMaxSpeed(HOMING_FAST_SPEED);   // toutes les vitesses de homing sont <= à celle-ci

  // 0) Si l'end-stop est déjà appuyé : on s'en écarte d'abord
  if (endstopPressed()) releaseEndstop();

  // 1) Approche RAPIDE vers x=0 (avec rampe d'accélération)
  stepper.setAcceleration(HOMING_FAST_ACCEL);
  stepper.setCurrentPosition(0);
  stepper.moveTo(-maxSteps);
  while (!endstopPressed()) {
    stepper.run();
    service();
    if (stepper.distanceToGo() == 0) return homingFail(prev);   // course max parcourue sans contact
  }

  // 2) Recul court pour libérer l'end-stop
  stepper.setCurrentPosition(0);
  stepper.moveTo((long)(HOMING_BACKOFF * STEPS_PER_UNIT));
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    service();
  }
  if (endstopPressed()) releaseEndstop();   // sécurité si le recul n'a pas suffi

  // 3) Approche LENTE de précision
  stepper.setSpeed(-HOMING_SLOW_SPEED);
  long n = 0;
  const long slowMax = (long)((HOMING_BACKOFF + 3) * STEPS_PER_UNIT);
  while (!endstopPressed()) {
    if (stepper.runSpeed()) n++;
    service();
    if (n > slowMax) return homingFail(prev);
  }

  // 4) Dégagement lent puis x=0
  stepper.setSpeed(HOMING_SLOW_SPEED);
  while (endstopPressed()) { stepper.runSpeed(); service(); }
  long extra = (long)(0.2 * STEPS_PER_UNIT);
  while (extra > 0) { if (stepper.runSpeed()) extra--; }

  stepper.setCurrentPosition(0);
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(ACCELERATION);
  homed = true;
  if (state == ERROR) state = IDLE;
  working = prev;
  return true;
}

bool moveToUnits(float x) {
  stepper.moveTo((long)(x * STEPS_PER_UNIT));
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    service();
    if (abortRequested()) {
      stepper.stop();                  // décélération propre
      while (stepper.run()) {}
      return false;
    }
  }
  return true;
}

// ====================== DISTRIBUTION ======================
bool pumpFor(uint8_t i, unsigned long ms) {
  digitalWrite(PIN_PUMP[i], RELAY_ON);
  bool ok = waitMs(ms);
  digitalWrite(PIN_PUMP[i], RELAY_OFF);
  return ok;
}

bool dispenseAlcohol(uint8_t doses) {
  doser.attach(PIN_SERVO);
  doser.write(SERVO_REST);
  bool ok = true;
  for (uint8_t d = 0; d < doses && ok; d++) {
    doser.write(SERVO_PUSH);
    ok = waitMs(SERVO_PUSH_MS);
    doser.write(SERVO_REST);
    if (ok) ok = waitMs(SERVO_REFILL_MS);
  }
  doser.write(SERVO_REST);
  delay(300);
  doser.detach();
  return ok;
}

void makeCocktail(uint8_t n) {
  if (n < 1 || n > NB_RECIPES) { reply("ERR:UNKNOWN"); return; }
  if (!glass) { reply("ERR:NO_GLASS"); return; }
  if (!homed && !homeAxis()) return;

  const Recipe& r = RECIPES[n - 1];
  for (uint8_t i = 0; i < 6; i++)
    if (dosesFor(r.alcMl[i]) > 0 && ALCOHOL_POS[i] < 0) { reply("ERR:ALCOHOL_POS"); return; }

  working = true;
  abortFlag = false;
  state = BUSY;
  Serial.print("START:"); Serial.println(r.name);
  Serial1.print("START:"); Serial1.println(r.name);

  bool ok = true;

  // 1) Jus (pompes l'une après l'autre au-dessus du verre)
  bool hasJuice = false;
  for (uint8_t i = 0; i < 4; i++) if (r.juiceMl[i] > 0) hasJuice = true;
  if (hasJuice) {
    ok = moveToUnits(JUICE_POS);
    for (uint8_t i = 0; i < 4 && ok; i++) {
      if (r.juiceMl[i] > 0) {
        ok = pumpFor(i, (unsigned long)r.juiceMl[i] * PUMP_MS_PER_ML[i]);
        if (ok) ok = waitMs(300);
      }
    }
  }

  // 2) Alcools : du plus proche au plus loin (Hard 1 -> Hard 6), donc un seul aller
  for (uint8_t i = 0; i < 6 && ok; i++) {
    uint8_t doses = dosesFor(r.alcMl[i]);
    if (doses == 0) continue;
    ok = moveToUnits(ALCOHOL_POS[i]);
    if (ok) ok = dispenseAlcohol(doses);
  }

  // 3) Retour au point de départ
  if (ok) ok = moveToUnits(0);

  allPumpsOff();
  if (ok) {
    state = DONE;
    working = false;
    reply("DONE");
  } else {
    doser.detach();
    state = IDLE;
    homeAxis();                 // remet le chariot à x=0 par sécurité
    working = false;
    abortFlag = false;
    reply("ERR:ABORTED");
  }
}

// ====================== TESTS DE POSITION ======================
float currentCm() { return stepper.currentPosition() / STEPS_PER_UNIT; }

void printPos(const char* label) {
  char f[14], b[64];
  dtostrf(currentCm(), 1, 2, f);
  snprintf(b, sizeof(b), "%s x=%s cm (%ld pas depuis HOME)", label, f, stepper.currentPosition());
  reply(b);
}

// Déplacement de test (ignore le détecteur de verre). STOP l'interrompt.
bool testGoto(float x, const char* label) {
  if (!homed) { reply("ERR:NOT_HOMED (fais HOME d'abord)"); return false; }
  if (x < 0) x = 0;
  if (x > MAX_TRAVEL) x = MAX_TRAVEL;
  working = true; abortFlag = false; testMove = true;
  bool ok = moveToUnits(x);
  testMove = false; working = false;
  if (!ok) abortFlag = false;
  printPos(ok ? label : "INTERROMPU");
  return ok;
}

void testTour() {
  if (!homeAxis()) return;
  printPos("HOME OK");
  working = true; abortFlag = false; testMove = true;

  bool ok = moveToUnits(JUICE_POS);
  if (ok) { printPos("TOUR:JUS"); ok = waitMs(4000); }

  for (uint8_t i = 0; i < 6 && ok; i++) {
    if (ALCOHOL_POS[i] < 0) continue;     // position non configurée : on saute
    ok = moveToUnits(ALCOHOL_POS[i]);
    if (ok) {
      char b[16];
      snprintf(b, sizeof(b), "TOUR:DISP%d", i + 1);
      printPos(b);
      ok = waitMs(4000);
    }
  }
  if (ok) ok = moveToUnits(0);

  testMove = false; working = false; abortFlag = false;
  reply(ok ? "TOUR:FIN" : "TOUR:INTERROMPU");
}

// ====================== COMMANDES ======================
void handleLine(char* line, uint8_t port) {
  (void)port;
  if (working) {
    if (strncmp(line, "STOP", 4) == 0) { abortFlag = true; reply("OK:STOP"); }
    else reply("ERR:BUSY");
    return;
  }

  if (strncmp(line, "STOP", 4) == 0) { allPumpsOff(); reply("OK:STOP"); return; }
  if (strncmp(line, "HOME", 4) == 0) { if (homeAxis()) { reply("OK:HOMED"); printPos("POS"); } return; }

  // --- Commandes de test des positions ---
  if (strncmp(line, "POS", 3) == 0) { printPos(homed ? "POS" : "POS (non homé)"); return; }

  if (strncmp(line, "GOTO:", 5) == 0) {
    testGoto(atof(line + 5), "GOTO");
    return;
  }

  if (strncmp(line, "JOG:", 4) == 0) {
    testGoto(currentCm() + atof(line + 4), "JOG");
    return;
  }

  if (strncmp(line, "JUICE", 5) == 0) { testGoto(JUICE_POS, "JUS"); return; }

  if (strncmp(line, "DISP:", 5) == 0) {
    int d = atoi(line + 5);
    if (d >= 1 && d <= 6 && ALCOHOL_POS[d - 1] >= 0) {
      char b[12];
      snprintf(b, sizeof(b), "DISP%d", d);
      testGoto(ALCOHOL_POS[d - 1], b);
    } else reply("ERR:DISP_ARGS (position non configuree)");
    return;
  }

  if (strncmp(line, "TOUR", 4) == 0) { testTour(); return; }

  // --- Commandes normales ---
  if (strncmp(line, "SENSOR", 6) == 0) {
    char b[32];
    snprintf(b, sizeof(b), "SENSOR:%d", analogRead(PIN_GLASS));
    reply(b);
    return;
  }

  if (strncmp(line, "STATUS", 6) == 0) {
    static const char* names[] = {"IDLE", "BUSY", "DONE", "ERROR"};
    char b[64];
    snprintf(b, sizeof(b), "STATUS:%s GLASS:%d HOMED:%d", names[state], glass, homed);
    reply(b);
    return;
  }

  if (strncmp(line, "PUMP:", 5) == 0) {       // PUMP:n:ms
    int p = 0; long ms = 0;
    if (sscanf(line + 5, "%d:%ld", &p, &ms) == 2 && p >= 1 && p <= 4 && ms > 0 && ms <= 30000) {
      working = true; abortFlag = false; testMove = true;   // test : pas besoin de verre
      pumpFor(p - 1, ms);
      testMove = false; working = false; abortFlag = false;
      reply("OK:PUMP");
    } else reply("ERR:PUMP_ARGS (PUMP:n:ms, n=1..4, ms<=30000)");
    return;
  }

  if (strncmp(line, "PUMPML:", 7) == 0) {     // PUMPML:n:ml  (utilise PUMP_MS_PER_ML : sert à vérifier la calibration)
    int p = 0; long ml = 0;
    if (sscanf(line + 7, "%d:%ld", &p, &ml) == 2 && p >= 1 && p <= 4 && ml > 0 && ml <= 300) {
      working = true; abortFlag = false; testMove = true;
      pumpFor(p - 1, (unsigned long)ml * PUMP_MS_PER_ML[p - 1]);
      testMove = false; working = false; abortFlag = false;
      reply("OK:PUMPML");
    } else reply("ERR:PUMPML_ARGS (PUMPML:n:ml, n=1..4, ml<=300)");
    return;
  }

  // --- Tests du servo (doseur d'alcool) ---
  if (strncmp(line, "SERVO:", 6) == 0) {      // SERVO:angle (0..180) : pour régler SERVO_REST / SERVO_PUSH
    int d = atoi(line + 6);
    if (d >= 0 && d <= 180) {
      working = true; abortFlag = false; testMove = true;
      doser.attach(PIN_SERVO);
      doser.write(d);
      waitMs(1000);
      doser.detach();
      testMove = false; working = false; abortFlag = false;
      char b[24];
      snprintf(b, sizeof(b), "OK:SERVO %d", d);
      reply(b);
    } else reply("ERR:SERVO_ARGS (0..180)");
    return;
  }

  if (strncmp(line, "DOSE:", 5) == 0) {       // DOSE:n : n doses à l'endroit où se trouve le chariot
    int n = atoi(line + 5);
    if (n >= 1 && n <= 5) {
      working = true; abortFlag = false; testMove = true;
      bool ok = dispenseAlcohol(n);
      testMove = false; working = false; abortFlag = false;
      reply(ok ? "OK:DOSE" : "OK:STOP");
    } else reply("ERR:DOSE_ARGS (1..5)");
    return;
  }

  // Cocktail : on prend le premier nombre trouvé (1, C3, COCKTAIL:12...)
  for (char* c = line; *c; c++) {
    if (*c >= '0' && *c <= '9') { makeCocktail((uint8_t)atoi(c)); return; }
  }
  reply("ERR:UNKNOWN_CMD");
}

// ====================== SETUP / LOOP ======================
void setup() {
  // Relais OFF avant de passer les pins en sortie (évite un déclenchement au boot)
  for (uint8_t i = 0; i < 4; i++) { digitalWrite(PIN_PUMP[i], RELAY_OFF); pinMode(PIN_PUMP[i], OUTPUT); }

  pinMode(PIN_ENABLE, OUTPUT);
  digitalWrite(PIN_ENABLE, LOW);          // active les drivers
  pinMode(PIN_ENDSTOP, INPUT_PULLUP);

  Serial.begin(115200);
  Serial1.begin(9600);

  strip.begin();
  strip.setBrightness(LED_BRIGHTNESS);
  strip.show();

  stepper.setPinsInverted(INVERT_DIR, false, false);
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(ACCELERATION);

  // Laisse le détecteur se stabiliser (LED blanc clignotant / rouge selon le verre)
  unsigned long t = millis();
  while (millis() - t < 400) { updateGlass(); updateLeds(); }

  if (AUTO_HOME_AT_BOOT) homeAxis();      // homing automatique (désactivé : on tape HOME dans le moniteur série)
  reply("READY (chariot immobile, tape HOME)");
}

void loop() {
  service();
  if (state == DONE && !glass) state = IDLE;   // verre retiré -> retour à l'état normal
}
