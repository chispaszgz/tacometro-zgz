// =====================================================================
//  TACOMETRO VMP  -  version para ESP32-C6 SuperMini (MakerGO)
// =====================================================================
//  Diferencias frente a la version original de ESP32 clasico:
//
//  1) BLUETOOTH. El C6 NO tiene Bluetooth Clasico (solo BLE 5.0), asi que
//     BluetoothSerial/SPP no existe. La impresora termica se ataca ahora por
//     BLE escribiendo los mismos bytes ESC/POS en una caracteristica GATT.
//     La clase ImpresoraBLE hereda de Print, de modo que el codigo de
//     impresion sigue usando write()/print()/println() sin cambios.
//  2) UART DEL GPS. En el C6 el "Serial2" del core es el LP-UART, con pines
//     fijos. El GPS pasa a Serial1, que si admite pines libres.
//  3) PINES. Los GPIO 32/27/23/16/17/2/15 del ESP32 clasico no existen o no
//     son utilizables en el C6; el mapa se ha rehecho evitando los pines de
//     strapping (4, 5, 8, 9) y los de USB (12, 13). GPIO15 si se usa, pero
//     solo como salida de LED: su condicion de strapping se lee al arrancar,
//     cuando el pin aun es entrada, asi que no interfiere.
//
//  IDE  -> Placa: "MakerGO ESP32 C6 SuperMini"
//       -> USB CDC On Boot: Enabled   (para ver el Monitor Serie por USB)
//       -> Partition Scheme: el que ofrezca >= 1,3 MB de app (BLE ocupa)
// =====================================================================

#include <Arduino.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <TinyGPSPlus.h>
#include <U8g2lib.h>
#include <Wire.h>

// Pila BLE (sustituye a BluetoothSerial.h)
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLESecurity.h>
#include <esp_mac.h>

#if !defined(CONFIG_IDF_TARGET_ESP32C6)
#error "Este sketch es para ESP32-C6. Selecciona 'MakerGO ESP32 C6 SuperMini' en Herramientas > Placa."
#endif

// Declarado aqui arriba a proposito: el preprocesador de Arduino vuelca todos
// los prototipos justo despues de los #include, y nombreTipoImpresora() lo usa
// en su firma. Si el enum se declarase mas abajo, el prototipo no compilaria.
enum TipoImpresora : uint8_t { IMP_BLUETOOTH = 0, IMP_CABLE = 1 };

// --- DISPLAY ---
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// =====================================================================
//  MAPA DE PINES DEL ESP32-C6 SUPERMINI
// =====================================================================
//  Libres tras este reparto: 16(TX), 17(RX), 21, 22, 23
//  De la placa en uso:       8 (LED RGB), 15 (LED integrado)
//  Reservados:               9 (BOOT), 12/13 (USB)
//  GPIO16/17 se dejan libres aposta: por ahi sale el log de arranque de la ROM.
// ---------------------------------------------------------------------
// I2C del OLED: coinciden con los pines por defecto de la variante, asi
// que U8g2 los usa aunque vuelva a llamar internamente a Wire.begin().
#define PIN_SDA 20
#define PIN_SCL 19

// --- GPS (Serial1) ---
// Al modulo GPS no se le envia ninguna orden, asi que su RX no se cablea y el
// TX del UART se deja sin asignar (-1). El core solo pone pines por defecto si
// RX y TX son negativos a la vez, de modo que GPIO7 queda realmente libre.
#define GPS_RX  6      // entra el TX del modulo GPS
#define GPS_TX  -1     // sin usar: libera GPIO7
#define GPS_BAUD 9600
#define SerialGPS Serial1

TinyGPSPlus gps;

double gpsLat  = 0.0;
double gpsLon  = 0.0;
uint8_t gpsHora = 0;
uint8_t gpsMin  = 0;
uint8_t gpsSeg  = 0;
uint8_t  gpsDia  = 0;
uint8_t  gpsMes  = 0;
uint16_t gpsAnio = 0;
bool    gpsFix  = false;

// --- HORA LOCAL (España) ---
int localDia = 0, localMes = 0, localAnio = 0;
int localHora = 0, localMin = 0, localSeg = 0;
int localDiaSemana = 0;  // 0=Domingo ... 6=Sabado

// --- PINES ---
const byte pinSensor   = 1;    // sensor de pulsos del rodillo (interrupcion)
const byte pinEmpezar  = 2;    // boton rojo
const byte pinImprimir = 3;    // boton azul
const byte pinMotor     = 0;   // puerta del MOSFET del motor de arranque (PWM)
const byte pinLedVerde  = 18;
const byte pinLedRojo   = 7;   // antes GPS_TX, libre al no cablear el RX del GPS
const byte pinLedAzul   = 14;
const byte pinLedSensor = 15;  // LED integrado: destella con cada pulso del sensor

// Si el LED de la placa luciera al reves (encendido en reposo), poner a 1.
#define LED_PLACA_INVERTIDO 0

inline void ledPlaca(bool encendido) {
  digitalWrite(pinLedSensor, LED_PLACA_INVERTIDO ? !encendido : encendido);
}

const float DIAMETRO_RODILLO = 6.0;

// --- FILTRO ANTIPICOS DEL SENSOR ---
// Un rebote del sensor o un pulso de ruido genera un intervalo minusculo y,
// por tanto, una velocidad disparada (miles de km/h). Cualquier lectura por
// encima de esta velocidad se considera espuria y se descarta en la
// interrupcion, de modo que ni siquiera contamina velMaxPico.
const float velMaxPlausible = 90.0;   // km/h: por encima se descarta como ruido
// Intervalo minimo coherente con velMaxPlausible (misma formula que
// leerVelocidad): kmh = (60e6/intervalo) * 3.14159 * D * 60 / 100000.
const unsigned long INTERVALO_MIN_US =
    (unsigned long)((60000000.0 * 3.14159 * DIAMETRO_RODILLO * 60.0 / 100000.0)
                    / velMaxPlausible);

// --- PARAMETROS DE LA PRUEBA ---
const float velocidadComienzo = 6.0;   // km/h: al superarse, se apaga el motor y arranca la prueba
const int   tiempoPrueba      = 30;    // segundos que dura la prueba
const float margenVelMax      = 2.0;   // km/h: margen +- para la velocidad maxima mantenida 5 s
const int   tiempoArranque    = 4;     // segundos de rampa de aceleracion del motor de arranque
const int   tiempoMaxArranque = 15;    // segundos esperando velocidadComienzo; si no se alcanza, vuelve a ESPERA

// --- PWM DEL MOTOR (MOSFET) ---
const int pwmFreqMotor = 20000;  // Hz (>20 kHz: fuera del rango audible, sin pitido)
const int pwmResMotor  = 8;      // bits de resolucion -> duty 0..255
const int pwmMaxMotor  = 255;    // duty maximo (100 %)
const int pwmMinMotor  = 200;    // duty inicial de la rampa (~80 %)

// --- ESTADO ---
// El flujo es: BIENVENIDA -> ESPERA -> ARRANQUE -> MEDICION -> RESULTADO
// Desde ESPERA, el azul abre el arbol de menus:
//   MENU -> DATOS | HISTORICO | PERIFERICOS
// Dentro del arbol los papeles de los botones son:
//   AZUL corto = mover cursor   AZUL largo = entrar/actuar   ROJO = a ESPERA
enum Estado { BIENVENIDA, ESPERA, MENU, DATOS, HISTORICO, PERIFERICOS,
              ARRANQUE, MEDICION, RESULTADO };
Estado estadoActual = BIENVENIDA;

// --- DATOS DE LA PRUEBA ---
unsigned long tiempoInicioArranque = 0;
unsigned long tiempoInicioPrueba = 0;
float velMaxPico = 0;       // pico absoluto alcanzado
float velMaxMantenida = 0;  // mayor velocidad sostenida 5 s (dentro del margen)

#define N_MUESTRAS 10        // 5 s con una mediana cada 500 ms
float muestras[N_MUESTRAS];
int   idxMuestra  = 0;
int   numMuestras = 0;

// --- SUAVIZADO POR MEDIANA (cada 500 ms) ---
// Durante la prueba se sub-muestrea rapido y, cada 500 ms, se toma la mediana
// de esas sub-muestras como valor oficial. La mediana ignora un pico aislado
// (aunque caiga dentro del rango plausible), de modo que la subida es
// progresiva y velMaxPico no registra esos picos.
#define VENTANA_MS       500 // periodo de consolidacion
#define SUB_INTERVALO_MS 50  // cada cuanto se sub-muestrea dentro de la ventana
#define SUB_N            12   // capacidad del buffer (VENTANA_MS/SUB_INTERVALO_MS + margen)

// --- HISTORICO COMPLETO PARA LA GRAFICA DEL TICKET ---
#define MAX_PUNTOS 60        // tiempoPrueba (30 s) / 500 ms
float histVel[MAX_PUNTOS];
int   nPuntos = 0;

// Bitmap monocromo de la grafica (1 bit = 1 px). Ancho multiplo de 8.
#define G_W 384              // ancho en px (todo el papel de 58 mm)
#define G_H 200              // alto en px
#define G_BYTES (G_W / 8)    // bytes por fila
uint8_t bmp[G_BYTES * G_H];  // 9600 bytes

// Tamano del buffer de la carga del QR de datos (ver construirCargaQR)
#define TAM_CARGA_QR 320

volatile unsigned long intervalo = 0;
volatile unsigned long tiempoUltimoPulso = 0;

// =====================================================================
//  AVISOS LUMINOSOS
// =====================================================================
// --- LED RGB DE LA PLACA (WS2812 en GPIO8): un color por pantalla ---
// Brillo bajo aposta: a plena potencia este LED deslumbra.
#define BRILLO_RGB 24

struct ColorRGB { uint8_t r, g, b; };
static const ColorRGB COLOR_ESTADO[] = {
  {BRILLO_RGB, BRILLO_RGB, BRILLO_RGB},   // BIENVENIDA:  blanco
  {0,          BRILLO_RGB, 0},            // ESPERA:      verde
  {0,          BRILLO_RGB, BRILLO_RGB},   // MENU:        cian
  {BRILLO_RGB / 2, 0,      BRILLO_RGB},   // DATOS:       violeta
  {BRILLO_RGB, BRILLO_RGB, 0},            // HISTORICO:   amarillo
  {BRILLO_RGB, 0,          BRILLO_RGB},   // PERIFERICOS: magenta
  {BRILLO_RGB, BRILLO_RGB / 3, 0},        // ARRANQUE:    naranja
  {BRILLO_RGB, 0,          0},            // MEDICION:    rojo
  {0,          0,          BRILLO_RGB},   // RESULTADO:   azul
};
static_assert(sizeof(COLOR_ESTADO) / sizeof(COLOR_ESTADO[0]) == RESULTADO + 1,
              "Falta un color: la tabla debe cubrir todos los estados");

// Solo se escribe al cambiar de pantalla: el protocolo del WS2812 es sensible
// al tiempo y no conviene repetirlo en cada vuelta del loop.
void actualizarLedRGB() {
  static int ultimo = -1;
  if ((int)estadoActual == ultimo) return;
  ultimo = (int)estadoActual;
  const ColorRGB& c = COLOR_ESTADO[estadoActual];
  rgbLedWrite(PIN_RGB_LED, c.r, c.g, c.b);
}

// --- DESTELLO DEL LED INTEGRADO CON CADA PULSO DEL SENSOR ---
// No se toca el LED dentro de la interrupcion: basta con vigilar la marca de
// tiempo del ultimo pulso. Por encima de unas 2000 rpm los pulsos se solapan y
// el LED se ve fijo, que es justo la indicacion util de "rodillo girando".
#define DESTELLO_SENSOR_MS 30

void actualizarLedSensor() {
  static unsigned long ultimoVisto = 0;
  static unsigned long apagarEn    = 0;

  noInterrupts();
  unsigned long t = tiempoUltimoPulso;
  interrupts();

  if (t != ultimoVisto) {              // ha entrado un pulso nuevo
    ultimoVisto = t;
    ledPlaca(true);
    apagarEn = millis() + DESTELLO_SENSOR_MS;
  } else if (apagarEn != 0 && (long)(millis() - apagarEn) >= 0) {
    ledPlaca(false);
    apagarEn = 0;
  }
}

// =====================================================================
//  IMPRESORA TERMICA POR BLE
// =====================================================================
// Sustituto de BluetoothSerial. Al heredar de Print se conservan write(),
// print() y println(), asi que las funciones imprimir(), imprimirQR() e
// imprimirGrafica() son identicas a las del sketch original salvo el nombre
// del objeto (SerialBT -> impresora).

// Diagnostico por el Monitor Serie al arrancar. Se puede forzar desde las
// opciones de compilacion (-DDIAGNOSTICO_BLE=2) sin tocar este archivo.
//   0 = desactivado (funcionamiento normal)
//   1 = lista los dispositivos BLE al alcance
//   2 = ademas conecta con la impresora y vuelca su arbol de servicios GATT
//   3 = ademas imprime un ticket de prueba con datos ficticios
#ifndef DIAGNOSTICO_BLE
#define DIAGNOSTICO_BLE 0
#endif

// Nombre anunciado por la impresora; basta con que aparezca como subcadena.
const char* NOMBRE_IMPRESORA = "MP210";

// Direccion BLE de la impresora. Si se deja vacia se busca solo por nombre y
// por perfil de servicio. Verificada por rastreo el 03/08/2026: en esta MP210
// la direccion BLE coincide con la MAC de Bluetooth Clasico que usaba el
// sketch antiguo. Si cambias de impresora, averiguala con DIAGNOSTICO_BLE = 1.
const char* MAC_IMPRESORA = "dc:0d:51:0f:f2:df";

// Perfiles GATT habituales en impresoras ESC/POS de 58 mm. Se busca cualquiera
// de ellos y, si no aparece ninguno, se usa la primera caracteristica que
// admita escritura.
struct PerfilImpresora { const char* servicio; const char* escritura; };
static const PerfilImpresora PERFILES[] = {
  {"000018f0-0000-1000-8000-00805f9b34fb", "00002af1-0000-1000-8000-00805f9b34fb"},
  {"0000ff00-0000-1000-8000-00805f9b34fb", "0000ff02-0000-1000-8000-00805f9b34fb"},
  {"0000ae30-0000-1000-8000-00805f9b34fb", "0000ae01-0000-1000-8000-00805f9b34fb"},
  {"49535343-fe7d-4ae5-8fa9-9fafd205e455", "49535343-8841-43f4-a8d4-ecbe34729bb3"},
  {"0000ffe0-0000-1000-8000-00805f9b34fb", "0000ffe1-0000-1000-8000-00805f9b34fb"},
};
static const int N_PERFILES = sizeof(PERFILES) / sizeof(PERFILES[0]);

// Interfaz comun a las dos impresoras. El codigo de impresion trabaja siempre
// contra este tipo, asi que cambiar de modelo no le afecta.
class Impresora : public Print {
public:
  virtual bool conectar()  = 0;   // deja la impresora lista para recibir bytes
  virtual bool conectado() = 0;
};

class ImpresoraBLE : public Impresora {
public:
  void begin();
  bool conectar() override { return conectarCon(12000); }
  bool conectar(uint32_t timeoutMs) { return conectarCon(timeoutMs); }
  bool conectado() override;
  void desconectar();

  size_t write(uint8_t b) override;
  size_t write(const uint8_t* datos, size_t len) override;
  void   flush() override;

private:
  bool conectarCon(uint32_t timeoutMs);
  BLEClient*               _cli = nullptr;
  BLERemoteCharacteristic* _chr = nullptr;
  bool     _conRespuesta = true;   // escritura con ACK: sirve de control de flujo
  uint16_t _tam = 20;              // bytes utiles por escritura GATT
  uint8_t  _buf[244];
  size_t   _n = 0;

  void enviarBloque();
  bool localizarCaracteristica();
};

// --- IMPRESORA POR CABLE (TTL serie) ---
// Modulo termico de 58 mm con interfaz USB/TTL/RS232; se usa la TTL, que es
// UART pura. El C6 solo tiene dos UART de alta velocidad: la 1 la ocupa el GPS,
// asi que la impresora va en la 0 (Serial0) reasignada a GPIO4/GPIO5.
// No se usan GPIO16/17 aposta: son los pines por defecto de UART0 y por ahi
// sale el log de arranque de la ROM, que la impresora imprimiria como basura.
#define IMP_TX      4        // ESP32 TX -> RX de la impresora
#define IMP_RX      5        // ESP32 RX <- TX de la impresora (opcional)
#define IMP_BAUD    9600     // ver la hoja de autotest de la impresora
#define IMP_BLOQUE  64       // bytes por rafaga
#define IMP_PAUSA   0        // ms entre rafagas; subir si pierde datos

class ImpresoraSerie : public Impresora {
public:
  void begin();
  bool conectar()  override { return true; }   // no hay enlace que negociar
  bool conectado() override { return true; }

  size_t write(uint8_t b) override;
  size_t write(const uint8_t* datos, size_t len) override;
  void   flush() override;
};

ImpresoraBLE   impresoraBLE;
ImpresoraSerie impresoraSerie;

// --- SELECCION DE IMPRESORA (pantalla de opciones, se recuerda en la NVS) ---
// El enum TipoImpresora esta declarado arriba del todo, junto a los #include.
TipoImpresora tipoImpresora = IMP_CABLE;
Impresora*    impresora     = &impresoraBLE;
Preferences   prefs;

void aplicarTipoImpresora() {
  impresora = (tipoImpresora == IMP_CABLE) ? (Impresora*)&impresoraSerie
                                           : (Impresora*)&impresoraBLE;
}

const char* nombreTipoImpresora(TipoImpresora t) {
  return (t == IMP_CABLE) ? "Cable (serie TTL)" : "Bluetooth (BLE)";
}

// --- AGENTE QUE REALIZA LA PRUEBA (se teclea en OPCIONES, sale en el ticket) ---
#define MAX_AGENTE 16
#define MAX_NOTA   22
char nombreAgente[MAX_AGENTE + 1] = "";
char notaPrueba[MAX_NOTA + 1]     = "";

// --- IDENTIDAD DEL APARATO ---
// FW_VERSION la cambias tu en cada version publicada; PROTO_VERSION solo
// cuando el formato de las respuestas JSON deje de ser compatible, para que
// la webapp pueda avisar en vez de fallar de forma rara.
#define FW_VERSION    "1.2.0"
#define PROTO_VERSION 1
#define MAX_ID_DISP   20
char idDispositivo[MAX_ID_DISP + 1] = "";

// Almacen de pruebas en LittleFS (ver guardarPrueba(), mas abajo)
bool fsListo = false;

// Solo se escribe en la NVS si el valor ha cambiado, para no gastar la flash.
void guardarOpciones() {
  if (prefs.getUChar("impresora", 0xFF) != (uint8_t)tipoImpresora) {
    prefs.putUChar("impresora", (uint8_t)tipoImpresora);
  }
  if (prefs.getString("agente", "") != String(nombreAgente)) {
    prefs.putString("agente", nombreAgente);
  }
  if (prefs.getString("nota", "") != String(notaPrueba)) {
    prefs.putString("nota", notaPrueba);
  }
}

void ImpresoraSerie::begin() {
  Serial0.begin(IMP_BAUD, SERIAL_8N1, IMP_RX, IMP_TX);
}

size_t ImpresoraSerie::write(uint8_t b) {
  return Serial0.write(b);
}

// Se trocea el envio y se espera a que cada rafaga salga por el cable: asi el
// buffer de la impresora no se desborda al mandar el bitmap de la grafica.
size_t ImpresoraSerie::write(const uint8_t* datos, size_t len) {
  size_t restan = len;
  while (restan > 0) {
    size_t n = (restan > IMP_BLOQUE) ? IMP_BLOQUE : restan;
    Serial0.write(datos, n);
    Serial0.flush();
    datos  += n;
    restan -= n;
#if IMP_PAUSA > 0
    delay(IMP_PAUSA);
#endif
  }
  return len;
}

void ImpresoraSerie::flush() {
  Serial0.flush();
}

void ImpresoraBLE::begin() {
  BLEDevice::init("TacometroVMP");
  BLEDevice::setMTU(247);          // hay que pedirlo antes de conectar
  _cli = BLEDevice::createClient();
}

bool ImpresoraBLE::conectado() {
  return _cli != nullptr && _cli->isConnected() && _chr != nullptr;
}

void ImpresoraBLE::desconectar() {
  if (_cli && _cli->isConnected()) _cli->disconnect();
  _chr = nullptr;
  _n = 0;
}

// Busca la impresora, se conecta y localiza la caracteristica de escritura.
bool ImpresoraBLE::conectarCon(uint32_t timeoutMs) {
  if (conectado()) return true;
  if (_cli == nullptr) return false;
  _chr = nullptr;
  _n = 0;

  // --- Rastreo ---
  BLEScan* scan = BLEDevice::getScan();
  scan->setActiveScan(true);       // pide el nombre completo en el scan response
  scan->setInterval(100);
  scan->setWindow(99);

  uint32_t segundos = timeoutMs / 1000;
  if (segundos < 3) segundos = 3;
  BLEScanResults* res = scan->start(segundos, false);

  bool encontrada = false;
  BLEAddress dir;
  uint8_t tipoDir = 0;

  for (int i = 0; res != nullptr && i < res->getCount(); i++) {
    BLEAdvertisedDevice d = res->getDevice(i);
    String nombre = d.getName();
    String mac    = d.getAddress().toString();
    bool coincide = false;

    if (strlen(MAC_IMPRESORA) > 0) {
      coincide = mac.equalsIgnoreCase(MAC_IMPRESORA);
    } else {
      if (nombre.length() > 0 && strlen(NOMBRE_IMPRESORA) > 0 &&
          nombre.indexOf(NOMBRE_IMPRESORA) >= 0) {
        coincide = true;
      }
      for (int k = 0; !coincide && k < N_PERFILES; k++) {
        if (d.isAdvertisingService(BLEUUID(String(PERFILES[k].servicio)))) coincide = true;
      }
    }

    if (coincide) {
      dir        = d.getAddress();
      tipoDir    = d.getAddressType();
      encontrada = true;
      Serial.printf("[BLE] Impresora: %s  (%s)\n", nombre.c_str(), mac.c_str());
      break;
    }
  }
  scan->stop();
  scan->clearResults();

  if (!encontrada) {
    Serial.println("[BLE] No se ha encontrado la impresora");
    return false;
  }

  // --- Conexion ---
  if (!_cli->connect(dir, tipoDir, timeoutMs)) {
    Serial.println("[BLE] Fallo al conectar");
    return false;
  }
  // Intervalo de conexion corto: acelera el envio del bitmap de la grafica.
  _cli->updateConnParams(12, 24, 0, 500);   // 15-30 ms, timeout 5 s
  delay(300);                               // margen para que la impresora se asiente

  if (!localizarCaracteristica()) {
    Serial.println("[BLE] No hay caracteristica de escritura");
    _cli->disconnect();
    return false;
  }

  uint16_t mtu = _cli->getMTU();
  _tam = (mtu > 23) ? (mtu - 3) : 20;       // 3 bytes de cabecera ATT
  if (_tam > sizeof(_buf)) _tam = sizeof(_buf);
  if (_tam < 20) _tam = 20;
  Serial.printf("[BLE] Conectada. MTU=%u, bloque=%u B, %s\n",
                mtu, _tam, _conRespuesta ? "con ACK" : "sin ACK");
  return true;
}

// Recorre el arbol GATT una sola vez y elige la mejor candidata:
//   3 = servicio y caracteristica de un perfil conocido
//   2 = caracteristica de un perfil conocido
//   1 = cualquier caracteristica escribible
bool ImpresoraBLE::localizarCaracteristica() {
  std::map<std::string, BLERemoteService*>* servicios = _cli->getServices();
  if (servicios == nullptr) return false;

  BLERemoteCharacteristic* mejor = nullptr;
  int mejorPunt = 0;

  for (auto& s : *servicios) {
    BLERemoteService* svc = s.second;
    BLEUUID uuidSvc = svc->getUUID();

    int idxPerfil = -1;
    for (int k = 0; k < N_PERFILES; k++) {
      if (uuidSvc.equals(BLEUUID(String(PERFILES[k].servicio)))) { idxPerfil = k; break; }
    }

    std::map<std::string, BLERemoteCharacteristic*>* cars = svc->getCharacteristics();
    if (cars == nullptr) continue;

    for (auto& c : *cars) {
      BLERemoteCharacteristic* chr = c.second;
      if (!chr->canWrite() && !chr->canWriteNoResponse()) continue;

      BLEUUID uuidChr = chr->getUUID();
      int punt = 1;
      if (idxPerfil >= 0 && uuidChr.equals(BLEUUID(String(PERFILES[idxPerfil].escritura)))) {
        punt = 3;
      } else {
        for (int k = 0; k < N_PERFILES; k++) {
          if (uuidChr.equals(BLEUUID(String(PERFILES[k].escritura)))) { punt = 2; break; }
        }
      }
      Serial.printf("[BLE]   svc %s / chr %s  (punt %d)\n",
                    uuidSvc.toString().c_str(), uuidChr.toString().c_str(), punt);
      if (punt > mejorPunt) { mejorPunt = punt; mejor = chr; }
    }
  }

  if (mejor == nullptr) return false;
  _chr = mejor;
  // Con ACK la propia impresora marca el ritmo; sin ACK hay que espaciar.
  _conRespuesta = _chr->canWrite();
  return true;
}

void ImpresoraBLE::enviarBloque() {
  if (_n == 0) return;
  if (_chr != nullptr && _cli != nullptr && _cli->isConnected()) {
    _chr->writeValue(_buf, _n, _conRespuesta);
    if (!_conRespuesta) delay(12);   // sin ACK: dar tiempo al buffer de la impresora
  }
  _n = 0;
}

size_t ImpresoraBLE::write(uint8_t b) {
  _buf[_n++] = b;
  if (_n >= _tam) enviarBloque();
  return 1;
}

size_t ImpresoraBLE::write(const uint8_t* datos, size_t len) {
  size_t restan = len;
  while (restan > 0) {
    size_t hueco = _tam - _n;
    if (hueco > restan) hueco = restan;
    memcpy(_buf + _n, datos, hueco);
    _n     += hueco;
    datos  += hueco;
    restan -= hueco;
    if (_n >= _tam) enviarBloque();
  }
  return len;
}

void ImpresoraBLE::flush() {
  enviarBloque();
  delay(50);   // margen para que la impresora vacie su cola
}

// Lista por el Monitor Serie todo lo que se anuncia por BLE.
void escanearBLE() {
  Serial.println("\n[BLE] Rastreando 8 s...");
  BLEScan* scan = BLEDevice::getScan();
  scan->setActiveScan(true);
  BLEScanResults* res = scan->start(8, false);
  for (int i = 0; res != nullptr && i < res->getCount(); i++) {
    BLEAdvertisedDevice d = res->getDevice(i);
    Serial.printf("  %-24s %s  RSSI %d\n",
                  d.getName().length() ? d.getName().c_str() : "(sin nombre)",
                  d.getAddress().toString().c_str(), d.getRSSI());
  }
  scan->clearResults();
  Serial.println("[BLE] Fin del rastreo\n");
}

// =====================================================================

void iniciarArranque();
void ejecutarArranque();
void iniciarMedida();
void ejecutarMedida();
void ejecutarResultado();
// =====================================================================
//  TECLADO BLE (HID sobre GATT)
// =====================================================================
// Verificado con el M7 el 04/08/2026: admite emparejamiento Just Works con
// bonding y entrega informes de 8 bytes en el formato estandar de teclado:
//   [modificadores][reservado][6 codigos HID]
// No hay evento de "tecla soltada": cada informe es el estado completo, asi
// que las teclas nuevas se detectan comparando con el informe anterior.
//
// Solo se traducen letras, digitos y espacio (siempre en MAYUSCULAS, lo que
// evita tener que seguir el estado de Shift y de Bloq Mayus, que este teclado
// reporta como tecla mantenida). No hay ñ ni acentos: sus codigos dependen de
// la distribucion y el OLED tampoco los tiene en esta fuente.
const char* NOMBRE_TECLADO = "M7";
const char* MAC_TECLADO    = "54:25:54:9b:00:02";
#define UUID_HID_SVC    "00001812-0000-1000-8000-00805f9b34fb"
#define UUID_HID_REPORT "00002a4d-0000-1000-8000-00805f9b34fb"

// Codigos que devuelve leer() ademas de los caracteres imprimibles
#define TEC_NADA     -1
#define TEC_ACEPTAR  '\n'
#define TEC_BORRAR   '\b'
#define TEC_CANCELAR 27

class TecladoBLE {
public:
  bool conectar(uint32_t timeoutMs = 12000);
  void desconectar();
  bool conectado() { return _cli != nullptr && _cli->isConnected(); }
  int  leer();                     // TEC_NADA si no hay nada pendiente
  static void alRecibir(BLERemoteCharacteristic* chr, uint8_t* datos,
                        size_t len, bool);
private:
  BLEClient* _cli = nullptr;
};

TecladoBLE teclado;

// Cola circular entre la tarea BLE (que rellena) y el loop (que consume)
static volatile char  colaTec[16];
static volatile uint8_t colaCab = 0, colaCol = 0;
static uint8_t ultimasTeclas[6] = {0};

static void encolarTecla(char c) {
  uint8_t sig = (colaCab + 1) % sizeof(colaTec);
  if (sig == colaCol) return;            // cola llena: se descarta
  colaTec[colaCab] = c;
  colaCab = sig;
}

// Codigo HID -> caracter. Siempre mayusculas.
static char traducirHID(uint8_t k) {
  if (k >= 0x04 && k <= 0x1D) return 'A' + (k - 0x04);   // A..Z
  if (k >= 0x1E && k <= 0x26) return '1' + (k - 0x1E);   // 1..9
  if (k == 0x27) return '0';
  if (k == 0x2C) return ' ';
  if (k == 0x28) return TEC_ACEPTAR;                      // Intro
  if (k == 0x2A) return TEC_BORRAR;                       // Retroceso
  if (k == 0x29) return TEC_CANCELAR;                     // Esc
  return 0;                                               // resto: se ignora
}

void TecladoBLE::alRecibir(BLERemoteCharacteristic*, uint8_t* d, size_t len, bool) {
  if (len < 8) return;
  for (int i = 2; i < 8; i++) {
    uint8_t k = d[i];
    if (k == 0) continue;
    bool yaEstaba = false;
    for (int j = 0; j < 6; j++) if (ultimasTeclas[j] == k) yaEstaba = true;
    if (!yaEstaba) {
      char c = traducirHID(k);
      if (c) encolarTecla(c);
    }
  }
  memcpy(ultimasTeclas, d + 2, 6);
}

int TecladoBLE::leer() {
  if (colaCol == colaCab) return TEC_NADA;
  char c = colaTec[colaCol];
  colaCol = (colaCol + 1) % sizeof(colaTec);
  return (int)c;
}

bool TecladoBLE::conectar(uint32_t timeoutMs) {
  if (conectado()) return true;

  // Just Works con bonding: sin cifrado el teclado no envia ninguna tecla
  BLESecurity::setCapability(ESP_IO_CAP_NONE);
  BLESecurity::setAuthenticationMode(true, false, true);

  BLEScan* scan = BLEDevice::getScan();
  scan->setActiveScan(true);
  BLEScanResults* res = scan->start(timeoutMs / 1000, false);

  bool hallado = false;
  BLEAddress dir;
  uint8_t tipo = 0;
  for (int i = 0; res != nullptr && i < res->getCount(); i++) {
    BLEAdvertisedDevice d = res->getDevice(i);
    if (d.getAddress().toString().equalsIgnoreCase(MAC_TECLADO) ||
        d.getName().indexOf(NOMBRE_TECLADO) >= 0) {
      dir = d.getAddress(); tipo = d.getAddressType(); hallado = true;
      break;
    }
  }
  scan->stop();
  scan->clearResults();
  if (!hallado) return false;

  if (_cli == nullptr) _cli = BLEDevice::createClient();
  if (!_cli->connect(dir, tipo, timeoutMs)) return false;
  if (!_cli->secureConnection()) { _cli->disconnect(); return false; }

  BLERemoteService* svc = _cli->getService(BLEUUID(String(UUID_HID_SVC)));
  if (svc == nullptr) { _cli->disconnect(); return false; }

  // Suscribirse a todos los informes de entrada del teclado
  int n = 0;
  std::map<std::string, BLERemoteCharacteristic*>* cars = svc->getCharacteristics();
  if (cars != nullptr) {
    for (auto& c : *cars) {
      if (c.second->getUUID().equals(BLEUUID(String(UUID_HID_REPORT))) &&
          c.second->canNotify()) {
        c.second->registerForNotify(TecladoBLE::alRecibir);
        n++;
      }
    }
  }
  if (n == 0) { _cli->disconnect(); return false; }

  memset(ultimasTeclas, 0, sizeof(ultimasTeclas));
  colaCab = colaCol = 0;
  return true;
}

void TecladoBLE::desconectar() {
  if (_cli && _cli->isConnected()) _cli->disconnect();
}

// =====================================================================

void imprimir();
void imprimirQR(const char* datos, uint8_t modulo, uint8_t nivelEC);
void imprimirGrafica();
#if DIAGNOSTICO_BLE >= 3
void pruebaImpresion();
#endif

void IRAM_ATTR detectarPulso() {
  unsigned long tiempoActual = micros();
  unsigned long delta = tiempoActual - tiempoUltimoPulso;
  if (delta < INTERVALO_MIN_US) return;   // pulso espurio (rebote/ruido): se ignora
  intervalo = delta;
  tiempoUltimoPulso = tiempoActual;
}

void actualizarGPS() {
  while (SerialGPS.available()) {
    if (gps.encode(SerialGPS.read())) {
      if (gps.location.isValid() && gps.location.isUpdated()) {
        gpsLat = gps.location.lat();
        gpsLon = gps.location.lng();
        gpsFix = true;
      }
      if (gps.time.isValid() && gps.time.isUpdated()) {
        gpsHora = gps.time.hour();
        gpsMin  = gps.time.minute();
        gpsSeg  = gps.time.second();
      }
      if (gps.date.isValid() && gps.date.isUpdated()) {
        gpsDia  = gps.date.day();
        gpsMes  = gps.date.month();
        gpsAnio = gps.date.year();
      }
    }
  }
  // Si hace mas de 5 s que no llega una posicion valida, se pierde el fix
  if (gps.location.age() > 5000) gpsFix = false;
}

// --- CONVERSION DE FECHA <-> DIAS DESDE EPOCA (algoritmo de H. Hinnant) ---
// Permite sumar el desfase horario gestionando automaticamente los cambios
// de dia, mes y anio.
long diasCivil(int y, int m, int d) {
  y -= m <= 2;
  long era = (y >= 0 ? y : y - 399) / 400;
  long yoe = y - era * 400;
  long doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  long doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + doe - 719468;
}

void civilDias(long z, int &y, int &m, int &d) {
  z += 719468;
  long era = (z >= 0 ? z : z - 146096) / 146097;
  long doe = z - era * 146097;
  long yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  y = (int)(yoe + era * 400);
  long doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  long mp = (5 * doy + 2) / 153;
  d = (int)(doy - (153 * mp + 2) / 5 + 1);
  m = (int)(mp + (mp < 10 ? 3 : -9));
  y += (m <= 2);
}

int diaSemana(int y, int m, int d) {  // 0=Domingo ... 6=Sabado
  long z = diasCivil(y, m, d);
  int w = (int)(((z % 7) + 4) % 7);
  return w < 0 ? w + 7 : w;
}

// Horario de verano en la UE: del ultimo domingo de marzo al ultimo
// domingo de octubre (cambios a las 01:00 UTC).
bool esHorarioVerano(int y, int mes, int dia, int horaUTC) {
  if (mes < 3 || mes > 10) return false;
  if (mes > 3 && mes < 10) return true;
  int ultimoDom = 31 - diaSemana(y, mes, 31);  // ultimo domingo del mes
  if (mes == 3) {
    if (dia != ultimoDom) return dia > ultimoDom;
    return horaUTC >= 1;
  } else {  // octubre
    if (dia != ultimoDom) return dia < ultimoDom;
    return horaUTC < 1;
  }
}

// Convierte la fecha/hora UTC del GPS a la hora oficial de España.
void calcularHoraLocal() {
  long dias = diasCivil(gpsAnio, gpsMes, gpsDia);
  long unix = dias * 86400L + gpsHora * 3600L + gpsMin * 60L + gpsSeg;
  int offset = esHorarioVerano(gpsAnio, gpsMes, gpsDia, gpsHora) ? 2 : 1;
  unix += offset * 3600L;

  long z = unix / 86400L;
  long resto = unix % 86400L;
  localHora = (int)(resto / 3600);
  localMin  = (int)((resto % 3600) / 60);
  localSeg  = (int)(resto % 60);
  civilDias(z, localAnio, localMes, localDia);
  localDiaSemana = (int)(((z % 7) + 4) % 7);
  if (localDiaSemana < 0) localDiaSemana += 7;
}

const char* nombreDiaSemana(int d) {
  static const char* dias[7] = {"Domingo", "Lunes", "Martes", "Miercoles",
                                "Jueves", "Viernes", "Sabado"};
  return (d >= 0 && d <= 6) ? dias[d] : "";
}

void mostrarEspera() {
  static unsigned long ultimaActualizacion = 0;
  if (millis() - ultimaActualizacion < 250) return;   // refresco cada 250 ms
  ultimaActualizacion = millis();

  u8g2.clearBuffer();
  char buf[40], lat[12], lon[12];

  // --- GPS ---
  u8g2.setFont(u8g2_font_5x7_tf);
  if (gpsFix) {
    dtostrf(gpsLat, 0, 5, lat);
    dtostrf(gpsLon, 0, 5, lon);
    snprintf(buf, sizeof(buf), "GPS: %s , %s", lat, lon);
  } else {
    snprintf(buf, sizeof(buf), "GPS: buscando (%d sat)",
             (int)gps.satellites.value());
  }
  u8g2.drawStr(0, 9, buf);

  // --- FECHA y HORA ---
  // "FECHA: Miercoles 23/02/26" son 25 caracteres: el maximo que entra en 5x7.
  if (gpsFix) {
    calcularHoraLocal();
    snprintf(buf, sizeof(buf), "FECHA: %s %02d/%02d/%02d",
             nombreDiaSemana(localDiaSemana), localDia, localMes, localAnio % 100);
    u8g2.drawStr(0, 21, buf);
    snprintf(buf, sizeof(buf), "HORA:  %02d:%02d:%02d", localHora, localMin, localSeg);
    u8g2.drawStr(0, 34, buf);
  } else {
    u8g2.drawStr(0, 21, "FECHA: --/--/--");
    u8g2.drawStr(0, 34, "HORA:  --:--:--");
  }

  u8g2.drawFrame(0, 44, 128, 1);
  const char* pie = "VERDE=PRUEBA  AZUL=HIST";
  u8g2.drawStr((128 - u8g2.getStrWidth(pie)) / 2, 56, pie);

  u8g2.sendBuffer();
}

// =====================================================================
//  ARBOL DE MENUS
// =====================================================================
//  ESPERA --azul--> MENU --> DATOS | HISTORICO | PERIFERICOS
//
//  Con solo dos botones y el rojo reservado para volver al principio, el
//  azul tiene que hacer doble papel: pulsacion corta mueve el cursor y
//  pulsacion larga entra o actua. La larga se emite sin esperar a soltar,
//  para que la respuesta sea inmediata.
#define PULSACION_LARGA_MS 700
#define PULSA_NADA  0
#define PULSA_CORTA 1
#define PULSA_LARGA 2

int leerAzul() {
  static bool estaba = false, largaEmitida = false;
  static unsigned long t0 = 0;
  bool ahora = (digitalRead(pinImprimir) == LOW);

  if (ahora && !estaba) { estaba = true; largaEmitida = false; t0 = millis(); }
  else if (ahora && !largaEmitida && millis() - t0 >= PULSACION_LARGA_MS) {
    largaEmitida = true;
    return PULSA_LARGA;
  }
  else if (!ahora && estaba) {
    estaba = false;
    if (!largaEmitida) return PULSA_CORTA;
  }
  return PULSA_NADA;
}

bool rojoPulsado() {
  static bool estaba = false;
  bool ahora = (digitalRead(pinEmpezar) == LOW);
  if (ahora && !estaba) { estaba = true; return true; }
  if (!ahora) estaba = false;
  return false;
}

// Evita que el boton que acaba de cerrar una pantalla dispare la siguiente
void esperarSoltar(byte pin) {
  while (digitalRead(pin) == LOW) delay(10);
  delay(40);
}

// --- Estado de navegacion ---
int menuCursor  = 0;      // 0=Datos 1=Historico 2=Perifericos
int datosCursor = 0;      // 0=Agente 1=Nota
int histCursor  = 0;
int histDetalle = -1;     // -1 = lista; si no, indice de la prueba abierta

// Campo de texto en edicion (apunta a nombreAgente o a notaPrueba)
char*       editCampo  = nullptr;
size_t      editMax    = 0;
const char* editTitulo = "";

void volverAEspera() {
  guardarOpciones();
  digitalWrite(pinLedRojo, 0);
  digitalWrite(pinLedAzul, 0);
  editCampo = nullptr;
  histDetalle = -1;
  teclado.desconectar();
  estadoActual = ESPERA;
  esperarSoltar(pinEmpezar);
}

// ---------------------------------------------------------------- MENU
void mostrarMenu() {
  static unsigned long ult = 0;
  if (millis() - ult < 200) return;
  ult = millis();

  static const char* ENTRADAS[3] = {"Datos", "Historico", "Perifericos"};

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr((128 - u8g2.getStrWidth("MENU")) / 2, 9, "MENU");
  u8g2.drawFrame(0, 11, 128, 1);

  u8g2.setFont(u8g2_font_6x10_tf);
  for (int i = 0; i < 3; i++) {
    int y = 24 + i * 11;
    if (i == menuCursor) {
      u8g2.drawBox(0, y - 8, 128, 10);
      u8g2.setDrawColor(0);
    }
    u8g2.drawStr(6, y, ENTRADAS[i]);
    u8g2.setDrawColor(1);
  }

  u8g2.setFont(u8g2_font_4x6_tf);
  const char* pie = "AZUL corto=mover largo=entrar";
  u8g2.drawStr((128 - u8g2.getStrWidth(pie)) / 2, 63, pie);
  u8g2.sendBuffer();
}

// --------------------------------------------------------- EDICION TEXTO
void mostrarEdicionTexto() {
  static unsigned long ult = 0;
  if (millis() - ult < 200) return;
  ult = millis();

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr((128 - u8g2.getStrWidth(editTitulo)) / 2, 9, editTitulo);
  u8g2.drawFrame(0, 11, 128, 1);

  u8g2.setFont(u8g2_font_5x7_tf);
  if (!teclado.conectado()) {
    u8g2.drawStr(0, 26, "Teclado M7 sin conectar.");
    u8g2.drawStr(0, 36, "Pulsa una tecla del M7 y");
    u8g2.drawStr(0, 46, "luego AZUL para enlazar.");
  } else {
    char linea[40];
    snprintf(linea, sizeof(linea), "%s%s", editCampo,
             ((millis() / 400) % 2) ? "_" : "");
    u8g2.drawStr(2, 30, linea);
    u8g2.drawFrame(0, 33, 128, 1);
    u8g2.drawStr(0, 45, "Escribe en el teclado");
  }

  u8g2.setFont(u8g2_font_4x6_tf);
  const char* pie = "INTRO=OK  ESC=CANCELAR  ROJO=SALIR";
  u8g2.drawStr((128 - u8g2.getStrWidth(pie)) / 2, 63, pie);
  u8g2.sendBuffer();
}

void procesarTeclado() {
  int c;
  while ((c = teclado.leer()) != TEC_NADA) {
    if (editCampo == nullptr) return;
    size_t n = strlen(editCampo);
    if (c == TEC_ACEPTAR || c == TEC_CANCELAR) {
      if (c == TEC_CANCELAR) {
        // Esc restaura lo ultimo guardado
        String previo = prefs.getString(editCampo == nombreAgente ? "agente" : "nota", "");
        previo.toCharArray(editCampo, editMax + 1);
      }
      editCampo = nullptr;
      teclado.desconectar();
    } else if (c == TEC_BORRAR) {
      if (n > 0) editCampo[n - 1] = '\0';
    } else if (n < editMax) {
      editCampo[n]     = (char)c;
      editCampo[n + 1] = '\0';
    }
  }
}

void iniciarEdicion(char* campo, size_t maxLen, const char* titulo) {
  editCampo  = campo;
  editMax    = maxLen;
  editTitulo = titulo;
  mostrarEdicionTexto();
  teclado.conectar();     // el enlace solo vive mientras se edita
}

// --------------------------------------------------------------- DATOS
void mostrarDatos() {
  if (editCampo) { mostrarEdicionTexto(); return; }

  static unsigned long ult = 0;
  if (millis() - ult < 250) return;
  ult = millis();

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr((128 - u8g2.getStrWidth("DATOS")) / 2, 9, "DATOS");
  u8g2.drawFrame(0, 11, 128, 1);

  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawStr(0, 24, datosCursor == 0 ? ">" : " ");
  u8g2.drawStr(8, 24, "Agente:");
  u8g2.drawStr(8, 33, nombreAgente[0] ? nombreAgente : "(vacio)");

  u8g2.drawStr(0, 47, datosCursor == 1 ? ">" : " ");
  u8g2.drawStr(8, 47, "Nota:");
  u8g2.drawStr(8, 56, notaPrueba[0] ? notaPrueba : "(vacio)");

  u8g2.setFont(u8g2_font_4x6_tf);
  const char* pie = "largo=editar  ROJO=salir";
  u8g2.drawStr((128 - u8g2.getStrWidth(pie)) / 2, 63, pie);
  u8g2.sendBuffer();
}

// ----------------------------------------------------------- HISTORICO
#define MAX_LISTA   40
#define LISTA_FILAS 4
char listaPruebas[MAX_LISTA][22];
int  nPruebas = 0;

void cargarListaPruebas() {
  nPruebas = 0;
  histCursor = 0;
  if (!fsListo) return;
  File dir = LittleFS.open("/");
  if (!dir) return;
  File f = dir.openNextFile();
  while (f && nPruebas < MAX_LISTA) {
    if (!f.isDirectory()) {
      const char* n = f.name();
      if (*n == '/') n++;                       // el core puede devolver la ruta
      strncpy(listaPruebas[nPruebas], n, sizeof(listaPruebas[0]) - 1);
      listaPruebas[nPruebas][sizeof(listaPruebas[0]) - 1] = '\0';
      nPruebas++;
    }
    f = dir.openNextFile();
  }
}

// Lee del CSV solo las lineas de cabecera que interesan para el detalle
void mostrarDetallePrueba() {
  static unsigned long ult = 0;
  if (millis() - ult < 400) return;
  ult = millis();

  char ruta[32];
  snprintf(ruta, sizeof(ruta), "/%s", listaPruebas[histDetalle]);

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, 7, listaPruebas[histDetalle]);
  u8g2.drawFrame(0, 9, 128, 1);

  File f = LittleFS.open(ruta, FILE_READ);
  if (!f) {
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(0, 30, "No se puede abrir");
  } else {
    u8g2.setFont(u8g2_font_5x7_tf);
    int y = 20;
    while (f.available() && y < 56) {
      String l = f.readStringUntil('\n');
      l.trim();
      int coma = l.indexOf(',');
      if (coma < 0) continue;
      String clave = l.substring(0, coma), valor = l.substring(coma + 1);
      char buf[34];
      if      (clave == "agente")        snprintf(buf, sizeof(buf), "Agente: %s", valor.c_str());
      else if (clave == "hora")          snprintf(buf, sizeof(buf), "Hora:   %s", valor.c_str());
      else if (clave == "vmax_pico_kmh") snprintf(buf, sizeof(buf), "V.pico: %s km/h", valor.c_str());
      else if (clave == "vmax_5s_kmh")   snprintf(buf, sizeof(buf), "V.5s:   %s km/h", valor.c_str());
      else if (clave == "t_s")           break;      // empieza la curva
      else continue;
      u8g2.drawStr(0, y, buf);
      y += 9;
    }
    f.close();
  }

  u8g2.drawFrame(0, 55, 128, 1);
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, 63, "rojo=volver");
  const char* pie = "azul=imprimir";
  u8g2.drawStr(128 - u8g2.getStrWidth(pie), 63, pie);
  u8g2.sendBuffer();
}

// --- IMPRIMIR UNA PRUEBA GUARDADA ---
// imprimir() trabaja sobre las variables globales de la ultima prueba. Para
// reimprimir una guardada se vuelcan sus datos en esas mismas variables, se
// imprime con el codigo de siempre y despues se restaura lo que habia. Asi el
// ticket de una prueba antigua sale identico al que se imprimio en su dia.
bool imprimiendoGuardada = false;

struct CopiaPrueba {
  float  pico, sost, hist[MAX_PUNTOS];
  int    n;
  char   agente[MAX_AGENTE + 1], nota[MAX_NOTA + 1];
  double lat, lon;
  bool   fix;
  int    dia, mes, anio, hora, min, seg, dsem;
};
static CopiaPrueba copiaPrueba;   // estatica: ~330 B que no conviene en la pila
// Sin el struct en la firma: el preprocesador de Arduino inyecta los prototipos
// antes de que este definido y no compilaria.

void guardarEstadoPrueba() {
  CopiaPrueba& c = copiaPrueba;
  c.pico = velMaxPico; c.sost = velMaxMantenida; c.n = nPuntos;
  memcpy(c.hist, histVel, sizeof(histVel));
  strncpy(c.agente, nombreAgente, sizeof(c.agente));
  strncpy(c.nota, notaPrueba, sizeof(c.nota));
  c.lat = gpsLat; c.lon = gpsLon; c.fix = gpsFix;
  c.dia = localDia; c.mes = localMes; c.anio = localAnio;
  c.hora = localHora; c.min = localMin; c.seg = localSeg; c.dsem = localDiaSemana;
}

void restaurarEstadoPrueba() {
  const CopiaPrueba& c = copiaPrueba;
  velMaxPico = c.pico; velMaxMantenida = c.sost; nPuntos = c.n;
  memcpy(histVel, c.hist, sizeof(histVel));
  strncpy(nombreAgente, c.agente, sizeof(nombreAgente));
  strncpy(notaPrueba, c.nota, sizeof(notaPrueba));
  gpsLat = c.lat; gpsLon = c.lon; gpsFix = c.fix;
  localDia = c.dia; localMes = c.mes; localAnio = c.anio;
  localHora = c.hora; localMin = c.min; localSeg = c.seg; localDiaSemana = c.dsem;
}

// Vuelca el CSV en las variables globales. Devuelve false si no se pudo leer.
bool cargarPruebaGuardada(const char* nombre) {
  char ruta[32];
  snprintf(ruta, sizeof(ruta), "/%s", nombre);
  File f = LittleFS.open(ruta, FILE_READ);
  if (!f) return false;

  nPuntos = 0; velMaxPico = 0; velMaxMantenida = 0;
  gpsFix = false; nombreAgente[0] = '\0'; notaPrueba[0] = '\0';
  localDia = localMes = localAnio = localHora = localMin = localSeg = 0;
  localDiaSemana = -1;                     // sin fecha, sin nombre de dia
  bool enCurva = false;

  while (f.available()) {
    String l = f.readStringUntil('\n');
    l.trim();
    int c = l.indexOf(',');
    if (c < 0) continue;
    String k = l.substring(0, c), v = l.substring(c + 1);
    if (enCurva) {
      if (nPuntos < MAX_PUNTOS) histVel[nPuntos++] = v.toFloat();
      continue;
    }
    if      (k == "t_s")           enCurva = true;
    else if (k == "agente")        v.toCharArray(nombreAgente, sizeof(nombreAgente));
    else if (k == "nota")          v.toCharArray(notaPrueba, sizeof(notaPrueba));
    else if (k == "vmax_pico_kmh") velMaxPico = v.toFloat();
    else if (k == "vmax_5s_kmh")   velMaxMantenida = v.toFloat();
    else if (k == "lat" && v.length()) { gpsLat = v.toDouble(); gpsFix = true; }
    else if (k == "lon" && v.length()) gpsLon = v.toDouble();
    else if (k == "fecha" && v.length() >= 10) {
      localDia  = v.substring(0, 2).toInt();
      localMes  = v.substring(3, 5).toInt();
      localAnio = v.substring(6, 10).toInt();
      localDiaSemana = diaSemana(localAnio, localMes, localDia);
    }
    else if (k == "hora" && v.length() >= 8) {
      localHora = v.substring(0, 2).toInt();
      localMin  = v.substring(3, 5).toInt();
      localSeg  = v.substring(6, 8).toInt();
    }
  }
  f.close();
  return true;
}

void imprimirPruebaGuardada(const char* nombre) {
  guardarEstadoPrueba();
  if (cargarPruebaGuardada(nombre)) {
    imprimiendoGuardada = true;
    imprimir();
    imprimiendoGuardada = false;
  } else {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr((128 - u8g2.getStrWidth("No se puede leer")) / 2, 35, "No se puede leer");
    u8g2.sendBuffer();
    delay(1200);
  }
  restaurarEstadoPrueba();
}

void mostrarHistorico() {
  if (histDetalle >= 0) { mostrarDetallePrueba(); return; }

  static unsigned long ult = 0;
  if (millis() - ult < 250) return;
  ult = millis();

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  char titulo[24];
  snprintf(titulo, sizeof(titulo), "HISTORICO (%d)", nPruebas);
  u8g2.drawStr((128 - u8g2.getStrWidth(titulo)) / 2, 9, titulo);
  u8g2.drawFrame(0, 11, 128, 1);

  u8g2.setFont(u8g2_font_5x7_tf);
  if (nPruebas == 0) {
    u8g2.drawStr(0, 30, "No hay pruebas guardadas");
  } else {
    // Ventana deslizante centrada en el cursor
    int primera = histCursor - LISTA_FILAS / 2;
    if (primera < 0) primera = 0;
    if (primera > nPruebas - LISTA_FILAS) primera = nPruebas - LISTA_FILAS;
    if (primera < 0) primera = 0;

    for (int i = 0; i < LISTA_FILAS && primera + i < nPruebas; i++) {
      int idx = primera + i;
      int y = 21 + i * 9;
      if (idx == histCursor) {
        u8g2.drawBox(0, y - 7, 128, 9);
        u8g2.setDrawColor(0);
      }
      u8g2.drawStr(2, y, listaPruebas[idx]);
      u8g2.setDrawColor(1);
    }
  }

  // Botonera: rojo a la izquierda; el azul lleva un rotulo encima y sus dos
  // pulsaciones debajo, a la derecha.
  u8g2.drawFrame(0, 52, 128, 1);
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(128 - u8g2.getStrWidth("azul"), 58, "azul");
  u8g2.drawStr(0, 64, "rojo=salir");
  const char* der = "corto=bajar largo=ver";
  u8g2.drawStr(128 - u8g2.getStrWidth(der), 64, der);
  u8g2.sendBuffer();
}

// --------------------------------------------------------- PERIFERICOS
void mostrarPerifericos() {
  static unsigned long ult = 0;
  if (millis() - ult < 250) return;
  ult = millis();

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr((128 - u8g2.getStrWidth("PERIFERICOS")) / 2, 9, "PERIFERICOS");
  u8g2.drawFrame(0, 11, 128, 1);

  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawStr(0, 26, "Tipo de impresora:");
  for (uint8_t i = 0; i <= 1; i++) {
    TipoImpresora t = (TipoImpresora)i;
    int y = 38 + i * 10;
    bool sel = (t == tipoImpresora);
    u8g2.drawStr(4, y, sel ? ">" : " ");
    u8g2.drawStr(12, y, nombreTipoImpresora(t));
  }

  u8g2.setFont(u8g2_font_4x6_tf);
  const char* pie = "AZUL=cambiar  ROJO=salir";
  u8g2.drawStr((128 - u8g2.getStrWidth(pie)) / 2, 63, pie);
  u8g2.sendBuffer();
}

// =====================================================================
//  PROTOCOLO POR USB (para la webapp de Chrome via Web Serial)
// =====================================================================
// Ordenes en texto plano, una por linea, para poder probarlas a mano desde el
// Monitor Serie. Respuestas en JSON de una sola linea, para que el navegador
// haga JSON.parse() directamente.
//
// Toda respuesta empieza por '{'. Los mensajes de depuracion del programa
// ([FS], [BLE]...) salen por el mismo puerto, asi que la webapp debe ignorar
// cualquier linea que no parsee como JSON. Es mas robusto que silenciarlos.
//
//   PING              comprobacion de vida
//   INFO              identidad, version y estado
//   LIST              pruebas guardadas, con tamano
//   GET <nombre>      contenido del CSV, en base64
//   DEL <nombre>      borra una prueba
//   CFG               agente, nota y tipo de impresora
//   SETID <texto>     cambia el identificador del aparato
//   SETAG <texto>     cambia el agente
//   SETNOTA <texto>   cambia la nota
//   SETIMP <tipo>     cambia la impresora: "cable" o "bluetooth"
#define MAX_CMD 96

static const char B64[] =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

void jsonCadena(const char* s) {
  Serial.write('"');
  for (; *s; s++) {
    if (*s == '"' || *s == '\\') { Serial.write('\\'); Serial.write(*s); }
    else if ((uint8_t)*s < 0x20)  Serial.printf("\\u%04X", (unsigned)*s);
    else Serial.write(*s);
  }
  Serial.write('"');
}

void respError(const char* cmd, const char* msg) {
  Serial.print("{\"ok\":false,\"cmd\":"); jsonCadena(cmd);
  Serial.print(",\"error\":");            jsonCadena(msg);
  Serial.println("}");
}

// Los campos de texto se normalizan a mayusculas y sin '*' para no romper la
// codificacion compacta del QR, que usa el juego alfanumerico y ese separador.
void copiarCampo(char* dest, size_t maxLen, const char* src) {
  size_t n = 0;
  while (*src && n < maxLen) {
    char c = *src++;
    if ((uint8_t)c < 0x20 || c == '*') continue;
    dest[n++] = toupper((unsigned char)c);
  }
  dest[n] = '\0';
}

void macBase(char* dest, size_t tam) {
  uint8_t m[6];
  if (esp_read_mac(m, ESP_MAC_BASE) == ESP_OK)
    snprintf(dest, tam, "%02X:%02X:%02X:%02X:%02X:%02X",
             m[0], m[1], m[2], m[3], m[4], m[5]);
  else
    snprintf(dest, tam, "");
}

// Sin identificador guardado se genera uno a partir de la MAC, que es unica
// por chip. Asi ningun aparato sale de fabrica sin identidad.
void asegurarIdDispositivo() {
  prefs.getString("id", "").toCharArray(idDispositivo, sizeof(idDispositivo));
  if (idDispositivo[0] == '\0') {
    uint8_t m[6];
    if (esp_read_mac(m, ESP_MAC_BASE) == ESP_OK)
      snprintf(idDispositivo, sizeof(idDispositivo), "TACO-%02X%02X%02X",
               m[3], m[4], m[5]);
    else
      strncpy(idDispositivo, "TACO-SINMAC", MAX_ID_DISP);
    prefs.putString("id", idDispositivo);
  }
}

int contarPruebas() {
  if (!fsListo) return 0;
  int n = 0;
  File dir = LittleFS.open("/");
  if (!dir) return 0;
  File f = dir.openNextFile();
  while (f) { if (!f.isDirectory()) n++; f = dir.openNextFile(); }
  return n;
}

// Base64 al vuelo: se emite segun se lee, sin reservar buffer para el archivo.
void enviarBase64(File& f) {
  uint8_t in[3];
  while (true) {
    int n = f.read(in, 3);
    if (n <= 0) break;
    Serial.write(B64[in[0] >> 2]);
    Serial.write(B64[((in[0] & 0x03) << 4) | (n > 1 ? (in[1] >> 4) : 0)]);
    Serial.write(n > 1 ? B64[((in[1] & 0x0F) << 2) | (n > 2 ? (in[2] >> 6) : 0)] : '=');
    Serial.write(n > 2 ? B64[in[2] & 0x3F] : '=');
    if (n < 3) break;
  }
}

void cmdInfo() {
  char mac[18];
  macBase(mac, sizeof(mac));

  Serial.print("{\"ok\":true,\"cmd\":\"INFO\"");
  Serial.printf(",\"proto\":%d", PROTO_VERSION);
  Serial.print(",\"fw\":");    jsonCadena(FW_VERSION);
  Serial.print(",\"build\":"); jsonCadena(__DATE__ " " __TIME__);
  Serial.print(",\"id\":");    jsonCadena(idDispositivo);
  Serial.print(",\"mac\":");   jsonCadena(mac);
  Serial.print(",\"chip\":");  jsonCadena(ESP.getChipModel());
  Serial.printf(",\"rev\":%d", (int)ESP.getChipRevision());
  Serial.printf(",\"cpuMHz\":%u", (unsigned)ESP.getCpuFreqMHz());
  Serial.printf(",\"flash\":%u", (unsigned)ESP.getFlashChipSize());
  Serial.printf(",\"heap\":%u", (unsigned)ESP.getFreeHeap());
  Serial.printf(",\"fsTotal\":%u", fsListo ? (unsigned)LittleFS.totalBytes() : 0);
  Serial.printf(",\"fsUsado\":%u", fsListo ? (unsigned)LittleFS.usedBytes() : 0);
  Serial.printf(",\"pruebas\":%d", contarPruebas());
  Serial.printf(",\"estado\":%d", (int)estadoActual);
  Serial.println("}");
}

void cmdList() {
  Serial.print("{\"ok\":true,\"cmd\":\"LIST\",\"pruebas\":[");
  if (fsListo) {
    File dir = LittleFS.open("/");
    if (dir) {
      File f = dir.openNextFile();
      bool primero = true;
      while (f) {
        if (!f.isDirectory()) {
          const char* n = f.name();
          if (*n == '/') n++;
          if (!primero) Serial.print(',');
          primero = false;
          Serial.print("{\"n\":"); jsonCadena(n);
          Serial.printf(",\"b\":%u}", (unsigned)f.size());
        }
        f = dir.openNextFile();
      }
    }
  }
  Serial.println("]}");
}

void cmdGet(const char* nombre) {
  if (!fsListo)    { respError("GET", "sin sistema de ficheros"); return; }
  if (!*nombre)    { respError("GET", "falta el nombre");         return; }
  char ruta[48];
  snprintf(ruta, sizeof(ruta), "/%s", nombre);
  File f = LittleFS.open(ruta, FILE_READ);
  if (!f)          { respError("GET", "no existe");               return; }

  Serial.print("{\"ok\":true,\"cmd\":\"GET\",\"n\":"); jsonCadena(nombre);
  Serial.printf(",\"b\":%u,\"b64\":\"", (unsigned)f.size());
  enviarBase64(f);
  f.close();
  Serial.println("\"}");
}

void cmdDel(const char* nombre) {
  if (!fsListo) { respError("DEL", "sin sistema de ficheros"); return; }
  if (!*nombre) { respError("DEL", "falta el nombre");         return; }
  char ruta[48];
  snprintf(ruta, sizeof(ruta), "/%s", nombre);
  if (!LittleFS.exists(ruta)) { respError("DEL", "no existe"); return; }
  if (!LittleFS.remove(ruta)) { respError("DEL", "no se pudo borrar"); return; }
  Serial.print("{\"ok\":true,\"cmd\":\"DEL\",\"n\":"); jsonCadena(nombre);
  Serial.println("}");
}

void cmdCfg() {
  Serial.print("{\"ok\":true,\"cmd\":\"CFG\",\"id\":"); jsonCadena(idDispositivo);
  Serial.print(",\"agente\":"); jsonCadena(nombreAgente);
  Serial.print(",\"nota\":");   jsonCadena(notaPrueba);
  Serial.print(",\"impresora\":");
  jsonCadena(tipoImpresora == IMP_CABLE ? "cable" : "bluetooth");
  Serial.println("}");
}

void ejecutarComando(char* linea) {
  // Separa la orden de su argumento
  char* arg = strchr(linea, ' ');
  if (arg) { *arg = '\0'; arg++; while (*arg == ' ') arg++; }
  else     { arg = linea + strlen(linea); }
  for (char* p = linea; *p; p++) *p = toupper((unsigned char)*p);

  if      (!strcmp(linea, "PING")) Serial.println("{\"ok\":true,\"cmd\":\"PING\"}");
  else if (!strcmp(linea, "INFO")) cmdInfo();
  else if (!strcmp(linea, "LIST")) cmdList();
  else if (!strcmp(linea, "GET"))  cmdGet(arg);
  else if (!strcmp(linea, "DEL"))  cmdDel(arg);
  else if (!strcmp(linea, "CFG"))  cmdCfg();
  else if (!strcmp(linea, "SETID")) {
    copiarCampo(idDispositivo, MAX_ID_DISP, arg);
    prefs.putString("id", idDispositivo);
    cmdCfg();
  }
  else if (!strcmp(linea, "SETAG")) {
    copiarCampo(nombreAgente, MAX_AGENTE, arg);
    prefs.putString("agente", nombreAgente);
    cmdCfg();
  }
  else if (!strcmp(linea, "SETNOTA")) {
    copiarCampo(notaPrueba, MAX_NOTA, arg);
    prefs.putString("nota", notaPrueba);
    cmdCfg();
  }
  else if (!strcmp(linea, "SETIMP")) {
    for (char* q = arg; *q; q++) *q = tolower((unsigned char)*q);
    if      (!strcmp(arg, "cable"))     tipoImpresora = IMP_CABLE;
    else if (!strcmp(arg, "bluetooth")) tipoImpresora = IMP_BLUETOOTH;
    else { respError("SETIMP", "usa 'cable' o 'bluetooth'"); return; }
    aplicarTipoImpresora();
    prefs.putUChar("impresora", (uint8_t)tipoImpresora);
    cmdCfg();
  }
  else respError(linea, "orden desconocida");
}

// No bloquea: se llama en cada vuelta del loop y consume lo que haya llegado.
void atenderPuertoSerie() {
  static char linea[MAX_CMD + 1];
  static uint8_t n = 0;
  // Vale cualquier fin de linea: \n, \r o \r\n. Al terminar una orden el
  // contador se pone a cero, asi que el segundo caracter del par no repite.
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (n > 0) { linea[n] = '\0'; ejecutarComando(linea); n = 0; }
      continue;
    }
    if (n < MAX_CMD) linea[n++] = c;
  }
}

// --- GENERADOR TEMPORAL DE PRUEBAS DE EJEMPLO (quitar cuando sobre) ---
// Se dejo a 0 tras generar los ejemplos. Ponerlo a 1 vuelve a crearlos, pero
// respeta los que ya existan: no machaca ninguna prueba real.
#define GENERAR_PRUEBAS_DEMO 0
#if GENERAR_PRUEBAS_DEMO
void crearPruebaDemo(const char* nombre, const char* agente, const char* nota,
                     const char* fecha, const char* hora, float pico, float sost) {
  char ruta[40], num[12];
  snprintf(ruta, sizeof(ruta), "/%s", nombre);
  if (LittleFS.exists(ruta)) { Serial.printf("[FS] %s ya existia\n", ruta); return; }

  File f = LittleFS.open(ruta, FILE_WRITE);
  if (!f) { Serial.printf("[FS] no se pudo crear %s\n", ruta); return; }

  f.printf("agente,%s\n", agente);
  f.printf("nota,%s\n", nota);
  f.printf("fecha,%s\n", fecha);
  f.printf("hora,%s\n", hora);
  f.println("lat,41.64880");
  f.println("lon,-0.88910");
  f.printf("duracion_s,%d\n", tiempoPrueba);
  dtostrf(pico, 0, 1, num); f.printf("vmax_pico_kmh,%s\n", num);
  dtostrf(sost, 0, 1, num); f.printf("vmax_5s_kmh,%s\n", num);
  f.println("t_s,vel_kmh");
  for (int i = 0; i < MAX_PUNTOS; i++) {
    float t = (float)i / (float)(MAX_PUNTOS - 1);
    dtostrf(pico * (1.0 - exp(-4.0 * t)), 0, 1, num);
    int decimas = i * VENTANA_MS / 100;
    f.printf("%d.%d,%s\n", decimas / 10, decimas % 10, num);
  }
  f.close();
  Serial.printf("[FS] creada %s\n", ruta);
}

void generarPruebasDemo() {
  crearPruebaDemo("20260223-111023.csv", "MANOLITO 23453", "RODILLO SECO",
                  "23/02/2026", "11:10:23", 27.4, 24.8);
  crearPruebaDemo("20260223-113512.csv", "MANOLITO 23453", "SEGUNDA VUELTA",
                  "23/02/2026", "11:35:12", 31.2, 29.0);
  crearPruebaDemo("20260224-092740.csv", "LAURA 10233", "",
                  "24/02/2026", "09:27:40", 22.9, 21.5);
}
#endif

void setup() {
  Serial.begin(115200);
  SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

  Wire.begin(PIN_SDA, PIN_SCL);
  u8g2.setI2CAddress(0x78);
  u8g2.begin();

  pinMode(pinSensor,   INPUT_PULLUP);
  pinMode(pinEmpezar,  INPUT_PULLUP);
  pinMode(pinImprimir, INPUT_PULLUP);
  // PWM para el MOSFET del motor de arranque (ESP32 core 3.x).
  ledcAttach(pinMotor, pwmFreqMotor, pwmResMotor);
  ledcWrite(pinMotor, 0);   // motor parado
  pinMode(pinLedAzul,   OUTPUT);
  pinMode(pinLedVerde,  OUTPUT);
  pinMode(pinLedRojo,   OUTPUT);
  pinMode(pinLedSensor, OUTPUT);
  ledPlaca(false);
  rgbLedWrite(PIN_RGB_LED, 0, 0, 0);   // LED RGB apagado hasta la primera pantalla

  attachInterrupt(digitalPinToInterrupt(pinSensor), detectarPulso, FALLING);

  // Tipo de impresora elegido la ultima vez (se conserva entre encendidos)
  prefs.begin("taco", false);
  uint8_t guardado = prefs.getUChar("impresora", (uint8_t)IMP_CABLE);
  tipoImpresora = (guardado == (uint8_t)IMP_CABLE) ? IMP_CABLE : IMP_BLUETOOTH;
  aplicarTipoImpresora();
  prefs.getString("agente", "").toCharArray(nombreAgente, sizeof(nombreAgente));
  prefs.getString("nota", "").toCharArray(notaPrueba, sizeof(notaPrueba));
  asegurarIdDispositivo();

  // Almacen de pruebas. El true formatea la particion la primera vez.
  fsListo = LittleFS.begin(true);
  Serial.printf("[FS] %s  (%u KB usados de %u KB)\n",
                fsListo ? "listo" : "FALLO",
                (unsigned)(LittleFS.usedBytes() / 1024),
                (unsigned)(LittleFS.totalBytes() / 1024));
#if GENERAR_PRUEBAS_DEMO
  if (fsListo) generarPruebasDemo();
#endif

  impresoraSerie.begin();   // UART de la impresora por cable
  impresoraBLE.begin();     // arranca la pila BLE (no conecta todavia)

#if DIAGNOSTICO_BLE >= 1
  escanearBLE();
#endif
#if DIAGNOSTICO_BLE >= 2
  Serial.println("[BLE] Probando conexion con la impresora...");
  if (impresoraBLE.conectar()) Serial.println("[BLE] OK: lista para imprimir");
  else                         Serial.println("[BLE] FALLO: no se pudo preparar la impresora");
#endif
#if DIAGNOSTICO_BLE >= 3
  pruebaImpresion();
#endif
}

#if DIAGNOSTICO_BLE >= 3
// Rellena datos ficticios y lanza un ticket completo (texto, grafica y QR)
// para comprobar la cadena ESC/POS de punta a punta sin hacer una prueba real.
void pruebaImpresion() {
  velMaxPico      = 27.4;
  velMaxMantenida = 24.8;
  nPuntos = MAX_PUNTOS;
  for (int i = 0; i < MAX_PUNTOS; i++) {          // curva de aceleracion tipica
    float t = (float)i / (float)(MAX_PUNTOS - 1);
    histVel[i] = 27.4 * (1.0 - exp(-4.0 * t));
  }
  gpsFix  = true;                                 // Zaragoza, para que salga el QR
  gpsLat  = 41.64880;
  gpsLon  = -0.88910;
  gpsAnio = 2026; gpsMes  = 8;  gpsDia = 3;
  gpsHora = 20;   gpsMin  = 15; gpsSeg = 0;       // UTC

  Serial.println("[BLE] Lanzando ticket de prueba...");
  unsigned long t0 = millis();
  imprimir();
  Serial.printf("[BLE] Ticket enviado en %lu ms\n", millis() - t0);
}
#endif

void loop() {
  atenderPuertoSerie();    // ordenes de la webapp por USB
  actualizarGPS();
  actualizarLedRGB();      // color de la placa segun la pantalla activa
  actualizarLedSensor();   // destello con cada pulso del rodillo

  switch (estadoActual) {
    case BIENVENIDA:
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr((128 - u8g2.getStrWidth("Tacometro")) / 2, 30, "Tacometro");
      u8g2.drawStr((128 - u8g2.getStrWidth("VMP")) / 2, 55, "VMP");
      u8g2.sendBuffer();
      // Espera atendida: durante estos 5 s el puerto USB sigue respondiendo
      { unsigned long t0 = millis();
        while (millis() - t0 < 5000) { atenderPuertoSerie(); delay(10); } }
      estadoActual = ESPERA;

      break;

    case ESPERA:
      mostrarEspera();
      digitalWrite(pinLedVerde, (millis() / 500) % 2);  // parpadeo cada 0,5 s
      digitalWrite(pinLedAzul, (millis() / 500) % 2);  // parpadeo cada 0,5 s
      if (digitalRead(pinEmpezar) == LOW) {
        delay(500);
        digitalWrite(pinLedVerde,0);
        digitalWrite(pinLedAzul,0);
        iniciarArranque();
        estadoActual = ARRANQUE;
      }
      if (digitalRead(pinImprimir) == LOW) {            // boton azul: historico
        digitalWrite(pinLedVerde, 0);
        digitalWrite(pinLedAzul,0);
        cargarListaPruebas();
        histDetalle = -1;
        estadoActual = HISTORICO;
        esperarSoltar(pinImprimir);
      }
      break;

    // ---- ARBOL DE MENUS: el rojo siempre devuelve a ESPERA ----
    case MENU: {
      mostrarMenu();
      if (rojoPulsado()) { volverAEspera(); break; }
      int p = leerAzul();
      if (p == PULSA_CORTA) menuCursor = (menuCursor + 1) % 3;
      else if (p == PULSA_LARGA) {
        if      (menuCursor == 0) { datosCursor = 0; estadoActual = DATOS; }
        else if (menuCursor == 1) { cargarListaPruebas(); histDetalle = -1;
                                    estadoActual = HISTORICO; }
        else                      { estadoActual = PERIFERICOS; }
        esperarSoltar(pinImprimir);
      }
      break;
    }

    case DATOS: {
      mostrarDatos();
      if (editCampo) {                    // editando con el teclado BLE
        procesarTeclado();                // Intro o Esc cierran la edicion
        if (rojoPulsado()) { volverAEspera(); break; }
        if (leerAzul() == PULSA_CORTA && !teclado.conectado()) teclado.conectar();
        break;
      }
      if (rojoPulsado()) { volverAEspera(); break; }
      int p = leerAzul();
      if (p == PULSA_CORTA) datosCursor = (datosCursor + 1) % 2;
      else if (p == PULSA_LARGA) {
        // El teclado solo se enlaza mientras se edita: asi nunca compite
        // con la impresora por el enlace BLE.
        if (datosCursor == 0) iniciarEdicion(nombreAgente, MAX_AGENTE, "AGENTE");
        else                  iniciarEdicion(notaPrueba,   MAX_NOTA,   "NOTA");
        esperarSoltar(pinImprimir);
      }
      break;
    }

    case HISTORICO: {
      mostrarHistorico();
      digitalWrite(pinLedRojo, (millis() / 500) % 2);   // parpadeo rojo y azul
      digitalWrite(pinLedAzul, (millis() / 500) % 2);
      int p = leerAzul();
      if (histDetalle >= 0) {                    // viendo una prueba
        if (rojoPulsado()) { histDetalle = -1; esperarSoltar(pinEmpezar); break; }
        if (p != PULSA_NADA) {
          imprimirPruebaGuardada(listaPruebas[histDetalle]);
          esperarSoltar(pinImprimir);
        }
        break;
      }
      if (rojoPulsado()) { volverAEspera(); break; }
      if (p == PULSA_CORTA && nPruebas > 0) histCursor = (histCursor + 1) % nPruebas;
      else if (p == PULSA_LARGA && nPruebas > 0) {
        histDetalle = histCursor;
        esperarSoltar(pinImprimir);
      }
      break;
    }

    case PERIFERICOS:
      mostrarPerifericos();
      if (rojoPulsado()) { volverAEspera(); break; }
      if (leerAzul() != PULSA_NADA) {
        tipoImpresora = (tipoImpresora == IMP_BLUETOOTH) ? IMP_CABLE : IMP_BLUETOOTH;
        aplicarTipoImpresora();
      }
      break;

    case ARRANQUE:
      digitalWrite(pinLedRojo, (millis() / 500) % 2);  // parpadeo cada 0,5 s
      ejecutarArranque();   // transita a MEDICION (velocidad alcanzada) o a ESPERA (timeout)
      if (digitalRead(pinEmpezar) == LOW) {            // abortar arranque a mano
        ledcWrite(pinMotor, 0);
        estadoActual = ESPERA;
        delay(500);
      }
      if (estadoActual != ARRANQUE) digitalWrite(pinLedRojo, 0);
      break;

    case MEDICION:
      digitalWrite(pinLedRojo, 1);   // fijo durante la medida
      ejecutarMedida();              // pone estadoActual = RESULTADO al cumplirse tiempoPrueba
      if (digitalRead(pinEmpezar) == LOW) {   // boton empezar: salir de la prueba a ESPERA
        estadoActual = ESPERA;
        ledcWrite(pinMotor, 0);
        delay(500);
      }
      if (estadoActual != MEDICION) digitalWrite(pinLedRojo, 0);
      break;

    case RESULTADO:
      digitalWrite(pinLedAzul,1);
      digitalWrite(pinLedRojo,1);
      ejecutarResultado();
      if (digitalRead(pinEmpezar) == LOW) {
        delay(1000);
        estadoActual = ESPERA;
        digitalWrite(pinLedAzul,0);
        digitalWrite(pinLedRojo,0);
      }
      if (digitalRead(pinImprimir) == LOW) {
        delay(200);
        imprimir();
      }

      break;
  }
}

void ejecutarResultado() {
  u8g2.clearBuffer();

  // Titulo centrado
  u8g2.setFont(u8g2_font_6x10_tf);
  const char* titulo = "RESULTADO";
  u8g2.drawStr((128 - u8g2.getStrWidth(titulo)) / 2, 9, titulo);
  u8g2.drawFrame(0, 11, 128, 1);

  // Datos de la prueba
  u8g2.setFont(u8g2_font_5x7_tf);
  char buf[26];

  snprintf(buf, sizeof(buf), "Duracion: %d s", tiempoPrueba);
  u8g2.drawStr(0, 22, buf);

  char num[10];
  dtostrf(velMaxPico, 0, 1, num);
  snprintf(buf, sizeof(buf), "V.max pico: %s km/h", num);
  u8g2.drawStr(0, 32, buf);

  dtostrf(velMaxMantenida, 0, 1, num);
  snprintf(buf, sizeof(buf), "V.max 5s: %s km/h", num);
  u8g2.drawStr(0, 42, buf);

  // Mensajes inferiores centrados (azul: imprimir / rojo: salir)
  const char* msgImprimir = "Imprimir boton azul";
  u8g2.drawStr((128 - u8g2.getStrWidth(msgImprimir)) / 2, 53, msgImprimir);
  const char* msgSalir = "Salir boton rojo";
  u8g2.drawStr((128 - u8g2.getStrWidth(msgSalir)) / 2, 62, msgSalir);

  u8g2.sendBuffer();
}

// Envia el ticket de resultados a la impresora termica BLE
// (ESC/POS, papel 58 mm = 32 columnas con fuente A).
void imprimir() {
  // Conectar con la impresora si todavia no lo esta
  if (!impresora->conectado()) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr((128 - u8g2.getStrWidth("Conectando...")) / 2, 35, "Conectando...");
    u8g2.sendBuffer();

    // La MP210 no siempre esta anunciandose cuando cae la ventana de rastreo,
    // asi que un unico intento falla de vez en cuando: se reintenta una vez.
    bool listo = impresora->conectar();      // rastreo + conexion: unos segundos
    if (!listo) listo = impresora->conectar();
    if (!listo) {
      u8g2.clearBuffer();
      u8g2.drawStr((128 - u8g2.getStrWidth("Sin impresora")) / 2, 35, "Sin impresora");
      u8g2.sendBuffer();
      delay(1500);
      return;
    }
  }

  // Con una prueba guardada la fecha/hora ya vienen del archivo
  if (!imprimiendoGuardada) calcularHoraLocal();

  char buf[40];
  char num[10];
  const char* sep = "--------------------------------";  // 32 col (papel 58 mm)

  // Inicializar impresora (ESC @)
  impresora->write(0x1B); impresora->write('@');

  // Cabecera centrada (ESC a 1)
  impresora->write(0x1B); impresora->write('a'); impresora->write((uint8_t)1);
  impresora->println("TACOMETRO VMP");
  impresora->println(sep);

  // Datos alineados a la izquierda (ESC a 0)
  impresora->write(0x1B); impresora->write('a'); impresora->write((uint8_t)0);

  snprintf(buf, sizeof(buf), "Fecha: %s %02d/%02d/%04d",
           nombreDiaSemana(localDiaSemana), localDia, localMes, localAnio);
  impresora->println(buf);
  snprintf(buf, sizeof(buf), "Hora:  %02d:%02d:%02d", localHora, localMin, localSeg);
  impresora->println(buf);

  if (nombreAgente[0]) {
    snprintf(buf, sizeof(buf), "Agente: %s", nombreAgente);
    impresora->println(buf);
  }
  if (notaPrueba[0]) {
    snprintf(buf, sizeof(buf), "Nota:   %s", notaPrueba);
    impresora->println(buf);
  }

  if (gpsFix) {
    impresora->println("Posicion GPS:");
    dtostrf(gpsLat, 0, 5, num);
    snprintf(buf, sizeof(buf), "  Lat: %s", num);
    impresora->println(buf);
    dtostrf(gpsLon, 0, 5, num);
    snprintf(buf, sizeof(buf), "  Lon: %s", num);
    impresora->println(buf);
  } else {
    impresora->println("Sin senal GPS");
  }
  impresora->println(sep);

  snprintf(buf, sizeof(buf), "Duracion prueba: %d s", tiempoPrueba);
  impresora->println(buf);

  dtostrf(velMaxPico, 0, 1, num);
  snprintf(buf, sizeof(buf), "Vel. max pico:  %s km/h", num);
  impresora->println(buf);

  dtostrf(velMaxMantenida, 0, 1, num);
  snprintf(buf, sizeof(buf), "Vel. max 5s:    %s km/h", num);
  impresora->println(buf);

  impresora->println(sep);

  // Grafica velocidad-tiempo de la prueba (centrada)
  if (nPuntos > 0) {
    impresora->write(0x1B); impresora->write('a'); impresora->write((uint8_t)1);  // centrar
    impresora->println("Velocidad (km/h) / tiempo (s)");
    imprimirGrafica();
    impresora->write(0x1B); impresora->write('a'); impresora->write((uint8_t)0);  // izquierda
    impresora->println(sep);
  }

  // QR con todos los datos de la prueba, para capturarlo con el movil.
  // Modulo 4 y correccion L: el nivel mas bajo deja mas sitio para datos, y a
  // 4 puntos por modulo el simbolo sale de unos 30 mm, comodo para la camara.
  {
    char carga[TAM_CARGA_QR];
    construirCargaQR(carga, sizeof(carga));

    impresora->write(0x1B); impresora->write('a'); impresora->write((uint8_t)1);
    impresora->println("--- DATOS DE LA PRUEBA ---");
    imprimirQR(carga, 4, 48);
    impresora->println("Escanear para exportar");
    impresora->write(0x1B); impresora->write('a'); impresora->write((uint8_t)0);
    impresora->println(sep);
  }

  // Codigo QR centrado con el enlace a Google Maps de la posicion
  if (gpsFix) {
    char latStr[12], lonStr[12], url[60];
    dtostrf(gpsLat, 0, 5, latStr);
    dtostrf(gpsLon, 0, 5, lonStr);
    snprintf(url, sizeof(url), "https://maps.google.com/?q=%s,%s", latStr, lonStr);

    impresora->write(0x1B); impresora->write('a'); impresora->write((uint8_t)1);  // centrar
    imprimirQR(url, 6, 49);
    impresora->println("Ver en Google Maps");
    impresora->write(0x1B); impresora->write('a'); impresora->write((uint8_t)0);  // izquierda
  }

  impresora->write('\n'); impresora->write('\n'); impresora->write('\n');  // avance de papel
  impresora->flush();

  // Aviso en pantalla
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr((128 - u8g2.getStrWidth("Imprimiendo...")) / 2, 35, "Imprimiendo...");
  u8g2.sendBuffer();
  delay(1500);
}

// =====================================================================
//  CARGA DEL QR DE DATOS
// =====================================================================
// El CSV completo son unos 800 B: en un QR saldrian ~100x100 modulos y sobre
// papel termico de 58 mm cada modulo quedaria tan fino que ninguna camara lo
// leeria. Por eso se codifica lo mismo de forma compacta (~215 caracteres),
// que cabe en un QR de 53x53 legible con holgura.
//
// FORMATO (documentado para la futura app del movil):
//   TZ1*DDMMYY*HHMMSS*LAT*LON*AGENTE*NOTA*DUR*PICO*SOST*CURVA
//     TZ1    version del formato
//     DUR    duracion en segundos
//     PICO   velocidad maxima de pico, en DECIMAS de km/h (274 = 27,4)
//     SOST   velocidad maxima sostenida 5 s, tambien en decimas
//     CURVA  un par de caracteres por punto, cada uno el valor en decimas
//            codificado en BASE 36 (0-9 A-Z). "7M" = 7*36+22 = 274 = 27,4 km/h
//
// Solo se usan caracteres del juego alfanumerico del QR (0-9 A-Z y  $%*+-./: ),
// que se codifica a 5,5 bits en vez de 8: de ahi que quepa tan comodo. El
// separador es '*' por el mismo motivo; '|' no pertenece a ese juego.
// (TAM_CARGA_QR se define arriba, junto al resto de constantes.)

static void base36dos(char* dest, int v) {
  static const char D[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
  if (v < 0) v = 0;
  if (v > 1295) v = 1295;      // tope de dos digitos en base 36
  dest[0] = D[v / 36];
  dest[1] = D[v % 36];
}

void construirCargaQR(char* dest, size_t tam) {
  char lat[14] = "", lon[14] = "";
  if (gpsFix) {
    dtostrf(gpsLat, 0, 5, lat);
    dtostrf(gpsLon, 0, 5, lon);
  }

  int n = snprintf(dest, tam, "TZ1*");
  if (gpsFix) {
    n += snprintf(dest + n, tam - n, "%02d%02d%02d*%02d%02d%02d*",
                  localDia, localMes, localAnio % 100,
                  localHora, localMin, localSeg);
  } else {
    n += snprintf(dest + n, tam - n, "**");
  }
  n += snprintf(dest + n, tam - n, "%s*%s*%s*%s*%d*%d*%d*",
                lat, lon, nombreAgente, notaPrueba, tiempoPrueba,
                (int)(velMaxPico * 10.0 + 0.5),
                (int)(velMaxMantenida * 10.0 + 0.5));

  // Curva: dos caracteres por punto
  for (int i = 0; i < nPuntos && (size_t)(n + 3) < tam; i++) {
    base36dos(dest + n, (int)(histVel[i] * 10.0 + 0.5));
    n += 2;
  }
  dest[n] = '\0';
}

// Imprime un codigo QR (ESC/POS, GS ( k) con la cadena indicada.
//   modulo  : tamano de cada modulo en puntos (1-16)
//   nivelEC : 48=L, 49=M, 50=Q, 51=H
void imprimirQR(const char* datos, uint8_t modulo, uint8_t nivelEC) {
  int len = strlen(datos);
  int store = len + 3;                  // cn + fn + m + datos
  uint8_t pL = store & 0xFF;
  uint8_t pH = (store >> 8) & 0xFF;

  // Seleccionar modelo 2
  impresora->write(0x1D); impresora->write('('); impresora->write('k');
  impresora->write((uint8_t)4); impresora->write((uint8_t)0);
  impresora->write((uint8_t)49); impresora->write((uint8_t)65);
  impresora->write((uint8_t)50); impresora->write((uint8_t)0);

  // Tamano del modulo en puntos
  impresora->write(0x1D); impresora->write('('); impresora->write('k');
  impresora->write((uint8_t)3); impresora->write((uint8_t)0);
  impresora->write((uint8_t)49); impresora->write((uint8_t)67); impresora->write(modulo);

  // Nivel de correccion de errores
  impresora->write(0x1D); impresora->write('('); impresora->write('k');
  impresora->write((uint8_t)3); impresora->write((uint8_t)0);
  impresora->write((uint8_t)49); impresora->write((uint8_t)69); impresora->write(nivelEC);

  // Guardar los datos en el buffer del simbolo
  impresora->write(0x1D); impresora->write('('); impresora->write('k');
  impresora->write(pL); impresora->write(pH);
  impresora->write((uint8_t)49); impresora->write((uint8_t)80); impresora->write((uint8_t)48);
  impresora->print(datos);

  // Imprimir el simbolo almacenado
  impresora->write(0x1D); impresora->write('('); impresora->write('k');
  impresora->write((uint8_t)3); impresora->write((uint8_t)0);
  impresora->write((uint8_t)49); impresora->write((uint8_t)81); impresora->write((uint8_t)48);
}


// =================== GRAFICA VELOCIDAD-TIEMPO ===================
// Fuente compacta de digitos 3x5 px (5 filas, 3 bits por fila) para rotular ejes.
static const uint8_t FONT3x5[10][5] = {
  {0b111, 0b101, 0b101, 0b101, 0b111}, // 0
  {0b010, 0b110, 0b010, 0b010, 0b111}, // 1
  {0b111, 0b001, 0b111, 0b100, 0b111}, // 2
  {0b111, 0b001, 0b111, 0b001, 0b111}, // 3
  {0b101, 0b101, 0b111, 0b001, 0b001}, // 4
  {0b111, 0b100, 0b111, 0b001, 0b111}, // 5
  {0b111, 0b100, 0b111, 0b101, 0b111}, // 6
  {0b111, 0b001, 0b010, 0b010, 0b010}, // 7
  {0b111, 0b101, 0b111, 0b101, 0b111}, // 8
  {0b111, 0b101, 0b111, 0b001, 0b111}, // 9
};

// Enciende un pixel del bitmap (MSB primero, 1 = negro).
void setPix(int x, int y) {
  if (x < 0 || x >= G_W || y < 0 || y >= G_H) return;
  bmp[y * G_BYTES + (x >> 3)] |= (0x80 >> (x & 7));
}

// Linea recta por el algoritmo de Bresenham.
void linea(int x0, int y0, int x1, int y1) {
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  while (true) {
    setPix(x0, y0);
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

int numAncho(int n) {                 // ancho en px de un numero (digito=3 + 1 separacion)
  if (n < 0) n = -n;
  int dig = 1;
  while (n >= 10) { n /= 10; dig++; }
  return dig * 4 - 1;
}

void drawNumber(int x, int y, int n) {
  char s[8];
  itoa(n, s, 10);
  for (int i = 0; s[i]; i++) {
    if (s[i] >= '0' && s[i] <= '9') {
      int d = s[i] - '0';
      for (int r = 0; r < 5; r++)
        for (int c = 0; c < 3; c++)
          if (FONT3x5[d][r] & (0b100 >> c)) setPix(x + c, y + r);
    }
    x += 4;
  }
}

// Construye el bitmap: ejes, rejilla, etiquetas y curva de velocidad.
void construirGrafica() {
  memset(bmp, 0, sizeof(bmp));
  const int mIzq = 24, mAb = 11, mTop = 3, mDer = 3;
  int x0   = mIzq;
  int yTop = mTop;
  int y0   = G_H - 1 - mAb;
  int w    = G_W - 1 - mDer - x0;
  int h    = y0 - yTop;

  // Escala Y redondeada al alza a multiplo de 5 km/h
  float vMax = (velMaxPico < 1.0) ? 5.0 : ceil(velMaxPico / 5.0) * 5.0;

  // Ejes
  linea(x0, yTop, x0, y0);          // eje Y
  linea(x0, y0, x0 + w, y0);        // eje X

  // Etiquetas y rejilla horizontal cada 5 km/h
  int vMaxI = (int)(vMax + 0.5);
  for (int v = 0; v <= vMaxI; v += 5) {
    int y = y0 - (int)((float)v / vMax * h);
    linea(x0 - 2, y, x0, y);                                  // marca
    for (int gx = x0 + 2; gx < x0 + w; gx += 6) setPix(gx, y); // rejilla punteada
    drawNumber(x0 - 3 - numAncho(v), y - 2, v);
  }

  // Etiquetas eje X cada 5 s, con rejilla vertical punteada
  for (int s = 0; s <= tiempoPrueba; s += 5) {
    int x = x0 + (int)((long)s * w / tiempoPrueba);
    linea(x, y0, x, y0 + 2);                                   // marca
    if (s > 0 && s < tiempoPrueba)
      for (int gy = y0 - 2; gy > yTop; gy -= 6) setPix(x, gy); // rejilla
    drawNumber(x - numAncho(s) / 2, y0 + 3, s);
  }

  // Curva (el eje X representa toda la duracion: si la prueba se abortó,
  // la traza acaba en el instante real)
  int pxA = 0, pyA = 0;
  for (int i = 0; i < nPuntos; i++) {
    int px = x0 + (int)((long)i * w / (MAX_PUNTOS - 1));
    float v = histVel[i];
    if (v < 0) v = 0;
    if (v > vMax) v = vMax;
    int py = y0 - (int)(v / vMax * h);
    if (i > 0) linea(pxA, pyA, px, py); else setPix(px, py);
    pxA = px; pyA = py;
  }
}

// Envia el bitmap a la impresora como imagen rasterizada (GS v 0).
void imprimirGrafica() {
  if (nPuntos == 0) return;
  construirGrafica();
  impresora->write(0x1D); impresora->write('v'); impresora->write('0');
  impresora->write((uint8_t)0);                       // modo normal
  impresora->write((uint8_t)(G_BYTES & 0xFF));        // xL
  impresora->write((uint8_t)((G_BYTES >> 8) & 0xFF)); // xH
  impresora->write((uint8_t)(G_H & 0xFF));            // yL
  impresora->write((uint8_t)((G_H >> 8) & 0xFF));     // yH
  impresora->write(bmp, sizeof(bmp));
  impresora->write('\n');
}

// =====================================================================
//  ALMACENAMIENTO DE PRUEBAS (LittleFS en la flash interna)
// =====================================================================
// Cada prueba se guarda como /AAAAMMDD-HHMMSS.csv en la particion de 1,5 MB
// que el esquema de particiones por defecto ya reserva sin usar. A unos 400 B
// por prueba caben del orden de 4000, muy por encima de lo previsible.
// (fsListo se declara junto al resto de globales, antes de setup())

// Sin cobertura GPS no hay fecha con la que nombrar el archivo, asi que se
// recurre a un contador que sobrevive a los apagones.
void nombreArchivoPrueba(char* destino, size_t tam) {
  if (gpsFix) {
    calcularHoraLocal();
    snprintf(destino, tam, "/%04d%02d%02d-%02d%02d%02d.csv",
             localAnio, localMes, localDia, localHora, localMin, localSeg);
  } else {
    uint32_t n = prefs.getULong("nprueba", 0) + 1;
    prefs.putULong("nprueba", n);
    snprintf(destino, tam, "/SINGPS-%06lu.csv", (unsigned long)n);
  }
}

bool guardarPrueba() {
  if (!fsListo) { Serial.println("[FS] sistema de ficheros no disponible"); return false; }

  char ruta[40];
  nombreArchivoPrueba(ruta, sizeof(ruta));

  File f = LittleFS.open(ruta, FILE_WRITE);
  if (!f) { Serial.printf("[FS] no se pudo crear %s\n", ruta); return false; }

  char num[12];
  f.printf("agente,%s\n", nombreAgente[0] ? nombreAgente : "");
  f.printf("nota,%s\n", notaPrueba[0] ? notaPrueba : "");
  if (gpsFix) {
    f.printf("fecha,%02d/%02d/%04d\n", localDia, localMes, localAnio);
    f.printf("hora,%02d:%02d:%02d\n", localHora, localMin, localSeg);
    dtostrf(gpsLat, 0, 5, num); f.printf("lat,%s\n", num);
    dtostrf(gpsLon, 0, 5, num); f.printf("lon,%s\n", num);
  } else {
    f.println("fecha,");
    f.println("hora,");
    f.println("lat,");
    f.println("lon,");
  }
  f.printf("duracion_s,%d\n", tiempoPrueba);
  dtostrf(velMaxPico, 0, 1, num);      f.printf("vmax_pico_kmh,%s\n", num);
  dtostrf(velMaxMantenida, 0, 1, num); f.printf("vmax_5s_kmh,%s\n", num);

  // Curva completa. El tiempo se compone con enteros a proposito: printf("%f")
  // no es de fiar en todas las configuraciones del core.
  f.println("t_s,vel_kmh");
  for (int i = 0; i < nPuntos; i++) {
    int decimas = i * VENTANA_MS / 100;
    dtostrf(histVel[i], 0, 1, num);
    f.printf("%d.%d,%s\n", decimas / 10, decimas % 10, num);
  }
  f.close();

  Serial.printf("[FS] guardada %s  (libre %u B)\n", ruta,
                (unsigned)(LittleFS.totalBytes() - LittleFS.usedBytes()));
  return true;
}

// Entra en la fase de arranque: empieza la rampa del motor desde el 80 %.
void iniciarArranque() {
  tiempoInicioArranque = millis();
  ledcWrite(pinMotor, pwmMinMotor);   // la rampa arranca en el 80 %
}

// Entra en la fase de medida: reinicia los datos y arranca el cronometro.
void iniciarMedida() {
  velMaxPico      = 0;
  velMaxMantenida = 0;
  idxMuestra      = 0;
  numMuestras     = 0;
  nPuntos         = 0;
  tiempoInicioPrueba = millis();
}

// Guarda una muestra en el buffer circular y actualiza la velocidad maxima
// mantenida durante 5 s: si en la ventana de 5 s la velocidad se mantuvo
// dentro de una banda de +-margenVelMax, se considera sostenida y se guarda
// la mayor de esas mesetas (su valor medio).
void registrarMuestra(float kmh) {
  muestras[idxMuestra] = kmh;
  idxMuestra = (idxMuestra + 1) % N_MUESTRAS;
  if (numMuestras < N_MUESTRAS) numMuestras++;

  if (numMuestras == N_MUESTRAS) {   // ya hay 5 s completos en la ventana
    float mn = muestras[0], mx = muestras[0], suma = 0;
    for (int i = 0; i < N_MUESTRAS; i++) {
      if (muestras[i] < mn) mn = muestras[i];
      if (muestras[i] > mx) mx = muestras[i];
      suma += muestras[i];
    }
    if (mx - mn <= 2 * margenVelMax) {       // meseta estable dentro del margen
      float media = suma / N_MUESTRAS;
      if (media > velMaxMantenida) velMaxMantenida = media;
    }
  }
}

// Mediana de un array de n elementos (no modifica el original). Para n par
// devuelve la media de los dos centrales. n es pequeno: ordenacion por insercion.
float mediana(const float* v, int n) {
  if (n <= 0) return 0;
  float tmp[SUB_N];
  if (n > SUB_N) n = SUB_N;
  for (int i = 0; i < n; i++) tmp[i] = v[i];
  for (int i = 1; i < n; i++) {
    float key = tmp[i];
    int j = i - 1;
    while (j >= 0 && tmp[j] > key) { tmp[j + 1] = tmp[j]; j--; }
    tmp[j + 1] = key;
  }
  if (n & 1) return tmp[n / 2];
  return (tmp[n / 2 - 1] + tmp[n / 2]) / 2.0;
}

// Devuelve la velocidad instantanea (km/h) y las RPM del rodillo por referencia.
void leerVelocidad(float &rpm, float &kmh) {
  rpm = 0;
  kmh = 0;
  // Copia atomica de las variables que modifica la interrupcion
  noInterrupts();
  unsigned long intervaloLocal   = intervalo;
  unsigned long ultimoPulsoLocal = tiempoUltimoPulso;
  interrupts();
  if (micros() - ultimoPulsoLocal < 2000000 && intervaloLocal > 0) {
    rpm = 60000000.0 / intervaloLocal;
    kmh = (rpm * 3.14159 * DIAMETRO_RODILLO * 60.0) / 100000.0;
  }
}

// ESTADO ARRANQUE: rampa de PWM del motor. Transita a MEDICION si se alcanza
// velocidadComienzo, o a ESPERA si pasan tiempoMaxArranque segundos sin lograrlo.
void ejecutarArranque() {
  static unsigned long ultimaActualizacion = 0;
  if (millis() - ultimaActualizacion < 250) return;   // refresco cada 250 ms
  ultimaActualizacion = millis();

  float rpm, kmh;
  leerVelocidad(rpm, kmh);

  // Rampa lineal de pwmMinMotor a pwmMaxMotor en tiempoArranque segundos.
  unsigned long tArr = millis() - tiempoInicioArranque;
  unsigned long rampaMs = (unsigned long)tiempoArranque * 1000UL;
  int duty = (tArr >= rampaMs)
               ? pwmMaxMotor
               : pwmMinMotor + (int)(tArr * (pwmMaxMotor - pwmMinMotor) / rampaMs);
  ledcWrite(pinMotor, duty);

  // Transicion: velocidad de salida alcanzada -> empezar la medida
  if (kmh >= velocidadComienzo) {
    ledcWrite(pinMotor, 0);
    iniciarMedida();
    estadoActual = MEDICION;
    return;
  }
  // Transicion: no se alcanza en tiempoMaxArranque -> volver a ESPERA
  if (tArr >= (unsigned long)tiempoMaxArranque * 1000UL) {
    ledcWrite(pinMotor, 0);
    estadoActual = ESPERA;
    return;
  }

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr((128 - u8g2.getStrWidth("ARRANCANDO...")) / 2, 10, "ARRANCANDO...");
  u8g2.drawFrame(0, 12, 128, 1);
  u8g2.setFont(u8g2_font_logisoso24_tn);
  u8g2.setCursor(5, 40);
  u8g2.print(kmh, 1);
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(80, 37, "km/h");

  char buf[14];
  snprintf(buf, sizeof(buf), "RPM:%d", (int)rpm);
  u8g2.drawStr(5, 56, buf);
  int restArr = tiempoMaxArranque - (int)(tArr / 1000);   // cuenta atras del timeout
  snprintf(buf, sizeof(buf), "%02ds", restArr);
  u8g2.drawStr(108, 56, buf);
  u8g2.sendBuffer();
}

// ESTADO MEDICION: prueba cronometrada que registra los datos. Transita a
// RESULTADO al cumplirse tiempoPrueba.
void ejecutarMedida() {
  static unsigned long ultimaActualizacion = 0;
  static unsigned long ultimoSubMuestreo   = 0;
  static unsigned long ultimoInicio        = 0;
  static float subMuestras[SUB_N];
  static int   nSub = 0;

  unsigned long ahora = millis();

  // Reinicio limpio de la ventana al comenzar una prueba nueva
  if (ultimoInicio != tiempoInicioPrueba) {
    ultimoInicio = tiempoInicioPrueba;
    nSub = 0;
    ultimaActualizacion = ahora;
    ultimoSubMuestreo   = ahora;
  }

  // Sub-muestreo rapido para alimentar la mediana de la ventana
  if (ahora - ultimoSubMuestreo >= SUB_INTERVALO_MS) {
    ultimoSubMuestreo = ahora;
    float rpmInst, kmhInst;
    leerVelocidad(rpmInst, kmhInst);
    if (nSub < SUB_N) subMuestras[nSub++] = kmhInst;
  }

  // Cada 500 ms se consolida el valor oficial como la mediana de la ventana
  if (ahora - ultimaActualizacion < VENTANA_MS) return;
  if (nSub == 0) return;                    // aun sin sub-muestras en la ventana
  ultimaActualizacion = ahora;

  float kmh = mediana(subMuestras, nSub);
  nSub = 0;                                 // reiniciar la ventana
  // RPM derivadas de la velocidad ya filtrada (inversa de leerVelocidad)
  float rpm = kmh * 100000.0 / (3.14159 * DIAMETRO_RODILLO * 60.0);

  if (kmh > velMaxPico) velMaxPico = kmh;   // pico absoluto (ya filtrado)
  registrarMuestra(kmh);                     // mantenida 5 s
  if (nPuntos < MAX_PUNTOS) histVel[nPuntos++] = kmh;   // historico para la grafica

  unsigned long transcurrido = millis() - tiempoInicioPrueba;
  if (transcurrido >= (unsigned long)tiempoPrueba * 1000UL) {
    guardarPrueba();            // el historico ya esta completo en este punto
    estadoActual = RESULTADO;   // fin de la prueba -> el loop pasa a RESULTADO
    return;
  }
  int restante = tiempoPrueba - (int)(transcurrido / 1000);

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr((128 - u8g2.getStrWidth("MEDICION EN VIVO")) / 2, 10, "MEDICION EN VIVO");
  u8g2.drawFrame(0, 12, 128, 1);

  // Velocidad grande
  u8g2.setFont(u8g2_font_logisoso24_tn);
  u8g2.setCursor(5, 40);
  u8g2.print(kmh, 1);
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(80, 37, "km/h");

  // RPM del rodillo y segundos restantes
  char buf[14];
  snprintf(buf, sizeof(buf), "RPM:%d", (int)rpm);
  u8g2.drawStr(5, 51, buf);
  snprintf(buf, sizeof(buf), "%02ds", restante);
  u8g2.drawStr(108, 51, buf);

  // Barra de progreso del tiempo de la prueba (0 -> tiempoPrueba)
  int ancho = (int)(transcurrido * 124L / ((long)tiempoPrueba * 1000L));
  u8g2.drawFrame(2, 55, 124, 8);
  u8g2.drawBox(2, 55, ancho, 8);
  u8g2.sendBuffer();
}
