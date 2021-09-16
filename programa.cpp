//Programa de comunicacion con
// Seleccionamos el modem que estamos usando
#define TINY_GSM_MODEM_SIM800 // Modem SIM800L

//Configuracion de puertos serie -----------------------------------------------
//Puerto serie para debug (Serial Monitor, default BaudRate 115200)
#define SerialMon Serial
//Puerto serie para comandos AT con Modem
#define SerialAT Serial1
#define SerialModBus Serial2

//Configuración GPRS / GSM -----------------------------------------------------
//Asignamos puerto serie para debug
#define TINY_GSM_DEBUG SerialMon
//Pin del chip celular a usar
#define GSM_PIN ""
//Credenciales de la red celular
const char apn[] = " "; // Aquí APN
const char gprsUser[] = " "; // Aquí usuario
const char gprsPass[] = " "; // Aquí password
const char simPIN[]   = "";

//Configuracion MQTT -----------------------------------------------------------
const char* broker = " "; // IP publica o dominio del broker MQTT
const char* mqttUsername = ""; // MQTT Usuario
const char* mqttPassword = ""; // MQTT Password

//Topicos de entradas, salidas de tipo discretas y analógicas
const char* topicOutput1 = " "; // aqui topico de salida 1
const char* topicOutput2 = " "; // aqui topico de salida 2
const char* topicOutput3 = " "; // aqui topico de salida 3
const char* topicOutput4 = " "; // aqui topico de salida 4

const char* topicOutStatus1 = " "; // aqui topico de status salida 1
const char* topicOutStatus2 = " "; // aqui topico de status salida 2
const char* topicOutStatus3 = " "; // aqui topico de status salida 3
const char* topicOutStatus4 = " "; // aqui topico de status salida 4

const char* topicInput1 = " "; // aqui topico entrada 1
const char* topicInput2 = " "; // aqui topico entrada 2
const char* topicInput3 = " "; // aqui topico entrada 3
const char* topicInput4 = " "; // aqui topico entrada 4

const char* topicAnalog1 = " "; //aqui topico entrada analógica 1
const char* topicAnalog2 = " "; //aqui topico entrada analógica 2

//Topicos de comunicacion industrial Modbus RTU
const char* topicModbusWrite = " "; aqui topico de escritura en registro holding modbus
const char* topic_freq_ref = " "; //topico de lectura de frecuencia 
const char* topic_freq_out = " "; //topico de lectura de frecuencia de salida
const char* topic_current = " "; //topico de lectura de corriente
const char* topic_volt_out = " "; //topico de lectura de voltaje
const char* topic_falla = " "; //topico de lectura de falla
const char* topic_stw = " "; //topico de lectura de estado


//Incluimos librerias adicionales requeridas
#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

#include <PubSubClient.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_BME280.h>

TinyGsmClient client(modem);
PubSubClient mqtt(client);

// TTGO T-Call Pines de conexión con SIM800L en tarjeta ------------------------
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26

#define OUTPUT_1             19
#define OUTPUT_2             18
#define OUTPUT_3             13
#define OUTPUT_4             25

#define INPUT_1             35
#define INPUT_2             34
#define INPUT_3             39
#define INPUT_4             36

#define HAB                 12

bool inStatus_1 = LOW;
bool inStatus_2 = LOW;
bool inStatus_3 = LOW;
bool inStatus_4 = LOW;

int OutStatus_1 = LOW;
int OutStatus_2 = LOW;
int OutStatus_3 = LOW;
int OutStatus_4 = LOW;

//------------------------------------------------------------------------------
uint32_t lastReconnectAttempt = 0;      //

// Variables analógicas --------------------------------------------------------
float Analog1 = 0;
float Analog2 = 0;
long lastMsg = 0;

//MODBUS RTU -------------------------------------------------------------------
#include <ModbusMaster.h>
//Configuracion de puertos seriales
//Instanciamos objeto
ModbusMaster nodoMB;

//Declaracion de pines Com ModbusRTU
#define MAX485_DE      15
#define MAX485_RE_NEG  14
#define MBUSRTU_RX     0
#define MBUSRTU_TX     2

//variable para almacenar resultado de registro holding
uint8_t result;
uint16_t data;
uint16_t freq_ref, freq_out, current, volt_out, fault, StateWord;
char frec_referencia[8], frec_salida[8], corriente[8], volt_salida[8],
    falla[2], estadoWord[8];

//funciones de transferencia ModbusRTU -----------------------------------------
void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

//Funcion Listener de MQTTT espera llegada de los topicos inscritos ------------
void mqttCallback(char* topic, byte* message, unsigned int len) {
  Serial.print("Llego mensaje en topico: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < len; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();


//Si se recibe un mensaje en el topico esp/output1, revisamos si el mensaje es verdadero o falso.
//Cambiamos el estado de la salida de acuerdo al mensaje -----(cambiar Serial por SerialMon)
  if (String(topic) == topicOutput1) {
    if (messageTemp == "true") {
      digitalWrite(OUTPUT_1, HIGH);
      mqtt.publish(topicOutStatus1, "1");
    }else if(messageTemp == "false"){
      digitalWrite(OUTPUT_1, LOW);
      mqtt.publish(topicOutStatus1, "0");
    }
  }

  if (String(topic) == topicOutput2) {
    if (messageTemp == "true") {
      digitalWrite(OUTPUT_2, HIGH);
      mqtt.publish(topicOutStatus2, "1");
    }else if(messageTemp == "false"){
      digitalWrite(OUTPUT_2, LOW);
      mqtt.publish(topicOutStatus2, "0");
    }
  }

  if (String(topic) == topicOutput3) {
    if (messageTemp == "true") {
      digitalWrite(OUTPUT_3, HIGH);
      mqtt.publish(topicOutStatus3, "1");
    }else if(messageTemp == "false"){
      digitalWrite(OUTPUT_3, LOW);
      mqtt.publish(topicOutStatus3, "0");
    }
  }

  if (String(topic) == topicOutput4) {
    if (messageTemp == "true") {
      digitalWrite(OUTPUT_4, HIGH);
      mqtt.publish(topicOutStatus4, "1");
    }else if(messageTemp == "false"){
      digitalWrite(OUTPUT_4, LOW);
      mqtt.publish(topicOutStatus4, "0");
    }
  }

//Escritura en registro holding
  if (String(topic) == topicModbusWrite) {
    //variables para conversion de datos
    char registro[messageTemp.length()+1];
    String regHolding;
    String datoHolding;
    int separ;
    int inicio;
    int fin;

    //convertimos el mensaje en un array de string y lo almacenamos en registro
    messageTemp.toCharArray(registro,messageTemp.length()+1);

    inicio = String(registro).indexOf("[");
    separ = String(registro).indexOf(",");
    fin = String(registro).indexOf("]");

    if (inicio != -1 && separ != -1 && fin  != -1 ) {
      for (int i = inicio+1; i < separ; i++) {
        regHolding = regHolding + String(registro[i]);
      }

      for (int i = separ+1; i < fin; i++) {
        datoHolding = datoHolding + String(registro[i]);
      }
      result = nodoMB.writeSingleRegister(regHolding.toInt()-1, datoHolding.toInt());
    } else {
      SerialMon.println("registro modbus recibido erroneo...");
    }
  }
}


//------------------------------------------------------------------------------
// Funcion de conexión a broker MQTT
boolean mqttConnect() {
  SerialMon.print("Conectando a: ");
  SerialMon.println(broker);

  //Conexión con broker MQTT sin autentificación
  //boolean status = mqtt.connect("GsmClientN");

  //Conexión con broker MQTT con autentificación
  boolean status = mqtt.connect("GsmClientN", mqttUsername, mqttPassword);

  if (status == false) {
    SerialMon.println(" Falla de conexion");
    ESP.restart();
    return false;
  }
  //Conexion creada correctamente y nos suscribimos a 4 topicos
  SerialMon.println(" conectado con exito");
  mqtt.subscribe(topicOutput1);
  mqtt.subscribe(topicOutput2);
  mqtt.subscribe(topicOutput3);
  mqtt.subscribe(topicOutput4);
  mqtt.subscribe(topicModbusWrite);

  return mqtt.connected();
}

// Void de configuracion -------------------------------------------------------
void setup() {
  // Set console baud rate de puerto serial Monitor
  SerialMon.begin(115200);
  delay(10);

  //Configuramos los pines del Modem Modem PwrKey / Reset y Power On
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  //Configuramos los pines de salidas discretas del dispositivo
  pinMode(OUTPUT_1, OUTPUT);
  pinMode(OUTPUT_2, OUTPUT);
  pinMode(OUTPUT_3, OUTPUT);
  pinMode(OUTPUT_4, OUTPUT);

  //Configuramos los pines de entradas discretas del dispositivo
  pinMode(INPUT_1, INPUT);
  pinMode(INPUT_2, INPUT);
  pinMode(INPUT_3, INPUT);
  pinMode(INPUT_4, INPUT);

  SerialMon.println("Espere...");

  //Configuramos el puerto de comunicación SerialAT para el modem
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(6000);

  //La restauracion lleva un tiempo
  //para no demorar tanto, llamar init() en lugar de restart()
  SerialMon.println("Inicializando modem...");
  modem.restart();
  // modem.init();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Informacion de modem: ");
  SerialMon.println(modemInfo);

  //Desbloque SIM Pin si es necesario
  if ( GSM_PIN && modem.getSimStatus() != 3 ) {
    modem.simUnlock(GSM_PIN);
  }

  //Conectamos a la APN
  SerialMon.print("Conectando a APN: ");
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" Falla de conexion con APN");
    ESP.restart();
  }
  else {
    SerialMon.println(" OK");
  }

  if (modem.isGprsConnected()) {
    SerialMon.println("GPRS conectado");
  }

  //SetUp del broker MQTT
  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);
  delay(1000);

  //----------------------------------------------------------------------------
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);

  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  // Comunicacion modbus a 9600 baudios
  SerialModBus.begin(9600, SERIAL_8E1, MBUSRTU_RX, MBUSRTU_TX);

  // Modbus slave ID 2
  nodoMB.begin(2, SerialModBus);

  // Callbacks de configuracion del RS485
  nodoMB.preTransmission(preTransmission);
  nodoMB.postTransmission(postTransmission);

  //Habilita entradas y salidas
  pinMode(HAB, OUTPUT);
  digitalWrite(HAB, true);
}

void loop() {
  //---------------------------------------------------------
  //Confirmamos conexion al broker MQTT
  if (!mqtt.connected()) {
    SerialMon.println("=== MQTT NO CONECTADO ===");
    // Reconnect every 10 seconds
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) {
        lastReconnectAttempt = 0;
      }
    }
    delay(100);
    return;
  }

  //Publicación en topicos de entradas digitales ---------------------------------
    if(inStatus_1 != digitalRead(INPUT_1)){
        Serial.print("Escritura en topico: ");
        Serial.print(topicInput1);
        Serial.print(": ");
        if(digitalRead(INPUT_1) == true){
          Serial.println("true");
          mqtt.publish(topicInput1, "true");
          inStatus_1 = true;
        }else{
          Serial.println("false");
          mqtt.publish(topicInput1, "false");
          inStatus_1 = false;
        }
    }

    if(inStatus_2 != digitalRead(INPUT_2)){
        Serial.print("Escritura en topico: ");
        Serial.print(topicInput2);
        Serial.print(": ");
        if(digitalRead(INPUT_2) == true){
          Serial.println("true");
          mqtt.publish(topicInput2, "true");
          inStatus_2 = true;
        }else{
          Serial.println("false");
          mqtt.publish(topicInput2, "false");
          inStatus_2 = false;
        }
    }

    if(inStatus_3 != digitalRead(INPUT_3)){
        Serial.print("Escritura en topico: ");
        Serial.print(topicInput3);
        Serial.print(": ");
        if(digitalRead(INPUT_3) == true){
          Serial.println("true");
          mqtt.publish(topicInput3, "true");
          inStatus_3 = true;
        }else{
          Serial.println("false");
          mqtt.publish(topicInput3, "false");
          inStatus_3 = false;
        }
    }

    if(inStatus_4 != digitalRead(INPUT_4)){
        Serial.print("Escritura en topico: ");
        Serial.print(topicInput4);
        Serial.print(": ");
        if(digitalRead(INPUT_4) == true){
          Serial.println("true");
          mqtt.publish(topicInput4, "true");
          inStatus_4 = true;
        }else{
          Serial.println("false");
          mqtt.publish(topicInput4, "false");
          inStatus_4 = false;
        }
    }

//Publicación en topicos analogicos con interrupción cada 30 segundos-----------
  long now = millis();
  if (now - lastMsg > 10000) {
    lastMsg = now;

    //Envio de datos analogicos -------------------------
    //Lectura de variables analógicas
    int analog1 = analogRead(32);
    int analog2 = analogRead(33);

    //Convertimos el valor a char array
    char analog1String[8];
    dtostrf(analog1, 1, 0, analog1String);
    Serial.print("Lectura analogica 1: ");
    Serial.println(analog1String);
    mqtt.publish(topicAnalog1, analog1String);

    //Convertimos el valor a char array
    char analog2String[8];
    //dtostrf convierte float en string
    dtostrf(analog2, 1, 0, analog2String);
    Serial.print("Lectura analogica 2: ");
    Serial.println(analog2String);
    mqtt.publish(topicAnalog2, analog2String);

    //Envio de datos holding ----------------------------
    result = nodoMB.readHoldingRegisters(30, 1);
    if (result == nodoMB.ku8MBSuccess){
      data = nodoMB.getResponseBuffer(0);
      freq_ref = data;
      dtostrf(freq_ref, 1, 0, frec_referencia);
      mqtt.publish(topic_freq_ref, frec_referencia);
    }

    result = nodoMB.readHoldingRegisters(23, 1);
    if (result == nodoMB.ku8MBSuccess){
      data = nodoMB.getResponseBuffer(0);
      freq_out = data;
      dtostrf(freq_out, 1, 0, frec_salida);
      mqtt.publish(topic_freq_out, frec_salida);
    }

    result = nodoMB.readHoldingRegisters(25, 1);
    if (result == nodoMB.ku8MBSuccess){
    data = nodoMB.getResponseBuffer(0);
      current = data;
      dtostrf(current, 1, 0, corriente);
      mqtt.publish(topic_current, corriente);
    }

    result = nodoMB.readHoldingRegisters(32, 1);
    if (result == nodoMB.ku8MBSuccess){
      data = nodoMB.getResponseBuffer(0);
      volt_out = data;
      dtostrf(volt_out, 1, 0, volt_salida);
      mqtt.publish(topic_volt_out, volt_salida);
    }

    result = nodoMB.readHoldingRegisters(53, 1);
    if (result == nodoMB.ku8MBSuccess){
      data = nodoMB.getResponseBuffer(0);
      fault = data;
      dtostrf(fault, 1, 0, falla);
      mqtt.publish(topic_falla, falla);
    }

    result = nodoMB.readHoldingRegisters(99, 1);
    if (result == nodoMB.ku8MBSuccess){
      data = nodoMB.getResponseBuffer(0);
      StateWord = data;
      dtostrf(StateWord, 1, 0, estadoWord);
      mqtt.publish(topic_stw, estadoWord);
    }
  }

  mqtt.loop();
}
