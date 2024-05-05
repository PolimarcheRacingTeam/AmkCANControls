#include <Arduino.h>
#include <CAN.h>
#include <CANSAME5x.h>
#include "amkfse.h"
#include "ERRORCODES.h"

CANSAME5x CAN;

static uint8_t can_msg[8] = {}; // vettore per immagazzinare i setpoint
// AMK Setpoints 1
uint16_t control = SET_NULL; // AMK_Control, Unsigned, 2 Byte
int16_t target_velocity = 45;       // AMK_TargetVelocity, Signed, 2 Byte
int16_t torque_limit_positive = 3; // AMK_TorqueLimitPositiv, Signed, 2 Byte
int16_t torque_limit_negative = -3; // AMK_TorqueLimitNegativ, Signed, 2 Byte

// Variabili per la ricezione dei messaggi CAN
uint8_t Actual[8] = {}; // vettore per immagazzinare i messaggi Actual Values 1 o 2
//  AMK Actual Values 1
uint16_t Status;  // AMK_Status
int16_t ActualVelocity; //  AMK_ActualVelocity
int16_t TorqueCurrent;  //  AMK_TorqueCurrent
int16_t MagnetCurrent;  //  AMK_MagnetCurrent
// AMK Actual Values 2
int16_t TempMotor;  // AMK_TempMotor
int16_t TempInverter; //  AMK_TempInverter
uint16_t ErrorInfo; //  AMK_ErrorInfo
int16_t TempIGBT; //  AMK_TempIGBT

// dichiarazione delle funzioni
uint8_t *build_message(uint16_t, int16_t, int16_t, int16_t);
bool send_message(uint8_t[], const int);
void print_received_message(int);
void receive_message(int);
// void printError(error_codes::Error);
// error_codes::Error validateError(uint16_t);

void setup()
{
  //  set dei bit di AMK_Control (0000000011100000)
  control |= AMK_DC_ON;
  control |= AMK_DRIVER_ENABLE;
  control |= AMK_INVERTER_ON;
  control &= AMK_ERROR_SET_OFF;
  // ============================AVVIO DELLA COMUNICAZIONE============================
  Serial.begin(9600);
  // while (!Serial);
  Serial.printf("CAN Receiver - Transmitter\n");

  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, false); // turn off STANDBY
  pinMode(PIN_CAN_BOOSTEN, OUTPUT);
  digitalWrite(PIN_CAN_BOOSTEN, true); // turn on booster

  // start the CAN bus at 500 kbps
  if (!CAN.begin(500E3))
  {
    Serial.printf("Starting CAN failed!\n");
    // se la comunicazione non avviene correttamente il led lampeggia
    while (1)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
  }

  // ============================RICEZIONE CAN============================
  // quando viene ricevuto un messaggio si imposta come funzione da eseguire la receive_message()
  CAN.onReceive(receive_message);
}

void loop()
{
  
  // ============================INVIO CAN============================
  uint8_t Setpoint[8] = {0};
  uint8_t *aux = build_message(control, target_velocity, torque_limit_positive, torque_limit_negative);
  for (int i = 0; i < 8; i++)
  {
    Setpoint[i] = aux[i];
  }

  // controllo se i messaggi sono stati inviati correttamente
  if (send_message(Setpoint, AMK_INVERTER_1_SETPOINTS_1) && send_message(Setpoint, AMK_INVERTER_2_SETPOINTS_1))
    Serial.printf("Done Sendind!\n");
  else
    Serial.printf("Error occurred while sending\n");
}

// viene richiamata ogni qualvolta che viene ricevuto un messaggio CAN
void receive_message(int packetSize)
{
  Serial.printf("Received CAN packet ...\n");
  long packet_ID = CAN.packetId();
  bool isActual1 = false;
  bool isActual2 = false;
  switch (packet_ID)
  {
  case AMK_INVERTER_1_ACTUAL_VALUES_1:
    Serial.printf("Actual Values 1 received from node 1: %d\n", (int)AMK_INVERTER_1_NODE_ADDRESS);
    isActual1 = true;
    break;
  case AMK_INVERTER_1_ACTUAL_VALUES_2:
    Serial.printf("Actual Values 2 received from node 1: %d\n ", (int)AMK_INVERTER_2_NODE_ADDRESS);
    isActual2 = true;
    break;
  case AMK_INVERTER_2_ACTUAL_VALUES_1:
    Serial.printf("Actual Values 1 received from node 2: %d\n", (int)AMK_INVERTER_2_NODE_ADDRESS);
    isActual1 = true;
    break;
  case AMK_INVERTER_2_ACTUAL_VALUES_2:
    Serial.printf("Actual Values 2 received from node 2: %d\n", (int)AMK_INVERTER_2_NODE_ADDRESS);
    isActual2 = false;
    break;
  default:
    Serial.printf("Unknown packet ID\n");
    break;
  }

  int i = 0;
  while (CAN.available())
  {
    Actual[i] = CAN.read();
    i++;
  }

  if (isActual1)
  {
    // lettura Actual Values 1
    Status = (Actual[1] << 8) | Actual[0];
    Serial.printf("AMK_Status: %d\n", Status);
    ActualVelocity = (Actual[3] << 8) | Actual[2];
    Serial.printf("AMK_ActualVelocity: %d\n", ActualVelocity);
    TorqueCurrent = (Actual[5] << 8) | Actual[4];
    Serial.printf("AMK_TorqueCurrent: %d\n", TorqueCurrent);
    MagnetCurrent = (Actual[7] << 8) | Actual[6];
    Serial.printf("AMK_MagnetCurrent: %d\n", MagnetCurrent);
  }
  else if (isActual2)
  {
    //Lettura Actual Values 2
    TempMotor = (Actual[1] << 8) | Actual[0];
    Serial.printf("AMK_TempMotor: %d\n",TempMotor);
    TempInverter = (Actual[3] << 8) | Actual[2];
    Serial.printf("AMK_TempInverter: %d\n", TempInverter);
    ErrorInfo = (Actual[5] << 8) | Actual[4];
    Serial.printf("AMK_ErrorInfo: %d\n", ErrorInfo);
    TempIGBT = (Actual[7] << 8) | Actual[6];
    Serial.printf("AMK_TempIGBT: %d\n", TempIGBT);
  }
  Serial.printf("READING DONE!\n");

#ifdef DEBUG_CAN
  Serial.printf("ID=%x\tDLC=%d\t", CAN.packetId(), CAN.packetDlc());
  for (int i = 0; i < CAN.packetDlc(); i++)
  {
    Serial.printf("0x%02X ", buffer[i]);
  }
  Serial.printf('\n');
#endif
}

uint8_t *build_message(uint16_t control, int16_t target_velocity, int16_t torque_limit_positive, int16_t torque_limit_negative)
{
  // (Formato Little Endian)
  // inserimento dei Byte di AMK_Control all'interno del messaggio che verrà spedito via CAN

  /*
  es.
  control = 6000 = 00010111 01110000
  can_msg[1] = 00010111 = 23
  can_msg[0] = 01110000 = 28672
  */
  can_msg[1] = (control >> 8) & 0xFF;
  can_msg[0] = control & 0xFF;

  // inserimento dei Byte di AMK_TargetVelocity all'interno del messaggio che verrà spedito via CAN
  can_msg[3] = (target_velocity >> 8) & 0xFF;
  can_msg[2] = target_velocity & 0xFF;
  // inserimento dei Byte di AMK_TorqueLimitPositive all'interno del messaggio
  can_msg[5] = (torque_limit_positive >> 8) & 0xFF;
  can_msg[4] = torque_limit_positive & 0xFF;

  // inserimento dei Byte di AMK_TorqueLimitNegative all'interno del messaggio
  can_msg[7] = (torque_limit_negative >> 8) & 0xFF;
  can_msg[6] = torque_limit_negative & 0xFF;

  return can_msg;
}

bool send_message(uint8_t message[8], const int INVERTER_X_SETPOINT_ADDRESS)
{
  Serial.printf("Sending packet to ... %d",  INVERTER_X_SETPOINT_ADDRESS);
  CAN.beginPacket(INVERTER_X_SETPOINT_ADDRESS); // indirizzo di arrivo del pacchetto
  size_t bytesSent = CAN.write(message, 8);
  CAN.endPacket();

  // Controllo del corretto invio del pacchetto
  if (bytesSent == sizeof(can_msg))
  {
    return true;
  }
  else
  {
    Serial.printf("Error occurred!\n");
    return false;
  }
}