#include <Arduino.h>
#include <CAN.h>
#include <CANSAME5x.h>
#include "amkfse.h"
#include "ERRORCODES.h"

CANSAME5x CAN;

// Variabili per l'invio dei messaggi CAN
static uint8_t can_msg[8] = {};
// uint16_t control = SET_CONTROL; // AMK_Control, Unsigned, 2 Byte
//  set dei bit di AMK_Control (0000000011100000)
/* control |= AMK_DC_ON;
control |= AMK_ENABLE;
control |= AMK_INVERTER_ON;
control &= AMK_ERROR_SET_OFF; */
uint16_t control = 224;

int16_t target_velocity = 45;       // AMK_TargetVelocity, Signed, 2 Byte
int16_t torque_limit_positive = 3; // AMK_TorqueLimitPositiv, Signed, 2 Byte
int16_t torque_limit_negative = 3; // AMK_TorqueLimitNegativ, Signed, 2 Byte

// Variabili per la ricezione dei messaggi CAN
uint8_t Actual1[8];
uint8_t Actual2[8];
uint16_t status;
uint16_t ActualVelocity;
uint16_t TorqueCurrent;
uint16_t MagnetCurrent;
int16_t TempMotor;
int16_t TempInverter;
uint16_t ErrorInfo;
int16_t TempIGBT;

// dichiarazione delle funzioni
uint8_t *build_message(uint16_t, int16_t, int16_t, int16_t);
bool send_message(uint8_t[], const int);
void print_received_message(int);
void receive_message(int);
void printError(error_codes::Error);
error_codes::Error validateError(uint16_t);

void setup()
{
  // ============================AVVIO DELLA COMUNICAZIONE============================
  Serial.begin(9600);
  // while (!Serial);
  Serial.println("CAN Receiver - Transmitter");

  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, false); // turn off STANDBY
  pinMode(PIN_CAN_BOOSTEN, OUTPUT);
  digitalWrite(PIN_CAN_BOOSTEN, true); // turn on booster

  // start the CAN bus at 500 kbps
  if (!CAN.begin(500E3))
  {
    Serial.println("Starting CAN failed!");
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
    Serial.println("DonE SENDING!");
  else
    Serial.println("Error occurred while sending");
}

// viene richiamata ogni qualvolta che viene ricevuto un messaggio CAN
void receive_message(int packetSize)
{
  Serial.println("Received CAN packet ...");
  long packet_ID = CAN.packetId();
  int isActual1 = 0;
  switch (packet_ID)
  {
  case AMK_INVERTER_1_ACTUAL_VALUES_1:
    Serial.print(" Actual Values 1 received from node 1 " + (int)AMK_INVERTER_1_NODE_ADDRESS);
    isActual1 = 1;
    break;
  case AMK_INVERTER_1_ACTUAL_VALUES_2:
    Serial.print("Actual Values 2 received from node 1 " + (int)AMK_INVERTER_2_NODE_ADDRESS);
    isActual1 = 2;
    break;
  case AMK_INVERTER_2_ACTUAL_VALUES_1:
    Serial.print(" Actual Values 1 received from node 2" + (int)AMK_INVERTER_2_NODE_ADDRESS);
    isActual1 = 1;
    break;
  case AMK_INVERTER_2_ACTUAL_VALUES_2:
    Serial.print("Actual Values 2 received from node 2 "+ (int)AMK_INVERTER_2_NODE_ADDRESS);
    isActual1 = 2;
    break;
  default:
    Serial.println("Unknown packet ID");
    break;
  }

  int i = 0;
  while (CAN.available())
  {
    Actual1[i] = CAN.read();
    i++;
  }

  Serial.print("control=  ");
  if (isActual1 == 1)
  {
    // Lettura actual values 1
    Serial.printf("%d velocita= ", Actual1[0]);
    Serial.print(" ");
    ActualVelocity = Actual1[3] << 8 | Actual1[2];
    Serial.printf("%d torque current =", ActualVelocity);
    Serial.print(" ");
    TorqueCurrent = Actual1[5] << 8 | Actual1[4];
    Serial.printf("%d magnet current= ", TorqueCurrent);
    Serial.print("  ");
    MagnetCurrent = Actual1[7] << 8 | Actual1[6];
    Serial.printf("%d ", MagnetCurrent);
    Serial.printf("");
  }
  else if (isActual1 == 2)
  {
    // Lettura actual values 2
    TempMotor = Actual2[1] << 8 | Actual2[0];
    Serial.printf("%d temperaturaMot=", TempMotor);
    Serial.print(" ");
    TempInverter = Actual2[3] << 8 | Actual2[2];
    Serial.printf("%d tempInv =", TempInverter);
    Serial.print(" ");
    ErrorInfo = Actual2[5] << 8 | Actual2[4];
    Serial.printf("%d error=", ErrorInfo);
    Serial.print(" ");
    TempIGBT = Actual2[7] << 8 | Actual2[6];
    Serial.printf("%d IGBT =", TempIGBT);
  }
  Serial.println();

#ifdef DEBUG_CAN
  Serial.printf("ID=%x\tDLC=%d\t", CAN.packetId(), CAN.packetDlc());
  for (int i = 0; i < CAN.packetDlc(); i++)
  {
    Serial.printf("0x%02X ", buffer[i]);
  }
  Serial.println();
#endif
}

uint8_t *build_message(uint16_t control, int16_t target_velocity, int16_t torque_limit_positive, int16_t torque_limit_negative)
{
  // (Formato Little Endian)
  // inserimento dei Byte di AMK_Control all'interno del messaggio che verrà spedito via CAN
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
  Serial.println("Sending packet to ... " + INVERTER_X_SETPOINT_ADDRESS);
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
    Serial.println("error occurred!");
    return false;
  }
}