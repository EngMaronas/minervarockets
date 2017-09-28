/*  Code written by Marcelo Maroñas @ Minerva Rockets (Federal University of Rio de Janeiro Rocketry Team) - September 6, 2017
 *  This is a basic example that uses the function Get_IMU to store the GY-80 variables in a structure.
 *  You can choose to print values to debug and test in the serial monitor.
 *  The data is printed in a CSV way, so you can copy and paste the serial monitor info into a notepad file and save as a CSV that can be opened in Excel or other CSV softwares.
 *  The structure IMU_s is given by :
 *      IMU_s->double acelerometro[2]; Where positions 0, 1 and 2 in the array are acelerometer x, y and z values respectively, in m/s².
 *      IMU_s->int magnetometro[2]; Where positions 0, 1 and 2 in the array are magnetic field x, y and z values respectively, in vector form.
 *      IMU_s->int giroscopio[2]; Where positions 0, 1 and 2 in the array are gyroscope x, y and z values respectively, in angular acceleration.
 *      IMU_s->double barometro[2]; Where positions 0, 1 and 2 in the array are pressure(in Pa), altitude(in Meters) and temperature(in Celsius) respectively.    
 *  Contact : marcelomaronas at poli.ufrj.br
 *  For more codes : github.com/engmaronas
 */

/* GY-80 Pins
 *  Vcc_In <----------------------> Arduino 5v
 *  Gnd    <----------------------> Arduino Gnd
 *  SDA    <----------------------> A4
 *  SCL    <----------------------> A5
 */
 
#include <GY80IMU.h> //Include the library GY80IMU
#include <SPI.h>
#include <NMEAGPS.h>
#include <GPSport.h>
//#include <SD.h>
#include <RH_RF95modificado.h>
#include <EEPROM.h>

#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3
#define EEPROMADDRESS 12
// Frequência do LoRa! [COUBRUF 2017: USAR 915.0]
#define RF95_FREQ 915.0


//File myFile;
//String filename = "DADOS0.CSV";
//int numArquivo = 0;


//Structure declaration
  IMU_s IMUs1; 
  IMU_s *pIMUs1 = &IMUs1;

  NMEAGPS  gps; // This parses the GPS characters
  gps_fix  fix; // This holds on to the latest values

//Use this variables to enable debugging via Serial Monitor
bool DebugSerial = 0;  //Prints the GY80 values from inside the "Get_IMU()" function
bool DebugSDSerial = 0; //Prints the values stored in the structure IMU_s

//Modify the Delay_Time variable to control how much info is printed on the serial monitor
float Delay_Time = 0;

//Free ram function Init
int freeRam () 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
//Free ram function End

// Criacao do objeto da classe RH_RF95
RH_RF95 rf95(RFM95_CS, RFM95_INT);

float latf, longf, altf;




void setup() {
  pinMode(RFM95_CS,OUTPUT);
//  pinMode(7, OUTPUT);
//  digitalWrite(7, LOW);
  DEBUG_PORT.begin(9600);
  gpsPort.begin(9600);
  while (!Serial);
  DEBUG_PORT.print( F("NMEAsimple.INO: started\n") );
  Wire.begin();

  //
  Serial.println(F("GY-80 IMU library "));
  Serial.println(F("by Marcelo Maronas"));
  Serial.println();
  //

  Serial.println(F("sep =, ")); //This line handles Excel CSV configuration.
  Serial.println(F("Time, Pressure, Altitude, Temperature, AcelX, AcelY, AcelZ, GyroX, GyroY, GyroZ, MagnetoX, MagnetoY, MagnetoZ, GPS Lat, GPS Lon, Satellites, GPS Altitude, GPS Speed, Free Ram"));
   
//   // initialize the SD card
//  if (!SD.begin(10)) {
//    Serial.println(F("initialization failed!"));
//    return;
//  }
//  Serial.println(F("initialization done."));
// 
//  for (int ff = 0; ff < 100; ff++){
//      numArquivo++;
//      filename = "DADOS" + String(numArquivo, DEC) + ".CSV";
//      if (!SD.exists(filename)){
//        break;
//      }
//   }
//
//  myFile = SD.open(filename, FILE_WRITE);
//  
//  if (myFile) {
//  myFile.println(F("sep =, ")); //This line handles Excel CSV configuration.
//  myFile.println(F("Time, Pressure, Altitude, Temperature, AcelX, AcelY, AcelZ, GyroX, GyroY, GyroZ, MagnetoX, MagnetoY, MagnetoZ, GPS Lat, GPS Lon, Satellites, GPS Altitude, GPS Speed, Free Ram"));
//  myFile.close();
//   } else {
//    // if the file didn't open, print an error:
//    Serial.println(F("error opening test.txt"));
//  }
//  digitalWrite(7, HIGH);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  delay(100);
  
  // Reinicialização Manual
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) { 
    Serial.println(F("Lora nao inicializado")); 
    }
  rf95.setFrequency(RF95_FREQ);
  
  //Definindo a potência da transmissao para 23dBm
  rf95.setTxPower(23, false);
}

//______________________________________________________________________________

void loop() {  
  //Stores the GY80 values into the pIMUs1 pointer
  digitalWrite(RFM95_CS,LOW);
    Get_IMU(pIMUs1, DebugSerial);
    while (gps.available( gpsPort )) {
    fix = gps.read();

    DEBUG_PORT.print( F("Location: ") );
    if (fix.valid.location) {
      latf = fix.latitude();
      Serial.print( latf, 6 );
      Serial.print( ',' );
      longf = fix.longitude();
      Serial.print( longf, 6 );
    }

    DEBUG_PORT.print( F(", Altitude: ") );
    if (fix.valid.altitude){
      altf = fix.altitude();
      Serial.print( altf );
    }
    DEBUG_PORT.println();
  }
//  digitalWrite(7, LOW);
//  myFile = SD.open(filename, FILE_WRITE);
//  //Delay time
//   delay(Delay_Time);
//
//     // if the file opened okay, write to it:
//  if (myFile) {
//    myFile.print(millis());myFile.print(F(" ,"));
//    myFile.print(pIMUs1->barometro[0]);myFile.print(F(" ,"));
//    myFile.print(pIMUs1->barometro[1]);myFile.print(F(" ,"));
//    myFile.print(pIMUs1->barometro[2]);myFile.print(F(" ,"));
//    myFile.print(pIMUs1->acelerometro[0]);myFile.print(F(" ,"));
//    myFile.print(pIMUs1->acelerometro[1]);myFile.print(F(" ,"));
//    myFile.print(pIMUs1->acelerometro[2]);myFile.print(F(" ,"));
//    myFile.print(pIMUs1->giroscopio[0]);myFile.print(F(" ,"));
//    myFile.print(pIMUs1->giroscopio[1]);myFile.print(F(" ,"));
//    myFile.print(pIMUs1->giroscopio[2]);myFile.print(F(" ,"));
//    myFile.print(pIMUs1->magnetometro[0]);myFile.print(F(" ,"));
//    myFile.print(pIMUs1->magnetometro[1]);myFile.print(F(" ,"));
//    myFile.print(pIMUs1->magnetometro[2]);myFile.print(F(" ,"));
//    myFile.print(fix.latitude(), 6);myFile.print(F(" ,"));
//    myFile.print(fix.longitude(), 6);myFile.print(F(" ,"));
//    myFile.print(F("Not available"));myFile.print(F(" ,"));
//    myFile.print(fix.altitude(), 3);myFile.print(F(" ,"));
//    myFile.print(F("Not available"));myFile.print(F(" ,"));
//    myFile.println(freeRam());
//    myFile.close();
//    Serial.println(F("Dados gravados 3"));
//  }
//  else {
//    // if the file didn't open, print an error:
//    Serial.println(F("error opening test.txt"));
//  }
//  digitalWrite(7, HIGH);
//    Serial.println("lol0"); 
//  //if DebugSDSerial = 1, write into the Serial Monitor
//  if (DebugSDSerial) {
//    Serial.print(millis());Serial.print(F(" ,"));
//    Serial.print(pIMUs1->barometro[0]);Serial.print(F(" ,"));
//    Serial.print(pIMUs1->barometro[1]);Serial.print(F(" ,"));
//    Serial.print(pIMUs1->barometro[2]);Serial.print(F(" ,"));
//    Serial.print(pIMUs1->acelerometro[0]);Serial.print(F(" ,"));
//    Serial.print(pIMUs1->acelerometro[1]);Serial.print(F(" ,"));
//    Serial.print(pIMUs1->acelerometro[2]);Serial.print(F(" ,"));
//    Serial.print(pIMUs1->giroscopio[0]);Serial.print(F(" ,"));
//    Serial.print(pIMUs1->giroscopio[1]);Serial.print(F(" ,"));
//    Serial.print(pIMUs1->giroscopio[2]);Serial.print(F(" ,"));
//    Serial.print(pIMUs1->magnetometro[0]);Serial.print(F(" ,"));
//    Serial.print(pIMUs1->magnetometro[1]);Serial.print(F(" ,"));
//    Serial.print(pIMUs1->magnetometro[2]);Serial.print(F(" ,"));
//    Serial.print(fix.latitude(), 6);Serial.print(F(" ,"));
//    Serial.print(fix.longitude(), 6);Serial.print(F(" ,"));
//    Serial.print(F("Not available"));Serial.print(F(" ,"));
//    Serial.print(fix.altitude(), 3);Serial.print(F(" ,"));
//    Serial.print(F("Not available"));Serial.print(F(" ,"));
//    Serial.println(freeRam());
//}
  Serial.println("lol1");
  //Verifica o numero do pacote que sera enviado 
  //Assim, eh possivel garantir que o numero do pacote
  //enviado sera sempre o numero certo
  long packetnum = EEPROMReadlong(EEPROMADDRESS);
  digitalWrite(RFM95_CS,LOW);
  //PACOTE A SER ENVIADO
  // OBS: os dados que sao floats passam por uma conversao manual em bytes para
  // serem adicionados ao radiopacket
  
  union u_tag {
    float b;
    uint8_t a[4];
    } u;

  float t; 
  
  uint8_t radiopacket[73];

  radiopacket[0] = (uint8_t)'M';
  radiopacket[1] = (uint8_t)'N';
  radiopacket[2] = (uint8_t)'R';
  radiopacket[3] = (uint8_t)'V';


  t = (float)packetnum;

  u.b = t;
  
  radiopacket[4] = (u.a[0]);
  radiopacket[5] = (u.a[1]);
  radiopacket[6] = (u.a[2]);
  radiopacket[7] = (u.a[3]);

  u.b = (float)millis();
  
  radiopacket[8] = (u.a[0]);
  radiopacket[9] = (u.a[1]);
  radiopacket[10] = (u.a[2]);
  radiopacket[11] = (u.a[3]);

      
  u.b = latf;

  radiopacket[12] = u.a[0];
  radiopacket[13] = u.a[1];
  radiopacket[14] = u.a[2];
  radiopacket[15] = u.a[3];

  u.b = longf;
  
  radiopacket[16] = u.a[0];
  radiopacket[17] = u.a[1];
  radiopacket[18] = u.a[2];
  radiopacket[19] = u.a[3];

  u.b = altf;

  radiopacket[20] = u.a[0];
  radiopacket[21] = u.a[1];
  radiopacket[22] = u.a[2];
  radiopacket[23] = u.a[3];

  u.b = pIMUs1->barometro[0];

  radiopacket[24] = u.a[0];
  radiopacket[25] = u.a[1];
  radiopacket[26] = u.a[2];
  radiopacket[27] = u.a[3];

  
  u.b = pIMUs1->barometro[1];

  radiopacket[28] = u.a[0];
  radiopacket[29] = u.a[1];
  radiopacket[30] = u.a[2];
  radiopacket[31] = u.a[3];

  
  u.b = pIMUs1->barometro[2];

  radiopacket[32] = u.a[0];
  radiopacket[33] = u.a[1];
  radiopacket[34] = u.a[2];
  radiopacket[35] = u.a[3];

  u.b = pIMUs1->acelerometro[0];

  radiopacket[36] = u.a[0];
  radiopacket[37] = u.a[1];
  radiopacket[38] = u.a[2];
  radiopacket[39] = u.a[3];

  u.b = pIMUs1->acelerometro[1];

  radiopacket[40] = u.a[0];
  radiopacket[41] = u.a[1];
  radiopacket[42] = u.a[2];
  radiopacket[43] = u.a[3];

  u.b = pIMUs1->acelerometro[2];

  radiopacket[44] = u.a[0];
  radiopacket[45] = u.a[1];
  radiopacket[46] = u.a[2];
  radiopacket[47] = u.a[3];

  u.b = pIMUs1->giroscopio[0];

  radiopacket[48] = u.a[0];
  radiopacket[49] = u.a[1];
  radiopacket[50] = u.a[2];
  radiopacket[51] = u.a[3];

  u.b = pIMUs1->giroscopio[1];

  radiopacket[52] = u.a[0];
  radiopacket[53] = u.a[1];
  radiopacket[54] = u.a[2];
  radiopacket[55] = u.a[3];

  u.b = pIMUs1->giroscopio[2];

  radiopacket[56] = u.a[0];
  radiopacket[57] = u.a[1];
  radiopacket[58] = u.a[2];
  radiopacket[59] = u.a[3];
  
  u.b = pIMUs1->magnetometro[0];

  radiopacket[60] = u.a[0];
  radiopacket[61] = u.a[1];
  radiopacket[62] = u.a[2];
  radiopacket[63] = u.a[3];

  u.b = pIMUs1->magnetometro[1];

  radiopacket[64] = u.a[0];
  radiopacket[65] = u.a[1];
  radiopacket[66] = u.a[2];
  radiopacket[67] = u.a[3];

  u.b = pIMUs1->magnetometro[2];

  radiopacket[68] = u.a[0];
  radiopacket[69] = u.a[1];
  radiopacket[70] = u.a[2];
  radiopacket[71] = u.a[3];
 //Dados seguintes devem ser inseridos da mesma forma

  Serial.println(F("Transmissao para slave começou"));
  for (int n = 0; n < 67; n ++) {
   Wire.beginTransmission(8);
   Wire.write(radiopacket[n]);
   Wire.endTransmission();
  }
  Serial.println(F("Transmissao para slave terminou"));
  rf95.send(radiopacket, sizeof(radiopacket));

  Serial.println("lol3");
  rf95.waitPacketSent();
  // Espera o pacote ser enviando

  packetnum++;
  
  //Escreve na EEPROM o ultimo pacote enviado
  EEPROMWritelong(EEPROMADDRESS,(packetnum));
  delay(100);
     Serial.println("lol4");
}

//This function will write a 4 byte (32bit) long to the eeprom at
//the specified address to address + 3.
void EEPROMWritelong(int address, long value)
      {
      //Decomposition from a long to 4 bytes by using bitshift.
      //One = Most significant -> Four = Least significant byte
      byte four = (value & 0xFF);
      byte three = ((value >> 8) & 0xFF);
      byte two = ((value >> 16) & 0xFF);
      byte one = ((value >> 24) & 0xFF);

      //Write the 4 bytes into the eeprom memory.
      EEPROM.write(address, four);
      EEPROM.write(address + 1, three);
      EEPROM.write(address + 2, two);
      EEPROM.write(address + 3, one);
      }

long EEPROMReadlong(long address)
      {
      //Read the 4 bytes from the eeprom memory.
      long four = EEPROM.read(address);
      long three = EEPROM.read(address + 1);
      long two = EEPROM.read(address + 2);
      long one = EEPROM.read(address + 3);

      //Return the recomposed long by using bitshift.
      return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
      }
