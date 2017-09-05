// COMENTARIOS MINERVA ROCKETS

// Esse codigo eh um dos codigos de exemplo feitos para o LoRa
// Procure o codigo LoRaRX para poder realizar o teste
// Comentarios em portugues foram adicionados e o codigo 
// foi levemente alterado. 

//Conexoes feitas :

//ARDUINO <--------------> LORA
// 5V     <--------------> VIN
// GND    <--------------> GND
// 13     <--------------> SCK 
// 12     <--------------> MISO
// 11     <--------------> MOSI
// 04*    <--------------> CS
// 03*    <--------------> G0
// 02     <--------------> RST

// * podem ser alterados. Os demais pinos sao particulares do Arduino UNO
//[COBRUF 2017 : Para o Arduino Nano, as seguintes conexoes devem ser feitas
// 3.3V   <--------------> SCK
// 13     <--------------> SCK
// 12     <--------------> MISO
// 11     <--------------> MOSI
// 04     <--------------> CS
// 03     <--------------> G0
// 02     <--------------> RST ]

#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3

// Frequência do LoRa! [COUBRUF 2017: USAR 915.0]
#define RF95_FREQ 915.0

// Criacao do objeto da classe RH_RF95
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Led na porta 13 pisca ao receber um pacote
#define LED 13

void setup() 
{
  pinMode(LED, OUTPUT);     
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  while (!Serial);
  Serial.begin(9600);
  delay(100);

  Serial.println("Arduino LoRa RX Test!");
  

  // Reinicialização Manual
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
  
  // Apos reinicializacao, eh necessesario setar o valor da frequencia
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

void loop()
{
  if (rf95.available())
  {
    // Verificacao de mensagem   
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    
    if (rf95.recv(buf, &len))
    {
      digitalWrite(LED, HIGH);
      RH_RF95::printBuffer("Recebido: ", buf, len);
      Serial.print("Mensagem Recebida: ");
      Serial.println((char*)buf);
       Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
      
      // Send a reply
      uint8_t data[] = "OH YEAHHHHH!!!";
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      Serial.println("Resposta foi enviada!");
      digitalWrite(LED, LOW);
    }
    else
    {
      Serial.println("Nenhuma mensagem foi recebida");
    }
  }
}
