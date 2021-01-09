//A2 = arduino receptor/escravo

//Incluindo a biblioteca para a comunicação I2C
#include <Wire.h>

//Incluindo a biblioteca para a comunicação I2C
#include <SPI.h>

//Incluindo as bibliotecas para a comunicação CAN
#include <can.h>
#include <mcp2515.h>

//definindo a entrada dos botões
#define UART 9
#define I2C  8
#define SPI_ 7
#define CAN_ 6

//outras definições

//variáveis CAN
MCP2515 mcp2515(10);
struct can_frame canMsg;
char msgDataE;

//variaveis SPI
volatile bool received;
char SlaveReceived, SlaveSend;

void setup()
{
  Serial.begin(9600);
  Wire.begin(127); //definindo o endereço do escravo no I2C
  
  //definindo botão como entrada
  pinMode(UART,INPUT);
  pinMode(I2C,INPUT); 
  pinMode(SPI_,INPUT);
  pinMode(CAN_,INPUT);  
  
//**funções I2C**//  
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  
//**funções SPI**//
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE); //SPI modo escravo
  received = false;
  SPI.attachInterrupt(); 
    
////**funções CAN**//
  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS,MCP_8MHZ);
  mcp2515.setNormalMode();
}

//***função para limpar o buffer****
void limpar_buffer() { 
   unsigned char temp = Serial.available();
   for (int i = temp; i>=0; i--) {
     Serial.read(); 
   }
}

char mensagem;//variável para o I2C

//***funções I2C*******************//
void receiveEvent(int bytes){
  mensagem = Wire.read();
}

void requestEvent(){
  Wire.write(mensagem);
}

//***funções SPI******************//
ISR(SPI_STC_vect){ //funcao padrao para receber o valor do mestre
  SlaveReceived = SPDR; //SPDR = registrador do SPI onde o valor enviado é armazenado
  received = true;
}

void loop(){ 
  
//define variaveis pros protocolos
  int Protocolo1 = digitalRead(UART);
  int Protocolo2 = digitalRead(I2C);
  int Protocolo3 = digitalRead(SPI_);
  int Protocolo4 = digitalRead(CAN_);
  
//**código do SPI******************
  if(received) {
    //Serial.println("mensagem escravo");
    SlaveSend = SlaveReceived; 
    SPDR = SlaveSend;
    received = false; 
  }
//***fim do SPI*********************

//**código do CAN******************  
  if( (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) &&(canMsg.can_id == 0x036) ) {
    msgDataE = canMsg.data[0];
  }
  delay(20);
  
  if(msgDataE != NULL) {
    canMsg.can_id == 0x035;
    canMsg.can_dlc = 1;
    canMsg.data[0] = msgDataE;
    mcp2515.sendMessage(&canMsg);
  }
//***fim do CAN*********************
  
  if(Protocolo1 == HIGH){//rodando o UART
//**código do UART******************
    
    for(byte i=0;i<1000;i=i+1){//for tem por função manter no protocolo 1
      if(Serial.available()){
        char cs = Serial.read();
        Serial.write(cs);
      }
    }
//**fim do UART*********************   

  }else if(Protocolo2 == HIGH){//rodando o I2C
    //**código do I2C*****
    //receiveEvent(bytes);
    //requestEvent();
    //***fim do I2C*****
    
  }else if(Protocolo3 == HIGH){//rodando o SPI
    //**código do SPI*****
    //***fim do SPI*****
    
  }else if(Protocolo4 == HIGH){//rodando o CAN
    //**código do CAN*****
    //***fim do CAN*****
  }
  
  //****fim do codigo
}
