//A1 = arduino transmissor/mestre 

//Incluindo a biblioteca para a comunicação I2C
#include <Wire.h>

//Incluindo a biblioteca para a comunicação SPI
#include <SPI.h>

//Incluindo as bibliotecas para a comunicação CAN
#include <can.h>
#include <mcp2515.h> 

//definindo a entrada dos botões que vão acionar os protocolos
#define UART 9
#define I2C  8
#define SPI_ 7
#define CAN_ 6

//defnições uart (sem necessidade)

//defnições i2c (sem necessidade)

//defnições spi
#define SS 10

//defnições can
MCP2515 mcp2515(10); // pino do SS
struct can_frame canMsg; //Declara uma estrutura de dados com base numa estrutura predefinida na biblioteca do Controlador CAN
char msgDataMS;
char msgDataMR;
bool aux_envio_can = true;

void setup()
{
  Serial.begin(9600); //iniciando o monitor serial em 9600bps
  Wire.begin(); //inicializando biblioteca i2c
  SPI.begin(); //inicializando biblioteca spi
  
  //definindo botões como entradas
  pinMode(UART,INPUT);
  pinMode(I2C,INPUT); 
  pinMode(SPI_,INPUT);
  pinMode(CAN_,INPUT);

  //Código pro SPI
  digitalWrite(SS, HIGH); //iniciando o escravo desabilitado
  //digitalWrite(8, LOW);   
  
  //Código pro CAN
  mcp2515.reset(); //reset do controlador can pelo spi do arduino
  mcp2515.setBitrate(CAN_500KBPS,MCP_8MHZ); //Configura a velocidade de comunicação CAN para 500KBPS com um Clock de 8MHz. Clock do Controlador MCP2515
  mcp2515.setNormalMode(); //mode de comunicação para garantir que mestre e escravo estão sincronizados
} // fim do setup

//***função para limpar o buffer****
void limpar_buffer() { 
  unsigned char temp = Serial.available();
  for (int i = temp; i>=0; i--) {
    Serial.read(); 
  }
}

void loop(){
   
//define variaveis pros protocolos
  int Protocolo1 = digitalRead(UART);
  int Protocolo2 = digitalRead(I2C);
  int Protocolo3 = digitalRead(SPI_);
  int Protocolo4 = digitalRead(CAN_);
  
//variavel UART, I2C e SPI
  int n=23; //para que se possa enviar mengens de diferentes tamanhos; char enviado[6] = "hello"; 
  char enviado[n+1] = "Melhor trainee do tesla"; 
  char recebido[n+1];
  
  for(byte i=0;i<n;i=i+1){//garantindo que o vetor recebido não esteja carregando informações de outro protocolo
    recebido[i]='_';
  }
  
//*****************PROTOCOLOS******************

  if(Protocolo1 == HIGH){//rodando o UART 
   //**código do UART*****************************
    
    limpar_buffer();
    for(int i=0;i<n;i=i+1){
      Serial.write(enviado[i]);//enviando cada parte do vetor
      delay(10);//delay serve para esperar resposta do recebedor
      if(Serial.available()){
      recebido[i]=Serial.read();
      }
    }      
    for(int i=0;i<n;i=i+1){
      Serial.print(recebido[i]);
    }
    delay(100);//serve para o botão so realizar a operação uma vez
               //este delay é importante para mensagens pequenas
  //**fim do UART********************************* 

  }else if(Protocolo2 == HIGH){//rodando o I2C
  //**código do I2C*******************************
    
    for (byte i = 0; i < n; i = i + 1) {
      Wire.beginTransmission(127); 
      Wire.write(enviado[i]);//enviando a mensagem
      Wire.endTransmission();
      Wire.requestFrom(127,1); //1ºarg. endereço do escravo, 2ºarg. a quantidade de bytes
      if(Wire.available()){
        recebido[i] = Wire.read(); 
      }
    }
    delay(100);//serve para o botão so realizar a operação uma vez
  //***fim do I2C*********************************

  }else if(Protocolo3 == HIGH){//rodando o SPI
  //**código do SPI*******************************
    
    char MasterSend, MasterReceived; 
    for (byte i = 0; i < n; i = i + 1) {
      digitalWrite(SS,LOW); //habilitando o escravo
      MasterSend = enviado[i]; //MasterSend é o caractere a ser enviado pelo mestre
      MasterReceived = SPI.transfer(MasterSend); //enviando o caracatere ao escravo e recebendo de volta
      recebido[i]=MasterReceived;
      digitalWrite(SS,HIGH); //desabilotando escravo
    } 
  //***fim do SPI*********************************
  
  }else if(Protocolo4 == HIGH){//rodando o CAN   
  //**código do CAN*******************************
  
 for(int i=0; i<n; i = i+1) {
  if(aux_envio_can == true) {
    msgDataMS = enviado[i];

    canMsg.can_id = 0x036; //identidade da mensagem 
    canMsg.can_dlc = 1; //número de bytes da mensagem (máx = 8)
    canMsg.data[0]= msgDataMS;
    mcp2515.sendMessage(&canMsg);
  }
  
    if( (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) && (canMsg.can_id == 0x035)) {
      msgDataMR = canMsg.data[1];
      recebido[i] = msgDataMR;
    }
  }
  aux_envio_can = false;
  }
  //***fim do CAN*********************************

  //***RELATÓRIO*********************************
  if(recebido[0]!='_'){//se tiver recebido algo
    
    Serial.print(" \n A mensagem enviada eh: "); 
    for (byte i = 0; i < n; i = i + 1) {
      Serial.print(enviado[i]);
    }
    Serial.print(" \n A mensagem recebida eh: ");
    for (byte i = 0; i < n; i = i + 1) {
      Serial.print(recebido[i]);
    }   
    
    float p=0; //declarando a variavel da porcentagem
    for (byte i = 0; i < n; i = i + 1) {
      if(enviado[i]==recebido[i]){ 
        p=p+1;
      }
    }
    p=(p*100)/n;  //calculo da porcentagem
    Serial.print("\n O sucesso do envio em porcentagem eh: "); 
    Serial.print(p); //exibição da porcentagem no monitor
  }
  

//fim do codigo
}
