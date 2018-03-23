// ROS headers
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#include <std_msgs/String.h>

// Program headers
//#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include <math.h>

int count = 0;
int count_display=0;

// Definir entradas do LCD
LiquidCrystal lcd(2, 3, 4, 5, 6, 7);

// Definir A0,A1,A2,A3 como entradas analogicas
float sensor1Pin = A0;
float sensor2Pin = A1;
float sensor3Pin = A2;
float sensor4Pin = A3;

// Definir D8 como entrada digital
int cali = 8;

// Definir variavel de leitura de distÃ¢ncia
float sensor1Val = 0;
float sensor2Val = 0;
float sensor3Val = 0;
float sensor4Val = 0;

// Definir variavel de leitura de distÃ¢ncia
float sensor1Vall = 0;
float sensor2Vall = 0;
float sensor3Vall = 0;
float sensor4Vall = 0;

// Definir variavel de leitura de distÃ¢ncia
float sensor1Vallc = 0;
float sensor2Vallc = 0;
float sensor3Vallc = 0;
float sensor4Vallc = 0;

// Valor maximo gama ADC
float declive1 = 0;
float declive2 = 0;
float declive3 = 0;
float declive4 = 0;

// Definir variavel intermÃ©dio para calibracao
float distance1 = 0;
float distance2 = 0;
float distance3 = 0;
float distance4 = 0;

// Definir variavel para valor de distancia apÃ³s calibracao
float distance1_cut = 0;
float distance2_cut = 0;
float distance3_cut = 0;
float distance4_cut = 0;

// Definir variavais para o pitch e o roll
float pitch_1 = 0;
float pitch_2 = 0;
float roll_1 = 0;
float roll_2 = 0;
float pitch = 0;
float roll = 0;

// Definir distancia entre sensores
float dist_1_3 = 3375;
float dist_2_4 = 3375;
float dist_1_2 = 1055;
float dist_3_4 = 875;

// Definir variavel de calibracao
int calival = 0;

ros::NodeHandle nh;

std_msgs::String str_msg;

std_msgs::Float32MultiArray DadosInclin;

ros::Publisher Inclin_pub("DadosInclin", &DadosInclin);

char dim0_label[] = "DadosInclin";

void setup()
{
  nh.initNode();

  // inicializar o array limpinho
  // DadosInclin.data.clear();
  DadosInclin.layout.dim = (std_msgs::MultiArrayDimension *)

      malloc(sizeof(std_msgs::MultiArrayDimension) * 2);

  // A partir daqui mudar todos os 2 para o num de elem do array
  // Definição do cabeçalho para a mensagem do tipo Float32MultiArray

  DadosInclin.layout.dim[0].label = dim0_label;
  DadosInclin.layout.dim[0].size = 3;
  DadosInclin.layout.dim[0].stride = 1 * 3;
  DadosInclin.layout.dim_length = 0;  // tirar esta linha se der chatices
  DadosInclin.layout.data_offset = 0;

  // Defenição do array de dados concreto
  DadosInclin.data_length = 3;

  DadosInclin.data = (float *)malloc(sizeof(float) * 3);

  nh.advertise(Inclin_pub);

  // Serial.begin(9600);

  lcd.begin(16, 2);

  lcd.print("Inclinometro");
  lcd.setCursor(0, 1);
  lcd.print("Proj. Automacao");
  delay(2000);
  lcd.clear();

  pinMode(cali, INPUT);

  // BTserial.begin(9600);
}
//**************************************************************************************
void loop()
{
  // Ler e guardar valor da entrada digital

  char state = '!';

  calival = digitalRead(cali);

  // Ler e guardar valor das entradas analogicas (mV)
  sensor1Val = analogRead(sensor1Pin);
  sensor2Val = analogRead(sensor2Pin);
  sensor3Val = analogRead(sensor3Pin);
  sensor4Val = analogRead(sensor4Pin);

  sensor1Vallc = sensor1Vallc + sensor1Val;
  sensor2Vallc = sensor2Vallc + sensor2Val;
  sensor3Vallc = sensor3Vallc + sensor3Val;
  sensor4Vallc = sensor4Vallc + sensor4Val;

  sensor1Vallc = 0;
  sensor2Vallc = 0;
  sensor3Vallc = 0;
  sensor4Vallc = 0;

  sensor1Vall = sensor1Val * 0.977520;
  sensor2Vall = sensor2Val * 0.977520;
  sensor3Vall = sensor3Val * 0.977520;
  sensor4Vall = sensor4Val * 0.977520;

  // conversão entre a leitura do sensor em V e a distancia em mm
  sensor1Vall = (0.613800 * sensor1Vall) + 75.801000;
  sensor2Vall = (0.615400 * sensor2Vall) + 75.692000;
  sensor3Vall = (0.613600 * sensor3Vall) + 75.042000;
  sensor4Vall = (0.616300 * sensor4Vall) + 75.373000;

  // counter1 = 0;
  // }

  // Introducao do valor de calibracao****
  if (calival == 1 || state == 'C')
  {
    distance1 = sensor1Vall;
    distance2 = sensor2Vall;
    distance3 = sensor3Vall;
    distance4 = sensor4Vall;

    state = '!';
  }

  // Efectuar calibracao******************
  distance1_cut = sensor1Vall - distance1;
  distance2_cut = sensor2Vall - distance2;
  distance3_cut = sensor3Vall - distance3;
  distance4_cut = sensor4Vall - distance4;

  // calculo do pich e do roll

  // Inclinacao para a frente***********************************
  if ((distance1_cut <= 0) && (distance3_cut >= 0))
  {
    pitch_1 = (asin((-distance1_cut + distance3_cut) / dist_1_3)) * 57.295700;
  }
  // Inclinacao para a trÃ¡s
  else
  {
    pitch_1 = (asin((-distance1_cut + distance3_cut) / dist_1_3)) * 57.295700;
  }

  // Inclinacao para a frente**********************************
  if ((distance2_cut <= 0) && (distance4_cut >= 0))
  {
    pitch_2 = (asin((-distance2_cut + distance4_cut) / dist_2_4)) * 57.295700;
  }
  // Inclinacao para a trÃ¡s
  else
  {
    pitch_2 = (asin((-distance2_cut + distance4_cut) / dist_2_4)) * 57.295700;
  }

  // Inclinacao para a direita********************************
  if ((distance1_cut <= 0) && (distance2_cut >= 0))
  {
    roll_1 = (asin((-distance1_cut + distance2_cut) / dist_1_2)) * 57.295700;
  }
  // Inclinacao para a esquerda
  else
  {
    roll_1 = (asin((-distance1_cut + distance2_cut) / dist_1_2)) * 57.295700;
  }

  // Inclinacao para a direita********************************
  if ((distance3_cut <= 0) && (distance4_cut >= 0))
  {
    roll_2 = (asin((-distance3_cut + distance4_cut) / dist_3_4)) * 57.295700;
  }
  // Inclinacao para a esquerda
  else
  {
    roll_2 = (asin((-distance3_cut + distance4_cut) / dist_3_4)) * 57.295700;
  }

  // Calculo do Pitch e do Roll*****************************************************

  pitch = (pitch_1 + pitch_2) / 2;

  roll = (roll_1 + roll_2) / 2;

if (count_display==25){
  lcd.clear();
  lcd.setCursor(0, 0);
  //*************************
  //**************************
  lcd.print("Pitch:");
  lcd.print(pitch, 5);
  //****************************
  lcd.setCursor(0, 1);
  //****************************
  lcd.print("Roll:");
  lcd.print(roll, 5);
  count_display=0;
  }
  else{
  count_display++;
  }

  float z_mean = (sensor1Vall + sensor2Vall + sensor3Vall + sensor4Vall) / 4;

  DadosInclin.data[0] = pitch;
  DadosInclin.data[1] = roll;
  DadosInclin.data[2] = z_mean;

  Inclin_pub.publish(&DadosInclin);

  //  ROS_INFO("Orientation info published!!");
  //  ROS_INFO("Pitch: %f, Roll: %f");

  nh.spinOnce();
  // 50Hz
  delay(20);
}

// distancia (mm)
// Serial.print(sensor1Vall, 5);
// Serial.print('\t');
// Serial.print(sensor2Vall, 5);
// Serial.print('\t');
// Serial.print(sensor3Vall, 5);
// Serial.print('\t');
// Serial.print(sensor4Vall, 5);
// Serial.print('\t');c
// Serial.print(pitch, 5);
// Serial.print('\t');
// Serial.print(roll, 5);
// Serial.print('\n');
