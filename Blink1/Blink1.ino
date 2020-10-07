/*
 * RoboPeak RPLIDAR Arduino Example
 * This example shows the easy and common way to fetch data from an RPLIDAR
 *
 * You may freely add your application code based on this template
 *
 * USAGE:
 * ---------------------------------
 * 1. Download this sketch code to your Arduino board
 * 2. Connect the RPLIDAR's serial port (RX/TX/GND) to your Arduino board (Pin 0 and Pin1)
 * 3. Connect the RPLIDAR's motor ctrl pin to the Arduino board pin 3
 */

 /*
  * Copyright (c) 2014, RoboPeak
  * All rights reserved.
  * RoboPeak.com
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  *
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
  * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
  * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
  * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
  * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
  * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  */

  // This sketch code is based on the RPLIDAR driver library provided by RoboPeak
#include "RPLidar.h"

// You need to create an driver instance 
RPLidar lidar;

#define RPLIDAR_MOTOR 17 // The PWM pin for control the speed of RPLIDAR's motor.
// This pin should connected with the RPLIDAR's MOTOCTRL signal 



const int count = 720;
RPLidarMeasurement massiv[count];

rplidar_response_device_info_t INF;
rplidar_response_device_health_t HELa;
long time_start = 0;
int ii = 0;
int pos_x = 240; //Центр машинки по Х
int pos_y = 160; //Центр машинки по Y
int X, Y;  // Итоговые координаты для экрана
byte flag;    // Стутус платы с экраном. получаем с передачей второго байта
byte chek_sum = 0;   // Байт контрольной суммы


struct Koordinat			  // структура для хранения данных получаемых с лидара 
{
	float distance; //distance value in mm unit
	float angle;    //anglue value in degree
	int angle_okrugl;    //anglue value in degree

	bool  startBit; //whether this point is belong to a new scan
	byte  quality;  //quality of the current measurement

	int x;		   // координата по х для экрана 480*320  которые передаем в экран
	int y;		    // координата по У для экрана 480*320 которые передаем в экран
	int time;     // Время рассчета 

};
struct XYmap         // Структура для возвращения из функции координат по Х и по У	в локальной системе
{
	int x;		   // координата по х в локальной системе координат относительно лидара. лидар в центре с координатами 0,0
	int y;		    // координата по У в локальной системе координат относительно лидара. лидар в центре с координатами 0,0
};

Koordinat Massiv[720];    //Создаем массив из расчета в 0,5 градуса

#include <SPI.h>
//Задаем пин Slave select для SPI
#define SS_PIN 10

void Init_USART3()
{
	//PIO_Configure(PIOA, PIO_PERIPH_A, PIO_PA8A_URXD | PIO_PA9A_UTXD, PIO_DEFAULT);

	//PMC->PMC_PCER0 = PMC_PCER0_PID8;  // UART power ON


	///*Включаем UART, подавая на него тактирование*/
	//pmc_enable_periph_clk(ID_USART3);

	//* Отключаем DMA для приёма и для передачи */
	USART3->US_PTCR = US_PTCR_RXTDIS | US_PTCR_TXTDIS;

	//* Enable the pull up on the Rx and Tx pin */
	PIOA->PIO_PUER = PIO_PD4B_TXD3 | PIO_PD5B_RXD3;

	// Reset and disable receiver and transmitter
	USART3->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS;

	// Configure mode
	uint32_t dwMode = US_MR_CHRL_8_BIT | US_MR_NBSTOP_1_BIT | US_MR_PAR_NO;
	uint32_t modeReg = dwMode & 0x00000E00;

	USART3->US_MR = modeReg;

	// Configure baudrate (asynchronous, no oversampling)
	uint32_t dwBaudRate = 9600;
	USART3->US_BRGR = (SystemCoreClock / dwBaudRate) >> 4;

	/*Конфигурируем прерывания*/
	/*Отключаем их*/
	USART3->US_IDR = 0xFFFFFFFF;

	/*Включаем нужные нам прерывания. */
	USART3->US_IER = US_IER_RXRDY | US_IER_OVRE | US_IER_FRAME;

	NVIC_EnableIRQ(USART3_IRQn);

	/*Включаем передачу и приём*/
	USART3->US_CR = US_CR_RXEN | US_CR_TXEN;

}

void setup() 
{
	Serial.begin(115200);
	// bind the RPLIDAR driver to the arduino hardware serial
	 
	lidar.begin(Serial1);

	// set pin modes
	pinMode(RPLIDAR_MOTOR, OUTPUT);
	lidar.getDeviceInfo(INF, 1000);
	Serial.print(" INF_firmware_version= ");		Serial.println(INF.firmware_version);
	Serial.print(" hardware_version= ");		Serial.println(INF.hardware_version);
	Serial.print(" model= ");		 Serial.println(INF.model);
	Serial.print(" serialnum= ");	
	for (byte i = 0; i < 16; i++)	{ Serial.print(INF.serialnum[i]);	}  	Serial.println();
	Serial.println("=====================================================================");
	lidar.getHealth(HELa, 1000);
	Serial.print(" HELa status = ");		Serial.println(HELa.status);
	Serial.print(" HELa error_code = ");		Serial.println(HELa.error_code);
	Serial.println("=====================================================================");
	delay(999);

	SPI.begin();
	//SPI.setDataMode(SPI_MODE3);
	//SPI.setBitOrder(MSBFIRST);
	//SPI.setClockDivider(SPI_CLOCK_DIV4);  // скорость работы по шине  SPI_CLOCK_DIV2 делитель 2 к частоте адруино 16 = 8 Мгерц

  //определяем пин slave select как выход
	pinMode(SS_PIN, OUTPUT);
	digitalWrite(SS_PIN, HIGH);  // ensure SS stays high for now

	time_start = millis(); // Время старта программы

	Init_USART3();
}

long abc = 0;
void USART3_Handler(void) 
{ 
	abc++;
	Serial.print("a= ");
}
// IT handlers
//void UART_Handler(void)
//{
//	NVIC_EnableIRQ((IRQn_Type)ID_UART);
//}

void WriteByte_SPI(uint8_t CS_PIN, char* data)  // Указываем на каком пине устройство и с какого регистра нужно прочитать данные
{
	//SPI.beginTransaction();       // записываем со скоростью 1 MHz делитель 16
	digitalWrite(CS_PIN, LOW);
	SPI.transfer(data[0]);
	digitalWrite(CS_PIN, HIGH);
	//SPI.endTransaction();
}
void WriteData_SPI(uint8_t CS_PIN, char* data_a, char* data_x, char* data_y )  // Указываем на каком пине устройство и с какого регистра нужно прочитать данные
{
	const static byte del = 6;
	//SPI.beginTransaction();       // записываем со скоростью 1 MHz делитель 16
	digitalWrite(CS_PIN, LOW);
	SPI.transfer(0x1A); // Отправляем первую команду
	delayMicroseconds(del);
	flag = SPI.transfer(0x1B);  // Отправляем вторую команду и принимаем байт статуса той платы
	delayMicroseconds(del);
	SPI.transfer(data_a[0]);		//Два байта угол
	chek_sum += data_a[0];
	delayMicroseconds(del);
	SPI.transfer(data_a[1]);
	chek_sum += data_a[1];
	delayMicroseconds(del);
	SPI.transfer(data_x[0]);		//Два байта координата по х
	chek_sum += data_x[0];
	delayMicroseconds(del);
	SPI.transfer(data_x[1]);
	chek_sum += data_x[1];
	delayMicroseconds(del);
	SPI.transfer(data_y[0]);		//Два байта координата по У
	chek_sum += data_y[0];
	delayMicroseconds(del);
	SPI.transfer(data_y[1]);
	chek_sum += data_y[1];
	delayMicroseconds(del);
	SPI.transfer(chek_sum);	  // последним посылаем контрольную сумму
	chek_sum = 0;  // Обнуляемся

	digitalWrite(CS_PIN, HIGH);
	//SPI.endTransaction();
}

int getXY(float angle_,float distance_,XYmap& ret_)		   //Функция которая расчитвает координаты по углу и дистанции(гипотенуза)
{
	if (angle_ >= 0 && angle_ < 90)
	{
		ret_.x = distance_ * cos(angle_ * (PI / 180));	   //Преобразуем из градусов в радианы и находим стороны треугольника
		ret_.y = distance_ * sin(angle_ * (PI / 180));
		return 0; //все хорошо
	}
	if (angle_ >= 90 && angle_ < 180)
	{
		angle_ = angle_ - 90;								 // отнимаем угол
		ret_.x = distance_ * - sin(angle_ * (PI / 180));		  // стороны меняются местами синус и уосинус и занк минус по х
		ret_.y = distance_ * cos(angle_ * (PI / 180));
		return 0; //все хорошо
	}
	if (angle_ >= 180 && angle_ < 270)
	{
		angle_ = angle_ - 180;								 // отнимаем угол
		ret_.x = distance_ * - cos(angle_ * (PI / 180));	   //Появляется знак минус 
		ret_.y = distance_ * - sin(angle_ * (PI / 180));	    //Появляется знак минус 
		return 0; //все хорошо
	}
	if (angle_ >= 270 && angle_ < 360)
	{
		angle_ = angle_ - 270;								 // отнимаем угол
		ret_.x = distance_ * sin(angle_ * (PI / 180));	   
		ret_.y = distance_ * -cos(angle_ * (PI / 180));	   //Появляется знак минус 
		return 0; //все хорошо
	}
	//Serial.print(" AAangle "); Serial.println(angle_);
	return -1;	//Ошибка

}

		XYmap tek_koordin;  // переменная в которую из функции придет результат , он запишется в адрес этой переменной
		int rezult;			// переменная в которую вернется результат функции
		int angle_okrugl;  // Тут храним округленный угол который является адресом ящейки в массиве

long time_a = 0;

void loop() 
{
	if (millis()- time_a > 1000)
	{
		time_a = millis();
		Serial.print("abc= ");
		Serial.println(abc);
	}
	long a = micros();
	if (IS_OK(lidar.waitPoint()))		// на передачу уходит примерно 100 микросекунд отстальное время, это ожидание в цикле по continue
	{
		long b = micros();
		
		if (lidar.getCurrentPoint().angle >= 0 && lidar.getCurrentPoint().angle < 360 )   // если возвращается номальный угол
		{
			if (lidar.getCurrentPoint().quality > 0)   //Если данные достоверные
			{

				rezult = getXY(lidar.getCurrentPoint().angle, lidar.getCurrentPoint().distance, tek_koordin);	  // Запрвшивваем координаты для полученных данных
				if (rezult == 0)	  //если все хорошо
				{
					angle_okrugl = lidar.getCurrentPoint().angle * 2.f;// увеличиваем в 2 раза и отбрасываем дробь. получаем номер строки в маасиве которая будет соответствовать половигне градуса если номер в массиве разденить пополам
					Massiv[angle_okrugl].angle = lidar.getCurrentPoint().angle; //anglue value in degree
					Massiv[angle_okrugl].angle_okrugl = angle_okrugl; //anglue value in degree

					Massiv[angle_okrugl].distance = lidar.getCurrentPoint().distance; //distance value in mm unit
					Massiv[angle_okrugl].quality = lidar.getCurrentPoint().quality; //quality of the current measurement
					Massiv[angle_okrugl].startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
					Massiv[angle_okrugl].x = pos_x + tek_koordin.x / 10.f;;		 //Пишем в массив  уже координаты с учетом положения машинки и масштабирования
					Massiv[angle_okrugl].y = pos_y + tek_koordin.y / 10.f;;		 //Пишем в массив   уже координаты с учетом положения машинки и масштабирования
					//int tta = -33;
					//int ttx = -4444;
					//int tty = -22222;

					//char* pa = (char*)&tta;	  // взять адрес переменной типа инт и преобразовать его в адрес типа чар, и присвоить его переменной обьявленной как перменная содержащая адрес типа чар
					//char* px = (char*)&ttx;	  // взять адрес переменной типа инт и преобразовать его в адрес типа чар, и присвоить его переменной обьявленной как перменная содержащая адрес типа чар
					//char* py = (char*)&tty;
					char* pa = (char*)&angle_okrugl;
					char* px = (char*)&Massiv[angle_okrugl].x;
					char* py = (char*)&Massiv[angle_okrugl].y;

					//long cc = micros();
					WriteData_SPI(SS_PIN, pa, px, py);
					//long dd = micros();
					//Massiv[angle_okrugl].time = dd - cc;// время потребовшееся на расчет	 должно быть меньше чем 1 секунда / число измерений например 2000 значит 500 микросекунд минус 100 на передачу данных из лидара

				}
				else	  // 
				{
					Serial.print("Err getXY...");			//Ошибку на экран
					Serial.print(" angle "); Serial.print(lidar.getCurrentPoint().angle);
					//Serial.print(" distance "); Serial.print(lidar.getCurrentPoint().distance);
					Serial.println("=");

				}
			}
		}
		else	  // если угол выходит за диапазон занчит что-то не то
		{
			//Serial.print ("Err angle...");			//Ошибку на экран и ничего не меняем в массиве
			//Serial.print(lidar.getCurrentPoint().angle);
			//Serial.println("===");
		}
		long c = micros();

		//perform data processing here... 
		//if (millis() - time_start > 3000)
		//{
		//	
		//	for (int i = 0; i < 720; i++)
		//	{
		//		Serial.print(" i "); Serial.print(i);

		//		//Serial.print(" angle "); Serial.print(Massiv[i].angle);
		//		Serial.print(" angle_okrugl "); Serial.print(Massiv[i].angle_okrugl);
		//		//Serial.print(" distance "); Serial.print(Massiv[i].distance);
		//		//Serial.print(" quality "); Serial.print(Massiv[i].quality);
		//		//Serial.print(" startBit "); Serial.print(Massiv[i].startBit);

		//		Serial.print(" x "); Serial.print(Massiv[i].x);
		//		Serial.print(" y "); Serial.print(Massiv[i].y);
		//		//Serial.print(" time "); Serial.print(Massiv[i].time);
		//		Serial.println();
		//	}
		//	delay(999999);
		//}
			

   /*
		massiv[ii].distance = distance;
		massiv[ii].angle = angle;
		massiv[ii].startBit = startBit;
		massiv[ii].quality = quality;
		ii++;
		//delayMicroseconds(100);
		if (ii == (count-1))
		{
			static long c = millis();
			for (int i = 0; i < count; i++)
			{
				Serial.print(" i= ;");		Serial.print(i);
				Serial.print("; angle=;");		Serial.print(massiv[i].angle);
				Serial.print("; distance= ;");		Serial.println(massiv[i].distance);

			}
			Serial.print("Time = ");		Serial.println(c-b);
			//myGLCD.fillRoundRect(0, 0, 200, 200);


			delay(99999999999);

		}
	 */
	}
	else 
	{
		analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
		//Serial.print("Error ");
		//Serial.println(millis());

		// try to detect RPLIDAR... 
		rplidar_response_device_info_t info;
		if (IS_OK(lidar.getDeviceInfo(info, 100))) {
			// detected...
			lidar.startScan(1,500);
			Serial.println("startScan !!!!!!!!!!!!");


			// start motor rotating at max allowed speed
			analogWrite(RPLIDAR_MOTOR, 255);
			delay(1000);
		}
	}
}
