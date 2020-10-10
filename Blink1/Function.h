
// This sketch code is based on the RPLIDAR driver library provided by RoboPeak
#include "RPLidar.h"
// You need to create an driver instance 
RPLidar lidar;
#define RPLIDAR_MOTOR 17 // The PWM pin for control the speed of RPLIDAR's motor.
// This pin should connected with the RPLIDAR's MOTOCTRL signal 

#include <SPI.h>
//Задаем пин Slave select для SPI
#define SS_PIN 10


const int count = 720;
RPLidarMeasurement massiv[count];

rplidar_response_device_info_t INF;
rplidar_response_device_health_t HELa;
rplidar_response_device_SAMPLERATE_t SAMPLERATE;
RPLidarMeasurement DATA;
rplidar_response_measurement_node_t DATA_RAW;
long time_start = 0;
int ii = 0;
int pos_x = 240; //Центр машинки по Х
int pos_y = 160; //Центр машинки по Y
int X, Y;  // Итоговые координаты для экрана
byte flag;    // Стутус платы с экраном. получаем с передачей второго байта
byte chek_sum = 0;   // Байт контрольной суммы


long a = 0;
long b = 0;
long c = 0;

long abc = 0;
int currentbyte = 0;
volatile long time_int = 0;
volatile long time_in = 0;

int bb = 0;
int cc = 0;
volatile byte headerbuf[128];
bool flag_response = false;
bool flag_data = false;


int rezult;			// переменная в которую вернется результат функции
int angle_okrugl;  // Тут храним округленный угол который является адресом ящейки в массиве

long time_a = 0;


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
XYmap tek_koordin;  // переменная в которую из функции придет результат , он запишется в адрес этой переменной


byte recvPos = 0;
byte count_byte = 0;
byte mode = 0; //Режим работы или 0 - прием ответа и обычных команд или 1-режим приема потока данных

u_result my_sendCommand(_u8 cmd, const void * payload, size_t payloadsize)
{

	rplidar_cmd_packet_t pkt_header;
	rplidar_cmd_packet_t * header = &pkt_header;
	_u8 checksum = 0;

	if (payloadsize && payload) {
		cmd |= RPLIDAR_CMDFLAG_HAS_PAYLOAD;
	}

	header->syncByte = RPLIDAR_CMD_SYNC_BYTE;
	header->cmd_flag = cmd;

	// send header first
	Serial1.write((uint8_t *)header, 2);

	if (cmd & RPLIDAR_CMDFLAG_HAS_PAYLOAD) {
		checksum ^= RPLIDAR_CMD_SYNC_BYTE;
		checksum ^= cmd;
		checksum ^= (payloadsize & 0xFF);

		// calc checksum
		for (size_t pos = 0; pos < payloadsize; ++pos) {
			checksum ^= ((_u8 *)payload)[pos];
		}

		// send size
		_u8 sizebyte = payloadsize;
		Serial1.write((uint8_t *)&sizebyte, 1);

		// send payload
		Serial1.write((uint8_t *)&payload, sizebyte);

		// send checksum
		Serial1.write((uint8_t *)&checksum, 1);
	}

	return RESULT_OK;
}


void get_Device_info()
{
	//count_byte = 27;
	mode = 0; //Режим приема команд
	my_sendCommand(RPLIDAR_CMD_GET_DEVICE_INFO, NULL, 0); // Отправляем команду
	delay(300);											   // ждем 300 милисекунд пока придет ответ примерно 90 микросекунд на байт
	Serial.print(" INF_firmware_version= ");		Serial.println(INF.firmware_version);
	Serial.print(" hardware_version= ");		Serial.println(INF.hardware_version);
	Serial.print(" model= ");		 Serial.println(INF.model);
	Serial.print(" serialnum= ");
	for (byte i = 0; i < 16; i++) { Serial.print(INF.serialnum[i]); }  	Serial.println();
	Serial.println("=====================================================================");
}

void buf_free(byte i_)
{
	for (byte i = 0; i <= i_; i++)
	{
		headerbuf[i] = 0;
	}
}

void my_USART0_Haldler()  // Мой обработчик прерывания по UART для приема данных с лидара
{
	currentbyte = USART0->US_RHR;				//Read new byte from UART_RHR 
	abc++;
	switch (mode)
	{
	case 0: 		// Принимаем заголовки и обычные данные 
	{
		switch (recvPos)
		{
		case 0:
			if (currentbyte != RPLIDAR_ANS_SYNC_BYTE1) { return; } break;
		case 1:
			if (currentbyte != RPLIDAR_ANS_SYNC_BYTE2) { recvPos = 0; return; }	break;
		}
		headerbuf[recvPos++] = currentbyte;			  //Записываем в буфер
		if (recvPos == 7)  // Если приняли все байты заголовка	 то решаем что делать дальше
		{
			if (headerbuf[2] == 0x05)		 // Если в третем байте пришла 0х05 то это ответ на команду сканирования и 
			{
				recvPos = 0;
				mode = 1;					   //  переходим на другой режим 
				return;
			}
			if (headerbuf[2] == 0x14)		 // Если в третем байте пришла 0х14 то это ответ на команду get_info
			{
				count_byte = headerbuf[2] + 7; // Смотрим сколько байт данных должно придти и прибавляем длинну заголовка, получаес колько всего байт надо принять
				return;
			}
			if (headerbuf[2] == 0x04)		 // Если в третем байте пришла 0х14 то это ответ на команду GET_SAMPLERATE
			{
				count_byte = headerbuf[2] + 7; // Смотрим сколько байт данных должно придти и прибавляем длинну заголовка, получаес колько всего байт надо принять
				return;
			}
			recvPos = 0;  // Если не пришел ни один нужный байт команды то сбраываем на начало
			return;

		}
		if (recvPos == count_byte) // ЕСли приняли полностью и заголовок и данные  Разбираем что приняли 
		{
			if (headerbuf[2] == 0x04)		 // Если в третем байте пришла 0х14 то это ответ на команду GET_SAMPLERATE
			{
				rplidar_response_device_SAMPLERATE_t* p = (rplidar_response_device_SAMPLERATE_t*)(&headerbuf[7]);        // обьявляем переменную
				SAMPLERATE = *p;
				//buf_free(); //чистим буфер
				recvPos = 0;
				count_byte = 0;
				flag_response = true;   //получили заголовок обрабатываем его в лупе
			}
			if (headerbuf[2] == 0x14)		 // Если в третем байте пришла 0х14 то это ответ на команду get_info
			{
				rplidar_response_device_info_t* p = (rplidar_response_device_info_t*)(&headerbuf[7]);        // обьявляем переменную того типа которого размер в буфере памяти и присваиваем адрес начала буфера преевращенный в тип какой надо
				INF = *p;									   // приваиваем в переменную значение начиная с нужного адреса
				//buf_free(); //чистим буфер
				recvPos = 0;
				count_byte = 0;
				flag_response = true;   //получили заголовок обрабатываем его в лупе
			}
		}
	}
	break;
	case 1:		   //принимаем поток данных
	{
		switch (recvPos)
		{
		case 0: // expect the sync bit and its reverse in this byte          
		{
			_u8 tmp = (currentbyte >> 1);
			if ((tmp ^ currentbyte) & 0x1)
			{
				// pass
			}
			else
			{
				return;
			}
		}
		break;
		case 1: // expect the highest bit to be 1
		{
			if (currentbyte & RPLIDAR_RESP_MEASUREMENT_CHECKBIT)
			{
				// pass
			}
			else
			{
				recvPos = 0;
				return;
			}
		}
		break;
		}
		headerbuf[recvPos++] = currentbyte;

		if (recvPos == sizeof(rplidar_response_measurement_node_t))
		{
			bb++;
			_rplidar_response_measurement_node_t* p = (_rplidar_response_measurement_node_t*)(&headerbuf[0]);        // обьявляем переменную
			DATA_RAW = *p;
			DATA.distance = DATA_RAW.distance_q2 / 4.0f;
			DATA.angle = (DATA_RAW.angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
			DATA.quality = (DATA_RAW.sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
			DATA.startBit = (DATA_RAW.sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT);
			recvPos = 0;
			flag_data = true;   //получили данные обрабатываем их в лупе
		}
	}
	break;
	}
}

void USART0_Handler(void) 		  // Функция которая подменяет стандартный обработчик
{
	//Serial.println("!");

	uint32_t status = USART0->US_CSR;  // считываем регистр статуса

	//Did we receive new byte?
	if ((status & US_CSR_RXRDY) == US_CSR_RXRDY)
	{
		time_in = micros();
		//Do something else
		my_USART0_Haldler();		 // Мой обработчик прерывания только для чтения
		time_int += (micros() - time_in);
	}
	else
	{
		Serial1.IrqHandler();        // Стандартный обработчик прерывания
	}
}

void WriteByte_SPI(uint8_t CS_PIN, char* data)  // Указываем на каком пине устройство и с какого регистра нужно прочитать данные
{
	//SPI.beginTransaction();       // записываем со скоростью 1 MHz делитель 16
	digitalWrite(CS_PIN, LOW);
	SPI.transfer(data[0]);
	digitalWrite(CS_PIN, HIGH);
	//SPI.endTransaction();
}
void WriteData_SPI(uint8_t CS_PIN, char* data_a, char* data_x, char* data_y)  // Указываем на каком пине устройство и с какого регистра нужно прочитать данные
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

int getXY(float angle_, float distance_, XYmap& ret_)		   //Функция которая расчитвает координаты по углу и дистанции(гипотенуза)
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
		ret_.x = distance_ * -sin(angle_ * (PI / 180));		  // стороны меняются местами синус и уосинус и занк минус по х
		ret_.y = distance_ * cos(angle_ * (PI / 180));
		return 0; //все хорошо
	}
	if (angle_ >= 180 && angle_ < 270)
	{
		angle_ = angle_ - 180;								 // отнимаем угол
		ret_.x = distance_ * -cos(angle_ * (PI / 180));	   //Появляется знак минус 
		ret_.y = distance_ * -sin(angle_ * (PI / 180));	    //Появляется знак минус 
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
void set_STOP()
{
	mode = 0;
	my_sendCommand(0x25, NULL, 0); // Отправляем команду
	analogWrite(RPLIDAR_MOTOR, 0);
	delay(10);
}
void set_START()
{
	mode = 0;
	my_sendCommand(0x20, NULL, 0); // Отправляем команду
	analogWrite(RPLIDAR_MOTOR, 255);
	delay(1000);

}