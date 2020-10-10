
// This sketch code is based on the RPLIDAR driver library provided by RoboPeak
#include "RPLidar.h"
// You need to create an driver instance 
RPLidar lidar;
#define RPLIDAR_MOTOR 17 // The PWM pin for control the speed of RPLIDAR's motor.
// This pin should connected with the RPLIDAR's MOTOCTRL signal 

#include <SPI.h>
//������ ��� Slave select ��� SPI
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
int pos_x = 240; //����� ������� �� �
int pos_y = 160; //����� ������� �� Y
int X, Y;  // �������� ���������� ��� ������
byte flag;    // ������ ����� � �������. �������� � ��������� ������� �����
byte chek_sum = 0;   // ���� ����������� �����


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


int rezult;			// ���������� � ������� �������� ��������� �������
int angle_okrugl;  // ��� ������ ����������� ���� ������� �������� ������� ������ � �������

long time_a = 0;


struct Koordinat			  // ��������� ��� �������� ������ ���������� � ������ 
{
	float distance; //distance value in mm unit
	float angle;    //anglue value in degree
	int angle_okrugl;    //anglue value in degree

	bool  startBit; //whether this point is belong to a new scan
	byte  quality;  //quality of the current measurement

	int x;		   // ���������� �� � ��� ������ 480*320  ������� �������� � �����
	int y;		    // ���������� �� � ��� ������ 480*320 ������� �������� � �����
	int time;     // ����� �������� 

};
struct XYmap         // ��������� ��� ����������� �� ������� ��������� �� � � �� �	� ��������� �������
{
	int x;		   // ���������� �� � � ��������� ������� ��������� ������������ ������. ����� � ������ � ������������ 0,0
	int y;		    // ���������� �� � � ��������� ������� ��������� ������������ ������. ����� � ������ � ������������ 0,0
};

Koordinat Massiv[720];    //������� ������ �� ������� � 0,5 �������
XYmap tek_koordin;  // ���������� � ������� �� ������� ������ ��������� , �� ��������� � ����� ���� ����������


byte recvPos = 0;
byte count_byte = 0;
byte mode = 0; //����� ������ ��� 0 - ����� ������ � ������� ������ ��� 1-����� ������ ������ ������

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
	mode = 0; //����� ������ ������
	my_sendCommand(RPLIDAR_CMD_GET_DEVICE_INFO, NULL, 0); // ���������� �������
	delay(300);											   // ���� 300 ���������� ���� ������ ����� �������� 90 ����������� �� ����
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

void my_USART0_Haldler()  // ��� ���������� ���������� �� UART ��� ������ ������ � ������
{
	currentbyte = USART0->US_RHR;				//Read new byte from UART_RHR 
	abc++;
	switch (mode)
	{
	case 0: 		// ��������� ��������� � ������� ������ 
	{
		switch (recvPos)
		{
		case 0:
			if (currentbyte != RPLIDAR_ANS_SYNC_BYTE1) { return; } break;
		case 1:
			if (currentbyte != RPLIDAR_ANS_SYNC_BYTE2) { recvPos = 0; return; }	break;
		}
		headerbuf[recvPos++] = currentbyte;			  //���������� � �����
		if (recvPos == 7)  // ���� ������� ��� ����� ���������	 �� ������ ��� ������ ������
		{
			if (headerbuf[2] == 0x05)		 // ���� � ������ ����� ������ 0�05 �� ��� ����� �� ������� ������������ � 
			{
				recvPos = 0;
				mode = 1;					   //  ��������� �� ������ ����� 
				return;
			}
			if (headerbuf[2] == 0x14)		 // ���� � ������ ����� ������ 0�14 �� ��� ����� �� ������� get_info
			{
				count_byte = headerbuf[2] + 7; // ������� ������� ���� ������ ������ ������ � ���������� ������ ���������, �������� ������ ����� ���� ���� �������
				return;
			}
			if (headerbuf[2] == 0x04)		 // ���� � ������ ����� ������ 0�14 �� ��� ����� �� ������� GET_SAMPLERATE
			{
				count_byte = headerbuf[2] + 7; // ������� ������� ���� ������ ������ ������ � ���������� ������ ���������, �������� ������ ����� ���� ���� �������
				return;
			}
			recvPos = 0;  // ���� �� ������ �� ���� ������ ���� ������� �� ��������� �� ������
			return;

		}
		if (recvPos == count_byte) // ���� ������� ��������� � ��������� � ������  ��������� ��� ������� 
		{
			if (headerbuf[2] == 0x04)		 // ���� � ������ ����� ������ 0�14 �� ��� ����� �� ������� GET_SAMPLERATE
			{
				rplidar_response_device_SAMPLERATE_t* p = (rplidar_response_device_SAMPLERATE_t*)(&headerbuf[7]);        // ��������� ����������
				SAMPLERATE = *p;
				//buf_free(); //������ �����
				recvPos = 0;
				count_byte = 0;
				flag_response = true;   //�������� ��������� ������������ ��� � ����
			}
			if (headerbuf[2] == 0x14)		 // ���� � ������ ����� ������ 0�14 �� ��� ����� �� ������� get_info
			{
				rplidar_response_device_info_t* p = (rplidar_response_device_info_t*)(&headerbuf[7]);        // ��������� ���������� ���� ���� �������� ������ � ������ ������ � ����������� ����� ������ ������ ������������� � ��� ����� ����
				INF = *p;									   // ���������� � ���������� �������� ������� � ������� ������
				//buf_free(); //������ �����
				recvPos = 0;
				count_byte = 0;
				flag_response = true;   //�������� ��������� ������������ ��� � ����
			}
		}
	}
	break;
	case 1:		   //��������� ����� ������
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
			_rplidar_response_measurement_node_t* p = (_rplidar_response_measurement_node_t*)(&headerbuf[0]);        // ��������� ����������
			DATA_RAW = *p;
			DATA.distance = DATA_RAW.distance_q2 / 4.0f;
			DATA.angle = (DATA_RAW.angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
			DATA.quality = (DATA_RAW.sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
			DATA.startBit = (DATA_RAW.sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT);
			recvPos = 0;
			flag_data = true;   //�������� ������ ������������ �� � ����
		}
	}
	break;
	}
}

void USART0_Handler(void) 		  // ������� ������� ��������� ����������� ����������
{
	//Serial.println("!");

	uint32_t status = USART0->US_CSR;  // ��������� ������� �������

	//Did we receive new byte?
	if ((status & US_CSR_RXRDY) == US_CSR_RXRDY)
	{
		time_in = micros();
		//Do something else
		my_USART0_Haldler();		 // ��� ���������� ���������� ������ ��� ������
		time_int += (micros() - time_in);
	}
	else
	{
		Serial1.IrqHandler();        // ����������� ���������� ����������
	}
}

void WriteByte_SPI(uint8_t CS_PIN, char* data)  // ��������� �� ����� ���� ���������� � � ������ �������� ����� ��������� ������
{
	//SPI.beginTransaction();       // ���������� �� ��������� 1 MHz �������� 16
	digitalWrite(CS_PIN, LOW);
	SPI.transfer(data[0]);
	digitalWrite(CS_PIN, HIGH);
	//SPI.endTransaction();
}
void WriteData_SPI(uint8_t CS_PIN, char* data_a, char* data_x, char* data_y)  // ��������� �� ����� ���� ���������� � � ������ �������� ����� ��������� ������
{
	const static byte del = 6;
	//SPI.beginTransaction();       // ���������� �� ��������� 1 MHz �������� 16
	digitalWrite(CS_PIN, LOW);
	SPI.transfer(0x1A); // ���������� ������ �������
	delayMicroseconds(del);
	flag = SPI.transfer(0x1B);  // ���������� ������ ������� � ��������� ���� ������� ��� �����
	delayMicroseconds(del);
	SPI.transfer(data_a[0]);		//��� ����� ����
	chek_sum += data_a[0];
	delayMicroseconds(del);
	SPI.transfer(data_a[1]);
	chek_sum += data_a[1];
	delayMicroseconds(del);
	SPI.transfer(data_x[0]);		//��� ����� ���������� �� �
	chek_sum += data_x[0];
	delayMicroseconds(del);
	SPI.transfer(data_x[1]);
	chek_sum += data_x[1];
	delayMicroseconds(del);
	SPI.transfer(data_y[0]);		//��� ����� ���������� �� �
	chek_sum += data_y[0];
	delayMicroseconds(del);
	SPI.transfer(data_y[1]);
	chek_sum += data_y[1];
	delayMicroseconds(del);
	SPI.transfer(chek_sum);	  // ��������� �������� ����������� �����
	chek_sum = 0;  // ����������

	digitalWrite(CS_PIN, HIGH);
	//SPI.endTransaction();
}

int getXY(float angle_, float distance_, XYmap& ret_)		   //������� ������� ���������� ���������� �� ���� � ���������(����������)
{
	if (angle_ >= 0 && angle_ < 90)
	{
		ret_.x = distance_ * cos(angle_ * (PI / 180));	   //����������� �� �������� � ������� � ������� ������� ������������
		ret_.y = distance_ * sin(angle_ * (PI / 180));
		return 0; //��� ������
	}
	if (angle_ >= 90 && angle_ < 180)
	{
		angle_ = angle_ - 90;								 // �������� ����
		ret_.x = distance_ * -sin(angle_ * (PI / 180));		  // ������� �������� ������� ����� � ������� � ���� ����� �� �
		ret_.y = distance_ * cos(angle_ * (PI / 180));
		return 0; //��� ������
	}
	if (angle_ >= 180 && angle_ < 270)
	{
		angle_ = angle_ - 180;								 // �������� ����
		ret_.x = distance_ * -cos(angle_ * (PI / 180));	   //���������� ���� ����� 
		ret_.y = distance_ * -sin(angle_ * (PI / 180));	    //���������� ���� ����� 
		return 0; //��� ������
	}
	if (angle_ >= 270 && angle_ < 360)
	{
		angle_ = angle_ - 270;								 // �������� ����
		ret_.x = distance_ * sin(angle_ * (PI / 180));
		ret_.y = distance_ * -cos(angle_ * (PI / 180));	   //���������� ���� ����� 
		return 0; //��� ������
	}
	//Serial.print(" AAangle "); Serial.println(angle_);
	return -1;	//������

}
void set_STOP()
{
	mode = 0;
	my_sendCommand(0x25, NULL, 0); // ���������� �������
	analogWrite(RPLIDAR_MOTOR, 0);
	delay(10);
}
void set_START()
{
	mode = 0;
	my_sendCommand(0x20, NULL, 0); // ���������� �������
	analogWrite(RPLIDAR_MOTOR, 255);
	delay(1000);

}