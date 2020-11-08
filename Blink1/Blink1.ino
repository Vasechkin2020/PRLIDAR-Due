
#include "Function.h"


void setup() 
{
	Serial.begin(115200);
	Serial.println("");
	// bind the RPLIDAR driver to the arduino hardware serial

	SPI.begin();
	//SPI.setDataMode(SPI_MODE3);
	//SPI.setBitOrder(MSBFIRST);
	//SPI.setClockDivider(SPI_CLOCK_DIV4);  // скорость работы по шине  SPI_CLOCK_DIV2 делитель 2 к частоте адруино 16 = 8 Мгерц

  //определяем пин slave select как выход
	pinMode(SS_PIN, OUTPUT);
	digitalWrite(SS_PIN, HIGH);  // ensure SS stays high for now
	 
	lidar.begin(Serial1);

	// set pin modes
	pinMode(RPLIDAR_MOTOR, OUTPUT);


	get_Device_info();
	set_STOP();
	set_START();

	////my_sendCommand(RPLIDAR_CMD_GET_DEVICE_SAMPLERATE, NULL, 0); // Отправляем команду
	//delay(300);											   // ждем 300 милисекунд пока придет ответ примерно 90 микросекунд на байт

	////lidar.getHealth(HELa, 1000);
	//Serial.print(" HELa status = ");		Serial.println(HELa.status);
	//Serial.print(" HELa error_code = ");		Serial.println(HELa.error_code);
	//Serial.println("=====================================================================");
	//delay(999);


	time_start = millis(); // Время старта программы

	//Init_USART3();
}




void loop() 
{
	if (millis()- time_a > 1000)
	{
		//Serial.print("time_int= ");
		//Serial.print(time_int);
		//time_int = 0;
		//Serial.print(" abc= ");
		//Serial.print(abc);
		//Serial.print(" bb= ");
		//Serial.print(bb);
		//Serial.print(" cc= ");
		//Serial.println(cc);
		Serial.print("b-a= ");
		Serial.print(b-a);
		Serial.print(" c-b= ");
		Serial.print(c - b);
		Serial.print(" c-a= ");
		Serial.println(c - a);

		bb = 0;
		abc = 0;

		time_a = millis();
		//delay(1000);
		//set_STOP();

	}

	if (flag_data == true)		// на передачу уходит примерно 100 микросекунд отстальное время, это ожидание в цикле по continue
	{
		a = micros();
		flag_data == false;
		
		if (DATA.angle >= 0 && DATA.angle < 360 )   // если возвращается номальный угол
		{
			if (DATA.quality > 0)   //Если данные достоверные
			{

				rezult = getXY(DATA.angle, DATA.distance, tek_koordin);	  // Запрвшивваем координаты для полученных данных
				if (rezult == 0)	  //если все хорошо
				{
					angle_okrugl = DATA.angle * 2.f;// увеличиваем в 2 раза и отбрасываем дробь. получаем номер строки в маасиве которая будет соответствовать половигне градуса если номер в массиве разденить пополам
					Massiv[angle_okrugl].angle = DATA.angle; //anglue value in degree
					Massiv[angle_okrugl].angle_okrugl = angle_okrugl; //anglue value in degree

					Massiv[angle_okrugl].distance = DATA.distance; //distance value in mm unit
					Massiv[angle_okrugl].quality = DATA.quality; //quality of the current measurement
					Massiv[angle_okrugl].startBit = DATA.startBit; //whether this point is belong to a new scan
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

					b = micros();
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
		c = micros();

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
			

		//massiv[ii].distance = distance;
		//massiv[ii].angle = angle;
		//massiv[ii].startBit = startBit;
		//massiv[ii].quality = quality;
		//ii++;
		////delayMicroseconds(100);
		//if (ii == (count-1))
		//{
		//	static long c = millis();
		//	for (int i = 0; i < count; i++)
		//	{
		//		Serial.print(" i= ;");		Serial.print(i);
		//		Serial.print("; angle=;");		Serial.print(massiv[i].angle);
		//		Serial.print("; distance= ;");		Serial.println(massiv[i].distance);

		//	}
		//	Serial.print("Time = ");		Serial.println(c-b);
		//	//myGLCD.fillRoundRect(0, 0, 200, 200);


		//	delay(99999999999);

		//}
	}
	//else 
	//{
	//	analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
	//	//Serial.print("Error ");
	//	//Serial.println(millis());

	//	// try to detect RPLIDAR... 
	//	rplidar_response_device_info_t info;
	//	if (IS_OK(lidar.getDeviceInfo(info, 100))) {
	//		// detected...
	//		lidar.startScan(1,500);
	//		Serial.println("startScan !!!!!!!!!!!!");


	//		// start motor rotating at max allowed speed
	//		analogWrite(RPLIDAR_MOTOR, 255);
	//		delay(1000);
	//	}
	//}
}
