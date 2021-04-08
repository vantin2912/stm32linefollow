# STM32 Line Following

## Pin Wiring
	I2C:
		B6: SDA
		B7: SCL

	L5696N:
		B15: Output M1
		A8: PWM M1
		A9: Output M2
		A10: PWM M2

	BTN:
		B12, B13, B14
	Sensor:
		A0->A7: 
		
	Servo:
		A11: 

	UART:
		B7: RX
		B6: TX

## PCB Note
	Res 150 Ohm: 0805 SMD
	Res 1k Ohm: 0603 SMD
	Led Blue: 0805 SMD
	Led: Led siêu sáng
	Cap 0.1 uF: 0805 SMD


## Code Note
Đọc Instruction.md

### ADC Note:
	Giá trị ADC đọc được lưu trong mảng ADC_Value[8] tương ứng PA0->PA7;
### PWM Note
	Tần số 10kHZ
	Dùng hàm USR1_Motor1_SetPWM(int32_t PWMVal);
	PWMVal >0 Động cơ quay cùng chiều;
	PWMVal <0 quay ngược lại;
	-7200 < PWMVal <7200
### UART Note
	Dùng printf và các hàm của stdio.h của c để gửi UART
### Encoder Note
	Biến đếm được lưu trong 2 thanh ghi TIM2->CNT và TIM3->CNT;
	Max giá trị là 0xFFFF;
	Dùng LL_TIM_GetCounter(TIMx) để lấy giá trị hoặc gọi trực tiếp TIMx->CNT cũng được
	Chưa xử lý ngắt tràn
### Servo
	Hàm Servo_SetAngle(float Angle);
	Giá trị góc từ -90 đến 90
	Giá trị xung để vị trí giữa là 4500 (Tăng giảm để góc giữa phù hợp với phần cứng)

	