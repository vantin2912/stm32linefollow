# Note
- Giá trị ADC đọc được sẽ được lưu trong biến uint16_t **Sensor_ADC_Value** ;
- Tiền tố MotorL = Động cơ bên trái, MotorR = Động cơ bên phải, Servo = Servo

# Các biến global
```c
uint16_t Sensor_ADC_Value[8];

Giá trị ADC đọc được từ cảm biến
** Note ** Giá trị của biến sẽ tự thay đổi mỗi lần phần cứng đọc ADC xong không cần tác động gì hết 

```


# Các hàm sử dụng
## Các hàm tương tự với động cơ bên phải

```c
void MotorL_EnablePWM (void); 

Bật xuất xung PWM để điều khiển động cơ bên trái 
```
    

```c
void MotorL_DisablePWM (void)

Tắt xuất xung PWM để điều khiển động cơ bên trái
```

```c
void MotorL_SetPWM (int32_t PWMVal)

Điều xung vào động cơ bên trái;

PWMVal có giá trị từ -7200 đến 7200 ứng với -100% -> 100% tốc độ;
PWMVal > 0 động cơ xoay tới;
PWMVal < 0 động cơ xoay lùi
```

```c
void Servo_SetAngle(float ServoAngle);

Điều khiển góc quay của Servo;
ServoAngle trong khoảng từ -90 đến 90 độ

```

```c
void Servo_SetAngle(float ServoAngle);

Điều khiển góc quay của Servo;
ServoAngle trong khoảng từ -90 đến 90 độ

```