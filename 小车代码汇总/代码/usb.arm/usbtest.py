import serial
import time

def send_servo_command(servo_id, pwm, time_ms, port='/dev/ttyUSB0', baudrate=115200):
    """发送舵机控制命令到USB串口"""
    # 验证参数范围（修改ID范围为0-5）
    if not (0 <= servo_id <= 5):
        raise ValueError("舵机ID必须在0-5之间")
    if not (500 <= pwm <= 2500):
        raise ValueError("PWM值必须在500-2500之间")
    if not (0 <= time_ms <= 9999):
        raise ValueError("时间必须在0-9999ms之间")
    
    # 格式化参数
    formatted_id = f"{servo_id:03d}"
    formatted_pwm = f"P{pwm:04d}"
    formatted_time = f"T{time_ms:04d}"
    
    # 构建命令
    command = f"#{formatted_id}{formatted_pwm}{formatted_time}!"
    
    # 发送命令
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            time.sleep(0.1)  # 等待串口初始化
            ser.write(command.encode())
            time.sleep(0.1)  # 等待命令发送完成
            response = ser.read_all().decode().strip()
            print(f"发送命令: {command}")
            if response:
                print(f"收到响应: {response}")
            return command
    except Exception as e:
        print(f"发送命令时出错: {e}")
        return None

# 使用示例
if __name__ == "__main__":
    # 根据实际情况修改串口名称
    send_servo_command(
        servo_id=1,     # 舵机ID，范围0-5
        pwm=1800,       # PWM值，范围500-2500
        time_ms=2000,    # 时间，范围0-9999ms
        port='COM5'     # Windows系统示例，Linux可能是'/dev/ttyUSB0'
    )    