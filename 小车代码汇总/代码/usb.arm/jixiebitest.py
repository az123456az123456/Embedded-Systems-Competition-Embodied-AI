import serial
import time

def generate_servo_commands(servo_info, time_ms):
    """
    生成舵机控制指令（不发送，仅生成）
    
    参数:
        servo_info (dict): 舵机信息字典 {舵机ID: PWM值, ...}
        time_ms (int): 执行时间（毫秒）
    
    返回:
        str: 生成的指令字符串（单指令或多指令组合）
    """
    if not isinstance(servo_info, dict) or not servo_info:
        raise ValueError("请提供格式为 {舵机ID: PWM值} 的非空字典")
    
    commands = []
    for servo_id, pwm in servo_info.items():
        # 验证参数范围
        if not (0 <= servo_id <= 5):
            raise ValueError(f"舵机ID {servo_id} 超出范围 (0-5)")
        if not (500 <= pwm <= 2500):
            raise ValueError(f"PWM值 {pwm} 超出范围 (500-2500)")
        
        # 生成单条指令
        formatted_id = f"{servo_id:03d}"
        formatted_pwm = f"P{pwm:04d}"
        formatted_time = f"T{time_ms:04d}"
        commands.append(f"#{formatted_id}{formatted_pwm}{formatted_time}!")
    
    # 单指令直接返回，多指令用花括号包裹
    return commands[0] if len(commands) == 1 else "{" + "".join(commands) + "}"

def send_generated_command(command, port='COM5', baudrate=115200):
    """发送已生成的舵机控制指令"""
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            time.sleep(0.1)  # 等待串口初始化
            ser.write(command.encode())
            time.sleep(0.1)  # 等待命令发送完成
            response = ser.read_all().decode().strip()
            print(f"发送命令: {command}")
            if response:
                print(f"收到响应: {response}")
            return True
    except Exception as e:
        print(f"发送命令时出错: {e}")
        return False
if __name__ == "__main__":
    try:
        # 示例1: 单舵机控制
        single_servo_cmd = generate_servo_commands({1: 2000}, 2000)
        print(f"生成单舵机指令: {single_servo_cmd}")
        send_generated_command(single_servo_cmd)
        
        # 示例2: 多舵机控制
        multi_servo_cmd = generate_servo_commands({
            0: 2000,  # 舵机0
            2: 2000,  # 舵机2
            #5: 1500   # 舵机5（夹爪）
        }, 2000)
        print(f"生成多舵机指令: {multi_servo_cmd}")
        send_generated_command(multi_servo_cmd)
        time.sleep(2.5)  # 等待动作完成
        
        # 示例3: 更复杂的动作序列
        print("\n执行完整动作序列:")
        
        # 1. 移动到初始位置
        home_cmd = generate_servo_commands({i: 1500 for i in range(6)}, 1000)
        print(f"生成归位指令: {home_cmd}")
        send_generated_command(home_cmd)
        time.sleep(2.5)  # 等待动作完成
        
        # 2. 张开夹爪
        '''open_gripper_cmd = generate_servo_commands({5: 500}, 1000)
        print(f"生成张开夹爪指令: {open_gripper_cmd}")
        send_generated_command(open_gripper_cmd)
        time.sleep(1)
        
        # 3. 闭合夹爪
        close_gripper_cmd = generate_servo_commands({5: 2500}, 1000)
        print(f"生成闭合夹爪指令: {close_gripper_cmd}")
        send_generated_command(close_gripper_cmd)
        time.sleep(1)'''
        
    except ValueError as ve:
        print(f"参数错误: {ve}")
    except Exception as e:
        print(f"执行出错: {e}")