import pyautogui
import time

def click_based_on_flag(flag, pos1=(500, 500), pos2=(800, 500)):
    """
    根据布尔值 flag 点击不同位置
    
    :param flag: 布尔值（1 或 0），1 点击 pos1，0 点击 pos2
    :param pos1: flag=1 时的点击坐标，默认 (500, 500)
    :param pos2: flag=0 时的点击坐标，默认 (800, 500)
    """
    # 确保输入是有效的布尔值（1 或 0）
    if flag not in (0, 1):
        raise ValueError("输入必须是 0 或 1")
    
    # 根据 flag 选择点击位置
    target_pos = pos1 if flag == 1 else pos2
    
    # 执行点击操作
    pyautogui.click(target_pos[0], target_pos[1])
    
    # 打印日志（可选）
    print(f"flag={flag}，已点击坐标：{target_pos}")
    
    # 短暂延迟防止连续点击冲突
    time.sleep(0.1)
# 测试函数
if __name__ == "__main__":
    # 点击第一个位置（500, 500）
    click_based_on_flag(1)
    
    # 等待1秒
    time.sleep(1)
    
    # 点击第二个位置（800, 500）
    click_based_on_flag(0)
    
    # 自定义位置
    click_based_on_flag(1, pos1=(300, 300), pos2=(600, 600))  # flag=1 时点击 (300, 300)