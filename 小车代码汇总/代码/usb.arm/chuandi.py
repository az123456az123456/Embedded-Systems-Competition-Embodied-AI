import sys
import time

print("等待参数传入... (按 Ctrl+C 退出)")

while True:
    # 检查是否有足够的参数（至少3个元素：脚本名 + 2个参数）
    if len(sys.argv) >= 3:
        sta = sys.argv[1]
        color = sys.argv[2]
        
        # 检查参数是否非空
        if sta and color:
            print(f"接收到的第一个参数：{sta}")
            print(f"接收到的第二个参数：{color}")
            break  # 成功接收参数后退出循环
        else:
            print("传入的参数存在空字符串，请检查")
    else:
        print("未检测到参数，继续等待...")
    
    # 避免CPU占用过高，暂停一段时间
