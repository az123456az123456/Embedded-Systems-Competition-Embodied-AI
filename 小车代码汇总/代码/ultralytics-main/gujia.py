from ultralytics import YOLO
from PIL import Image
import cv2
# 加载预训练的人体姿态估计模型
model = YOLO('yolov8n-pose.pt')

# 替换为你的图片路径
image_path = 'data/OIP-C.webp'
results = model(image_path)
# 运行姿态估计
img = cv2.imread(image_path)
keypoints = results[0].keypoints.data.cpu().numpy()
for person_kpts in keypoints:
    for x, y, conf in person_kpts:
        if conf > 0.3:
            cv2.circle(img, (int(x), int(y)), 5, (0, 255, 0), -1)
cv2.imwrite('custom_skeleton.jpg', img)
# 获取第一张结果图
'''result = results[0]

# 绘制人体骨架和关键点
annotated_img = result.plot(pose_mask = 0.0)

# 保存结果图片
output_path = 'skeleton_result.jpg'
Image.fromarray(annotated_img).save(output_path)

print(f"骨架识别结果已保存至: {output_path}")'''