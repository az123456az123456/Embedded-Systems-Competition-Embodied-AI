#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
深度 ReID 特征提取器 —— ResNet50-IBN-a + BNNeck（2048 维）

模型结构取自开源项目 yolov8_reid（https://github.com/YINYIPENG-EN/yolov8_reid）
的 modeling/backbones/resnet_ibn_a.py 与 modeling/baseline.py（开放源码，免费）。
权重 ReID_resnet50_ibn_a.pth（Market1501 训练，README 标称 mAP 92% / Rank-1 97%）。

用法：
    reid = DeepReID(weight_path="yolov8_reid/ReID_resnet50_ibn_a.pth")
    feat = reid.extract(bgr_crop)            # -> np.float32 (2048,)，已 L2 归一化
    sim  = reid.cosine(feat_a, feat_b)       # -> [-1, 1]，越大越像

相比 HSV 颜色直方图，深度特征看的是语义外观（纹理/形状/颜色组合），
对衣服颜色接近、光线变化、距离变化都稳得多。
"""

import cv2
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

__all__ = ["DeepReID"]

# 训练时使用的输入尺寸（H, W）与 ImageNet 归一化参数，必须和训练一致
_INPUT_H, _INPUT_W = 384, 128
_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


class IBN(nn.Module):
    """Instance-Batch Normalization：一半通道走 IN、一半走 BN，抗光照/风格变化。"""

    def __init__(self, planes):
        super().__init__()
        half1 = int(planes / 2)
        self.half = half1
        half2 = planes - half1
        self.IN = nn.InstanceNorm2d(half1, affine=True)
        self.BN = nn.BatchNorm2d(half2)

    def forward(self, x):
        split = torch.split(x, self.half, 1)
        out1 = self.IN(split[0].contiguous())
        out2 = self.BN(split[1].contiguous())
        return torch.cat((out1, out2), 1)


class Bottleneck_IBN(nn.Module):
    expansion = 4

    def __init__(self, inplanes, planes, ibn=False, stride=1, downsample=None):
        super().__init__()
        self.conv1 = nn.Conv2d(inplanes, planes, kernel_size=1, bias=False)
        self.bn1 = IBN(planes) if ibn else nn.BatchNorm2d(planes)
        self.conv2 = nn.Conv2d(planes, planes, kernel_size=3, stride=stride,
                               padding=1, bias=False)
        self.bn2 = nn.BatchNorm2d(planes)
        self.conv3 = nn.Conv2d(planes, planes * self.expansion, kernel_size=1, bias=False)
        self.bn3 = nn.BatchNorm2d(planes * self.expansion)
        self.relu = nn.ReLU(inplace=True)
        self.downsample = downsample
        self.stride = stride

    def forward(self, x):
        residual = x
        out = self.conv1(x)
        out = self.bn1(out)
        out = self.relu(out)
        out = self.conv2(out)
        out = self.bn2(out)
        out = self.relu(out)
        out = self.conv3(out)
        out = self.bn3(out)
        if self.downsample is not None:
            residual = self.downsample(x)
        out += residual
        out = self.relu(out)
        return out


class ResNet_IBN(nn.Module):
    """ResNet50-IBN-a 主干。last_stride=1 使 layer4 不下采样，得到更大的特征图。"""

    def __init__(self, last_stride, block, layers):
        scale = 64
        self.inplanes = scale
        super().__init__()
        self.conv1 = nn.Conv2d(3, scale, kernel_size=7, stride=2, padding=3, bias=False)
        self.bn1 = nn.BatchNorm2d(scale)
        self.relu = nn.ReLU(inplace=True)
        self.maxpool = nn.MaxPool2d(kernel_size=3, stride=2, padding=1)
        self.layer1 = self._make_layer(block, scale, layers[0])
        self.layer2 = self._make_layer(block, scale * 2, layers[1], stride=2)
        self.layer3 = self._make_layer(block, scale * 4, layers[2], stride=2)
        self.layer4 = self._make_layer(block, scale * 8, layers[3], stride=last_stride)

    def _make_layer(self, block, planes, blocks, stride=1):
        downsample = None
        if stride != 1 or self.inplanes != planes * block.expansion:
            downsample = nn.Sequential(
                nn.Conv2d(self.inplanes, planes * block.expansion,
                          kernel_size=1, stride=stride, bias=False),
                nn.BatchNorm2d(planes * block.expansion),
            )
        layers = []
        ibn = True
        if planes == 512:
            ibn = False
        layers.append(block(self.inplanes, planes, ibn, stride, downsample))
        self.inplanes = planes * block.expansion
        for _ in range(1, blocks):
            layers.append(block(self.inplanes, planes, ibn))
        return nn.Sequential(*layers)

    def forward(self, x):
        x = self.conv1(x)
        x = self.bn1(x)
        x = self.relu(x)
        x = self.maxpool(x)
        x = self.layer1(x)
        x = self.layer2(x)
        x = self.layer3(x)
        x = self.layer4(x)
        return x


class DeepReID(nn.Module):
    """加载 ReID_resnet50_ibn_a.pth，对人形裁剪图提 2048 维 L2 归一化特征。"""

    def __init__(self, weight_path, device="cpu"):
        super().__init__()
        self.backbone = ResNet_IBN(1, Bottleneck_IBN, [3, 4, 6, 3])
        self.gap = nn.AdaptiveAvgPool2d(1)
        self.bottleneck = nn.BatchNorm1d(2048)   # BNNeck
        self.device = device

        state = torch.load(weight_path, map_location="cpu")
        if isinstance(state, dict) and "state_dict" in state:
            state = state["state_dict"]
        # 权重里主干叫 base.*，这里把模型主干改名为 backbone.*；
        # 同时丢弃 classifier（Market1501 分类头）和 base.fc（ImageNet 1000 类头）。
        remap = {}
        for k, v in state.items():
            if k.startswith("classifier.") or k.startswith("base.fc."):
                continue
            if k.startswith("base."):
                k = "backbone." + k[len("base."):]
            remap[k] = v
        missing, unexpected = self.load_state_dict(remap, strict=False)
        if missing or unexpected:
            print(f"[DeepReID] 权重键不完全匹配：缺 {len(missing)} 个、多 {len(unexpected)} 个")
            if missing:
                print("  缺:", missing[:3])
            if unexpected:
                print("  多:", unexpected[:3])

        # 该权重里混有海量"次正规数"（~1e-45，占非零参数约 80%），在 x86 CPU 上
        # 涉及次正规数的运算会走微码异常处理，慢 10~100 倍。这些值对输出贡献
        # ~1e-38（≈0），可安全清零，实测前向 2.9s → 53ms 且特征不变。
        n_sub = 0
        for p in self.parameters():
            mask = p.data.abs() < 1.18e-38
            n_sub += int(mask.sum().item())
            p.data.masked_fill_(mask, 0.0)
        if n_sub:
            print(f"[DeepReID] 已清零 {n_sub} 个次正规数权重（提速约 50x，特征不变）")

        self.to(device)
        self.eval()
        for p in self.parameters():
            p.requires_grad_(False)

        # 运行时同样打开硬件级 flush-denormal，避免激活值里再出现次正规数拖慢速度
        try:
            torch.set_flush_denormal(True)
        except Exception:
            pass

    def forward(self, x):
        feat = self.gap(self.backbone(x))       # (b, 2048, 1, 1)
        feat = feat.view(feat.shape[0], -1)     # (b, 2048)
        feat = self.bottleneck(feat)            # BNNeck
        return F.normalize(feat, dim=1, p=2)    # L2 归一化 → 等价余弦

    def extract(self, bgr_roi):
        """输入 cv2 BGR 人形裁剪图，输出 2048 维 L2 归一化特征 np.float32。

        图太小（<12px）时返回 None，由调用方跳过。
        """
        if bgr_roi is None or bgr_roi.size == 0:
            return None
        h, w = bgr_roi.shape[:2]
        if h < 12 or w < 12:
            return None

        img = cv2.resize(bgr_roi, (_INPUT_W, _INPUT_H))      # (W, H) = (128, 384)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        t = torch.from_numpy(np.ascontiguousarray(img)).permute(2, 0, 1).float() / 255.0
        mean = torch.from_numpy(_MEAN).view(3, 1, 1)
        std = torch.from_numpy(_STD).view(3, 1, 1)
        t = (t - mean) / std

        with torch.no_grad():
            feat = self.forward(t.unsqueeze(0).to(self.device))
        return feat[0].cpu().numpy().astype(np.float32)

    @staticmethod
    def cosine(f1, f2):
        """余弦相似度 [-1, 1]，越大越像。特征已 L2 归一化时点积即可，这里仍做归一化保险。"""
        if f1 is None or f2 is None:
            return -1.0
        n1 = float(np.linalg.norm(f1))
        n2 = float(np.linalg.norm(f2))
        if n1 == 0.0 or n2 == 0.0:
            return -1.0
        return float(np.dot(f1, f2) / (n1 * n2))
