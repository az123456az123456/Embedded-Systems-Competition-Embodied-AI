#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""中文语义指令解析器。

把自然语言指令解析成结构化意图，供任务6/导航复用。

示例：
    "找到红色水瓶"  -> SemanticIntent(action=SEARCH, target=bottle, color=red)
    "跟着那个人"    -> SemanticIntent(action=FOLLOW, target=person, color=None)
    "停止"          -> SemanticIntent(action=STOP, target=None, color=None)
    "走到左上角"    -> SemanticIntent(action=GOTO, target=None, color=None, zone=top-left)
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Optional


COLOR_ALIASES = {
    "红": "red",
    "红色": "red",
    "绿色": "green",
    "绿": "green",
    "蓝色": "blue",
    "蓝": "blue",
}

TARGET_ALIASES = {
    "人": "person",
    "人员": "person",
    "行人": "person",
    "同学": "person",
    "队友": "person",
    "陌生人": "person",
    "水瓶": "bottle",
    "瓶子": "bottle",
    "水壶": "bottle",
    "饮料瓶": "bottle",
    "杯子": "cup",
    "水杯": "cup",
    "马克杯": "cup",
    "手机": "cell phone",
    "电话": "cell phone",
    "书包": "backpack",
    "椅子": "chair",
    "桌子": "dining table",
    "球": "sports ball",
}

ACTION_ALIASES = {
    "找到": "SEARCH",
    "找": "SEARCH",
    "搜索": "SEARCH",
    "寻找": "SEARCH",
    "识别": "SEARCH",
    "跟着": "FOLLOW",
    "跟随": "FOLLOW",
    "追踪": "FOLLOW",
    "盯": "FOLLOW",
    "走到": "GOTO",
    "去": "GOTO",
    "移动到": "GOTO",
    "导航到": "GOTO",
    "抓取": "GRAB",
    "拿": "GRAB",
    "捡起": "GRAB",
    "停车": "STOP",
    "停止": "STOP",
    "停": "STOP",
}

ZONE_ALIASES = {
    "左上角": "top-left",
    "左下角": "bottom-left",
    "右上角": "top-right",
    "右下角": "bottom-right",
    "中心": "center",
    "中央": "center",
    "终点": "goal",
}


@dataclass(frozen=True)
class SemanticIntent:
    raw: str
    action: str = "SEARCH"
    target: Optional[str] = None
    color: Optional[str] = None
    zone: Optional[str] = None
    unknown_terms: tuple = field(default_factory=tuple)


def _normalize(text: str) -> str:
    text = text.strip().lower()
    text = re.sub(r"[，。！？!?、\s]+", " ", text)
    return text


def _find_alias(text: str, aliases: dict[str, str]) -> Optional[str]:
    for key in sorted(aliases, key=len, reverse=True):
        if key in text:
            return aliases[key]
    return None


def parse_instruction(text: str) -> SemanticIntent:
    """解析一句中文指令，无法识别目标时抛出 ValueError。"""

    raw = text.strip()
    normalized = _normalize(text)
    if not normalized:
        raise ValueError("指令为空")

    action = _find_alias(normalized, ACTION_ALIASES) or "SEARCH"
    target = _find_alias(normalized, TARGET_ALIASES)
    color = _find_alias(normalized, COLOR_ALIASES)
    zone = _find_alias(normalized, ZONE_ALIASES)

    if action == "STOP":
        pass
    elif target is None and zone is None:
        # 至少需要目标或区域，否则没有可执行对象。
        raise ValueError(f"无法从指令中识别目标：{raw!r}")

    unknown = tuple(
        token for token in normalized.split()
        if not any(alias in token for alias in ACTION_ALIASES)
        and not any(alias in token for alias in TARGET_ALIASES)
        and not any(alias in token for alias in COLOR_ALIASES)
        and not any(alias in token for alias in ZONE_ALIASES)
    )
    return SemanticIntent(
        raw=raw,
        action=action,
        target=target,
        color=color,
        zone=zone,
        unknown_terms=unknown,
    )
