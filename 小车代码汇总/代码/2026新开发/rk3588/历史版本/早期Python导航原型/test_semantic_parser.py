#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""中文语义解析单元测试。"""

import unittest

from semantic_parser import parse_instruction


class SemanticParserTest(unittest.TestCase):
    def test_find_red_bottle(self):
        intent = parse_instruction("找到红色水瓶")
        self.assertEqual(intent.action, "SEARCH")
        self.assertEqual(intent.target, "bottle")
        self.assertEqual(intent.color, "red")

    def test_follow_person(self):
        intent = parse_instruction("跟着那个人")
        self.assertEqual(intent.action, "FOLLOW")
        self.assertEqual(intent.target, "person")
        self.assertIsNone(intent.color)

    def test_stop(self):
        intent = parse_instruction("停止")
        self.assertEqual(intent.action, "STOP")
        self.assertIsNone(intent.target)

    def test_parking(self):
        intent = parse_instruction("停车")
        self.assertEqual(intent.action, "STOP")

    def test_goto_zone(self):
        intent = parse_instruction("走到左上角")
        self.assertEqual(intent.action, "GOTO")
        self.assertEqual(intent.zone, "top-left")

    def test_unknown_raises(self):
        with self.assertRaises(ValueError):
            parse_instruction("随便看看")


if __name__ == "__main__":
    unittest.main()
