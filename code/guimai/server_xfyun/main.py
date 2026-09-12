#!/usr/bin/env python3
from __future__ import annotations

import argparse
import base64
import hashlib
import hmac
import json
import logging
import math
import os
import re
import socket
import ssl
import time
import unicodedata
import wave
from dataclasses import dataclass
from datetime import datetime
from time import mktime
from typing import List
from urllib.parse import urlencode
from wsgiref.handlers import format_date_time

from websocket import WebSocketTimeoutException, create_connection

MAGIC_HEADER = b"\xA1\xB2\xC3\xD4"
FRAME_SIZE = 645
PAYLOAD_OFFSET = 5

STATUS_FIRST_FRAME = 0
STATUS_CONTINUE_FRAME = 1
STATUS_LAST_FRAME = 2

g_continue_packet_count = 0
g_dump_fp = None
g_audio_parts: List[bytes] = []
g_audio_start_ts = 0.0
g_audio_last_ts = 0.0
g_audio_meta_rate = 0
g_audio_meta_samples = 0
g_audio_meta_elapsed_ms = 0
g_debug_xfyun = False

MAX_RECORD_SECONDS = 60.0
COMMAND_MODE_LEGACY = "legacy"
COMMAND_MODE_SEQUENCE = "sequence"
DEFAULT_XFYUN_CONNECT_TIMEOUT_S = 5.0
DEFAULT_XFYUN_RECV_TIMEOUT_S = 0.1
DEFAULT_XFYUN_SEND_TIMEOUT_S = 10.0
DEFAULT_XFYUN_FINAL_TIMEOUT_S = 20.0
DEFAULT_XFYUN_CHUNK_MS = 100


@dataclass
class XfCredential:
    appid: str
    api_key: str
    api_secret: str


def create_xfyun_url(cred: XfCredential) -> str:
    host = "ws-api.xfyun.cn"
    base_url = "wss://ws-api.xfyun.cn/v2/iat"
    date = format_date_time(mktime(datetime.now().timetuple()))
    signature_origin = f"host: {host}\n" f"date: {date}\n" "GET /v2/iat HTTP/1.1"
    signature_sha = hmac.new(
        cred.api_secret.encode("utf-8"),
        signature_origin.encode("utf-8"),
        digestmod=hashlib.sha256,
    ).digest()
    signature_sha = base64.b64encode(signature_sha).decode("utf-8")
    authorization_origin = (
        f'api_key="{cred.api_key}", algorithm="hmac-sha256", '
        f'headers="host date request-line", signature="{signature_sha}"'
    )
    authorization = base64.b64encode(authorization_origin.encode("utf-8")).decode("utf-8")
    return f"{base_url}?{urlencode({'authorization': authorization, 'date': date, 'host': host})}"


def parse_xfyun_message(message: str) -> tuple[str, bool, dict]:
    try:
        obj = json.loads(message)
    except json.JSONDecodeError:
        logging.warning("xfyun json decode failed, raw=%r", message[:160])
        return "", False, {}

    code = int(obj.get("code", -1))
    if code != 0:
        logging.warning("xfyun error: code=%s message=%s raw=%s", code, obj.get("message", ""), message[:1000])
        return "", True, obj

    data = obj.get("data", {})
    result = data.get("result", {})

    text = []
    for ws_item in result.get("ws", []):
        for cw_item in ws_item.get("cw", []):
            value = cw_item.get("w", "")
            if value:
                text.append(value)

    is_last = bool(result.get("ls")) or int(data.get("status", 1)) == 2
    if g_debug_xfyun:
        logging.info("xfyun raw status=%s ls=%s text=%r raw=%s", data.get("status", ""), result.get("ls", ""), "".join(text), message[:1000])

    if is_last and not text:
        logging.warning(
            "xfyun last frame but empty text, status=%s result=%s",
            data.get("status", ""),
            result,
        )
    return "".join(text), is_last, obj


def _append_unique(items: List[str], value: str) -> None:
    if value and value not in items:
        items.append(value)


def _wrong_gbk_to_utf8(text: str, encoding: str) -> str:
    raw = bytearray()
    for ch in text:
        # Windows mojibake often shows UTF-8 byte 0x80 as U+20AC while the
        # surrounding bytes were decoded as GBK/CP936.
        if ch == "\u20ac":
            raw.append(0x80)
            continue
        try:
            raw.extend(ch.encode(encoding))
        except UnicodeEncodeError:
            continue
    if not raw:
        return ""
    return bytes(raw).decode("utf-8", errors="replace")


def _normalize_asr_command_text(text: str) -> str:
    normalized = text
    for token in ("0", "O", "o", "\u96f6", "\u3007"):
        normalized = normalized.replace("\u95e8" + token, "\u95e8\u6d1e")
    return normalized


def text_match_candidates(text: str) -> List[str]:
    candidates: List[str] = []
    _append_unique(candidates, text.strip())
    compact = "".join(text.split())
    _append_unique(candidates, compact)
    normalized = _normalize_asr_command_text(text).strip()
    _append_unique(candidates, normalized)
    _append_unique(candidates, "".join(normalized.split()))
    for encoding in ("gbk", "cp936", "gb18030"):
        repaired = _wrong_gbk_to_utf8(text, encoding).strip()
        _append_unique(candidates, repaired)
        _append_unique(candidates, repaired.replace("\ufffd", ""))
        _append_unique(candidates, "".join(repaired.split()))
        repaired_normalized = _normalize_asr_command_text(repaired).strip()
        _append_unique(candidates, repaired_normalized)
        _append_unique(candidates, "".join(repaired_normalized.split()))
    return candidates


class XfyunSession:
    def __init__(
        self,
        cred: XfCredential,
        connect_timeout_s: float = DEFAULT_XFYUN_CONNECT_TIMEOUT_S,
        recv_timeout_s: float = DEFAULT_XFYUN_RECV_TIMEOUT_S,
        send_timeout_s: float = DEFAULT_XFYUN_SEND_TIMEOUT_S,
    ) -> None:
        self._cred = cred
        self._ws = None
        self.last_raw = ""
        self.last_json: dict = {}
        self.recv_count = 0
        self.connect_timeout_s = connect_timeout_s
        self.recv_timeout_s = recv_timeout_s
        self.send_timeout_s = send_timeout_s

    def connect(self) -> None:
        url = create_xfyun_url(self._cred)
        self._ws = create_connection(url, timeout=self.connect_timeout_s, sslopt={"cert_reqs": ssl.CERT_NONE})
        self._ws.settimeout(self.recv_timeout_s)

    def close(self) -> None:
        if self._ws is not None:
            try:
                self._ws.close()
            finally:
                self._ws = None

    def send_audio(self, status: int, payload: bytes) -> None:
        if self._ws is None:
            raise RuntimeError("xfyun websocket is not connected")

        frame = {
            "data": {
                "status": status,
                "format": "audio/L16;rate=16000",
                "audio": base64.b64encode(payload).decode("utf-8"),
                "encoding": "raw",
            }
        }
        if status == STATUS_FIRST_FRAME:
            frame["common"] = {"app_id": self._cred.appid}
            frame["business"] = {
                "domain": "iat",
                "language": "zh_cn",
                "accent": "mandarin",
                "vinfo": 1,
                "vad_eos": 10000,
            }
        self._ws.settimeout(self.send_timeout_s)
        try:
            self._ws.send(json.dumps(frame, ensure_ascii=False))
        finally:
            self._ws.settimeout(self.recv_timeout_s)

    def recv_text(self, timeout_s: float) -> tuple[str, bool]:
        if self._ws is None:
            return "", False

        deadline = time.time() + timeout_s
        chunks: List[str] = []
        is_last = False
        while time.time() < deadline:
            try:
                msg = self._ws.recv()
            except WebSocketTimeoutException:
                if timeout_s < 1.0:
                    break
                continue
            self.recv_count += 1
            self.last_raw = msg
            text, is_last, obj = parse_xfyun_message(msg)
            if obj:
                self.last_json = obj
            if text:
                chunks.append(text)
            if is_last:
                break
        return "".join(chunks), is_last


_BROKEN_COMMAND_KEYWORDS_TEXT = r'''
COMMAND_KEYWORDS = {
    1: ["向前直行十米", "向前直行"],
    2: ["后退直行十米", "倒车直行十米", "后退直行", "倒车直行"],
    3: ["打开双闪灯", "打开双闪", "开双闪", "双闪"],
    4: ["打开左转灯", "打开左转", "开左转", "左转"],
    5: ["打开右转灯", "打开右转", "开右转", "右转"],
    6: ["打开近光灯", "打开近光", "开近光", "近光"],
    7: ["打开远光灯", "打开远光", "开远光", "远光"],
    8: ["打开雾灯", "开雾灯", "雾灯"],
    9: ["蛇形前进十米", "蛇形前进"],
    10: ["蛇形后退十米", "蛇形后退"],
    11: ["顺时针转一圈", "顺时针转", "顺时针"],
    12: ["逆时针转一圈", "逆时针转", "逆时针"],
    13: ["停进停车区一", "停车区一"],
    14: ["停进停车区二", "停车区二"],
    15: ["停进停车区三", "停车区三"],
    16: ["三八二一"],
    17: ["引擎授权", "引擎", "授权"],
    18: ["科目一"],
    19: ["科目二"],
    20: ["科目三"],
    21: ["科目四"],
    22: ["测试"],
    23: ["停止播放", "停止"],
    24: ["播放", "播放音乐"],
    25: ["你好"],
    26: ["暂停播放", "暂停"],
    27: ["继续播放", "继续"],
    28: ["上一首"],
    29: ["下一首"],
}


'''

_BROKEN_COMMAND_KEYWORDS_TEXT_2 = r'''
COMMAND_KEYWORDS = {
    1: ["向前直行十米", "向前直行"],
    2: ["后退直行十米", "倒车直行十米", "后退直行", "倒车直行"],
    3: ["打开双闪灯", "打开双闪", "开双闪", "双闪"],
    4: ["打开左转灯", "打开左转", "开左转", "左转"],
    5: ["打开右转灯", "打开右转", "开右转", "右转"],
    6: ["打开近光灯", "打开近光", "开近光", "近光"],
    7: ["打开远光灯", "打开远光", "开远光", "远光"],
    8: ["打开雾灯", "开雾灯", "雾灯"],
    9: ["蛇形前进十米", "蛇形前进"],
    10: ["蛇形后退十米", "蛇形后退"],
    11: ["顺时针转一圈", "顺时针转", "顺时针"],
    12: ["逆时针转一圈", "逆时针转", "逆时针"],
    13: ["停进停车区一", "停车区一"],
    14: ["停进停车区二", "停车区二"],
    15: ["停进停车区三", "停车区三"],
    16: ["三八二一"],
    17: ["引擎授权", "引擎", "授权"],
    18: ["科目一"],
    19: ["科目二"],
    20: ["科目三"],
    21: ["科目四"],
    22: ["测试"],
    23: ["停止播放", "停止"],
    24: ["播放", "播放音乐"],
    25: ["你好"],
    26: ["暂停播放", "暂停"],
    27: ["继续播放", "继续"],
    28: ["上一首"],
    29: ["下一首"],
}
'''

COMMAND_KEYWORDS = {
    1: ["\u5411\u524d\u76f4\u884c\u5341\u7c73", "\u5411\u524d\u76f4\u884c10\u7c73", "\u5411\u524d\u76f4\u884c", "\u76f4\u884c\u5341\u7c73", "\u76f4\u884c10\u7c73", "\u524d\u884c\u5341\u7c73", "\u524d\u884c10\u7c73", "\u524d\u8fdb\u5341\u7c73", "\u524d\u8fdb10\u7c73", "\u5411\u524d\u5341\u7c73", "\u5411\u524d10\u7c73"],
    2: ["\u540e\u9000\u76f4\u884c\u5341\u7c73", "\u540e\u9000\u76f4\u884c10\u7c73", "\u5012\u8f66\u76f4\u884c\u5341\u7c73", "\u5012\u8f66\u76f4\u884c10\u7c73", "\u540e\u9000\u76f4\u884c", "\u5012\u8f66\u76f4\u884c", "\u540e\u9000\u5341\u7c73", "\u540e\u900010\u7c73", "\u540e\u9000\u5341\u7c73", "\u540e\u900010\u7c73"],
    3: ["\u6253\u5f00\u53cc\u95ea\u706f", "\u6253\u5f00\u53cc\u95ea", "\u5f00\u53cc\u95ea", "\u53cc\u95ea"],
    4: ["\u6253\u5f00\u5de6\u8f6c\u5411\u706f", "\u6253\u5f00\u5de6\u8f6c\u706f", "\u6253\u5f00\u5de6\u8f6c", "\u5f00\u5de6\u8f6c\u5411\u706f", "\u5f00\u5de6\u8f6c\u706f", "\u5f00\u5de6\u8f6c", "\u5de6\u8f6c\u5411\u706f", "\u5de6\u8f6c\u706f"],
    5: ["\u6253\u5f00\u53f3\u8f6c\u5411\u706f", "\u6253\u5f00\u53f3\u8f6c\u706f", "\u6253\u5f00\u53f3\u8f6c", "\u5f00\u53f3\u8f6c\u5411\u706f", "\u5f00\u53f3\u8f6c\u706f", "\u5f00\u53f3\u8f6c", "\u53f3\u8f6c\u5411\u706f", "\u53f3\u8f6c\u706f"],
    6: ["\u6253\u5f00\u8fd1\u5149\u706f", "\u6253\u5f00\u8fd1\u5149", "\u5f00\u8fd1\u5149", "\u8fd1\u5149"],
    7: ["\u6253\u5f00\u8fdc\u5149\u706f", "\u6253\u5f00\u8fdc\u5149", "\u5f00\u8fdc\u5149", "\u8fdc\u5149"],
    8: ["\u6253\u5f00\u96fe\u706f", "\u5f00\u96fe\u706f", "\u96fe\u706f"],
    9: ["\u86c7\u5f62\u524d\u8fdb\u5341\u7c73", "\u86c7\u5f62\u524d\u8fdb10\u7c73", "\u86c7\u5f62\u524d\u8fdb"],
    10: ["\u86c7\u5f62\u540e\u9000\u5341\u7c73", "\u86c7\u5f62\u540e\u900010\u7c73", "\u86c7\u5f62\u540e\u9000"],
    11: ["\u987a\u65f6\u9488\u8f6c\u4e00\u5708", "\u987a\u65f6\u9488\u8f6c", "\u987a\u65f6\u9488"],
    12: ["\u9006\u65f6\u9488\u8f6c\u4e00\u5708", "\u9006\u65f6\u9488\u8f6c", "\u9006\u65f6\u9488"],
    13: ["\u505c\u8fdb\u505c\u8f66\u533a\u4e00", "\u505c\u8fdb\u505c\u8f66\u533a1", "\u505c\u8f66\u533a\u4e00", "\u505c\u8f66\u533a1"],
    14: ["\u505c\u8fdb\u505c\u8f66\u533a\u4e8c", "\u505c\u8fdb\u505c\u8f66\u533a2", "\u505c\u8f66\u533a\u4e8c", "\u505c\u8f66\u533a2"],
    15: ["\u505c\u8fdb\u505c\u8f66\u533a\u4e09", "\u505c\u8fdb\u505c\u8f66\u533a3", "\u505c\u8f66\u533a\u4e09", "\u505c\u8f66\u533a3"],
    16: ["\u4e09\u516b\u4e8c\u4e00"],
    17: ["\u5f15\u64ce\u6388\u6743", "\u5f15\u64ce", "\u6388\u6743"],
    18: ["\u79d1\u76ee\u4e00"],
    19: ["\u79d1\u76ee\u4e8c"],
    20: ["\u79d1\u76ee\u4e09"],
    21: ["\u79d1\u76ee\u56db"],
    22: ["\u6d4b\u8bd5"],
    23: ["\u505c\u6b62\u64ad\u653e", "\u505c\u6b62"],
    24: ["\u64ad\u653e", "\u64ad\u653e\u97f3\u4e50"],
    25: ["\u4f60\u597d"],
    26: ["\u6682\u505c\u64ad\u653e", "\u6682\u505c"],
    27: ["\u7ee7\u7eed\u64ad\u653e", "\u7ee7\u7eed"],
    28: ["\u4e0a\u4e00\u9996"],
    29: ["\u4e0b\u4e00\u9996"],
    30: ["\u6253\u5f00\u8f66\u5185\u7167\u660e\u706f", "\u8f66\u5185\u7167\u660e\u706f", "\u8f66\u5185\u7167\u660e", "\u8f66\u5185\u706f", "\u7167\u660e\u706f"],
    31: ["\u6253\u5f00\u96e8\u5237\u5668", "\u6253\u5f00\u96e8\u5237", "\u5f00\u96e8\u5237", "\u96e8\u5237\u5668", "\u96e8\u5237"],
    32: ["\u9e23\u7b1b1\u79d2\u949f", "\u9e23\u7b1b\u4e00\u79d2\u949f", "\u9e23\u7b1b1\u79d2", "\u9e23\u7b1b\u4e00\u79d2"],
    33: ["\u9e23\u7b1b\u4e24\u79d2\u949f", "\u9e23\u7b1b\u4e8c\u79d2\u949f", "\u9e23\u7b1b2\u79d2\u949f", "\u9e23\u7b1b\u4e24\u79d2", "\u9e23\u7b1b\u4e8c\u79d2", "\u9e23\u7b1b2\u79d2"],
    34: ["\u9e23\u7b1b\u4e09\u79d2\u949f", "\u9e23\u7b1b3\u79d2\u949f", "\u9e23\u7b1b\u4e09\u79d2", "\u9e23\u7b1b3\u79d2"],
    35: ["\u9e23\u7b1b\u4e24\u58f0", "\u9e23\u7b1b\u4e8c\u58f0", "\u9e23\u7b1b2\u58f0"],
    36: ["\u9e23\u7b1b\u4e09\u58f0", "\u9e23\u7b1b3\u58f0"],
    37: ["\u9e23\u7b1b\u56db\u58f0", "\u9e23\u7b1b4\u58f0"],
    38: ["\u957f\u77ed\u9e23\u7b1b"],
    39: ["\u6025\u4fc3\u9e23\u7b1b"],
    40: ["\u8b66\u62a5\u9e23\u7b1b"],
    41: ["\u5de6\u8f6c", "\u5411\u5de6\u8f6c", "\u8f66\u8f86\u5de6\u8f6c", "\u8f66\u4f53\u5de6\u8f6c", "\u5de6\u8f6c\u4e5d\u5341\u5ea6", "\u5de6\u8f6c90\u5ea6"],
    42: ["\u53f3\u8f6c", "\u5411\u53f3\u8f6c", "\u8f66\u8f86\u53f3\u8f6c", "\u8f66\u4f53\u53f3\u8f6c", "\u53f3\u8f6c\u4e5d\u5341\u5ea6", "\u53f3\u8f6c90\u5ea6"],
    43: ["\u95e8\u6d1e\u4e00\u53f3\u4fa7\u8fd4\u56de", "\u95e8\u6d1e1\u53f3\u4fa7\u8fd4\u56de", "\u4e00\u53f7\u95e8\u6d1e\u53f3\u4fa7\u8fd4\u56de", "\u95e8\u6d1e\u4e00\u53f3\u8fd4\u56de", "\u95e8\u6d1e1\u53f3\u8fd4\u56de"],
    44: ["\u95e8\u6d1e\u4e00\u8fd4\u56de", "\u95e8\u6d1e1\u8fd4\u56de", "\u4e00\u53f7\u95e8\u6d1e\u8fd4\u56de", "\u95e8\u6d1e\u4e00\u539f\u8def\u8fd4\u56de", "\u95e8\u6d1e1\u539f\u8def\u8fd4\u56de"],
    45: ["\u95e8\u6d1e\u4e8c\u8fd4\u56de", "\u95e8\u6d1e2\u8fd4\u56de", "\u4e8c\u53f7\u95e8\u6d1e\u8fd4\u56de", "\u95e8\u6d1e\u4e8c\u539f\u8def\u8fd4\u56de", "\u95e8\u6d1e2\u539f\u8def\u8fd4\u56de"],
    46: ["\u95e8\u6d1e\u4e09\u8fd4\u56de", "\u95e8\u6d1e3\u8fd4\u56de", "\u4e09\u53f7\u95e8\u6d1e\u8fd4\u56de", "\u95e8\u6d1e\u4e09\u539f\u8def\u8fd4\u56de", "\u95e8\u6d1e3\u539f\u8def\u8fd4\u56de"],
    47: ["\u95e8\u6d1e\u4e09\u5de6\u4fa7\u8fd4\u56de", "\u95e8\u6d1e3\u5de6\u4fa7\u8fd4\u56de", "\u4e09\u53f7\u95e8\u6d1e\u5de6\u4fa7\u8fd4\u56de", "\u95e8\u6d1e\u4e09\u5de6\u8fd4\u56de", "\u95e8\u6d1e3\u5de6\u8fd4\u56de"],
    48: ["\u901a\u8fc7\u95e8\u6d1e\u4e00\u5de6\u4fa7", "\u901a\u8fc7\u95e8\u6d1e1\u5de6\u4fa7", "\u901a\u8fc7\u4e00\u53f7\u95e8\u6d1e\u5de6\u4fa7", "\u95e8\u6d1e\u4e00\u5de6\u4fa7", "\u95e8\u6d1e1\u5de6\u4fa7"],
    49: ["\u901a\u8fc7\u95e8\u6d1e\u4e00\u6d1e", "\u901a\u8fc7\u95e8\u6d1e1\u6d1e", "\u901a\u8fc7\u95e8\u6d1e\u4e00", "\u901a\u8fc7\u95e8\u6d1e1", "\u901a\u8fc7\u4e00\u53f7\u95e8\u6d1e", "\u95e8\u6d1e\u4e00\u6d1e", "\u95e8\u6d1e1\u6d1e"],
    50: ["\u901a\u8fc7\u95e8\u6d1e\u4e8c", "\u901a\u8fc7\u95e8\u6d1e2", "\u901a\u8fc7\u4e8c\u53f7\u95e8\u6d1e", "\u95e8\u6d1e\u4e8c\u901a\u8fc7", "\u95e8\u6d1e2\u901a\u8fc7"],
    51: ["\u901a\u8fc7\u95e8\u6d1e\u4e09", "\u901a\u8fc7\u95e8\u6d1e3", "\u901a\u8fc7\u4e09\u53f7\u95e8\u6d1e", "\u95e8\u6d1e\u4e09\u901a\u8fc7", "\u95e8\u6d1e3\u901a\u8fc7"],
    52: ["\u901a\u8fc7\u95e8\u6d1e\u4e09\u53f3\u4fa7", "\u901a\u8fc7\u95e8\u6d1e3\u53f3\u4fa7", "\u901a\u8fc7\u4e09\u53f7\u95e8\u6d1e\u53f3\u4fa7", "\u95e8\u6d1e\u4e09\u53f3\u4fa7", "\u95e8\u6d1e3\u53f3\u4fa7"],
}


def _extend_asr_keyword_aliases() -> None:
    for keys in COMMAND_KEYWORDS.values():
        for key in list(keys):
            if "\u95e8\u6d1e" not in key:
                continue
            for alias_head in ("\u95e80", "\u95e8O", "\u95e8o", "\u95e8\u96f6", "\u95e8\u3007"):
                alias = key.replace("\u95e8\u6d1e", alias_head)
                if alias not in keys:
                    keys.append(alias)


_extend_asr_keyword_aliases()


COMMAND_NAMES = {
    1: "\u5411\u524d\u76f4\u884c\u5341\u7c73",
    2: "\u540e\u9000\u76f4\u884c\u5341\u7c73",
    3: "\u6253\u5f00\u53cc\u95ea\u706f",
    4: "\u6253\u5f00\u5de6\u8f6c\u5411\u706f",
    5: "\u6253\u5f00\u53f3\u8f6c\u5411\u706f",
    6: "\u6253\u5f00\u8fd1\u5149\u706f",
    7: "\u6253\u5f00\u8fdc\u5149\u706f",
    8: "\u6253\u5f00\u96fe\u706f",
    9: "\u86c7\u5f62\u524d\u8fdb\u5341\u7c73",
    10: "\u86c7\u5f62\u540e\u9000\u5341\u7c73",
    11: "\u987a\u65f6\u9488\u8f6c\u4e00\u5708",
    12: "\u9006\u65f6\u9488\u8f6c\u4e00\u5708",
    13: "\u505c\u8fdb\u505c\u8f66\u533a\u4e00",
    14: "\u505c\u8fdb\u505c\u8f66\u533a\u4e8c",
    15: "\u505c\u8fdb\u505c\u8f66\u533a\u4e09",
    16: "\u4e09\u516b\u4e8c\u4e00",
    17: "\u5f15\u64ce\u6388\u6743",
    18: "\u79d1\u76ee\u4e00",
    19: "\u79d1\u76ee\u4e8c",
    20: "\u79d1\u76ee\u4e09",
    21: "\u79d1\u76ee\u56db",
    22: "\u6d4b\u8bd5",
    23: "\u505c\u6b62\u64ad\u653e",
    24: "\u64ad\u653e\u97f3\u4e50",
    25: "\u4f60\u597d",
    26: "\u6682\u505c\u64ad\u653e",
    27: "\u7ee7\u7eed\u64ad\u653e",
    28: "\u4e0a\u4e00\u9996",
    29: "\u4e0b\u4e00\u9996",
    30: "\u6253\u5f00\u8f66\u5185\u7167\u660e\u706f",
    31: "\u6253\u5f00\u96e8\u5237\u5668",
    32: "\u9e23\u7b1b1\u79d2\u949f",
    33: "\u9e23\u7b1b\u4e24\u79d2\u949f",
    34: "\u9e23\u7b1b\u4e09\u79d2\u949f",
    35: "\u9e23\u7b1b\u4e24\u58f0",
    36: "\u9e23\u7b1b\u4e09\u58f0",
    37: "\u9e23\u7b1b\u56db\u58f0",
    38: "\u957f\u77ed\u9e23\u7b1b",
    39: "\u6025\u4fc3\u9e23\u7b1b",
    40: "\u8b66\u62a5\u9e23\u7b1b",
    41: "\u5de6\u8f6c",
    42: "\u53f3\u8f6c",
    43: "\u95e8\u6d1e\u4e00\u53f3\u4fa7\u8fd4\u56de",
    44: "\u95e8\u6d1e\u4e00\u8fd4\u56de",
    45: "\u95e8\u6d1e\u4e8c\u8fd4\u56de",
    46: "\u95e8\u6d1e\u4e09\u8fd4\u56de",
    47: "\u95e8\u6d1e\u4e09\u5de6\u4fa7\u8fd4\u56de",
    48: "\u901a\u8fc7\u95e8\u6d1e\u4e00\u5de6\u4fa7",
    49: "\u901a\u8fc7\u95e8\u6d1e\u4e00\u6d1e",
    50: "\u901a\u8fc7\u95e8\u6d1e\u4e8c",
    51: "\u901a\u8fc7\u95e8\u6d1e\u4e09",
    52: "\u901a\u8fc7\u95e8\u6d1e\u4e09\u53f3\u4fa7",
}


_RULE_DIGIT_TRANSLATION = str.maketrans(
    {
        "０": "0",
        "１": "1",
        "２": "2",
        "３": "3",
        "４": "4",
        "\u96f6": "0",
        "\u3007": "0",
        "\u4e00": "1",
        "\u5e7a": "1",
        "\u58f9": "1",
        "\u4e8c": "2",
        "\u4e24": "2",
        "\u4fe9": "2",
        "\u8d30": "2",
        "\u4e09": "3",
        "\u53c1": "3",
        "\u56db": "4",
        "\u8086": "4",
        "\u5341": "10",
        "\u62fe": "10",
    }
)


def _strip_command_match_separators(text: str) -> str:
    return "".join(
        ch for ch in text
        if (not ch.isspace()) and (not unicodedata.category(ch).startswith("P"))
    )


def _canonicalize_rule_text(text: str) -> str:
    normalized = _strip_command_match_separators(text).translate(_RULE_DIGIT_TRANSLATION)
    normalized = normalized.replace("\u86c7\u884c", "\u86c7\u5f62")
    normalized = re.sub(r"\u95e8[0Oo]", "\u95e8\u6d1e", normalized)
    normalized = normalized.replace("\u95e8\u6d1e\u7ffb", "\u95e8\u6d1e3")
    normalized = normalized.replace("\u95e8\u7ffb", "\u95e8\u6d1e3")
    normalized = re.sub(r"\u95e8(?:\u6d1e)?[\u5df2\u4ee5](?=(?:\u5de6|\u53f3|\u6d1e|\u8fd4|\u539f|\u901a|\u524d|\u540e|\u76f4|[,，。；;]|$))", "\u95e8\u6d1e1", normalized)
    normalized = re.sub(r"([123])\u53f7\u95e8\u6d1e?", "\u95e8\u6d1e\\1", normalized)
    normalized = re.sub(r"\u95e8\u6d1e([123])\u53f7", "\u95e8\u6d1e\\1", normalized)
    normalized = re.sub(r"\u95e8([123])(?=(?:\u5de6|\u53f3|\u6d1e|\u8fd4|\u539f|\u901a|\u524d|\u540e|\u76f4|[,，。；;]|$))", "\u95e8\u6d1e\\1", normalized)
    return normalized


COMMAND_RULE_PATTERNS: List[tuple[int, tuple[str, ...]]] = [
    (4, (r"(?:\u6253\u5f00|\u5f00)?\u5de6\u8f6c(?:\u5411)?\u706f",)),
    (5, (r"(?:\u6253\u5f00|\u5f00)?\u53f3\u8f6c(?:\u5411)?\u706f",)),
    (3, (r"(?:\u6253\u5f00|\u5f00)?\u53cc\u95ea(?:\u706f)?",)),
    (6, (r"(?:\u6253\u5f00|\u5f00)?\u8fd1\u5149(?:\u706f)?",)),
    (7, (r"(?:\u6253\u5f00|\u5f00)?\u8fdc\u5149(?:\u706f)?",)),
    (8, (r"(?:\u6253\u5f00|\u5f00)?\u96fe\u706f",)),
    (30, (r"(?:\u6253\u5f00|\u5f00)?(?:\u8f66\u5185)?\u7167\u660e\u706f", r"\u8f66\u5185\u706f")),
    (31, (r"(?:\u6253\u5f00|\u5f00)?\u96e8\u5237(?:\u5668)?",)),
    (38, (r"\u957f\u77ed\u9e23\u7b1b",)),
    (39, (r"\u6025\u4fc3\u9e23\u7b1b",)),
    (40, (r"\u8b66\u62a5\u9e23\u7b1b",)),
    (32, (r"\u9e23\u7b1b1\u79d2(?:\u949f)?",)),
    (33, (r"\u9e23\u7b1b2\u79d2(?:\u949f)?",)),
    (34, (r"\u9e23\u7b1b3\u79d2(?:\u949f)?",)),
    (35, (r"\u9e23\u7b1b2\u58f0",)),
    (36, (r"\u9e23\u7b1b3\u58f0",)),
    (37, (r"\u9e23\u7b1b4\u58f0",)),
    (48, (r"(?:\u901a\u8fc7)?\u95e8\u6d1e1\u5de6\u4fa7",)),
    (52, (r"(?:\u901a\u8fc7)?\u95e8\u6d1e3\u53f3\u4fa7",)),
    (49, (r"\u901a\u8fc7\u95e8\u6d1e1(?:\u6d1e)?", r"\u95e8\u6d1e1\u6d1e")),
    (50, (r"\u901a\u8fc7\u95e8\u6d1e2", r"\u95e8\u6d1e2\u901a\u8fc7")),
    (51, (r"\u901a\u8fc7\u95e8\u6d1e3", r"\u95e8\u6d1e3\u901a\u8fc7")),
    (43, (r"\u95e8\u6d1e1\u53f3(?:\u4fa7)?(?:\u539f\u8def)?\u8fd4\u56de",)),
    (47, (r"\u95e8\u6d1e3\u5de6(?:\u4fa7)?(?:\u539f\u8def)?\u8fd4\u56de",)),
    (44, (r"\u95e8\u6d1e1(?:\u539f\u8def)?\u8fd4\u56de",)),
    (45, (r"\u95e8\u6d1e2(?:\u539f\u8def)?\u8fd4\u56de",)),
    (46, (r"\u95e8\u6d1e3(?:\u539f\u8def)?\u8fd4\u56de",)),
    (9, (r"\u86c7\u5f62\u524d\u8fdb(?:10\u7c73)?",)),
    (10, (r"\u86c7\u5f62\u540e\u9000(?:10\u7c73)?",)),
    (2, (r"(?:\u540e\u9000|\u5012\u8f66)(?:\u76f4\u884c)?(?:10\u7c73)?",)),
    (1, (r"(?:\u5411\u524d)?(?:\u76f4\u884c|\u524d\u8fdb|\u524d\u884c)(?:10\u7c73)?", r"\u5411\u524d10\u7c73")),
    (12, (r"\u9006\u65f6\u9488(?:\u8f6c)?(?:1\u5708)?",)),
    (11, (r"\u987a\u65f6\u9488(?:\u8f6c)?(?:1\u5708)?",)),
    (41, (r"\u5de6\u8f6c(?!\u5411?\u706f|\u5411)(?:90\u5ea6)?",)),
    (42, (r"\u53f3\u8f6c(?!\u5411?\u706f|\u5411)(?:90\u5ea6)?",)),
    (13, (r"(?:\u505c\u8fdb)?\u505c\u8f66\u533a1",)),
    (14, (r"(?:\u505c\u8fdb)?\u505c\u8f66\u533a2",)),
    (15, (r"(?:\u505c\u8fdb)?\u505c\u8f66\u533a3",)),
    (16, (r"3821",)),
    (17, (r"\u5f15\u64ce\u6388\u6743", r"\u5f15\u64ce", r"\u6388\u6743")),
    (18, (r"\u79d1\u76ee1",)),
    (19, (r"\u79d1\u76ee2",)),
    (20, (r"\u79d1\u76ee3",)),
    (21, (r"\u79d1\u76ee4",)),
    (22, (r"\u6d4b\u8bd5",)),
    (23, (r"\u505c\u6b62\u64ad\u653e", r"\u505c\u6b62")),
    (26, (r"\u6682\u505c\u64ad\u653e", r"\u6682\u505c")),
    (27, (r"\u7ee7\u7eed\u64ad\u653e", r"\u7ee7\u7eed")),
    (24, (r"\u64ad\u653e\u97f3\u4e50", r"\u64ad\u653e")),
    (25, (r"\u4f60\u597d",)),
    (28, (r"\u4e0a1\u9996",)),
    (29, (r"\u4e0b1\u9996",)),
]


def _match_commands_exact(text: str) -> List[int]:
    matches: List[int] = []
    for cmd, keys in COMMAND_KEYWORDS.items():
        for key in keys:
            if key in text:
                matches.append(cmd)
                break
    return matches


def _filter_ambiguous_commands(text: str, matches: List[int]) -> List[int]:
    if not matches:
        return matches

    light_hint = any(token in text for token in ("\u706f", "\u8f6c\u5411", "\u6253\u5f00", "\u5f00"))
    if light_hint:
        if 4 in matches and 41 in matches:
            matches = [cmd for cmd in matches if cmd != 41]
        if 5 in matches and 42 in matches:
            matches = [cmd for cmd in matches if cmd != 42]
        return matches
    if 41 in matches:
        matches = [cmd for cmd in matches if cmd != 4]
    if 42 in matches:
        matches = [cmd for cmd in matches if cmd != 5]
    return matches


def match_commands(text: str) -> tuple[List[int], str]:
    for candidate in text_match_candidates(text):
        matches = _filter_ambiguous_commands(candidate, _match_commands_exact(candidate))
        if matches:
            return matches, candidate
        if "\u9e23\u7b1b" in candidate and "\u79d2" in candidate:
            if any(token in candidate for token in ("\u4e09", "3")):
                return [34], candidate
            if any(token in candidate for token in ("\u4e8c", "\u4e24", "2")):
                return [33], candidate
            return [32], candidate
    return [], text


def _order_command_hits(found: List[tuple[int, int, int, int]]) -> List[int]:
    if not found:
        return []

    found.sort()
    ordered: List[int] = []
    last_end = -1
    for start, _, end, cmd in found:
        if start < last_end:
            continue
        ordered.append(cmd)
        last_end = end
    return ordered


def _match_command_sequence_candidate(candidate: str) -> List[int]:
    found: List[tuple[int, int, int, int]] = []
    for cmd, keys in COMMAND_KEYWORDS.items():
        for key in keys:
            start = candidate.find(key)
            while start >= 0:
                end = start + len(key)
                found.append((start, -len(key), end, cmd))
                start = candidate.find(key, start + 1)

    return _order_command_hits(found)


def _match_command_rule_candidate(candidate: str) -> tuple[List[int], str]:
    normalized = _canonicalize_rule_text(candidate)
    found: List[tuple[int, int, int, int]] = []
    for cmd, patterns in COMMAND_RULE_PATTERNS:
        for pattern in patterns:
            for match in re.finditer(pattern, normalized):
                start = match.start()
                end = match.end()
                found.append((start, -(end - start), end, cmd))

    return _order_command_hits(found), normalized


def match_command_sequence(text: str) -> tuple[List[int], str]:
    best_cmds: List[int] = []
    best_candidate = text
    for candidate in text_match_candidates(text):
        ordered = _match_command_sequence_candidate(candidate)
        if len(ordered) > len(best_cmds):
            best_cmds = ordered
            best_candidate = candidate
        rule_ordered, rule_candidate = _match_command_rule_candidate(candidate)
        if len(rule_ordered) >= len(best_cmds):
            best_cmds = rule_ordered
            best_candidate = rule_candidate

    return best_cmds, best_candidate


def match_commands_by_mode(text: str, command_mode: str) -> tuple[List[int], str]:
    if command_mode == COMMAND_MODE_SEQUENCE:
        return match_command_sequence(text)
    return match_commands(text)


def send_commands(sock: socket.socket, cmds: List[int]) -> None:
    if not cmds:
        return

    low_seq: List[int] = []
    for cmd in cmds:
        if 1 <= cmd <= 15:
            low_seq.append(cmd)
            continue

        if low_seq:
            sock.sendall(bytes(low_seq + [0]))
            low_seq = []

        if 16 <= cmd <= 255:
            sock.sendall(bytes([cmd]))

    if low_seq:
        sock.sendall(bytes(low_seq + [0]))

def send_text_to_board(sock: socket.socket, text: str) -> None:
    payload = text.encode("gbk", errors="replace")
    if not payload:
        logging.info("send_text_to_board skipped: empty payload")
        return
    # Use printable ASCII markers only (all > 0x10), so board-side byte parser
    # forwards them through UART path without entering command framing branch.
    logging.info("send_text_to_board len=%d encoding=gbk text=%s", len(payload), text)
    try:
        sock.sendall(b"[TXT]")
        sock.sendall(payload)
        sock.sendall(b"[END]")
        logging.info("send_text_to_board done")
    except OSError as e:
        logging.exception("send_text_to_board failed: %s", e)


def resample_pcm16_mono(raw: bytes, src_rate: float, dst_rate: int = 16000) -> bytes:
    src_count = len(raw) // 2
    if src_count < 2 or src_rate <= 0:
        return raw[: src_count * 2]

    dst_count = max(1, int(round(src_count * dst_rate / src_rate)))
    step = src_rate / float(dst_rate)
    out = bytearray(dst_count * 2)

    def sample_at(index: int) -> int:
        off = index * 2
        return int.from_bytes(raw[off : off + 2], "little", signed=True)

    for out_index in range(dst_count):
        pos = out_index * step
        i = int(pos)
        if i >= src_count - 1:
            value = sample_at(src_count - 1)
        else:
            frac = pos - i
            s0 = sample_at(i)
            s1 = sample_at(i + 1)
            value = int(round(s0 + (s1 - s0) * frac))
        out[out_index * 2 : out_index * 2 + 2] = int(value).to_bytes(2, "little", signed=True)

    return bytes(out)


def pcm16_stats(raw: bytes) -> tuple[int, int, float, float]:
    count = len(raw) // 2
    if count <= 0:
        return 0, 0, 0.0, 0.0

    peak = 0
    sum_sq = 0
    zero_cross = 0
    prev = 0
    for i in range(count):
        off = i * 2
        value = int.from_bytes(raw[off : off + 2], "little", signed=True)
        av = abs(value)
        if av > peak:
            peak = av
        sum_sq += value * value
        if i > 0 and ((value < 0 <= prev) or (value >= 0 > prev)):
            zero_cross += 1
        prev = value
    rms = (sum_sq / count) ** 0.5
    zcr = zero_cross / count
    return count, peak, rms, zcr


def pcm16_diagnostics(raw: bytes, sample_rate: int = 16000) -> dict:
    count = len(raw) // 2
    if count <= 0:
        return {
            "avg_abs": 0.0,
            "mean": 0.0,
            "min": 0,
            "max": 0,
            "clip": 0,
            "zero": 0,
            "abs_p50": 0,
            "abs_p90": 0,
            "abs_p99": 0,
            "win_min_rms": 0.0,
            "win_max_rms": 0.0,
            "dominant_hz": 0.0,
            "dominant_ratio": 0.0,
        }

    values: List[int] = []
    abs_values: List[int] = []
    total = 0
    sum_abs = 0
    clip = 0
    zero = 0
    mn = 32767
    mx = -32768
    for i in range(count):
        off = i * 2
        value = int.from_bytes(raw[off : off + 2], "little", signed=True)
        av = abs(value)
        values.append(value)
        abs_values.append(av)
        total += value
        sum_abs += av
        if value == 0:
            zero += 1
        if value <= -32768 or value >= 32767:
            clip += 1
        if value < mn:
            mn = value
        if value > mx:
            mx = value

    abs_values.sort()

    def percentile(pct: float) -> int:
        if not abs_values:
            return 0
        idx = int(round((len(abs_values) - 1) * pct))
        return abs_values[idx]

    win = max(1, sample_rate // 10)
    win_rms_values: List[float] = []
    for start in range(0, count, win):
        part = values[start : start + win]
        if not part:
            continue
        ss = sum(v * v for v in part)
        win_rms_values.append((ss / len(part)) ** 0.5)

    dominant_hz = 0.0
    dominant_ratio = 0.0
    n = min(count, 4096)
    if n >= 512:
        start = max(0, (count - n) // 2)
        segment = values[start : start + n]
        segment_mean = sum(segment) / n
        best_mag = 0.0
        total_mag = 0.0
        best_k = 0
        max_k = min(n // 2, int(2000.0 * n / sample_rate))
        min_k = max(1, int(80.0 * n / sample_rate))
        for k in range(min_k, max_k + 1):
            re = 0.0
            im = 0.0
            for j, sample in enumerate(segment):
                angle = 2.0 * math.pi * k * j / n
                centered = sample - segment_mean
                re += centered * math.cos(angle)
                im -= centered * math.sin(angle)
            mag = (re * re + im * im) ** 0.5
            total_mag += mag
            if mag > best_mag:
                best_mag = mag
                best_k = k
        if best_k > 0 and total_mag > 0.0:
            dominant_hz = best_k * sample_rate / n
            dominant_ratio = best_mag / total_mag

    return {
        "avg_abs": sum_abs / count,
        "mean": total / count,
        "min": mn,
        "max": mx,
        "clip": clip,
        "zero": zero,
        "abs_p50": percentile(0.50),
        "abs_p90": percentile(0.90),
        "abs_p99": percentile(0.99),
        "win_min_rms": min(win_rms_values) if win_rms_values else 0.0,
        "win_max_rms": max(win_rms_values) if win_rms_values else 0.0,
        "dominant_hz": dominant_hz,
        "dominant_ratio": dominant_ratio,
    }


def log_audio_diagnostics(audio: bytes) -> None:
    diag = pcm16_diagnostics(audio, 16000)
    logging.info(
        "audio diag avg_abs=%.1f mean=%.1f min=%d max=%d clip=%d zero=%d abs_p50/p90/p99=%d/%d/%d win_rms=%.1f..%.1f dom=%.1fHz ratio=%.3f",
        diag["avg_abs"],
        diag["mean"],
        diag["min"],
        diag["max"],
        diag["clip"],
        diag["zero"],
        diag["abs_p50"],
        diag["abs_p90"],
        diag["abs_p99"],
        diag["win_min_rms"],
        diag["win_max_rms"],
        diag["dominant_hz"],
        diag["dominant_ratio"],
    )
    if diag["clip"] > 0:
        logging.warning("audio has clipped samples; reduce analog/digital gain before ASR testing")
    if diag["dominant_ratio"] > 0.08 and diag["dominant_hz"] > 80.0:
        logging.warning("audio looks tonal/non-speech dominant_hz=%.1f ratio=%.3f; verify active input route and microphone", diag["dominant_hz"], diag["dominant_ratio"])
    if diag["win_max_rms"] < 250.0:
        logging.warning("audio level is very low; speak close to the active microphone or increase gain")


def write_wav(path: str, raw: bytes, sample_rate: int) -> None:
    with wave.open(path, "wb") as fp:
        fp.setnchannels(1)
        fp.setsampwidth(2)
        fp.setframerate(sample_rate)
        fp.writeframes(raw)


def finish_text(
    sock: socket.socket,
    session: XfyunSession,
    send_back_cmd: bool,
    command_mode: str,
    text_acc: List[str],
    flag: int,
) -> None:
    final_text = "".join(text_acc).strip()
    if final_text:
        logging.info("xfyun final: %s", final_text)
        if send_back_cmd:
            cmds, matched_text = match_commands_by_mode(final_text, command_mode)
            if cmds:
                if matched_text != final_text:
                    logging.info("command match normalized text: %s", matched_text)
                logging.info(
                    "send commands mode=%s: %s %s",
                    command_mode,
                    cmds,
                    [COMMAND_NAMES.get(cmd, "") for cmd in cmds],
                )
                send_commands(sock, cmds)
            else:
                logging.info("send commands: none")
        send_text_to_board(sock, final_text)
    else:
        logging.warning(
            "xfyun final empty (flag=%d, continue_count=%d, parts=%d, recv=%d)",
            flag,
            g_continue_packet_count,
            len(text_acc),
            session.recv_count,
        )
        if session.last_json:
            logging.warning("xfyun last json: %s", json.dumps(session.last_json, ensure_ascii=False)[:2000])
        elif session.last_raw:
            logging.warning("xfyun last raw: %s", session.last_raw[:2000])
    session.close()


def recognize_buffered_audio(
    sock: socket.socket,
    cred: XfCredential,
    raw: bytes,
    observed_seconds: float,
    send_back_cmd: bool,
    command_mode: str,
    xfyun_send_timeout_s: float,
    xfyun_final_timeout_s: float,
    xfyun_chunk_ms: int,
    text_acc: List[str],
) -> None:
    src_samples = len(raw) // 2
    if src_samples <= 0:
        logging.warning("buffered audio empty")
        return

    if observed_seconds <= 0.05 or observed_seconds > MAX_RECORD_SECONDS:
        fallback_seconds = 0.0
        if g_audio_meta_rate > 0 and src_samples > 0:
            fallback_seconds = src_samples / float(g_audio_meta_rate)
        if fallback_seconds <= 0.05 or fallback_seconds > MAX_RECORD_SECONDS:
            fallback_seconds = g_audio_last_ts - g_audio_start_ts
        logging.warning(
            "observed duration abnormal %.3fs, fallback %.3fs",
            observed_seconds,
            fallback_seconds,
        )
        observed_seconds = fallback_seconds

    src_rate = src_samples / observed_seconds if observed_seconds > 0.05 else 16000.0
    if src_rate < 100.0 or src_rate > 24000.0:
        logging.warning("estimated source rate abnormal %.1f, use 16000", src_rate)
        src_rate = 16000.0

    audio = resample_pcm16_mono(raw, src_rate, 16000)
    raw_count, raw_peak, raw_rms, raw_zcr = pcm16_stats(raw)
    out_count, out_peak, out_rms, out_zcr = pcm16_stats(audio)
    logging.info(
        "buffered audio samples=%d observed=%.3fs src_rate=%.1f peak=%d rms=%.1f zcr=%.3f -> samples=%d peak=%d rms=%.1f zcr=%.3f rate=16000",
        raw_count,
        observed_seconds,
        src_rate,
        raw_peak,
        raw_rms,
        raw_zcr,
        out_count,
        out_peak,
        out_rms,
        out_zcr,
    )
    log_audio_diagnostics(audio)
    if g_dump_fp is not None:
        base = os.path.splitext(g_dump_fp.name)[0]
        try:
            write_wav(base + "_raw.wav", raw, max(1000, int(round(src_rate))))
            write_wav(base + "_16k.wav", audio, 16000)
            logging.info("wav dump written: %s_raw.wav / %s_16k.wav", base, base)
        except OSError as e:
            logging.warning("wav dump write failed: %s", e)

    session = XfyunSession(cred, send_timeout_s=xfyun_send_timeout_s)
    session.connect()
    text_acc.clear()
    chunk_size = max(640, int(16000 * 2 * max(20, xfyun_chunk_ms) / 1000))
    chunks = [audio[i : i + chunk_size] for i in range(0, len(audio), chunk_size)]
    if not chunks:
        chunks = [b""]
    logging.info(
        "xfyun send chunks=%d chunk_size=%d chunk_ms=%d audio_bytes=%d send_timeout=%.1f final_timeout=%.1f",
        len(chunks),
        chunk_size,
        xfyun_chunk_ms,
        len(audio),
        xfyun_send_timeout_s,
        xfyun_final_timeout_s,
    )

    for index, chunk in enumerate(chunks):
        if index == 0:
            status = STATUS_FIRST_FRAME
        elif index == len(chunks) - 1:
            status = STATUS_LAST_FRAME
        else:
            status = STATUS_CONTINUE_FRAME
        session.send_audio(status, chunk)
        if status != STATUS_LAST_FRAME:
            time.sleep(len(chunk) / 2 / 16000.0)
        part, is_last = session.recv_text(0.25 if status != STATUS_LAST_FRAME else xfyun_final_timeout_s)
        if g_debug_xfyun and (part or is_last or status == STATUS_LAST_FRAME):
            logging.info("xfyun recv after chunk=%d/%d status=%d part_len=%d is_last=%s recv_count=%d", index + 1, len(chunks), status, len(part), is_last, session.recv_count)
        if part:
            text_acc.append(part)
            logging.info("xfyun partial: %s", part)
        if is_last:
            finish_text(sock, session, send_back_cmd, command_mode, text_acc, status)
            return

    if len(chunks) == 1:
        session.send_audio(STATUS_LAST_FRAME, b"")
        part, _ = session.recv_text(xfyun_final_timeout_s)
        if part:
            text_acc.append(part)
            logging.info("xfyun partial: %s", part)
    finish_text(sock, session, send_back_cmd, command_mode, text_acc, STATUS_LAST_FRAME)


def feed_packet(
    sock: socket.socket,
    session: XfyunSession | None,
    cred: XfCredential,
    flag: int,
    payload: bytes,
    send_back_cmd: bool,
    command_mode: str,
    xfyun_send_timeout_s: float,
    xfyun_final_timeout_s: float,
    xfyun_chunk_ms: int,
    text_acc: List[str],
) -> XfyunSession | None:
    global g_continue_packet_count, g_audio_parts, g_audio_start_ts, g_audio_last_ts
    global g_audio_meta_rate, g_audio_meta_samples, g_audio_meta_elapsed_ms

    is_meta_payload = payload[:4] == b"GMD1"

    if flag == 3:
        logging.info("packet flag=3 heartbeat")
        return session

    if g_dump_fp is not None and flag in (STATUS_CONTINUE_FRAME, STATUS_LAST_FRAME) and not is_meta_payload:
        try:
            g_dump_fp.write(payload)
        except OSError as e:
            logging.warning("pcm dump write failed: %s", e)

    if flag == STATUS_FIRST_FRAME:
        g_continue_packet_count = 0
        g_audio_parts = []
        g_audio_start_ts = time.time()
        g_audio_last_ts = g_audio_start_ts
        g_audio_meta_rate = 0
        g_audio_meta_samples = 0
        g_audio_meta_elapsed_ms = 0
        if is_meta_payload:
            g_audio_meta_rate = int.from_bytes(payload[4:8], "little", signed=False)
            g_audio_meta_samples = int.from_bytes(payload[8:12], "little", signed=False)
            g_audio_meta_elapsed_ms = int.from_bytes(payload[12:16], "little", signed=False)
            logging.info(
                "packet metadata: sample_rate=%d samples=%d elapsed_ms=%d",
                g_audio_meta_rate,
                g_audio_meta_samples,
                g_audio_meta_elapsed_ms,
            )
        logging.info("packet flag=0 first")
        if session is not None:
            session.close()
        text_acc.clear()
        return session

    if flag in (STATUS_CONTINUE_FRAME, STATUS_LAST_FRAME):
        g_audio_last_ts = time.time()
        if flag == STATUS_CONTINUE_FRAME:
            g_audio_parts.append(payload)
            g_continue_packet_count += 1
            if (g_continue_packet_count % 20) == 1:
                logging.info("packet flag=1 continue count=%d", g_continue_packet_count)
        elif flag == STATUS_LAST_FRAME:
            if is_meta_payload:
                g_audio_meta_rate = int.from_bytes(payload[4:8], "little", signed=False)
                g_audio_meta_samples = int.from_bytes(payload[8:12], "little", signed=False)
                g_audio_meta_elapsed_ms = int.from_bytes(payload[12:16], "little", signed=False)
                logging.info(
                    "packet final metadata: sample_rate=%d samples=%d elapsed_ms=%d",
                    g_audio_meta_rate,
                    g_audio_meta_samples,
                    g_audio_meta_elapsed_ms,
                )
            else:
                g_audio_parts.append(payload)
            raw_audio = b"".join(g_audio_parts)
            if g_audio_meta_samples > 0:
                raw_audio = raw_audio[: g_audio_meta_samples * 2]
            logging.info("packet flag=2 last, continue_count=%d", g_continue_packet_count)
            recognize_buffered_audio(
                sock,
                cred,
                raw_audio,
                (g_audio_meta_elapsed_ms / 1000.0)
                if g_audio_meta_elapsed_ms > 0
                else (g_audio_meta_samples / g_audio_meta_rate)
                if g_audio_meta_rate > 0 and g_audio_meta_samples > 0
                else (g_audio_last_ts - g_audio_start_ts),
                send_back_cmd,
                command_mode,
                xfyun_send_timeout_s,
                xfyun_final_timeout_s,
                xfyun_chunk_ms,
                text_acc,
            )
            return None

    else:
        logging.warning("packet unknown flag=%d", flag)

    return session


def parse_stream(
    sock: socket.socket,
    session: XfyunSession | None,
    cred: XfCredential,
    buffer: bytearray,
    send_back_cmd: bool,
    command_mode: str,
    xfyun_send_timeout_s: float,
    xfyun_final_timeout_s: float,
    xfyun_chunk_ms: int,
    text_acc: List[str],
) -> XfyunSession | None:
    while True:
        pos = buffer.find(MAGIC_HEADER)
        if pos < 0:
            if len(buffer) > 3:
                del buffer[:-3]
            return session
        if pos > 0:
            del buffer[:pos]
        if len(buffer) < FRAME_SIZE:
            return session

        packet = bytes(buffer[:FRAME_SIZE])
        del buffer[:FRAME_SIZE]
        flag = packet[4]
        payload = packet[PAYLOAD_OFFSET:]
        session = feed_packet(
            sock,
            session,
            cred,
            flag,
            payload,
            send_back_cmd,
            command_mode,
            xfyun_send_timeout_s,
            xfyun_final_timeout_s,
            xfyun_chunk_ms,
            text_acc,
        )


def run_server(
    host: str,
    port: int,
    cred: XfCredential,
    send_back_cmd: bool,
    command_mode: str,
    dump_dir: str,
    debug_xfyun: bool,
    xfyun_send_timeout_s: float,
    xfyun_final_timeout_s: float,
    xfyun_chunk_ms: int,
) -> None:
    global g_dump_fp
    global g_debug_xfyun
    g_debug_xfyun = debug_xfyun
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(1)
    logging.info(
        "server listen on %s:%d command_mode=%s xfyun_send_timeout=%.1f final_timeout=%.1f chunk_ms=%d",
        host,
        port,
        command_mode,
        xfyun_send_timeout_s,
        xfyun_final_timeout_s,
        xfyun_chunk_ms,
    )

    while True:
        client, addr = server.accept()
        logging.info("client connected: %s:%d", addr[0], addr[1])
        if dump_dir:
            try:
                os.makedirs(dump_dir, exist_ok=True)
                dump_path = os.path.join(
                    dump_dir,
                    time.strftime("guimai_%Y%m%d_%H%M%S.pcm", time.localtime()),
                )
                g_dump_fp = open(dump_path, "wb")
                logging.info("pcm dump enabled: %s", dump_path)
            except OSError as e:
                g_dump_fp = None
                logging.warning("pcm dump open failed: %s", e)
        session: XfyunSession | None = None
        buffer = bytearray()
        text_acc: List[str] = []
        recv_total = 0
        try:
            while True:
                data = client.recv(2048)
                if not data:
                    break
                recv_total += len(data)
                if recv_total == len(data) or (recv_total % 8192) < len(data):
                    logging.info(
                        "tcp recv total=%d len=%d first=%s buffer_before=%d",
                        recv_total,
                        len(data),
                        data[:16].hex(" "),
                        len(buffer),
                    )
                buffer.extend(data)
                session = parse_stream(
                    client,
                    session,
                    cred,
                    buffer,
                    send_back_cmd,
                    command_mode,
                    xfyun_send_timeout_s,
                    xfyun_final_timeout_s,
                    xfyun_chunk_ms,
                    text_acc,
                )
        except (ConnectionResetError, BrokenPipeError):
            logging.warning("client disconnected unexpectedly")
        except Exception:
            logging.exception("client loop failed")
        finally:
            if session is not None:
                session.close()
            if g_dump_fp is not None:
                try:
                    g_dump_fp.close()
                finally:
                    g_dump_fp = None
            client.close()
            logging.info("client closed")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Guimai TCP -> XFYun IAT relay")
    p.add_argument("--host", default="0.0.0.0", help="TCP bind host")
    p.add_argument("--port", type=int, default=8080, help="TCP bind port")
    p.add_argument("--appid", required=True, help="XFYun APPID")
    p.add_argument("--apikey", required=True, help="XFYun APIKey")
    p.add_argument("--apisecret", required=True, help="XFYun APISecret")
    p.add_argument(
        "--send-back-cmd",
        dest="send_back_cmd",
        action="store_true",
        default=True,
        help="map recognized text to command bytes and send back to board (default: enabled)",
    )
    p.add_argument(
        "--no-send-back-cmd",
        dest="send_back_cmd",
        action="store_false",
        help="do not send recognized command bytes back to board",
    )
    p.add_argument(
        "--dump-dir",
        default="",
        help="optional directory to dump incoming raw PCM stream",
    )
    p.add_argument(
        "--command-mode",
        choices=(COMMAND_MODE_LEGACY, COMMAND_MODE_SEQUENCE),
        default=COMMAND_MODE_SEQUENCE,
        help="command matching mode after ASR final text (default: sequence)",
    )
    p.add_argument(
        "--xfyun-send-timeout",
        type=float,
        default=DEFAULT_XFYUN_SEND_TIMEOUT_S,
        help="seconds allowed for each WebSocket audio frame send (default: 10)",
    )
    p.add_argument(
        "--xfyun-final-timeout",
        type=float,
        default=DEFAULT_XFYUN_FINAL_TIMEOUT_S,
        help="seconds to wait for XFYun final result after last audio frame (default: 20)",
    )
    p.add_argument(
        "--xfyun-chunk-ms",
        type=int,
        default=DEFAULT_XFYUN_CHUNK_MS,
        help="audio duration per XFYun frame in milliseconds (default: 100)",
    )
    p.add_argument(
        "--debug-xfyun",
        action="store_true",
        help="log raw XFYun response snippets for protocol diagnosis",
    )
    return p.parse_args()


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(message)s",
    )
    args = parse_args()
    cred = XfCredential(args.appid, args.apikey, args.apisecret)
    run_server(
        args.host,
        args.port,
        cred,
        args.send_back_cmd,
        args.command_mode,
        args.dump_dir,
        args.debug_xfyun,
        args.xfyun_send_timeout,
        args.xfyun_final_timeout,
        args.xfyun_chunk_ms,
    )
