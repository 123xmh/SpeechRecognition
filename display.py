# Copyright (c)  2025  Xiaomi Corporation
import os
from time import localtime, strftime
import jieba
import time


def get_current_time():
    return strftime("%H:%M", localtime())


def clear_console():
    os.system("cls" if os.name == "nt" else "clear")


def get_protocol_by_name(name, txt_file='protocol.txt'):
    with open(txt_file, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith(name + ','):
                return line.split(',', 1)[1]
    return None


class Display:
    def __init__(self, udp_comm=None):  # 添加udp_comm参数
        # self.sentences = []
        self.last_sentence = None
        self.udp_comm = udp_comm  # 存储UDP通信对象
        self.currentText = []
        self.words = ""
        self.protocol = ""
        self.last_sentence_protocol = ""
        self.protocol_sent = False
        self.last_send_time = 0  # 记录上次发送时间
        self.number = 1  # 记录当前显示的序号
        self.number = 200  # 限制语音识别的文字数量
        
        # 加载自定义词典
        jieba.load_userdict('PhraseReplacement/dict.txt')
        
        with open('PhraseReplacement/dict.txt', 'r', encoding='utf-8') as f:
            self.all_words = set(line.strip() for line in f if line.strip())
        
        self.valid_words_list = list(self.all_words)
        
    def set_udp_comm(self, udp_comm):
        """设置UDP通信对象"""
        self.udp_comm = udp_comm

    def update_text(self, text):
        self.words = text
        s_jieba = jieba.lcut(text)        
        self.currentText = [word for word in s_jieba if word in self.all_words]
        self.protocol_sent = False

    def finalize_current_sentence(self, is_endpoint=True):
        if self.currentText:
            protocols = []
            for word in self.currentText:
                protocol = get_protocol_by_name(word)
                if protocol:
                    protocols.append(protocol)
            
            sentence_protocol = "|".join(protocols) if protocols else ""
            
            # 存储端点标志
            self.last_sentence = {
                "time": get_current_time(),
                "text": "，".join(self.currentText),
                "protocol": sentence_protocol,
                "status": "待发送",
                "is_endpoint": is_endpoint  # 存储端点标志
            }
            
            # self.sentences.append(self.last_sentence)
            self.protocol_sent = False
            
            # 当历史记录变化时，尝试发送数据
            self.send_last_sentence()
        
        self.currentText = []
        self.words = ""
        
    def send_last_sentence(self):
        """发送最后一个句子（确保只发送一次）"""
        if not self.udp_comm or not self.last_sentence or self.protocol_sent:
            return
            
        # 确保不重复发送
        current_time = time.time()
        if current_time - self.last_send_time < 0.1:  # 100ms内不重复发送
            return
            
        self.last_send_time = current_time
        
        # 创建符合接收端要求的JSON数据结构
        data = {
            'timestamp': current_time,  # 使用time.time()返回的浮点数
            'text': self.last_sentence["text"],
            'protocol': self.last_sentence["protocol"],
            'is_endpoint': True  # 由于这是完整句子的发送，所以总是True
        }
        
        if self.udp_comm.send(data):
            self.protocol_sent = True
            self.last_sentence["status"] = "已发送"
            
            # 等待确认
            timeout = 1.0
            start_time = time.time()
            while not self.udp_comm.ack_received and (time.time() - start_time) < timeout:
                time.sleep(0.01)
            
            if self.udp_comm.ack_received:
                self.last_sentence["status"] = "已发送并确认"
            else:
                self.last_sentence["status"] = "已发送但未确认"
            
            # 清除确认标志
            self.udp_comm.ack_received = False
        
    def add_send_status(self, status):
        """添加发送状态信息"""
        self.send_status = status

    def update_send_status(self, status):
        """更新最后一个句子的发送状态"""
        if self.last_sentence:
            self.last_sentence["status"] = status

    def display(self, incremental=False):
        """
        显示内容，支持增量显示模式
        :param incremental: 是否增量显示（不刷新整个屏幕）
        """
        if not incremental:
            clear_console()
            print("=== Speech Recognition with Next-gen Kaldi ===")
            print("Time:", get_current_time())
            print("-" * 30)

        # 显示历史记录（仅在非增量模式或首次显示时）
        if not incremental or not hasattr(self, '_displayed_history'):
            if self.last_sentence:
                if self.last_sentence['protocol']:
                    print(f"[{self.last_sentence['time']}] {self.number}. 识别结果：{self.last_sentence['text']}    发送状态: {self.last_sentence['status']}")

                print("-" * 30)
                self.last_sentence = None  # 清除最后一个句子，避免重复显示

                self.number += 1

            self._displayed_history = True  # 标记历史已显示

        # 显示当前识别内容
        if self.words.strip():
            if len(self.words) > self.number:
                self.words = self.words[-self.number:]  # 限制显示长度
            # 在增量模式下，使用回车符覆盖当前行
            if incremental:
                print(f"\r{get_current_time()} Recognizing: {self.words}", end="", flush=True)
            else:
                print(f"{get_current_time()} Recognizing: {self.words}")
        elif incremental:
            # 没有识别内容时清除当前行
            print("", end="", flush=True)