import os
from time import localtime, strftime
import jieba


def get_current_time():
    return strftime("%H:%M", localtime())


def clear_console():
    os.system("cls" if os.name == "nt" else "clear")


def get_protocol_by_name(name, txt_file='PhraseReplacement/protocol.txt'):
    with open(txt_file, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith(name + ','):
                return line.split(',', 1)[1]
    return None


class Display:
    def __init__(self):
        self.last_sentence = None
        self.currentText = []
        self.words = ""
        self.protocol = ""
        self.last_sentence_protocol = ""
        self.number = 1
        
        # 加载自定义词典
        jieba.load_userdict('PhraseReplacement/dict.txt')
        
        with open('PhraseReplacement/dict.txt', 'r', encoding='utf-8') as f:
            self.all_words = set(line.strip() for line in f if line.strip())
        
        self.valid_words_list = list(self.all_words)
        
    def update_text(self, text):
        self.words = text
        s_jieba = jieba.lcut(text)        
        self.currentText = [word for word in s_jieba if word in self.all_words]

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
                "is_endpoint": is_endpoint
            }
        
        self.currentText = []
        self.words = ""
        
    def display(self, incremental=False):
        """
        显示内容，支持增量显示模式
        :param incremental: 是否增量显示（不刷新整个屏幕）
        """
        if not incremental:
            clear_console()
            print("=== ROS2 语音识别节点 ===")
            print("Time:", get_current_time())
            print("-" * 30)

        # 显示历史记录（仅在非增量模式或首次显示时）
        if not incremental or not hasattr(self, '_displayed_history'):
            if self.last_sentence:
                if self.last_sentence['protocol']:
                    print(f"[{self.last_sentence['time']}] {self.number}. 识别结果：{self.last_sentence['text']}")

                print("-" * 30)
                self.last_sentence = None  # 清除最后一个句子，避免重复显示
                self.number += 1

            self._displayed_history = True  # 标记历史已显示

        # 显示当前识别内容
        if self.words.strip():
            if len(self.words) > 10:
                self.words = self.words[-10:]  # 限制显示长度
            # 在增量模式下，使用回车符覆盖当前行
            if incremental:
                print(f"\r{get_current_time()} Recognizing: {self.words}", end="", flush=True)
            else:
                print(f"{get_current_time()} Recognizing: {self.words}")
        elif incremental:
            # 没有识别内容时清除当前行
            print("", end="", flush=True)