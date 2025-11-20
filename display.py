# Copyright (c)  2025  Xiaomi Corporation
import os
from time import localtime, strftime
import jieba


def get_current_time():
    return strftime("%Y-%m-%d %H:%M:%S", localtime())


def clear_console():
    os.system("cls" if os.name == "nt" else "clear")


class Display:
    def __init__(self):
        self.sentences = []
        self.currentText = ""
        self.recording_status = "开始"

    def update_recording_status(self, status):
        self.recording_status = status

    def update_text(self, text):
        self.currentText = text

    def finalize_current_sentence(self):
        if self.currentText.strip():
            self.sentences.append((get_current_time(), self.currentText))
        self.currentText = ""

    def display(self):
        clear_console()
        print("=== Speech Recognition with Next-gen Kaldi ===")
        print("Time:", get_current_time())
        # print(f"录音状态: {self.recording_status} (按回车键切换)")    # 录音状态
        print("-" * 40)

        # display history sentences
        if self.sentences:
            for i, (when, text) in enumerate(self.sentences):
                print(f"[{when}] {i + 1}. {text}")
            print("-" * 40)

        if self.currentText.strip():
            print("Recognizing:", self.currentText)