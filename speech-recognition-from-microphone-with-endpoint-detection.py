#!/usr/bin/env python3

# Real-time speech recognition from a microphone with sherpa-onnx Python API
# with endpoint detection and keyboard control.

import argparse
import sys
from pathlib import Path
from display import Display
import threading
import time
import select
import termios
import tty
import os

try:
    import sounddevice as sd
except ImportError:
    print("Please install sounddevice first. You can use")
    print()
    print("  pip install sounddevice")
    print()
    print("to install it")
    sys.exit(-1)

import sherpa_onnx


def assert_file_exists(filename: str):
    assert Path(filename).is_file(), (
        f"{filename} does not exist!\n"
        "Please refer to "
        "https://k2-fsa.github.io/sherpa/onnx/pretrained_models/index.html to download it"
    )


def get_args():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    parser.add_argument(
        "--tokens",
        type=str,
        required=True,
        help="Path to tokens.txt",
    )

    parser.add_argument(
        "--encoder",
        type=str,
        required=True,
        help="Path to the encoder model",
    )

    parser.add_argument(
        "--decoder",
        type=str,
        required=True,
        help="Path to the decoder model",
    )

    parser.add_argument(
        "--joiner",
        type=str,
        required=True,
        help="Path to the joiner model",
    )

    parser.add_argument(
        "--decoding-method",
        type=str,
        default="greedy_search",
        help="Valid values are greedy_search and modified_beam_search",
    )

    parser.add_argument(
        "--provider",
        type=str,
        default="cpu",
        help="Valid values: cpu, cuda, coreml",
    )

    parser.add_argument(
        "--hotwords-file",
        type=str,
        default="",
        help="""
        The file containing hotwords, one words/phrases per line, and for each
        phrase the bpe/cjkchar are separated by a space. For example:

        ▁▁HE LL O ▁▁WORLD
        你 好 世 界
        """,
    )

    parser.add_argument(
        "--hotwords-score",
        type=float,
        default=1.5,
        help="""
        The hotword score of each token for biasing word/phrase. Used only if
        --hotwords-file is given.
        """,
    )

    parser.add_argument(
        "--blank-penalty",
        type=float,
        default=0.0,
        help="""
        The penalty applied on blank symbol during decoding.
        Note: It is a positive value that would be applied to logits like
        this `logits[:, 0] -= blank_penalty` (suppose logits.shape is
        [batch_size, vocab] and blank id is 0).
        """,
    )

    parser.add_argument(
        "--hr-dict-dir",
        type=str,
        default="",
        help="If not empty, it is the jieba dict directory for homophone replacer",
    )

    parser.add_argument(
        "--hr-lexicon",
        type=str,
        default="",
        help="If not empty, it is the lexicon.txt for homophone replacer",
    )

    parser.add_argument(
        "--hr-rule-fsts",
        type=str,
        default="",
        help="If not empty, it is the replace.fst for homophone replacer",
    )

    return parser.parse_args()


def create_recognizer(args):
    assert_file_exists(args.encoder)
    assert_file_exists(args.decoder)
    assert_file_exists(args.joiner)
    assert_file_exists(args.tokens)
    
    recognizer = sherpa_onnx.OnlineRecognizer.from_transducer(
        tokens=args.tokens,
        encoder=args.encoder,
        decoder=args.decoder,
        joiner=args.joiner,
        num_threads=1,
        sample_rate=16000,
        feature_dim=80,
        enable_endpoint_detection=True,
        rule1_min_trailing_silence=2.4,
        rule2_min_trailing_silence=1.2,
        rule3_min_utterance_length=300,
        decoding_method=args.decoding_method,
        provider=args.provider,
        hotwords_file=args.hotwords_file,
        hotwords_score=args.hotwords_score,
        blank_penalty=args.blank_penalty,
        hr_dict_dir=args.hr_dict_dir,
        hr_rule_fsts=args.hr_rule_fsts,
        hr_lexicon=args.hr_lexicon,
    )
    return recognizer


class RecordingController:
    def __init__(self):
        # 修改：默认状态为暂停
        self.recording = False
        self.lock = threading.Lock()
        self.last_status_change = time.time()
        
    def toggle_recording(self):
        with self.lock:
            current_time = time.time()
            # 防抖处理，避免快速连续按键
            if current_time - self.last_status_change > 0.5:
                self.recording = not self.recording
                self.last_status_change = current_time
                return True
            return False
    
    def is_recording(self):
        with self.lock:
            return self.recording


def check_stdin_command():
    """检查标准输入是否有控制命令（来自Qt界面）"""
    try:
        # 非阻塞检查stdin
        if select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline().strip()
            return line
    except:
        pass
    return None


def main():
    args = get_args()

    devices = sd.query_devices()
    if len(devices) == 0:
        print("No microphone devices found")
        sys.exit(0)

    print(devices)
    default_input_device_idx = sd.default.device[0]
    print(f'Use default device: {devices[default_input_device_idx]["name"]}')

    recognizer = create_recognizer(args)
    controller = RecordingController()
    
    # 保存终端设置
    old_settings = None
    # 只有在stdin是终端时才设置终端模式
    if sys.stdin.isatty():
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            # 设置终端为无缓冲模式
            tty.setcbreak(sys.stdin.fileno())
        except:
            old_settings = None

    try:
        print("程序已启动，当前状态: 暂停")
        print("按回车键开始录音，再次按回车键暂停")

        sample_rate = 48000
        samples_per_read = int(0.1 * sample_rate)  # 0.1 second = 100 ms

        # 修改：程序启动时不创建音频流，等待用户按回车键开始
        audio_stream = None
        stream = None
        display = Display()
        # 修改：默认显示暂停状态
        display.update_recording_status("暂停")

        while True:
            # 检查来自Qt界面的控制命令
            command = check_stdin_command()
            if command is not None:
                # 接收到Qt界面发送的命令，切换录音状态
                if controller.toggle_recording():
                    status = "开始" if controller.is_recording() else "暂停"
                    display.update_recording_status(status)
                    display.update_text("")  # 清空当前识别文本
                    print(f"\n录音状态已切换为: {status}")
                    
                    # 处理音频流的开启和关闭
                    if controller.is_recording():
                        # 开始录音，创建音频流和识别流
                        if audio_stream is not None and audio_stream.active:
                            audio_stream.close()
                        
                        audio_stream = sd.InputStream(
                            channels=1, 
                            dtype="float32", 
                            samplerate=sample_rate,
                            blocksize=samples_per_read
                        )
                        audio_stream.start()
                        
                        # 创建新的识别流
                        stream = recognizer.create_stream()
                        print("开始录音...")
                    else:
                        # 暂停录音，关闭音频流
                        if audio_stream is not None and audio_stream.active:
                            audio_stream.stop()
                            audio_stream.close()
                            audio_stream = None
                        
                        # 重置识别流
                        if stream is not None:
                            recognizer.reset(stream)
                            stream = None
                        print("录音已暂停")
            
            # 如果是终端模式，也检查键盘输入（用于调试）
            if sys.stdin.isatty():
                try:
                    if select.select([sys.stdin], [], [], 0)[0]:
                        key = sys.stdin.read(1)
                        if key == '\n' or key == '\r':  # 回车键
                            if controller.toggle_recording():
                                status = "开始" if controller.is_recording() else "暂停"
                                display.update_recording_status(status)
                                display.update_text("")  # 清空当前识别文本
                                print(f"\n录音状态已切换为: {status}")
                                
                                # 处理音频流的开启和关闭（同上）
                                if controller.is_recording():
                                    if audio_stream is not None and audio_stream.active:
                                        audio_stream.close()
                                    
                                    audio_stream = sd.InputStream(
                                        channels=1, 
                                        dtype="float32", 
                                        samplerate=sample_rate,
                                        blocksize=samples_per_read
                                    )
                                    audio_stream.start()
                                    stream = recognizer.create_stream()
                                    print("开始录音...")
                                else:
                                    if audio_stream is not None and audio_stream.active:
                                        audio_stream.stop()
                                        audio_stream.close()
                                        audio_stream = None
                                    if stream is not None:
                                        recognizer.reset(stream)
                                        stream = None
                                    print("录音已暂停")
                except:
                    pass
                
            # 检查录音状态
            if not controller.is_recording():
                # 暂停状态，只更新显示，不处理音频
                display.display()
                time.sleep(0.1)
                continue
                
            # 录音状态，处理音频数据
            if audio_stream is None or not audio_stream.active:
                # 确保音频流是活动的
                time.sleep(0.1)
                continue
                
            # 读取音频数据
            if audio_stream.read_available >= samples_per_read:
                samples, _ = audio_stream.read(samples_per_read)
                samples = samples.reshape(-1)
                
                if stream is not None:
                    stream.accept_waveform(sample_rate, samples)
                    
                    # 处理识别结果
                    while recognizer.is_ready(stream):
                        recognizer.decode_stream(stream)

                    is_endpoint = recognizer.is_endpoint(stream)
                    result = recognizer.get_result(stream)

                    display.update_text(result)
                    display.display()

                    if is_endpoint:
                        if result:
                            display.finalize_current_sentence()
                            display.display()
                        recognizer.reset(stream)
            else:
                # 没有足够的数据可读，短暂等待
                time.sleep(0.01)
    
    finally:
        # 恢复终端设置
        if old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        # 确保关闭音频流
        if audio_stream is not None and audio_stream.active:
            audio_stream.stop()
            audio_stream.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nCaught Ctrl + C. Exiting")