#!/usr/bin/env python3

import argparse
import sys
from pathlib import Path
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

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

# 导入自定义模块
from VoiceCommand import VoiceResultPublisher
from ButtonState import ButtonStateSubscriber
from display import Display


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

        ▁HE LL O ▁WORLD
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
        "--num-threads",
        type=int,
        default=1,
        help="""
        调用NPU模式：
        --num-threads=1  选择 RKNN_NPU_CORE_AUTO
        --num-threads=0  选择 RKNN_NPU_CORE_0
        --num-threads=-1 选择 RKNN_NPU_CORE_1
        --num-threads=-2 选择 RKNN_NPU_CORE_2
        --num-threads=-3 选择 RKNN_NPU_CORE_0_1
        --num-threads=-4 选择 RKNN_NPU_CORE_0_1_2
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
        num_threads=args.num_threads,
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


class SpeechRecognitionNode:
    def __init__(self, args):
        rclpy.init()
        
        # 创建ROS2节点
        self.voice_publisher = VoiceResultPublisher()
        self.display = Display()
        
        # 录音状态标志
        self.recording_flag = [False]
        
        # 创建按钮状态订阅器
        self.button_subscriber = ButtonStateSubscriber(self.recording_flag)
        
        # 创建执行器
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.voice_publisher)
        self.executor.add_node(self.button_subscriber)
        
        # 语音识别器
        self.recognizer = create_recognizer(args)
        
        # 音频流设置
        self.sample_rate = 48000
        self.samples_per_read = int(0.1 * self.sample_rate)
        self.audio_stream = None
        
        # 上一个显示的文本
        self.last_displayed_text = ""
        
        # 状态标志
        self.running = True
        self.last_recording_status = None
        
        print("语音识别节点已启动，等待按钮状态...")
        
    def process_audio_loop(self):
        """音频处理循环"""
        stream = self.recognizer.create_stream()
        
        try:
            while self.running:
                current_time = time.time()
                
                # 处理录音状态变化
                if not self.recording_flag[0]:
                    if self.audio_stream is not None:
                        self.audio_stream.close()
                        self.audio_stream = None
                        
                    # 暂停时清理显示
                    if self.last_displayed_text:
                        self.display.update_text("")
                        self.display.display()
                        self.last_displayed_text = ""
                        
                    time.sleep(0.1)
                    continue
                
                # 创建音频流（如果需要）
                if self.audio_stream is None:
                    self.audio_stream = sd.InputStream(
                        channels=1, 
                        dtype="float32", 
                        samplerate=self.sample_rate
                    )
                    self.audio_stream.start()
                    print("开始录音...")

                # 读取音频数据
                samples, _ = self.audio_stream.read(self.samples_per_read)
                samples = samples.reshape(-1)
                stream.accept_waveform(self.sample_rate, samples)
                
                # 处理识别结果
                while self.recognizer.is_ready(stream):
                    self.recognizer.decode_stream(stream)

                # 处理识别结果和端点检测
                is_endpoint = self.recognizer.is_endpoint(stream)
                result = self.recognizer.get_result(stream)
                new_text = result.strip() if isinstance(result, str) else ""
                
                # 更新显示控制
                should_refresh = False
                
                # 检测到端点时，处理最终结果
                if is_endpoint and new_text:
                    # 更新显示
                    self.display.update_text(new_text)
                    self.display.finalize_current_sentence(is_endpoint=True)
                    
                    # 发布最终文本结果
                    self.voice_publisher.publish_text_result(new_text, is_final=True)
                    
                    # 如果有协议，发布指令
                    if hasattr(self.display, 'last_sentence') and self.display.last_sentence:
                        protocol = self.display.last_sentence.get("protocol", "")
                        if protocol:
                            # 这里需要根据协议解析出分类和操作码
                            # 假设协议格式为 "category,operation"
                            try:
                                parts = protocol.split('|')[0].split(',')
                                if len(parts) >= 2:
                                    category = int(parts[0])
                                    operation = int(parts[1])
                                    self.voice_publisher.publish_command_result(
                                        category=category,
                                        operation=operation,
                                        command_id=1,  # 可以根据需要设置
                                        command_text=new_text
                                    )
                            except (ValueError, IndexError) as e:
                                print(f"协议解析失败: {e}")
                    
                    should_refresh = True
                    self.recognizer.reset(stream)
                    self.last_displayed_text = ""
                
                # 文本变化处理
                elif new_text != self.last_displayed_text and new_text:
                    # 发布中间文本结果
                    self.voice_publisher.publish_text_result(new_text, is_final=False)
                    
                    # 更新显示
                    self.display.update_text(new_text)
                    self.display.display(incremental=True)
                    should_refresh = True
                
                # 需要刷新显示
                if should_refresh:
                    self.last_displayed_text = new_text
                
        except KeyboardInterrupt:
            print("\n收到中断信号，正在退出...")
        except Exception as e:
            print(f"音频处理错误: {e}")
        finally:
            if self.audio_stream is not None:
                self.audio_stream.close()
    
    def run(self):
        """运行节点"""
        # 启动音频处理线程
        audio_thread = threading.Thread(target=self.process_audio_loop, daemon=True)
        audio_thread.start()
        
        try:
            # 运行ROS2执行器
            self.executor.spin()
        except KeyboardInterrupt:
            print("\n正在关闭节点...")
        finally:
            self.running = False
            self.executor.shutdown()
            rclpy.shutdown()
            print("节点已关闭")


def main():
    args = get_args()
    
    # 检查音频设备
    devices = sd.query_devices()
    if len(devices) == 0:
        print("未找到麦克风设备")
        sys.exit(0)
    
    # 创建并运行语音识别节点
    speech_node = SpeechRecognitionNode(args)
    speech_node.run()


if __name__ == "__main__":
    main()