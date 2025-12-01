"""
Whisper STT Standalone Test Script
Riley 음성인식 기능 테스트용

Usage:
    # 1. Install whisper
    pip install openai-whisper

    # 2. Record test audio (마이크 녹음)
    python test_whisper_standalone.py --record test_audio.wav

    # 3. Transcribe audio file
    python test_whisper_standalone.py --file test_audio.wav

    # 4. Live recording + transcription
    python test_whisper_standalone.py --live

Requirements:
    pip install openai-whisper sounddevice soundfile numpy
"""

import argparse
import sys
import tempfile
import os
from pathlib import Path

# Optional imports
try:
    import whisper
    WHISPER_AVAILABLE = True
except ImportError:
    WHISPER_AVAILABLE = False
    print("[Error] openai-whisper not installed. Run: pip install openai-whisper")

try:
    import sounddevice as sd
    import soundfile as sf
    import numpy as np
    AUDIO_AVAILABLE = True
except ImportError:
    AUDIO_AVAILABLE = False
    print("[Warning] Audio recording not available. Run: pip install sounddevice soundfile numpy")


class WhisperTester:
    """Standalone Whisper STT tester"""

    def __init__(self, model_name="base"):
        self.model_name = model_name
        self._model = None

    def load_model(self):
        """Load Whisper model (lazy loading)"""
        if not WHISPER_AVAILABLE:
            print("[Error] Whisper not available")
            return False

        if self._model is None:
            print(f"[Whisper] Loading model '{self.model_name}'...")
            print("  (First load may take a while to download)")
            self._model = whisper.load_model(self.model_name)
            print(f"[Whisper] Model loaded successfully")
        return True

    def transcribe_file(self, audio_path: str) -> str:
        """Transcribe audio file to text"""
        if not self.load_model():
            return None

        if not os.path.exists(audio_path):
            print(f"[Error] File not found: {audio_path}")
            return None

        print(f"[Whisper] Transcribing: {audio_path}")
        result = self._model.transcribe(audio_path)

        text = result.get("text", "").strip()
        language = result.get("language", "unknown")

        print(f"[Whisper] Detected language: {language}")
        print(f"[Whisper] Transcription: {text}")

        return text

    def transcribe_bytes(self, audio_bytes: bytes, suffix=".wav") -> str:
        """Transcribe audio bytes to text"""
        if not self.load_model():
            return None

        # Save to temp file
        with tempfile.NamedTemporaryFile(suffix=suffix, delete=False) as f:
            f.write(audio_bytes)
            temp_path = f.name

        try:
            return self.transcribe_file(temp_path)
        finally:
            os.unlink(temp_path)

    def record_audio(self, output_path: str, duration: float = 5.0, sample_rate: int = 16000):
        """Record audio from microphone"""
        if not AUDIO_AVAILABLE:
            print("[Error] Audio recording not available")
            return False

        print(f"[Recording] Duration: {duration}s, Sample rate: {sample_rate}Hz")
        print("[Recording] Speak now...")

        try:
            # Record audio
            recording = sd.rec(
                int(duration * sample_rate),
                samplerate=sample_rate,
                channels=1,
                dtype='float32'
            )
            sd.wait()

            # Save to file
            sf.write(output_path, recording, sample_rate)
            print(f"[Recording] Saved to: {output_path}")
            return True

        except Exception as e:
            print(f"[Error] Recording failed: {e}")
            return False

    def live_transcribe(self, duration: float = 5.0):
        """Record and transcribe in one step"""
        if not AUDIO_AVAILABLE:
            print("[Error] Audio recording not available")
            return None

        # Create temp file for recording
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
            temp_path = f.name

        try:
            if self.record_audio(temp_path, duration):
                return self.transcribe_file(temp_path)
            return None
        finally:
            if os.path.exists(temp_path):
                os.unlink(temp_path)


def test_with_sample_text():
    """Test transcription with built-in sample (requires recording first)"""
    tester = WhisperTester(model_name="base")

    print("\n" + "="*60)
    print("Whisper STT Test")
    print("="*60)

    # Just load model to verify installation
    if tester.load_model():
        print("\n[Success] Whisper is working correctly!")
        print("[Info] Available models: tiny, base, small, medium, large")
        print("  - tiny: fastest, least accurate")
        print("  - base: good balance (recommended)")
        print("  - small/medium: better accuracy")
        print("  - large: best accuracy, requires GPU")
        return True
    return False


def main():
    parser = argparse.ArgumentParser(description="Whisper STT Standalone Test")
    parser.add_argument("--model", default="base", help="Whisper model (tiny/base/small/medium/large)")
    parser.add_argument("--record", metavar="FILE", help="Record audio to file")
    parser.add_argument("--file", metavar="FILE", help="Transcribe audio file")
    parser.add_argument("--live", action="store_true", help="Live record + transcribe")
    parser.add_argument("--duration", type=float, default=5.0, help="Recording duration (seconds)")
    parser.add_argument("--test", action="store_true", help="Test Whisper installation")

    args = parser.parse_args()

    tester = WhisperTester(model_name=args.model)

    if args.test:
        test_with_sample_text()

    elif args.record:
        tester.record_audio(args.record, duration=args.duration)

    elif args.file:
        text = tester.transcribe_file(args.file)
        if text:
            print(f"\n[Result] {text}")

    elif args.live:
        print(f"\n[Live Mode] Recording {args.duration} seconds...")
        text = tester.live_transcribe(duration=args.duration)
        if text:
            print(f"\n[Result] {text}")

    else:
        # Default: interactive mode
        print("\n" + "="*60)
        print("Whisper STT Interactive Test")
        print("="*60)

        if not tester.load_model():
            return

        if AUDIO_AVAILABLE:
            print("\nOptions:")
            print("  1. Record and transcribe (5 seconds)")
            print("  2. Record and transcribe (10 seconds)")
            print("  3. Transcribe existing file")
            print("  q. Quit")

            while True:
                choice = input("\nChoice: ").strip()

                if choice == "1":
                    text = tester.live_transcribe(duration=5.0)
                    if text:
                        print(f"\n>>> {text}")

                elif choice == "2":
                    text = tester.live_transcribe(duration=10.0)
                    if text:
                        print(f"\n>>> {text}")

                elif choice == "3":
                    file_path = input("File path: ").strip()
                    text = tester.transcribe_file(file_path)
                    if text:
                        print(f"\n>>> {text}")

                elif choice.lower() == "q":
                    break

        else:
            print("\n[Warning] Audio recording not available")
            print("To enable recording: pip install sounddevice soundfile numpy")

            file_path = input("\nEnter audio file path to transcribe (or 'q' to quit): ").strip()
            if file_path.lower() != 'q':
                text = tester.transcribe_file(file_path)
                if text:
                    print(f"\n>>> {text}")


if __name__ == "__main__":
    main()
