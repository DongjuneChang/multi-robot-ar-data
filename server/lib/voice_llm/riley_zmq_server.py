"""
Riley ZMQ Server - Standalone test server
Unity ↔ Python ZMQ 2채널 통신 테스트용

Channel #1 (5565): Unity PUB → Python SUB (음성/명령)
Channel #2 (5566): Python PUB → Unity SUB (응답)

Usage:
    python riley_zmq_server.py [--ollama-url http://localhost:11434]
"""

import zmq
import json
import time
import base64
import tempfile
import os
import argparse
import threading
from pathlib import Path

# Optional: Whisper for STT
try:
    import whisper
    WHISPER_AVAILABLE = True
except ImportError:
    WHISPER_AVAILABLE = False
    print("[Warning] Whisper not available - STT disabled")

# Optional: Ollama for LLM
try:
    import requests
    REQUESTS_AVAILABLE = True
except ImportError:
    REQUESTS_AVAILABLE = False
    print("[Warning] requests not available - LLM disabled")


class RileyZMQServer:
    """
    Riley ZMQ Server for Voice AI Agent

    Receives voice commands from Unity via ZMQ,
    processes through Whisper STT and Ollama LLM,
    sends responses back to Unity.
    """

    def __init__(self, unity_pub_port=5565, python_pub_port=5566, ollama_url="http://localhost:11434"):
        self.unity_pub_port = unity_pub_port
        self.python_pub_port = python_pub_port
        self.ollama_url = ollama_url
        self.ollama_model = "llama3.2:3b"

        self.context = None
        self.sub_socket = None
        self.pub_socket = None
        self._running = False

        # Whisper model (lazy load)
        self._whisper_model = None

        # Conversation history per user
        self.conversations = {}

        # System prompt
        self.system_prompt = """You are Riley, a helpful AI assistant in a Mixed Reality environment.
You assist users with robot control, task execution, and general questions.
Keep responses concise and helpful.
Respond in the same language as the user's input."""

    def start(self):
        """Start ZMQ server"""
        print(f"[RileyServer] Starting...")
        print(f"  SUB port: {self.unity_pub_port} (Unity → Python)")
        print(f"  PUB port: {self.python_pub_port} (Python → Unity)")
        print(f"  Ollama: {self.ollama_url}")

        self.context = zmq.Context()

        # Subscribe to Unity messages
        self.sub_socket = self.context.socket(zmq.SUB)
        self.sub_socket.bind(f"tcp://*:{self.unity_pub_port}")
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")

        # Publish responses to Unity
        self.pub_socket = self.context.socket(zmq.PUB)
        self.pub_socket.bind(f"tcp://*:{self.python_pub_port}")

        self._running = True
        print("[RileyServer] Ready! Waiting for messages...")

        self._message_loop()

    def stop(self):
        """Stop ZMQ server"""
        self._running = False

        if self.sub_socket:
            self.sub_socket.close()
        if self.pub_socket:
            self.pub_socket.close()
        if self.context:
            self.context.term()

        print("[RileyServer] Stopped")

    def _message_loop(self):
        """Main message processing loop"""
        poller = zmq.Poller()
        poller.register(self.sub_socket, zmq.POLLIN)

        while self._running:
            try:
                socks = dict(poller.poll(100))  # 100ms timeout

                if self.sub_socket in socks:
                    message = self.sub_socket.recv_string()
                    self._handle_message(message)

            except zmq.ZMQError as e:
                if self._running:
                    print(f"[RileyServer] ZMQ Error: {e}")
            except KeyboardInterrupt:
                print("\n[RileyServer] Interrupted")
                break
            except Exception as e:
                print(f"[RileyServer] Error: {e}")

    def _handle_message(self, message_str):
        """Handle incoming message from Unity"""
        try:
            data = json.loads(message_str)
            msg_type = data.get("type", "")

            print(f"[RileyServer] Received: {msg_type}")

            if msg_type == "HEARTBEAT":
                self._send_heartbeat_response()

            elif msg_type == "VOICE":
                self._handle_voice(data)

            elif msg_type == "COMMAND":
                self._handle_command(data)

            elif msg_type == "VOICE_COMMAND":
                # Text-based voice command (already transcribed)
                self._handle_text_command(data)

            elif msg_type == "AUDIO_FOR_STT":
                # WAV audio from Unity for Whisper STT processing
                self._handle_audio_for_stt(data)

            elif msg_type == "WAKE_WORD_DETECTED":
                # Wake word detected - log but no action needed
                print(f"[RileyServer] Wake word detected at {data.get('data', {}).get('timestamp', 'unknown')}")

            elif msg_type == "LISTENING_ENDED":
                # Listening ended - log but no action needed
                print(f"[RileyServer] Listening ended")

            else:
                print(f"[RileyServer] Unknown message type: {msg_type}")

        except json.JSONDecodeError as e:
            print(f"[RileyServer] Invalid JSON: {e}")

    def _send_heartbeat_response(self):
        """Send heartbeat response"""
        response = {
            "type": "HEARTBEAT_RESPONSE",
            "timestamp": int(time.time() * 1000)
        }
        self.pub_socket.send_string(json.dumps(response))

    def _handle_voice(self, data):
        """Handle voice/audio data"""
        user_id = data.get("user_id", "unknown")
        audio_base64 = data.get("audio", "")
        audio_format = data.get("audio_format", "wav")

        if not audio_base64:
            self._send_error("No audio data received")
            return

        # Decode audio
        try:
            audio_bytes = base64.b64decode(audio_base64)
        except Exception as e:
            self._send_error(f"Failed to decode audio: {e}")
            return

        # STT with Whisper
        text = self._transcribe(audio_bytes)
        if not text:
            self._send_error("Failed to transcribe audio")
            return

        print(f"[RileyServer] STT Result: {text}")

        # Generate LLM response
        response_text = self._generate_response(text, user_id)

        # Send response
        self._send_response(response_text, user_id)

    def _handle_audio_for_stt(self, data):
        """Handle AUDIO_FOR_STT from Unity (WAV bytes for Whisper processing)

        Unity sends:
        {
            "type": "AUDIO_FOR_STT",
            "data": {
                "audio_base64": "...",
                "format": "wav",
                "sample_rate": 16000,
                "channels": 1,
                "byte_size": 12345,
                "timestamp": 1234567890
            },
            "device_id": "...",
            "session_time": 123.45
        }
        """
        # Extract from 'data' wrapper (Unity's RileyMessenger wraps payload)
        payload = data.get("data", data)

        device_id = data.get("device_id", "unknown")
        audio_base64 = payload.get("audio_base64", "")
        audio_format = payload.get("format", "wav")
        sample_rate = payload.get("sample_rate", 16000)
        byte_size = payload.get("byte_size", 0)

        print(f"[RileyServer] AUDIO_FOR_STT - Format: {audio_format}, SampleRate: {sample_rate}, Size: {byte_size} bytes")

        if not audio_base64:
            self._send_error("No audio data in AUDIO_FOR_STT")
            return

        # Decode audio
        try:
            audio_bytes = base64.b64decode(audio_base64)
            print(f"[RileyServer] Decoded audio: {len(audio_bytes)} bytes")
        except Exception as e:
            self._send_error(f"Failed to decode audio: {e}")
            return

        # STT with Whisper
        text = self._transcribe(audio_bytes)
        if not text:
            self._send_error("Failed to transcribe audio")
            return

        print(f"[RileyServer] STT Result: {text}")

        # Generate LLM response
        response_text = self._generate_response(text, device_id)

        # Send response
        self._send_response(response_text, device_id)

    def _handle_command(self, data):
        """Handle text command"""
        user_id = data.get("user_id", "unknown")
        command = data.get("command", "")

        if not command:
            self._send_error("No command received")
            return

        # Generate LLM response
        response_text = self._generate_response(command, user_id)

        # Send response
        self._send_response(response_text, user_id)

    def _handle_text_command(self, data):
        """Handle VOICE_COMMAND (pre-transcribed text)"""
        user_id = data.get("user_id", "unknown")
        text = data.get("text", "")

        if not text:
            self._send_error("No text received")
            return

        print(f"[RileyServer] Voice Command: {text}")

        # Generate LLM response
        response_text = self._generate_response(text, user_id)

        # Send response
        self._send_response(response_text, user_id)

    def _transcribe(self, audio_bytes):
        """Transcribe audio using Whisper"""
        if not WHISPER_AVAILABLE:
            return "[STT not available - Whisper not installed]"

        # Load model if needed
        if self._whisper_model is None:
            print("[RileyServer] Loading Whisper model...")
            self._whisper_model = whisper.load_model("base")
            print("[RileyServer] Whisper model loaded")

        # Save to temp file
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
            f.write(audio_bytes)
            temp_path = f.name

        try:
            result = self._whisper_model.transcribe(temp_path)
            return result.get("text", "").strip()
        finally:
            os.unlink(temp_path)

    def _generate_response(self, text, user_id):
        """Generate response using Ollama LLM"""
        if not REQUESTS_AVAILABLE:
            return f"[Echo] {text}"

        # Get/create conversation history
        if user_id not in self.conversations:
            self.conversations[user_id] = []

        history = self.conversations[user_id]

        # Build prompt
        prompt = ""
        if self.system_prompt:
            prompt += f"{self.system_prompt}\n\n"

        for msg in history[-10:]:  # Last 10 messages
            role = "Human" if msg["role"] == "user" else "Assistant"
            prompt += f"{role}: {msg['content']}\n"

        prompt += f"Human: {text}\nAssistant:"

        try:
            response = requests.post(
                f"{self.ollama_url}/api/generate",
                json={
                    "model": self.ollama_model,
                    "prompt": prompt,
                    "stream": False,
                    "options": {
                        "num_predict": 512,
                        "temperature": 0.7
                    }
                },
                timeout=60
            )

            if response.status_code == 200:
                result = response.json().get("response", "").strip()

                # Update history
                history.append({"role": "user", "content": text})
                history.append({"role": "assistant", "content": result})

                return result
            else:
                return f"[LLM Error] Status {response.status_code}"

        except requests.Timeout:
            return "[LLM Timeout] Please try again"
        except requests.ConnectionError:
            return f"[LLM Connection Error] Cannot connect to {self.ollama_url}"
        except Exception as e:
            return f"[LLM Error] {str(e)}"

    def _send_response(self, text, user_id=None):
        """Send response to Unity"""
        response = {
            "type": "RESPONSE",
            "text": text,
            "timestamp": int(time.time() * 1000)
        }

        if user_id:
            response["user_id"] = user_id

        self.pub_socket.send_string(json.dumps(response))
        print(f"[RileyServer] Sent response: {text[:50]}...")

    def _send_error(self, message):
        """Send error to Unity"""
        response = {
            "type": "ERROR",
            "message": message,
            "timestamp": int(time.time() * 1000)
        }
        self.pub_socket.send_string(json.dumps(response))
        print(f"[RileyServer] Error: {message}")


def main():
    parser = argparse.ArgumentParser(description="Riley ZMQ Server")
    parser.add_argument("--unity-port", type=int, default=5565, help="Unity publish port")
    parser.add_argument("--python-port", type=int, default=5566, help="Python publish port")
    parser.add_argument("--ollama-url", default="http://localhost:11434", help="Ollama API URL")
    parser.add_argument("--model", default="llama3.2:3b", help="Ollama model name")

    args = parser.parse_args()

    server = RileyZMQServer(
        unity_pub_port=args.unity_port,
        python_pub_port=args.python_port,
        ollama_url=args.ollama_url
    )
    server.ollama_model = args.model

    try:
        server.start()
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


if __name__ == "__main__":
    main()
