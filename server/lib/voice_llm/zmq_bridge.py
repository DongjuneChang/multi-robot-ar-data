"""
ZMQ Bridge for Unity ↔ Python communication
2채널 PUSH/PULL 구조:
- Channel #1 (5565): Unity → Python (음성/명령)
- Channel #2 (5566): Python → Unity (응답)

Based on: 2025-11-29_voice_plan_ai_unity_and_server_by_gpt.md
"""

import zmq
import json
import threading
import time
from typing import Callable, Optional, Dict, Any


class ZMQBridge:
    """
    ZeroMQ Bridge for Unity ↔ Python bidirectional communication

    Unity sends:
        - type: "VOICE" - WAV audio data (base64)
        - type: "COMMAND" - Text command
        - type: "HEARTBEAT" - Connection check

    Python responds:
        - type: "RESPONSE" - LLM text response
        - type: "TTS_AUDIO" - TTS audio data (base64)
        - type: "HEARTBEAT_RESPONSE" - Connection confirmation
    """

    def __init__(self, unity_pub_port: int = 5565, python_pub_port: int = 5566):
        """
        Initialize ZMQ Bridge

        Args:
            unity_pub_port: Port where Unity publishes (Python subscribes)
            python_pub_port: Port where Python publishes (Unity subscribes)
        """
        self.unity_pub_port = unity_pub_port
        self.python_pub_port = python_pub_port

        self.context: Optional[zmq.Context] = None
        self.sub_socket: Optional[zmq.Socket] = None  # Unity → Python
        self.pub_socket: Optional[zmq.Socket] = None  # Python → Unity

        self._running = False
        self._listener_thread: Optional[threading.Thread] = None
        self._message_handler: Optional[Callable[[Dict[str, Any]], None]] = None

    def start(self):
        """Start ZMQ bridge (both subscriber and publisher)"""
        if self._running:
            print("[ZMQBridge] Already running")
            return

        try:
            self.context = zmq.Context()

            # Subscriber socket (Unity → Python)
            # Unity publishes to 5565, Python subscribes
            self.sub_socket = self.context.socket(zmq.SUB)
            self.sub_socket.connect(f"tcp://localhost:{self.unity_pub_port}")
            self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all
            self.sub_socket.setsockopt(zmq.RCVTIMEO, 100)  # 100ms timeout for non-blocking

            # Publisher socket (Python → Unity)
            # Python publishes to 5566, Unity subscribes
            self.pub_socket = self.context.socket(zmq.PUB)
            self.pub_socket.bind(f"tcp://*:{self.python_pub_port}")

            self._running = True

            # Start listener thread
            self._listener_thread = threading.Thread(target=self._listen_loop, daemon=True)
            self._listener_thread.start()

            print(f"[ZMQBridge] Started - SUB:{self.unity_pub_port}, PUB:{self.python_pub_port}")

        except Exception as e:
            print(f"[ZMQBridge] Failed to start: {e}")
            self.stop()
            raise

    def stop(self):
        """Stop ZMQ bridge and cleanup"""
        self._running = False

        if self._listener_thread:
            self._listener_thread.join(timeout=1.0)
            self._listener_thread = None

        if self.sub_socket:
            try:
                self.sub_socket.close()
            except:
                pass
            self.sub_socket = None

        if self.pub_socket:
            try:
                self.pub_socket.close()
            except:
                pass
            self.pub_socket = None

        if self.context:
            try:
                self.context.term()
            except:
                pass
            self.context = None

        print("[ZMQBridge] Stopped")

    def set_message_handler(self, handler: Callable[[Dict[str, Any]], None]):
        """
        Set callback for incoming messages from Unity

        Args:
            handler: Callback function that receives message dict
        """
        self._message_handler = handler

    def publish(self, data: Dict[str, Any]):
        """
        Publish message to Unity

        Args:
            data: Message dict to send
        """
        if not self.pub_socket or not self._running:
            print("[ZMQBridge] Cannot publish - not running")
            return

        try:
            json_str = json.dumps(data)
            self.pub_socket.send_string(json_str)
        except Exception as e:
            print(f"[ZMQBridge] Publish error: {e}")

    def send_response(self, text: str, user_id: str = None, audio_base64: str = None):
        """
        Send LLM response to Unity

        Args:
            text: Response text
            user_id: User ID for routing
            audio_base64: Optional TTS audio as base64
        """
        response = {
            "type": "RESPONSE",
            "text": text,
            "timestamp": int(time.time() * 1000)
        }

        if user_id:
            response["user_id"] = user_id

        if audio_base64:
            response["audio"] = audio_base64
            response["audio_format"] = "wav"

        self.publish(response)

    def send_heartbeat_response(self):
        """Send heartbeat response to Unity"""
        self.publish({
            "type": "HEARTBEAT_RESPONSE",
            "timestamp": int(time.time() * 1000)
        })

    def _listen_loop(self):
        """Background thread for receiving messages from Unity"""
        print("[ZMQBridge] Listener started")

        while self._running:
            try:
                if not self.sub_socket:
                    break

                # Non-blocking receive with timeout
                message = self.sub_socket.recv_string(flags=zmq.NOBLOCK)

                try:
                    data = json.loads(message)
                    self._handle_message(data)
                except json.JSONDecodeError as e:
                    print(f"[ZMQBridge] Invalid JSON: {e}")

            except zmq.Again:
                # No message available, continue
                pass
            except zmq.ZMQError as e:
                if self._running:
                    print(f"[ZMQBridge] ZMQ error: {e}")
                break
            except Exception as e:
                if self._running:
                    print(f"[ZMQBridge] Listener error: {e}")
                time.sleep(0.1)

        print("[ZMQBridge] Listener stopped")

    def _handle_message(self, data: Dict[str, Any]):
        """Handle incoming message from Unity"""
        msg_type = data.get("type", "")

        # Auto-respond to heartbeat
        if msg_type == "HEARTBEAT":
            self.send_heartbeat_response()
            return

        # Forward to handler
        if self._message_handler:
            try:
                self._message_handler(data)
            except Exception as e:
                print(f"[ZMQBridge] Handler error: {e}")
        else:
            print(f"[ZMQBridge] No handler for message type: {msg_type}")


# Standalone test
if __name__ == "__main__":
    def test_handler(data):
        print(f"Received: {data}")

    bridge = ZMQBridge()
    bridge.set_message_handler(test_handler)
    bridge.start()

    print("ZMQ Bridge running. Press Ctrl+C to stop...")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.stop()
