"""
AI Bot 참여자 클래스
- telephone.md 설계 기반
- voice-llm-test (Ollama + ZMQ + TTS) 통합
- voice-riley-multiplatform (STT) 참고
"""

import sys
import io
from pathlib import Path

# Add voice_llm to path
sys.path.insert(0, str(Path(__file__).parent.parent / 'lib' / 'voice_llm'))

try:
    from zmq_bridge import ZMQBridge
except (ImportError, FileNotFoundError) as e:
    ZMQBridge = None
    print(f"Warning: ZMQBridge not available: {e}")

# ChromaDB for conversation history
try:
    from chroma_client import get_chroma_client
    CHROMA_AVAILABLE = True
except ImportError as e:
    get_chroma_client = None
    CHROMA_AVAILABLE = False
    print(f"Warning: ChromaDB client not available: {e}")


class AIBot:
    """AI Bot participant for Teams + AI system"""

    def __init__(self, user_id, room, llm_config, zmq_config, tts_config, actual_user_id=None):
        """
        Initialize AI Bot

        Args:
            user_id: Bot key (format: "{room}_{user_id}")
            room: Room ID for Socket.IO
            llm_config: LLM configuration (model, timeout, url)
            zmq_config: ZMQ ports configuration
            tts_config: TTS engine configuration
            actual_user_id: Actual user ID (e.g., "alice")
        """
        self.user_id = user_id  # This is actually bot_key
        self.room = room
        self.actual_user_id = actual_user_id if actual_user_id else user_id

        # LLM configuration
        self.llm_provider = llm_config.get('provider', 'ollama')  # 'ollama' or 'gemini'
        self.llm_model = llm_config.get('model', 'llama3.2')
        self.llm_timeout = llm_config.get('timeout', 180)
        self.llm_url = llm_config.get('url', 'http://localhost:11434')
        self.llm_api_key = llm_config.get('api_key', '')
        self.system_prompt = llm_config.get('system_prompt', 'You are a helpful AI assistant.')

        # Conversation history (LangChain ConversationBufferMemory pattern)
        self.conversation_history = []
        self.max_history = 10  # Keep last 10 exchanges

        # ZMQ Unity bridge
        if ZMQBridge:
            self.zmq = ZMQBridge(
                unity_pub_port=zmq_config.get('unity_pub_port', 5565),
                python_pub_port=zmq_config.get('python_pub_port', 5566)
            )
        else:
            self.zmq = None

        # TTS configuration
        self.tts_engine = tts_config.get('engine', 'browser')
        self.tts_lang = tts_config.get('default_language', 'ko-KR')
        self.tts_speed = tts_config.get('voice_speed', 1.0)

        # LLM client (lazy initialization)
        self._llm_client = None

    def start(self):
        """Start AI Bot services"""
        if self.zmq:
            self.zmq.start()

    def stop(self):
        """Stop AI Bot services"""
        if self.zmq:
            self.zmq.stop()

    @property
    def llm_client(self):
        """Lazy load LLM client based on provider"""
        if self._llm_client is None:
            try:
                if self.llm_provider == 'gemini':
                    self._llm_client = GeminiClient(
                        model=self.llm_model,
                        timeout=self.llm_timeout,
                        api_key=self.llm_api_key
                    )
                else:  # default: ollama
                    self._llm_client = OllamaClient(
                        model=self.llm_model,
                        timeout=self.llm_timeout,
                        url=self.llm_url
                    )
            except Exception as e:
                print(f"Failed to initialize LLM client: {e}")
        return self._llm_client

    def on_human_voice(self, text):
        """
        Handle human voice input (STT result)

        Args:
            text: Transcribed text from human speech

        Returns:
            dict: Response data with text and audio
        """
        # Generate LLM response
        response_text = self.generate_response(text)

        # Convert to speech if not browser TTS
        audio_data = None
        if self.tts_engine != 'browser':
            audio_data = self.text_to_speech(response_text)

        return {
            'bot_id': self.user_id,
            'text': response_text,
            'audio': audio_data,
            'audio_format': 'mp3' if audio_data else None,
            'tts_engine': self.tts_engine
        }

    def generate_response(self, user_text):
        """
        Generate AI response using LLM with system prompt and conversation history

        Args:
            user_text: Input text from user

        Returns:
            str: AI response
        """
        if not self.llm_client:
            return "LLM client not available"

        try:
            # Generate response with system prompt and history
            response = self.llm_client.generate(
                user_text,
                system_prompt=self.system_prompt,
                conversation_history=self.conversation_history
            )

            # Update conversation history (LangChain pattern)
            self.conversation_history.append({
                'role': 'user',
                'content': user_text
            })
            self.conversation_history.append({
                'role': 'assistant',
                'content': response
            })

            # Limit history size
            if len(self.conversation_history) > self.max_history * 2:
                self.conversation_history = self.conversation_history[-self.max_history * 2:]

            # Save to ChromaDB (organized by room AND user)
            if CHROMA_AVAILABLE:
                try:
                    chroma_client = get_chroma_client()

                    # Save user message
                    chroma_client.add_message(
                        content=user_text,
                        role='user',
                        room=self.room,
                        bot_key=self.user_id,
                        user_id=self.actual_user_id,
                        sender=self.actual_user_id
                    )

                    # Save assistant response
                    chroma_client.add_message(
                        content=response,
                        role='assistant',
                        room=self.room,
                        bot_key=self.user_id,
                        user_id=self.actual_user_id,
                        sender=f"RILEY_{self.actual_user_id}"
                    )
                except Exception as e:
                    print(f"Warning: Failed to save to ChromaDB: {e}")

            return response
        except Exception as e:
            return f"LLM error: {str(e)}"

    def text_to_speech(self, text):
        """
        Convert text to speech audio

        Args:
            text: Text to synthesize

        Returns:
            bytes: Audio data (MP3) or None
        """
        if self.tts_engine == 'gtts':
            try:
                from gtts import gTTS
                tts = gTTS(text=text, lang=self.tts_lang.split('-')[0])
                audio_fp = io.BytesIO()
                tts.write_to_fp(audio_fp)
                audio_fp.seek(0)
                return audio_fp.read()
            except Exception as e:
                print(f"TTS error: {e}")
                return None

        elif self.tts_engine == 'browser':
            # Browser-side TTS (no server-side generation)
            return None

        return None

    def send_robot_command(self, command):
        """
        Send robot control command via ZMQ

        Args:
            command: Command dict to send to Unity/robot
        """
        if self.zmq:
            self.zmq.publish({
                'type': 'robot_command',
                'command': command,
                'source': 'ai_bot',
                'user_id': self.user_id
            })


class GeminiClient:
    """Google Gemini API client"""

    def __init__(self, model='gemini-2.0-flash', timeout=180, api_key=None):
        self.model = model
        self.timeout = timeout
        self.api_key = api_key
        self._client = None

    def _get_client(self):
        if self._client is None:
            try:
                import google.generativeai as genai
                genai.configure(api_key=self.api_key)
                self._client = genai.GenerativeModel(self.model)
            except ImportError:
                raise Exception("google-generativeai package not installed")
        return self._client

    def generate(self, prompt, system_prompt=None, conversation_history=None):
        """
        Generate response from Gemini with system prompt and conversation history
        """
        model = self._get_client()

        # Build messages for Gemini
        contents = []

        # Build conversation context
        context = ""
        if system_prompt:
            context = f"{system_prompt}\n\n"

        if conversation_history:
            for msg in conversation_history:
                role = "Human" if msg['role'] == 'user' else "Assistant"
                context += f"{role}: {msg['content']}\n"

        context += f"Human: {prompt}\nAssistant:"

        try:
            response = model.generate_content(context)
            return response.text.strip()
        except Exception as e:
            raise Exception(f"Gemini API error: {str(e)}")


class OllamaClient:
    """Simple Ollama API client"""

    def __init__(self, model='llama3.2', timeout=180, url='http://localhost:11434'):
        self.model = model
        self.timeout = timeout
        self.url = url

    def generate(self, prompt, system_prompt=None, conversation_history=None):
        """
        Generate response from Ollama with system prompt and conversation history

        Args:
            prompt: Current user input
            system_prompt: System/persona prompt
            conversation_history: List of previous messages [{'role': 'user'|'assistant', 'content': str}]

        Returns:
            str: Generated response
        """
        import requests

        # Build full prompt with system + history + current
        full_prompt = ""

        # Add system prompt
        if system_prompt:
            full_prompt += f"{system_prompt}\n\n"

        # Add conversation history
        if conversation_history:
            for msg in conversation_history:
                role = "Human" if msg['role'] == 'user' else "Assistant"
                full_prompt += f"{role}: {msg['content']}\n"

        # Add current user message
        full_prompt += f"Human: {prompt}\nAssistant:"

        response = requests.post(
            f'{self.url}/api/generate',
            json={
                'model': self.model,
                'prompt': full_prompt,
                'stream': False,
                'options': {
                    'num_predict': 2048,   # Allow longer responses (GPU mode)
                    'num_ctx': 8192,       # Extended context window
                    'temperature': 0.3,    # Lower randomness for consistency
                    'top_p': 0.9,          # Nucleus sampling
                    'repeat_penalty': 1.1  # Reduce repetition
                }
            },
            timeout=self.timeout
        )

        if response.status_code == 200:
            return response.json().get('response', '').strip()
        else:
            raise Exception(f"Ollama API error: {response.status_code}")
