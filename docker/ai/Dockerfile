FROM python:3.10-slim

WORKDIR /app

# Install dependencies
COPY lib/voice_llm/requirements.txt /app/requirements.txt
RUN pip install --no-cache-dir -r requirements.txt \
    && pip install flask flask-socketio pyyaml requests chromadb google-generativeai

# Copy application
COPY flask_app/ /app/flask_app/
COPY lib/ /app/lib/
COPY config/ /app/config/

# Shared data (read-only mount)
# Mount: -v /home/anl/Downloads/multi-robot-ar-data-main:/shared_data:ro

WORKDIR /app/flask_app

# Environment variables
ENV PYTHONUNBUFFERED=1
ENV FLASK_APP=app.py

# Expose port
EXPOSE 5000

CMD ["python3", "app.py"]
