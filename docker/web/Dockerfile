# Multi-HRI Web Server Dockerfile
FROM python:3.10-slim

WORKDIR /app

# Install dependencies
COPY server/requirements.txt /app/
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code (new api/ structure)
COPY server/main.py /app/server/
COPY server/routes_manager.py /app/server/
COPY server/api/ /app/server/api/

WORKDIR /app/server

# Environment
ENV PYTHONUNBUFFERED=1
ENV DOCKER_ENV=1
ENV FLASK_APP=main.py

# Expose port
EXPOSE 5000

# Run
CMD ["python", "main.py"]
