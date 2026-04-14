FROM python:3.12-slim

WORKDIR /app

# Install ifssim Python client
COPY python/ /ifssim_client/
RUN pip install --no-cache-dir /ifssim_client

# Install backend dependencies
COPY tools/mission_control/backend/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy backend source
COPY tools/mission_control/backend/ .

ENV IFSSIM_HOST=host.docker.internal
ENV IFSSIM_PORT=41451

EXPOSE 8000
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
