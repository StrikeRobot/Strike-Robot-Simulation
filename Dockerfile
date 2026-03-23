# Stage 1: Build the frontend (Node.js)
FROM node:18-alpine AS builder

WORKDIR /app

# Copy package files and install dependencies
COPY package.json package-lock.json ./
RUN npm ci

# Copy source code
COPY src ./src
COPY assets ./assets
# We need to copy other files that might be needed for build if referenced, 
# but based on package.json, only ./src/main.js is built to ./build
# However, the simple http server serves the root, so we need everything in the final image.

# Run the build script
RUN npm run build

# Stage 2: Runtime (Python)
FROM python:3.10-slim

WORKDIR /app

# Install system dependencies (needed for some python packages or tools)
# git might be needed if dependencies are git urls
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
# collected from imports in socket_server.py and run_server.py
RUN pip install --no-cache-dir \
    websockets \
    numpy \
    pyyaml \
    onnxruntime

# Copy all project files to the container
COPY . /app

# Copy the built assets from the builder stage
COPY --from=builder /app/build /app/build
# Note: The 'node_modules' are not copied to runtime as we only need the built artifacts 
# and the python runtime. However, if 'mujoco-js' or 'three' are needed at runtime 
# (not just bundled), we might need them. 
# Looking at package.json, 'esbuild' bundles everything into ./build/main.js? 
# The build command is: esbuild ./src/main.js --bundle ...
# So we should be good with just the build directory and existing assets.

# Expose ports
# 8000: HTTP Server (Frontend)
# 8765: WebSocket Server (Backend)
EXPOSE 8000 8765

# Environment variable for ROOT_DIR (can be overridden at runtime)
# Default is set to a generic path, user must mount volume or change this.
ENV ROOT_DIR="/config"

# Script to run both servers? 
# Since we have two separate scripts, it's better to let the user decide or run one by default.
# But for convenience, let's create a small shell script to run both or just use one.
# For now, let's default to the python http server for the frontend, 
# as the user likely wants to see the visualization.
# But the socket server is needed for the logic.
# A supervisor or simple bash script is best.

COPY start.sh /app/start.sh
RUN chmod +x /app/start.sh

CMD ["/app/start.sh"]
