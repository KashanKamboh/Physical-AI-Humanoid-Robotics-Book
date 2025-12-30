#!/usr/bin/env python3
"""
Start script for the RAG Chatbot backend service.
This script starts the FastAPI backend with uvicorn.
"""
import os
import subprocess
import sys
from dotenv import load_dotenv

def start_backend():
    """Start the FastAPI backend service."""
    # Load environment variables
    load_dotenv()

    # Check if required environment variables are set (OpenRouter)
    openrouter_key = os.getenv('OPENROUTER_API_KEY')

    if not openrouter_key:
        print("Error: Missing required environment variable. OPENROUTER_API_KEY must be set")
        print("Please set it in a .env file or as an environment variable.")
        sys.exit(1)

    # Start the backend service
    backend_path = os.path.join(os.path.dirname(__file__), "backend", "api.py")

    if not os.path.exists(backend_path):
        print(f"Error: Backend file not found at {backend_path}")
        sys.exit(1)

    print("Starting RAG Chatbot backend service...")
    print(f"Loading configuration from: {backend_path}")

    try:
        # Run uvicorn to serve the FastAPI app
        cmd = [
            sys.executable, "-m", "uvicorn",
            "backend.api:app",
            "--host", "0.0.0.0",
            "--port", str(int(os.getenv("PORT", 8000))),
            "--reload"  # Enable auto-reload for development
        ]

        print(f"Running command: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

    except subprocess.CalledProcessError as e:
        print(f"Error starting backend: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nShutting down backend service...")
        sys.exit(0)

if __name__ == "__main__":
    start_backend()