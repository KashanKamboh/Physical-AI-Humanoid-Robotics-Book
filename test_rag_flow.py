#!/usr/bin/env python3
"""
Test script for RAG Chatbot flow.
This script tests the complete RAG flow: query -> semantic retrieval -> agent response.
"""
import requests
import json
import sys
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

def test_rag_flow():
    """Test the RAG flow with sample queries."""
    # Backend URL - adjust this based on where your backend is running
    backend_url = os.getenv("BACKEND_URL", "http://localhost:8000")

    # Sample test queries
    test_queries = [
        "What is the robotic nervous system?",
        "Explain ROS 2 in the context of this course",
        "What are the four modules in this course?",
        "Tell me about NVIDIA Isaac",
        "What is the capstone project?"
    ]

    print(f"Testing RAG flow against backend: {backend_url}")
    print("=" * 60)

    for i, query in enumerate(test_queries, 1):
        print(f"\nTest {i}: {query}")
        print("-" * 40)

        try:
            # Make request to the backend
            response = requests.post(
                f"{backend_url}/query",
                json={
                    "query": query,
                    "top_k": 3,
                    "min_score": 0.1,
                    "max_tokens": 1000,
                    "temperature": 0.7
                },
                headers={"Content-Type": "application/json"},
                timeout=30
            )

            if response.status_code == 200:
                result = response.json()
                print(f"Response: {result['response'][:200]}...")
                print(f"Retrieved chunks: {result['retrieved_chunks']}")
                print(f"Sources: {len(result['sources'])} source(s) found")

                # Show source information if available
                if result['sources']:
                    print("Top source:")
                    top_source = result['sources'][0]
                    print(f"  Title: {top_source.get('title', 'N/A')}")
                    print(f"  Section: {top_source.get('section', 'N/A')}")
                    print(f"  Score: {top_source.get('similarity_score', 'N/A'):.3f}")
            else:
                print(f"Error: {response.status_code} - {response.text}")

        except requests.exceptions.RequestException as e:
            print(f"Request failed: {str(e)}")
        except json.JSONDecodeError:
            print(f"Invalid JSON response: {response.text}")
        except Exception as e:
            print(f"Unexpected error: {str(e)}")

    print("\n" + "=" * 60)
    print("RAG flow testing completed.")

def test_health_check():
    """Test the health check endpoint."""
    backend_url = os.getenv("BACKEND_URL", "http://localhost:8000")

    try:
        response = requests.get(f"{backend_url}/health", timeout=10)
        if response.status_code == 200:
            health_data = response.json()
            print(f"Health check: {health_data['status']}")
            if health_data['status'] == 'healthy':
                print("All systems operational")
                return True
            else:
                print(f"Issues detected: {health_data.get('details', {})}")
                return False
        else:
            print(f"Health check failed: {response.status_code}")
            return False
    except Exception as e:
        print(f"Health check error: {str(e)}")
        return False

if __name__ == "__main__":
    print("RAG Chatbot - System Test")
    print("=" * 60)

    # Test health first
    health_ok = test_health_check()

    if health_ok:
        print("\nProceeding with RAG flow tests...")
        test_rag_flow()
    else:
        print("\nHealth check failed. Please ensure the backend is running.")
        sys.exit(1)