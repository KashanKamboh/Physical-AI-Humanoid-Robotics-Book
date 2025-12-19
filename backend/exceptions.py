"""
Custom exception classes for the RAG Ingestion Pipeline
"""


class CohereAPIError(Exception):
    """Exception raised when Cohere API returns an error."""
    def __init__(self, message: str, status_code: int = None):
        self.message = message
        self.status_code = status_code
        super().__init__(self.message)


class QdrantError(Exception):
    """Exception raised when Qdrant operations fail."""
    def __init__(self, message: str):
        self.message = message
        super().__init__(self.message)


class ValidationError(Exception):
    """Exception raised when input validation fails."""
    def __init__(self, message: str):
        self.message = message
        super().__init__(self.message)


class RequestError(Exception):
    """Exception raised when HTTP requests fail."""
    def __init__(self, message: str, status_code: int = None):
        self.message = message
        self.status_code = status_code
        super().__init__(self.message)


class ParseError(Exception):
    """Exception raised when HTML parsing fails."""
    def __init__(self, message: str):
        self.message = message
        super().__init__(self.message)


class ContentError(Exception):
    """Exception raised when content extraction fails."""
    def __init__(self, message: str):
        self.message = message
        super().__init__(self.message)


class RateLimitError(Exception):
    """Exception raised when API rate limits are exceeded."""
    def __init__(self, message: str = "API rate limit exceeded"):
        self.message = message
        super().__init__(self.message)