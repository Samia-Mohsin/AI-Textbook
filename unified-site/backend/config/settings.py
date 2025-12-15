import os
from dotenv import load_dotenv
from typing import Optional

# Load environment variables from .env file
load_dotenv()

class Settings:
    # Cohere Configuration
    COHERE_API_KEY: str = os.getenv("COHERE_API_KEY", "")
    COHERE_MODEL: str = os.getenv("COHERE_MODEL", "embed-multilingual-v3.0")

    # Qdrant Configuration
    QDRANT_URL: str = os.getenv("QDRANT_URL", "http://localhost:6333")
    QDRANT_API_KEY: Optional[str] = os.getenv("QDRANT_API_KEY")
    QDRANT_COLLECTION_NAME: str = os.getenv("QDRANT_COLLECTION_NAME", "book_content")

    # Content Processing Configuration
    CHUNK_SIZE: int = int(os.getenv("CHUNK_SIZE", "512"))
    CHUNK_OVERLAP: int = int(os.getenv("CHUNK_OVERLAP", "50"))
    MAX_CONTENT_SIZE: int = int(os.getenv("MAX_CONTENT_SIZE", "100000"))

    # Crawling Configuration
    CRAWL_DELAY: float = float(os.getenv("CRAWL_DELAY", "1.0"))
    REQUEST_TIMEOUT: int = int(os.getenv("REQUEST_TIMEOUT", "30"))
    MAX_RETRIES: int = int(os.getenv("MAX_RETRIES", "3"))

    # Validation
    def validate(self):
        """Validate that all required settings are present"""
        errors = []

        if not self.COHERE_API_KEY:
            errors.append("COHERE_API_KEY is required")

        if not self.QDRANT_URL:
            errors.append("QDRANT_URL is required")

        if errors:
            raise ValueError(f"Configuration validation failed: {'; '.join(errors)}")

# Global settings instance
settings = Settings()