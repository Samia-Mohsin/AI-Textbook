#!/usr/bin/env python3
"""
Simple test script to validate the ingestion pipeline components
"""

import sys
import os
import logging

# Add the backend directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from config.settings import settings
from services.content_extraction import ContentExtractor, SiteCrawler
from services.text_chunking import TextChunker
from services.embedding_service import CohereEmbeddingService
from services.vector_storage import QdrantVectorStorage
from ingest.ingestion_pipeline import IngestionPipeline

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def test_configuration():
    """Test that configuration is properly loaded"""
    logger.info("Testing configuration...")
    try:
        settings.validate()
        logger.info("✓ Configuration validation passed")
        return True
    except ValueError as e:
        logger.error(f"✗ Configuration validation failed: {e}")
        return False

def test_content_extraction():
    """Test content extraction functionality"""
    logger.info("Testing content extraction...")
    try:
        extractor = ContentExtractor()

        # Test with a simple HTML snippet
        sample_html = """
        <html>
        <head><title>Test Page</title></head>
        <body>
            <h1>Main Title</h1>
            <p>This is a test paragraph with some content to extract.</p>
            <p>Here's another paragraph for testing purposes.</p>
            <div class="content">
                <h2>Section Title</h2>
                <p>Content in a section div.</p>
            </div>
        </body>
        </html>
        """

        result = extractor.extract_content(sample_html, "https://example.com/test")

        assert result['title'] == "Test Page"
        assert "test paragraph" in result['content']
        assert len(result['paragraphs']) > 0

        logger.info("✓ Content extraction test passed")
        return True
    except Exception as e:
        logger.error(f"✗ Content extraction test failed: {e}")
        return False

def test_text_chunking():
    """Test text chunking functionality"""
    logger.info("Testing text chunking...")
    try:
        chunker = TextChunker(chunk_size=100, overlap=20)

        # Test content that will be chunked
        test_content = "This is a test sentence. " * 50  # Creates content longer than chunk size

        # Create a mock paragraph structure
        paragraphs = [
            {'text': test_content, 'type': 'p', 'level': None}
        ]

        chunks = chunker.chunk_by_paragraphs(paragraphs, "https://example.com/test", "Test Title")

        assert len(chunks) > 0
        assert all(len(chunk.content) <= 105 for chunk in chunks)  # Allow slight overrun

        logger.info(f"✓ Text chunking test passed - created {len(chunks)} chunks")
        return True
    except Exception as e:
        logger.error(f"✗ Text chunking test failed: {e}")
        return False

def test_services_instantiation():
    """Test that services can be instantiated (without calling external APIs)"""
    logger.info("Testing service instantiation...")
    try:
        # Test configuration
        settings.validate()

        # Test content extraction service
        extractor = ContentExtractor()
        crawler = SiteCrawler()

        # Test chunking service
        chunker = TextChunker()

        # Test vector storage service (this will try to connect to Qdrant)
        try:
            vector_storage = QdrantVectorStorage()
            logger.info("✓ Vector storage service instantiated and connected to Qdrant")
        except Exception as e:
            logger.warning(f"⚠ Vector storage service connection failed (expected if Qdrant not running): {e}")

        logger.info("✓ Service instantiation test passed")
        return True
    except Exception as e:
        logger.error(f"✗ Service instantiation test failed: {e}")
        return False

def test_pipeline_creation():
    """Test that ingestion pipeline can be created"""
    logger.info("Testing pipeline creation...")
    try:
        # This will fail if required services can't be instantiated
        pipeline = IngestionPipeline()
        status = pipeline.get_pipeline_status()

        logger.info(f"✓ Pipeline created successfully: {status}")
        return True
    except Exception as e:
        logger.error(f"✗ Pipeline creation test failed: {e}")
        return False

def main():
    """Run all tests"""
    logger.info("Starting ingestion pipeline validation tests...")

    tests = [
        ("Configuration", test_configuration),
        ("Content Extraction", test_content_extraction),
        ("Text Chunking", test_text_chunking),
        ("Service Instantiation", test_services_instantiation),
        ("Pipeline Creation", test_pipeline_creation),
    ]

    results = []
    for test_name, test_func in tests:
        logger.info(f"\n--- Running {test_name} Test ---")
        success = test_func()
        results.append((test_name, success))

    # Summary
    logger.info("\n" + "="*50)
    logger.info("TEST RESULTS SUMMARY:")
    logger.info("="*50)

    passed = 0
    total = len(results)

    for test_name, success in results:
        status = "PASS" if success else "FAIL"
        logger.info(f"{test_name}: {status}")
        if success:
            passed += 1

    logger.info(f"\nOverall: {passed}/{total} tests passed")

    if passed == total:
        logger.info("🎉 All tests passed! The ingestion pipeline is ready for use.")
        return 0
    else:
        logger.info("⚠ Some tests failed. Please review the errors above.")
        return 1

if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)