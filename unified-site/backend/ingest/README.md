# Content Ingestion Pipeline for RAG Chatbot

This directory contains the implementation of the content ingestion pipeline that powers the RAG (Retrieval-Augmented Generation) chatbot system.

## Overview

The ingestion pipeline performs the following steps:
1. **Crawling**: Crawls specified URLs to extract content from Docusaurus-based websites
2. **Processing**: Cleans and chunks the extracted content into semantic segments
3. **Embedding**: Generates vector embeddings using Cohere's embedding models
4. **Storage**: Stores the embeddings in Qdrant vector database with metadata
5. **Indexing**: Makes content searchable for the RAG chatbot

## Components

### Core Services
- `content_extraction.py`: Web crawling and content extraction utilities
- `text_chunking.py`: Text chunking with configurable size and overlap
- `embedding_service.py`: Cohere API integration for embedding generation
- `vector_storage.py`: Qdrant vector database operations
- `ingestion_pipeline.py`: Main orchestration pipeline

### Scripts
- `run_ingestion.py`: Command-line script to run the ingestion pipeline
- `test_ingestion.py`: Validation tests for the pipeline components

## Configuration

The pipeline uses the following environment variables:

```bash
# Cohere Configuration
COHERE_API_KEY=your_cohere_api_key_here
COHERE_MODEL=embed-multilingual-v3.0  # Default model

# Qdrant Configuration
QDRANT_URL=http://localhost:6333  # Or your Qdrant Cloud URL
QDRANT_API_KEY=your_qdrant_api_key  # If using authentication
QDRANT_COLLECTION_NAME=book_content  # Collection name

# Content Processing Configuration
CHUNK_SIZE=512  # Size of text chunks in characters
CHUNK_OVERLAP=50  # Overlap between chunks
MAX_CONTENT_SIZE=100000  # Maximum content size to process

# Crawling Configuration
CRAWL_DELAY=1.0  # Delay between requests in seconds
REQUEST_TIMEOUT=30  # Request timeout in seconds
MAX_RETRIES=3  # Number of retry attempts
```

## Usage

### Running the Ingestion Pipeline

```bash
cd unified-site/backend
python scripts/run_ingestion.py --urls https://yoursite.com/docs https://yoursite.com/tutorial
```

Additional options:
```bash
# Specify maximum pages to crawl per URL
python scripts/run_ingestion.py --urls https://yoursite.com/docs --max-pages 50

# Run incremental update (only new/changed content)
python scripts/run_ingestion.py --urls https://yoursite.com/docs --incremental

# Clear existing vectors before ingestion
python scripts/run_ingestion.py --urls https://yoursite.com/docs --clear
```

### Running Tests

```bash
cd unified-site/backend
python scripts/test_ingestion.py
```

## Features

- **Smart Crawling**: Respects robots.txt and crawl delays
- **Content Extraction**: Extracts main content while filtering out navigation, ads, etc.
- **Semantic Chunking**: Maintains sentence boundaries and paragraph structure
- **Duplicate Detection**: Prevents re-indexing of unchanged content
- **Incremental Updates**: Only processes new or modified content
- **Error Handling**: Comprehensive retry logic and error reporting
- **Progress Tracking**: Detailed logging of ingestion progress

## Architecture

```
[URLs] → [Crawler] → [Content Extractor] → [Text Chunker] → [Cohere Embedding] → [Qdrant Storage]
```

The pipeline is designed to be idempotent - running it multiple times will only process new or changed content, making it safe to re-run.

## Integration with RAG System

The stored vectors are used by the RAG chatbot to:
- Find relevant content based on user queries
- Provide context to the language model
- Generate accurate, source-based responses
- Include proper citations to original content