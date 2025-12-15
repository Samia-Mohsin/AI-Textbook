#!/usr/bin/env python3
"""
Script to run the content ingestion pipeline for the RAG chatbot.
"""

import argparse
import sys
import os
import logging
from typing import List

# Add the backend directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from config.settings import settings
from ingest.ingestion_pipeline import IngestionPipeline

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def main():
    parser = argparse.ArgumentParser(description='Run the content ingestion pipeline for RAG chatbot')
    parser.add_argument('--urls', nargs='+', required=True, help='URLs to crawl and index')
    parser.add_argument('--max-pages', type=int, default=100, help='Maximum number of pages to crawl per URL')
    parser.add_argument('--incremental', action='store_true', help='Run incremental update instead of full ingestion')
    parser.add_argument('--clear', action='store_true', help='Clear existing vectors before ingestion')

    args = parser.parse_args()

    # Validate configuration
    try:
        settings.validate()
        logger.info("Configuration validated successfully")
    except ValueError as e:
        logger.error(f"Configuration validation failed: {e}")
        sys.exit(1)

    # Create ingestion pipeline
    pipeline = IngestionPipeline()

    # Clear collection if requested
    if args.clear:
        logger.info("Clearing existing vectors from collection...")
        if pipeline.vector_storage.clear_collection():
            logger.info("Collection cleared successfully")
        else:
            logger.error("Failed to clear collection")
            sys.exit(1)

    # Run ingestion
    try:
        if args.incremental:
            logger.info("Running incremental update...")
            results = pipeline.incremental_update(
                urls=args.urls,
                max_pages=args.max_pages
            )
        else:
            logger.info("Running full ingestion...")
            results = pipeline.run_ingestion(
                urls=args.urls,
                max_pages=args.max_pages
            )

        # Print results
        logger.info("=== INGESTION RESULTS ===")
        logger.info(f"Status: {results['status']}")
        logger.info(f"URLs processed: {results['urls_processed']}")
        logger.info(f"Pages crawled: {results['pages_crawled']}")
        logger.info(f"Chunks created: {results['chunks_created']}")
        logger.info(f"Embeddings generated: {results['embeddings_generated']}")
        logger.info(f"Vectors stored: {results['vectors_stored']}")

        if results['errors']:
            logger.error(f"Errors occurred: {len(results['errors'])}")
            for error in results['errors']:
                logger.error(f"  - {error}")

        # Exit with error code if there were errors
        if results['status'] == 'failed':
            sys.exit(1)
        elif 'error' in results['status'] or results['errors']:
            sys.exit(2)  # Partial success with errors

    except Exception as e:
        logger.error(f"Fatal error running ingestion: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()