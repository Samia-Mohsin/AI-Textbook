"""
Vercel entry point for NeuralReader API
This file serves as the entry point for Vercel deployment
"""
from main import app

# This ensures that Vercel can properly import and run the application
# The app instance is imported from main.py and will be used by Vercel
api = app

# For Vercel Python runtime, we need to make sure the app is available as 'api'
# Vercel will automatically detect and run this when deployed