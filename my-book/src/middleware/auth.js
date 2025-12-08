// Authentication middleware for the Physical AI & Humanoid Robotics Book platform
const jwt = require('jsonwebtoken');
const UserService = require('../services/userService');

// JWT secret - in production, use environment variable
const JWT_SECRET = process.env.JWT_SECRET || 'your-super-secret-jwt-key-change-in-production';

const authenticateToken = async (req, res, next) => {
  // Get token from header
  const authHeader = req.headers['authorization'];
  const token = authHeader && authHeader.split(' ')[1]; // Bearer TOKEN

  if (!token) {
    return res.status(401).json({
      error: 'Access denied. No token provided.'
    });
  }

  try {
    // Verify token
    const decoded = jwt.verify(token, JWT_SECRET);

    // Get user from database using the ID in the token
    const user = await UserService.findUserById(decoded.userId);

    if (!user) {
      return res.status(401).json({
        error: 'Token is valid but user not found.'
      });
    }

    // Add user to request object
    req.user = user;
    next();
  } catch (error) {
    return res.status(403).json({
      error: 'Invalid token.'
    });
  }
};

// Middleware to check if user is admin
const requireAdmin = async (req, res, next) => {
  if (!req.user) {
    return res.status(401).json({
      error: 'Access denied. User not authenticated.'
    });
  }

  if (req.user.role !== 'admin') {
    return res.status(403).json({
      error: 'Access denied. Admin role required.'
    });
  }

  next();
};

// Middleware to check if user is educator
const requireEducator = async (req, res, next) => {
  if (!req.user) {
    return res.status(401).json({
      error: 'Access denied. User not authenticated.'
    });
  }

  if (req.user.role !== 'educator' && req.user.role !== 'admin') {
    return res.status(403).json({
      error: 'Access denied. Educator or admin role required.'
    });
  }

  next();
};

module.exports = {
  authenticateToken,
  requireAdmin,
  requireEducator
};