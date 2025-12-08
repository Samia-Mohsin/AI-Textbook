// User service for the Physical AI & Humanoid Robotics Book platform
const User = require('../models/User');
const bcrypt = require('bcryptjs');
const jwt = require('jsonwebtoken');

// In-memory storage for development - in production, use a proper database
let users = [];
let nextId = 1;

// JWT secret - in production, use environment variable
const JWT_SECRET = process.env.JWT_SECRET || 'your-super-secret-jwt-key-change-in-production';

class UserService {
  // Create a new user
  static async createUser(userData) {
    // Check if user with email already exists
    const existingUser = users.find(user => user.email === userData.email);
    if (existingUser) {
      throw new Error('User with this email already exists');
    }

    // Hash the password
    const saltRounds = 10;
    const passwordHash = await bcrypt.hash(userData.password, saltRounds);

    // Create new user instance
    const newUser = new User(
      nextId++,
      userData.email,
      passwordHash,
      userData.name,
      new Date(),
      new Date(),
      userData.role || 'student'
    );

    // Add to users array
    users.push(newUser);

    // Return user without password hash
    const { passwordHash: _, ...userWithoutPassword } = newUser;
    return userWithoutPassword;
  }

  // Find user by email
  static async findUserByEmail(email) {
    return users.find(user => user.email === email);
  }

  // Find user by ID
  static async findUserById(id) {
    return users.find(user => user.id === parseInt(id));
  }

  // Authenticate user (login)
  static async authenticateUser(email, password) {
    const user = await this.findUserByEmail(email);
    if (!user) {
      throw new Error('Invalid email or password');
    }

    const isPasswordValid = await bcrypt.compare(password, user.passwordHash);
    if (!isPasswordValid) {
      throw new Error('Invalid email or password');
    }

    // Return user without password hash and generate JWT
    const { passwordHash: _, ...userWithoutPassword } = user;
    const token = jwt.sign(
      { userId: user.id, email: user.email },
      JWT_SECRET,
      { expiresIn: '24h' }
    );

    return { user: userWithoutPassword, token };
  }

  // Update user progress
  static async updateUserProgress(userId, moduleId, chapterId, status = 'completed') {
    const user = await this.findUserById(userId);
    if (!user) {
      throw new Error('User not found');
    }

    user.updateProgress(moduleId, chapterId, status);
    return user;
  }

  // Update user preferences
  static async updateUserPreferences(userId, preferences) {
    const user = await this.findUserById(userId);
    if (!user) {
      throw new Error('User not found');
    }

    user.updatePreferences(preferences);
    return user;
  }

  // Get all users (admin function)
  static async getAllUsers() {
    return users.map(({ passwordHash: _, ...user }) => user);
  }

  // Update user role (admin function)
  static async updateUserRole(userId, newRole) {
    const user = await this.findUserById(userId);
    if (!user) {
      throw new Error('User not found');
    }

    user.role = newRole;
    user.updatedAt = new Date();
    return user;
  }
}

module.exports = UserService;