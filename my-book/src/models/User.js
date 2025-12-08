// User model for the Physical AI & Humanoid Robotics Book platform
// This is a placeholder for the actual user model implementation
// The actual implementation will depend on the database choice (file-based, PostgreSQL, etc.)

class User {
  constructor(id, email, passwordHash, name, createdAt, updatedAt, role = 'student') {
    this.id = id;
    this.email = email;
    this.passwordHash = passwordHash; // Should be a securely hashed password
    this.name = name;
    this.createdAt = createdAt || new Date();
    this.updatedAt = updatedAt || new Date();
    this.role = role; // 'student', 'educator', 'admin'
    this.progress = {}; // Track progress through modules/chapters
    this.preferences = {}; // User preferences for personalization
  }

  // Method to update user preferences
  updatePreferences(newPreferences) {
    this.preferences = { ...this.preferences, ...newPreferences };
    this.updatedAt = new Date();
  }

  // Method to update progress for a specific module/chapter
  updateProgress(moduleId, chapterId, status = 'completed') {
    if (!this.progress[moduleId]) {
      this.progress[moduleId] = {};
    }
    this.progress[moduleId][chapterId] = {
      status,
      completedAt: new Date(),
      lastAccessed: new Date()
    };
    this.updatedAt = new Date();
  }

  // Method to get user's progress for a specific module
  getModuleProgress(moduleId) {
    return this.progress[moduleId] || {};
  }

  // Method to get overall progress percentage
  getOverallProgress() {
    const modules = Object.keys(this.progress);
    if (modules.length === 0) return 0;

    let completedChapters = 0;
    let totalChapters = 0;

    modules.forEach(moduleId => {
      const moduleProgress = this.progress[moduleId];
      const chapters = Object.keys(moduleProgress);
      totalChapters += chapters.length;

      chapters.forEach(chapterId => {
        if (moduleProgress[chapterId].status === 'completed') {
          completedChapters++;
        }
      });
    });

    return totalChapters > 0 ? Math.round((completedChapters / totalChapters) * 100) : 0;
  }
}

module.exports = User;