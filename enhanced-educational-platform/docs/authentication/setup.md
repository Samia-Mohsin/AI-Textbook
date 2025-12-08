---
sidebar_position: 1
---

# Authentication Setup

The Enhanced Educational Platform uses Better-Auth for secure user authentication and management. This system provides enterprise-grade security with support for multiple authentication providers.

## Better-Auth Configuration

### 1. Installation

First, install the required Better-Auth packages:

```bash
npm install better-auth @better-auth/node @better-auth/react
```

### 2. Server-Side Setup

Create the authentication API endpoint:

```javascript
// pages/api/auth/[...betterAuth].js
import { auth } from "../../../src/config/auth";

export default auth;
```

### 3. Client-Side Setup

Initialize the client-side authentication:

```javascript
// src/config/auth-client.js
import { initClient } from "better-auth/client";
import { reactHooks } from "@better-auth/react";

export const client = initClient(auth, {
  baseURL: process.env.NEXT_PUBLIC_SITE_URL || "http://localhost:3000",
});

export const { useAuth } = reactHooks(client);
```

### 4. Environment Variables

Ensure these variables are set in your `.env` file:

```env
# Authentication Secret (generate with: openssl rand -base64 32)
AUTH_SECRET=your-very-long-secret-key-here

# OAuth Providers
GITHUB_ID=your-github-client-id
GITHUB_SECRET=your-github-client-secret
GOOGLE_ID=your-google-client-id
GOOGLE_SECRET=your-google-client-secret

# Database
DATABASE_URL="postgresql://username:password@localhost:5432/educational_platform"
```

## Database Schema

Better-Auth automatically creates the necessary database tables. For enhanced functionality, you can extend the schema:

```sql
-- Extended user profile table
CREATE TABLE user_profiles (
  id SERIAL PRIMARY KEY,
  user_id TEXT NOT NULL REFERENCES better_auth_user(id),
  learning_level VARCHAR(20) DEFAULT 'beginner',
  preferred_language VARCHAR(10) DEFAULT 'en',
  learning_goals TEXT[],
  accessibility_preferences JSONB,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Learning progress tracking
CREATE TABLE learning_progress (
  id SERIAL PRIMARY KEY,
  user_id TEXT NOT NULL REFERENCES better_auth_user(id),
  content_id VARCHAR(100) NOT NULL,
  progress_percentage INTEGER DEFAULT 0,
  time_spent_seconds INTEGER DEFAULT 0,
  completed_at TIMESTAMP,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

## Authentication Components

### 1. Sign In/Up Component

```jsx
// src/components/Auth/AuthForm.jsx
import React, { useState } from 'react';
import { useAuth } from '../../config/auth-client';

const AuthForm = () => {
  const { signIn, signUp } = useAuth();
  const [isSignUp, setIsSignUp] = useState(false);
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    name: ''
  });

  const handleSubmit = async (e) => {
    e.preventDefault();

    try {
      if (isSignUp) {
        await signUp({
          email: formData.email,
          password: formData.password,
          name: formData.name
        });
      } else {
        await signIn({
          email: formData.email,
          password: formData.password
        });
      }
    } catch (error) {
      console.error('Authentication error:', error);
    }
  };

  return (
    <div className="auth-form">
      <form onSubmit={handleSubmit}>
        {isSignUp && (
          <input
            type="text"
            placeholder="Full Name"
            value={formData.name}
            onChange={(e) => setFormData({...formData, name: e.target.value})}
            required={isSignUp}
          />
        )}
        <input
          type="email"
          placeholder="Email"
          value={formData.email}
          onChange={(e) => setFormData({...formData, email: e.target.value})}
          required
        />
        <input
          type="password"
          placeholder="Password"
          value={formData.password}
          onChange={(e) => setFormData({...formData, password: e.target.value})}
          required
        />
        <button type="submit">
          {isSignUp ? 'Sign Up' : 'Sign In'}
        </button>
      </form>

      <button onClick={() => setIsSignUp(!isSignUp)}>
        {isSignUp ? 'Already have an account? Sign In' : "Don't have an account? Sign Up"}
      </button>
    </div>
  );
};

export default AuthForm;
```

### 2. Protected Route Component

```jsx
// src/components/Auth/ProtectedRoute.jsx
import React from 'react';
import { useAuth } from '../../config/auth-client';
import { Navigate } from 'react-router-dom';

const ProtectedRoute = ({ children, requiredRole = null }) => {
  const { session, isPending } = useAuth();

  if (isPending) {
    return <div>Loading...</div>;
  }

  if (!session) {
    return <Navigate to="/signin" />;
  }

  if (requiredRole && session.user.role !== requiredRole) {
    return <div>Access denied. Required role: {requiredRole}</div>;
  }

  return children;
};

export default ProtectedRoute;
```

## Social Authentication

### GitHub OAuth Setup

1. Go to GitHub Developer Settings
2. Create a new OAuth application
3. Set the callback URL to: `http://localhost:3000/api/auth/callback/github`
4. Add the client ID and secret to your environment variables

### Google OAuth Setup

1. Go to Google Cloud Console
2. Create a new project or select existing one
3. Enable the Google+ API
4. Create OAuth 2.0 credentials
5. Add authorized redirect URIs: `http://localhost:3000/api/auth/callback/google`

## User Profile Management

### Profile Update Component

```jsx
// src/components/UserProfile/ProfileEditor.jsx
import React, { useState, useEffect } from 'react';
import { useAuth } from '../../config/auth-client';

const ProfileEditor = () => {
  const { session, update } = useAuth();
  const [profile, setProfile] = useState({
    name: '',
    email: '',
    learningLevel: 'beginner',
    preferredLanguage: 'en',
    learningGoals: []
  });

  useEffect(() => {
    if (session) {
      setProfile({
        name: session.user.name,
        email: session.user.email,
        learningLevel: session.user.profile?.learningLevel || 'beginner',
        preferredLanguage: session.user.profile?.preferredLanguage || 'en',
        learningGoals: session.user.profile?.learningGoals || []
      });
    }
  }, [session]);

  const handleSubmit = async (e) => {
    e.preventDefault();

    try {
      await update({
        name: profile.name,
        profile: {
          learningLevel: profile.learningLevel,
          preferredLanguage: profile.preferredLanguage,
          learningGoals: profile.learningGoals
        }
      });
    } catch (error) {
      console.error('Profile update error:', error);
    }
  };

  return (
    <form onSubmit={handleSubmit}>
      <input
        type="text"
        value={profile.name}
        onChange={(e) => setProfile({...profile, name: e.target.value})}
        placeholder="Name"
      />
      <select
        value={profile.learningLevel}
        onChange={(e) => setProfile({...profile, learningLevel: e.target.value})}
      >
        <option value="beginner">Beginner</option>
        <option value="intermediate">Intermediate</option>
        <option value="advanced">Advanced</option>
      </select>
      <select
        value={profile.preferredLanguage}
        onChange={(e) => setProfile({...profile, preferredLanguage: e.target.value})}
      >
        <option value="en">English</option>
        <option value="ur">اردو</option>
      </select>
      <button type="submit">Update Profile</button>
    </form>
  );
};

export default ProfileEditor;
```

## Security Best Practices

### 1. Session Management

- Sessions are automatically managed by Better-Auth
- Configure session timeout in your auth configuration
- Implement secure cookie settings for production

### 2. Password Policies

```javascript
// src/config/auth.config.js
export const auth = betterAuth({
  // ... other config
  password: {
    enabled: true,
    // Require strong passwords
    requireStrongPassword: true,
    // Minimum length
    minLength: 8,
  },
});
```

### 3. Rate Limiting

Implement rate limiting to prevent abuse:

```javascript
// src/middleware.js
export { auth as middleware } from "./src/config/auth";

export const config = {
  matcher: ["/((?!api|_next/static|_next/image|favicon.ico).*)"],
};
```

## Testing Authentication

### Unit Tests

```javascript
// tests/auth.test.js
import { test, expect } from '@playwright/test';

test('should allow user registration', async ({ page }) => {
  await page.goto('/signup');

  await page.fill('input[name="email"]', 'test@example.com');
  await page.fill('input[name="password"]', 'securePassword123');
  await page.click('button[type="submit"]');

  await expect(page).toHaveURL('/dashboard');
});
```

## Next Steps

After setting up authentication:

1. [Configure user profiles and preferences](./user-profiles.md)
2. [Set up the personalization engine](../personalization/user-preferences.md)
3. [Implement role-based access control](../authorization/rbac.md)

Continue to [User Profiles](./user-profiles.md) for advanced user management features.