---
sidebar_position: 1
---

# Installation

This guide will walk you through setting up the Enhanced Educational Platform on your local development environment.

## Prerequisites

Before you begin, ensure you have the following installed:

- **Node.js**: Version 18.0 or higher
- **npm**: Version 8.0 or higher (or Yarn v1.22+)
- **Git**: Version control system
- **A code editor** (VS Code recommended)

## Quick Setup

The fastest way to get started is using the automated setup script:

```bash
# Clone the repository
git clone https://github.com/your-username/enhanced-educational-platform.git
cd enhanced-educational-platform

# Install dependencies
npm install

# Create environment file
cp .env.example .env

# Start the development server
npm run dev
```

Your platform will be available at `http://localhost:3000`.

## Manual Setup

### 1. Environment Configuration

Create a `.env` file in the root directory with the following structure:

```env
# Better-Auth Configuration
AUTH_SECRET=your-secret-key-here
GITHUB_ID=your-github-client-id
GITHUB_SECRET=your-github-client-secret
GOOGLE_ID=your-google-client-id
GOOGLE_SECRET=your-google-client-secret

# Database Configuration
DATABASE_URL="postgresql://username:password@localhost:5432/educational_platform"

# AI Services
OPENAI_API_KEY=your-openai-api-key
PINECONE_API_KEY=your-pinecone-api-key
PINECONE_INDEX_NAME=educational-content

# Supabase Configuration
SUPABASE_URL=your-supabase-url
SUPABASE_KEY=your-supabase-anon-key

# Application Settings
NEXT_PUBLIC_SITE_URL=http://localhost:3000
```

### 2. Database Setup

The platform uses PostgreSQL for user profiles and personalization data:

```bash
# Using Prisma (recommended)
npx prisma db push
npx prisma generate
```

### 3. Vector Database Setup

For AI-powered content retrieval, set up Pinecone:

```bash
# Create an index in Pinecone named 'educational-content'
# with appropriate dimensions for your embedding model
```

### 4. Initialize Better-Auth

Create the authentication configuration file:

```javascript
// src/config/auth.config.js
import { betterAuth } from "better-auth";
import { postgresAdapter } from "@better-auth/postgres-adapter";
import { prisma } from "./prisma";

export const auth = betterAuth({
  database: postgresAdapter(prisma, {
    provider: "pg",
  }),
  socialProviders: {
    github: {
      clientId: process.env.GITHUB_ID,
      clientSecret: process.env.GITHUB_SECRET,
    },
    google: {
      clientId: process.env.GOOGLE_ID,
      clientSecret: process.env.GOOGLE_SECRET,
    },
  },
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: true,
  },
  account: {
    accountModel: {
      create: {
        includeSocialAccounts: true,
      },
    },
  },
});
```

### 5. Run the Development Server

```bash
# Start the Docusaurus development server
npm run dev

# Or with environment variables
npx docusaurus start
```

## Configuration Files

### Custom Components

The platform includes several custom components for enhanced functionality:

```javascript
// src/components/UserProfile/UserProfile.jsx
import React from 'react';
import { useAuth } from '@better-auth/react';

const UserProfile = () => {
  const { session } = useAuth();

  if (!session) {
    return <div>Please sign in to view your profile</div>;
  }

  return (
    <div className="user-profile">
      <h2>Welcome, {session.user.name}!</h2>
      <p>Email: {session.user.email}</p>
      <p>Learning Level: {session.user.profile.learningLevel || 'Beginner'}</p>
    </div>
  );
};

export default UserProfile;
```

## Troubleshooting

### Common Issues

**Issue**: `Error: Cannot find module 'better-auth'`
**Solution**: Run `npm install` to install all dependencies

**Issue**: `Database connection failed`
**Solution**: Verify your `DATABASE_URL` in the `.env` file

**Issue**: Authentication not working
**Solution**: Ensure OAuth provider credentials are correctly configured

## Next Steps

After successful installation:

1. [Configure user authentication](../authentication/setup.md)
2. [Set up personalization engine](../personalization/user-preferences.md)
3. [Enable AI automation](../ai-automation/subagents.md)
4. [Add multilingual support](../localization/multilingual.md)

Continue to the [Configuration Guide](./configuration.md) for advanced setup options.