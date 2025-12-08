---
sidebar_position: 1
---

# Introduction

Welcome to the Enhanced Educational Platform! This platform represents the next generation of educational technology, combining:

- **Advanced Personalization**: Content adapts to each learner's profile, preferences, and progress
- **AI-Powered Automation**: Claude Code subagents handle content generation and management
- **Secure Authentication**: Better-Auth provides enterprise-grade user management
- **Multilingual Support**: Full Urdu translation with cultural adaptation
- **Intelligent Learning**: Adaptive algorithms optimize the learning experience

## Key Features

### Personalization Engine
- Chapter-level content adaptation
- Skill-based learning paths
- Progress tracking and analytics
- User preference management

### AI Automation
- Claude Code subagents for content generation
- Automated quality assurance
- Translation validation and improvement
- Personalization algorithm optimization

### Authentication & User Management
- Multi-provider OAuth (Google, GitHub, Microsoft)
- Role-based access control
- Learning profile tracking
- Privacy-compliant data handling

### Multilingual Support
- Urdu translation with RTL layout support
- Dynamic language switching
- Cultural content adaptation
- Professional localization

## Getting Started

To begin using the Enhanced Educational Platform, follow these steps:

1. **Set up your development environment** by following the [Installation Guide](./getting-started/installation.md)
2. **Configure authentication** with [Better-Auth](./authentication/setup.md)
3. **Customize personalization settings** in the [User Preferences](./personalization/user-preferences.md) section
4. **Explore AI automation capabilities** with our [Subagents Framework](./ai-automation/subagents.md)
5. **Enable multilingual support** following the [Localization Guide](./localization/multilingual.md)

## Architecture Overview

The platform follows a modern, scalable architecture:

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Frontend      │    │   Backend       │    │     AI/ML       │
│   (Docusaurus)  │◄──►│   (Node.js)     │◄──►│   (Claude)      │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Authentication │    │   Database      │    │   Vector Store  │
│   (Better-Auth) │    │  (PostgreSQL)   │    │  (Pinecone)     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

Ready to get started? Continue with the [Installation Guide](./getting-started/installation.md).