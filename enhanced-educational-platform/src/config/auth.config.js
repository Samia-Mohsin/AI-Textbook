import { betterAuth } from "better-auth";
import { prismaAdapter } from "@better-auth/prisma-adapter";
import { prisma } from "../lib/prisma";

export const auth = betterAuth({
  database: prismaAdapter(prisma, {
    provider: "postgresql", // or "mysql", "sqlite"
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
    sendEmailVerificationOnSignUp: true,
    async sendVerificationEmail(user, url) {
      // Custom email verification logic
      console.log(`Verification email sent to ${user.email} with URL: ${url}`);
    },
  },
  account: {
    accountModel: {
      create: {
        includeSocialAccounts: true,
      },
    },
  },
  session: {
    expiresIn: 7 * 24 * 60 * 60, // 7 days
    updateAge: 24 * 60 * 60, // 24 hours
  },
  secret: process.env.AUTH_SECRET,
  baseURL: process.env.NEXT_PUBLIC_SITE_URL,
  email: {
    enabled: true,
    from: "noreply@enhanced-educational-platform.com",
  },
  // Custom user fields for educational platform
  user: {
    additionalFields: {
      profile: {
        type: "JSON",
        defaultValue: {
          learningLevel: "beginner",
          preferredLanguage: "en",
          learningGoals: [],
          accessibilityPreferences: {},
          learningHistory: [],
          achievements: []
        }
      },
      role: {
        type: "string",
        defaultValue: "student"
      }
    }
  }
});