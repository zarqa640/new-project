# Deployment Guide for Physical AI Humanoid Robotics

This guide will help you deploy your Docusaurus site to Vercel.

## Prerequisites

- A GitHub account with this repository
- A Vercel account (sign up at https://vercel.com)
- The Vercel CLI installed (optional): `npm install -g vercel`

## Deploying to Vercel

### Option 1: Using the Vercel Dashboard

1. Go to https://vercel.com and sign in
2. Click "Add New Project"
3. Select "Import Git Repository"
4. Find and select your `new-project` repository
5. Configure your project settings:
   - **FRAMEWORK PRESET**: Select "Other"
   - **ROOT DIRECTORY**: Set to `/` (root)
   - **BUILD COMMAND**: `npm run build` or `yarn build`
   - **OUTPUT DIRECTORY**: `build`
   - **INSTALL COMMAND**: `npm install` or `yarn install`
6. Click "Deploy"

### Option 2: Using the Vercel CLI

1. Install the Vercel CLI: `npm install -g vercel`
2. Navigate to your project directory
3. Run: `vercel`
4. Follow the prompts, ensuring you set:
   - Build command: `npm run build`
   - Output directory: `build`
   - Install command: `npm install`

### Option 3: Using GitHub Integration

1. Connect your GitHub account to Vercel
2. Import your repository
3. Vercel will automatically detect it's a Docusaurus project
4. The `vercel.json` file in this repository provides the necessary configuration

## Configuration Files

This repository includes:

- `vercel.json` - Configuration for Vercel deployment (corrected to avoid route conflicts)
- `docusaurus.config.js` - Docusaurus configuration
- `package.json` - Project dependencies and scripts
- `sidebars.js` - Navigation sidebar configuration

## Important Notes

1. The Docusaurus site will be built using the `npm run build` command
2. The output will be in the `build` directory
3. Vercel will serve the static files from the `build` directory
4. Make sure all dependencies are properly listed in `package.json`

## Troubleshooting

**Issue**: "No config file found in site dir"
- **Solution**: Make sure you're pointing Vercel to the root directory, not the build directory
- Ensure the build command is set to `npm run build` or `yarn build`

**Issue**: Build fails
- **Solution**: Check that all dependencies are properly specified in package.json
- Ensure Node.js version requirements are met (>=20.0 as specified in package.json)

**Issue**: If you encounter route configuration errors like "If `rewrites`, `redirects`, `headers`, `cleanUrls` or `trailingSlash` are used, then `routes` cannot be present"
- **Solution**: This has been fixed in the current `vercel.json` configuration

## Environment Variables

If needed, you can add environment variables in the Vercel dashboard under Settings > Environment Variables.

## Custom Domain

To add a custom domain:
1. Go to your project settings in Vercel
2. Navigate to "Domains"
3. Add your custom domain and follow the instructions to update DNS settings