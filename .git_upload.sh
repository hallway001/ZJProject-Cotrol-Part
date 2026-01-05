#!/bin/bash
# Git upload script for loader_control project
# Usage: ./git_upload.sh <github_repo_url>

if [ -z "$1" ]; then
    echo "Usage: ./git_upload.sh <github_repo_url>"
    echo "Example: ./git_upload.sh https://github.com/username/repo_name.git"
    exit 1
fi

REPO_URL=$1

echo "Setting up remote repository..."
git remote add origin $REPO_URL 2>/dev/null || git remote set-url origin $REPO_URL

echo "Fetching from remote..."
git fetch origin

echo "Checking if remote has commits..."
if git rev-parse --verify origin/main >/dev/null 2>&1; then
    echo "Remote repository has commits. Force pushing to replace all content..."
    echo "WARNING: This will DELETE all existing content in the remote repository!"
    read -p "Are you sure? (yes/no): " confirm
    if [ "$confirm" = "yes" ]; then
        git push -f origin main
        echo "Successfully pushed to GitHub!"
    else
        echo "Operation cancelled."
        exit 1
    fi
else
    echo "Remote repository is empty. Pushing normally..."
    git push -u origin main
    echo "Successfully pushed to GitHub!"
fi

