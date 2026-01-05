@echo off
REM Git upload script for Windows
REM Usage: .git_upload.bat <github_repo_url>

if "%1"=="" (
    echo Usage: .git_upload.bat ^<github_repo_url^>
    echo Example: .git_upload.bat https://github.com/username/repo_name.git
    exit /b 1
)

set REPO_URL=%1

echo Setting up remote repository...
git remote add origin %REPO_URL% 2>nul || git remote set-url origin %REPO_URL%

echo Fetching from remote...
git fetch origin

echo Checking if remote has commits...
git rev-parse --verify origin/main >nul 2>&1
if %errorlevel%==0 (
    echo Remote repository has commits. Force pushing to replace all content...
    echo WARNING: This will DELETE all existing content in the remote repository!
    set /p confirm="Are you sure? (yes/no): "
    if /i "%confirm%"=="yes" (
        git push -f origin main
        echo Successfully pushed to GitHub!
    ) else (
        echo Operation cancelled.
        exit /b 1
    )
) else (
    echo Remote repository is empty. Pushing normally...
    git push -u origin main
    echo Successfully pushed to GitHub!
)

