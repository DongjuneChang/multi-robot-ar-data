# Git Commands for Multi-Robot AR Data Repository

## Repository Information
- **Repository Name**: multi-robot-ar-data
- **URL**: https://github.com/DongjuneChang/multi-robot-ar-data.git
- **Purpose**: Configuration files for multi-robot AR system
- **Created**: 2025-08-21

## Basic Git Commands

### 1. Initial Setup
```bash
# Clone the repository
git clone https://github.com/DongjuneChang/multi-robot-ar-data.git

# Navigate to repository
cd multi-robot-ar-data

# Check remote URL
git remote -v
```

### 2. Daily Workflow
```bash
# Check current status
git status

# Pull latest changes from remote
git pull origin main

# Add all changes
git add .

# Or add specific files
git add map_data.yaml
git add device_config.yaml

# Commit with message
git commit -m "Update device configuration"

# Push to remote
git push origin main
```

### 3. Authentication
```bash
# When prompted for credentials
Username: dongjune.chang@gmail.com
Password: [Your Personal Access Token]  # NOT your GitHub password!

# To save credentials (optional)
git config credential.helper store
```

### 4. Common Operations

#### Update Device Configuration
```bash
cd /mnt/d/Data_AR/dev/multi-robot-ar-data
git add device_config.yaml
git commit -m "Update HoloLens device IP addresses"
git push
```

#### Update Map Data
```bash
cd /mnt/d/Data_AR/dev/multi-robot-ar-data
git add map_data.yaml
git commit -m "Add new QR code registration"
git push
```

#### Update Multiple Files
```bash
cd /mnt/d/Data_AR/dev/multi-robot-ar-data
git add .
git commit -m "Update system configuration files"
git push
```

### 5. Checking History
```bash
# View commit history
git log --oneline -5

# Check what changed
git diff HEAD~1

# Check specific file history
git log map_data.yaml
```

### 6. Troubleshooting

#### Authentication Failed
```bash
# Clear stored credentials
git config --global --unset credential.helper

# Use Personal Access Token instead of password
# Generate at: https://github.com/settings/tokens
```

#### Merge Conflicts
```bash
# Pull with rebase to avoid merge commits
git pull --rebase origin main

# If conflicts occur, resolve them, then:
git add .
git rebase --continue
git push
```

#### Undo Last Commit (before push)
```bash
git reset --soft HEAD~1
# Make changes
git add .
git commit -m "New message"
```

## File Structure
```
multi-robot-ar-data/
├── README.md                # Repository documentation
├── GIT_COMMANDS.md         # This file
├── map_data.yaml           # QR codes and user registrations
├── device_config.yaml      # HoloLens device configurations
├── streaming_config.yaml   # Streaming settings
└── ngrok_config.yml        # ngrok tunnel configuration
```

## Important Files

### map_data.yaml
- Contains QR code registrations
- User device mappings
- Robot spawn configurations
- Camera tracking data

### device_config.yaml
- HoloLens Device Portal credentials
- Device IP addresses
- Streaming quality presets

## Quick Reference

### Check Everything
```bash
cd /mnt/d/Data_AR/dev/multi-robot-ar-data
git status
git log --oneline -3
```

### Update Everything
```bash
cd /mnt/d/Data_AR/dev/multi-robot-ar-data
git pull
git add .
git commit -m "Update configuration files"
git push
```

### Sync with Local Flask App
```bash
# Copy from git repo to Flask app
cp /mnt/d/Data_AR/dev/multi-robot-ar-data/*.yaml \
   /mnt/d/Data_AR/dev/multi_hri2/python/management_app/config/

# Or vice versa
cp /mnt/d/Data_AR/dev/multi_hri2/python/management_app/config/*.yaml \
   /mnt/d/Data_AR/dev/multi-robot-ar-data/
```

## Notes
- Always pull before making changes to avoid conflicts
- Use descriptive commit messages
- Don't commit sensitive information (passwords, tokens)
- The repository is public, so avoid private data

## Contact
- Repository Owner: Dongjune Chang
- Email: dongjune.chang@gmail.com

---
Last Updated: 2025-08-21