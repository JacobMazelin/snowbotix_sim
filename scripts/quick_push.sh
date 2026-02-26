#!/bin/bash
# Quick auto-commit and push script
# Usage: ./scripts/quick_push.sh ["commit message"]

cd "$(dirname "$0")/.." || exit 1

# Get commit message
if [ -z "$1" ]; then
    COMMIT_MSG="Update from VS Code - $(date '+%Y-%m-%d %H:%M:%S')"
else
    COMMIT_MSG="$1"
fi

echo "🚀 Quick Push to GitHub"
echo "======================"
echo ""

# Check if there are changes
if [ -z "$(git status --porcelain)" ]; then
    echo "✓ No changes to commit"
    echo ""
    echo "Pushing anyway to trigger deployment check..."
else
    echo "→ Staging changes..."
    git add -A
    
    echo "→ Committing: $COMMIT_MSG"
    git commit -m "$COMMIT_MSG"
fi

echo "→ Pushing to GitHub..."
git push origin main

echo ""
echo "✅ Pushed successfully!"
echo ""
echo "📊 Check deployment status:"
echo "   https://github.com/JacobMazelin/snowbotix_sim/actions"
echo ""
echo "⏱️  Changes will appear in Foxglove in ~30-60 seconds"
echo ""
echo "🔗 View your repo: https://github.com/JacobMazelin/snowbotix_sim"
