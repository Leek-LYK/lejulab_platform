#!/bin/bash
# 把 staging/ 推送到独立的下游 aarch64 仓库
# 参数: $1 = 下游仓库 url (git@host:org/repo.git)
#       $2 = 上游分支名（直接作为下游分支名）
# 关键: 下游仓库的 git 历史与上游 .git 完全独立

set -euo pipefail

DOWNSTREAM_URL="${1:?usage: $0 <downstream-url> <branch>}"
BRANCH="${2:?usage: $0 <downstream-url> <branch>}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(realpath "$SCRIPT_DIR/../..")"
STAGING="$PROJECT_DIR/staging"

[ -d "$STAGING" ] || { echo "[sync] ERROR: $STAGING missing — run package.sh first"; exit 1; }

# ============================================================
# SSH key 设置
# 同时挂 ed25519 与 rsa（GitLab 那边只有 ed25519 注册了 deploy key，
# id_rsa 作为未来 fallback；ssh 会按顺序 offer，找到匹配的即可）
# ============================================================
echo "[sync] setting up ssh agent..."
SSH_KEYS=()
for k in /home/gitlab-runner/.ssh/id_ed25519 /home/gitlab-runner/.ssh/id_rsa; do
    [ -f "$k" ] && SSH_KEYS+=("$k")
done
if [ ${#SSH_KEYS[@]} -eq 0 ]; then
    echo "[sync] ERROR: no SSH key found in /home/gitlab-runner/.ssh/"
    exit 1
fi
eval "$(ssh-agent -s)" >/dev/null
for k in "${SSH_KEYS[@]}"; do
    ssh-add "$k" 2>&1 | tail -1
done
export GIT_SSH_COMMAND="ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no"

# ============================================================
# clone / 初始化下游仓库
# ============================================================
TARGET="/tmp/lejulab-aarch64-downstream-${BRANCH//\//_}"
rm -rf "$TARGET"

echo "[sync] cloning downstream branch '$BRANCH' from $DOWNSTREAM_URL..."
if git clone --branch "$BRANCH" --single-branch "$DOWNSTREAM_URL" "$TARGET" 2>/dev/null; then
    echo "[sync] branch $BRANCH exists, checked out"
else
    echo "[sync] branch $BRANCH does not exist, will create it"
    rm -rf "$TARGET"
    if git clone "$DOWNSTREAM_URL" "$TARGET" 2>/dev/null; then
        cd "$TARGET"
        # 看远端默认分支存不存在，没有则从空 repo init
        if git rev-parse --verify HEAD >/dev/null 2>&1; then
            git checkout -b "$BRANCH" 2>/dev/null || git checkout "$BRANCH"
        else
            # 完全空的仓库
            git checkout -b "$BRANCH"
        fi
        cd - >/dev/null
    else
        # clone 失败（可能是从未推送过的空仓） — 本地初始化
        echo "[sync] remote clone failed, initializing local repo..."
        mkdir -p "$TARGET"
        cd "$TARGET"
        git init
        git checkout -b "$BRANCH"
        git remote add origin "$DOWNSTREAM_URL"
        cd - >/dev/null
    fi
fi

# 用户信息（CI runner 默认可能没配）
cd "$TARGET"
git config user.email "ci@lejulab"
git config user.name "lejulab-ci"

# ============================================================
# rsync 覆盖式同步
#   --delete 保证已删除文件在下游一并消失
#   --exclude='.git'             双保险，绝不让 staging 内任何 .git 残留入侵下游历史
#   --exclude='.gitlab-ci.yml'
#   --exclude='ci_scripts'
#     现在的 package.sh 本就不会把它们写进 staging，但万一未来有人改了脚本
#     把这两条目复制进来了，这里再过滤一道，避免泄漏到公开下游
# ============================================================
echo "[sync] rsync staging -> $TARGET..."
rsync -a --delete \
  --exclude='.git' \
  --exclude='.gitlab-ci.yml' \
  --exclude='ci_scripts' \
  --exclude='ci_scripts/**' \
  "$STAGING/" "$TARGET/"

# ============================================================
# 提交
# ============================================================
cd "$TARGET"
git add -A

if git diff --cached --quiet; then
    echo "[sync] no changes detected, skipping commit & push"
    exit 0
fi

COMMIT_MSG="sync ${CI_PROJECT_URL:-unknown}/commit/${CI_COMMIT_SHA:-$(date -u -Iseconds)}"
git commit -m "$COMMIT_MSG"

# ============================================================
# 强制推送（下游分支是发布快照，由 CI 强写）
# ============================================================
echo "[sync] pushing -f to origin $BRANCH..."
git push -f origin "$BRANCH"

echo "[sync] done. downstream branch '$BRANCH' updated."
