#!/bin/bash
SCRIPT_DIR=$(dirname "$(realpath "$0")")
CI_SCRIPTS_DIR=$(realpath "$SCRIPT_DIR/../")
PROJECT_DIR=$(realpath "$CI_SCRIPTS_DIR/../") # project: lejulab_platform

# Check if required parameters are provided
if [ $# -ne 3 ]; then
    echo -e "\033[31mUsage: $0 <lejulab_repo_url> <lejulab_repo_dir> <branch>\033[0m"
    echo -e "\033[31mInvalid number of arguments\033[0m"
    exit 1
fi

LEJULAB_REPO_URL="$1"
LEJULAB_REPO_DIR="$2"
LEJULAB_BRANCH="$3"

###############################################
# Helper functions
###############################################
echo_warn() {
    echo -e "\033[33m$1\033[0m"  # Yellow text
}

echo_success() {
    echo -e "\033[32m$1\033[0m"  # Green text
}

echo_error() {
    echo -e "\033[31m$1\033[0m"  # Red text
}

clear_up() {
    echo_success "Cleaning up..."

    # Remove the installed directory if it exists
    if [ -d "$PROJECT_DIR/installed_for_lejulab" ]; then
        rm -rf "$PROJECT_DIR/installed_for_lejulab"
        echo "Removed installed_for_lejulab directory"
    fi

    # Check if catkin build was run before (look for build directory)
    if [ -d "$PROJECT_DIR/build" ]; then
        echo "Previous catkin build detected, running catkin clean..."
        cd "$PROJECT_DIR"
        catkin clean -y
    fi
    # git reset...
}

exit_with_failure() {
    echo_error "❌ Error: $1"
    clear_up
    exit 1
}
######################################################

restruct_lejulab_lejusdk() {
    local installed_dir=$1
    local target_repo_dir=$2
    local target_dir="$target_repo_dir/src/lejusdk"

    # remove old files
    rm -rf "$target_dir"

    # source packages 以源码的pacakge同步
    source_packages=(
        "lejusdk-utils"
        "examples"
    )
    # library packages 以库的package同步
    library_packages=(
        "lejusdk-lowlevel"
        "lejusdk-vr"
    )

    # copy source packages
    for package in "${source_packages[@]}"; do
        echo ">>>> Copy $package to $target_dir/$package"
        local package_dir="$PROJECT_DIR/src/lejusdk/$package"
        mkdir -p "$target_dir/$package"
        cp -r "$package_dir/" "$target_dir/"
    done

    # copy library packages
    for package in "${library_packages[@]}"; do
        rm -rf "$target_dir/$package"
        local pkg_header_dir="$target_dir/$package/include"
        local pkg_lib_dir="$target_dir/$package/lib"
        local so_name="lib$package.so"

        mkdir -p "$pkg_header_dir"
        mkdir -p "$pkg_lib_dir"

        echo ">>>> Copy $package to $pkg_header_dir/$package"
        mv "$installed_dir/include/$package" "$pkg_header_dir" || exit_with_failure "Failed to move $package to $pkg_header_dir"
        echo ">>>> Copy $so_name to $pkg_lib_dir"
        cp "$installed_dir/lib/$so_name" "$pkg_lib_dir" || exit_with_failure "Failed to move $so_name to $pkg_lib_dir"
        # catkin pacakge files
        cp "${PROJECT_DIR}/src/lejusdk/$package/CMakeLists.txt.lejulab" "$target_dir/$package/CMakeLists.txt"
        cp "${PROJECT_DIR}/src/lejusdk/$package/package.xml.lejulab" "$target_dir/$package/package.xml"
    done

    cp ${CI_SCRIPTS_DIR}/.DONT_EDIT "$target_dir"
}

restruct_lejulab_installed() {
    local installed_dir=$1
    local target_repo_dir=$2
    local target_dir="$target_repo_dir/installed"

    echo_success ">>>> Restruct lejulab installed ws"

    rm -rf "$target_dir"
    mkdir -p "$target_dir/bin"
    mkdir -p "$target_dir/lib"
    mkdir -p "$target_dir/share"
    mkdir -p "$target_dir/include"

    # sync setup files and other non-directory files
    # Move all non-directory files from $installed_dir to $target_dir
    for file in $(find "$installed_dir" -maxdepth 1 -type f); do
        filename=$(basename "$file")
        echo ">>>> Moving $filename to $target_dir"
        mv "$file" "$target_dir/" || exit_with_failure "Failed to move $filename to $target_dir"
    done

    # sync bin
    if [ -d "$installed_dir/bin" ]; then
        cp -r "$installed_dir/bin"/* "$target_dir/bin/" || exit_with_failure "Failed to copy bin files"
    fi
    
    # sync include
    if [ -d "$installed_dir/include" ]; then
        cp -r "$installed_dir/include"/* "$target_dir/include/" || exit_with_failure "Failed to copy include files"
    fi

    # sync lib: exclude static libraries (.a), include all other files
    cp -r "$installed_dir/lib"/* "$target_dir/lib/" || exit_with_failure "Failed to copy lib files"
    for file in $(find "$target_dir/lib" -name "*.a"); do
        echo ">>>> Remove static library $file"
        rm "$file" || exit_with_failure "Failed to remove $file"
    done

    # sync cyclonedds dynamic libraries (required by lejusdk-lowlevel and lejusdk-vr)
    local third_party_dir="$PROJECT_DIR/src/lejusdk/3rd_party"
    echo ">>>> Copy CycloneDDS libraries to $target_dir/lib"
    cp -P "$third_party_dir/cyclonedds-0.10.2/lib"/libddsc.so* "$target_dir/lib/" || exit_with_failure "Failed to copy libddsc.so"
    cp -P "$third_party_dir/cyclonedds-cxx-0.10.2/lib"/libddscxx.so* "$target_dir/lib/" || exit_with_failure "Failed to copy libddscxx.so"

    # sync share - only whitelisted packages
    WHITELIST_FILE="${CI_SCRIPTS_DIR}/sync/package_whitelists.txt"
    if [ -f "$WHITELIST_FILE" ]; then
        # Read packages into array to avoid subshell issues
        mapfile -t packages < <(grep -v '^[[:space:]]*#' "$WHITELIST_FILE" | grep -v '^[[:space:]]*$')
        for package in "${packages[@]}"; do
            # Skip empty lines
            if [ -z "$package" ]; then
                continue
            fi
            echo "Installing $package..."
            package_dir="$installed_dir/share/$package"
            if [ -d "$package_dir" ]; then
                echo ">>>> Copy share directory for $package"
                cp -r "$package_dir" "$target_dir/share/" || exit_with_failure "Failed to copy share files for $package"
            fi
        done
    fi
}

restruct_lejulab_controller() {
    local target_repo_dir=$1

    # sync leju-controllers
    source_packages=(
        "leju-controllers"
        "leju_assets"
        "leju_launch"
        "leju-remote"
        "leju-ik"
        "leju-vr-control"
        # 先不以源码的方式发布
        # "leju-mujoco-sim"
        # "leju-joystick"
    )
    for package in "${source_packages[@]}"; do
        package_dir="$PROJECT_DIR/src/$package"
        if [ -d "$package_dir" ]; then
            echo ">>>> Copy $package"
            cp -r "$package_dir" "$target_repo_dir/src/" || exit_with_failure "Failed to copy $package"
        fi
    done
}

restruct_lejulab_lejusdk_docs() {
    local target_repo_dir=$1
    local target_docs_dir="$target_repo_dir/docs/leju-sdk"
    local gen_docs_script="$PROJECT_DIR/src/lejusdk/gen_docs.sh"
    local source_html_dir="$PROJECT_DIR/src/lejusdk/docs/output/html"

    echo_success ">>>> Generating and syncing LejuSDK documentation"

    # 生成文档
    if [ -f "$gen_docs_script" ]; then
        echo ">>>> Running gen_docs.sh..."
        chmod +x "$gen_docs_script"
        bash "$gen_docs_script" || exit_with_failure "Failed to generate LejuSDK documentation"
    else
        exit_with_failure "gen_docs.sh not found at $gen_docs_script"
    fi

    # 检查生成的 HTML 目录是否存在
    if [ ! -d "$source_html_dir" ]; then
        exit_with_failure "Generated HTML docs not found at $source_html_dir"
    fi

    # 清理目标目录并复制文档
    rm -rf "$target_docs_dir"
    mkdir -p "$target_docs_dir"
    cp -r "$source_html_dir"/* "$target_docs_dir/" || exit_with_failure "Failed to copy LejuSDK docs to $target_docs_dir"

    echo_success ">>>> LejuSDK documentation synced to $target_docs_dir"
}

restruct_lejulab() {
    local target_repo_dir=$1

    mkdir -p "$target_repo_dir"

    # sync README (lejulab_README.md -> README.md)
    if [ -f "$PROJECT_DIR/lejulab_README.md" ]; then
        echo ">>>> Copy lejulab_README.md to README.md"
        cp "$PROJECT_DIR/lejulab_README.md" "$target_repo_dir/README.md" || exit_with_failure "Failed to copy lejulab_README.md"
    else
        echo_warn "lejulab_README.md not found, skipping README sync"
    fi

    # sync docs, scripts, docker
    cp -r  "$PROJECT_DIR/docs" "$target_repo_dir/" || exit_with_failure "Failed to copy docs"
    cp -r  "$PROJECT_DIR/scripts" "$target_repo_dir/" || exit_with_failure "Failed to copy scripts"
    cp -r  "$PROJECT_DIR/docker" "$target_repo_dir/" || exit_with_failure "Failed to copy docker"

    # sync lejusdk
    restruct_lejulab_lejusdk "$PROJECT_DIR/installed_for_lejulab" "$target_repo_dir"

    # sync installed
    restruct_lejulab_installed "$PROJECT_DIR/installed_for_lejulab" "$target_repo_dir"

    # sync controller
    restruct_lejulab_controller "$target_repo_dir"

    # sync lejusdk docs (生成并同步 LejuSDK 文档到 docs/leju-sdk)
    restruct_lejulab_lejusdk_docs "$target_repo_dir"
}

sync_to_lejulab() {
    local lejulab_repo_dir=$1

    cd "$lejulab_repo_dir"

    # Check if there are any changes to commit
    echo_success ">>> Checking for file changes..."

    # Check for untracked files and modified files
    echo  ">>> Checking for untracked files <<<"
    git status
    echo  ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"

    # Check if there are local changes to commit
    if [ -n "$(git status --porcelain)" ]; then
        echo_warn "Changes detected, committing..."

        # Add all changes
        git add . || exit_with_failure "Failed to add changes to git"

        # Commit with a timestamp
        local commit_message="sync $CI_PROJECT_URL/commit/$CI_COMMIT_SHA"

        git commit -m "$commit_message" || exit_with_failure "Failed to commit changes"
        echo_success "Changes committed successfully"
    else
        echo_success "No changes to commit"
    fi

    # Always push to remote
    git push -f origin $CI_COMMIT_BRANCH || exit_with_failure "Git push failed"

    cd "$PROJECT_DIR"
}

catkin_install_ws() {
    cd "$PROJECT_DIR"
    catkin clean -y
    catkin config -DCMAKE_BUILD_TYPE=Release -DCMAKE_ASM_COMPILER=/usr/bin/as
    catkin config --install --install-space installed_for_lejulab
    if [ $? -ne 0 ]; then
        exit_with_failure "Error: catkin config --install failed"
    fi
    # 仅编译这些包
    catkin build leju-hardware lejusdk-lowlevel lejusdk-vr lejusdk-utils leju-joystick leju-mujoco-sim lejusdk-recorder
    if [ $? -ne 0 ]; then
        exit_with_failure "Error: catkin build failed"
    fi
}

clone_check_lejulab() {
    local lejulab_repo_url=$1
    local lejulab_repo_dir=$2
    local branch=$3

    echo_success ">>> Checking lejulab repository..."

    if [ -d "$lejulab_repo_dir" ]; then
        echo_warn "Repository exists at $lejulab_repo_dir, resetting..."

        # Change to repository directory
        cd "$lejulab_repo_dir" || exit_with_failure "Failed to enter repository directory: $lejulab_repo_dir"

        # Check if it's a git repository
        if [ ! -d ".git" ]; then
            echo_warn "Directory exists but is not a git repository, removing and cloning..."
            cd "$PROJECT_DIR"
            git clone "$lejulab_repo_url" "$lejulab_repo_dir" || exit_with_failure "Failed to clone lejulab repository"
            cd "$lejulab_repo_dir"
        else
            # Repository exists, fetch latest changes
            echo ">>> Fetching latest changes..."
            git fetch origin || exit_with_failure "Failed to fetch from origin"
        fi
    else
        echo ">>> Repository does not exist, cloning..."
        mkdir -p "$lejulab_repo_dir"
        git clone "$lejulab_repo_url" "$lejulab_repo_dir" || exit_with_failure "Failed to clone lejulab repository"
        cd "$lejulab_repo_dir"
    fi

    # 彻底清理工作目录状态（解决合并冲突、unmerged 文件等问题）
    echo ">>> Cleaning up working directory..."
    git merge --abort 2>/dev/null || true
    git rebase --abort 2>/dev/null || true
    git reset --hard HEAD 2>/dev/null || true
    git clean -dfx 2>/dev/null || true

    # Ensure we're on the correct branch and clean up
    echo ">>> Switching to branch: $branch"

    # Check if branch exists locally or remotely
    if git show-ref --verify --quiet refs/heads/"$branch"; then
        # Branch exists locally
        echo ">>> Branch exists locally"
        git checkout "$branch" || exit_with_failure "Failed to checkout branch: $branch"
    elif git ls-remote --heads origin "$branch" | grep -q "$branch"; then
        echo ">>> Branch exists remotely"
        # Branch exists remotely, create and track it
        git checkout -b "$branch" origin/"$branch" || exit_with_failure "Failed to checkout branch: $branch"
    else
        # Branch doesn't exist, create it from master
        echo_warn "Branch '$branch' does not exist remotely, creating new branch from master"
        # 先强制重置 master 分支到远程状态
        git checkout master || git checkout -B master origin/master || exit_with_failure "Failed to checkout master"
        git reset --hard origin/master || exit_with_failure "Failed to reset to origin/master"
        git clean -dfx || true
        git checkout -b "$branch" || exit_with_failure "Failed to create branch: $branch"
    fi

    # 强制重置到远程分支状态，避免合并冲突（同步会完全覆盖内容）
    if git ls-remote --heads origin "$branch" | grep -q "$branch"; then
        echo ">>> Resetting to origin/$branch..."
        git reset --hard origin/"$branch" || exit_with_failure "Failed to reset to origin/$branch"
    fi

    # Clean up any untracked files
    git clean -dfx || exit_with_failure "Failed to clean working directory"

    cd "$PROJECT_DIR"
    echo_success "Repository is ready on branch: $branch"
}

###  main ###
source /opt/ros/noetic/setup.bash

echo_success ">>> Syncing repo Url: $LEJULAB_REPO_URL"
echo_success ">>> Syncing repo Dir: $LEJULAB_REPO_DIR"
echo_success ">>> Syncing repo Branch: $LEJULAB_BRANCH"

# clone lejulab 仓库
clone_check_lejulab "$LEJULAB_REPO_URL" "$LEJULAB_REPO_DIR" "$LEJULAB_BRANCH"

# catkin build 编译
catkin_install_ws

# 调整 lejulab 目录结构
restruct_lejulab "$LEJULAB_REPO_DIR"

# 同步到 lejulab
sync_to_lejulab "$LEJULAB_REPO_DIR"

# 清除缓存
clear_up
