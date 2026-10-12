#!/bin/bash
# Shared parser for the aarch64 open-source release policy.
# The closed root CMakeLists supplies the default source package set;
# opensource_packages.txt only contains stub/binary overrides.

parse_opensource_cmake() {
    local cmake_file="${1:?usage: parse_opensource_cmake <CMakeLists.txt>}"
    [ -f "$cmake_file" ] || { echo "[manifest] ERROR: missing open-source CMakeLists: $cmake_file" >&2; return 1; }

    local package_file="$(dirname "$cmake_file")/source_packages.cmake"

    SRC_PKGS=()
    while IFS= read -r pkg; do
        [ -z "$pkg" ] && continue
        local duplicate=0 existing
        for existing in "${SRC_PKGS[@]}"; do
            [ "$existing" = "$pkg" ] && duplicate=1 && break
        done
        [ "$duplicate" -eq 0 ] && SRC_PKGS+=("$pkg")
    done < <(
        {
            cat "$cmake_file"
            [ -f "$package_file" ] && cat "$package_file"
        } |
        grep -E '^[[:space:]]*add_subdirectory[[:space:]]*\([[:space:]]*"?src/[A-Za-z0-9_.-]+(/[A-Za-z0-9_.-]+)*"?[[:space:]]*\)' |
            sed -E 's/^[[:space:]]*add_subdirectory[[:space:]]*\([[:space:]]*"?src\/([^"[:space:])]+)"?[[:space:]]*\).*/\1/'
    )
    [ "${#SRC_PKGS[@]}" -gt 0 ] || {
        echo "[manifest] ERROR: no src add_subdirectory entries found in $cmake_file" >&2
        return 1
    }
}

parse_manifest() {
    local manifest="${1:?usage: parse_manifest <manifest>}"
    [ -f "$manifest" ] || { echo "[manifest] ERROR: manifest missing: $manifest" >&2; return 1; }

    STUB_PKGS=()
    BINARY_PKGS=()
    declare -gA BINARY_SUBDIRS=()
    declare -gA BINARY_SEEN=()
    local raw line tier spec pkg sub line_no=0
    while IFS= read -r raw || [ -n "$raw" ]; do
        line_no=$((line_no + 1))
        line="${raw//$'\r'/}"
        line="$(printf '%s' "$line" | sed 's/^\xEF\xBB\xBF//; s/#.*//; s/^[[:space:]]*//; s/[[:space:]]*$//')"
        [ -z "$line" ] && continue
        read -r tier spec extra <<< "$line"
        [ -n "${tier:-}" ] && [ -n "${spec:-}" ] && [ -z "${extra:-}" ] || {
            echo "[manifest] ERROR: malformed line $line_no: $raw" >&2; return 1;
        }
        case "$tier" in
            source)
                echo "[manifest] ERROR: source tier is obsolete at line $line_no; omit it to use default source" >&2
                return 1 ;;
            stub|binary) ;;
            *)
                echo "[manifest] ERROR: unknown tier '$tier' at line $line_no" >&2
                return 1 ;;
        esac
        pkg="${spec%%:*}"
        [[ "$pkg" =~ ^[A-Za-z0-9_.-]+(/[A-Za-z0-9_.-]+)*$ && "$pkg" != *..* ]] || {
            echo "[manifest] ERROR: invalid package path at line $line_no: $pkg" >&2; return 1;
        }
        local -a arr
        if [ "$tier" = stub ]; then
            [ "$spec" = "$pkg" ] || { echo "[manifest] ERROR: stub cannot have subpath at line $line_no" >&2; return 1; }
            [ -z "${BINARY_SUBDIRS[$pkg]+x}" ] || { echo "[manifest] ERROR: $pkg declared as both stub and binary" >&2; return 1; }
            for sub in "${STUB_PKGS[@]}"; do [ "$sub" = "$pkg" ] && { echo "[manifest] ERROR: duplicate stub: $pkg" >&2; return 1; }; done
            STUB_PKGS+=("$pkg")
        else
            if [ "$spec" != "$pkg" ]; then
                [ -n "${spec#*:}" ] || { echo "[manifest] ERROR: empty binary subpath at line $line_no" >&2; return 1; }
                IFS=',' read -ra arr <<< "${spec#*:}"
                for sub in "${arr[@]}"; do
                    sub="$(printf '%s' "$sub" | sed 's/^[[:space:]]*//; s/[[:space:]]*$//')"
                    [ -n "$sub" ] && [[ "$sub" =~ ^[A-Za-z0-9_.-]+(/[A-Za-z0-9_.-]+)*$ && "$sub" != *..* ]] || { echo "[manifest] ERROR: invalid binary subpath at line $line_no" >&2; return 1; }
                done
                BINARY_SUBDIRS["$pkg"]="${spec#*:}"
            else
                BINARY_SUBDIRS["$pkg"]=""
            fi
            [ -z "${BINARY_SEEN[$pkg]+x}" ] || { echo "[manifest] ERROR: duplicate binary: $pkg" >&2; return 1; }
            for sub in "${STUB_PKGS[@]}"; do [ "$sub" = "$pkg" ] && { echo "[manifest] ERROR: $pkg declared as both stub and binary" >&2; return 1; }; done
            BINARY_SEEN["$pkg"]=1
            BINARY_PKGS+=("$pkg")
        fi
    done < "$manifest"
}
