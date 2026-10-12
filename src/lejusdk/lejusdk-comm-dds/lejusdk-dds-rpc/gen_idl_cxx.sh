#!/bin/bash
#
# gen_idl_cxx.sh - Generate C++ bindings from IDL files
#
# This script generates C++ code from IDL files using CycloneDDS idlc compiler.
# The generated files are placed in src-generated/ and include/ directories.
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Create output directories
mkdir -p src-generated include/rpc_idl

echo "Generating C++ files from IDL..."
echo ""

# Process each IDL file (including subdirectories)
idl_files=$(find idl -name "*.idl" 2>/dev/null)
if [ -z "$idl_files" ]; then
    echo "No IDL files found in idl/"
    exit 1
fi

for file in $idl_files; do
    base=$(basename "$file" .idl)
    echo "Processing $file"

    # Generate C++ code using idlc (-I idl for include search path)
    if idlc -l cxx -I idl -o . "$file"; then
        # Move header to include/rpc_idl/
        if [ -f "${base}.hpp" ]; then
            mv "${base}.hpp" include/rpc_idl/
            echo "  Generated include/rpc_idl/${base}.hpp"
        else
            echo "  Warning: ${base}.hpp not found"
        fi

        # Move source to src-generated/
        if [ -f "${base}.cpp" ]; then
            mv "${base}.cpp" src-generated/
            echo "  Generated src-generated/${base}.cpp"
        else
            echo "  Warning: ${base}.cpp not found"
        fi
    else
        echo "  Failed to process $file"
        exit 1
    fi
done

echo ""
echo "Generated files:"
echo "================"
echo "Headers:"
ls -la include/rpc_idl/*.hpp 2>/dev/null || echo "  No .hpp files found"
echo ""
echo "Sources:"
ls -la src-generated/*.cpp 2>/dev/null || echo "  No .cpp files found"
echo ""
echo "Done."
