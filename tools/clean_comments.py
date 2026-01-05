#!/usr/bin/env python3
"""
Script to clean unnecessary comments from Python files.

Removes:
- Obvious comments (e.g., "# Increment counter" before "counter += 1")
- Commented-out code
- Redundant comments
- Empty comment lines

Keeps:
- Docstrings
- Complex logic explanations
- TODOs/FIXMEs
- License/copyright headers
"""

import re
import sys
from pathlib import Path
from typing import List, Tuple


def is_obvious_comment(comment: str, next_line: str) -> bool:
    """Check if a comment is obvious/redundant with the code."""
    comment_lower = comment.lower().strip()
    next_lower = next_line.lower().strip()
    
    # Patterns of obvious comments
    obvious_patterns = [
        (r'#\s*increment', r'\+\s*=\s*1'),
        (r'#\s*decrement', r'-\s*=\s*1'),
        (r'#\s*return', r'^return'),
        (r'#\s*import', r'^import|^from'),
        (r'#\s*print', r'^print'),
        (r'#\s*set\s+\w+', r'='),
        (r'#\s*create\s+\w+', r'='),
        (r'#\s*initialize', r'='),
    ]
    
    for comment_pat, code_pat in obvious_patterns:
        if re.search(comment_pat, comment_lower) and re.search(code_pat, next_lower):
            return True
    
    return False


def is_commented_code(line: str) -> bool:
    """Check if a line is commented-out code."""
    stripped = line.strip()
    if not stripped.startswith('#'):
        return False
    
    # Remove the # and check if it looks like code
    potential_code = stripped[1:].strip()
    
    # Code patterns
    code_patterns = [
        r'^(import|from)\s+\w+',
        r'^(def|class)\s+\w+',
        r'^\w+\s*=\s*.+',
        r'^\w+\.\w+\(',
        r'^(if|for|while|try|except|with)\s+',
        r'^(return|pass|break|continue)\b',
    ]
    
    for pattern in code_patterns:
        if re.match(pattern, potential_code):
            return True
    
    return False


def should_keep_comment(line: str, next_line: str = "") -> bool:
    """Determine if a comment should be kept."""
    stripped = line.strip()
    
    # Keep docstrings
    if '"""' in line or "'''" in line:
        return True
    
    # Keep TODOs, FIXMEs, NOTEs
    if re.search(r'#\s*(TODO|FIXME|NOTE|HACK|XXX|IMPORTANT|WARNING):', stripped, re.IGNORECASE):
        return True
    
    # Keep copyright/license
    if re.search(r'#\s*(copyright|license|author)', stripped, re.IGNORECASE):
        return True
    
    # Keep section headers (e.g., # ===== Section =====)
    if re.match(r'#\s*[=\-]{3,}', stripped):
        return True
    
    # Keep substantial comments (>40 chars, not just code description)
    if len(stripped) > 40 and not is_commented_code(line):
        return True
    
    # Remove commented code
    if is_commented_code(line):
        return False
    
    # Remove obvious comments
    if next_line and is_obvious_comment(line, next_line):
        return False
    
    # Remove empty comments
    if stripped == '#':
        return False
    
    # Keep everything else (be conservative)
    return True


def clean_file(filepath: Path, dry_run: bool = True) -> Tuple[int, int]:
    """
    Clean unnecessary comments from a Python file.
    
    Returns:
        (lines_removed, lines_kept)
    """
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            lines = f.readlines()
    except Exception as e:
        print(f"Error reading {filepath}: {e}")
        return 0, 0
    
    cleaned_lines = []
    removed_count = 0
    in_docstring = False
    
    for i, line in enumerate(lines):
        # Track docstrings
        if '"""' in line or "'''" in line:
            in_docstring = not in_docstring
            cleaned_lines.append(line)
            continue
        
        # Always keep non-comment lines and lines in docstrings
        if in_docstring or not line.strip().startswith('#'):
            cleaned_lines.append(line)
            continue
        
        # Check if we should keep this comment
        next_line = lines[i + 1] if i + 1 < len(lines) else ""
        if should_keep_comment(line, next_line):
            cleaned_lines.append(line)
        else:
            removed_count += 1
            if not dry_run:
                print(f"  Removed: {line.rstrip()}")
    
    if not dry_run and removed_count > 0:
        with open(filepath, 'w', encoding='utf-8') as f:
            f.writelines(cleaned_lines)
    
    return removed_count, len(lines) - removed_count


def main():
    """Main function."""
    import argparse
    
    parser = argparse.ArgumentParser(description="Clean unnecessary comments from Python files")
    parser.add_argument('paths', nargs='+', help='Files or directories to clean')
    parser.add_argument('--dry-run', action='store_true', help='Show what would be removed without actually removing')
    parser.add_argument('--exclude', nargs='*', default=['deps', 'venv', '__pycache__', 'tests'], help='Directories to exclude')
    
    args = parser.parse_args()
    
    # Collect all Python files
    files_to_clean = []
    for path_str in args.paths:
        path = Path(path_str)
        if path.is_file() and path.suffix == '.py':
            files_to_clean.append(path)
        elif path.is_dir():
            for py_file in path.rglob('*.py'):
                # Check if file is in excluded directory
                if not any(excl in py_file.parts for excl in args.exclude):
                    files_to_clean.append(py_file)
    
    if not files_to_clean:
        print("No Python files found to clean.")
        return
    
    print(f"Found {len(files_to_clean)} Python files to analyze")
    if args.dry_run:
        print("DRY RUN MODE - No files will be modified\n")
    else:
        print("LIVE MODE - Files will be modified\n")
    
    total_removed = 0
    total_kept = 0
    
    for filepath in sorted(files_to_clean):
        removed, kept = clean_file(filepath, dry_run=args.dry_run)
        if removed > 0:
            print(f"{filepath}: {removed} comments removed, {kept} lines kept")
            total_removed += removed
            total_kept += kept
    
    print(f"\nTotal: {total_removed} comments removed from {len(files_to_clean)} files")
    
    if args.dry_run:
        print("\nRun without --dry-run to actually remove comments")


if __name__ == "__main__":
    main()
