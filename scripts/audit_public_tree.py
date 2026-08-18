#!/usr/bin/env python3
"""Fail when the public Git tree contains private tooling, secrets or run data."""

import argparse
import re
import subprocess
import sys
from pathlib import Path, PurePosixPath

FORBIDDEN_PATHS = (
    re.compile(r"(^|/)(CLAUDE\.md|\.claude|\.codex)(/|$)", re.I),
    re.compile(r"(^|/)\.env(?:\..*)?$", re.I),
    re.compile(r"(^|/)(flight-log-.*\.json|closed-loop-report\.json|perception-quality-report\.json)$", re.I),
    re.compile(r"(^|/).+-(?:manifest|anchors)\.json$", re.I),
    re.compile(r"^(?:models|experiment_data|artifacts|logs)/", re.I),
)
CONTENT_RULES = (
    ("private-key", re.compile(r"-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----")),
    ("jwt", re.compile(r"\beyJ[A-Za-z0-9_-]{10,}\.[A-Za-z0-9_-]{10,}\.[A-Za-z0-9_-]{10,}\b")),
    ("home-path", re.compile(r"(?:^|[\s'\"=:(])(?:/home/[^/\s]+|/Users/[^/\s]+)(?:/|\b)")),
    ("windows-user-path", re.compile(r"\b[A-Za-z]:\\Users\\[^\\\s]+\\")),
    ("credential-assignment", re.compile(r"(?i)\b(?:api[_-]?key|access[_-]?token|secret|password)\b\s*[:=]\s*['\"][A-Za-z0-9_./+=-]{20,}['\"]")),
)
TEXT_SUFFIXES = {
    ".c", ".cc", ".cpp", ".css", ".dockerfile", ".go", ".h", ".html",
    ".ini", ".java", ".js", ".json", ".md", ".mjs", ".py", ".rs",
    ".sh", ".toml", ".ts", ".tsx", ".txt", ".yaml", ".yml",
}
TEXT_NAMES = {"Dockerfile", "LICENSE", "NOTICE"}
ROOT = Path(__file__).resolve().parents[1]
LEGACY_HOME_PATH_EXCEPTIONS = {"dependencies.versions.json", "fix-clash-rules.sh"}
PUBLIC_CLIENT_TOKEN_DECLARATION = "const DEFAULT_CESIUM_ION_TOKEN = "


def git_output(*args, binary=False):
    return subprocess.check_output(
        ["git", *args], text=not binary, cwd=ROOT, stderr=subprocess.DEVNULL
    )


def tracked_paths(staged=False):
    command = ("diff", "--cached", "--name-only", "--diff-filter=ACMR") if staged else ("ls-files",)
    paths = [line for line in git_output(*command).splitlines() if line]
    return paths if staged else [path for path in paths if (ROOT / path).is_file()]


def file_bytes(path, staged=False):
    if staged:
        return git_output("show", f":{path}", binary=True)
    return (ROOT / path).read_bytes()


def is_text_path(path):
    pure = PurePosixPath(path)
    return pure.name in TEXT_NAMES or pure.suffix.lower() in TEXT_SUFFIXES


def audit(staged=False):
    findings = []
    for path in tracked_paths(staged):
        for rule in FORBIDDEN_PATHS:
            if rule.search(path):
                findings.append((path, 0, "forbidden-public-path"))
                break
        if not is_text_path(path):
            continue
        if path == "scripts/audit_public_tree.py":
            continue
        try:
            source = file_bytes(path, staged).decode("utf-8")
        except (OSError, UnicodeDecodeError, subprocess.CalledProcessError):
            continue
        for line_number, line in enumerate(source.splitlines(), 1):
            for label, pattern in CONTENT_RULES:
                if label == "home-path" and path in LEGACY_HOME_PATH_EXCEPTIONS:
                    continue
                if (
                    path == "src/cesium-token.js"
                    and label in {"jwt", "credential-assignment"}
                    and PUBLIC_CLIENT_TOKEN_DECLARATION in line
                ):
                    # CesiumJS needs this deliberately public, browser-visible
                    # client token. The exception is restricted to one named
                    # declaration and never applies to server credentials.
                    continue
                if pattern.search(line):
                    findings.append((path, line_number, label))
    return sorted(set(findings))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--staged", action="store_true", help="scan only staged additions and modifications")
    args = parser.parse_args(argv)
    findings = audit(staged=args.staged)
    if findings:
        for path, line, rule in findings:
            location = f"{path}:{line}" if line else path
            print(f"ERROR: {location}: {rule}", file=sys.stderr)
        return 1
    print("Public tree audit passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
