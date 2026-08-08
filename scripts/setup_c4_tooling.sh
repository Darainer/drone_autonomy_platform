#!/usr/bin/env bash
# Install the C4 diagram rendering toolchain used by scripts/generate_c4.py:
# a headless JRE + Graphviz + the PlantUML jar (fetched from Maven Central,
# pinned version). Idempotent — safe to re-run.
#
# The jar is a plain file, so it defaults to a user-writable location
# (no sudo needed for that part). Java and Graphviz are system packages and
# need root via apt-get — each is skipped automatically if already on PATH.
# On a machine with no apt-get (or no root/sudo), this fails with a clear
# message instead of a confusing apt-get error.
set -euo pipefail

PLANTUML_VERSION="${PLANTUML_VERSION:-1.2025.4}"
PLANTUML_JAR="${PLANTUML_JAR:-${XDG_DATA_HOME:-$HOME/.local/share}/plantuml/plantuml.jar}"

# Resolve how (if at all) we can install system packages, once, for both
# graphviz and the JRE below.
need_apt() {
    if ! command -v apt-get >/dev/null; then
        echo "error: apt-get not found — install $1 manually, or set PLANTUML_JAR / ensure java and dot are on PATH" >&2
        exit 1
    fi
    SUDO=""
    if [ "$(id -u)" != 0 ]; then
        if command -v sudo >/dev/null; then
            SUDO="sudo"
        else
            echo "error: not root and no sudo available — install $1 manually (apt-get install $1)" >&2
            exit 1
        fi
    fi
    if [ -z "${APT_UPDATED:-}" ]; then
        $SUDO apt-get update -qq
        APT_UPDATED=1
    fi
}

if ! command -v java >/dev/null; then
    need_apt default-jre-headless
    $SUDO apt-get install -y -qq default-jre-headless
fi

if ! command -v dot >/dev/null; then
    need_apt graphviz
    $SUDO apt-get install -y -qq graphviz
fi

if [ ! -s "$PLANTUML_JAR" ]; then
    mkdir -p "$(dirname "$PLANTUML_JAR")"
    curl -fsSL -o "$PLANTUML_JAR" \
        "https://repo1.maven.org/maven2/net/sourceforge/plantuml/plantuml/${PLANTUML_VERSION}/plantuml-${PLANTUML_VERSION}.jar"
fi

echo "dot:      $(dot -V 2>&1)"
echo "plantuml: $(java -jar "$PLANTUML_JAR" -version 2>/dev/null | head -1)"
echo "OK — generate views with: python scripts/generate_c4.py"
