#!/usr/bin/env bash
# Install the C4 diagram rendering toolchain used by scripts/generate_c4.py:
# Graphviz + the PlantUML jar (fetched from Maven Central, pinned version).
# Idempotent — safe to re-run. Needs Java (any JRE >= 8) already installed.
#
# The jar is a plain file, so it defaults to a user-writable location
# (no sudo needed for that part). Graphviz is a system package and still
# needs root via apt-get — skipped automatically if `dot` is already on PATH.
set -euo pipefail

PLANTUML_VERSION="${PLANTUML_VERSION:-1.2025.4}"
PLANTUML_JAR="${PLANTUML_JAR:-${XDG_DATA_HOME:-$HOME/.local/share}/plantuml/plantuml.jar}"

if ! command -v java >/dev/null; then
    echo "error: java not found — install a JRE first (e.g. apt-get install default-jre-headless)" >&2
    exit 1
fi

if ! command -v dot >/dev/null; then
    SUDO=""
    [ "$(id -u)" != 0 ] && SUDO="sudo"
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
