#!/bin/bash
# inspect_topics.sh — Lightweight runtime topic inspection for Gazebo Harmonic
#
# Usage:
#   ./scripts/inspect_topics.sh              # List all Gazebo topics
#   ./scripts/inspect_topics.sh --world NAME # Filter topics for a specific world
#
# This script queries Gazebo's topic list at runtime.  It does NOT hardcode
# topic names — the actual topics depend on the loaded world and models.
# Use this output to configure ros_gz_bridge mappings later.

set -euo pipefail

# Check if Gazebo is running by trying to list topics
if ! command -v gz &>/dev/null; then
    echo "❌ 'gz' command not found. Is Gazebo Harmonic installed?"
    echo "   Install: sudo apt install gz-harmonic"
    exit 1
fi

echo "📡 Gazebo runtime topics"
echo "========================"
echo ""

# Get the list of topics
TOPICS=$(gz topic -l 2>/dev/null || true)

if [ -z "${TOPICS}" ]; then
    echo "⚠️  No topics found. Is Gazebo running?"
    echo "   Start Gazebo first: gz sim -v4"
    exit 1
fi

echo "${TOPICS}"

echo ""
echo "========================"
echo "💡 To inspect a specific topic, run:"
echo "   gz topic -e -t <TOPIC_NAME> --duration 5"
echo ""
echo "💡 Use this topic list to configure ros_gz_bridge parameter files."
