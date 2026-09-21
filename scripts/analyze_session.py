#!/usr/bin/env python3
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('session', type=Path)
    args = parser.parse_args()
    report = json.loads((args.session / 'report.json').read_text())
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
