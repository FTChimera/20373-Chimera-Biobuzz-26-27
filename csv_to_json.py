#!/usr/bin/env python3

import argparse
import csv
import json
from pathlib import Path


def convert_csv_to_json(csv_path: Path, json_path: Path) -> None:
    rows = []
    with csv_path.open("r", encoding="utf-8-sig", newline="") as infile:
        reader = csv.DictReader(infile)
        for row in reader:
            rows.append(row)

    with json_path.open("w", encoding="utf-8") as outfile:
        json.dump(rows, outfile, indent=2)
        outfile.write("\n")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert a CSV file into a JSON array of row objects."
    )
    parser.add_argument("csv_file", type=Path, help="Path to the input CSV file")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="Path to output JSON file (defaults to input filename with .json extension)",
    )
    args = parser.parse_args()

    csv_path = args.csv_file
    if not csv_path.is_file():
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    json_path = args.output if args.output else csv_path.with_suffix(".json")
    convert_csv_to_json(csv_path, json_path)


if __name__ == "__main__":
    main()
