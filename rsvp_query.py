#!/usr/bin/env python3
"""Query RSVP submissions stored by the wedding website."""

from __future__ import annotations

import argparse
import json
import sqlite3
from datetime import UTC, datetime
from pathlib import Path


ROOT = Path(__file__).resolve().parent
DEFAULT_DATABASE = ROOT / "data" / "rsvps.sqlite3"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--database",
        type=Path,
        default=DEFAULT_DATABASE,
        help=f"SQLite database path (default: {DEFAULT_DATABASE})",
    )
    parser.add_argument(
        "--status",
        choices=("all", "accepts", "declines"),
        default="all",
        help="Filter responses by attendance status",
    )
    parser.add_argument("--limit", type=int, default=100, help="Maximum rows to show")
    parser.add_argument("--json", action="store_true", help="Print rows as JSON")
    parser.add_argument("--summary", action="store_true", help="Print attendance totals")
    return parser.parse_args()


def format_timestamp(timestamp: int) -> str:
    return datetime.fromtimestamp(timestamp, tz=UTC).strftime("%Y-%m-%d %H:%M UTC")


def main() -> None:
    args = parse_args()
    if not args.database.exists():
        raise SystemExit(f"Database does not exist: {args.database}")

    connection = sqlite3.connect(f"file:{args.database}?mode=ro", uri=True)
    connection.row_factory = sqlite3.Row

    if args.summary:
        rows = connection.execute(
            """
            SELECT
                COUNT(*) AS total,
                SUM(attending = 'joyfully-accepts') AS attending,
                SUM(attending = 'regretfully-declines') AS declined
            FROM rsvps
            """
        ).fetchone()
        print(f"Total responses: {rows['total']}")
        print(f"Joyfully accepts: {rows['attending'] or 0}")
        print(f"Regretfully declines: {rows['declined'] or 0}")
        return

    where = ""
    parameters: list[object] = []
    if args.status == "accepts":
        where = "WHERE attending = ?"
        parameters.append("joyfully-accepts")
    elif args.status == "declines":
        where = "WHERE attending = ?"
        parameters.append("regretfully-declines")
    parameters.append(max(1, args.limit))

    rows = connection.execute(
        f"""
        SELECT id, received_at, name, attending, dietary, note
        FROM rsvps
        {where}
        ORDER BY received_at DESC, id DESC
        LIMIT ?
        """,
        parameters,
    ).fetchall()

    records = [dict(row) for row in rows]
    if args.json:
        for record in records:
            record["received_at_iso"] = datetime.fromtimestamp(
                record["received_at"], tz=UTC
            ).isoformat()
        print(json.dumps(records, indent=2, ensure_ascii=False))
        return

    if not records:
        print("No RSVP responses found.")
        return

    for record in records:
        status = "YES" if record["attending"] == "joyfully-accepts" else "NO"
        print(
            f"#{record['id']}  {status:<3}  {format_timestamp(record['received_at'])}  "
            f"{record['name']}"
        )
        if record["dietary"]:
            print(f"     Dietary: {record['dietary']}")
        if record["note"]:
            print(f"     Note: {record['note']}")


if __name__ == "__main__":
    main()
