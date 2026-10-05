"""Tag demo children and stop only processes owned by this checkout (Linux)."""

import argparse
import os
import signal
import time
from pathlib import Path

MARKER = "ONBOARD_AUTONOMY_DEMO_ROOT"
PROJECT_ROOT = str(Path(__file__).resolve().parents[1])


def tagged_processes() -> list[tuple[int, int]]:
    """Return PID/start-time pairs; never select by a shared command name."""
    expected = os.fsencode(f"{MARKER}={PROJECT_ROOT}")
    result = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdecimal() or int(entry.name) == os.getpid():
            continue
        try:
            if entry.stat().st_uid != os.getuid():
                continue
            if expected not in (entry / "environ").read_bytes().split(b"\0"):
                continue
            fields = (entry / "stat").read_text().rpartition(") ")[2].split()
            if fields[0] != "Z":
                result.append((int(entry.name), int(fields[19])))
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    return result


def signal_process(pid: int, started: int, signum: int) -> None:
    # pidfd keeps a PID recycled during cleanup from targeting a new process.
    try:
        descriptor = os.pidfd_open(pid)
    except ProcessLookupError:
        return
    try:
        fields = Path(f"/proc/{pid}/stat").read_text().rpartition(") ")[2].split()
        if int(fields[19]) == started:
            signal.pidfd_send_signal(descriptor, signum)
    except (FileNotFoundError, ProcessLookupError):
        pass
    finally:
        os.close(descriptor)


def stop_demo() -> int:
    pending = tagged_processes()
    if not pending:
        print("No tagged OnboardAutonomy demo processes are running.")
        return 0
    print("Stopping this checkout's OnboardAutonomy demo processes...")
    deadline = time.monotonic() + 3.0
    while pending and time.monotonic() < deadline:
        for pid, started in pending:
            signal_process(pid, started, signal.SIGTERM)
        time.sleep(0.1)
        pending = tagged_processes()
    for pid, started in pending:
        signal_process(pid, started, signal.SIGKILL)
    deadline = time.monotonic() + 1.0
    while tagged_processes() and time.monotonic() < deadline:
        time.sleep(0.05)
    if tagged_processes():
        print("Some tagged demo processes did not stop.")
        return 1
    print("Demo processes stopped.")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("run", "stop"))
    parser.add_argument("command", nargs=argparse.REMAINDER)
    options = parser.parse_args()
    if options.action == "stop":
        if options.command:
            parser.error("stop accepts no command")
        return stop_demo()
    command = options.command
    if command[:1] == ["--"]:
        command = command[1:]
    if not command:
        parser.error("run requires a command after --")
    os.execvpe(command[0], command, os.environ | {MARKER: PROJECT_ROOT})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
