"""URI validation, recent selections, and radio-only discovery."""

from dataclasses import asdict, dataclass, replace
import json
import math
import os
from pathlib import Path
import re
import tempfile
import time

DEFAULT_ADDRESS = 0xE7E7E7E7E7
URI = re.compile(r"radio://(\d+)/(\d+)/(250K|1M|2M)(?:/([0-9a-f]{1,10}))?", re.I)


class PickerError(ValueError):
    """An error that can be shown directly to the user."""


def normalize_uri(value: str, default_address: int = DEFAULT_ADDRESS) -> str:
    match = URI.fullmatch(value.strip()) if isinstance(value, str) else None
    if not match or not 0 <= int(match[2]) <= 125:
        raise PickerError("Expected radio://INDEX/CHANNEL/RATE/ADDRESS; channel 0–125, rate 250K, 1M, or 2M.")
    address = int(match[4], 16) if match[4] else default_address
    return f"radio://{int(match[1])}/{int(match[2])}/{match[3].upper()}/{address:010X}"


@dataclass(frozen=True)
class Robot:
    uri: str
    name: str = ""
    last_selected: float = 0


DEMO_RECENTS = (
    Robot("radio://0/80/2M/E7E7E7E702", "demo-02", 2),
    Robot("radio://0/80/2M/E7E7E7E701", "demo-01", 1),
)


def default_history(demo: bool = False) -> Path:
    filename = "robots-demo.json" if demo else "robots.json"
    return Path(__file__).resolve().parents[2] / ".cf-container" / filename


def load_history(path: Path) -> list[Robot]:
    try:
        data = json.loads(path.read_text())
    except FileNotFoundError:
        return []
    except (OSError, ValueError) as exc:
        raise PickerError(f"Cannot read history {path}: {exc}") from exc
    try:
        if not isinstance(data, dict) or not isinstance(data.get("robots"), list):
            raise ValueError("expected an object containing a robots list")
        robots = {}
        for entry in data["robots"]:
            robot = Robot(normalize_uri(entry["uri"]), entry.get("name", ""), float(entry.get("last_selected", 0)))
            if not isinstance(robot.name, str) or not math.isfinite(robot.last_selected):
                raise ValueError("invalid nickname or selection time")
            old = robots.get(robot.uri)
            if old is None or robot.last_selected >= old.last_selected:
                robots[robot.uri] = robot
        return sorted(robots.values(), key=lambda robot: (-robot.last_selected, robot.uri))
    except (KeyError, TypeError, ValueError) as exc:
        raise PickerError(f"Invalid history {path}: {exc}") from exc


def record_selection(path: Path, robot: Robot, timestamp: float) -> Robot:
    robots = {item.uri: item for item in load_history(path)}
    previous = robots.get(robot.uri)
    robot = replace(robot, name=robot.name or (previous.name if previous else ""), last_selected=timestamp)
    robots[robot.uri] = robot
    save_history(path, list(robots.values()))
    return robot


def save_history(path: Path, robots: list[Robot]) -> None:
    temporary = None
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(mode="w", dir=path.parent, delete=False) as output:
            temporary = Path(output.name)
            json.dump({"robots": [asdict(item) for item in robots]}, output, indent=2, allow_nan=False)
            output.write("\n")
        os.replace(temporary, path)
    except OSError as exc:
        raise PickerError(f"Cannot save history {path}: {exc}") from exc
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def scan_radios(addresses: list[int]) -> list[Robot]:
    """Only access Crazyradio; do not scan USB-connected Crazyflies."""
    try:
        from cflib.crtp.radiodriver import RadioDriver, RadioManager

        probe = RadioManager.open(0)
        probe.close()
        found = []
        for address in addresses:
            for uri, _ in RadioDriver().scan_interface(address):
                found.append(Robot(normalize_uri(uri, address)))
        return found
    except Exception as exc:
        raise PickerError(f"Radio scan failed: {exc}. Check the Crazyradio connection and host USB permissions.") from exc


def demo_scan(addresses: list[int]) -> list[Robot]:
    return [DEMO_RECENTS[1], Robot("radio://0/90/2M/E7E7E7E703", "demo-03")]


class Picker:
    def __init__(self, history: Path, demo: bool = False, scanner=None, clock=time.time):
        self.history = history
        self.demo = demo
        self.clock = clock
        self.scanner = scanner if scanner is not None else (demo_scan if demo else scan_radios)
        self.robots = {robot.uri: robot for robot in (DEMO_RECENTS if demo else ())}
        self.robots.update({robot.uri: robot for robot in load_history(history)})
        self.detected: set[str] = set()

    def rows(self) -> list[Robot]:
        return sorted(self.robots.values(), key=lambda robot: (-robot.last_selected, robot.uri))

    def label(self, robot: Robot) -> str:
        name = f"{robot.name} — " if robot.name else ""
        status = "detected" if robot.uri in self.detected else "saved; not checked"
        return f"{name}{robot.uri} [{status}]"

    def addresses(self) -> list[int]:
        return sorted({DEFAULT_ADDRESS} | {int(uri.rsplit("/", 1)[1], 16) for uri in self.robots})

    def scan(self) -> list[Robot]:
        try:
            return self.scanner(self.addresses())
        except PickerError:
            raise
        except Exception as exc:
            raise PickerError(f"Scan failed: {exc}") from exc

    def merge_scan(self, found: list[Robot]) -> None:
        self.detected = set()
        for robot in found:
            uri = normalize_uri(robot.uri)
            old = self.robots.get(uri)
            self.robots[uri] = replace(old or robot, uri=uri, name=(old.name if old and old.name else robot.name))
            self.detected.add(uri)

    def choose(self, robot: Robot) -> str:
        robot = replace(robot, uri=normalize_uri(robot.uri), name=robot.name.strip())
        return record_selection(self.history, robot, self.clock()).uri

    def rename(self, uri: str, name: str) -> Robot:
        uri = normalize_uri(uri)
        name = name.strip()
        if not name:
            raise PickerError("Enter a nickname before saving.")
        if uri not in self.robots:
            raise PickerError("Select a drone to rename.")
        saved = {robot.uri: robot for robot in load_history(self.history)}
        robot = replace(saved.get(uri, self.robots[uri]), name=name)
        saved[uri] = robot
        save_history(self.history, list(saved.values()))
        self.robots[uri] = robot
        return robot
