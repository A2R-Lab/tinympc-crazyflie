"""Recent-drone picker using Textual, with a Typer command-line interface."""

import asyncio
from datetime import datetime
from pathlib import Path

import typer
from rich.text import Text
from textual import on, work
from textual.app import App, ComposeResult
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.screen import ModalScreen
from textual.widgets import Button, DataTable, Footer, Input, Static

from .core import Picker, PickerError, Robot, default_history


class RenameDialog(ModalScreen[str | None]):
    BINDINGS = [("escape", "cancel", "Cancel")]

    def __init__(self, robot: Robot):
        super().__init__()
        self.robot = robot

    def compose(self) -> ComposeResult:
        with Vertical(id="rename-dialog"):
            yield Static("Rename drone", id="rename-title")
            yield Static(self.robot.uri, id="rename-uri", markup=False)
            yield Input(value=self.robot.name, placeholder="Nickname", id="rename-name", select_on_focus=True)
            yield Static("Saved locally on this computer.", id="rename-hint", markup=False)
            with Horizontal(id="rename-buttons"):
                yield Button("Cancel", id="rename-cancel")
                yield Button("Save name", id="rename-save", variant="primary")

    def on_mount(self) -> None:
        self.query_one(Input).focus()

    @on(Button.Pressed, "#rename-save")
    @on(Input.Submitted, "#rename-name")
    def save(self) -> None:
        name = self.query_one(Input).value.strip()
        if not name:
            self.query_one("#rename-hint", Static).update("Enter a nickname before saving.")
            self.query_one("#rename-hint").add_class("error")
            self.query_one(Input).focus()
            return
        self.dismiss(name)

    @on(Button.Pressed, "#rename-cancel")
    def action_cancel(self) -> None:
        self.dismiss(None)


class DronePickerApp(App[str | None]):
    TITLE = "Crazyflie"
    BINDINGS = [("escape", "cancel", "Cancel"), ("ctrl+c", "cancel", "Cancel"), ("f2", "rename", "Rename"), ("f5", "scan", "Scan")]
    CSS_PATH = "textual_ui.tcss"

    def __init__(self, picker: Picker):
        super().__init__()
        self.picker = picker
        self.row_uris: list[str] = []
        self.scanning = False
        self.session: list[str] = []

    def record(self, message: str) -> None:
        self.session.append(f"{datetime.now().astimezone():%H:%M:%S %Z}  {message}")

    def print_summary(self) -> None:
        typer.echo("\nCrazyflie · session summary" + (" [DEMO]" if self.picker.demo else ""), err=True)
        for entry in self.session:
            typer.echo(f"  {entry}", err=True)
        typer.echo("", err=True)

    def compose(self) -> ComposeResult:
        with VerticalScroll(id="body"):
            with Vertical(id="workspace"):
                with Horizontal(id="brand"):
                    yield Static("◈", id="mark")
                    with Vertical(id="heading"):
                        yield Static("Choose your Crazyflie", id="title")
                        yield Static("Recent selections first · saved drones may be offline", id="intro")
                    yield Static("DEMO" if self.picker.demo else "RADIO", id="mode")
                with Vertical(id="fleet"):
                    yield DataTable(id="robots", cursor_type="row", zebra_stripes=False)
                    yield Static("No saved drones yet. Scan or enter a URI below.", id="empty")
                yield Static("↑ ↓ to browse · Enter to choose · F2 to rename", id="status", markup=False)
                with Horizontal(id="manual"):
                    yield Input(placeholder="Radio URI", id="uri")
                    yield Input(placeholder="Nickname (optional)", id="nickname")
                    yield Button("Use URI", id="manual-use")
                with Horizontal(id="buttons"):
                    yield Button("Scan", id="scan")
                    yield Button("Rename", id="rename")
                    yield Static("", id="button-spacer")
                    yield Button("Cancel", id="cancel")
                    yield Button("Choose drone →", id="choose", variant="primary")
        yield Footer()

    def on_mount(self) -> None:
        self.query_one("#fleet").border_title = "Your drones"
        self.query_one("#manual").border_title = "Enter a URI"
        self.query_one(DataTable).add_columns("Drone", "Radio URI", "Availability")
        self.refresh_rows()
        self.query_one(DataTable).focus()

    def on_resize(self, event) -> None:
        self.set_class(event.size.width < 72, "compact")

    def refresh_rows(self, selected_uri: str | None = None) -> None:
        table = self.query_one(DataTable)
        table.clear()
        self.row_uris = []
        for robot in self.picker.rows():
            detected = robot.uri in self.picker.detected
            status = Text("● Detected" if detected else "○ Unchecked", style="#7dd9b6" if detected else "#9aabba")
            table.add_row(Text(robot.name or "—"), robot.uri, status, key=robot.uri)
            self.row_uris.append(robot.uri)
        self.query_one("#fleet").border_subtitle = f"{len(self.row_uris)} drones · most recent first"
        self.query_one("#empty").display = not self.row_uris
        self.query_one("#choose", Button).disabled = self.scanning or not self.row_uris
        self.query_one("#rename", Button).disabled = self.scanning or not self.row_uris
        if selected_uri in self.row_uris:
            table.move_cursor(row=self.row_uris.index(selected_uri))

    def status(self, message: str, error: bool = False) -> None:
        self.query_one("#status", Static).update(message)
        self.query_one("#status").set_class(error, "error")
        self.record(("Error: " if error else "") + message)

    def set_scanning(self, value: bool) -> None:
        self.scanning = value
        for name in ("scan", "choose", "manual-use", "rename"):
            self.query_one(f"#{name}", Button).disabled = value
        self.query_one("#scan", Button).label = "Scanning…" if value else "Scan"
        if not value:
            self.query_one("#choose", Button).disabled = not self.row_uris
            self.query_one("#rename", Button).disabled = not self.row_uris

    @on(Button.Pressed, "#scan")
    def scan_button(self) -> None:
        self.action_scan()

    def action_scan(self) -> None:
        if not self.scanning and not isinstance(self.screen, RenameDialog):
            self.set_scanning(True)
            self.status(f"Scanning {len(self.picker.addresses())} known radio address(es)...")
            self.run_scan()

    @on(Button.Pressed, "#rename")
    def action_rename(self) -> None:
        if self.scanning or isinstance(self.screen, RenameDialog):
            return
        row = self.query_one(DataTable).cursor_row
        if not 0 <= row < len(self.row_uris):
            return
        robot = self.picker.robots[self.row_uris[row]]

        def save_name(name: str | None) -> None:
            if name is None:
                self.record(f"Rename cancelled: {robot.uri}")
                return
            try:
                renamed = self.picker.rename(robot.uri, name)
            except PickerError as exc:
                self.status(str(exc), error=True)
                return
            self.refresh_rows(selected_uri=robot.uri)
            self.status(f'Renamed "{robot.name or "Unnamed drone"}" → "{renamed.name}" · {robot.uri}')
            self.query_one(DataTable).focus()

        self.push_screen(RenameDialog(robot), save_name)

    @work(exclusive=True)
    async def run_scan(self) -> None:
        try:
            found = await asyncio.to_thread(self.picker.scan)
            self.picker.merge_scan(found)
            self.refresh_rows()
            self.status(f"Scan complete · {len(self.picker.detected)} drone(s) detected.")
            for robot in self.picker.rows():
                if robot.uri in self.picker.detected:
                    self.record(f"Detected: {robot.name or 'Unnamed drone'} · {robot.uri}")
        except PickerError as exc:
            self.status(str(exc), error=True)
        self.set_scanning(False)

    def choose(self, robot: Robot, source: str = "drone list") -> None:
        try:
            uri = self.picker.choose(robot)
            previous = self.picker.robots.get(uri)
            name = robot.name.strip() or (previous.name if previous else "") or "Unnamed drone"
            self.record(f"Selected ({source}): {name} · {uri}")
            self.exit(uri)
        except PickerError as exc:
            self.status(str(exc), error=True)

    @on(DataTable.RowSelected)
    def row_selected(self, event: DataTable.RowSelected) -> None:
        if not self.scanning:
            self.choose(self.picker.robots[str(event.row_key.value)])

    @on(Button.Pressed, "#choose")
    def choose_button(self) -> None:
        row = self.query_one(DataTable).cursor_row
        if 0 <= row < len(self.row_uris):
            self.choose(self.picker.robots[self.row_uris[row]])
        else:
            self.status("No drone selected. Scan for drones or enter a URI.")

    @on(Button.Pressed, "#manual-use")
    def manual_button(self) -> None:
        self.choose(Robot(self.query_one("#uri", Input).value, self.query_one("#nickname", Input).value), source="manual URI")

    @on(Button.Pressed, "#cancel")
    def cancel_button(self) -> None:
        self.action_cancel()

    def action_cancel(self) -> None:
        self.record("Cancelled · no drone selected.")
        self.exit(None)


cli = typer.Typer(add_completion=False)


@cli.command()
def pick(
    demo: bool = typer.Option(False, help="Use simulated drones; never access USB or radio."),
    history: Path | None = typer.Option(None, help="Override the saved-selection file."),
) -> None:
    """Choose a drone and print its radio URI."""
    try:
        picker = Picker(history or default_history(demo), demo=demo)
        app = DronePickerApp(picker)
        uri = app.run()
        app.print_summary()
        if uri is None:
            typer.echo("Selection cancelled.", err=True)
            raise typer.Exit(130)
        typer.echo(uri)
    except PickerError as exc:
        typer.echo(f"Error: {exc}", err=True)
        raise typer.Exit(1) from exc


if __name__ == "__main__":
    cli()
