import asyncio
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

from textual.widgets import Button, DataTable, Input, Static
from typer.testing import CliRunner

from tools.drone_picker.core import (
    DEFAULT_ADDRESS, DEMO_RECENTS, Picker, PickerError, Robot,
    default_history, load_history, normalize_uri,
)
from tools.drone_picker.textual_ui import DronePickerApp, RenameDialog, cli

NEW_URI = "radio://0/90/2M/E7E7E7E703"


class TemporaryHistory:
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.history = Path(self.directory.name) / "robots.json"


class CoreTests(TemporaryHistory, unittest.TestCase):
    def test_normalize_default_and_scanned_addresses(self):
        self.assertEqual(normalize_uri(" radio://0/080/2m "), "radio://0/80/2M/E7E7E7E7E7")
        self.assertEqual(normalize_uri("radio://0/80/2M", 0xAA), "radio://0/80/2M/00000000AA")
        self.assertEqual(normalize_uri("radio://1/125/250k/aa"), "radio://1/125/250K/00000000AA")

    def test_invalid_uris(self):
        for uri in ("", "usb://0", "radio://0/126/2M", "radio://0/80/3M", "radio://0/80/2M/ZZ", "radio://0/80/2M;echo", "radio://0/80/2M?a=1"):
            with self.subTest(uri=uri), self.assertRaises(PickerError):
                normalize_uri(uri)

    def test_persistent_recency_and_deduplication(self):
        first = Picker(self.history, clock=lambda: 10)
        first.choose(Robot(DEMO_RECENTS[0].uri, "robot two"))
        second = Picker(self.history, clock=lambda: 20)
        second.choose(DEMO_RECENTS[1])
        third = Picker(self.history, clock=lambda: 30)
        third.choose(Robot(DEMO_RECENTS[0].uri))
        saved = load_history(self.history)
        self.assertEqual([robot.uri for robot in saved], [DEMO_RECENTS[0].uri, DEMO_RECENTS[1].uri])
        self.assertEqual(saved[0].name, "robot two")
        self.assertEqual(saved[0].last_selected, 30)

    def test_corrupt_history_is_not_overwritten(self):
        self.history.write_text("{broken")
        with self.assertRaisesRegex(PickerError, "Cannot read history"):
            Picker(self.history)
        self.assertEqual(self.history.read_text(), "{broken")

    def test_bad_record_is_an_actionable_error(self):
        self.history.write_text(json.dumps({"robots": [{"uri": "bad"}]}))
        with self.assertRaisesRegex(PickerError, "Invalid history"):
            Picker(self.history)

    def test_scan_uses_default_and_saved_addresses_without_saving_discoveries(self):
        Picker(self.history, clock=lambda: 10).choose(Robot("radio://0/80/2M/00000000AA", "my robot"))
        calls = []

        def scanner(addresses):
            calls.append(addresses)
            return [Robot("radio://0/80/2M/00000000AA"), Robot(NEW_URI)]

        picker = Picker(self.history, scanner=scanner)
        picker.merge_scan(picker.scan())
        self.assertEqual(set(calls[0]), {DEFAULT_ADDRESS, 0xAA})
        self.assertEqual(picker.rows()[0].name, "my robot")
        self.assertEqual(len(picker.detected), 2)
        self.assertEqual(len(load_history(self.history)), 1)

    def test_demo_never_calls_radio_backend_and_uses_separate_history(self):
        with patch("tools.drone_picker.core.scan_radios", side_effect=AssertionError("Hardware access")):
            picker = Picker(self.history, demo=True)
            picker.merge_scan(picker.scan())
        self.assertEqual(len(picker.rows()), 3)
        self.assertFalse(self.history.exists())
        self.assertNotEqual(default_history(True), default_history(False))

    def test_scan_error_keeps_recent_entries(self):
        def scanner(addresses):
            raise PickerError("No radio")

        picker = Picker(self.history, demo=True, scanner=scanner)
        before = picker.rows()
        with self.assertRaisesRegex(PickerError, "No radio"):
            picker.scan()
        self.assertEqual(picker.rows(), before)

    def test_save_failure_is_actionable(self):
        picker = Picker(self.history, demo=True)
        self.history.mkdir()
        with self.assertRaises(PickerError):
            picker.choose(DEMO_RECENTS[0])

    def test_rename_preserves_uri_recency_and_scan_name(self):
        Picker(self.history, clock=lambda: 10).choose(DEMO_RECENTS[0])
        Picker(self.history, clock=lambda: 20).choose(DEMO_RECENTS[1])
        picker = Picker(self.history)
        renamed = picker.rename(DEMO_RECENTS[0].uri, "  Lab drone  ")
        self.assertEqual(renamed.uri, DEMO_RECENTS[0].uri)
        self.assertEqual(renamed.last_selected, 10)
        self.assertEqual(renamed.name, "Lab drone")
        reloaded = Picker(self.history, scanner=lambda addresses: [Robot(renamed.uri)])
        reloaded.merge_scan(reloaded.scan())
        self.assertEqual(reloaded.robots[renamed.uri].name, "Lab drone")
        self.assertEqual(reloaded.rows()[0].uri, DEMO_RECENTS[1].uri)

    def test_rename_newly_detected_drone_saves_without_selecting(self):
        picker = Picker(self.history, demo=True, clock=lambda: 30)
        picker.merge_scan(picker.scan())
        picker.rename(NEW_URI, "New drone")
        saved = load_history(self.history)
        self.assertEqual(len(saved), 1)
        self.assertEqual((saved[0].uri, saved[0].name, saved[0].last_selected), (NEW_URI, "New drone", 0))
        picker.choose(Robot(NEW_URI))
        self.assertEqual(load_history(self.history)[0].name, "New drone")

    def test_rename_rejects_blank_and_handles_write_failure(self):
        picker = Picker(self.history, demo=True)
        with self.assertRaisesRegex(PickerError, "Enter a nickname"):
            picker.rename(DEMO_RECENTS[0].uri, " ")
        self.assertFalse(self.history.exists())
        with patch("tools.drone_picker.core.save_history", side_effect=PickerError("Cannot save history")):
            with self.assertRaisesRegex(PickerError, "Cannot save history"):
                picker.rename(DEMO_RECENTS[0].uri, "Unsaved name")
        self.assertEqual(picker.robots[DEMO_RECENTS[0].uri].name, "demo-02")


class TextualTests(TemporaryHistory, unittest.IsolatedAsyncioTestCase):
    async def test_rename_scanned_drone_persists_keeps_selection_and_reports(self):
        app = DronePickerApp(Picker(self.history, demo=True))
        async with app.run_test(size=(80, 24)) as pilot:
            await pilot.click("#scan")
            await app.workers.wait_for_complete()
            await pilot.pause()
            table = app.query_one(DataTable)
            table.move_cursor(row=app.row_uris.index(NEW_URI))
            await pilot.click("#rename")
            self.assertIsInstance(app.screen, RenameDialog)
            app.screen.query_one("#rename-name", Input).value = "Lab [A]"
            await pilot.press("enter")
            await pilot.pause()
            self.assertNotIsInstance(app.screen, RenameDialog)
            self.assertEqual(table.get_row_at(table.cursor_row)[0].plain, "Lab [A]")
            self.assertEqual(app.row_uris[table.cursor_row], NEW_URI)
            self.assertIn('"demo-03" → "Lab [A]"', app.session[-1])
            self.assertIn(NEW_URI, app.session[-1])
            await pilot.click("#cancel")
        saved = load_history(self.history)[0]
        self.assertEqual((saved.uri, saved.name, saved.last_selected), (NEW_URI, "Lab [A]", 0))

    async def test_rename_f2_escape_does_not_exit_picker_or_save(self):
        app = DronePickerApp(Picker(self.history, demo=True))
        async with app.run_test(size=(80, 24)) as pilot:
            await pilot.press("f2")
            self.assertIsInstance(app.screen, RenameDialog)
            self.assertEqual(app.screen.query_one(Input).value, "demo-02")
            app.screen.query_one(Input).value = "Discarded name"
            await pilot.press("escape")
            self.assertNotIsInstance(app.screen, RenameDialog)
            self.assertEqual(app.query_one(DataTable).get_row_at(0)[0].plain, "demo-02")
            self.assertFalse(self.history.exists())
            self.assertIn("Rename cancelled", app.session[-1])
            await pilot.click("#cancel")

    async def test_rename_validates_blank_and_displays_save_errors(self):
        app = DronePickerApp(Picker(self.history, demo=True))
        async with app.run_test(size=(80, 24)) as pilot:
            await pilot.click("#rename")
            app.screen.query_one(Input).value = " "
            await pilot.click("#rename-save")
            self.assertIsInstance(app.screen, RenameDialog)
            self.assertIn("Enter a nickname", str(app.screen.query_one("#rename-hint", Static).render()))
            app.screen.query_one(Input).value = "Unsaved name"
            with patch.object(app.picker, "rename", side_effect=PickerError("Cannot save history")):
                await pilot.press("enter")
                await pilot.pause()
            self.assertIn("Cannot save history", str(app.query_one("#status", Static).render()))
            self.assertEqual(app.query_one(DataTable).get_row_at(0)[0].plain, "demo-02")
            await pilot.click("#cancel")

    async def test_recent_first_and_selection(self):
        app = DronePickerApp(Picker(self.history, demo=True, clock=lambda: 100))
        async with app.run_test(size=(100, 28)) as pilot:
            self.assertEqual(app.query_one(DataTable).get_row_at(0)[1], DEMO_RECENTS[0].uri)
            await pilot.click("#choose")
        self.assertEqual(app.return_value, DEMO_RECENTS[0].uri)
        self.assertEqual(load_history(self.history)[0].uri, DEMO_RECENTS[0].uri)
        self.assertIn(f"Selected (drone list): demo-02 · {DEMO_RECENTS[0].uri}", app.session[-1])

    async def test_scan_adds_results(self):
        app = DronePickerApp(Picker(self.history, demo=True))
        async with app.run_test(size=(100, 28)) as pilot:
            await pilot.click("#scan")
            await app.workers.wait_for_complete()
            await pilot.pause()
            self.assertEqual(app.query_one(DataTable).row_count, 3)
            self.assertIn(NEW_URI, app.row_uris)
            await pilot.click("#cancel")
        self.assertFalse(self.history.exists())

    async def test_manual_validation_and_selection(self):
        app = DronePickerApp(Picker(self.history, demo=True))
        async with app.run_test(size=(100, 28)) as pilot:
            app.query_one("#uri", Input).value = "bad"
            await pilot.click("#manual-use")
            self.assertIn("Expected radio", str(app.query_one("#status", Static).render()))
            app.query_one("#uri", Input).value = ""
            await pilot.click("#uri")
            await pilot.press(*NEW_URI)
            app.query_one("#nickname", Input).value = "new robot"
            await pilot.click("#manual-use")
        self.assertEqual(app.return_value, NEW_URI)
        self.assertEqual(load_history(self.history)[0].name, "new robot")
        self.assertTrue(any("Error: Expected radio" in entry for entry in app.session))
        self.assertIn(f"Selected (manual URI): new robot · {NEW_URI}", app.session[-1])

    async def test_scan_error_is_shown_without_losing_history(self):
        def scanner(addresses):
            raise PickerError("No Crazyradio connected")

        app = DronePickerApp(Picker(self.history, demo=True, scanner=scanner))
        async with app.run_test(size=(100, 28)) as pilot:
            await pilot.click("#scan")
            await app.workers.wait_for_complete()
            await pilot.pause()
            self.assertIn("No Crazyradio", str(app.query_one("#status", Static).render()))
            self.assertEqual(app.query_one(DataTable).row_count, 2)
            await pilot.click("#cancel")

    async def test_cancel_is_available_while_scan_is_running(self):
        gate = threading.Event()

        def scanner(addresses):
            gate.wait(10)
            return []

        app = DronePickerApp(Picker(self.history, demo=True, scanner=scanner))
        try:
            async with app.run_test(size=(100, 28)) as pilot:
                await pilot.click("#scan")
                await pilot.pause()
                self.assertTrue(app.scanning)
                self.assertTrue(app.query_one("#scan", Button).disabled)
                self.assertTrue(app.query_one("#rename", Button).disabled)
                self.assertFalse(app.query_one("#cancel", Button).disabled)
                await pilot.click("#cancel")
                gate.set()
        finally:
            gate.set()
        self.assertIsNone(app.return_value)
        self.assertFalse(self.history.exists())
        self.assertIn("Cancelled", app.session[-1])


class TyperTests(TemporaryHistory, unittest.TestCase):
    def test_session_report_survives_ui_exit_and_keeps_stdout_uri_only(self):
        for cancel in (False, True):
            def run_session(app):
                async def interact():
                    async with app.run_test(size=(80, 24)) as pilot:
                        await pilot.click("#scan")
                        await app.workers.wait_for_complete()
                        await pilot.pause()
                        if cancel:
                            app.action_cancel()
                        else:
                            await pilot.click("#choose")
                asyncio.run(interact())
                return app.return_value

            with self.subTest(cancel=cancel), patch.object(DronePickerApp, "run", run_session):
                result = CliRunner().invoke(cli, ["--demo", "--history", str(self.history)])
            self.assertEqual(result.exit_code, 130 if cancel else 0, result.output)
            self.assertIn("session summary [DEMO]", result.stderr)
            self.assertIn("Scan complete · 2 drone(s) detected.", result.stderr)
            self.assertIn(NEW_URI, result.stderr)
            if cancel:
                self.assertIn("Cancelled", result.stderr)
                self.assertEqual(result.stdout, "")
            else:
                self.assertIn("Selected (drone list): demo-02", result.stderr)
                self.assertEqual(result.stdout.strip(), DEMO_RECENTS[0].uri)

    def test_help(self):
        result = CliRunner().invoke(cli, ["--help"])
        self.assertEqual(result.exit_code, 0)
        self.assertIn("--demo", result.output)

    def test_history_error_exits_before_opening_ui(self):
        self.history.write_text("{broken")
        with patch.object(DronePickerApp, "run", side_effect=AssertionError("Should not open UI")):
            result = CliRunner().invoke(cli, ["--demo", "--history", str(self.history)])
        self.assertEqual(result.exit_code, 1)
        self.assertIn("Cannot read history", result.output)


if __name__ == "__main__":
    unittest.main()
