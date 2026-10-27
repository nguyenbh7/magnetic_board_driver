# Safe Hall measurement logging

The desktop app records `MLXBF002` compact board-frame binary logs.

## Workflow

The desktop app groups the instructions, file-selection controls, current/last
filename, log status, and warnings inside a **Measurement logging** bordered
panel. Follow the numbered instructions shown at the top of the panel.

1. Select **Select File** and choose a **new** `.bin` filename. The app creates
   that file with the `MLXBF002` header, and refuses names that already exist.
2. Click **Start Field Stream** to begin acquiring data. The active sensor
   board frames are appended to the open log.
3. Click **Stop Field Stream** if you want to pause. With the log still open,
   **Start Field Stream** resumes appending to that same file.
4. Click **Stop logging** to finish the experiment. The app drains queued frames,
   flushes the output buffer, and closes the log.
5. For the next experiment, select a **new** filename before starting its stream.

**Important:** Starting the field stream with no file selected only displays
live data; it does not save a new file. **Stop logging does not stop the field
stream**; stop the stream separately as needed.

## Overwrite and loss protection

- An existing path is protected with Rust's atomic
  `OpenOptions::create_new(true)`. The application will display an
  **EXISTING FILE PROTECTED** notice and keep the old file unchanged.
- **Select File** is unavailable while a log is open or draining, and
  the async file dialog's completion handler independently checks that
  state. Two simultaneous selection dialogs are prevented.
- **Stop logging** is inactive when no log is open or after a stop was
  requested. Draining the pending queue finishes before an explicit
  flush and close.
- Disk-write and flush failures are displayed as errors. Any affected
  file requires integrity inspection before scientific use.
- The UI shows the current/last log path and log notices.

The native save dialog may also ask for overwrite confirmation. That
confirmation is *not* the app's security boundary; the atomic exclusive
open refuses existing files regardless of the dialog result.

## Suggested filenames

For traceability use a unique experiment identifier, date, and run number,
for example `GK_mannequin_20261027_run01.bin` and
`GK_mannequin_20261027_run02.bin`.

## Developer checks

Run the logging safety tests from the repository root:

```powershell
cargo test -p app logging_safety_tests
```

They verify that existing file bytes are never changed and that a new
recording begins with the correct eight-byte file header.

For a manual GUI smoke test, try selecting an existing disposable test file,
then a new path; start/stop the stream twice while leaving the new log open;
finally stop logging and confirm the file size is preserved when attempting
to select the old path again. Run this with disposable files, not lab data.

### Notes for operators

The offline parser expects every board frame to have a valid length-prefixed
Postcard payload. A log with only the `MLXBF002` header means no
board frames were recorded. A write failure in the middle of a frame can
leave a truncated final payload; check and retain such recordings for
diagnosis rather than treating them as valid experimental results.
