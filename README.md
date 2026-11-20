# Polyphony milestone (MSP432E401Y player, up to 3 notes at once)

Chords and overlapping melodies of up to three notes, in every instrument preset (sine, clarinet, trumpet, marimba).
A one-note-at-a-time song sounds exactly as before. **Built and tested on a PC with the real firmware source; not yet
run on the board.** The previous milestone is saved as `uart_tx_queue_milestone_presets_milestone.zip`.

Read in this order:

1. `POLYPHONY.md` — what changed and why, your teacher's idea, decisions, limits, how it was tested, how to build it.
2. `HARDWARE_TEST_GUIDE.md` — **section 12** is the board test for polyphony (sections 1 to 11 are the earlier tests, still valid).
3. `PI_SENDER_INTEGRATION.md` — for the Raspberry Pi / web app side: protocol, presets, preparing sheet-music MIDI.
4. `IMPLEMENTATION_SUMMARY.md`, `PRESENTATION_OUTLINE_AND_NOTES.md`, `TEST_RESULTS_RECORD.txt` — earlier milestones' documents.

Run all PC-side tests (no board, **plays no sound**): `cd test_tools` then `python run_polyphony_tests.py`
First check on the real board (this is what makes sound): `cd test_tools\board_tests` then `python board_hello.py --port COM5`

| Folder | Contents |
|---|---|
| `projects/gpiointerrupt_freertos` | the CCS application project (polyphonic) |
| `projects/freertos_kernel_glo` | FreeRTOS V10.2.1 library project, unchanged |
| `test_tools/` | the new polyphony tests and MIDI tools, the instrument table generator, earlier offline checks; `board_tests/` = PC sender, board test scripts, MIDI test files |
| `waveforms/` | the oscilloscope captures the instrument presets were fitted to |
| `pi_sender_reference/` | reference material for the Pi sender |
| `checkpoints/`, `docs_previous_milestone/`, `evidence/`, `dependencies/` | earlier milestones |

Open this folder in CCS Theia as its **own workspace**: its project names match the previous milestone's (see `POLYPHONY.md` section 9).
