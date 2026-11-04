/* User approved 80KiB on 2026-10-30 after the real 64KiB link failed.
 * Keep the 4096-byte stack and the previous 1320-byte gap below it:
 * 81920 - 4096 - 1320 = 76504. Includes all code/data/BSS/alignment.
 * Any further increase requires another explicit resource decision. */
ASSERT(__image_end <= 76504, "navigation image exceeds approved 76504-byte image limit")
