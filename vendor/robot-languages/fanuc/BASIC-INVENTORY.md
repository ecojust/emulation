# Basic /09 complete chapter inventory

Reference: FANUC B-83284EN/09, the exact pinned PDF in SOURCE.json. This inventory covers every numbered chapter and appendix; the detailed source productions, regression tests and precise unresolved ASCII cases are in COVERAGE.md. Descriptions below are independently written summaries of scope. “No new TP production” means the chapter operates through menus, existing TP statements or another file/language; it does not assert that the physical function is simulated.

| Chapter | Inspected subject and source-language outcome |
| --- | --- |
| 1 | Product/manual scope. Identifies separate Arc B-83284EN-3, Spot B-83284EN-4 and Dispense B-83284EN-5 application manuals; their full languages are not defined by this Basic profile. |
| 2 | Robot/controller/pendant/software overview; no new TP production. |
| 3 | I/O mapping, automatic start, frames, alarms, working/interference areas, payload and clock configuration. Uses menus/system variables; TP productions corresponding to these settings are specified in chapter 4. |
| 4 | Full instruction-family inventory: motion, pallet, typed register/string/I/O, branch/call/argument-wizard, wait, skip, load/frame, control, timers/system variables, multi-group, FOR, mixed logic and diagnostics. Implemented families are enumerated in COVERAGE.md. The quoted argument-wizard ASCII serialization in §4.7.7.5 is implemented, not dismissed as pendant-only metadata. |
| 5 | Program creation/editing, searching, shifting, comments, protection and related views; editing UI operations do not add an executable TP grammar. Existing `!`, `--`, `//`, position and instruction forms are parsed. |
| 6 | Program start/test/backward/pause/recovery operations; execution semantics, not new TP source productions. |
| 7 | Program/position/register/pallet/alarm/status displays. Register values and system state are external data, not extra `/MN` opcodes. |
| 8 | File/media/transfer utilities and §8.10 ASCII loader. `/PROG /ATTR /APPL /MN /POS /END`, common attributes, application trees and multi-group positions parsed; specific Point Logic/pallet serialization evidence gaps remain named in COVERAGE.md. |
| 9 | Utility subchapters inventoried individually below; actual source functions separated from UI/configuration. |
| 10 | B/BX/E/EX pallet operations and configuration. Implemented CONFIG/BOTTOM/PATTERN/ROUTE, triple-index P and per-mode checks. All-point arrangement word and auxiliary bottom-point key lack verified ASCII spelling; actual exported LS needed. |
| 11 | Pendant browser/layout/screens, key assignments, operation panels. Panel/menu settings are not TP source instructions. |
| 12 | iHMI setup/teaching/run/utility screens; no new TP source production. |
| 13 | Operation without a standard pendant, including iRProgrammer/JITP/JCGTP UI; uses existing programs/instructions. |
| 14 | Operation without a physical mode switch; control/configuration, no new TP opcode. |
| 15 | Tablet pendant operations and UI; no new TP opcode. |
| Appendix A | Complete compact TP instruction diagrams cross-checked against chapter 4; amendments include SELECT R/AR/SR values, conditional CALL arguments, UFRAME/UTOOL operands and ACC ambiguity. Descriptive syntax summaries occasionally omit required LS punctuation; actual ASCII examples take precedence for serialization. |
| Appendix B | Start modes, mastering, robot version/status, diagnostics and axis/hardware setup; controller operations, no new TP source language. |
| Appendix C | System-variable catalog. `$name[index,...].$field` source form parsed; a variable's installed existence, declared type and writability depend on the controller. This is a semantic/profile boundary, not arbitrary-expression assignment permission. |
| Appendix D | Collecting research/diagnostic files; no new TP production. |

## Complete chapter 9 utility inventory

| Subchapter | Source-language result |
| --- | --- |
| 9.1 | Configured macros, including CALL-style argument values. The label-to-program dictionary is external; unresolved labels retain source and a specific warning. No fake opcode vocabulary derived from arbitrary example labels. |
| 9.2–9.3 | Shift/frame-change editing tools transform existing program data; no extra TP statement. |
| 9.4 | LOCK PREG and UNLOCK PREG implemented. |
| 9.5–9.6 | TB/TA/DB source actions, units and literal bounds implemented; calls, outputs and Point Logic entry are distinguished. |
| 9.7 | Point Logic entry parsed. Nested non-motion child-program LS serialization is not provided by the pendant illustrations; requires a nonempty exported file. |
| 9.8 | Direct output motion options and max-five limit implemented. |
| 9.9 | MONITOR/END and Cond-only WHEN implemented, including typed comparisons, up to five same AND/OR conditions and fixed CALL target. |
| 9.10 | Auxiliary-axis collision monitoring configuration; no new TP opcode. |
| 9.11 | Password/user/USB/GI login configuration. KAREL helpers and password files are separate formats; ordinary CALL/RUN can reference helper programs. |
| 9.12 | Background task registration is external configuration. F/M references, M mixed-expression definition/empty clear, TC_ONLINE expression/ENABLE/DISABLE implemented. Whether a program is assigned to BG and its restricted execution eligibility is not inferable from one LS file. |
| 9.13 | Path resumption settings and runtime behavior; no new TP instruction. |
| 9.14 | RUN and multi-task/group program configuration; source forms implemented, scheduling not executed. |
| 9.15–9.17 | Alarm-severity configuration, diagnostic logs and controller web tools; no new TP grammar. |
| 9.18 | Group-mask exchange editing tool; no new executable statement. |
| 9.19 | A circle-arc line and documented motion-option compatibility. Runtime look-ahead restrictions across called programs are semantic/cross-file checks, not a missing line production. |
| 9.20 | Vision-assisted mastering workflow; no new TP production from this Basic section. |
| 9.21 | KAREL overview, registration and CALL integration. KAREL itself is a separate language and not the chosen TP/LS profile. |
| 9.22 | Software operation panel; UI/control configuration, not an extra TP instruction. |

## Source ambiguity remaining after inventory

The manual is accessible in full; no Basic chapter is still awaiting a generic inventory pass. Exact unresolved serialized data and contradictory printed examples are recorded in COVERAGE.md. They require controller exports/version evidence, not a binary compiler merely to continue ordinary grammar work. The source editor does not execute background tasks, macros, I/O or motion-option timing.
