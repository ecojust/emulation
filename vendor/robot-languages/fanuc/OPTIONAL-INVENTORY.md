# FANUC Optional /09 chapter inventory

This inventory covers all 66 numbered chapters of **B-83284EN-2/09**, the Optional Function Operator's Manual. Chapter numbers and page numbers below refer to the printed manual, not the PDF viewer's page counter. The [fixed public mirror](https://github.com/zhaomengkang/FANUC-Robot-Manual/blob/9e9998c2744b94794414a372063410fa20507fc6/Manual/eDoc/R-30iB%20Plus/content/B-83284EN-2_REV_09/book.pdf) and its recorded hash in [SOURCE.json](SOURCE.json) identify the evidence. The PDF and extracted text are not redistributed. This table records source productions and their boundaries; it is not a claim that the application implements the controller options or verifies their runtime behavior.

The inventory distinguishes new TP productions from existing calls, assignments and attributes; pendant operations do not automatically introduce LS keywords. KAREL programs called by TP still use the existing `CALL` grammar. Their implementations, KAREL built-in routines, XML/network packets and NC programs are separate formats. Generic macros require the controller's alias configuration; a displayed macro name is not evidence of a new built-in instruction.

Implementation locations:

- [fanuc-parser.ts](../../../src/programming/languages/fanuc-parser.ts): Basic grammar, shared expressions/attributes/motion structure and common Optional forms.
- [fanuc-optional.ts](../../../src/programming/languages/fanuc-optional.ts): independently implemented Optional statement, speed and motion-option productions.
- [fanuc-optional-laser.ts](../../../src/programming/languages/fanuc-optional-laser.ts): §62 laser productions.
- [programming-fanuc-optional.test.ts](../../../tests/programming-fanuc-optional.test.ts): authored valid/invalid source examples through the real LS parser, including operands and statement-versus-motion context.
- [COVERAGE.md](COVERAGE.md): shared Basic/Optional source-evidence gaps and controller-profile boundaries.

## Chapters 1–20

| Chapter / printed pages | Source production or classification | Implementation / boundary |
| --- | --- | --- |
| 1 Preface, p1 | Scope, related documents and operating prerequisites. | No TP production. |
| 2 Overview, p3 | Optional-function overview. | No TP production. |
| 3 Softfloat, p5 | `SOFTFLOAT[schedule]`, `SOFTFLOAT END`, `FOLLOW UP`; joint Softfloat also permits a motion modifier. | Parsed. Schedule must be a positive integer; the text says **usually** ten schedules, so a higher number requires a profile warning rather than a fabricated universal maximum. Cartesian Softfloat's different runtime scope remains explicit. |
| 4 Continuous rotation, p16 | `CTV` motion option with signed percentage or indirect R; §4.5 uses `CALL CALIBNBT(group,axis)`. | CTV grammar parsed; calibration reuses typed CALL arguments. Motor/group configuration and rotation are not executed. |
| 5 Operation-group DO, p25 | Motion DO setup screen and DO/RO assignments. | Existing I/O grammar; no invented `GROUP_DO` instruction. |
| 6 Automatic error recovery, p27 | Indexed `RESUME_PROG` / `MAINT_PROG`, `CLEAR_RESUME_PROG`, `RETURN_PATH_DSBL`; five recovery schedules. | Parsed. The printed example's `RESUME_PATH_DSBL` spelling is preserved with a manual-conflict warning; recovery execution depends on controller tasks and settings. |
| 7 Remote TCP, p51 | `RTCP` motion option. | Parsed option; external TCP/frame/controller semantics are not inferred. |
| 8 High-sensitivity collision detection, p56 | `COL DETECT`, `COL GUARD ADJUST`, optional GP mask and literal/R sensitivity. | Parsed, including omitted sensitivity where documented. This is controller collision-detection configuration, not a mesh-collision guarantee. |
| 9 Load estimation, p62 | Estimation/calibration screens, measurement poses and system settings. | No additional TP opcode. |
| 10 Payload over / setting confirmation, p72 | Confirmation menus, signals and settings. | No additional TP opcode. |
| 11 Gravity compensation, p76 | Compensation setup and calibration. | No additional TP opcode. |
| 12 Operation log book, p80 | Logging menus, variables and log files. | Log records are not LS instructions. |
| 13 Program toolbox, p93 | §13.1 Limit Set menu. | No `SOFTLIMIT` production; range-setting operations are controller configuration. |
| 14 Advanced constant path, p96 | `AP_LD` / `RT_LD` including glued constants or R, `CR` termination, `PSPD`, `max_speed` in the speed field. | Parsed. Linear Distance is L-only; CR's documented range is 0–1000 mm. PSPD is not restricted to L. §14.4.1 says unsupported max-speed runtime combinations disable that feature; this is distinct from malformed source. |
| 15 Singularity avoidance, p106 | Jog indication and per-program DETAIL switch. | No `AVOID` instruction. Runtime controller singularity handling is not simulated by recognizing a keyword. |
| 16 Path switching, p108 | `PS distance mm time sec, [IF(condition),] assignment` / `TC_ONLINE(condition)`. | Parsed with documented operand families and bounded per-statement operations. Cross-line/call execution timing and aggregate controller limits remain semantic checks. |
| 17 Auxiliary-axis servo off / local stop, p114 | Hardware, system variables, mapped I/O and examples using DO/WAIT/CALL/RUN/macros. | Existing productions; no `LOCAL_STOP` opcode. |
| 18 Dual motor drive, p125 | Hardware and controller variable setup. | No additional TP opcode. |
| 19 Multi-UOP interface, p129 | UOP mapping and program-start interface; existing RUN/RSR operation. | No additional TP opcode. |
| 20 Error-code output, p144 | Alarm-code DI/DO mapping and variables. | No additional TP opcode. |

## Chapters 21–40

| Chapter / printed pages | Source production or classification | Implementation / boundary |
| --- | --- | --- |
| 21 Data monitor, p149 | §21.3 `Sample Start[schedule]` / `Sample End`; §21.2 defines five schedules. | Parsed, schedule 1–5. Sampling configuration, files and displays are not TP grammar. |
| 22 Brake check, p167 | §22.2 example uses `$BCCFG.$BCK_START=1`, `WAIT $BCCFG.$BCK_START=0`, ordinary J/L and time WAIT. | Existing system-variable assignment and WAIT grammar; no `BRAKE_CHECK` opcode. Controller brake tests are not performed. |
| 23 Panel Wizard, p170 | Panel/control creation and running KAREL via configured buttons. | UI configuration, not additional LS grammar. |
| 24 Enhanced mirror image, p196 | Program/position transformation menus: parallel, rotated, frame-plane and extended-axis mirror operations. | Editor operations on existing point data, not a `MIRROR` instruction. |
| 25 Customize support, p211 | KAREL start/abort/auto-run configuration and custom menus. | Controller setup; KAREL remains a separate language. |
| 26 KAREL execution-history record, p221 | Event logging, KAREL example and ASCII event-report format; §26.4.3 TP example uses J, WAIT, R assignment, LBL and IF/JMP. | Existing TP productions; ASCII log files are not LS programs. |
| 27 Torque limit, p240 | §27.1 standalone `TORQ_LIMIT percent%`; §27.2 `CALL TPTRQLIM(group,axis,limit)`. | Parsed literal 0–100 percent form; axis-specific call uses existing CALL grammar. Device-dependent effective limits are separate. |
| 28 TCP speed output, p244 | `TCP_SPD` controller speed-output configuration production. | Parsed by the shared parser; actual I/O output and scheduling are not implemented. |
| 29 TP DRAM/file storage, p249 | CMOS/SHADOW/ONDEMAND/FILE storage; §29.3.3 explicitly adds a `STORAGE` item when saving LS. | No new /MN opcode. The manual names the LS metadata item but does not show its serialized container/value spelling; a real LS export with storage metadata is needed. Existing calls and storage menus are distinct from that specific source-evidence gap. |
| 30 Cycle-time logging, p259 | Display/log export; §30.3 example includes existing motion, WAIT and application SPOT. | Logging adds no TP production. Full SpotTool behavior needs its application profile/manual. |
| 31 Math function instruction, p267 | Optional numeric functions and their documented argument shapes, including two-argument ATAN2. | Shared typed expression grammar; R/AR operands are kept distinct from strings, positions and conditions. |
| 32 Servo tool change, p278 | `TOOL ATTACH` / `TOOL DETACH` and setup of tool/group data. | Parsed. Physical tool matching, axes and servo behavior require controller configuration. |
| 33 Operation without SHIFT, p306 | Jog/test-execution button sequence and conditions. | No additional TP opcode. |
| 34 High-speed SHIFT, p309 | T1 key behavior, override system variables and jog/test operation. | No additional TP opcode. |
| 35 iRDiagnostics, p312 | Diagnostic program generation, reducer/servo diagnosis and motion profiler. | Existing programs/settings. ACC advice does not introduce a new instruction. |
| 36 Menu Utility, p325 | Five **macros**: Prompt Box Msg(menu), Prompt Box YN(menu,answer_reg), List Menu(menu,answer_reg), Status Menu(menu), Op. Entry Menu(menu). | Existing macro-call structure and typed call arguments. Menu can be a number/name; the controller's registered macro alias/program mapping is required. This chapter does not establish a new LS built-in for the display labels. Unknown aliases remain visibly unverified. |
| 37 4D graphics, p340 | Models, collision visualization, node maps and full-screen display. | Controller UI configuration, not new TP syntax. |
| 38 Robot-to-robot data transfer, p359 | §38.4 TP `CALL RGETNREG/RSETNREG/RGETPREG/RSETPREG(...)`; §38.6 KAREL remote-access built-ins. | Existing typed CALL grammar. KAREL return values, built-in declarations and networking are separate; do not treat KAREL examples as TP assignments. |
| 39 Touch sensing, p386 | §39.5 `Search Start[schedule or R] PR[...]`, `Search End`, `Touch Offset PR[...]`, `Touch Offset End`, and final motion option `Search[±X/±Y/±Z]`. | Parsed, schedule 1–32 or indirect R. Search must be the last motion option; actual direction frame/speed/end conditions come from configured schedules. |
| 40 Touch skip, p418 | §40.2 `CALL SETSKCOL(s,g)` / `CALL CLSKP_Gg`. | Existing CALL production; no invented `TOUCH_SKIP` opcode. |

## Chapters 41–66

| Chapter / printed pages | Source production or classification | Implementation / boundary |
| --- | --- | --- |
| 41 MROT, p420 | Motion option for Joint motion or Cartesian motion with WJNT. | Parsed. WJNT presence is checked after the clause, not by inventing a mandatory lexical order. A joint-defined destination's different runtime effect is documented, not a reason to reject its syntax. |
| 42 Robot isolation, p422 | Isolation switches, DCS/RPI interaction and teaching lock. | Configuration and safety-device operation, not new TP syntax. |
| 43 Anti-deflection, p425 | §43.2 example uses `CALL GETBQCUR(group)`, `CALL CALTANG(group,pr)`, Offset PR, DO and WAIT. | Existing productions. KAREL variables/compensation calculations are separate and not executed. |
| 44 Interface panel, p428 | Button/lamp/display properties, state conditions, mapped registers/I/O and panel backup. | Controller interface configuration. References to F/M and I/O reuse Basic data types; no new statement. |
| 45 Special jog sequence, p458 | SHIFT/jog key sequence and `$JOG_SP_MODE` / `$SCR.$COORDOVRD` setup. | No additional TP opcode. |
| 46 Enhanced motion editing, p460 | Inserting/deleting standard motion and converting stored speed/position data. | Pendant editing operations on existing statements, not new productions. |
| 47 Finishing package, p471 | Package bundling the functions of chapters 45 and 46. | No additional TP opcode. |
| 48 Joint-position output, p472 | `$SV_GUN_CTRL[n]` group/axis/GO/scale setup; up to five configured outputs. | Controller variables and GO data; no new statement. |
| 49 Expanded registers, p474 | Configured R/PR capacities, NUMREG/POSREG variable-file backup and restore. | Existing register grammar. Counts depend on installed groups/options; do not hard-code the unexpanded default as a lexical maximum. |
| 50 Stitch, p477 | §50.1.1 standalone `STITCH[i]` and `STITCH END`; §50.1.2 gives condition numbers 1–20. | Parsed, including schedule bounds and standalone-only context. Signal timing, L/C-only stitch intervals and equipment settings are runtime semantics. |
| 51 Visual diagnostics, p485 | §51.1 custom-logic program attributes: BUSY_LAMP_OFF=3, ABORT_REQUEST=3, PAUSE_REQUEST=7 and no motion-group mask. | Existing LS attribute/TCD fields. The chapter lists values, not a different serialized attribute container. Display state expressions/configuration are not TP statements. |
| 52 PDF viewer, p497 | Opening and navigating PDF documents. | No TP production. |
| 53 Help/diagnostics, p499 | DIAG/HELP key, alarm help and menus. | No TP production. |
| 54 Maintenance reminder, p505 | Setup, records, ZDT/iRConnect and XML maintenance records. | Controller/service data, not LS instructions; external networking setup is referenced separately. |
| 55 HMI communication, p518 | Modbus/OPC-UA data maps, register/position/string/variable addresses and dynamic assignment. §55.4 CLRASG/SETASG are text commands written to holding-register address 16385 or the OPC-UA Command node. | Network commands and controller configuration, not TP statements. Existing LOCK PREG is referenced; KAREL-variable address strings are not new TP expression syntax. |
| 56 Fault/incident report, p558 | Alarm incident/time graphs and display operation. | No TP production. |
| 57 CORETEC bead inspection, p560 | §57.3 CALL CORE_START/CORE_END, RUN, MONITOR and a WHEN/edge-condition example. | Existing Basic productions; KAREL inspection services and Ethernet behavior are not implemented. |
| 58 RSI interface, p564 | §58.3 TP CALL RSI_CREATE/ON/WRXML/OFF/DELETE; KAREL `_I` built-ins and XML UDP telegrams. | Existing CALL grammar. KAREL signatures, XML files and remote-sensor protocol are separate formats. |
| 59 Shape generation, p571 | Menu-generated shape programs, process subprograms and templates; examples use ordinary J/L/C, frames, calls and labels. | Generates existing TP grammar; no `SHAPE` opcode. Application-process behavior needs its application profile. |
| 60 iRCalibration Signature, p587 | ACCU_START/ACCU_END/CALB_START/CALB_END and measurement routines called as KAREL programs. | Existing CALL grammar. Calibration menus, files, measurement hardware and correction parameters are not a new language production. |
| 61 NC program conversion, p607 | Configurable O/N/G/F/position/K/M-code input; §61.5 provides converted LS output using existing TP productions. | NC is a **separate input language**. This LS profile does not parse or convert G-code, nor pretend the NC keyword set is TP syntax. |
| 62 FANUC fiber laser, p621 | §62.6 PRE_LS/LS/LE/LP; §62.8 POWER_CONTROL; §62.9 SLOPEUP/SLOPEDOWN; §62.10 WIRE_CONTROL; §62.11 MINUTE_OUTPUT_RQ; §62.12 CALL/PR_CALL LS_PO; §62.13 PR_CALL LS_TA. | Parsed with named-field/arity/range/context checks. Device settings, oscillators, I/O and feedback are not executed. Printed form disagreements are preserved with diagnostics, detailed below. |
| 63 VIA, p704 | VIA(DEFAULT), VIA(FINE,Z=...), VIA(CNT,Z=...,RT_LD=...,AP_LD=...). | Parsed with the documented fields and motion-type restrictions. Planned via paths are not simulated by the source parser. |
| 64 CRW shape generation, p706 | iHMI generates point/path welding programs using §62 laser and existing CALL/RUN. | No additional opcode. Actual Compact Remote Welder setup and application program contents need the installed equipment/profile. |
| 65 Spline motion, p711 | S motion, speed/termination and §65.9 supported motion-option table. | Parsed S statements and documented option restrictions; controller spline interpolation and runtime blending are not simulated. See the exact table boundary below. |
| 66 Learning Path Control, p722 | Learning menu automatically records LRN motion-modifier data in TP programs. | **§66.5 explicitly says ASCII LS does not convert learned data and conversion loses it.** No ASCII `LRN[n]` production is invented. An actual serialized extension would need its own version/export evidence. |

## Specific compatibility and source-evidence boundaries

**Spline options.** §65.9 explicitly lists WJNT, ACC, PTH, Offset PR, Tool_Offset PR, RTCP, timed/distance CALL (TB/TA/DB), Arc Start, SS/SE, COORD, PSPD, VOFFSET and WJNT MROT. It lists INC, Skip, high-speed Skip, SPOT, Weave, Search, CR and CTV as unavailable. The table is a statement about S motion; it must not be applied indiscriminately to A/C or used to invent all application-specific argument grammars. The Basic manual's explicit A/C support, such as ORNT_BASE, must be considered separately. Arc/dispense profiles still require their application syntax and configuration.

**MROT.** §41 describes J, or Cartesian motion with WJNT, and shows `WJNT MROT`. It does not establish that reversing the textual order is illegal. It distinguishes Cartesian destinations from joint-defined destinations at runtime. Circular passing-point versus destination clauses are kept separate by the parser; cross-clause inheritance is not established by this chapter.

**Macros.** §36 identifies the five menu helpers as macros. Basic /09 §9.1 and §4.7.6 govern their registration and arguments. The controller's macro mapping is missing configuration, not justification for treating every unknown phrase as a valid built-in. Unresolved macro source remains preserved and visibly unverified.

**Laser.** §62 contains both bracket-form diagrams and parenthesized CALL examples. The parser accepts the documented variants. LP's field diagram uses I while an example uses V; that alias is warned. LE's formal form has SOF while the slope example omits it; omission is warned. Limits absent from this manual are not fabricated from likely device settings. LS_TA is PR_CALL-only and its stated conflict with slope controls does not automatically prohibit LS_PO.

**LRN / ASCII.** §66.5 is an explicit serialization limitation, not a temporarily unreviewed chapter. Controller-generated binary TP learned payload is outside an LS text grammar. Source containing an unestablished LRN textual extension remains preserved and diagnosed rather than silently accepted or stripped.

**Storage metadata.** §29.3.3 explicitly identifies a new LS `STORAGE` item, but does not provide the complete ASCII line/container or enumerate serialized value tokens. The UI's CMOS/SHADOW/SHADOW ONDEMAND/FILE labels alone do not establish their exact LS encoding. A controller LS export saved from SHADOW, SHADOW ONDEMAND and FILE storage, together with its software revision, would resolve this boundary. The inspected fanuc-lsp/tpDev sources and public code searches combining STORAGE with MNEDITOR or DEFAULT_GROUP yielded no usable specimen. Unknown attributes/sections are retained and diagnosed; this is not silently treated as checked syntax.

**Stitch regression coverage.** The chapter audit found STITCH after the initial Optional parser pass. Authored regressions cover valid schedules 1 and 20, the end form, invalid range/non-integer/indirect schedule operands and motion-attached use. S-attached laser variants also have explicit negative cases based on §65.9; the examples showing L in §62 are not used to invent a blanket J/A prohibition.
