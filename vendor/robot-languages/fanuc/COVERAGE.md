# FANUC TP/LS grammar audit — R-30iB Basic /09

This is an implementation gap table, not a complete-language claim. Tests count regression cases; keyword/catalog counts do not measure grammar completeness. The selected reference is **FANUC B-83284EN/09, Basic Function Operator’s Manual**, covering R-30iB and the controller variants named on its cover. `/09` is a document revision, not a claim about a particular installed firmware version. Optional forms are separately tied to **B-83284EN-2/09**. KAREL is a different language and outside this TP/LS profile.

## Accessible primary material

Both full FANUC-authored manuals were accessible from the public GitHub mirror at pinned commit `9e9998c2744b94794414a372063410fa20507fc6`. The repository is a mirror, **not FANUC’s official host**. No authentication bypass was used. Exact URLs, byte sizes and SHA-256 are in [SOURCE.json](SOURCE.json). The manuals prohibit reproduction; PDFs, extracted text, diagrams and copied prose are not shipped. Descriptions and regression fixtures here are independently authored.

The primary material is available: remaining work must **not** be described as blocked by inaccessible manuals. Basic §8.10.5 supplies ASCII-file examples; chapter 4 and Appendix A.3 supply instruction-format diagrams. They do not provide a machine-readable complete LS EBNF, and pendant display punctuation occasionally differs from ASCII export. The remaining ASCII variants need explicit productions and positive/negative fixtures, not a vendor binary compiler as a prerequisite.

The complete numbered-chapter and utility inventory is recorded in [BASIC-INVENTORY.md](BASIC-INVENTORY.md).

## Basic /09 chapter coverage

“Parsed” below means the named source production is recognized and malformed forms have checks. It does not mean every operand type combination or controller-dependent variation has been exhausted.

| Manual section / printed pages | Implemented source productions | Specific remaining grammar gap |
| --- | --- | --- |
| §4.1 pp160–165; §8.10.5 pp518–523 | Names, reserved device names, Macro/Cond/Collection/Job/Process, common ATTR/TCD/date/group/control fields | Unknown application attributes remain explicit warnings. Collection is documented as 7DC3+; /09 is not a firmware number. |
| §4.2 pp165–166 | Numbered/unnumbered colon-prefixed statements, semicolons, continuations, strings and comments | Line-number ordering and actual controller line-count limits are not semantic validation. |
| §4.3 pp168–195; §9.19 pp665–682 | J/L/two-position C/one-position A, direct and indirect speed, units, FINE/CNT; **one C passing-point extra option**; ACC, Skip/captured PR, offsets, INC, EV/Ind.EV, ORNT_BASE | The documented J/cartesian Wjnt/MROT rules, A/S prohibited option families, CR/J/S distinction and TB/TA/DB bounds are checked. Runtime compatibility with installed axes/options is separate. ACC 101–500 has a warning: the body says 0–100 but Appendix A.3.2 says 0–500. |
| §4.4, §4.5.5, §10; §8.10.5 | PALLETIZING-B/BX/E/EX/END, PAL route references; PL vectors and add/subtract; CONFIG/BOTTOM/PATTERN/ROUTE AST, triple-index pallet P, nested groups, mode-specific fields, duplicates and required fields | **E/EX all-point arrangement enum spelling and auxiliary BOTTOM key serialization need a real ASCII export.** LINE and numeric spacing are parsed. Unknown arrangement words are preserved with an explicit serialization warning. B/E reject BX/EX PATTERN data. |
| §4.5.1–4.5.4 pp198–203 | Typed R/PR/PR-component/GP-qualified PR, LPOS/JPOS/UFRAME/UTOOL, nested indices; ordinary arithmetic max five operators and no mixed add/subtract with multiply/divide | Dynamic AR and system-variable types need their declaration/runtime context; they are not invented. |
| §4.5.6 pp205–207 | String/numeric conversion and ordered concatenation; STRLEN/FINDSTR/SUBSTR have typed targets, SR/AR sources, scalar start/length | Runtime string contents and conversion values are separate from the parsed source production. |
| §4.6 pp208–211 | I/O assignments, DO/RO PULSE, finite time, read-only operand rejection | Installed signal counts/wiring and controller pulse timing are configuration/semantics. |
| §4.7 pp212–239 | LBL/JMP; ordinary IF and mixed IF/THEN/ELSE/ENDIF; ten-level IF/FOR nesting; SELECT constants/R/AR/SR (Appendix A.3.7) and JMP/CALL-only actions; CALL max30 typed parameters/string max34, quoted argument-wizard descriptions with value and 7DC3 version metadata; parameterless RUN | Configured multiword macro labels need the controller macro dictionary. Pallet comparison diagrams and one example disagree on PL-vs-R; see evidence notes below. |
| §4.8–4.9 pp239–245 | Time/conditional WAIT, TIMEOUT, SKIP CONDITION; ordinary AND or OR max five comparisons, ON+/OFF-/ERR_NUM; mixed conditions reject edges/ERR_NUM | Runtime timeout/event behavior is not executed. |
| §4.10–4.13 pp246–250 | PAYLOAD plans and multi-GP mask; offsets; frame setup only whole PR, scalar frame selection | User/tool frame existence and installed group indices require controller configuration. |
| §4.14–4.15 pp250–255 | PAUSE/ABORT/END, RSR, UALM, TIMER, OVERRIDE, MESSAGE, system-variable structures and multi-index arrays, maximum speeds | System-variable mutability/declared type and synchronization are external semantic information. Arithmetic expressions cannot be assignment left sides. |
| §4.16–4.17; §9.14 | Parameterless RUN, independent/simultaneous distinct GP1–GP8 blocks with J/L clauses | Additional controller exports would corroborate the grouped ASCII layout; dynamic group configuration is separate. |
| §4.18 pp258–266 | FOR R with scalar TO/DOWNTO bounds; ENDFOR matching and nested limits | Jumping across blocks and runtime loop values are not evaluated. |
| §4.19 pp267–270 | Numeric/Boolean mixed arithmetic, comparisons, AND/OR/!, precedence; full positions/pallets/strings rejected in mixed expressions; conditional assignments/PULSE require mixed conditions | NOT/XOR and TRUE/FALSE are **not** silently accepted as /09 TP expression aliases; TRUE/FALSE remain valid APPL literals. The manual's approximate 20-item capacity depends on data representation, so no invented exact threshold is enforced. |
| §4.20 pp271–272 | DIAG_REC and DIAG_REC_SEC operand counts and scalar families | Both printed bracket forms accepted; actual controller export would resolve punctuation variants. |
| §8.10.5 pp518–523 | Ordered sections; common ATTR; Handling/Spot/Arc/Dispense/cycle APPL fields; full pallet trees above; P groups, Cartesian CONFIG, joint/linear axes, required commas and UF/UT | Other application-specific attributes require that application's manual/profile. They are preserved and warned individually. |
| §9.1 pp525–535 | Existing configured macros remain saved as unresolved statements | Need a macro label-to-program dictionary to distinguish arbitrary configured labels from misspelled instructions. No false successful validation of unknown macros. |
| §9.4–9.6 pp555–574 | LOCK/UNLOCK PREG; TB/TA/DB with CALL/output | TA is after completion. Runtime timing and all cross-option combinations are distinct work. |
| §9.7 pp575–578 | POINT_LOGIC marker recognized with a specific unresolved-data warning | **Nonempty child-program ASCII serialization not established by the manual or researched fixtures.** See evidence below. |
| §9.8 pp579–580 | Direct DO/RO/GO/AO options, max five, typed output assignment | Controller timing is not executed. |
| §9.9 pp581–588 | MONITOR / MONITOR END; Cond subtype only WHEN; typed numeric/Boolean/error comparisons, up to five same AND/OR conditions, CALL action | Monitored programs and their system configuration are external semantics. |
| §9.12 pp622–630 | F/M references, M mixed definition and empty clear; TC_ONLINE expression/ENABLE/DISABLE | Background-logic execution eligibility depends on an externally selected BG profile; source does not identify a running BG task. |
| Other Basic chapters | Operation/setup/UI, calibration, file utilities | Password XML, custom screens, KAREL and configuration files are distinct formats, not counted as TP grammar. |

## Optional /09 chapter audit

The following productions are implemented, with independently authored valid/invalid regression cases. An option notice records installation requirements; it does not make an ordinary recognized instruction an unknown grammar. The complete chapter inventory is recorded in [OPTIONAL-INVENTORY.md](OPTIONAL-INVENTORY.md); catalog size is not evidence of completeness.

| Optional section | Source grammar / boundary |
| --- | --- |
| §3 Softfloat | SOFTFLOAT schedule/END, FOLLOW UP, applicable motion option. Positive schedule identifiers parsed; values above the documented **usual** ten schedules get a configuration warning, not an invented universal error. |
| §4 Continuous rotation | CTV numeric/R motion option and documented range; no standalone CTV instruction invented. |
| §5 Operation group DO | **No new TP opcode**: Motion DO setup and existing DO/RO syntax. |
| §6 Error recovery | Indexed RESUME_PROG/MAINT_PROG, fixed program/CLEAR, CLEAR_RESUME_PROG, RETURN_PATH_DSBL; the RESUME_PATH_DSBL example spelling is retained with an explicit manual-disagreement warning. |
| §7 Remote TCP; §8 collision | RTCP option, COL DETECT, COL GUARD ADJUST including optional GP mask and sensitivity. |
| §13.1 Soft limits; §15 singularity avoidance; §17 local stop | **No new TP opcode**: Limit Set menu, per-program DETAIL switch, and configured hardware/I/O/system variables respectively. SOFTLIMIT/AVOID/LOCAL_STOP are not fabricated. |
| §14 Advanced constant path | AP_LD/RT_LD, including tight literal/R spellings; CR distance (not CNT percentage); PSPD integer; MAX_SPEED only L. |
| §16 Path switching | PS trigger distance/time plus optional mixed IF and assignment/TC_ONLINE, allowed operand families, no AR/full PR/strings/pulse, bounded formula operations. |
| §21 Data monitor | SAMPLE START schedule and SAMPLE END. |
| §27 Torque limit | TORQ_LIMIT percentage grammar. |
| §28 TCP speed | TCP_SPD scalar reference. |
| §31 Math J593 | SQRT/SIN/COS/TAN/ASIN/ACOS/ATAN/ATAN2/LN/EXP/ABS/TRUNC/ROUND, exact arity, only R/AR function arguments, degree units. |
| §32 Servo Tool Change | TOOL ATTACH/DETACH group operand. |
| §39 Touch Sensing | SEARCH START/END, TOUCH OFFSET/END, Search[±X/Y/Z] motion option. |
| §40 Touch Skip | Uses existing CALL SETSKCOL / CLSKP_Gn production. No TOUCH_SKIP opcode invented. |
| §41 MROT; §42 isolation teaching | MROT applies to J or Cartesian Wjnt; isolation teaching is a UI function, not an opcode. |
| §50 Stitch | Standalone STITCH[1..20] / STITCH END; rejected non-integer, out-of-range, indirect and motion-attached forms. Stitch interval equipment behavior is separate. |
| §62 Laser | PRE_LS CH/PO, LS/LE/LP named fields, POWER_CONTROL, SLOPEUP/DOWN, WIRE_CONTROL, MINUTE_OUTPUT_RQ, CALL/PR_CALL LS_PO, PR_CALL-only LS_TA, formal brackets and screenshot parentheses. Exact manual conflicts (LP I vs screenshot V; LE SOF omitted in slope screenshot) produce specific warnings. |
| §63 VIA | DEFAULT/FINE/CNT and named Z/RT_LD/AP_LD fields; SCARA/7DF1/20 scope; motion compatibility checked. |
| §65 Spline R904 | S target/speed/termination and documented compatible option families; INC/Skip/Search/Spot/Weave/CTV/CR, EV/Ind.EV, BREAK and direct signal options are rejected. Instructions reached through external calls and runtime look-ahead are separate semantic checks. |
| KAREL, external NC/telegram formats, dedicated ArcTool/SpotTool/iRVision/tracking application manuals | Separate languages/profiles; the Basic+Optional pair does not define their complete language. Select the actual application manual/software option before claiming coverage. |
| §66 Learning Path Control | §66.5 explicitly says learned LRN data is **not converted to ASCII** and is lost on LS export. The pendant-generated binary learning payload is not an omitted LS bracket production; no fictitious `LRN[n]` grammar is introduced. |
| Software v9.40 local registers | A separately versioned later extension, not asserted as a Basic /09 feature; count/metadata checked. |

## Precise remaining ASCII evidence

- **Point Logic:** Basic §9.7 gives pendant insertion/editing of nested non-motion statements but no nonempty LS serialization. Public code search found the pinned ONE Robotics `tp.rb` fixture with only a `POINT_LOGIC` marker, no payload. The inspected fanuc-lsp implementation also does not parse this payload. Neither proves a child-block grammar. Required input: a controller/ROBOGUIDE `.LS` with a nonempty point-logic block, plus software version and option, containing at least assignment and conditional instruction. Unknown source is retained; a marker cannot be reported as a fully checked child program.
- **Pallet E/EX:** Basic §8.10.5 establishes CONFIG/BOTTOM/PATTERN/ROUTE, LINE/fixed interval, and FIX/INTER. Chapter 10 explains all-point arrangement and auxiliary points on the pendant but does not establish the corresponding enum/key ASCII spelling. Required input: an E/EX `.LS` with all-point arrangement and an auxiliary bottom point. Do not invent a POINT enum or an AUX key.
- **Contradictory printed productions:** Basic §4.7.4's pallet-compare diagram permits PL or a triple vector, while its first example uses R. The implemented typed rule follows the diagram; that disputed PL-vs-R case needs an actual export/controller-version check. ACC and Optional laser ambiguities are separately diagnosed as described above.
- **Program STORAGE metadata:** Optional §29.3.3 explicitly adds a STORAGE item when exporting LS, but does not give its ASCII container or enum spellings. Public code searches combining STORAGE with MNEDITOR or DEFAULT_GROUP found no corroborating file; inspected fanuc-lsp/tpDev sources also do not establish it. Required input: a controller LS exported in CMOS/SHADOW/ONDEMAND/FILE storage modes, with controller version. Unknown fields/sections remain preserved with explicit unsupported diagnostics; menu labels are not guessed into a grammar.
- **Additional configuration placements:** Basic §4.3.2 describes an abstract four-placement `(F,L,U,T,turns)` representation; the dedicated ASCII example in §8.10.5 supplies a three-flag CONFIG string. The abstract diagram does not establish the LS spelling/order for an extra left/right flag. A four-flag-looking CONFIG remains preserved and explicitly unverified, not declared universally invalid. Actual controller ASCII/version evidence is required before supporting it as a verified production.
- **External configured names:** Macro labels, application attribute names, system-variable declarations and installed motion groups require the corresponding controller profile. Their absence is missing semantic/configuration input, not a missing license to parse ordinary TP.

The independently inspected public `DEP_PALLET.LS` at `depjr1985/RoboticsProgramming@76f3971e6dda1b3bcd23ce0c1c13dce14b872ed7` exercises a real B-mode CONFIG/BOTTOM/ROUTE tree and parses with zero syntax/unsupported diagnostics. It was used read-only and is not redistributed. This corroborates that format, not E/EX or Point Logic serialization.

## What is still needed for a complete selected ASCII profile

The inventories identify concrete source forms rather than keyword counts. Remaining serialization evidence is limited to the explicitly named cases above: nonempty Point Logic, all-point/auxiliary pallet data, STORAGE metadata, additional configuration placements and contradictory printed examples. Each needs a representative exported LS plus software/option version, or an authoritative ASCII specification. An unrecognized field or payload stays saved and visibly unverified.

A configured macro dictionary resolves external labels; system-variable declarations, dynamic values, called-program behavior, installed axes and option combinations resolve controller semantics. Those inputs are not a reason to leave a documented ordinary source production unimplemented. Additional ArcTool/SpotTool/Dispense/iRVision/tracking application languages require their own selected manual revision; they are not silently included in the Basic+Optional claim.

Binary `.TP` compilation, numerical execution, I/O simulation, controller semantics and certification are separate work. Their absence neither proves nor excuses a missing source production. Conversely, grammar success does not imply those capabilities.
