# RAPID language sources and coverage

The shipped editor grammar is original TypeScript code in `src/programming/languages/rapid-parser.ts`. It is a recursive-descent syntax analyzer, not ABB's compiler or an imported keyword-only grammar. The runtime has no RAPID parser dependency.

## Official profiles

| Profile | Kernel reference | Instruction/function/type reference | Named reference entries |
| --- | --- | --- | --- |
| RobotWare 6 | 6.16.03, `3HAC050946-001` revision L | 6.16.04, `3HAC050917-001` revision AC | 691 = 367 instruction/statement + 223 function/operator + 101 type |
| RobotWare 7 | 7.21, `3HAC065039-001` revision K | 7.21, `3HAC065038-001` revision W | 665 = 344 instruction/statement + 227 function/operator + 94 type |

Default editor profile: RobotWare 7.21. `createRapidLanguage('6.16')` creates the separately versioned profile. A name count is evidence of **reference-name coverage**, not proof that every overload, option or controller behavior has been validated. Control-flow constructs/operators count as numbered entries in these manuals; they are not all callable APIs.

Official stable references:

- [RW6 kernel](https://search.abb.com/library/Download.aspx?DocumentID=3HAC050946-001&LanguageCode=en&Action=Launch), SHA-256 `eef8727f12189ab9bbd2850b36a846788c403c8609389692a27382b41abc2986`
- [RW7 kernel](https://search.abb.com/library/Download.aspx?DocumentID=3HAC065039-001&LanguageCode=en&Action=Launch), SHA-256 `ee3bff00e1fa726f8ac74cccef76c7c7ed780d97ccbe517205a6eac2ea072694`
- [RW6 reference, pinned PDF](https://library.e.abb.com/public/c74d669abbda47debfa11262540152d6/3HAC050917%20TRM%20RAPID%20RW%206-en.pdf), SHA-256 `0b027a9d7c569407edd3c74d0789e0fc97b688672b9e80b261878d775eda06b1`
- [RW7 reference, pinned PDF](https://library.e.abb.com/public/3399634bb95042bb8fe7632cbb8f2255/3HAC065038%20TRM%20RAPID%20RW%207-en.pdf), SHA-256 `e62e169817b9ab867403fdac557c69cc2f891003cba900041c87317cb283911f`

ABB retains copyright in its manuals. PDFs, prose descriptions, examples and syntax-summary pages are **not redistributed** here. The generated JSON retains only names, argument/type facts, synthetic call signatures and reference identifiers. Chinese explanations and snippets are authored for this project. Common APIs have operation-specific explanations; other entries provide an interface-family summary, named/type parameter facts and a link to the exact reference profile. They do not reproduce complete usage documentation.

## Module formats and encoding

RobotWare 7.1 and later use `.modx` for program modules and `.sysx` for system modules. These are UTF-8 RAPID text, not XML or archive containers. The RW7 editor profile lists these extensions first and retains legacy `.mod`/`.sys` for source editing. RW6 keeps `.mod`/`.sys` as its formats and warns when a modern extension is opened. Program description files such as `.pgf`, encrypted modules and RobotStudio projects are not ordinary module text and are not decoded by this editor.

The official basis is [RAPID overview, 3HAC065040-001](https://search.abb.com/library/Download.aspx?DocumentID=3HAC065040-001&LanguageCode=en&Action=Launch), RobotWare 7.21.2, revision T, §1.1.3 pp19–20. The inspected PDF SHA-256 is `cffc87a358019f8473f340714414bba43785004f007f735e4ab5415e6baacd28`. It explains that modern extensions tell the compiler to read UTF-8, and that manually converting legacy modules requires UTF-8 **without BOM**, not merely renaming a file. RW7 kernel revision K §16 p155 separately distinguishes stored UTF-8 source from the internal representation created when a program is loaded. RW6 kernel revision L §2.1 p19 describes its ISO 8859-1 character set.

The source I/O layer saves UTF-8 and preserves the original BOM and newline characters. The analyzer warns for a BOM in an RW7 modern module, or non-ASCII UTF-8 content retained under an RW7 legacy extension; ASCII legacy modules receive no encoding warning. These diagnostics do not silently rename files, strip bytes, or certify a controller-ready conversion. Diagnostic locations remain UTF-16 offsets, including a two-unit Unicode character.

On 2026-11-09, the following fixed public revisions were independently fetched and decoded as UTF-8. All four began with plain `MODULE` text, contained no BOM or NUL, and returned zero diagnostics from the RW7 core syntax analyzer. One `.sysx` retained 118 CRLF pairs. This verifies actual source-format compatibility, not EGM/Modbus option semantics or controller execution. Their source bodies are not vendored.

| Public source | Bytes | SHA-256 |
| --- | ---: | --- |
| [GoFa EGM Module1.modx](https://github.com/rparak/Unity3D_ABB_CRB_15000_GoFa_EGM/blob/8ac4fc10bfc6e2bb31b955248938f2e1104c6df7/ABB_RS/EGM/Basic/T_ROB1/Module1.modx) | 4833 | `69bd11b46c1c48456b596d6e4eb301e46e0c6466480dca40775ffbd0c3f0569e` |
| [Module_Rubik.modx](https://github.com/roboticslab-uc3m/gofa-rubik-solver/blob/52d1aff02b1728ce0b5ce689e0c1dd00df14b68b/rapid/Module_Rubik.modx) | 4139 | `96c5867456a6b6a4e3ede2e1be25c29e5d195e7e1fed5d036ceaa0f96c9518f9` |
| [ABB-RARO ASIModbusData.sysx](https://github.com/ABB-RARO/rapid-modbus-tcp/blob/667a030a767dff8619ff164bf3e93ab63b631b17/src/ModbusTCP/gripper/ASIModbusData.sysx) | 939 | `810923c3a8e101b36286477d09a7e13415df2c2600d74bf95c8d54596fa371c1` |
| [GoFa module_EGM.sysx](https://github.com/eliasbitsch/ABB_GoFa/blob/707f0241acbed5ce4e5d239ffd55cccf62067f45/rapid/module_EGM.sysx) | 3587 | `8876b46118654f91969741eb9e41b9f06af99d51df6da8447d9b8c3e2b0d2334` |

## Extractor attribution and reproducibility

The two `.mjs` extractor files are adapted from [ichbinmeraj/abb-rws-vscode](https://github.com/ichbinmeraj/abb-rws-vscode/tree/c213a07746432b359aa3b8c6ecfbf1b4e8b9c545), commit `c213a07746432b359aa3b8c6ecfbf1b4e8b9c545`, copyright 2026 Meraj Safari, Apache-2.0. Original `LICENSE` and `NOTICE` accompany them. We include no controller/RWS networking code and no runtime dependency on that extension.

Local modifications:

1. Accept en/em dashes in entry headings (otherwise `CamWaitLoadJob` disappeared).
2. Normalize typographic single quotes in formal syntax (`PoseInv` was otherwise lost).
3. Recognize either reference document number in page furniture.
4. Preserve the complete temporary syntax block until extraction; do not truncate at 500 characters.
5. Read multiline return-type declarations and the factual `Return value / Data type` field as a fallback (`StrFormat`, `GetSignalOrigin` and longer UI signatures).
6. Preserve that fallback when the formal syntax does not repeat the return type.
7. The wrapper supplies RW6 `SCWrite`'s argument shape from its Arguments section; that legacy entry has no formal Syntax section. Its ignored `ToNode` parameter retains the manual's generic `datatype` label instead of inventing a precise type.

Regenerate or check with Node and Poppler's `pdftotext` installed:

```sh
node scripts/generate-rapid-catalog.mjs --cache-dir /tmp/emulation-rapid-research
node scripts/generate-rapid-catalog.mjs --cache-dir /tmp/emulation-rapid-research --check
```

Missing PDFs are fetched from pinned URLs into the cache. SHA-256 is checked before extraction. The generator compares every numbered instruction/function/type heading with generated entries, fails on any omitted name/category/empty callable signature and checks the independently recorded category totals in tests. A changed remote PDF is a failure, not an implicit profile upgrade.

## Kernel-production coverage matrix

The matrix is keyed to kernel chapters and exercised by `tests/programming-rapid-parser.test.ts`. Tests use independently written valid programs and malformed variants, not samples copied wholesale from ABB. The parser accepts more than one module per source buffer.

| Kernel production family | Implemented legal forms | Invalid counterparts exercised |
| --- | --- | --- |
| §2 lexical elements | Latin-1 identifiers, case-insensitive words, decimal/exponent/radix literals, doubled quotes/backslashes, hex escapes, comments | Leading underscore/reserved/long identifiers, invalid radix/exponent, bad escape, unterminated string |
| §2 types | `LOCAL RECORD`, record members, `ALIAS`, type placeholders | Missing member or alias name; empty record; type after data |
| §3 data declarations | `VAR/PERS/CONST`, `LOCAL`, `TASK VAR/PERS`, multidimensional dimensions and aggregate initialization | Missing constant initializer, `TASK CONST`, trailing dimension comma, data after routine |
| §4 expressions | Logical/relational/arithmetic precedence, signed simple expressions, arrays/record access, aggregates, function calls, named/optional/conditional arguments | Missing operand, chained relation, empty aggregate, missing member, trailing/missing argument separator |
| §5 simple statements | Assignments, procedure calls, `%expression%` late binding, labels/GOTO, CONNECT, RETURN/RAISE, EXIT/RETRY/TRYNEXT | Invalid assignment target, missing label/target/WITH, malformed dynamic call, extraneous no-arg operand |
| §5 compound statements | IF/ELSEIF/ELSE, compact IF, FOR/STEP, WHILE, TEST/CASE/DEFAULT | Missing THEN/DO/TO/colon, mismatched terminator, compound compact-IF body |
| §6 routines | `LOCAL PROC/FUNC/TRAP`, typed return, required/optional/alternative params, `VAR/PERS/INOUT`, wildcard array parameters | Missing return type/name, malformed alternatives, concrete parameter dimension, nested routine |
| §6 handlers | Ordered BACKWARD (PROC only), ERROR with optional number list, UNDO | BACKWARD on FUNC, wrong handler order, trailing error-number comma |
| §9 modules | MODULE/ENDMODULE, all five module attributes, type→data→routine order | Invalid attribute/separator, module-level executable statement, wrong declaration order |
| Editor placeholders | All 13 kernel placeholders with non-executable warning | Unknown angle-bracket content remains syntax tokens/errors |
| RW7 version difference | BREAK and CONTINUE reserved statements | RW7 rejects them as identifiers; RW6 still permits user-defined names with those spellings |

Resource limits: 1,000,000 UTF-16 source units, 150,000 tokens, 128 recursive nesting levels, at most 200 diagnostics. Errors remain visible when limits are hit. Parent editor also enforces its byte limit and worker deadline. EOF line comments are deliberately accepted in the editor. Comments are lexically skipped, including between tokens, rather than diagnosed by their exact placement within kernel list productions.

## Explicit gaps and boundaries

- This is **core-language syntax coverage**, not a claim of full RobotWare compilation or execution equivalence. No static type inference, symbol linkage across modules/tasks, argument overload checking, constant evaluation, lvalue access-mode checking, control-flow reachability, I/O configuration or option licensing is implemented here.
- The named API set covers the listed main references. Separate option/application manuals (for example EGM and application-specific welding packages), user libraries and future RobotWare revisions are not part of that count.
- Signature extraction is verified structurally and by targeted tricky cases, not by invoking every API on a matching controller. Some manual signatures themselves contain inconsistencies; versioned source links remain available for review.
- The source editor retains Unicode text; RW6 source encoding/character availability and exact controller numeric/string limits still need controller validation. Parser identifier rules follow the kernel's Latin-1 letter set. RW7 Unicode strings/comments are accepted.
- Plain text modules only: encrypted modules and RobotStudio/container formats require a separate decoder. All editor placeholders warn; their acceptance does not make a program executable.
- Generated language documentation does not enable simulation of arbitrary instructions. The existing validated movement-program simulation remains a separate feature.

## Other grammar candidates reviewed

- `pevers/rapid` at `5f3a08f9d148ffd357daaa5b6718a626151a6bf5`, MIT: useful LALRPOP precedent, but examined code limits dimensions, misses LOCAL FUNC/TRAP forms and has lexical/conditional-argument gaps. Not incorporated.
- `antlr/grammars-v4/rapid` at `c306207ac870e7c5a95763b518ba3e3a1ca61409`, BSD-3-Clause: missing records/aliases, aggregates and error/backward/undo handlers. Not incorporated or represented as complete.
- `verbotics/vscode-abb-rapid`, MIT: TextMate highlighting, not a full validating parser. Not incorporated.
