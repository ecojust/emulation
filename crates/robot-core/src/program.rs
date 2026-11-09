//! Controller-independent motion IR and controller-specific source-file generation.
//!
//! Callers submit a planning request, never a client-supplied "successful" plan.
//! Only the planner can construct a validated program accepted by a postprocessor.
use crate::planner::plan_move_l_with_cancel;
use crate::types::{FailureCode, JointAngles, PlanRequest, PlanResult, Pose, RobConf};
use serde::{Deserialize, Serialize};
use std::fmt::{self, Write};
use std::sync::atomic::{AtomicBool, Ordering};

const ABB_MODELS: &[&str] = &["irb1100-4-0.58", "irb1100-4-0.475"];

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum RobotBrand {
    Abb,
    Fanuc,
    Kuka,
    Yaskawa,
}

/// Language profiles are distinct from robot geometry and a validated controller version.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ControllerTarget {
    AbbRapid,
    FanucLs,
    KukaKrl,
    YaskawaInform,
}

impl ControllerTarget {
    pub fn brand(self) -> RobotBrand {
        match self {
            Self::AbbRapid => RobotBrand::Abb,
            Self::FanucLs => RobotBrand::Fanuc,
            Self::KukaKrl => RobotBrand::Kuka,
            Self::YaskawaInform => RobotBrand::Yaskawa,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub struct ControllerCapability {
    pub brand: RobotBrand,
    pub controller: ControllerTarget,
    pub display_name: String,
    pub export_supported: bool,
    pub supported_model_ids: Vec<String>,
    /// Proposed source formats for unsupported adapters, not evidence of compatibility.
    pub file_extensions: Vec<String>,
    pub note: String,
}

pub fn controller_capabilities() -> Vec<ControllerCapability> {
    [
        (
            ControllerTarget::AbbRapid,
            "ABB RAPID",
            true,
            &["mod"][..],
            "支持 IRB1100 标称运动学模块生成；控制器版本、工具和工作站仍需离线复核。",
        ),
        (
            ControllerTarget::FanucLs,
            "发那科 FANUC",
            false,
            &["ls"][..],
            "尚未适配机器人模型、配置数据与控制器版本；不生成 LS/TP 文件。",
        ),
        (
            ControllerTarget::KukaKrl,
            "库卡 KUKA",
            false,
            &["src", "dat"][..],
            "尚未适配机器人模型、S/T 姿态配置与 KSS 版本；不生成 KRL 文件。",
        ),
        (
            ControllerTarget::YaskawaInform,
            "安川 YASKAWA",
            false,
            &["jbi"][..],
            "尚未适配机器人模型、脉冲/直角坐标与控制器版本；不生成 INFORM 文件。",
        ),
    ]
    .into_iter()
    .map(
        |(controller, name, export_supported, extensions, note)| ControllerCapability {
            brand: controller.brand(),
            controller,
            display_name: name.into(),
            export_supported,
            supported_model_ids: if export_supported {
                ABB_MODELS.iter().map(|id| (*id).into()).collect()
            } else {
                vec![]
            },
            file_extensions: extensions.iter().map(|value| (*value).into()).collect(),
            note: note.into(),
        },
    )
    .collect()
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct ProgramExportRequest {
    pub program_name: String,
    pub controller: ControllerTarget,
    pub motion: PlanRequest,
}

/// Brand-specific posture data stays in an explicit adapter payload, not universal cf1/cfx.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum ControllerConfiguration {
    AbbConfData { confdata: RobConf },
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ToolFrame {
    Flange,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ReferenceFrame {
    RobotBase,
}

/// Common geometric data: millimetres, degrees, scalar-first unit quaternion.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct CartesianTarget {
    pub pose: Pose,
    pub joints_deg: JointAngles,
    pub controller_configuration: ControllerConfiguration,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum MotionInstruction {
    #[serde(rename_all = "camelCase")]
    Linear {
        target: CartesianTarget,
        speed_mm_per_second: f64,
        /// Exact end stop; future blending requires explicit semantics and validation.
        exact_stop: bool,
    },
}

/// Interchange representation; deserializing it does not establish motion validity.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct MotionProgram {
    pub schema_version: u32,
    pub name: String,
    pub robot_model_id: String,
    pub robot_brand: RobotBrand,
    pub start_joints_deg: JointAngles,
    pub tool_frame: ToolFrame,
    pub reference_frame: ReferenceFrame,
    pub instructions: Vec<MotionInstruction>,
}

/// No Deserialize or public constructor: renderer input must originate in native planning.
#[derive(Clone, Debug)]
pub struct ValidatedProgram {
    ir: MotionProgram,
    warnings: Vec<String>,
}

impl ValidatedProgram {
    pub fn ir(&self) -> &MotionProgram {
        &self.ir
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct ProgramFile {
    pub file_name: String,
    pub content: String,
    pub mime_type: String,
    pub controller: ControllerTarget,
    pub model_id: String,
    pub warnings: Vec<String>,
}

/// A controller export may require a coherent set of source/data files (e.g. KRL).
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct ProgramBundle {
    pub controller: ControllerTarget,
    pub model_id: String,
    pub files: Vec<ProgramFile>,
    pub warnings: Vec<String>,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub struct ProgramError {
    pub code: String,
    pub message: String,
}

impl fmt::Display for ProgramError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.message)
    }
}
impl std::error::Error for ProgramError {}

fn error(code: &str, message: impl Into<String>) -> ProgramError {
    ProgramError {
        code: code.into(),
        message: message.into(),
    }
}

/// Implementations map validated common motion semantics to a controller's real language.
pub trait ProgramPostprocessor: Sync {
    fn controller(&self) -> ControllerTarget;
    fn supports_robot_model(&self, model_id: &str) -> bool;
    fn emit(&self, program: &ValidatedProgram) -> Result<ProgramBundle, ProgramError>;
}

pub struct AbbRapidPostprocessor;

static ABB_POSTPROCESSOR: AbbRapidPostprocessor = AbbRapidPostprocessor;

pub fn postprocessor_for(
    controller: ControllerTarget,
) -> Result<&'static dyn ProgramPostprocessor, ProgramError> {
    match controller {
        ControllerTarget::AbbRapid => Ok(&ABB_POSTPROCESSOR),
        _ => Err(error(
            "unsupported_controller",
            "该品牌的机器人模型与控制器后处理尚未适配，不能导出可执行模块。",
        )),
    }
}

/// Keep module/file names deliberately narrower than arbitrary controller identifiers.
pub fn validate_program_name(name: &str) -> Result<(), ProgramError> {
    let mut chars = name.bytes();
    if name.len() > 32
        || !chars.next().is_some_and(|c| c.is_ascii_alphabetic())
        || chars.any(|c| !c.is_ascii_alphanumeric() && c != b'_')
    {
        return Err(error(
            "invalid_program_name",
            "模块名须以英文字母开头，由 1–32 个英文字母、数字或下划线组成。",
        ));
    }
    let lower = name.to_ascii_lowercase();
    const RESERVED: &[&str] = &[
        "alias",
        "and",
        "backward",
        "case",
        "connect",
        "const",
        "default",
        "div",
        "do",
        "else",
        "elseif",
        "endfor",
        "endfunc",
        "endif",
        "endmodule",
        "endproc",
        "endrecord",
        "endtest",
        "endtrap",
        "endwhile",
        "error",
        "exit",
        "false",
        "for",
        "from",
        "func",
        "goto",
        "if",
        "in",
        "inout",
        "local",
        "mod",
        "module",
        "noreturn",
        "not",
        "noview",
        "or",
        "pers",
        "proc",
        "raise",
        "readonly",
        "record",
        "retry",
        "return",
        "step",
        "stop",
        "test",
        "then",
        "to",
        "trap",
        "true",
        "try",
        "undo",
        "var",
        "viewonly",
        "while",
        "with",
        "xor",
        "main",
        "tool0",
        "wobj0",
        "emulationstart",
        "emulationtarget",
        "emulationspeed",
        "emulationactual",
        "abs",
        "cjointt",
        "tpwrite",
        "confl",
        "movel",
        "moveabsj",
        "speeddata",
        "jointtarget",
        "robtarget",
        "num",
        "con",
        "prn",
        "aux",
        "nul",
        "com1",
        "com2",
        "com3",
        "com4",
        "com5",
        "com6",
        "com7",
        "com8",
        "com9",
        "lpt1",
        "lpt2",
        "lpt3",
        "lpt4",
        "lpt5",
        "lpt6",
        "lpt7",
        "lpt8",
        "lpt9",
    ];
    if RESERVED.contains(&lower.as_str()) {
        return Err(error(
            "invalid_program_name",
            "模块名与 RAPID 关键字、内置标识符或系统保留文件名冲突。",
        ));
    }
    Ok(())
}

pub fn export_program(request: &ProgramExportRequest) -> Result<ProgramFile, ProgramError> {
    export_program_with_cancel(request, &AtomicBool::new(false))
}

pub fn export_program_with_cancel(
    request: &ProgramExportRequest,
    cancel: &AtomicBool,
) -> Result<ProgramFile, ProgramError> {
    let mut bundle = export_program_bundle_with_cancel(request, cancel)?;
    if bundle.files.len() != 1 {
        return Err(error(
            "multiple_files_require_bundle",
            "该控制器程序包含多个文件，请使用完整程序文件包导出接口。",
        ));
    }
    Ok(bundle.files.remove(0))
}

pub fn export_program_bundle(
    request: &ProgramExportRequest,
) -> Result<ProgramBundle, ProgramError> {
    export_program_bundle_with_cancel(request, &AtomicBool::new(false))
}

pub fn export_program_bundle_with_cancel(
    request: &ProgramExportRequest,
    cancel: &AtomicBool,
) -> Result<ProgramBundle, ProgramError> {
    if cancel.load(Ordering::Relaxed) {
        return Err(error("cancelled", "模块生成已取消。"));
    }
    validate_program_name(&request.program_name)?;
    let postprocessor = postprocessor_for(request.controller)?;
    if !postprocessor.supports_robot_model(&request.motion.model_id) {
        return Err(error(
            "unsupported_robot_model",
            "该机器人型号尚未通过所选控制器后处理的适配验证。",
        ));
    }
    let plan = match plan_move_l_with_cancel(&request.motion, cancel) {
        PlanResult::Success(plan) => plan,
        PlanResult::Failure(failure) => {
            let code = if failure.code == FailureCode::Cancelled {
                "cancelled"
            } else {
                "plan_rejected"
            };
            return Err(error(code, failure.message));
        }
    };
    if cancel.load(Ordering::Relaxed) {
        return Err(error("cancelled", "模块生成已取消。"));
    }
    let program = ValidatedProgram {
        ir: MotionProgram {
            schema_version: 1,
            name: request.program_name.clone(),
            robot_model_id: request.motion.model_id.clone(),
            robot_brand: RobotBrand::Abb,
            start_joints_deg: request.motion.start_joints,
            tool_frame: ToolFrame::Flange,
            reference_frame: ReferenceFrame::RobotBase,
            instructions: vec![MotionInstruction::Linear {
                target: CartesianTarget {
                    pose: plan.selected.pose,
                    joints_deg: plan.selected.joints,
                    controller_configuration: ControllerConfiguration::AbbConfData {
                        confdata: plan.selected.configuration,
                    },
                },
                speed_mm_per_second: request.motion.speed_mm_per_second,
                exact_stop: true,
            }],
        },
        warnings: plan.warnings,
    };
    let output = postprocessor.emit(&program)?;
    if cancel.load(Ordering::Relaxed) {
        return Err(error("cancelled", "模块生成已取消。"));
    }
    Ok(output)
}

fn decimal(value: f64, places: usize) -> Result<String, ProgramError> {
    if !value.is_finite() {
        return Err(error("invalid_program", "模块包含非有限数值，不能导出。"));
    }
    let mut text = format!("{value:.places$}");
    if text.contains('.') {
        while text.ends_with('0') {
            text.pop();
        }
        if text.ends_with('.') {
            text.pop();
        }
    }
    if text == "-0" {
        text = "0".into();
    }
    Ok(text)
}

fn tuple(values: &[f64], places: usize) -> Result<String, ProgramError> {
    Ok(format!(
        "[{}]",
        values
            .iter()
            .map(|value| decimal(*value, places))
            .collect::<Result<Vec<_>, _>>()?
            .join(",")
    ))
}

impl ProgramPostprocessor for AbbRapidPostprocessor {
    fn controller(&self) -> ControllerTarget {
        ControllerTarget::AbbRapid
    }

    fn supports_robot_model(&self, model_id: &str) -> bool {
        ABB_MODELS.contains(&model_id)
    }

    fn emit(&self, program: &ValidatedProgram) -> Result<ProgramBundle, ProgramError> {
        let ir = program.ir();
        validate_program_name(&ir.name)?;
        if ir.robot_brand != RobotBrand::Abb
            || !self.supports_robot_model(&ir.robot_model_id)
            || ir.instructions.len() != 1
        {
            return Err(error(
                "unsupported_program",
                "当前 ABB 后处理仅支持经验证的单段 IRB1100 MoveL。",
            ));
        }
        let MotionInstruction::Linear {
            target,
            speed_mm_per_second,
            exact_stop,
        } = &ir.instructions[0];
        if !exact_stop {
            return Err(error(
                "unsupported_program",
                "尚未实现路径过渡区验证，不能导出混合运动。",
            ));
        }
        let ControllerConfiguration::AbbConfData { confdata } = &target.controller_configuration;
        let start = tuple(&ir.start_joints_deg, 6)?;
        let position = tuple(&target.pose.position, 3)?;
        let orientation = tuple(&target.pose.orientation, 9)?;
        let configuration = format!(
            "[{},{},{},{}]",
            confdata[0], confdata[1], confdata[2], confdata[3]
        );
        let speed = decimal(*speed_mm_per_second, 3)?;
        let mut content = String::with_capacity(2500);
        writeln!(content, "MODULE {}", ir.name).unwrap();
        writeln!(
            content,
            "    ! Generated from validated nominal {} kinematics.",
            ir.robot_model_id
        )
        .unwrap();
        writeln!(
            content,
            "    ! Requires simulated start joints; no positioning motion is added."
        )
        .unwrap();
        writeln!(
            content,
            "    ! Verify controller, calibration, tool0 and wobj0 before execution."
        )
        .unwrap();
        writeln!(
            content,
            "    LOCAL CONST jointtarget emulationStart := [{start},[9E9,9E9,9E9,9E9,9E9,9E9]];"
        )
        .unwrap();
        writeln!(content, "    LOCAL CONST robtarget emulationTarget := [{position},{orientation},{configuration},[9E9,9E9,9E9,9E9,9E9,9E9]];").unwrap();
        writeln!(
            content,
            "    LOCAL CONST speeddata emulationSpeed := [{speed},500,5000,1000];"
        )
        .unwrap();
        writeln!(content, "\n    PROC main()\n        VAR jointtarget emulationActual;\n        emulationActual := CJointT();").unwrap();
        for axis in 1..=6 {
            writeln!(content, "        IF Abs(emulationActual.robax.rax_{axis}-emulationStart.robax.rax_{axis}) > 0.01 THEN").unwrap();
            writeln!(content, "            TPWrite \"Start axis {axis} mismatch. No motion.\";\n            RETURN;\n        ENDIF").unwrap();
        }
        writeln!(content, "        ConfL \\On;\n        MoveL emulationTarget,emulationSpeed,fine,tool0\\WObj:=wobj0;\n    ENDPROC\nENDMODULE").unwrap();
        let mut warnings = program.warnings.clone();
        warnings.push("模块不包含到起点的定位运动；运行前应先确认当前六轴角与仿真起点一致，模块会以 0.01° 容差逐轴检查。".into());
        warnings.push("已完成标称运动学与离散路径检查；尚未通过对应 RobotStudio/RobotWare 版本编译、碰撞或实机验收。".into());
        let file = ProgramFile {
            file_name: format!("{}.mod", ir.name),
            content: content.replace('\n', "\r\n"),
            mime_type: "text/plain; charset=utf-8".into(),
            controller: self.controller(),
            model_id: ir.robot_model_id.clone(),
            warnings: warnings.clone(),
        };
        Ok(ProgramBundle {
            controller: self.controller(),
            model_id: ir.robot_model_id.clone(),
            files: vec![file],
            warnings,
        })
    }
}
