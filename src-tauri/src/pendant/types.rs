use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum Mode {
    Linear,
    Joint,
}
#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum Space {
    World,
    Tool,
}

#[derive(Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct PendantAddress {
    pub name: String,
    pub url: String,
}
/// Contains a pairing capability. Never log or persist this value.
#[derive(Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct PendantSession {
    pub session_id: String,
    pub port: u16,
    pub urls: Vec<PendantAddress>,
    pub connected: bool,
    pub expires_at_unix_ms: u64,
}
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct PendantPose {
    pub position: [f64; 3],
    pub orientation: [f64; 4],
}
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct PendantState {
    pub robot_name: String,
    pub model_id: String,
    pub joints: [f64; 6],
    pub tcp: Option<PendantPose>,
    pub available: bool,
    pub reason: Option<String>,
    pub moving: bool,
}
impl Default for PendantState {
    fn default() -> Self {
        Self {
            robot_name: String::new(),
            model_id: String::new(),
            joints: [0.; 6],
            tcp: None,
            available: false,
            reason: Some("等待电脑端就绪".into()),
            moving: false,
        }
    }
}
impl PendantState {
    pub(crate) fn validate(&self) -> Result<(), String> {
        if self.robot_name.chars().count() > 160
            || self.model_id.len() > 160
            || self
                .reason
                .as_ref()
                .is_some_and(|value| value.chars().count() > 320)
            || self
                .joints
                .iter()
                .any(|value| !value.is_finite() || value.abs() > 1_000_000.)
        {
            return Err("示教器状态无效".into());
        }
        if self.available
            && (self.robot_name.is_empty() || self.model_id.is_empty() || self.tcp.is_none())
        {
            return Err("可控状态需要完整的机器人和TCP数据".into());
        }
        if let Some(pose) = &self.tcp {
            let norm = pose
                .orientation
                .iter()
                .map(|value| value * value)
                .sum::<f64>();
            if pose
                .position
                .iter()
                .any(|value| !value.is_finite() || value.abs() > 1_000_000_000.)
                || !norm.is_finite()
                || (norm - 1.).abs() > 0.001
            {
                return Err("示教器坐标无效".into());
            }
        }
        Ok(())
    }
}
/// Only accepted, current-lease input reaches the desktop. The frontend also
/// checks epoch and receive time before starting or accepting any async solve.
#[derive(Clone, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct PendantInput {
    pub session_id: String,
    pub epoch: u64,
    pub sequence: u64,
    pub received_at_ms: u64,
    pub mode: Mode,
    pub space: Space,
    pub axes: [f64; 6],
    pub speed: f64,
    pub active: bool,
    pub reason: Option<String>,
}

#[derive(Deserialize)]
#[serde(tag = "type", rename_all = "lowercase", deny_unknown_fields)]
pub(crate) enum ClientMessage {
    Auth {
        token: String,
    },
    Input {
        epoch: u64,
        sequence: u64,
        mode: Mode,
        space: Space,
        axes: [f64; 6],
        speed: f64,
        active: bool,
    },
}

pub(crate) fn valid_input(
    epoch: u64,
    sequence: u64,
    mode: Mode,
    axes: &[f64; 6],
    speed: f64,
    active: bool,
) -> bool {
    const MAX_JS_INTEGER: u64 = 9_007_199_254_740_991;
    epoch > 0
        && epoch <= MAX_JS_INTEGER
        && sequence > 0
        && sequence <= MAX_JS_INTEGER
        && speed.is_finite()
        && (0.0..=1.0).contains(&speed)
        && axes
            .iter()
            .all(|value| value.is_finite() && (-1.0..=1.0).contains(value))
        && (mode != Mode::Linear || axes[3..].iter().all(|value| *value == 0.))
        && (active || axes.iter().all(|value| *value == 0.))
}
