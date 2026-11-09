use emulation_core::{
    kinematics::{forward_kinematics, inverse_kinematics, inverse_kinematics_cancellable},
    model::get_robot_model,
};

#[test]
fn endpoint_ik_cancellation_never_returns_partial_solutions_or_changes_later_queries() {
    for id in ["irb1100-4-0.58", "irb1100-4-0.475"] {
        let model = get_robot_model(id).unwrap();
        let pose = forward_kinematics(&model, &model.ready_joints)
            .unwrap()
            .pose;
        let complete = inverse_kinematics(&model, &pose, &model.ready_joints);
        assert!(complete.len() > 4);
        assert!(
            inverse_kinematics_cancellable(&model, &pose, &model.ready_joints, || true).is_err()
        );
        let mut calls = 0;
        let interrupted =
            inverse_kinematics_cancellable(&model, &pose, &model.ready_joints, || {
                calls += 1;
                calls >= 10
            });
        assert!(interrupted.is_err());
        assert_eq!(calls, 10);
        let retry =
            inverse_kinematics_cancellable(&model, &pose, &model.ready_joints, || false).unwrap();
        assert_eq!(
            serde_json::to_value(retry).unwrap(),
            serde_json::to_value(complete).unwrap()
        );
    }
}
