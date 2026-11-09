import { planMoveL } from "./planner";
import type { PlanRequest, PlanResult } from "./types";

self.onmessage = (event: MessageEvent<PlanRequest>) => {
  let result: PlanResult;
  try {
    result = planMoveL(event.data);
  } catch {
    result = { status: "failure", code: "invalid_input", message: "目标或起始参数无效，无法生成 MoveL" };
  }
  self.postMessage(result);
};
