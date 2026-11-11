export const INPUT_INTERVAL_MS: number;
export const STATE_TIMEOUT_MS: number;
export type PendantMode = 'linear' | 'joint';
export interface MobileRobotState {
  robotName: string; modelId: string; joints: number[];
  tcp: { position: number[]; orientation: number[] } | null;
  available: boolean; reason: string | null; moving: boolean;
}
export interface MobileInput {
  type: 'input'; epoch: number; sequence: number; mode: PendantMode;
  space: 'world' | 'tool'; axes: number[]; speed: number; active: boolean;
}
export interface MobileSnapshot {
  phase: 'disconnected' | 'connecting' | 'connected' | 'error'; connected: boolean; holding: boolean;
  epoch: number | null; sessionId: string | null; mode: PendantMode; pair: number;
  space: 'world' | 'tool'; speed: number; vector: number[]; robot: MobileRobotState | null;
  available: boolean; feedback: string;
}
export interface MobileController {
  snapshot(): MobileSnapshot;
  connecting(): void; receive(message: unknown): boolean; start(vector?: number[]): boolean;
  move(vector: number[]): void; end(message?: string): void; stop(message?: string): void;
  configure(patch: Partial<Pick<MobileSnapshot, 'mode' | 'pair' | 'space' | 'speed'>>): void;
  disconnect(message?: string, failed?: boolean): void; dispose(): void;
}
export function joystickVector(dx: number, dy: number, radius: number, deadZone?: number): number[];
export function joystickAxes(mode: PendantMode, pair: number, vector: number[]): number[];
export function validRobotState(value: unknown): value is MobileRobotState;
export function createPendantController(options: {
  send(packet: MobileInput): void;
  onChange?(state: MobileSnapshot): void;
  now?(): number;
  setInterval?(callback: () => void, milliseconds: number): unknown;
  clearInterval?(handle: unknown): void;
}): MobileController;
export function mountPendant(document: Document, window: Window): () => void;
