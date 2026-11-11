import type { JointAngles, Pose } from '../robot/types';

export interface PendantAddress { name: string; url: string }
export interface PendantSession {
  sessionId: string;
  port: number;
  urls: PendantAddress[];
  connected: boolean;
  expiresAtUnixMs: number;
}
export interface PendantInput {
  sessionId: string;
  epoch: number;
  sequence: number;
  receivedAtMs: number;
  mode: 'linear' | 'joint';
  space: 'world' | 'tool';
  axes: [number, number, number, number, number, number];
  speed: number;
  active: boolean;
}
export interface PendantState {
  robotName: string;
  modelId: string;
  joints: JointAngles;
  tcp: Pose | null;
  available: boolean;
  reason: string | null;
  moving: boolean;
}
export interface PendantTransport {
  start(): Promise<PendantSession>;
  stop(): Promise<void>;
  status(): Promise<PendantSession | null>;
  publish(sessionId: string, state: PendantState): Promise<void>;
  listen(callback: (input: PendantInput) => void): Promise<() => void>;
}
