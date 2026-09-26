import { ElevationState, TargetState } from '../types';

export interface TargetController {
  run(): void;
  stop(): void;
  pop(): void;
  emergencyStop(): void;
  resetEmergency(): void;
  getState(): TargetState;
  subscribe(listener: (state: TargetState) => void): () => void;
  setSpeed(speedMps: number): void;
  destroy(): void;
}

export class SimulatedTargetController implements TargetController {
  private state: TargetState = {
    positionMeters: 0.000,
    direction: 'STOPPED',
    elevation: 'DOWN',
    speedMps: 0.30,
    systemStatus: 'READY',
    exposureRemainingSeconds: 0,
    autoDropSeconds: 5.0,
  };

  private listeners: Set<(state: TargetState) => void> = new Set();
  private animationFrameId: number | null = null;
  private lastTimestamp: number = 0;
  private exposureTimerId: ReturnType<typeof setInterval> | null = null;

  constructor(initialElevation: ElevationState = 'DOWN') {
    this.state.elevation = initialElevation;
    if (initialElevation === 'UP') {
      this.startExposureTimer();
    }
  }

  private notify() {
    const currentState = { ...this.state };
    this.listeners.forEach((listener) => listener(currentState));
  }

  private startExposureTimer() {
    this.clearExposureTimer();
    this.state.exposureRemainingSeconds = this.state.autoDropSeconds;

    const intervalMs = 100;
    this.exposureTimerId = setInterval(() => {
      if (this.state.elevation !== 'UP' || this.state.systemStatus === 'EMERGENCY_STOP') {
        this.clearExposureTimer();
        return;
      }

      const nextVal = Math.max(0, this.state.exposureRemainingSeconds - (intervalMs / 1000));
      this.state.exposureRemainingSeconds = Number(nextVal.toFixed(1));

      if (this.state.exposureRemainingSeconds <= 0) {
        // Automatically drop target after 5 seconds
        this.state.elevation = 'DOWN';
        this.clearExposureTimer();
      }

      this.notify();
    }, intervalMs);
  }

  private clearExposureTimer() {
    if (this.exposureTimerId !== null) {
      clearInterval(this.exposureTimerId);
      this.exposureTimerId = null;
    }
  }

  private step = (timestamp: number) => {
    if (this.state.direction === 'STOPPED' || this.state.systemStatus === 'EMERGENCY_STOP') {
      this.animationFrameId = null;
      return;
    }

    if (this.lastTimestamp === 0) {
      this.lastTimestamp = timestamp;
    }

    const deltaSeconds = Math.min((timestamp - this.lastTimestamp) / 1000, 0.1);
    this.lastTimestamp = timestamp;

    const deltaDistance = this.state.speedMps * deltaSeconds;

    if (this.state.direction === 'FORWARD') {
      let nextPos = this.state.positionMeters + deltaDistance;
      if (nextPos >= 2.0) {
        nextPos = 2.0;
        this.state.direction = 'BACKWARD'; // reverse back towards shooter
      }
      this.state.positionMeters = Number(nextPos.toFixed(4));
    } else if (this.state.direction === 'BACKWARD') {
      let nextPos = this.state.positionMeters - deltaDistance;
      if (nextPos <= 0.0) {
        nextPos = 0.0;
        this.state.direction = 'FORWARD'; // move away from shooter again
      }
      this.state.positionMeters = Number(nextPos.toFixed(4));
    }

    this.notify();
    this.animationFrameId = requestAnimationFrame(this.step);
  };

  public run(): void {
    if (this.state.systemStatus === 'EMERGENCY_STOP') return;

    if (this.state.direction === 'STOPPED') {
      this.state.direction = this.state.positionMeters >= 2.0 ? 'BACKWARD' : 'FORWARD';
      this.state.systemStatus = 'MOVING';
      this.lastTimestamp = 0;
      this.notify();

      if (!this.animationFrameId) {
        this.animationFrameId = requestAnimationFrame(this.step);
      }
    }
  }

  public stop(): void {
    if (this.state.systemStatus === 'EMERGENCY_STOP') return;

    this.state.direction = 'STOPPED';
    this.state.systemStatus = 'READY';
    if (this.animationFrameId) {
      cancelAnimationFrame(this.animationFrameId);
      this.animationFrameId = null;
    }
    this.notify();
  }

  public pop(): void {
    if (this.state.systemStatus === 'EMERGENCY_STOP') return;

    if (this.state.elevation === 'UP') {
      // Manual drop
      this.state.elevation = 'DOWN';
      this.clearExposureTimer();
      this.state.exposureRemainingSeconds = 0;
      this.notify();
    } else {
      // Pop UP with 5-second automatic drop
      this.state.elevation = 'UP';
      this.startExposureTimer();
      this.notify();
    }
  }

  public emergencyStop(): void {
    this.state.direction = 'STOPPED';
    this.state.systemStatus = 'EMERGENCY_STOP';
    this.clearExposureTimer();
    if (this.animationFrameId) {
      cancelAnimationFrame(this.animationFrameId);
      this.animationFrameId = null;
    }
    this.notify();
  }

  public resetEmergency(): void {
    this.state.systemStatus = 'READY';
    this.notify();
  }

  public getState(): TargetState {
    return { ...this.state };
  }

  public subscribe(listener: (state: TargetState) => void): () => void {
    this.listeners.add(listener);
    listener({ ...this.state });
    return () => {
      this.listeners.delete(listener);
    };
  }

  public setSpeed(speedMps: number): void {
    this.state.speedMps = Math.max(0.05, Math.min(2.0, speedMps));
    this.notify();
  }

  public resetToOrigin(): void {
    this.stop();
    this.clearExposureTimer();
    this.state.positionMeters = 0.0;
    this.state.elevation = 'DOWN';
    this.state.exposureRemainingSeconds = 0;
    this.state.systemStatus = 'READY';
    this.notify();
  }

  public destroy(): void {
    if (this.animationFrameId) {
      cancelAnimationFrame(this.animationFrameId);
      this.animationFrameId = null;
    }
    this.clearExposureTimer();
    this.listeners.clear();
  }
}

/**
 * Skeleton / Interface preparation for future ESP32 physical target carriage controller
 */
export class ESP32TargetController implements TargetController {
  private state: TargetState = {
    positionMeters: 0.0,
    direction: 'STOPPED',
    elevation: 'DOWN',
    speedMps: 0.30,
    systemStatus: 'READY',
    exposureRemainingSeconds: 0,
    autoDropSeconds: 5.0,
  };

  run(): void {
    console.info('[ESP32TargetController] Sending CMD_RUN via serial/WiFi transport');
  }

  stop(): void {
    console.info('[ESP32TargetController] Sending CMD_STOP via serial/WiFi transport');
  }

  pop(): void {
    console.info('[ESP32TargetController] Sending CMD_POP via serial/WiFi transport');
  }

  emergencyStop(): void {
    console.warn('[ESP32TargetController] Sending CMD_ESTOP via hardware pin/packet');
  }

  resetEmergency(): void {
    console.info('[ESP32TargetController] Resetting ESTOP');
  }

  getState(): TargetState {
    return { ...this.state };
  }

  subscribe(_listener: (state: TargetState) => void): () => void {
    return () => {};
  }

  setSpeed(_speedMps: number): void {}

  destroy(): void {}
}
