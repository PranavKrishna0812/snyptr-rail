export type ElevationState = 'UP' | 'DOWN';
export type MovementDirection = 'FORWARD' | 'BACKWARD' | 'STOPPED';
export type SystemOperationalStatus = 'READY' | 'MOVING' | 'EMERGENCY_STOP' | 'FAULT';
export type ShotOutcome = 'HIT' | 'MISS';

export interface PointMm {
  x: number; // 0 to 150 mm (from top-left of 15x15cm target)
  y: number; // 0 to 150 mm
}

export interface TargetState {
  positionMeters: number; // 0.000 to 2.000 m
  direction: MovementDirection;
  elevation: ElevationState;
  speedMps: number; // default 0.30 m/s
  systemStatus: SystemOperationalStatus;
  exposureRemainingSeconds: number; // 5.0 to 0.0s when UP
  autoDropSeconds: number; // default 5.0s
}

export interface OptimalROI {
  x: number; // mm from left
  y: number; // mm from top
  width: number; // mm
  height: number; // mm
}

export interface CalibrationConfig {
  targetSizeMm: number; // 150 mm
  optimalROI: OptimalROI;
  laserDetectionThreshold: number; // 0-255 (for future camera provider)
}

export interface ShotRecord {
  id: number;
  shotNumber: number; // 1 to 20
  timestamp: string; // HH:MM:SS
  positionMeters: number; // e.g. 1.372
  xMm: number;
  yMm: number;
  result: ShotOutcome;
  distanceFromOptimalMm: number;
  directionFromOptimal: string; // 'CENTER' | 'RIGHT' | 'LEFT' | 'HIGH' | 'LOW' | 'HIGH-RIGHT' etc.
}

export interface DistanceBandStats {
  band: string; // '0.0–0.5 m' | '0.5–1.0 m' | '1.0–1.5 m' | '1.5–2.0 m'
  minM: number;
  maxM: number;
  totalShots: number;
  hits: number;
  misses: number;
  accuracy: number | null; // null if insufficient data (< 2 shots)
}

export interface SessionReport {
  sessionId: string;
  startedAt: string;
  endedAt: string;
  totalShots: number;
  hits: number;
  misses: number;
  accuracy: number;
  distanceBands: DistanceBandStats[];
  missDistribution: {
    byBand: Record<string, number>;
    byDirection: Record<string, number>;
    dominantDirection: string | null;
    highestMissBand: string | null;
  };
  observations: string[];
  trainingFocus: string[];
  shots: ShotRecord[];
}
