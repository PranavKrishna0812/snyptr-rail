import { OptimalROI, PointMm, ShotOutcome, TargetState } from '../types';

export interface ShotResult {
  hit: boolean;
  result: ShotOutcome;
  pointMm: PointMm;
  distanceFromOptimalMm: number;
  directionFromOptimal: string;
  positionMeters: number;
}

export interface ShotDetectionProvider {
  detectShot(pointMm: PointMm, targetState: TargetState): ShotResult | null;
  getOptimalROI(): OptimalROI;
  setOptimalROI(roi: OptimalROI): void;
}

export class SimulationShotDetectionProvider implements ShotDetectionProvider {
  private optimalROI: OptimalROI;

  constructor(initialROI: OptimalROI = { x: 50, y: 50, width: 50, height: 50 }) {
    this.optimalROI = initialROI;
  }

  public getOptimalROI(): OptimalROI {
    return { ...this.optimalROI };
  }

  public setOptimalROI(roi: OptimalROI): void {
    this.optimalROI = { ...roi };
  }

  public detectShot(pointMm: PointMm, targetState: TargetState): ShotResult | null {
    // If target is DOWN, shot cannot register on target face
    if (targetState.elevation === 'DOWN') {
      return null;
    }

    const { x, y } = pointMm;
    const isInside =
      x >= this.optimalROI.x &&
      x <= this.optimalROI.x + this.optimalROI.width &&
      y >= this.optimalROI.y &&
      y <= this.optimalROI.y + this.optimalROI.height;

    const optCenterX = this.optimalROI.x + this.optimalROI.width / 2;
    const optCenterY = this.optimalROI.y + this.optimalROI.height / 2;
    const dx = x - optCenterX;
    const dy = y - optCenterY;
    const distMm = Math.sqrt(dx * dx + dy * dy);

    // Direction calculation
    let direction = 'CENTER';
    const deadband = 5.0; // 5mm deadband for true center
    if (Math.abs(dx) > deadband || Math.abs(dy) > deadband) {
      const horizontal = dx > deadband ? 'RIGHT' : dx < -deadband ? 'LEFT' : '';
      const vertical = dy > deadband ? 'LOW' : dy < -deadband ? 'HIGH' : '';
      direction = [vertical, horizontal].filter(Boolean).join('-');
    }

    return {
      hit: isInside,
      result: isInside ? 'HIT' : 'MISS',
      pointMm,
      distanceFromOptimalMm: Number(distMm.toFixed(1)),
      directionFromOptimal: direction || 'CENTER',
      positionMeters: targetState.positionMeters,
    };
  }
}

/**
 * Skeleton / Interface for future OV5647 Camera + ESP32-P4 computer vision detection
 */
export class ESP32CameraShotDetectionProvider implements ShotDetectionProvider {
  private optimalROI: OptimalROI = { x: 50, y: 50, width: 50, height: 50 };

  getOptimalROI(): OptimalROI {
    return this.optimalROI;
  }

  setOptimalROI(roi: OptimalROI): void {
    this.optimalROI = roi;
  }

  detectShot(_pointMm: PointMm, _targetState: TargetState): ShotResult | null {
    console.info('[ESP32CameraShotDetectionProvider] Awaiting laser centroid frame from OV5647 via ESP32-P4');
    return null;
  }
}
