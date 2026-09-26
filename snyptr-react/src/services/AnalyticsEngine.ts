import { DistanceBandStats, SessionReport, ShotRecord } from '../types';

export class AnalyticsEngine {
  private static readonly BANDS = [
    { label: '0.0–0.5 m', min: 0.0, max: 0.5 },
    { label: '0.5–1.0 m', min: 0.5, max: 1.0 },
    { label: '1.0–1.5 m', min: 1.0, max: 1.5 },
    { label: '1.5–2.0 m', min: 1.5, max: 2.01 }, // inclusive of 2.000m
  ];

  public static generateSessionReport(
    sessionId: string,
    startedAt: string,
    shots: ShotRecord[]
  ): SessionReport {
    const endedAt = new Date().toLocaleTimeString('en-GB');
    const totalShots = shots.length;
    const hits = shots.filter((s) => s.result === 'HIT').length;
    const misses = shots.filter((s) => s.result === 'MISS').length;
    const accuracy = totalShots > 0 ? (hits / totalShots) * 100 : 0;

    // 1. Distance Band Breakdown
    const distanceBands: DistanceBandStats[] = this.BANDS.map((b) => {
      const bandShots = shots.filter(
        (s) => s.positionMeters >= b.min && s.positionMeters < b.max
      );
      const bandHits = bandShots.filter((s) => s.result === 'HIT').length;
      const bandMisses = bandShots.filter((s) => s.result === 'MISS').length;
      const hasEnoughData = bandShots.length >= 2;

      return {
        band: b.label,
        minM: b.min,
        maxM: b.max,
        totalShots: bandShots.length,
        hits: bandHits,
        misses: bandMisses,
        accuracy: hasEnoughData ? (bandHits / bandShots.length) * 100 : null,
      };
    });

    // 2. Miss Distribution Analysis
    const missShots = shots.filter((s) => s.result === 'MISS');
    const missByBand: Record<string, number> = {};
    const missByDirection: Record<string, number> = {};

    this.BANDS.forEach((b) => {
      missByBand[b.label] = 0;
    });

    missShots.forEach((m) => {
      // Find matching band
      const matchedBand = this.BANDS.find(
        (b) => m.positionMeters >= b.min && m.positionMeters < b.max
      );
      const bandLabel = matchedBand ? matchedBand.label : '1.5–2.0 m';
      missByBand[bandLabel] = (missByBand[bandLabel] || 0) + 1;

      // Group directions (e.g. RIGHT, LEFT, HIGH, LOW, etc.)
      const dir = m.directionFromOptimal;
      missByDirection[dir] = (missByDirection[dir] || 0) + 1;
    });

    // Determine highest miss band
    let highestMissBand: string | null = null;
    let maxMissCount = 0;
    Object.entries(missByBand).forEach(([band, count]) => {
      if (count > maxMissCount) {
        maxMissCount = count;
        highestMissBand = band;
      }
    });

    // Determine dominant miss direction
    let dominantDirection: string | null = null;
    let maxDirCount = 0;
    Object.entries(missByDirection).forEach(([dir, count]) => {
      if (count > maxDirCount) {
        maxDirCount = count;
        dominantDirection = dir;
      }
    });

    // 3. Evidence-Based Observations & Training Focus (Strictly derived from data)
    const observations: string[] = [];
    const trainingFocus: string[] = [];

    if (totalShots < 5) {
      observations.push('Insufficient data for a reliable trend.');
    } else {
      // Accuracy overview
      if (accuracy >= 80) {
        observations.push(`High baseline accuracy recorded (${accuracy.toFixed(1)}% hit rate across ${totalShots} shots).`);
      } else if (accuracy >= 60) {
        observations.push(`Moderate overall accuracy (${accuracy.toFixed(1)}%). ${misses} out of ${totalShots} shots missed the optimal zone.`);
      } else {
        observations.push(`Sub-optimal grouping observed (${accuracy.toFixed(1)}% hit rate with ${misses} misses).`);
      }

      // Distance band trends
      const shortBands = distanceBands.slice(0, 2);
      const farBands = distanceBands.slice(2, 4);

      const shortHits = shortBands.reduce((acc, b) => acc + b.hits, 0);
      const shortTotal = shortBands.reduce((acc, b) => acc + b.totalShots, 0);
      const farHits = farBands.reduce((acc, b) => acc + b.hits, 0);
      const farTotal = farBands.reduce((acc, b) => acc + b.totalShots, 0);

      const shortAcc = shortTotal > 0 ? (shortHits / shortTotal) * 100 : null;
      const farAcc = farTotal > 0 ? (farHits / farTotal) * 100 : null;

      if (shortAcc !== null && farAcc !== null && shortTotal >= 3 && farTotal >= 3) {
        if (shortAcc > farAcc + 15) {
          observations.push(`Accuracy significantly decreased as target distance increased (${shortAcc.toFixed(0)}% at 0.0–1.0m vs ${farAcc.toFixed(0)}% at 1.0–2.0m).`);
          trainingFocus.push('Maintain tighter aim and steady trigger pull during extended rail travel (>1.0 m).');
        } else if (farAcc >= shortAcc) {
          observations.push(`Consistent precision maintained across near and far distance brackets.`);
        }
      }

      // Concentration of misses by band
      if (highestMissBand && maxMissCount >= 2) {
        observations.push(`${maxMissCount} of ${misses} misses occurred in the ${highestMissBand} distance bracket.`);
        trainingFocus.push(`Concentrate drills at the ${highestMissBand} bracket to eliminate cluster errors.`);
      }

      // Directional bias analysis
      if (dominantDirection && maxDirCount >= 2) {
        observations.push(`Directional miss bias detected: ${maxDirCount} misses dispersed towards the ${dominantDirection} of the optimal zone.`);
        trainingFocus.push(`Correct sight alignment / grip pressure to reduce ${dominantDirection}-side deviation.`);
      }

      // Default training focus if shooter had clean hits
      if (misses === 0) {
        observations.push('Flawless 20-shot qualification score achieved in optimal zone.');
        trainingFocus.push('Advance to higher rail movement velocity or shorter engagement windows.');
      }
    }

    return {
      sessionId,
      startedAt,
      endedAt,
      totalShots,
      hits,
      misses,
      accuracy,
      distanceBands,
      missDistribution: {
        byBand: missByBand,
        byDirection: missByDirection,
        dominantDirection,
        highestMissBand,
      },
      observations,
      trainingFocus,
      shots,
    };
  }
}
