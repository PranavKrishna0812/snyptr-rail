import { useEffect, useMemo, useRef, useState } from 'react';
import { Header } from './components/Header';
import { TargetView } from './components/TargetView';
import { RailPositionControl } from './components/RailPositionControl';
import { SessionStats } from './components/SessionStats';
import { ShotHistoryTable } from './components/ShotHistoryTable';
import { SessionReportModal } from './components/SessionReportModal';
import { CalibrationModal } from './components/CalibrationModal';
import { SimulatedTargetController } from './services/TargetController';
import { SimulationShotDetectionProvider } from './services/ShotDetectionProvider';
import { AnalyticsEngine } from './services/AnalyticsEngine';
import {
  CalibrationConfig,
  PointMm,
  SessionReport,
  ShotRecord,
  TargetState,
} from './types';

const DEFAULT_CALIBRATION: CalibrationConfig = {
  targetSizeMm: 150,
  optimalROI: {
    x: 50,
    y: 50,
    width: 50,
    height: 50,
  },
  laserDetectionThreshold: 200,
};

export default function App() {
  const [calibration, setCalibration] = useState<CalibrationConfig>(() => {
    try {
      const saved = localStorage.getItem('snyptr_calibration_v1');
      if (saved) return JSON.parse(saved);
    } catch {
      // fallback
    }
    return DEFAULT_CALIBRATION;
  });

  const [showCalibration, setShowCalibration] = useState<boolean>(false);
  const [sessionNumber, setSessionNumber] = useState<number>(1);
  const [sessionStartTime] = useState<string>(() => new Date().toLocaleTimeString('en-GB'));
  const [shots, setShots] = useState<ShotRecord[]>([]);
  const [lastShot, setLastShot] = useState<ShotRecord | null>(null);
  const [report, setReport] = useState<SessionReport | null>(null);

  const controller = useMemo(() => new SimulatedTargetController('DOWN'), []);
  const detector = useMemo(
    () => new SimulationShotDetectionProvider(calibration.optimalROI),
    [calibration.optimalROI]
  );

  const [targetState, setTargetState] = useState<TargetState>(() => controller.getState());
  const isEmergencyStop = targetState.systemStatus === 'EMERGENCY_STOP';
  const isSessionLocked = shots.length >= 20;

  useEffect(() => {
    const unsubscribe = controller.subscribe((state) => {
      setTargetState(state);
    });
    return () => {
      unsubscribe();
      controller.destroy();
    };
  }, [controller]);

  useEffect(() => {
    detector.setOptimalROI(calibration.optimalROI);
  }, [calibration.optimalROI, detector]);

  const controllerRef = useRef(controller);
  controllerRef.current = controller;

  // Global Keyboard Controls (R, S, 0, ESC)
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      const activeEl = document.activeElement as HTMLElement | null;
      if (activeEl && (activeEl.tagName === 'INPUT' || activeEl.tagName === 'TEXTAREA')) {
        return;
      }

      const key = e.key.toUpperCase();

      if (key === 'ESCAPE') {
        e.preventDefault();
        controllerRef.current.emergencyStop();
      } else if (key === 'R') {
        e.preventDefault();
        controllerRef.current.run();
      } else if (key === 'S') {
        e.preventDefault();
        controllerRef.current.stop();
      } else if (key === '0') {
        e.preventDefault();
        controllerRef.current.pop();
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
    };
  }, []);

  const handleSaveCalibration = (newConfig: CalibrationConfig) => {
    setCalibration(newConfig);
    try {
      localStorage.setItem('snyptr_calibration_v1', JSON.stringify(newConfig));
    } catch {
      // Ignore
    }
  };

  const handleTargetClick = (pointMm: PointMm) => {
    if (isSessionLocked || isEmergencyStop) {
      return;
    }

    const currentTarget = controller.getState();
    const result = detector.detectShot(pointMm, currentTarget);
    if (!result) return;

    const now = new Date();
    const timeStr = `${String(now.getHours()).padStart(2, '0')}:${String(now.getMinutes()).padStart(2, '0')}:${String(now.getSeconds()).padStart(2, '0')}`;

    const newShot: ShotRecord = {
      id: Date.now(),
      shotNumber: shots.length + 1,
      timestamp: timeStr,
      positionMeters: result.positionMeters,
      xMm: pointMm.x,
      yMm: pointMm.y,
      result: result.result,
      distanceFromOptimalMm: result.distanceFromOptimalMm,
      directionFromOptimal: result.directionFromOptimal,
    };

    const updatedShots = [...shots, newShot];
    setLastShot(newShot);
    setShots(updatedShots);

    if (updatedShots.length === 20) {
      controller.stop();
      const generatedReport = AnalyticsEngine.generateSessionReport(
        String(sessionNumber).padStart(2, '0'),
        sessionStartTime,
        updatedShots
      );
      setReport(generatedReport);
    }
  };

  const handleStartNewSession = () => {
    setShots([]);
    setLastShot(null);
    setReport(null);
    setSessionNumber((prev) => prev + 1);
    controller.resetToOrigin();
  };

  const hits = shots.filter((s) => s.result === 'HIT').length;
  const misses = shots.filter((s) => s.result === 'MISS').length;
  const totalShots = shots.length;
  const accuracy = totalShots > 0 ? (hits / totalShots) * 100 : 0;

  return (
    <div style={{
      display: 'flex',
      flexDirection: 'column',
      height: '100vh',
      backgroundColor: 'var(--mil-canvas)',
      color: 'var(--text-white)',
      overflow: 'hidden',
    }}>
      {/* Tactical Header */}
      <Header
        sessionNumber={sessionNumber}
        systemStatus={targetState.systemStatus}
        isEmergencyStop={isEmergencyStop}
        onOpenCalibration={() => setShowCalibration(true)}
        onResetEmergency={() => controller.resetEmergency()}
        onStartNewSession={handleStartNewSession}
      />

      {/* Main Operational Container */}
      <div style={{
        flex: 1,
        padding: '12px 18px',
        display: 'flex',
        flexDirection: 'column',
        gap: '12px',
        minHeight: 0,
      }}>
        {/* Top Session Scoring HUD */}
        <SessionStats
          totalShots={totalShots}
          maxShots={20}
          hits={hits}
          misses={misses}
          accuracy={accuracy}
          onStartNewSession={handleStartNewSession}
        />

        {/* Main 2-Column Combat Console */}
        <main style={{
          flex: 1,
          display: 'grid',
          gridTemplateColumns: 'minmax(440px, 1.15fr) minmax(460px, 1.25fr)',
          gap: '12px',
          minHeight: 0,
        }}>
          {/* Left Column: Hero Target Stage */}
          <div style={{ minHeight: 0, height: '100%' }}>
            <TargetView
              elevation={targetState.elevation}
              optimalROI={calibration.optimalROI}
              lastShot={lastShot}
              recentShots={shots.slice(-10)}
              isLocked={isSessionLocked || isEmergencyStop}
              exposureRemainingSeconds={targetState.exposureRemainingSeconds}
              onTargetClick={handleTargetClick}
              onPopTarget={() => controller.pop()}
            />
          </div>

          {/* Right Column: Master Command Deck & Impact Stream */}
          <div style={{
            display: 'flex',
            flexDirection: 'column',
            gap: '12px',
            minHeight: 0,
            height: '100%',
          }}>
            {/* Top: Massive [R], [S], [0] Controls + Continuous Distance + Rail Track */}
            <div style={{ flex: '1 1 65%', minHeight: 0 }}>
              <RailPositionControl
                targetState={targetState}
                onRun={() => controller.run()}
                onStop={() => controller.stop()}
                onPop={() => controller.pop()}
                onEmergencyStop={() => controller.emergencyStop()}
              />
            </div>

            {/* Bottom: Clean Recent Impact Stream (No clutter!) */}
            <div style={{ flex: '1 1 35%', minHeight: '140px' }}>
              <ShotHistoryTable shots={shots} />
            </div>
          </div>
        </main>
      </div>

      {/* Calibration Modal */}
      {showCalibration && (
        <CalibrationModal
          config={calibration}
          onSave={handleSaveCalibration}
          onClose={() => setShowCalibration(false)}
        />
      )}

      {/* Automatic 20-Shot Session Complete Report */}
      {report && (
        <SessionReportModal
          report={report}
          optimalROI={calibration.optimalROI}
          onStartNewSession={handleStartNewSession}
          onClose={() => setReport(null)}
        />
      )}
    </div>
  );
}
